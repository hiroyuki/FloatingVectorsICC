// Worker-side helper around the per-camera memory-mapped file. Wraps the raw
// MemoryMappedFile / MemoryMappedViewAccessor pair, holds a pointer for the
// duration of the session (avoids reacquiring per frame), and exposes typed
// read/write helpers for the global header and slot regions defined by
// BodyTracking.Shared.K4abtWorkerSharedLayout.
//
// Threading model: the worker is single-threaded for IPC operations. The
// only concurrency comes from the seq-lock pattern guarding individual
// slots — readers tolerate a writer interleaving, but the worker never
// has two reader threads on the same slot.

using System;
using System.IO.MemoryMappedFiles;
using System.Runtime.InteropServices;
using System.Threading;
using BodyTracking.Shared;

namespace K4abtWorker
{
    internal sealed unsafe class MmfRegion : IDisposable
    {
        private readonly MemoryMappedFile _mmf;
        private readonly MemoryMappedViewAccessor _view;
        private byte* _basePtr;
        private bool _pointerAcquired;

        public int DepthW { get; }
        public int DepthH { get; }
        public int IrW { get; }
        public int IrH { get; }
        public long ViewSize { get; }

        public MmfRegion(string mmfName, int depthW, int depthH, int irW, int irH)
        {
            DepthW = depthW;
            DepthH = depthH;
            IrW = irW;
            IrH = irH;

            _mmf = MemoryMappedFile.OpenExisting(mmfName, MemoryMappedFileRights.ReadWrite);

            long expected = K4abtWorkerSharedLayout.TotalMmfSize(depthW, depthH, irW, irH);
            // Map the full region. The host is responsible for allocating exactly the same
            // size; if the host asked for a different layout the header verify catches it.
            _view = _mmf.CreateViewAccessor(0, expected, MemoryMappedFileAccess.ReadWrite);
            ViewSize = _view.Capacity;

            byte* p = null;
            _view.SafeMemoryMappedViewHandle.AcquirePointer(ref p);
            if (p == null)
            {
                _view.Dispose();
                _mmf.Dispose();
                throw new InvalidOperationException("AcquirePointer on MMF view returned null");
            }
            _basePtr = p;
            _pointerAcquired = true;
        }

        // --- Header field accessors ---

        public uint ReadHeaderUint(int offset) => *(uint*)(_basePtr + offset);
        public ulong ReadHeaderUlong(int offset) => *(ulong*)(_basePtr + offset);
        public void WriteHeaderUint(int offset, uint value) => *(uint*)(_basePtr + offset) = value;

        /// <summary>Worker increments the consumed counter atomically.</summary>
        public ulong IncrementConsumed()
        {
            // Interlocked.Increment treats the long as signed but we only need the
            // monotonic counter; reinterpret as uint64 on read.
            long* p = (long*)(_basePtr + K4abtWorkerSharedLayout.OffsetConsumed);
            long n = Interlocked.Increment(ref *p);
            return (ulong)n;
        }

        public ulong ReadProduced() => Volatile.Read(ref *(ulong*)(_basePtr + K4abtWorkerSharedLayout.OffsetProduced));
        public ulong ReadDropped() => Volatile.Read(ref *(ulong*)(_basePtr + K4abtWorkerSharedLayout.OffsetDropped));
        public ulong ReadConsumed() => Volatile.Read(ref *(ulong*)(_basePtr + K4abtWorkerSharedLayout.OffsetConsumed));

        public uint ReadLatestInputSlot()
            => Volatile.Read(ref *(uint*)(_basePtr + K4abtWorkerSharedLayout.OffsetLatestInputSlot));

        public void WriteLatestOutputSlot(uint slotIdx)
            => Volatile.Write(ref *(uint*)(_basePtr + K4abtWorkerSharedLayout.OffsetLatestOutputSlot), slotIdx);

        /// <summary>
        /// Verify magic / layout_version / resolution match the args this worker was
        /// launched with. Returns null on success or a short error description on
        /// mismatch — the caller maps that to ExitLayoutMismatch.
        /// </summary>
        public string VerifyHeader()
        {
            uint magic = ReadHeaderUint(K4abtWorkerSharedLayout.OffsetMagic);
            if (magic != K4abtWorkerSharedLayout.Magic)
                return $"magic mismatch: got 0x{magic:X8}, expected 0x{K4abtWorkerSharedLayout.Magic:X8}";

            uint version = ReadHeaderUint(K4abtWorkerSharedLayout.OffsetLayoutVersion);
            if (version != K4abtWorkerSharedLayout.LayoutVersion)
                return $"layout_version mismatch: got {version}, expected {K4abtWorkerSharedLayout.LayoutVersion}";

            int dW = (int)ReadHeaderUint(K4abtWorkerSharedLayout.OffsetDepthW);
            int dH = (int)ReadHeaderUint(K4abtWorkerSharedLayout.OffsetDepthH);
            int iW = (int)ReadHeaderUint(K4abtWorkerSharedLayout.OffsetIrW);
            int iH = (int)ReadHeaderUint(K4abtWorkerSharedLayout.OffsetIrH);
            if (dW != DepthW || dH != DepthH || iW != IrW || iH != IrH)
                return $"resolution mismatch: header={dW}x{dH} depth + {iW}x{iH} IR, " +
                       $"args={DepthW}x{DepthH} depth + {IrW}x{IrH} IR";

            uint inSlots = ReadHeaderUint(K4abtWorkerSharedLayout.OffsetInputSlotCount);
            uint outSlots = ReadHeaderUint(K4abtWorkerSharedLayout.OffsetOutputSlotCount);
            if (inSlots != K4abtWorkerSharedLayout.InputSlotCount || outSlots != K4abtWorkerSharedLayout.OutputSlotCount)
                return $"slot count mismatch: got input={inSlots} output={outSlots}";

            return null;
        }

        /// <summary>
        /// Allocate a 1032-byte unmanaged buffer and copy the calibration blob from
        /// the MMF header into it. Caller owns the returned IntPtr and must Free it.
        /// We never bind the bytes to a managed k4a_calibration_t struct (see
        /// K4ACalibration.cs for the layout-by-offsets convention).
        /// </summary>
        public IntPtr AllocAndCopyCalibration()
        {
            IntPtr buf = Marshal.AllocHGlobal(K4abtWorkerSharedLayout.CalibrationBlobBytes);
            byte* src = _basePtr + K4abtWorkerSharedLayout.OffsetCalibration;
            Buffer.MemoryCopy(src, (void*)buf, K4abtWorkerSharedLayout.CalibrationBlobBytes,
                              K4abtWorkerSharedLayout.CalibrationBlobBytes);
            return buf;
        }

        // --- Slot read (input) ---

        public struct InputSlotSnapshot
        {
            public ulong Seq;
            public ulong FrameId;
            public ulong TsNs;
            public uint Flags;
        }

        /// <summary>
        /// Try to read input slot at <paramref name="slotIdx"/> using the seq-lock
        /// protocol: read seq → copy payload → re-read seq → accept iff both
        /// reads matched and were even (committed). Returns false on torn read,
        /// caller should skip this frame.
        /// </summary>
        public bool TryReadInputSlot(int slotIdx, byte[] depthDst, byte[] irDst, out InputSlotSnapshot snap)
        {
            snap = default;
            long slotOffset = K4abtWorkerSharedLayout.InputSlotOffset(slotIdx, DepthW, DepthH, IrW, IrH);
            byte* slot = _basePtr + slotOffset;

            ulong seqBefore = Volatile.Read(ref *(ulong*)(slot + K4abtWorkerSharedLayout.SlotOffsetSeq));
            if ((seqBefore & 1) != 0) return false; // writer in progress

            ulong frameId = *(ulong*)(slot + K4abtWorkerSharedLayout.SlotOffsetFrameId);
            ulong tsNs = *(ulong*)(slot + K4abtWorkerSharedLayout.SlotOffsetTsNs);
            uint flags = *(uint*)(slot + K4abtWorkerSharedLayout.SlotOffsetFlags);

            // Memory barrier so the seq read above is ordered before the payload reads
            // below, and the second seq read happens after the payload.
            Thread.MemoryBarrier();

            int depthBytes = DepthW * DepthH * 2;
            int irBytes = IrW * IrH * 2;
            if (depthDst.Length < depthBytes || irDst.Length < irBytes)
                throw new ArgumentException("destination buffer too small");

            byte* depthSrc = slot + K4abtWorkerSharedLayout.InputDepthPayloadOffset();
            byte* irSrc = slot + K4abtWorkerSharedLayout.InputIrPayloadOffset(DepthW, DepthH);

            fixed (byte* dDst = depthDst)
                Buffer.MemoryCopy(depthSrc, dDst, depthDst.Length, depthBytes);
            fixed (byte* iDst = irDst)
                Buffer.MemoryCopy(irSrc, iDst, irDst.Length, irBytes);

            Thread.MemoryBarrier();
            ulong seqAfter = Volatile.Read(ref *(ulong*)(slot + K4abtWorkerSharedLayout.SlotOffsetSeq));
            if (seqAfter != seqBefore || (seqAfter & 1) != 0) return false; // torn

            snap.Seq = seqBefore;
            snap.FrameId = frameId;
            snap.TsNs = tsNs;
            snap.Flags = flags;
            return true;
        }

        // --- Slot write (output) ---

        // Per-slot writer state lives outside this class (Program tracks lastWrittenSeq
        // for each output slot). The helpers here assume the caller picks slotIdx and
        // owns the seq monotonicity invariant.

        /// <summary>Bump the seq to (lastWrittenSeq + 1) — odd = writing — and return it.</summary>
        public ulong BeginWriteOutputSlot(int slotIdx, ulong lastWrittenSeq)
        {
            long slotOffset = K4abtWorkerSharedLayout.OutputSlotOffset(slotIdx, DepthW, DepthH, IrW, IrH);
            byte* slot = _basePtr + slotOffset;
            ulong writing = lastWrittenSeq + 1;
            Volatile.Write(ref *(ulong*)(slot + K4abtWorkerSharedLayout.SlotOffsetSeq), writing);
            Thread.MemoryBarrier();
            return writing;
        }

        /// <summary>Commit the output slot (writingSeq + 1, even). Returns the new committed seq.</summary>
        public ulong CommitOutputSlot(int slotIdx, ulong writingSeq)
        {
            long slotOffset = K4abtWorkerSharedLayout.OutputSlotOffset(slotIdx, DepthW, DepthH, IrW, IrH);
            byte* slot = _basePtr + slotOffset;
            ulong committed = writingSeq + 1;
            Thread.MemoryBarrier();
            Volatile.Write(ref *(ulong*)(slot + K4abtWorkerSharedLayout.SlotOffsetSeq), committed);
            return committed;
        }

        /// <summary>
        /// Pointer to the start of an output slot's payload region, used by the
        /// skeleton writer to fill body records directly. The caller must be
        /// inside a Begin/Commit pair when writing here.
        /// </summary>
        public byte* OutputSlotPayloadPtr(int slotIdx)
        {
            long slotOffset = K4abtWorkerSharedLayout.OutputSlotOffset(slotIdx, DepthW, DepthH, IrW, IrH);
            return _basePtr + slotOffset;
        }

        public void Dispose()
        {
            if (_pointerAcquired)
            {
                try { _view.SafeMemoryMappedViewHandle.ReleasePointer(); }
                catch { /* best-effort */ }
                _pointerAcquired = false;
                _basePtr = null;
            }
            _view?.Dispose();
            _mmf?.Dispose();
        }
    }
}
