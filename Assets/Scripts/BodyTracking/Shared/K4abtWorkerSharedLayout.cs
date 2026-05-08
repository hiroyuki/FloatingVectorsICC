// Shared layout / sync constants for the k4abt worker IPC contract.
// MUST stay Unity-free: this file is compiled into both Unity (auto-discovery
// in Assets/) and the standalone Workers/K4abtWorker .NET console project, so
// any UnityEngine.* dependency would break the worker build.
//
// Contract overview (see Plans/issue-10-k4abt-worker-process.md for the full
// description and rationale):
//   - Per-camera memory-mapped file with a global header followed by
//     N input slots (Unity -> worker: depth+IR) and N output slots (worker ->
//     Unity: skeletons). Slots are written/read with a seq-lock pattern so a
//     reader can detect torn writes without locking.
//   - Named events live in the Local\ namespace and are scoped by a per-Unity-
//     session GUID so multiple Unity instances do not collide.
//   - All sizes are computed at runtime from depth/IR resolution; no magic
//     byte counts in the binary protocol.

using System;

namespace BodyTracking.Shared
{
    public static class K4abtWorkerSharedLayout
    {
        // 'K' '4' 'B' 'T' (little-endian uint32). Worker rejects the MMF on mismatch.
        public const uint Magic = 0x54_42_34_4Bu;
        public const uint LayoutVersion = 1;

        public const int InputSlotCount = 2;   // double-buffer (latest-wins backpressure)
        public const int OutputSlotCount = 2;
        public const int K4abtJointCount = 32; // K4ABT_JOINT_COUNT (k4abttypes.h)
        public const int MaxBodies = 6;        // K4ABT_MAX_BODIES (k4abttypes.h L24)

        // Calibration blob size: anchored to the layout K4ACalibration.cs writes by
        // hand-computed offsets. NEVER define a managed k4a_calibration_t struct here
        // (the existing code intentionally avoids it; see K4ACalibration.cs L1-L11).
        public const int CalibrationBlobBytes = 1032;

        // --- Global header field offsets (bytes) ---
        public const int OffsetMagic = 0;             // u32
        public const int OffsetLayoutVersion = 4;     // u32
        public const int OffsetDepthW = 8;            // u32
        public const int OffsetDepthH = 12;
        public const int OffsetIrW = 16;
        public const int OffsetIrH = 20;
        public const int OffsetDepthBpp = 24;         // u32 = 2 (Y16)
        public const int OffsetIrBpp = 28;
        public const int OffsetInputSlotCount = 32;
        public const int OffsetOutputSlotCount = 36;
        public const int OffsetProduced = 40;         // u64 — host increments each successful enqueue
        public const int OffsetConsumed = 48;         // u64 — worker increments each successful read
        public const int OffsetDropped = 56;          // u64 — host increments when overwriting an unread slot
        public const int OffsetCalibration = 64;      // CalibrationBlobBytes raw bytes
        public const int OffsetLatestInputSlot = 64 + CalibrationBlobBytes;   // u32 last committed input idx
        public const int OffsetLatestOutputSlot = OffsetLatestInputSlot + 4;  // u32 last committed output idx
        public const int GlobalHeaderRawBytes = OffsetLatestOutputSlot + 4;
        public const int Alignment = 256;

        public static int GlobalHeaderBytes => Align(GlobalHeaderRawBytes, Alignment);

        // --- Per-slot header (input + output share the prefix) ---
        public const int SlotOffsetSeq = 0;        // u64, odd = writing / even = committed
        public const int SlotOffsetFrameId = 8;    // u64
        public const int SlotOffsetTsNs = 16;      // u64
        public const int SlotOffsetFlags = 24;     // u32
        public const int SlotOffsetReserved = 28;  // u32 (reserved, written zero)
        public const int SlotPayloadStart = 32;

        // Input slot flags
        public const uint InputFlagDepthValid = 1u << 0;
        public const uint InputFlagIrValid = 1u << 1;

        // Output slot extra header (after the common SlotOffsetSeq..Reserved prefix)
        public const int OutSlotOffsetBodyCount = 24;   // u32 — overlaps SlotOffsetFlags slot; output reuses flags region as count
        public const int OutSlotOffsetReserved = 28;
        public const int OutSlotOffsetBodies = 32;
        // Body record: id(u32) + reserved(u32) + 32 joints { pos(float3=12) + quat(float4=16) + confidence(u32=4) }
        public const int JointRecordBytes = 12 + 16 + 4;             // 32
        public const int BodyRecordBytes = 8 + K4abtJointCount * JointRecordBytes; // 8 + 32*32 = 1032

        // --- Sizing helpers (no magic numbers; everything from W*H*BPP) ---

        public static int Align(int n, int alignment) => (n + alignment - 1) & ~(alignment - 1);

        public static int InputSlotSize(int depthW, int depthH, int irW, int irH)
        {
            int payload = depthW * depthH * 2 + irW * irH * 2;
            return Align(SlotPayloadStart + payload, Alignment);
        }

        public static int OutputSlotSize()
        {
            int payload = MaxBodies * BodyRecordBytes;
            return Align(OutSlotOffsetBodies + payload, Alignment);
        }

        public static long TotalMmfSize(int depthW, int depthH, int irW, int irH)
        {
            return (long)GlobalHeaderBytes
                + (long)InputSlotSize(depthW, depthH, irW, irH) * InputSlotCount
                + (long)OutputSlotSize() * OutputSlotCount;
        }

        public static long InputSlotOffset(int slotIndex, int depthW, int depthH, int irW, int irH)
        {
            return (long)GlobalHeaderBytes + (long)InputSlotSize(depthW, depthH, irW, irH) * slotIndex;
        }

        public static long OutputSlotOffset(int slotIndex, int depthW, int depthH, int irW, int irH)
        {
            return (long)GlobalHeaderBytes
                + (long)InputSlotSize(depthW, depthH, irW, irH) * InputSlotCount
                + (long)OutputSlotSize() * slotIndex;
        }

        // Per-slot payload offsets within an input slot (absolute relative to slot start).
        public static int InputDepthPayloadOffset() => SlotPayloadStart;
        public static int InputIrPayloadOffset(int depthW, int depthH) => SlotPayloadStart + depthW * depthH * 2;

        // --- IPC name builders (Local\ namespace, session-scoped) ---

        public static string Prefix(Guid sessionGuid, string serial)
            => $"FB_K4ABT_{sessionGuid:N}_{Sanitize(serial)}";

        public static string MmfName(Guid sessionGuid, string serial) => $@"Local\{Prefix(sessionGuid, serial)}_mmf";
        public static string InputEventName(Guid sessionGuid, string serial) => $@"Local\{Prefix(sessionGuid, serial)}_in";
        public static string OutputEventName(Guid sessionGuid, string serial) => $@"Local\{Prefix(sessionGuid, serial)}_out";
        public static string ReadyEventName(Guid sessionGuid, string serial) => $@"Local\{Prefix(sessionGuid, serial)}_ready";
        public static string ShutdownEventName(Guid sessionGuid, string serial) => $@"Local\{Prefix(sessionGuid, serial)}_shutdown";

        private static string Sanitize(string s)
        {
            if (string.IsNullOrEmpty(s)) return "unknown";
            char[] buf = new char[s.Length];
            for (int i = 0; i < s.Length; i++)
            {
                char c = s[i];
                bool ok = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
                          (c >= '0' && c <= '9') || c == '_' || c == '-';
                buf[i] = ok ? c : '_';
            }
            return new string(buf);
        }

        // --- Timeout constants (k4abt API uses int timeout_in_ms; see K4ABTNative.cs) ---

        public const int EnqueueTimeoutMs = 200;
        public const int PopTimeoutMs = 500;
        public const int InputWaitTimeoutMs = 1_000;     // shutdown-check granularity in worker loop
        public const int ReadyWaitTimeoutMs = 15_000;    // tracker_create + ONNX model load
        public const int TeardownWaitMs = 3_000;         // host-side WaitForExit before Process.Kill
        public const int ParentWatchdogIntervalMs = 1_000;

        // Worker exit codes
        public const int ExitOk = 0;
        public const int ExitParentDied = 1;
        public const int ExitLayoutMismatch = 2;
        public const int ExitTrackerCreateFailed = 3;
        public const int ExitArgError = 4;
    }
}
