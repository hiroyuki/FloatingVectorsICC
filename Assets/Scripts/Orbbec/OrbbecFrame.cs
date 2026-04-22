using System;

namespace Orbbec
{
    /// <summary>
    /// IDisposable wrapper around ob_frame. Covers both individual frames and
    /// framesets (a frameset is itself an ob_frame).
    /// </summary>
    public sealed class OrbbecFrame : IDisposable
    {
        internal IntPtr Handle { get; private set; }
        private bool _disposed;

        internal OrbbecFrame(IntPtr handle)
        {
            if (handle == IntPtr.Zero) throw new ArgumentNullException(nameof(handle));
            Handle = handle;
        }

        // --- Common metadata ---

        public ObFrameType Type
        {
            get
            {
                ThrowIfDisposed();
                var v = OrbbecNative.ob_frame_get_type(Handle, out var err);
                OrbbecException.ThrowIfNotEmpty(err);
                return v;
            }
        }

        public ObFormat Format
        {
            get
            {
                ThrowIfDisposed();
                var v = OrbbecNative.ob_frame_get_format(Handle, out var err);
                OrbbecException.ThrowIfNotEmpty(err);
                return v;
            }
        }

        public ulong Index
        {
            get
            {
                ThrowIfDisposed();
                var v = OrbbecNative.ob_frame_get_index(Handle, out var err);
                OrbbecException.ThrowIfNotEmpty(err);
                return v;
            }
        }

        public ulong TimestampUs
        {
            get
            {
                ThrowIfDisposed();
                var v = OrbbecNative.ob_frame_get_timestamp_us(Handle, out var err);
                OrbbecException.ThrowIfNotEmpty(err);
                return v;
            }
        }

        /// <summary>Raw native data pointer. Valid until this frame is disposed.</summary>
        public IntPtr DataPointer
        {
            get
            {
                ThrowIfDisposed();
                var p = OrbbecNative.ob_frame_get_data(Handle, out var err);
                OrbbecException.ThrowIfNotEmpty(err);
                return p;
            }
        }

        public uint DataSize
        {
            get
            {
                ThrowIfDisposed();
                var n = OrbbecNative.ob_frame_get_data_size(Handle, out var err);
                OrbbecException.ThrowIfNotEmpty(err);
                return n;
            }
        }

        // --- Video frame ---

        public uint VideoWidth
        {
            get
            {
                ThrowIfDisposed();
                var v = OrbbecNative.ob_video_frame_get_width(Handle, out var err);
                OrbbecException.ThrowIfNotEmpty(err);
                return v;
            }
        }

        public uint VideoHeight
        {
            get
            {
                ThrowIfDisposed();
                var v = OrbbecNative.ob_video_frame_get_height(Handle, out var err);
                OrbbecException.ThrowIfNotEmpty(err);
                return v;
            }
        }

        // --- Points frame ---

        /// <summary>
        /// Multiplier to convert raw point coords to millimeters.
        /// (Apply before any further unit conversion.)
        /// </summary>
        public float PointsCoordinateValueScale
        {
            get
            {
                ThrowIfDisposed();
                var v = OrbbecNative.ob_points_frame_get_coordinate_value_scale(Handle, out var err);
                OrbbecException.ThrowIfNotEmpty(err);
                return v;
            }
        }

        // --- Frameset accessors (only valid when Type == ObFrameType.Set) ---

        public OrbbecFrame GetDepthFrame()
        {
            ThrowIfDisposed();
            var h = OrbbecNative.ob_frameset_get_depth_frame(Handle, out var err);
            OrbbecException.ThrowIfNotEmpty(err);
            return h == IntPtr.Zero ? null : new OrbbecFrame(h);
        }

        public OrbbecFrame GetColorFrame()
        {
            ThrowIfDisposed();
            var h = OrbbecNative.ob_frameset_get_color_frame(Handle, out var err);
            OrbbecException.ThrowIfNotEmpty(err);
            return h == IntPtr.Zero ? null : new OrbbecFrame(h);
        }

        public OrbbecFrame GetPointsFrame()
        {
            ThrowIfDisposed();
            var h = OrbbecNative.ob_frameset_get_points_frame(Handle, out var err);
            OrbbecException.ThrowIfNotEmpty(err);
            return h == IntPtr.Zero ? null : new OrbbecFrame(h);
        }

        public void Dispose()
        {
            if (_disposed) return;
            _disposed = true;
            if (Handle != IntPtr.Zero)
            {
                OrbbecNative.ob_delete_frame(Handle, out _);
                Handle = IntPtr.Zero;
            }
            GC.SuppressFinalize(this);
        }

        ~OrbbecFrame() => Dispose();

        private void ThrowIfDisposed()
        {
            if (_disposed) throw new ObjectDisposedException(nameof(OrbbecFrame));
        }
    }
}
