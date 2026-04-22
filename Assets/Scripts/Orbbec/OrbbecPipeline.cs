using System;

namespace Orbbec
{
    /// <summary>IDisposable wrapper around ob_pipeline.</summary>
    public sealed class OrbbecPipeline : IDisposable
    {
        internal IntPtr Handle { get; private set; }
        private bool _disposed;
        private bool _started;

        internal OrbbecPipeline(OrbbecDevice device)
        {
            if (device == null) throw new ArgumentNullException(nameof(device));
            Handle = OrbbecNative.ob_create_pipeline_with_device(device.Handle, out var err);
            OrbbecException.ThrowIfNotEmpty(err);
            if (Handle == IntPtr.Zero)
                throw new InvalidOperationException("ob_create_pipeline_with_device returned null.");
        }

        public void EnableFrameSync()
        {
            ThrowIfDisposed();
            OrbbecNative.ob_pipeline_enable_frame_sync(Handle, out var err);
            OrbbecException.ThrowIfNotEmpty(err);
        }

        public void Start(OrbbecConfig config)
        {
            ThrowIfDisposed();
            if (config == null) throw new ArgumentNullException(nameof(config));
            OrbbecNative.ob_pipeline_start_with_config(Handle, config.Handle, out var err);
            OrbbecException.ThrowIfNotEmpty(err);
            _started = true;
        }

        public void Stop()
        {
            ThrowIfDisposed();
            if (!_started) return;
            OrbbecNative.ob_pipeline_stop(Handle, out var err);
            OrbbecException.ThrowIfNotEmpty(err);
            _started = false;
        }

        /// <summary>
        /// Block up to <paramref name="timeoutMs"/> ms for a frameset.
        /// Returns null if no frameset arrived within the timeout.
        /// Caller owns the returned frame (must Dispose).
        /// </summary>
        public OrbbecFrame WaitForFrameset(uint timeoutMs)
        {
            ThrowIfDisposed();
            var frameHandle = OrbbecNative.ob_pipeline_wait_for_frameset(Handle, timeoutMs, out var err);
            OrbbecException.ThrowIfNotEmpty(err);
            return frameHandle == IntPtr.Zero ? null : new OrbbecFrame(frameHandle);
        }

        public void Dispose()
        {
            if (_disposed) return;
            _disposed = true;
            if (Handle != IntPtr.Zero)
            {
                if (_started)
                {
                    OrbbecNative.ob_pipeline_stop(Handle, out _);
                    _started = false;
                }
                OrbbecNative.ob_delete_pipeline(Handle, out _);
                Handle = IntPtr.Zero;
            }
            GC.SuppressFinalize(this);
        }

        ~OrbbecPipeline() => Dispose();

        private void ThrowIfDisposed()
        {
            if (_disposed) throw new ObjectDisposedException(nameof(OrbbecPipeline));
        }
    }

    /// <summary>IDisposable wrapper around ob_config.</summary>
    public sealed class OrbbecConfig : IDisposable
    {
        internal IntPtr Handle { get; private set; }
        private bool _disposed;

        public OrbbecConfig()
        {
            Handle = OrbbecNative.ob_create_config(out var err);
            OrbbecException.ThrowIfNotEmpty(err);
            if (Handle == IntPtr.Zero)
                throw new InvalidOperationException("ob_create_config returned null.");
        }

        /// <summary>Pass 0 (ObConst.WidthAny etc.) for any of width/height/fps to let the SDK pick.</summary>
        public void EnableVideoStream(ObStreamType streamType, uint width, uint height, uint fps, ObFormat format)
        {
            ThrowIfDisposed();
            OrbbecNative.ob_config_enable_video_stream(Handle, streamType, width, height, fps, format, out var err);
            OrbbecException.ThrowIfNotEmpty(err);
        }

        public void SetAlignMode(ObAlignMode mode)
        {
            ThrowIfDisposed();
            OrbbecNative.ob_config_set_align_mode(Handle, mode, out var err);
            OrbbecException.ThrowIfNotEmpty(err);
        }

        public void SetFrameAggregateOutputMode(ObFrameAggregateOutputMode mode)
        {
            ThrowIfDisposed();
            OrbbecNative.ob_config_set_frame_aggregate_output_mode(Handle, mode, out var err);
            OrbbecException.ThrowIfNotEmpty(err);
        }

        public void Dispose()
        {
            if (_disposed) return;
            _disposed = true;
            if (Handle != IntPtr.Zero)
            {
                OrbbecNative.ob_delete_config(Handle, out _);
                Handle = IntPtr.Zero;
            }
            GC.SuppressFinalize(this);
        }

        ~OrbbecConfig() => Dispose();

        private void ThrowIfDisposed()
        {
            if (_disposed) throw new ObjectDisposedException(nameof(OrbbecConfig));
        }
    }
}
