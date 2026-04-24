using System;

namespace Orbbec
{
    /// <summary>IDisposable wrapper around ob_device.</summary>
    public sealed class OrbbecDevice : IDisposable
    {
        internal IntPtr Handle { get; private set; }
        private bool _disposed;

        internal OrbbecDevice(IntPtr handle)
        {
            if (handle == IntPtr.Zero) throw new ArgumentNullException(nameof(handle));
            Handle = handle;
        }

        /// <summary>
        /// Read device info. Returns a fully-populated managed snapshot; the underlying
        /// ob_device_info is freed before this method returns.
        /// </summary>
        public OrbbecDeviceInfo GetInfo()
        {
            ThrowIfDisposed();
            var infoHandle = OrbbecNative.ob_device_get_device_info(Handle, out var err);
            OrbbecException.ThrowIfNotEmpty(err);
            if (infoHandle == IntPtr.Zero)
                throw new InvalidOperationException("ob_device_get_device_info returned null.");

            try
            {
                string serial = OrbbecNative.ReadUtf8(
                    OrbbecNative.ob_device_info_get_serial_number(infoHandle, out err));
                OrbbecException.ThrowIfNotEmpty(err);

                string name = OrbbecNative.ReadUtf8(
                    OrbbecNative.ob_device_info_get_name(infoHandle, out err));
                OrbbecException.ThrowIfNotEmpty(err);

                string uid = OrbbecNative.ReadUtf8(
                    OrbbecNative.ob_device_info_get_uid(infoHandle, out err));
                OrbbecException.ThrowIfNotEmpty(err);

                string fw = OrbbecNative.ReadUtf8(
                    OrbbecNative.ob_device_info_get_firmware_version(infoHandle, out err));
                OrbbecException.ThrowIfNotEmpty(err);

                string conn = OrbbecNative.ReadUtf8(
                    OrbbecNative.ob_device_info_get_connection_type(infoHandle, out err));
                OrbbecException.ThrowIfNotEmpty(err);

                return new OrbbecDeviceInfo(serial, name, uid, fw, conn);
            }
            finally
            {
                OrbbecNative.ob_delete_device_info(infoHandle, out _);
            }
        }

        public OrbbecPipeline CreatePipeline()
        {
            ThrowIfDisposed();
            return new OrbbecPipeline(this);
        }

        // --- Multi-device sync (MultipleDevices.h) ---

        /// <summary>
        /// Bitmap of supported <see cref="ObMultiDeviceSyncMode"/> values. Callers typically
        /// check `(bitmap &amp; (ushort)mode) != 0` before selecting a mode.
        /// </summary>
        public ushort GetSupportedSyncModeBitmap()
        {
            ThrowIfDisposed();
            ushort bitmap = OrbbecNative.ob_device_get_supported_multi_device_sync_mode_bitmap(Handle, out var err);
            OrbbecException.ThrowIfNotEmpty(err);
            return bitmap;
        }

        public void SetSyncConfig(ObMultiDeviceSyncConfig config)
        {
            ThrowIfDisposed();
            OrbbecNative.ob_device_set_multi_device_sync_config(Handle, ref config, out var err);
            OrbbecException.ThrowIfNotEmpty(err);
        }

        public ObMultiDeviceSyncConfig GetSyncConfig()
        {
            ThrowIfDisposed();
            var config = OrbbecNative.ob_device_get_multi_device_sync_config(Handle, out var err);
            OrbbecException.ThrowIfNotEmpty(err);
            return config;
        }

        /// <summary>Send a one-shot capture command (only effective in SOFTWARE_TRIGGERING mode).</summary>
        public void TriggerCapture()
        {
            ThrowIfDisposed();
            OrbbecNative.ob_device_trigger_capture(Handle, out var err);
            OrbbecException.ThrowIfNotEmpty(err);
        }

        /// <summary>
        /// Align the device's internal timer with the host. Recommended at startup and
        /// periodically (~every 60 min) to limit drift. After this call, frame timestamps
        /// across devices become comparable.
        /// </summary>
        public void TimerSyncWithHost()
        {
            ThrowIfDisposed();
            OrbbecNative.ob_device_timer_sync_with_host(Handle, out var err);
            OrbbecException.ThrowIfNotEmpty(err);
        }

        public void Dispose()
        {
            if (_disposed) return;
            _disposed = true;
            if (Handle != IntPtr.Zero)
            {
                OrbbecNative.ob_delete_device(Handle, out _);
                Handle = IntPtr.Zero;
            }
            GC.SuppressFinalize(this);
        }

        ~OrbbecDevice() => Dispose();

        private void ThrowIfDisposed()
        {
            if (_disposed) throw new ObjectDisposedException(nameof(OrbbecDevice));
        }
    }

    public sealed class OrbbecDeviceInfo
    {
        public string Serial { get; }
        public string Name { get; }
        public string Uid { get; }
        public string FirmwareVersion { get; }
        public string ConnectionType { get; }

        internal OrbbecDeviceInfo(string serial, string name, string uid, string fw, string conn)
        {
            Serial = serial; Name = name; Uid = uid;
            FirmwareVersion = fw; ConnectionType = conn;
        }

        public override string ToString() =>
            $"{Name} sn={Serial} uid={Uid} fw={FirmwareVersion} conn={ConnectionType}";
    }
}
