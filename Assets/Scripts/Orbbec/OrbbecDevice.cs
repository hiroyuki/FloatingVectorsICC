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
