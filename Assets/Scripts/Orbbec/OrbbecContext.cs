using System;
using System.Collections.Generic;

namespace Orbbec
{
    /// <summary>
    /// IDisposable wrapper around ob_context. One per process is recommended;
    /// PointCloudCameraManager owns the lifecycle.
    /// </summary>
    public sealed class OrbbecContext : IDisposable
    {
        internal IntPtr Handle { get; private set; }
        private bool _disposed;

        public OrbbecContext()
        {
            Handle = OrbbecNative.ob_create_context(out var err);
            OrbbecException.ThrowIfNotEmpty(err);
            if (Handle == IntPtr.Zero)
                throw new InvalidOperationException("ob_create_context returned null without error.");
        }

        /// <summary>Set the directory containing depthengine / filters DLLs.</summary>
        /// <remarks>Must be called BEFORE constructing the context for the first time
        /// (and before opening any device).</remarks>
        public static void SetExtensionsDirectory(string directory)
        {
            OrbbecNative.ob_set_extensions_directory(directory, out var err);
            OrbbecException.ThrowIfNotEmpty(err);
        }

        public static void SetLoggerSeverity(ObLogSeverity severity)
        {
            OrbbecNative.ob_set_logger_severity(severity, out var err);
            OrbbecException.ThrowIfNotEmpty(err);
        }

        /// <summary>
        /// Snapshot of currently connected devices. The native list is consumed
        /// internally and released before this method returns.
        /// </summary>
        public IReadOnlyList<OrbbecDeviceDescriptor> QueryDevices()
        {
            ThrowIfDisposed();
            var result = new List<OrbbecDeviceDescriptor>();

            var listHandle = OrbbecNative.ob_query_device_list(Handle, out var err);
            OrbbecException.ThrowIfNotEmpty(err);
            if (listHandle == IntPtr.Zero) return result;

            try
            {
                uint count = OrbbecNative.ob_device_list_get_count(listHandle, out err);
                OrbbecException.ThrowIfNotEmpty(err);

                for (uint i = 0; i < count; i++)
                {
                    var serial = OrbbecNative.ReadUtf8(
                        OrbbecNative.ob_device_list_get_device_serial_number(listHandle, i, out err));
                    OrbbecException.ThrowIfNotEmpty(err);

                    var name = OrbbecNative.ReadUtf8(
                        OrbbecNative.ob_device_list_get_device_name(listHandle, i, out err));
                    OrbbecException.ThrowIfNotEmpty(err);

                    var uid = OrbbecNative.ReadUtf8(
                        OrbbecNative.ob_device_list_get_device_uid(listHandle, i, out err));
                    OrbbecException.ThrowIfNotEmpty(err);

                    result.Add(new OrbbecDeviceDescriptor(i, serial, name, uid));
                }
            }
            finally
            {
                OrbbecNative.ob_delete_device_list(listHandle, out _);
            }
            return result;
        }

        /// <summary>Open a device by its serial number (preferred for multi-device setups).</summary>
        public OrbbecDevice OpenDeviceBySerial(string serialNumber)
        {
            ThrowIfDisposed();
            if (string.IsNullOrEmpty(serialNumber))
                throw new ArgumentException("Serial number must be non-empty.", nameof(serialNumber));

            var listHandle = OrbbecNative.ob_query_device_list(Handle, out var err);
            OrbbecException.ThrowIfNotEmpty(err);
            if (listHandle == IntPtr.Zero)
                throw new InvalidOperationException("Device list query returned null.");

            try
            {
                var deviceHandle = OrbbecNative.ob_device_list_get_device_by_serial_number(
                    listHandle, serialNumber, out err);
                OrbbecException.ThrowIfNotEmpty(err);
                if (deviceHandle == IntPtr.Zero)
                    throw new InvalidOperationException(
                        $"Device with serial '{serialNumber}' not found.");
                return new OrbbecDevice(deviceHandle);
            }
            finally
            {
                OrbbecNative.ob_delete_device_list(listHandle, out _);
            }
        }

        public void Dispose()
        {
            if (_disposed) return;
            _disposed = true;
            if (Handle != IntPtr.Zero)
            {
                OrbbecNative.ob_delete_context(Handle, out _);
                Handle = IntPtr.Zero;
            }
            GC.SuppressFinalize(this);
        }

        ~OrbbecContext() => Dispose();

        private void ThrowIfDisposed()
        {
            if (_disposed) throw new ObjectDisposedException(nameof(OrbbecContext));
        }
    }

    /// <summary>Lightweight metadata snapshot of a device entry from the device list.</summary>
    public sealed class OrbbecDeviceDescriptor
    {
        public uint Index { get; }
        public string Serial { get; }
        public string Name { get; }
        public string Uid { get; }

        internal OrbbecDeviceDescriptor(uint index, string serial, string name, string uid)
        {
            Index = index;
            Serial = serial;
            Name = name;
            Uid = uid;
        }

        public override string ToString() => $"{Name} (sn={Serial}, uid={Uid})";
    }
}
