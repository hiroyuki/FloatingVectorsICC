using System;
using System.Collections.Generic;

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

        /// <summary>
        /// True when the device exposes <see cref="OrbbecFrame.GlobalTimestampUs"/> values
        /// (drift-corrected device→host time). Femto Bolt firmware versions vary; check before
        /// relying on the global timestamp instead of the system timestamp.
        /// </summary>
        public bool IsGlobalTimestampSupported()
        {
            ThrowIfDisposed();
            bool supported = OrbbecNative.ob_device_is_global_timestamp_supported(Handle, out var err);
            OrbbecException.ThrowIfNotEmpty(err);
            return supported;
        }

        /// <summary>
        /// Enable or disable the global-timestamp service. Disabled by default in the SDK; call
        /// once after opening the device and before starting the pipeline. With it enabled,
        /// frame.GlobalTimestampUs returns a host-clock-aligned value comparable across devices.
        /// </summary>
        public void EnableGlobalTimestamp(bool enable)
        {
            ThrowIfDisposed();
            OrbbecNative.ob_device_enable_global_timestamp(Handle, enable, out var err);
            OrbbecException.ThrowIfNotEmpty(err);
        }

        // --- Depth work mode (Advanced.h) ---

        /// <summary>Name of the currently active depth work mode.</summary>
        public string GetCurrentDepthWorkModeName()
        {
            ThrowIfDisposed();
            IntPtr p = OrbbecNative.ob_device_get_current_depth_work_mode_name(Handle, out var err);
            OrbbecException.ThrowIfNotEmpty(err);
            return OrbbecNative.ReadUtf8(p);
        }

        /// <summary>
        /// Switch to a depth work mode by name. The name must match one reported by
        /// <see cref="GetDepthWorkModeNames"/> for the current device / firmware.
        /// Must be called before the pipeline starts.
        /// </summary>
        public void SwitchDepthWorkModeByName(string modeName)
        {
            ThrowIfDisposed();
            if (string.IsNullOrEmpty(modeName))
                throw new ArgumentException("modeName must be non-empty.", nameof(modeName));
            var status = OrbbecNative.ob_device_switch_depth_work_mode_by_name(Handle, modeName, out var err);
            OrbbecException.ThrowIfNotEmpty(err);
            if (status != ObStatus.Ok)
                throw new InvalidOperationException(
                    $"ob_device_switch_depth_work_mode_by_name(\"{modeName}\") returned {status}.");
        }

        /// <summary>
        /// Enumerate available depth work mode names. Allocates and frees the SDK list internally.
        /// </summary>
        public IReadOnlyList<string> GetDepthWorkModeNames()
        {
            ThrowIfDisposed();
            IntPtr list = OrbbecNative.ob_device_get_depth_work_mode_list(Handle, out var err);
            OrbbecException.ThrowIfNotEmpty(err);
            if (list == IntPtr.Zero) return Array.Empty<string>();

            var names = new List<string>();
            try
            {
                uint count = OrbbecNative.ob_depth_work_mode_list_get_count(list, out err);
                OrbbecException.ThrowIfNotEmpty(err);
                for (uint i = 0; i < count; i++)
                {
                    var item = OrbbecNative.ob_depth_work_mode_list_get_item(list, i, out err);
                    OrbbecException.ThrowIfNotEmpty(err);
                    if (!string.IsNullOrEmpty(item.Name)) names.Add(item.Name);
                }
            }
            finally
            {
                OrbbecNative.ob_delete_depth_work_mode_list(list, out _);
            }
            return names;
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
