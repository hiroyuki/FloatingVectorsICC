using System;

namespace Orbbec
{
    /// <summary>
    /// IDisposable wrapper around ob_filter. Filter names come from the SDK
    /// (e.g. "PointCloudFilter", "Align", "FormatConverter"); see
    /// libobsensor/hpp/Filter.hpp for the full registry.
    /// </summary>
    public sealed class OrbbecFilter : OrbbecHandle
    {
        public string Name { get; }

        public OrbbecFilter(string name)
        {
            if (string.IsNullOrEmpty(name)) throw new ArgumentException("name", nameof(name));
            Name = name;
            Handle = OrbbecNative.ob_create_filter(name, out var err);
            OrbbecException.ThrowIfNotEmpty(err);
            if (Handle == IntPtr.Zero)
                throw new InvalidOperationException($"ob_create_filter('{name}') returned null.");
        }

        public void SetConfigValue(string configName, double value)
        {
            ThrowIfDisposed();
            OrbbecNative.ob_filter_set_config_value(Handle, configName, value, out var err);
            OrbbecException.ThrowIfNotEmpty(err);
        }

        public double GetConfigValue(string configName)
        {
            ThrowIfDisposed();
            double v = OrbbecNative.ob_filter_get_config_value(Handle, configName, out var err);
            OrbbecException.ThrowIfNotEmpty(err);
            return v;
        }

        /// <summary>
        /// Synchronous process. Returns a new frame the caller must Dispose.
        /// </summary>
        public OrbbecFrame Process(OrbbecFrame input)
        {
            ThrowIfDisposed();
            if (input == null) throw new ArgumentNullException(nameof(input));
            var outHandle = OrbbecNative.ob_filter_process(Handle, input.Handle, out var err);
            OrbbecException.ThrowIfNotEmpty(err);
            return outHandle == IntPtr.Zero ? null : new OrbbecFrame(outHandle);
        }

        protected override void ReleaseHandle(IntPtr handle) =>
            OrbbecNative.ob_delete_filter(handle, out _);
    }
}
