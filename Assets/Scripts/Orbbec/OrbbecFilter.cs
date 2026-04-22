using System;

namespace Orbbec
{
    /// <summary>
    /// IDisposable wrapper around ob_filter. Filter names come from the SDK
    /// (e.g. "PointCloudFilter", "Align", "FormatConverter"); see
    /// libobsensor/hpp/Filter.hpp for the full registry.
    /// </summary>
    public sealed class OrbbecFilter : IDisposable
    {
        internal IntPtr Handle { get; private set; }
        public string Name { get; }
        private bool _disposed;

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

        public void Dispose()
        {
            if (_disposed) return;
            _disposed = true;
            if (Handle != IntPtr.Zero)
            {
                OrbbecNative.ob_delete_filter(Handle, out _);
                Handle = IntPtr.Zero;
            }
            GC.SuppressFinalize(this);
        }

        ~OrbbecFilter() => Dispose();

        private void ThrowIfDisposed()
        {
            if (_disposed) throw new ObjectDisposedException(nameof(OrbbecFilter));
        }
    }
}
