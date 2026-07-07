using System;

namespace Orbbec
{
    /// <summary>
    /// Base class for IDisposable wrappers around native ob_* handles.
    /// Centralizes the shared handle field, dispose guard, finalizer and
    /// disposed-check; subclasses implement <see cref="ReleaseHandle"/> with
    /// their ob_delete_* call.
    /// </summary>
    public abstract class OrbbecHandle : IDisposable
    {
        internal IntPtr Handle { get; private protected set; }
        private bool _disposed;

        /// <summary>Call the matching ob_delete_* for this wrapper's handle.</summary>
        protected abstract void ReleaseHandle(IntPtr handle);

        /// <summary>
        /// Hook invoked before <see cref="ReleaseHandle"/> when the handle is
        /// still live (e.g. OrbbecPipeline stops the pipeline before deleting it).
        /// </summary>
        protected virtual void OnBeforeRelease() { }

        public void Dispose()
        {
            if (_disposed) return;
            _disposed = true;
            if (Handle != IntPtr.Zero)
            {
                OnBeforeRelease();
                ReleaseHandle(Handle);
                Handle = IntPtr.Zero;
            }
            GC.SuppressFinalize(this);
        }

        ~OrbbecHandle() => Dispose();

        protected void ThrowIfDisposed()
        {
            if (_disposed) throw new ObjectDisposedException(GetType().Name);
        }
    }
}
