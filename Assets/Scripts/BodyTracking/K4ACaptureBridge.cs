// Wraps a v2 Y16 depth buffer in a k4a_capture_t so we can hand it to the
// k4abt tracker without ever owning the device through the K4A API. Lifetime:
// the byte buffer is copied into native memory at create time and freed when
// the underlying k4a_image_t's refcount drops to 0 (the tracker keeps a ref
// for the duration of inference and drops it when the body frame is popped).
//
// Compiled into both Unity and the standalone k4abt_worker.exe — diagnostics
// route through WorkerLog (Unity-bound to Debug.Log; worker-bound to Console)
// rather than UnityEngine.Debug directly so the worker build stays Unity-free.

using System;
using System.Runtime.InteropServices;
using BodyTracking.Shared;

namespace BodyTracking
{
    public static class K4ACaptureBridge
    {
        // The k4a release callback is a function pointer the SDK calls back into
        // managed code from a worker thread. We keep one static delegate alive
        // (so the GC doesn't collect it) and pass it to every image create call.
        private static readonly k4a_memory_destroy_cb_t s_releaseCb = ReleaseUnmanagedBuffer;

#if UNITY_2017_1_OR_NEWER
        // IL2CPP requires this attribute for delegates handed to native code; the
        // standalone .NET 8 worker build doesn't define UNITY_* and ignores it.
        [AOT.MonoPInvokeCallback(typeof(k4a_memory_destroy_cb_t))]
#endif
        private static void ReleaseUnmanagedBuffer(IntPtr buffer, IntPtr context)
        {
            // We allocated the buffer via Marshal.AllocHGlobal in CreateDepthImageFromY16,
            // so the matching release is FreeHGlobal. context is unused.
            if (buffer != IntPtr.Zero) Marshal.FreeHGlobal(buffer);
        }

        /// <summary>
        /// Allocate a k4a_image_t (DEPTH16) backed by a copy of the supplied Y16 row-major
        /// buffer. The image takes ownership of the copied bytes and frees them when
        /// released. Returns IntPtr.Zero on failure.
        /// </summary>
        public static IntPtr CreateDepthImageFromY16(byte[] y16, int byteCount, int width, int height,
                                                      ulong deviceTimestampUsec)
        {
            if (y16 == null || byteCount <= 0 || width <= 0 || height <= 0)
                return IntPtr.Zero;

            int strideBytes = width * 2; // DEPTH16 = 2 bytes per pixel
            int expected = strideBytes * height;
            if (byteCount < expected)
            {
                WorkerLog.Error(
                    $"[K4ACaptureBridge] depth buffer too small: got {byteCount} bytes, " +
                    $"expected {expected} for {width}x{height} Y16");
                return IntPtr.Zero;
            }

            IntPtr nativeBuf = Marshal.AllocHGlobal(expected);
            Marshal.Copy(y16, 0, nativeBuf, expected);

            var rc = K4ANative.k4a_image_create_from_buffer(
                k4a_image_format_t.K4A_IMAGE_FORMAT_DEPTH16,
                width, height, strideBytes,
                nativeBuf, (UIntPtr)expected,
                s_releaseCb, IntPtr.Zero,
                out IntPtr image);

            if (rc != k4a_result_t.K4A_RESULT_SUCCEEDED)
            {
                Marshal.FreeHGlobal(nativeBuf);
                WorkerLog.Error("[K4ACaptureBridge] k4a_image_create_from_buffer failed");
                return IntPtr.Zero;
            }

            K4ANative.k4a_image_set_device_timestamp_usec(image, deviceTimestampUsec);
            return image;
        }

        /// <summary>
        /// Create a k4a_capture_t containing the supplied depth image. Adds a ref to the
        /// image internally (so it is safe to release the caller's handle afterwards).
        /// Caller releases the returned capture via <see cref="K4ANative.k4a_capture_release"/>.
        /// Returns IntPtr.Zero on failure.
        /// </summary>
        public static IntPtr CreateCaptureWithDepth(IntPtr depthImage)
        {
            if (depthImage == IntPtr.Zero) return IntPtr.Zero;

            var rc = K4ANative.k4a_capture_create(out IntPtr capture);
            if (rc != k4a_result_t.K4A_RESULT_SUCCEEDED)
            {
                WorkerLog.Error("[K4ACaptureBridge] k4a_capture_create failed");
                return IntPtr.Zero;
            }
            K4ANative.k4a_capture_set_depth_image(capture, depthImage);
            return capture;
        }

        /// <summary>
        /// Build a k4a_capture_t with a depth image and (if supplied) a proper IR image.
        /// Pass <paramref name="ir16"/> = null and <paramref name="irByteCount"/> = 0 to
        /// fall back to using the depth bytes as a stand-in IR image — the tracker runs
        /// end-to-end but accuracy drops because the BT model uses IR brightness as a real
        /// cue. The full path with a real IR stream is the supported one.
        /// Returns IntPtr.Zero on failure.
        /// </summary>
        public static IntPtr CreateCaptureFromDepthAndIR(
            byte[] depth16, int depthByteCount, int depthWidth, int depthHeight,
            byte[] ir16, int irByteCount, int irWidth, int irHeight,
            ulong deviceTimestampUsec)
        {
            IntPtr depth = CreateDepthImageFromY16(depth16, depthByteCount, depthWidth, depthHeight, deviceTimestampUsec);
            if (depth == IntPtr.Zero) return IntPtr.Zero;

            byte[] irSrc = ir16;
            int irCount = irByteCount;
            int irW = irWidth;
            int irH = irHeight;
            // Fallback path: reuse depth as IR if no real IR stream is plumbed through.
            if (irSrc == null || irCount <= 0 || irW <= 0 || irH <= 0)
            {
                irSrc = depth16;
                irCount = depthByteCount;
                irW = depthWidth;
                irH = depthHeight;
            }

            IntPtr ir = CreateIrImageFromY16(irSrc, irCount, irW, irH, deviceTimestampUsec);
            if (ir == IntPtr.Zero)
            {
                K4ANative.k4a_image_release(depth);
                return IntPtr.Zero;
            }

            var rc = K4ANative.k4a_capture_create(out IntPtr capture);
            if (rc != k4a_result_t.K4A_RESULT_SUCCEEDED)
            {
                WorkerLog.Error("[K4ACaptureBridge] k4a_capture_create failed");
                K4ANative.k4a_image_release(depth);
                K4ANative.k4a_image_release(ir);
                return IntPtr.Zero;
            }
            K4ANative.k4a_capture_set_depth_image(capture, depth);
            K4ANative.k4a_capture_set_ir_image(capture, ir);
            // capture_set_*_image bumped each image's refcount; release our caller-side refs.
            K4ANative.k4a_image_release(depth);
            K4ANative.k4a_image_release(ir);
            return capture;
        }

        /// <summary>Backward-compat wrapper that uses depth as the IR stand-in.</summary>
        public static IntPtr CreateCaptureFromDepthY16(byte[] y16, int byteCount, int width, int height,
                                                        ulong deviceTimestampUsec)
        {
            return CreateCaptureFromDepthAndIR(
                y16, byteCount, width, height,
                null, 0, 0, 0,
                deviceTimestampUsec);
        }

        /// <summary>
        /// Build a k4a_image_t in IR16 format backed by a copy of the supplied Y16 buffer.
        /// Used as a stand-in until v2 gives us a real passive-IR stream.
        /// </summary>
        public static IntPtr CreateIrImageFromY16(byte[] y16, int byteCount, int width, int height,
                                                    ulong deviceTimestampUsec)
        {
            if (y16 == null || byteCount <= 0 || width <= 0 || height <= 0)
                return IntPtr.Zero;

            int strideBytes = width * 2; // IR16 = 2 bytes per pixel
            int expected = strideBytes * height;
            if (byteCount < expected) return IntPtr.Zero;

            IntPtr nativeBuf = Marshal.AllocHGlobal(expected);
            Marshal.Copy(y16, 0, nativeBuf, expected);

            var rc = K4ANative.k4a_image_create_from_buffer(
                k4a_image_format_t.K4A_IMAGE_FORMAT_IR16,
                width, height, strideBytes,
                nativeBuf, (UIntPtr)expected,
                s_releaseCb, IntPtr.Zero,
                out IntPtr image);

            if (rc != k4a_result_t.K4A_RESULT_SUCCEEDED)
            {
                Marshal.FreeHGlobal(nativeBuf);
                WorkerLog.Error("[K4ACaptureBridge] k4a_image_create_from_buffer (IR16) failed");
                return IntPtr.Zero;
            }
            K4ANative.k4a_image_set_device_timestamp_usec(image, deviceTimestampUsec);
            return image;
        }
    }
}
