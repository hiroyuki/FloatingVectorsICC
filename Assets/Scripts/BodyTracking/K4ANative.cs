// P/Invoke surface for the Orbbec K4A Wrapper (k4a.dll).
// Only the subset needed to build a synthetic k4a_capture_t around v2 depth
// frames and hand it to the body tracker. All declarations transcribed from
//   D:\OrbbecSDK_K4A_Wrapper_v2.0.11_windows_202510221441\include\k4a\k4a.h
//   D:\OrbbecSDK_K4A_Wrapper_v2.0.11_windows_202510221441\include\k4a\k4atypes.h
// Do NOT add or change signatures based on guesses. Re-check the headers
// before editing any function in this file.

using System;
using System.Runtime.InteropServices;

namespace BodyTracking
{
    // --- Enums (k4atypes.h) ---

    public enum k4a_result_t
    {
        K4A_RESULT_SUCCEEDED = 0,
        K4A_RESULT_FAILED,
    }

    public enum k4a_wait_result_t
    {
        K4A_WAIT_RESULT_SUCCEEDED = 0,
        K4A_WAIT_RESULT_FAILED,
        K4A_WAIT_RESULT_TIMEOUT,
    }

    /// <summary>k4atypes.h: depth_mode enum tags (drives BT model selection too).</summary>
    public enum k4a_depth_mode_t
    {
        K4A_DEPTH_MODE_OFF = 0,
        K4A_DEPTH_MODE_NFOV_2X2BINNED, // 320x288
        K4A_DEPTH_MODE_NFOV_UNBINNED,  // 640x576
        K4A_DEPTH_MODE_WFOV_2X2BINNED, // 512x512
        K4A_DEPTH_MODE_WFOV_UNBINNED,  // 1024x1024
        K4A_DEPTH_MODE_PASSIVE_IR,     // 1024x1024
    }

    public enum k4a_color_resolution_t
    {
        K4A_COLOR_RESOLUTION_OFF = 0,
        K4A_COLOR_RESOLUTION_720P,
        K4A_COLOR_RESOLUTION_1080P,
        K4A_COLOR_RESOLUTION_1440P,
        K4A_COLOR_RESOLUTION_1536P,
        K4A_COLOR_RESOLUTION_2160P,
        K4A_COLOR_RESOLUTION_3072P,
    }

    public enum k4a_image_format_t
    {
        K4A_IMAGE_FORMAT_COLOR_MJPG = 0,
        K4A_IMAGE_FORMAT_COLOR_NV12,
        K4A_IMAGE_FORMAT_COLOR_YUY2,
        K4A_IMAGE_FORMAT_COLOR_BGRA32,
        K4A_IMAGE_FORMAT_DEPTH16,
        K4A_IMAGE_FORMAT_IR16,
        K4A_IMAGE_FORMAT_CUSTOM8,
        K4A_IMAGE_FORMAT_CUSTOM16,
        K4A_IMAGE_FORMAT_CUSTOM,
    }

    public enum k4a_calibration_type_t
    {
        K4A_CALIBRATION_TYPE_UNKNOWN = -1,
        K4A_CALIBRATION_TYPE_DEPTH = 0,
        K4A_CALIBRATION_TYPE_COLOR = 1,
        K4A_CALIBRATION_TYPE_GYRO = 2,
        K4A_CALIBRATION_TYPE_ACCEL = 3,
        K4A_CALIBRATION_TYPE_NUM = 4,
    }

    public enum k4a_calibration_model_type_t
    {
        K4A_CALIBRATION_LENS_DISTORTION_MODEL_UNKNOWN = 0,
        K4A_CALIBRATION_LENS_DISTORTION_MODEL_THETA,
        K4A_CALIBRATION_LENS_DISTORTION_MODEL_POLYNOMIAL_3K,
        K4A_CALIBRATION_LENS_DISTORTION_MODEL_RATIONAL_6KT,
        K4A_CALIBRATION_LENS_DISTORTION_MODEL_BROWN_CONRADY,
    }

    // --- Structs (k4atypes.h) ---

    /// <summary>k4a_float3_t: union of {x,y,z} / float[3]. Sequential 12 bytes.</summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct k4a_float3_t
    {
        public float X, Y, Z;
    }

    /// <summary>k4a_calibration_extrinsics_t: float rotation[9] + float translation[3] = 48 bytes.</summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct k4a_calibration_extrinsics_t
    {
        // Row-major 3x3 rotation, then 3-element translation in millimeters.
        public float R00, R01, R02, R10, R11, R12, R20, R21, R22;
        public float T0, T1, T2;
    }

    /// <summary>
    /// k4a_calibration_intrinsic_parameters_t: union of named struct (15 floats) and float[15].
    /// Sequential 60 bytes. Names mirror header.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct k4a_calibration_intrinsic_parameters_t
    {
        public float Cx, Cy, Fx, Fy;
        public float K1, K2, K3, K4, K5, K6;
        public float Codx, Cody;
        public float P2, P1;
        public float MetricRadius;
    }

    /// <summary>k4a_calibration_intrinsics_t: enum + uint + 60-byte parameters = 68 bytes.</summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct k4a_calibration_intrinsics_t
    {
        public k4a_calibration_model_type_t Type;
        public uint ParameterCount;
        public k4a_calibration_intrinsic_parameters_t Parameters;
    }

    /// <summary>
    /// k4a_calibration_camera_t: extrinsics(48) + intrinsics(68) + width(4) + height(4) + metric_radius(4) = 128.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct k4a_calibration_camera_t
    {
        public k4a_calibration_extrinsics_t Extrinsics;
        public k4a_calibration_intrinsics_t Intrinsics;
        public int ResolutionWidth;
        public int ResolutionHeight;
        public float MetricRadius;
    }

    // k4a_calibration_t is large (~1032 bytes) and contains a 4x4 array of extrinsics.
    // Phase 1 leaves construction to Phase 3 — call sites pass the calibration as an
    // IntPtr (allocated and filled via Marshal helpers in K4ACaptureBridge). That avoids
    // hand-marshaling fixed multi-dim arrays inside this declaration file.

    /// <summary>
    /// Memory release callback signature (k4atypes.h: k4a_memory_destroy_cb_t).
    /// Called by k4a when the image's reference count reaches 0 to free the buffer.
    /// </summary>
    [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
    public delegate void k4a_memory_destroy_cb_t(IntPtr buffer, IntPtr context);

    // --- DllImports (k4a.h) ---

    public static class K4ANative
    {
        public const string DLL = "k4a";

        // capture lifecycle
        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern k4a_result_t k4a_capture_create(out IntPtr capture_handle);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern void k4a_capture_release(IntPtr capture_handle);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern void k4a_capture_set_depth_image(IntPtr capture_handle, IntPtr image_handle);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr k4a_capture_get_depth_image(IntPtr capture_handle);

        // image lifecycle
        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern k4a_result_t k4a_image_create_from_buffer(
            k4a_image_format_t format,
            int width_pixels,
            int height_pixels,
            int stride_bytes,
            IntPtr buffer,
            UIntPtr buffer_size,
            k4a_memory_destroy_cb_t buffer_release_cb,
            IntPtr buffer_release_cb_context,
            out IntPtr image_handle);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern void k4a_image_release(IntPtr image_handle);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern void k4a_image_set_device_timestamp_usec(IntPtr image_handle, ulong timestamp_usec);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern void k4a_image_set_system_timestamp_nsec(IntPtr image_handle, ulong timestamp_nsec);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern int k4a_image_get_width_pixels(IntPtr image_handle);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern int k4a_image_get_height_pixels(IntPtr image_handle);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern int k4a_image_get_stride_bytes(IntPtr image_handle);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern UIntPtr k4a_image_get_size(IntPtr image_handle);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern IntPtr k4a_image_get_buffer(IntPtr image_handle);
    }
}
