// P/Invoke surface for the Microsoft Azure Kinect Body Tracking SDK (k4abt.dll).
// Only the subset needed to drive the BT loop: tracker create / enqueue capture /
// pop result / read joints. All declarations transcribed from
//   C:\Program Files\Azure Kinect Body Tracking SDK\sdk\include\k4abt.h
//   C:\Program Files\Azure Kinect Body Tracking SDK\sdk\include\k4abttypes.h
// Do NOT add or change signatures based on guesses. Re-check the headers
// before editing any function in this file.

using System;
using System.Runtime.InteropServices;

namespace BodyTracking
{
    // --- Enums (k4abttypes.h) ---

    /// <summary>k4abt_joint_id_t. K4ABT_JOINT_COUNT == 32.</summary>
    public enum k4abt_joint_id_t
    {
        K4ABT_JOINT_PELVIS = 0,
        K4ABT_JOINT_SPINE_NAVEL,
        K4ABT_JOINT_SPINE_CHEST,
        K4ABT_JOINT_NECK,
        K4ABT_JOINT_CLAVICLE_LEFT,
        K4ABT_JOINT_SHOULDER_LEFT,
        K4ABT_JOINT_ELBOW_LEFT,
        K4ABT_JOINT_WRIST_LEFT,
        K4ABT_JOINT_HAND_LEFT,
        K4ABT_JOINT_HANDTIP_LEFT,
        K4ABT_JOINT_THUMB_LEFT,
        K4ABT_JOINT_CLAVICLE_RIGHT,
        K4ABT_JOINT_SHOULDER_RIGHT,
        K4ABT_JOINT_ELBOW_RIGHT,
        K4ABT_JOINT_WRIST_RIGHT,
        K4ABT_JOINT_HAND_RIGHT,
        K4ABT_JOINT_HANDTIP_RIGHT,
        K4ABT_JOINT_THUMB_RIGHT,
        K4ABT_JOINT_HIP_LEFT,
        K4ABT_JOINT_KNEE_LEFT,
        K4ABT_JOINT_ANKLE_LEFT,
        K4ABT_JOINT_FOOT_LEFT,
        K4ABT_JOINT_HIP_RIGHT,
        K4ABT_JOINT_KNEE_RIGHT,
        K4ABT_JOINT_ANKLE_RIGHT,
        K4ABT_JOINT_FOOT_RIGHT,
        K4ABT_JOINT_HEAD,
        K4ABT_JOINT_NOSE,
        K4ABT_JOINT_EYE_LEFT,
        K4ABT_JOINT_EAR_LEFT,
        K4ABT_JOINT_EYE_RIGHT,
        K4ABT_JOINT_EAR_RIGHT,
        K4ABT_JOINT_COUNT,
    }

    public enum k4abt_joint_confidence_level_t
    {
        K4ABT_JOINT_CONFIDENCE_NONE = 0,    // out of range
        K4ABT_JOINT_CONFIDENCE_LOW = 1,     // predicted (e.g. occluded)
        K4ABT_JOINT_CONFIDENCE_MEDIUM = 2,  // current SDK ceiling
        K4ABT_JOINT_CONFIDENCE_HIGH = 3,    // future SDK
        K4ABT_JOINT_CONFIDENCE_LEVELS_COUNT = 4,
    }

    public enum k4abt_sensor_orientation_t
    {
        K4ABT_SENSOR_ORIENTATION_DEFAULT = 0,
        K4ABT_SENSOR_ORIENTATION_CLOCKWISE90,
        K4ABT_SENSOR_ORIENTATION_COUNTERCLOCKWISE90,
        K4ABT_SENSOR_ORIENTATION_FLIP180,
    }

    public enum k4abt_tracker_processing_mode_t
    {
        K4ABT_TRACKER_PROCESSING_MODE_GPU = 0,        // best-available GPU (DirectML on Windows)
        K4ABT_TRACKER_PROCESSING_MODE_CPU,
        K4ABT_TRACKER_PROCESSING_MODE_GPU_CUDA,
        K4ABT_TRACKER_PROCESSING_MODE_GPU_TENSORRT,
        K4ABT_TRACKER_PROCESSING_MODE_GPU_DIRECTML,   // explicit DirectML
    }

    // --- Structs (k4abttypes.h) ---

    /// <summary>k4a_quaternion_t: union {wxyz / float[4]}. Sequential 16 bytes.</summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct k4a_quaternion_t
    {
        public float W, X, Y, Z;
    }

    /// <summary>
    /// k4abt_tracker_configuration_t: enum + enum + int32 + const char*.
    /// model_path is marshaled as UTF-8 LP string (null = use default).
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct k4abt_tracker_configuration_t
    {
        public k4abt_sensor_orientation_t SensorOrientation;
        public k4abt_tracker_processing_mode_t ProcessingMode;
        public int GpuDeviceId;
        [MarshalAs(UnmanagedType.LPUTF8Str)] public string ModelPath;
    }

    /// <summary>k4abt_joint_t: float3 position(mm) + quaternion + confidence enum = 32 bytes.</summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct k4abt_joint_t
    {
        public k4a_float3_t Position;          // millimeters
        public k4a_quaternion_t Orientation;   // normalized
        public k4abt_joint_confidence_level_t ConfidenceLevel;
    }

    /// <summary>k4abt_skeleton_t: 32 joints inline = 32 * 32 = 1024 bytes.</summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct k4abt_skeleton_t
    {
        // Flat fixed array of K4ABT_JOINT_COUNT (32) joints. ByValArray with SizeConst
        // marshals it as an inline C array, matching the header layout exactly.
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = (int)k4abt_joint_id_t.K4ABT_JOINT_COUNT)]
        public k4abt_joint_t[] Joints;
    }

    // --- Constants (k4abttypes.h) ---

    public static class K4ABTConsts
    {
        public const int K4ABT_JOINT_COUNT = (int)k4abt_joint_id_t.K4ABT_JOINT_COUNT; // 32
        public const uint K4ABT_INVALID_BODY_ID = 0xFFFFFFFFu;
        public const byte K4ABT_BODY_INDEX_MAP_BACKGROUND = 255;
        public const float K4ABT_DEFAULT_TRACKER_SMOOTHING_FACTOR = 0.0f;
        public const int K4A_WAIT_INFINITE = -1;
    }

    // --- DllImports (k4abt.h) ---

    public static class K4ABTNative
    {
        public const string DLL = "k4abt";

        // tracker lifecycle
        // sensor_calibration is `const k4a_calibration_t*` — we pass a heap-allocated
        // buffer (Marshal.AllocHGlobal + StructureToPtr) from K4ACaptureBridge so
        // that the (large, nested) k4a_calibration_t doesn't need to be hand-marshaled
        // here. See k4abt.h around line 64 for the C signature.
        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern k4a_result_t k4abt_tracker_create(
            IntPtr sensor_calibration,
            k4abt_tracker_configuration_t config,
            out IntPtr tracker_handle);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern void k4abt_tracker_destroy(IntPtr tracker_handle);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern void k4abt_tracker_set_temporal_smoothing(IntPtr tracker_handle, float smoothing_factor);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern void k4abt_tracker_shutdown(IntPtr tracker_handle);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern k4a_wait_result_t k4abt_tracker_enqueue_capture(
            IntPtr tracker_handle,
            IntPtr sensor_capture_handle,
            int timeout_in_ms);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern k4a_wait_result_t k4abt_tracker_pop_result(
            IntPtr tracker_handle,
            out IntPtr body_frame_handle,
            int timeout_in_ms);

        // body frame access
        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern void k4abt_frame_release(IntPtr body_frame_handle);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern uint k4abt_frame_get_num_bodies(IntPtr body_frame_handle);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern k4a_result_t k4abt_frame_get_body_skeleton(
            IntPtr body_frame_handle,
            uint index,
            out k4abt_skeleton_t skeleton);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern uint k4abt_frame_get_body_id(IntPtr body_frame_handle, uint index);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        public static extern ulong k4abt_frame_get_device_timestamp_usec(IntPtr body_frame_handle);
    }
}
