// P/Invoke surface for OrbbecSDK v2 (libobsensor).
// All declarations transcribed directly from headers under
// C:\dev\OrbbecSDK_v2\include\libobsensor\h\ — do NOT add or change
// signatures based on guesses. Re-check the corresponding header before
// editing any function in this file.

using System;
using System.Runtime.InteropServices;

namespace Orbbec
{
    // --- Enums (ObTypes.h) ---

    public enum ObSensorType
    {
        Unknown = 0,
        IR = 1,
        Color = 2,
        Depth = 3,
        Accel = 4,
        Gyro = 5,
        IRLeft = 6,
        IRRight = 7,
    }

    public enum ObStreamType
    {
        Unknown = -1,
        Video = 0,
        IR = 1,
        Color = 2,
        Depth = 3,
        Accel = 4,
        Gyro = 5,
        IRLeft = 6,
        IRRight = 7,
    }

    public enum ObFrameType
    {
        Unknown = -1,
        Video = 0,
        IR = 1,
        Color = 2,
        Depth = 3,
        Accel = 4,
        Set = 5,
        Points = 6,
        Gyro = 7,
        IRLeft = 8,
        IRRight = 9,
    }

    public enum ObFormat
    {
        Unknown = -1,
        YUYV = 0,
        UYVY = 2,
        NV12 = 3,
        NV21 = 4,
        MJPG = 5,
        H264 = 6,
        H265 = 7,
        Y16 = 8,
        Y8 = 9,
        Gray = 13,
        I420 = 15,
        Point = 19,         // OBPoint (xyz)
        RGBPoint = 20,      // OBColorPoint (xyz + rgb)
        RGB = 22,
        BGR = 23,
        BGRA = 25,
        Z16 = 28,
        RGBA = 31,
    }

    public enum ObAlignMode
    {
        Disable = 0,
        D2CHwMode = 1,
        D2CSwMode = 2,
    }

    public enum ObFrameAggregateOutputMode
    {
        AllTypeFrameRequire = 0,
        ColorFrameRequire = 1,
        AnySituation = 2,
        Disable = 3,
    }

    public enum ObLogSeverity
    {
        Debug = 0,
        Info = 1,
        Warn = 2,
        Error = 3,
        Fatal = 4,
        Off = 5,
    }

    /// <summary>
    /// Multi-device sync modes. Values are bit-flags as declared in ObTypes.h
    /// (ob_multi_device_sync_mode): ob_device_get_supported_multi_device_sync_mode_bitmap
    /// returns OR of these. A single ob_multi_device_sync_config.syncMode field takes
    /// exactly one of these values.
    /// </summary>
    [Flags]
    public enum ObMultiDeviceSyncMode
    {
        FreeRun             = 1 << 0,
        Standalone          = 1 << 1,
        Primary             = 1 << 2,
        Secondary           = 1 << 3,
        SecondarySynced     = 1 << 4,
        SoftwareTriggering  = 1 << 5,
        HardwareTriggering  = 1 << 6,
        IrImuSync           = 1 << 7,
        SoftwareSynced      = 1 << 8,
    }

    // --- Structs (ObTypes.h) ---

    /// <summary>OBColorPoint: matches OB_FORMAT_RGB_POINT layout (6 floats, 24 bytes).</summary>
    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct ObColorPoint
    {
        public float X, Y, Z;
        public float R, G, B;
    }

    /// <summary>OBPoint: matches OB_FORMAT_POINT layout (3 floats, 12 bytes).</summary>
    [StructLayout(LayoutKind.Sequential, Pack = 1)]
    public struct ObPoint
    {
        public float X, Y, Z;
    }

    /// <summary>
    /// ob_multi_device_sync_config (ObTypes.h). Default natural alignment (Sequential, no Pack)
    /// gives: enum(4) + int*3(12) + bool(1)+pad(3) + int*2(8) = 28 bytes, matching the C layout
    /// on MSVC/GCC-Windows where both enums and _Bool take 1 and 4 bytes respectively with
    /// 4-byte alignment between ints.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct ObMultiDeviceSyncConfig
    {
        public ObMultiDeviceSyncMode SyncMode;
        public int DepthDelayUs;
        public int ColorDelayUs;
        public int Trigger2ImageDelayUs;
        [MarshalAs(UnmanagedType.I1)] public bool TriggerOutEnable;
        public int TriggerOutDelayUs;
        public int FramesPerTrigger;
    }

    public enum ObDepthWorkModeTag
    {
        DeviceDefault = 0,  // OB_DEVICE_DEPTH_WORK_MODE
        Custom        = 1,  // OB_CUSTOM_DEPTH_WORK_MODE
    }

    /// <summary>ob_camera_distortion_model (ObTypes.h).</summary>
    public enum ObCameraDistortionModel
    {
        None         = 0,
        ModifiedBrownConrady = 1,
        InverseBrownConrady  = 2,
        BrownConrady         = 3,
        KannalaBrandt4       = 4,
    }

    /// <summary>ob_camera_intrinsic (ObTypes.h): 4 floats + 2 int16 = 20 bytes.</summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct ObCameraIntrinsic
    {
        public float Fx, Fy, Cx, Cy;
        public short Width, Height;
    }

    /// <summary>ob_camera_distortion (ObTypes.h): 8 floats + enum model = 36 bytes.</summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct ObCameraDistortion
    {
        public float K1, K2, K3, K4, K5, K6;
        public float P1, P2;
        public ObCameraDistortionModel Model;
    }

    /// <summary>ob_extrinsic / ob_d2c_transform (ObTypes.h): rot[9] + trans[3] = 48 bytes. trans is in millimeters.</summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct ObExtrinsic
    {
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 9)] public float[] Rot;    // row-major 3x3
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 3)] public float[] Trans;  // mm
    }

    /// <summary>
    /// ob_camera_param (ObTypes.h): intrinsic pair + distortion pair + depth-to-color transform + mirrored flag.
    /// Total = 20+20+36+36+48+1+pad(3) = 164 bytes.
    /// The <c>Transform</c> field is the depth-to-color extrinsic in millimeters.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct ObCameraParam
    {
        public ObCameraIntrinsic DepthIntrinsic;
        public ObCameraIntrinsic RgbIntrinsic;
        public ObCameraDistortion DepthDistortion;
        public ObCameraDistortion RgbDistortion;
        public ObExtrinsic Transform;
        [MarshalAs(UnmanagedType.I1)] public bool IsMirrored;
    }

    /// <summary>
    /// ob_depth_work_mode (ObTypes.h): uint8_t checksum[16] + char name[32] + enum tag.
    /// Natural alignment gives 16 + 32 + 4 = 52 bytes.
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct ObDepthWorkMode
    {
        [MarshalAs(UnmanagedType.ByValArray, SizeConst = 16)]
        public byte[] Checksum;

        [MarshalAs(UnmanagedType.ByValTStr, SizeConst = 32)]
        public string Name;

        public ObDepthWorkModeTag Tag;
    }

    // --- Convenience constants (ObTypes.h) ---

    public static class ObConst
    {
        public const int WidthAny = 0;
        public const int HeightAny = 0;
        public const int FpsAny = 0;
        public const int FormatAny = -1; // OB_FORMAT_UNKNOWN
    }

    /// <summary>
    /// Native function declarations for OrbbecSDK.dll. Add new entries by
    /// transcribing from C:\dev\OrbbecSDK_v2\include\libobsensor\h\*.h, never
    /// from memory.
    /// </summary>
    internal static class OrbbecNative
    {
        private const string DLL = "OrbbecSDK";

        // === Error.h ===

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void ob_delete_error(IntPtr error);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern ObStatus ob_error_get_status(IntPtr error);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr ob_error_get_message(IntPtr error);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr ob_error_get_function(IntPtr error);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr ob_error_get_args(IntPtr error);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern ObExceptionType ob_error_get_exception_type(IntPtr error);

        // === Context.h ===

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr ob_create_context(out IntPtr error);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void ob_delete_context(IntPtr context, out IntPtr error);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr ob_query_device_list(IntPtr context, out IntPtr error);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void ob_set_extensions_directory(
            [MarshalAs(UnmanagedType.LPUTF8Str)] string directory,
            out IntPtr error);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void ob_set_logger_severity(ObLogSeverity severity, out IntPtr error);

        // === Device.h (device list) ===

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void ob_delete_device_list(IntPtr list, out IntPtr error);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern uint ob_device_list_get_count(IntPtr list, out IntPtr error);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr ob_device_list_get_device(IntPtr list, uint index, out IntPtr error);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr ob_device_list_get_device_by_serial_number(
            IntPtr list,
            [MarshalAs(UnmanagedType.LPUTF8Str)] string serialNumber,
            out IntPtr error);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr ob_device_list_get_device_serial_number(
            IntPtr list, uint index, out IntPtr error);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr ob_device_list_get_device_name(
            IntPtr list, uint index, out IntPtr error);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr ob_device_list_get_device_uid(
            IntPtr list, uint index, out IntPtr error);

        // === Device.h (device + info) ===

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void ob_delete_device(IntPtr device, out IntPtr error);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr ob_device_get_device_info(IntPtr device, out IntPtr error);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void ob_delete_device_info(IntPtr info, out IntPtr error);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr ob_device_info_get_serial_number(IntPtr info, out IntPtr error);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr ob_device_info_get_name(IntPtr info, out IntPtr error);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr ob_device_info_get_uid(IntPtr info, out IntPtr error);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr ob_device_info_get_firmware_version(IntPtr info, out IntPtr error);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr ob_device_info_get_connection_type(IntPtr info, out IntPtr error);

        // === MultipleDevices.h ===

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern ushort ob_device_get_supported_multi_device_sync_mode_bitmap(
            IntPtr device, out IntPtr error);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void ob_device_set_multi_device_sync_config(
            IntPtr device, ref ObMultiDeviceSyncConfig config, out IntPtr error);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern ObMultiDeviceSyncConfig ob_device_get_multi_device_sync_config(
            IntPtr device, out IntPtr error);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void ob_device_trigger_capture(IntPtr device, out IntPtr error);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void ob_device_timer_sync_with_host(IntPtr device, out IntPtr error);

        // === Device.h (global timestamp) ===

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        [return: MarshalAs(UnmanagedType.I1)]
        internal static extern bool ob_device_is_global_timestamp_supported(IntPtr device, out IntPtr error);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void ob_device_enable_global_timestamp(
            IntPtr device,
            [MarshalAs(UnmanagedType.I1)] bool enable,
            out IntPtr error);

        // === Advanced.h (depth work mode) ===

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr ob_device_get_current_depth_work_mode_name(IntPtr device, out IntPtr error);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern ObStatus ob_device_switch_depth_work_mode_by_name(
            IntPtr device,
            [MarshalAs(UnmanagedType.LPUTF8Str)] string modeName,
            out IntPtr error);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr ob_device_get_depth_work_mode_list(IntPtr device, out IntPtr error);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern uint ob_depth_work_mode_list_get_count(IntPtr list, out IntPtr error);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern ObDepthWorkMode ob_depth_work_mode_list_get_item(
            IntPtr list, uint index, out IntPtr error);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void ob_delete_depth_work_mode_list(IntPtr list, out IntPtr error);

        // === Pipeline.h ===

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr ob_create_pipeline_with_device(IntPtr device, out IntPtr error);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void ob_delete_pipeline(IntPtr pipeline, out IntPtr error);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void ob_pipeline_start_with_config(
            IntPtr pipeline, IntPtr config, out IntPtr error);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void ob_pipeline_stop(IntPtr pipeline, out IntPtr error);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr ob_pipeline_wait_for_frameset(
            IntPtr pipeline, uint timeoutMs, out IntPtr error);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void ob_pipeline_enable_frame_sync(IntPtr pipeline, out IntPtr error);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void ob_pipeline_disable_frame_sync(IntPtr pipeline, out IntPtr error);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern ObCameraParam ob_pipeline_get_camera_param(IntPtr pipeline, out IntPtr error);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern ObCameraParam ob_pipeline_get_camera_param_with_profile(
            IntPtr pipeline, uint colorWidth, uint colorHeight, uint depthWidth, uint depthHeight,
            out IntPtr error);

        // === Pipeline.h (config) ===

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr ob_create_config(out IntPtr error);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void ob_delete_config(IntPtr config, out IntPtr error);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void ob_config_enable_video_stream(
            IntPtr config, ObStreamType streamType,
            uint width, uint height, uint fps, ObFormat format,
            out IntPtr error);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void ob_config_set_align_mode(
            IntPtr config, ObAlignMode mode, out IntPtr error);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void ob_config_set_frame_aggregate_output_mode(
            IntPtr config, ObFrameAggregateOutputMode mode, out IntPtr error);

        // === Filter.h ===

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr ob_create_filter(
            [MarshalAs(UnmanagedType.LPUTF8Str)] string name, out IntPtr error);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void ob_delete_filter(IntPtr filter, out IntPtr error);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void ob_filter_set_config_value(
            IntPtr filter,
            [MarshalAs(UnmanagedType.LPUTF8Str)] string configName,
            double value,
            out IntPtr error);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern double ob_filter_get_config_value(
            IntPtr filter,
            [MarshalAs(UnmanagedType.LPUTF8Str)] string configName,
            out IntPtr error);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr ob_filter_process(
            IntPtr filter, IntPtr frame, out IntPtr error);

        // === Frame.h ===

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern void ob_delete_frame(IntPtr frame, out IntPtr error);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr ob_frame_get_data(IntPtr frame, out IntPtr error);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern uint ob_frame_get_data_size(IntPtr frame, out IntPtr error);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern ObFormat ob_frame_get_format(IntPtr frame, out IntPtr error);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern ObFrameType ob_frame_get_type(IntPtr frame, out IntPtr error);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern ulong ob_frame_get_index(IntPtr frame, out IntPtr error);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern ulong ob_frame_get_timestamp_us(IntPtr frame, out IntPtr error);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern ulong ob_frame_get_system_timestamp_us(IntPtr frame, out IntPtr error);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern ulong ob_frame_get_global_timestamp_us(IntPtr frame, out IntPtr error);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern float ob_points_frame_get_coordinate_value_scale(IntPtr frame, out IntPtr error);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern uint ob_video_frame_get_width(IntPtr frame, out IntPtr error);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern uint ob_video_frame_get_height(IntPtr frame, out IntPtr error);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr ob_frameset_get_depth_frame(IntPtr frameset, out IntPtr error);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr ob_frameset_get_color_frame(IntPtr frameset, out IntPtr error);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr ob_frameset_get_ir_frame(IntPtr frameset, out IntPtr error);

        [DllImport(DLL, CallingConvention = CallingConvention.Cdecl)]
        internal static extern IntPtr ob_frameset_get_points_frame(IntPtr frameset, out IntPtr error);

        // === Helpers (not P/Invoke) ===

        internal static string ReadUtf8(IntPtr p)
        {
            return p == IntPtr.Zero ? string.Empty : (Marshal.PtrToStringUTF8(p) ?? string.Empty);
        }
    }
}
