using System;
using System.Runtime.InteropServices;

namespace Orbbec
{
    /// <summary>Maps ob_status from ObTypes.h.</summary>
    public enum ObStatus
    {
        Ok = 0,
        Error = 1,
    }

    /// <summary>Maps ob_exception_type from ObTypes.h.</summary>
    public enum ObExceptionType
    {
        Unknown = 0,
        StdException = 1,
        CameraDisconnected = 2,
        Platform = 3,
        InvalidValue = 4,
        WrongApiCallSequence = 5,
        NotImplemented = 6,
        IO = 7,
        Memory = 8,
        UnsupportedOperation = 9,
        AccessDenied = 10,
    }

    /// <summary>
    /// Wraps an ob_error returned by OrbbecSDK. Construct via <see cref="ThrowIfNotEmpty"/>
    /// after every native call that takes an ob_error**.
    /// </summary>
    public class OrbbecException : Exception
    {
        public ObExceptionType ExceptionType { get; }
        public ObStatus Status { get; }
        public string FunctionName { get; }
        public string CallArgs { get; }

        private OrbbecException(string message, ObStatus status, string func, string args, ObExceptionType type)
            : base(BuildMessage(message, func, args, type))
        {
            ExceptionType = type;
            Status = status;
            FunctionName = func;
            CallArgs = args;
        }

        private static string BuildMessage(string msg, string func, string args, ObExceptionType type)
        {
            return $"OrbbecSDK error in {func}({args}): {msg} [type={type}]";
        }

        /// <summary>
        /// If <paramref name="errorPtr"/> is non-null, read its fields, free it,
        /// then throw an <see cref="OrbbecException"/>.
        /// </summary>
        public static void ThrowIfNotEmpty(IntPtr errorPtr)
        {
            if (errorPtr == IntPtr.Zero) return;

            ObStatus status;
            string message, function, args;
            ObExceptionType type;
            try
            {
                status = OrbbecNative.ob_error_get_status(errorPtr);
                message = PtrToUtf8(OrbbecNative.ob_error_get_message(errorPtr));
                function = PtrToUtf8(OrbbecNative.ob_error_get_function(errorPtr));
                args = PtrToUtf8(OrbbecNative.ob_error_get_args(errorPtr));
                type = OrbbecNative.ob_error_get_exception_type(errorPtr);
            }
            finally
            {
                OrbbecNative.ob_delete_error(errorPtr);
            }
            throw new OrbbecException(message, status, function, args, type);
        }

        private static string PtrToUtf8(IntPtr p)
        {
            return p == IntPtr.Zero ? string.Empty : (Marshal.PtrToStringUTF8(p) ?? string.Empty);
        }
    }
}
