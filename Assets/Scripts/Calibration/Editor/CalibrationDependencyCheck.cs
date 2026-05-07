using System;
using System.Linq;
using System.Reflection;
using UnityEditor;
using UnityEngine;

namespace Calibration.EditorTools
{
    public static class CalibrationDependencyCheck
    {
        [MenuItem("Window/Calibration/Check Dependencies")]
        public static void Check()
        {
            string opencv = ProbeOpenCVForUnity();
            string yaml = ProbeYamlDotNet();
            Debug.Log($"[Calibration] OpenCV for Unity: {opencv}");
            Debug.Log($"[Calibration] YamlDotNet: {yaml}");
        }

        public static bool IsOpenCVForUnityAvailable()
        {
            return Type.GetType("OpenCVForUnity.CoreModule.Mat, OpenCVForUnity") != null
                || FindOpenCVAssembly() != null;
        }

        public static bool IsYamlDotNetAvailable()
        {
            return Type.GetType("YamlDotNet.RepresentationModel.YamlStream, YamlDotNet") != null;
        }

        private static Assembly FindOpenCVAssembly()
        {
            return AppDomain.CurrentDomain.GetAssemblies()
                .FirstOrDefault(a => a.GetName().Name == "EnoxSoftware.OpenCVForUnity");
        }

        private static string ProbeOpenCVForUnity()
        {
            try
            {
                var asm = FindOpenCVAssembly();
                if (asm == null) return "missing — Assets/OpenCVForUnity/ not imported";

                // Cv2.getVersionString() lives in OpenCVForUnity.CoreModule.Core
                var coreType = asm.GetType("OpenCVForUnity.CoreModule.Core");
                if (coreType == null) return "loaded but Core type not found";

                var versionMethod = coreType.GetMethod("getVersionString", BindingFlags.Public | BindingFlags.Static);
                if (versionMethod == null) return $"loaded (asm version {asm.GetName().Version}) but getVersionString missing";

                string version = (string)versionMethod.Invoke(null, null);
                return $"OK (OpenCV native version {version}, asm {asm.GetName().Version})";
            }
            catch (Exception e)
            {
                return $"failed — {e.GetType().Name}: {e.Message}";
            }
        }

        private static string ProbeYamlDotNet()
        {
            try
            {
                var yamlStream = Type.GetType("YamlDotNet.RepresentationModel.YamlStream, YamlDotNet");
                if (yamlStream == null) return "missing — YamlDotNet.dll not loaded";
                var asm = yamlStream.Assembly;
                return $"OK (assembly {asm.GetName().Version})";
            }
            catch (Exception e)
            {
                return $"failed — {e.GetType().Name}: {e.Message}";
            }
        }
    }
}
