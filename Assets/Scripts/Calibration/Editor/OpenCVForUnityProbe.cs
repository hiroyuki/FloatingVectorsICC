using System.IO;
using System.Linq;
using System.Reflection;
using System.Text;
using UnityEditor;
using UnityEngine;

namespace Calibration.EditorTools
{
    public static class OpenCVForUnityProbe
    {
        [MenuItem("Window/Calibration/Probe OpenCV for Unity Aruco API")]
        public static void Probe()
        {
            var sb = new StringBuilder();
            var asm = System.AppDomain.CurrentDomain.GetAssemblies()
                .FirstOrDefault(a => a.GetName().Name == "EnoxSoftware.OpenCVForUnity");
            sb.AppendLine("EnoxSoftware.OpenCVForUnity loaded: " + (asm != null));
            if (asm != null)
            {
                sb.AppendLine();
                sb.AppendLine("=== ArucoModule + ObjdetectModule + Calib3dModule classes ===");
                foreach (var t in asm.GetTypes()
                    .Where(t => t.Namespace != null
                                && (t.Namespace.IndexOf("Aruco", System.StringComparison.OrdinalIgnoreCase) >= 0
                                 || t.Namespace.IndexOf("Objdetect", System.StringComparison.OrdinalIgnoreCase) >= 0
                                 || t.Namespace.IndexOf("Calib3d", System.StringComparison.OrdinalIgnoreCase) >= 0))
                    .OrderBy(t => t.FullName))
                {
                    sb.AppendLine(t.FullName);
                }

                sb.AppendLine();
                sb.AppendLine("=== Objdetect static methods ===");
                var objdetectStatic = asm.GetType("OpenCVForUnity.ObjdetectModule.Objdetect");
                if (objdetectStatic != null)
                {
                    foreach (var m in objdetectStatic.GetMethods(BindingFlags.Public | BindingFlags.Static)
                        .Where(m => m.Name.IndexOf("predefined", System.StringComparison.OrdinalIgnoreCase) >= 0
                                 || m.Name.IndexOf("dictionary", System.StringComparison.OrdinalIgnoreCase) >= 0)
                        .OrderBy(m => m.Name))
                    {
                        var ps = string.Join(", ", m.GetParameters().Select(p => p.ParameterType.Name + " " + p.Name));
                        sb.AppendLine(m.ReturnType.Name + " " + m.Name + "(" + ps + ")");
                    }
                }

                sb.AppendLine();
                sb.AppendLine("=== Objdetect constants (DICT_*) ===");
                if (objdetectStatic != null)
                {
                    foreach (var f in objdetectStatic.GetFields(BindingFlags.Public | BindingFlags.Static)
                        .Where(f => f.Name.IndexOf("DICT_", System.StringComparison.OrdinalIgnoreCase) >= 0)
                        .OrderBy(f => f.Name))
                    {
                        sb.AppendLine($"{f.FieldType.Name} {f.Name} = {f.GetValue(null)}");
                    }
                }

                sb.AppendLine();
                sb.AppendLine("=== CharucoBoard constructors ===");
                var charucoBoard = asm.GetType("OpenCVForUnity.ObjdetectModule.CharucoBoard");
                if (charucoBoard != null)
                {
                    foreach (var c in charucoBoard.GetConstructors())
                    {
                        var ps = string.Join(", ", c.GetParameters().Select(p => p.ParameterType.Name + " " + p.Name));
                        sb.AppendLine("CharucoBoard(" + ps + ")");
                    }
                }
            }
            string path = Path.Combine(Application.dataPath, "..", "_opencvforunity_probe.txt");
            File.WriteAllText(path, sb.ToString());
            Debug.Log("[OpenCVForUnityProbe] Wrote " + path);
        }
    }
}
