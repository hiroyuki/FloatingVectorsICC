using System.IO;
using UnityEditor;
using UnityEditor.TestTools.TestRunner.Api;
using UnityEngine;

namespace Calibration.EditorTools
{
    public static class CalibrationTestRunner
    {
        [MenuItem("Window/Calibration/Run Math Tests")]
        public static void RunMathTests()
        {
            var api = ScriptableObject.CreateInstance<TestRunnerApi>();
            var filter = new Filter
            {
                testMode = TestMode.EditMode,
                assemblyNames = new[] { "Calibration.Tests.Editor" },
            };
            api.RegisterCallbacks(new Callbacks());
            api.Execute(new ExecutionSettings(filter));
            Debug.Log("[CalibrationTestRunner] Started Calibration.Tests.Editor");
        }

        private class Callbacks : ICallbacks
        {
            public void RunStarted(ITestAdaptor testsToRun) { }
            public void TestStarted(ITestAdaptor test) { }
            public void TestFinished(ITestResultAdaptor result) { }

            public void RunFinished(ITestResultAdaptor testResults)
            {
                string path = Path.Combine(Application.dataPath, "..", "_test_results.txt");
                using var sw = new StreamWriter(path);
                sw.WriteLine($"status: {testResults.TestStatus}");
                sw.WriteLine($"pass: {testResults.PassCount}");
                sw.WriteLine($"fail: {testResults.FailCount}");
                sw.WriteLine($"inconclusive: {testResults.InconclusiveCount}");
                sw.WriteLine($"skip: {testResults.SkipCount}");
                sw.WriteLine($"duration: {testResults.Duration:F3}s");
                sw.WriteLine();
                Dump(sw, testResults, 0);
                Debug.Log($"[CalibrationTestRunner] {testResults.PassCount} pass, {testResults.FailCount} fail (status {testResults.TestStatus})");
            }

            private static void Dump(StreamWriter sw, ITestResultAdaptor r, int depth)
            {
                string indent = new string(' ', depth * 2);
                if (r.Children == null || !r.HasChildren)
                {
                    sw.WriteLine($"{indent}[{r.TestStatus}] {r.Name} ({r.Duration:F3}s)");
                    if (!string.IsNullOrEmpty(r.Message))
                    {
                        foreach (var line in r.Message.Split('\n'))
                            sw.WriteLine($"{indent}  msg: {line}");
                    }
                }
                else
                {
                    sw.WriteLine($"{indent}{r.Name}: {r.PassCount}/{r.PassCount + r.FailCount}");
                    foreach (var c in r.Children) Dump(sw, c, depth + 1);
                }
            }
        }
    }
}
