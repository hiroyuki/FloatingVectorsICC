// Offline bodies_main recompute (issue #35).
//
// Reads a recording's depth_main + ir_main frames, feeds each capture through
// k4abt via the live K4abtWorkerHost machinery, and rewrites bodies_main from
// the result — tagging every body frame with its SOURCE depth-frame timestamp.
// This guarantees body↔depth timestamp alignment (the original recordings had a
// ~49h skew because bodies_main was stamped from a different clock) and keeps
// the skeleton in the same camera frame as the depth mesh, which #34 needs.
//
// Why a deterministic offline driver instead of the live playback BT path: the
// worker only ever processes its LATEST input slot (real-time, drops frames). To
// get an exact 1:1 depth→bodies mapping we feed one capture, BLOCK until that
// frame's skeletons come back, then feed the next. Runs in edit mode driven by a
// cancelable progress bar; no Play mode needed.
//
// Windows only (k4abt). On other platforms StartWorker no-ops and the run aborts.

using System;
using System.Collections.Generic;
using System.IO;
using System.Threading;
using BodyTracking;
using Orbbec;
using PointCloud;
using UnityEditor;
using UnityEngine;
using Debug = UnityEngine.Debug;

namespace BodyTracking.EditorTools
{
    public sealed class OfflineBodyRecomputeWindow : EditorWindow
    {
        private const string PrefKeyRoot = "FloatingVectors.OfflineBodyRecompute.Root";

        private string _root = "";
        private bool _backupExisting = true;
        private bool _singlePersonOnly = true; // CLAUDE.md: 1-person install — keep body index 0
        private Vector2 _scroll;
        private readonly List<string> _log = new List<string>();

        // Per-frame handshake state, written by the OnSkeletonsReady handler.
        private bool _gotOutput;
        private ulong _outTsNs;
        private int _outBodyCount;
        private BodySnapshot[] _outBodies;

        // Per-frame timeout: a single enqueue→pop normally returns in well under a
        // second; 10s covers first-frame GPU warm-up without hanging forever.
        private const int FrameTimeoutMs = 10000;
        // The host's ReadyWatcher kills the worker at K4abtWorkerSharedLayout.ReadyWaitTimeoutMs
        // (15s) if the BT model hasn't loaded; give it a little headroom so a genuine
        // kill is detected (via Pump → HasExited) before this fallback fires.
        private const int ReadyTimeoutMs = 20000;

        [MenuItem("Window/Body Tracking/Recompute Bodies (offline)")]
        public static void Open()
        {
            var w = GetWindow<OfflineBodyRecomputeWindow>("Recompute Bodies");
            w.minSize = new Vector2(460, 320);
            w._root = EditorPrefs.GetString(PrefKeyRoot, "");
            w.Show();
        }

        private void OnGUI()
        {
            EditorGUILayout.HelpBox(
                "Re-runs k4abt on a recording's depth_main + ir_main and rewrites bodies_main " +
                "with depth-aligned timestamps (issue #35). Windows + BT SDK required. " +
                "This blocks the Editor while running; use Cancel in the progress bar to stop.",
                MessageType.Info);

            EditorGUILayout.Space();
            using (new EditorGUILayout.HorizontalScope())
            {
                EditorGUILayout.PrefixLabel("Recording root");
                _root = EditorGUILayout.TextField(_root);
                if (GUILayout.Button("Browse", GUILayout.Width(70)))
                {
                    string picked = EditorUtility.OpenFolderPanel("Select recording root (contains dataset/)", _root, "");
                    if (!string.IsNullOrEmpty(picked)) { _root = picked; EditorPrefs.SetString(PrefKeyRoot, _root); GUI.FocusControl(null); }
                }
            }
            EditorGUILayout.LabelField(" ", "Folder that contains 'dataset/' and 'calibration/extrinsics.yaml'.", EditorStyles.miniLabel);

            _backupExisting = EditorGUILayout.Toggle(
                new GUIContent("Back up existing bodies_main", "Rename the old bodies_main to bodies_main.<timestamp>.bak before overwriting."),
                _backupExisting);
            _singlePersonOnly = EditorGUILayout.Toggle(
                new GUIContent("Single person (keep body 0)", "Keep only the first detected body per frame (CLAUDE.md: 1-person install)."),
                _singlePersonOnly);

            EditorGUILayout.Space();
            using (new EditorGUI.DisabledScope(string.IsNullOrEmpty(_root)))
            {
                if (GUILayout.Button("Recompute bodies_main", GUILayout.Height(30)))
                {
                    EditorPrefs.SetString(PrefKeyRoot, _root);
                    RunAll();
                }
            }

            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Log", EditorStyles.boldLabel);
            using (var sv = new EditorGUILayout.ScrollViewScope(_scroll, GUILayout.ExpandHeight(true)))
            {
                _scroll = sv.scrollPosition;
                foreach (var line in _log) EditorGUILayout.LabelField(line, EditorStyles.miniLabel);
            }
        }

        private void Log(string msg)
        {
            _log.Add(msg);
            if (_log.Count > 500) _log.RemoveAt(0);
            Debug.Log($"[OfflineBodyRecompute] {msg}");
            Repaint();
        }

        private void RunAll()
        {
            if (Application.platform != RuntimePlatform.WindowsEditor)
            {
                Log("ABORT: k4abt is Windows-only; run this on the Windows machine.");
                return;
            }
            if (!Directory.Exists(PointCloudRecording.DatasetRoot(_root)))
            {
                Log($"ABORT: no 'dataset/' under '{_root}'.");
                return;
            }

            // Calibration is required to build the k4abt calibration blob.
            Dictionary<string, PointCloudRecording.DeviceCalibration> calibBySerial;
            try
            {
                calibBySerial = new Dictionary<string, PointCloudRecording.DeviceCalibration>();
                foreach (var c in PointCloudRecording.ReadExtrinsicsYaml(_root))
                    calibBySerial[c.Serial] = c;
            }
            catch (Exception e)
            {
                Log($"ABORT: calibration/extrinsics.yaml — {e.Message}");
                return;
            }

            var devices = new List<(string serial, string deviceDir)>();
            foreach (var d in PointCloudRecording.EnumerateDevices(_root)) devices.Add(d);
            if (devices.Count == 0) { Log("ABORT: no devices under dataset/."); return; }

            _log.Clear();
            Log($"Found {devices.Count} device(s) under {_root}");

            // One hidden host for the whole batch; its OnDestroy tears down any worker.
            var hostGo = EditorUtility.CreateGameObjectWithHideFlags("OfflineBTHost", HideFlags.HideAndDontSave);
            var host = hostGo.AddComponent<K4abtWorkerHost>();
            host.OnSkeletonsReady += OnSkeletons;
            _outBodies = new BodySnapshot[BodyTracking.Shared.K4abtWorkerSharedLayout.MaxBodies];
            for (int i = 0; i < _outBodies.Length; i++) _outBodies[i] = new BodySnapshot();

            bool canceled = false;
            try
            {
                foreach (var (serial, deviceDir) in devices)
                {
                    if (!calibBySerial.TryGetValue(serial, out var calib))
                    {
                        Log($"SKIP {serial}: no calibration entry in extrinsics.yaml");
                        continue;
                    }
                    canceled = !RunDevice(host, serial, deviceDir, calib);
                    if (canceled) { Log("Canceled."); break; }
                }
            }
            catch (Exception e)
            {
                Log($"ERROR: {e}");
            }
            finally
            {
                host.OnSkeletonsReady -= OnSkeletons;
                UnityEngine.Object.DestroyImmediate(hostGo);
                EditorUtility.ClearProgressBar();
            }
            if (!canceled) Log("Done.");
        }

        // Returns false if the user canceled.
        private bool RunDevice(K4abtWorkerHost host, string serial, string deviceDir,
                               PointCloudRecording.DeviceCalibration calib)
        {
            string depthPath = Path.Combine(deviceDir, PointCloudRecording.DepthSensorName);
            string irPath = Path.Combine(deviceDir, PointCloudRecording.IRSensorName);
            string bodiesPath = Path.Combine(deviceDir, PointCloudRecording.BodiesSensorName);

            if (!File.Exists(depthPath)) { Log($"SKIP {serial}: no depth_main"); return true; }
            if (!File.Exists(irPath)) { Log($"SKIP {serial}: no ir_main (k4abt requires IR)"); return true; }

            var (depthW, depthH) = PointCloudRecording.ReadRcsvHeaderDimensions(depthPath);
            var (irW, irH) = PointCloudRecording.ReadRcsvHeaderDimensions(irPath);
            if (depthW <= 0 || depthH <= 0) { Log($"SKIP {serial}: depth_main header has no dimensions"); return true; }
            if (irW <= 0 || irH <= 0) { irW = depthW; irH = depthH; }

            // Color resolution only feeds the calibration blob's color enum (k4abt uses
            // depth). Prefer the recorded color stream's header, fall back to intrinsics.
            string colorPath = Path.Combine(deviceDir, PointCloudRecording.ColorSensorName);
            var (colorW, colorH) = PointCloudRecording.ReadRcsvHeaderDimensions(colorPath);
            if (colorW <= 0 || colorH <= 0) { colorW = calib.ColorIntrinsic.Width; colorH = calib.ColorIntrinsic.Height; }

            var camParam = new ObCameraParam
            {
                DepthIntrinsic = calib.DepthIntrinsic,
                RgbIntrinsic = calib.ColorIntrinsic,
                DepthDistortion = calib.DepthDistortion,
                RgbDistortion = calib.ColorDistortion,
                Transform = calib.DepthToColor,
                IsMirrored = false,
            };

            using var depthStream = new PointCloudRecording.RcsvFrameStream(depthPath);
            using var irStream = new PointCloudRecording.RcsvFrameStream(irPath);

            // IR ts -> record index, so we can pair each depth frame with the IR frame
            // captured at the same instant (robust to any count mismatch / reordering).
            var irByTs = new Dictionary<ulong, int>(irStream.Count);
            for (int i = 0; i < irStream.Count; i++) irByTs[irStream.TimestampNsAt(i)] = i;

            int expectedDepth = depthW * depthH * 2;
            int expectedIr = irW * irH * 2;
            byte[] depthBuf = new byte[expectedDepth];
            byte[] irBuf = new byte[expectedIr];

            Log($"{serial}: {depthStream.Count} depth frames, {irStream.Count} ir frames @ depth {depthW}x{depthH}");

            if (!host.StartWorker(serial, camParam, depthW, depthH, irW, irH, colorW, colorH))
            {
                Log($"SKIP {serial}: StartWorker failed (worker exe missing / not Windows). See console.");
                return true;
            }

            try
            {
                // Wait for the worker to load the BT model.
                long readyDeadline = NowMs() + ReadyTimeoutMs;
                while (!host.IsReady(serial))
                {
                    if (!host.HasSession(serial)) { Log($"SKIP {serial}: worker died during startup."); return true; }
                    host.Pump();
                    Thread.Sleep(5);
                    if (NowMs() > readyDeadline) { Log($"SKIP {serial}: worker not ready in {ReadyTimeoutMs}ms."); return true; }
                    if (EditorUtility.DisplayCancelableProgressBar("Recompute bodies", $"{serial}: waiting for BT model…", 0f))
                        return false;
                }

                int total = depthStream.Count;
                int irMissing = 0, noResult = 0;
                var outFrames = new List<PointCloudRecording.Frame>(total);

                for (int i = 0; i < total; i++)
                {
                    if ((i & 7) == 0 &&
                        EditorUtility.DisplayCancelableProgressBar(
                            "Recompute bodies", $"{serial}: frame {i + 1}/{total}", (float)i / Math.Max(1, total)))
                    {
                        return false;
                    }

                    var df = depthStream[i];
                    ulong ts = df.TimestampNs;
                    int dN = Math.Min(df.ByteCount, expectedDepth);
                    Array.Copy(df.Bytes, depthBuf, dN);

                    byte[] irArg = null; int irArgBytes = 0;
                    if (irByTs.TryGetValue(ts, out int irIdx))
                    {
                        var irf = irStream[irIdx]; // NB: invalidates depthStream's scratch is separate (own stream)
                        int iN = Math.Min(irf.ByteCount, expectedIr);
                        Array.Copy(irf.Bytes, irBuf, iN);
                        irArg = irBuf; irArgBytes = iN;
                    }
                    else irMissing++;

                    // Feed one capture and block until its skeletons return.
                    _gotOutput = false; _outBodyCount = 0; _outTsNs = 0;
                    if (!host.EnqueueFrame(serial, depthBuf, dN, irArg, irArgBytes, ts))
                    {
                        // Worker not ready / buffer rejected — record an empty frame to
                        // keep depth↔bodies index alignment, then continue.
                        AppendBodies(outFrames, ts, 0);
                        noResult++;
                        continue;
                    }

                    long deadline = NowMs() + FrameTimeoutMs;
                    bool matched = false;
                    while (NowMs() <= deadline)
                    {
                        if (!host.HasSession(serial)) { Log($"{serial}: worker died at frame {i}."); return true; }
                        host.Pump();
                        if (_gotOutput)
                        {
                            // Accept only the output stamped with THIS frame's depth ts.
                            // A late result from a previously timed-out frame carries an
                            // older ts — discard it and keep waiting, so one stale output
                            // can't cascade into a run of empty frames (1:1 must hold).
                            if (_outTsNs == ts) { matched = true; break; }
                            _gotOutput = false;
                        }
                        Thread.Sleep(1);
                    }

                    if (!matched) { AppendBodies(outFrames, ts, 0); noResult++; continue; }

                    int count = _singlePersonOnly ? Math.Min(_outBodyCount, 1) : _outBodyCount;
                    AppendBodies(outFrames, ts, count);
                }

                WriteBodies(serial, bodiesPath, outFrames);
                Log($"{serial}: wrote {outFrames.Count} body frames" +
                    (irMissing > 0 ? $" ({irMissing} without matching IR)" : "") +
                    (noResult > 0 ? $" ({noResult} empty/no-result)" : ""));
            }
            finally
            {
                host.StopWorker(serial);
            }
            return true;
        }

        // Encodes _outBodies[0..count) for the just-tracked frame into a fresh payload
        // and appends it stamped with the source depth timestamp.
        private void AppendBodies(List<PointCloudRecording.Frame> frames, ulong tsNs, int count)
        {
            int sz = RecordedBodySerializer.FrameSize(count);
            byte[] payload = new byte[sz];
            RecordedBodySerializer.Encode(_outBodies, count, payload);
            frames.Add(new PointCloudRecording.Frame { TimestampNs = tsNs, Bytes = payload, ByteCount = sz });
        }

        private void WriteBodies(string serial, string bodiesPath, List<PointCloudRecording.Frame> frames)
        {
            if (_backupExisting && File.Exists(bodiesPath))
            {
                string bak = $"{bodiesPath}.{DateTime.Now:yyyyMMdd_HHmmss}.bak";
                File.Move(bodiesPath, bak);
                Log($"{serial}: backed up old bodies_main -> {Path.GetFileName(bak)}");
            }
            PointCloudRecording.WriteRcsv(bodiesPath, PointCloudRecording.BuildBodiesHeaderYaml(serial), frames);
        }

        // Fired synchronously from K4abtWorkerHost.Pump on this thread. Copy out of the
        // host's reused buffer before returning.
        private void OnSkeletons(string serial, ulong tsNs, BodySnapshot[] bodies, int count)
        {
            int n = Math.Min(count, _outBodies.Length);
            for (int i = 0; i < n; i++)
            {
                _outBodies[i].Id = bodies[i].Id;
                Array.Copy(bodies[i].Joints, _outBodies[i].Joints, _outBodies[i].Joints.Length);
            }
            _outBodyCount = n;
            _outTsNs = tsNs;
            _gotOutput = true;
        }

        private static long NowMs() => DateTime.UtcNow.Ticks / TimeSpan.TicksPerMillisecond;
    }
}
