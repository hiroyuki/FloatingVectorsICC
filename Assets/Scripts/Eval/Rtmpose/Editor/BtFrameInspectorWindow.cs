// BT Frame Inspector — per-camera, per-frame body-tracking debug view.
//
// Pick a frame (or grab the current playback frame), pick a camera (or all 4
// fused in world space), and the window spawns into the scene:
//   - that frame's point cloud (toggle)
//   - the recorded k4abt skeleton (cyan, joints + bones)
//   - the RTMPose skeleton computed on the spot (orange, joints + bones)
// so each camera's tracking can be verified one by one at the exact frame.
// Works in Edit mode (no Play needed). Scene is never saved by this tool.
//
// Menu: FloatingVectors > Eval BT > Frame Inspector

using System;
using System.Collections.Generic;
using System.IO;
using BodyTracking;
using Calibration;
using Orbbec;
using PointCloud;
using UnityEditor;
using UnityEngine;

namespace BodyTracking.Eval.Rtmpose
{
    public sealed class BtFrameInspectorWindow : EditorWindow
    {
        [MenuItem("FloatingVectors/Eval BT/Frame Inspector", priority = 10)]
        public static void Open() => GetWindow<BtFrameInspectorWindow>("BT Frame Inspector");

        string _root = "D:/Dropbox/projects/ICC/Recordings/RecordingBase/2026-07-14_15-50-24";
        string _host = "PAN-SHI";
        int _frame = 800;
        // worldSpace defaults ON so inspection viz lands where the playback clouds
        // are — with it OFF the viz sits at the origin in camera-local coords and
        // looks like a misaligned "second person" next to the playback.
        bool _showCloud = true, _showK4abt = true, _showRtmpose = true, _worldSpace = true;
        Vector2 _scroll;
        string _log = "";

        // cached inference backend (ONNX load is slow — keep it alive)
        static OrtRtmposeBackend s_backend;
        static readonly List<IDisposable> s_keep = new List<IDisposable>();

        static readonly Color K4Col = Color.cyan;
        static readonly Color RtCol = new Color(1f, 0.45f, 0.05f);

        // 15-joint bone pairs for the RTMPose skeleton
        static readonly (EvalJointId a, EvalJointId b)[] EvalBones =
        {
            (EvalJointId.Pelvis, EvalJointId.Neck), (EvalJointId.Neck, EvalJointId.Head),
            (EvalJointId.Neck, EvalJointId.ShoulderL), (EvalJointId.ShoulderL, EvalJointId.ElbowL), (EvalJointId.ElbowL, EvalJointId.WristL),
            (EvalJointId.Neck, EvalJointId.ShoulderR), (EvalJointId.ShoulderR, EvalJointId.ElbowR), (EvalJointId.ElbowR, EvalJointId.WristR),
            (EvalJointId.Pelvis, EvalJointId.HipL), (EvalJointId.HipL, EvalJointId.KneeL), (EvalJointId.KneeL, EvalJointId.AnkleL),
            (EvalJointId.Pelvis, EvalJointId.HipR), (EvalJointId.HipR, EvalJointId.KneeR), (EvalJointId.KneeR, EvalJointId.AnkleR),
        };

        void OnGUI()
        {
            _scroll = EditorGUILayout.BeginScrollView(_scroll);
            EditorGUILayout.LabelField("Session", EditorStyles.boldLabel);
            _root = EditorGUILayout.TextField("Session Root", _root);
            _host = EditorGUILayout.TextField("Host", _host);

            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Frame", EditorStyles.boldLabel);
            using (new EditorGUILayout.HorizontalScope())
            {
                _frame = EditorGUILayout.IntField("Frame Index", _frame);
                if (GUILayout.Button("Grab & Freeze", GUILayout.Width(110))) GrabPlaybackFrame();
                using (new EditorGUI.DisabledScope(!EditorApplication.isPaused))
                    if (GUILayout.Button("Resume ▶", GUILayout.Width(80)))
                    {
                        ClearViz(); // inspection viz would linger over the playback as a "second person"
                        EditorApplication.isPaused = false;
                    }
            }
            if (Application.isPlaying)
                EditorGUILayout.LabelField(EditorApplication.isPaused
                    ? "⏸ FROZEN — ribbons/point cloud held at this instant"
                    : "▶ playing — Grab & Freeze pauses everything at the moment you click", EditorStyles.miniLabel);

            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Show", EditorStyles.boldLabel);
            _showCloud = EditorGUILayout.Toggle("Point cloud", _showCloud);
            _showK4abt = EditorGUILayout.Toggle("k4abt (cyan)", _showK4abt);
            _showRtmpose = EditorGUILayout.Toggle("RTMPose (orange)", _showRtmpose);
            _worldSpace = EditorGUILayout.Toggle("World space (extrinsics)", _worldSpace);

            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Inspect camera", EditorStyles.boldLabel);
            var serials = ListSerials();
            using (new EditorGUILayout.HorizontalScope())
            {
                foreach (var s in serials)
                    if (GUILayout.Button(s.Substring(Mathf.Max(0, s.Length - 4)))) Inspect(new[] { s });
            }
            if (GUILayout.Button("All cameras (world fused)")) { _worldSpace = true; Inspect(serials.ToArray()); }
            if (GUILayout.Button("Clear scene viz")) ClearViz();

            EditorGUILayout.Space();
            EditorGUILayout.LabelField("Result", EditorStyles.boldLabel);
            EditorGUILayout.TextArea(_log, GUILayout.MinHeight(120));
            EditorGUILayout.EndScrollView();
        }

        List<string> ListSerials()
        {
            var list = new List<string>();
            try { foreach (var (serial, _) in PointCloudRecording.EnumerateDevices(_root)) list.Add(serial); } catch { }
            list.Sort();
            return list;
        }

        void GrabPlaybackFrame()
        {
            // Freeze the WHOLE editor first (recorder pause alone lets the ribbon
            // effects keep fading in real time), THEN read the frozen playhead.
            if (Application.isPlaying) EditorApplication.isPaused = true;

            var recs = FindObjectsByType<SensorRecorder>(FindObjectsSortMode.None);
            if (recs.Length == 0) { _log = "no SensorRecorder (enter Play + load a recording first)"; return; }
            var rec = recs[0];
            double sec = rec.CurrentPlayheadSeconds;
            // map seconds -> nearest depth frame index of the first track
            var tracks = rec.GetRecordedDepthTracks();
            if (tracks.Count == 0) { _log = "no tracks loaded"; return; }
            var frames = tracks[0].DepthFrames;
            if (frames.Count == 0) { _log = "no depth frames"; return; }
            ulong t0 = frames[0].TimestampNs;
            int best = 0; double bd = double.MaxValue;
            for (int i = 0; i < frames.Count; i++)
            {
                double t = (frames[i].TimestampNs - t0) / 1e9;
                double d = Math.Abs(t - sec);
                if (d < bd) { bd = d; best = i; }
            }
            _frame = best;
            _log = $"grabbed playback frame: {sec:F2}s -> frame {best} (Δ{bd * 1000:F0}ms)" +
                   (Application.isPlaying ? "\neditor FROZEN — inspect cameras now; Resume ▶ to continue playback" : "");
        }

        void Inspect(string[] serials)
        {
            ClearViz();
            try
            {
                EnsureBackend();
                var calib = PointCloudRecording.ReadExtrinsicsYaml(_root);
                var parent = new GameObject("BTInspect");
                var sb = new System.Text.StringBuilder();
                sb.AppendLine($"frame={_frame} world={_worldSpace}");

                // configure adapter volume selection once (world transforms from calib)
                var adapter = new RtmPoseAdapter(s_backend) { confThreshold = 0.3f, redetectEveryN = 1 };
                if (calib != null)
                    foreach (var dc in calib)
                        if (dc.GlobalTrColorCamera.HasValue) adapter.SetWorldTransform(dc.Serial, dc.GlobalTrColorCamera.Value);
                adapter.SetCaptureVolume(new Vector3(0, 200, 3000), new Vector3(1100, 1500, 1100));
                EvalSkeleton lastSkel = null;
                adapter.OnSkeletons += f => lastSkel = f.Primary();

                foreach (var serial in serials)
                {
                    var dc = FindCalib(calib, serial);
                    if (dc == null) { sb.AppendLine($"{serial}: no calibration"); continue; }
                    var cam = new ObCameraParam
                    {
                        DepthIntrinsic = dc.DepthIntrinsic, RgbIntrinsic = dc.ColorIntrinsic,
                        DepthDistortion = dc.DepthDistortion, RgbDistortion = dc.ColorDistortion,
                        Transform = dc.DepthToColor, IsMirrored = false,
                    };
                    adapter.Configure(new EvalCameraContext(serial,
                        dc.DepthIntrinsic.Width, dc.DepthIntrinsic.Height,
                        dc.ColorIntrinsic.Width, dc.ColorIntrinsic.Height, cam));

                    string devDir = Path.Combine(PointCloudRecording.HostDir(_root, _host), $"FemtoBolt_{serial}");
                    string colorPath = Path.Combine(devDir, PointCloudRecording.ColorSensorName);
                    string depthPath = Path.Combine(devDir, PointCloudRecording.DepthSensorName);
                    string bodiesPath = Path.Combine(devDir, PointCloudRecording.BodiesSensorName);
                    if (!File.Exists(colorPath) || !File.Exists(depthPath)) { sb.AppendLine($"{serial}: streams missing"); continue; }

                    byte[] color, depth; ulong ts;
                    int cw, ch, dw, dh;
                    using (var cs = new PointCloudRecording.RcsvFrameStream(colorPath))
                    {
                        int fi = Mathf.Clamp(_frame, 0, cs.Count - 1);
                        var f = cs[fi]; color = Copy(f); ts = f.TimestampNs;
                    }
                    (cw, ch) = PointCloudRecording.ReadRcsvHeaderDimensions(colorPath);
                    using (var ds = new PointCloudRecording.RcsvFrameStream(depthPath)) { depth = Copy(ds[Nearest(ds, ts)]); }
                    (dw, dh) = PointCloudRecording.ReadRcsvHeaderDimensions(depthPath);

                    // per-camera parent (world extrinsic or identity), flipY like the renderer
                    var camGo = new GameObject($"cam_{serial}");
                    camGo.transform.SetParent(parent.transform, false);
                    if (_worldSpace && dc.GlobalTrColorCamera.HasValue)
                        ExtrinsicsApply.ApplyToTransform(camGo.transform, dc.GlobalTrColorCamera.Value);
                    camGo.transform.localScale = new Vector3(1f, -1f, 1f);

                    if (_showCloud)
                    {
                        var recon = new PointCloudReconstructor($"inspect_{serial}");
                        if (recon.Dispatch(depth, depth.Length, dw, dh, color, color.Length, cw, ch, cam))
                        {
                            s_keep.Add(recon);
                            var cg = new GameObject("cloud");
                            cg.transform.SetParent(camGo.transform, false);
                            cg.AddComponent<MeshFilter>().sharedMesh = recon.Mesh;
                            var mr = cg.AddComponent<MeshRenderer>();
                            var sh = Shader.Find("Orbbec/PointCloudUnlit");
                            if (sh != null) mr.sharedMaterial = new Material(sh);
                        }
                        else recon.Dispose();
                    }

                    int k4n = 0, rtn = 0;
                    if (_showK4abt) k4n = SpawnK4abt(camGo.transform, bodiesPath, ts, cam);
                    if (_showRtmpose)
                    {
                        lastSkel = null;
                        var raw = new RawFrameData(depth, depth.Length, dw, dh, color, color.Length, cw, ch, Array.Empty<byte>(), 0, 0, 0, ts / 1000UL);
                        adapter.SubmitFrame(serial, raw, ts);
                        if (lastSkel != null) rtn = SpawnRtmpose(camGo.transform, lastSkel);
                    }
                    sb.AppendLine($"{serial}: k4abt joints={k4n}, rtmpose joints={rtn}");
                }

                FrameSceneView(parent);
                _log = sb.ToString() + "\ncyan=k4abt orange=RTMPose. Rotate the Scene view; use toggles + camera buttons to isolate.";
            }
            catch (Exception e) { _log = "EXCEPTION: " + e.Message + "\n" + e.StackTrace; }
        }

        void EnsureBackend()
        {
            if (s_backend != null) return;
            string modelsDir = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "eval", "models"));
            string yolox = FirstOnnx(Path.Combine(modelsDir, "yolox-m"));
            string rtm = FirstOnnx(Path.Combine(modelsDir, "rtmpose-m"));
            if (yolox == null || rtm == null) throw new Exception("ONNX models not found under eval/models");
            s_backend = new OrtRtmposeBackend(yolox, rtm);
        }

        int SpawnK4abt(Transform parent, string bodiesPath, ulong ts, ObCameraParam cam)
        {
            if (!File.Exists(bodiesPath)) return 0;
            using var bs = new PointCloudRecording.RcsvFrameStream(bodiesPath);
            int bi = Nearest(bs, ts); if (bi < 0) return 0;
            var f = bs[bi];
            var buf = new BodySnapshot[6]; for (int i = 0; i < buf.Length; i++) buf[i] = new BodySnapshot();
            int n = RecordedBodySerializer.Decode(f.Bytes, f.ByteCount, buf); if (n <= 0) return 0;
            var body = buf[0];
            var R = cam.Transform.Rot; var T = cam.Transform.Trans;
            var pos = new Dictionary<int, Vector3>();
            int cnt = 0;
            for (int ji = 0; ji < K4ABTConsts.K4ABT_JOINT_COUNT; ji++)
            {
                if (!BodyTrackingShared.IsDrawnJoint((k4abt_joint_id_t)ji)) continue;
                var jt = body.Joints[ji];
                if (jt.ConfidenceLevel <= k4abt_joint_confidence_level_t.K4ABT_JOINT_CONFIDENCE_NONE) continue;
                var p = jt.Position;
                Vector3 colorMm = new Vector3(
                    R[0] * p.X + R[1] * p.Y + R[2] * p.Z + T[0],
                    R[3] * p.X + R[4] * p.Y + R[5] * p.Z + T[1],
                    R[6] * p.X + R[7] * p.Y + R[8] * p.Z + T[2]);
                Vector3 local = colorMm * 0.001f;
                pos[ji] = local;
                Sphere(parent, $"k4_{(k4abt_joint_id_t)ji}", local, 0.032f, K4Col);
                cnt++;
            }
            foreach (var (a, b) in BodyTrackingShared.Bones)
                if (pos.TryGetValue((int)a, out var pa) && pos.TryGetValue((int)b, out var pb))
                    Line(parent, $"k4bone_{a}_{b}", pa, pb, K4Col, 0.008f);
            return cnt;
        }

        int SpawnRtmpose(Transform parent, EvalSkeleton skel)
        {
            var pos = new Dictionary<int, Vector3>();
            int cnt = 0;
            for (int j = 0; j < EvalSkeleton.JointCount; j++)
            {
                if (!skel.Joints[j].Valid) continue;
                Vector3 local = skel.Joints[j].PositionMm * 0.001f;
                pos[j] = local;
                Sphere(parent, $"rt_{(EvalJointId)j}", local, 0.04f, RtCol);
                cnt++;
            }
            foreach (var (a, b) in EvalBones)
                if (pos.TryGetValue((int)a, out var pa) && pos.TryGetValue((int)b, out var pb))
                    Line(parent, $"rtbone_{a}_{b}", pa, pb, RtCol, 0.01f);
            return cnt;
        }

        static void Sphere(Transform parent, string name, Vector3 local, float scale, Color col)
        {
            var s = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            s.name = name; s.transform.SetParent(parent, false);
            s.transform.localPosition = local; s.transform.localScale = Vector3.one * scale;
            var r = s.GetComponent<Renderer>(); if (r != null) r.material.color = col;
        }

        static void Line(Transform parent, string name, Vector3 a, Vector3 b, Color col, float w)
        {
            var go = new GameObject(name);
            go.transform.SetParent(parent, false);
            var lr = go.AddComponent<LineRenderer>();
            lr.useWorldSpace = false;
            lr.positionCount = 2;
            lr.SetPosition(0, a); lr.SetPosition(1, b);
            lr.startWidth = lr.endWidth = w;
            lr.material = new Material(Shader.Find("Sprites/Default"));
            lr.startColor = lr.endColor = col;
        }

        void ClearViz()
        {
            var old = GameObject.Find("BTInspect");
            if (old != null) DestroyImmediate(old);
            foreach (var d in s_keep) { try { d.Dispose(); } catch { } }
            s_keep.Clear();
        }

        void FrameSceneView(GameObject parent)
        {
            Vector3 c = Vector3.zero; int n = 0;
            foreach (var t in parent.GetComponentsInChildren<Transform>())
                if (t.name.StartsWith("rt_") || t.name.StartsWith("k4_")) { c += t.position; n++; }
            if (n == 0) return;
            c /= n;
            var sv = SceneView.lastActiveSceneView;
            if (sv != null) { sv.pivot = c; sv.size = 1.5f; sv.Repaint(); }
        }

        /// <summary>Console-callable a/b diagnosis: adapter without vs with volume selection.</summary>
        public static string DebugAdapter(string root, string host, string serial, int frame)
        {
            string dev = Path.Combine(PointCloudRecording.HostDir(root, host), $"FemtoBolt_{serial}");
            byte[] color, depth; ulong ts;
            using (var cs = new PointCloudRecording.RcsvFrameStream(Path.Combine(dev, PointCloudRecording.ColorSensorName)))
            { var f = cs[frame]; color = Copy(f); ts = f.TimestampNs; }
            var (cw, ch) = PointCloudRecording.ReadRcsvHeaderDimensions(Path.Combine(dev, PointCloudRecording.ColorSensorName));
            using (var ds = new PointCloudRecording.RcsvFrameStream(Path.Combine(dev, PointCloudRecording.DepthSensorName)))
            { depth = Copy(ds[Nearest(ds, ts)]); }
            var (dw, dh) = PointCloudRecording.ReadRcsvHeaderDimensions(Path.Combine(dev, PointCloudRecording.DepthSensorName));

            var calib = PointCloudRecording.ReadExtrinsicsYaml(root);
            var dc = FindCalib(calib, serial);
            if (dc == null) return "no calib";
            var cam = new ObCameraParam
            {
                DepthIntrinsic = dc.DepthIntrinsic, RgbIntrinsic = dc.ColorIntrinsic,
                DepthDistortion = dc.DepthDistortion, RgbDistortion = dc.ColorDistortion,
                Transform = dc.DepthToColor, IsMirrored = false,
            };
            string modelsDir = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "eval", "models"));
            var backend = s_backend ?? new OrtRtmposeBackend(
                FirstOnnx(Path.Combine(modelsDir, "yolox-m")), FirstOnnx(Path.Combine(modelsDir, "rtmpose-m")));
            s_backend = backend;
            var raw = new RawFrameData(depth, depth.Length, dw, dh, color, color.Length, cw, ch, Array.Empty<byte>(), 0, 0, 0, ts / 1000UL);

            var sb = new System.Text.StringBuilder();
            var ctx = new EvalCameraContext(serial, dw, dh, cw, ch, cam);

            var a1 = new RtmPoseAdapter(backend) { confThreshold = 0.3f, redetectEveryN = 1 };
            a1.Configure(in ctx);
            int n1 = -1; a1.OnSkeletons += f => { var p = f.Primary(); n1 = p != null ? p.ValidCount() : 0; };
            a1.SubmitFrame(serial, in raw, ts);
            sb.AppendLine($"(a) no volume: validJoints={n1}");

            var a2 = new RtmPoseAdapter(backend) { confThreshold = 0.3f, redetectEveryN = 1 };
            a2.Configure(in ctx);
            if (dc.GlobalTrColorCamera.HasValue) a2.SetWorldTransform(serial, dc.GlobalTrColorCamera.Value);
            a2.SetCaptureVolume(new Vector3(0, 200, 3000), new Vector3(1100, 1500, 1100));
            int n2 = -1; a2.OnSkeletons += f => { var p = f.Primary(); n2 = p != null ? p.ValidCount() : 0; };
            a2.SubmitFrame(serial, in raw, ts);
            sb.AppendLine($"(b) with volume: validJoints={n2}");

            // manual replication of the volume test with intermediate values
            var lift = new DepthLift();
            bool aligned = lift.BuildAligned(depth, dw, dh, cw, ch, cam);
            var boxes = new DetBox[8];
            int nDet = backend.Detect(color, cw, ch, boxes);
            sb.AppendLine($"(c) manual: aligned={aligned} nDet={nDet}");
            var eWt = dc.GlobalTrColorCamera ?? default;
            for (int i = 0; i < nDet; i++)
            {
                float bx = 0.5f * (boxes[i].X1 + boxes[i].X2), by = 0.5f * (boxes[i].Y1 + boxes[i].Y2);
                float d = lift.SampleMm(Mathf.RoundToInt(bx), Mathf.RoundToInt(by), 5);
                string extra = "";
                if (d > 0)
                {
                    Vector3 camMm = DepthLift.Backproject(bx, by, d, cw, ch, cam);
                    var R = eWt.Rot; var T = eWt.Trans;
                    Vector3 w = (R != null && T != null)
                        ? new Vector3(R[0]*camMm.x+R[1]*camMm.y+R[2]*camMm.z+T[0], R[3]*camMm.x+R[4]*camMm.y+R[5]*camMm.z+T[1], R[6]*camMm.x+R[7]*camMm.y+R[8]*camMm.z+T[2])
                        : camMm;
                    extra = $" world=({w.x:F0},{w.y:F0},{w.z:F0})";
                }
                sb.AppendLine($"  det{i} score={boxes[i].Score:F2} center=({bx:F0},{by:F0}) d={d:F0}mm{extra}");
            }
            return sb.ToString();
        }

        static PointCloudRecording.DeviceCalibration FindCalib(IReadOnlyList<PointCloudRecording.DeviceCalibration> calib, string serial)
        {
            if (calib == null) return null;
            foreach (var c in calib) if (string.Equals(c.Serial, serial, StringComparison.OrdinalIgnoreCase)) return c;
            return null;
        }

        static string FirstOnnx(string dir)
        {
            if (!Directory.Exists(dir)) return null;
            var f = Directory.GetFiles(dir, "end2end.onnx", SearchOption.AllDirectories);
            return f.Length > 0 ? f[0] : null;
        }

        static byte[] Copy(PointCloudRecording.Frame f) { var b = new byte[f.ByteCount]; Buffer.BlockCopy(f.Bytes, 0, b, 0, f.ByteCount); return b; }

        static int Nearest(PointCloudRecording.RcsvFrameStream s, ulong ts)
        {
            int best = -1; ulong bd = ulong.MaxValue;
            for (int i = 0; i < s.Count; i++)
            {
                ulong t = s.TimestampNsAt(i), d = t > ts ? t - ts : ts - t;
                if (d < bd) { bd = d; best = i; }
            }
            return best;
        }
    }
}
