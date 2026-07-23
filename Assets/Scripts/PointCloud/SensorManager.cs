// Multi-device coordinator. Enumerates Femto Bolts on Start and spawns
// one PointCloudRenderer GameObject per device. Owns the OrbbecRuntime
// lifecycle for the scene.

using System;
using System.Collections.Generic;
using System.IO;
using Calibration;
using Orbbec;
using UnityEngine;

namespace PointCloud
{
    public class SensorManager : MonoBehaviour, global::Shared.IStagedShutdown
    {
        [Header("Mode")]
        [Tooltip("When ON, Start() does NOT enumerate Femto Bolt devices or spawn live " +
                 "PointCloudRenderers — Play simply drives SensorRecorder playback of " +
                 "whatever recording SensorRecorder.folderPath points at. Use to test " +
                 "scenes without the camera rig plugged in. When OFF the original live " +
                 "capture path runs.")]
        public bool playbackOnly = false;

        [Tooltip("Material assigned to spawned renderers.")]
        public Material defaultPointMaterial;

        [Header("Per-device defaults")]
        public uint depthWidth = 640;
        public uint depthHeight = 576;
        public uint depthFps = 30;
        public uint colorWidth = 1280;
        public uint colorHeight = 720;
        public uint colorFps = 30;
        [Tooltip("Color stream pixel format applied to every spawned renderer. RGB = uncompressed " +
                 "(saturates USB3 with 4 cameras at 1080p). MJPG = compressed (~10x lighter, " +
                 "matches OrbbecViewer's default).")]
        public ObFormat colorFormat = ObFormat.RGB;
        [Tooltip("Stream that depth gets aligned TO via the Align filter (D2C target).")]
        public ObStreamType alignTargetStream = ObStreamType.Color;

        [Tooltip("Optional shared bounding box applied to every spawned renderer.")]
        public BoundingVolume defaultBoundingBox;

        [Tooltip("Optional shared decimater applied to every spawned renderer.")]
        public PointCloudDecimater defaultDecimater;

        [Tooltip("Optional shared cumulative snapshotter applied to every spawned renderer.")]
        public PointCloudCumulative defaultCumulative;

        [Tooltip("Optional shared floor mask applied to every spawned renderer — hides the " +
                 "bare floor except around the visitor's feet.")]
        public PointCloudFloorMask defaultFloorMask;

        [Header("Multi-device sync")]
        [Tooltip("Sync mode applied to every spawned renderer. SyncHubPro: all devices set to " +
                 "Secondary (the hub generates trigger pulses on VSYNC, every camera receives " +
                 "via VSYNC_IN). DaisyChain: index 0 = Primary (generates the pulse), rest = " +
                 "Secondary (forwards to the next camera). Note: Femto Bolt firmware exposes " +
                 "Secondary, not SecondarySynced — observed bitmap 0x000F = FreeRun|Standalone|" +
                 "Primary|Secondary.")]
        public SyncTopology syncTopology = SyncTopology.SyncHubPro;
        [Tooltip("When true, every spawned renderer rewrites the device's sync config on pipeline " +
                 "start using syncTopology above. Set false to leave the device's existing sync " +
                 "mode alone (e.g. when configured externally via OrbbecViewer / Sync Hub Pro " +
                 "tooling). syncTopology is ignored in that case.")]
        public bool applySyncConfig = true;
        [Tooltip("Stagger trigger2ImageDelayUs across devices to reduce iToF NIR pulse interference. " +
                 "Each device gets index * step microseconds. 0 disables the stagger.")]
        [Min(0)]
        public int trigger2ImageDelayStepUs = 160;
        [Tooltip("Call ob_device_timer_sync_with_host on each device at startup so their frame " +
                 "timestamps share a host-time reference.")]
        public bool enableTimerSyncWithHost = true;
        [Tooltip("Call ob_device_enable_global_timestamp(true) so frame.GlobalTimestampUs becomes " +
                 "host-clock-aligned. The recorder uses this as the recording timestamp.")]
        public bool enableGlobalTimestamp = true;

        public enum SyncTopology
        {
            /// <summary>Sync Hub Pro: hub fans trigger out, cam0 = Primary (seeds the chain), rest = Secondary.</summary>
            SyncHubPro = 0,
            /// <summary>Camera-to-camera daisy chain: cam0 = Primary, rest = Secondary.</summary>
            DaisyChain = 1,
            /// <summary>No hardware sync; each device runs Standalone (use only for single-camera or testing).</summary>
            Standalone = 2,
        }

        [Header("Depth work mode")]
        [Tooltip("Depth work mode name applied to every spawned renderer before its pipeline starts. " +
                 "Leave empty to keep each device's current mode. Run the renderer's 'Log Depth Work " +
                 "Modes' context menu to list the available names for your firmware.")]
        public string depthWorkMode = string.Empty;

        [Header("Display")]
        [Tooltip("Where to register spawned Live renderers for visibility toggling. " +
                 "Auto-found via FindFirstObjectByType if left null.")]
        public PointCloudView view;

        [Header("Camera markers")]
        [Tooltip("Attach a wireframe square-pyramid (frustum) marker to each spawned renderer. " +
                 "The apex sits at the camera origin and the base opens along the view direction, " +
                 "so each camera's position and aim are visible inside the merged point cloud. " +
                 "Rides the renderer transform, so it tracks the applied extrinsics.")]
        public bool showCameraMarkers = false;
        [Tooltip("Frustum length (apex -> base) in metres.")]
        public float markerLength = 0.2f;
        [Tooltip("Frustum base half-width / half-height in metres.")]
        public float markerBaseHalfWidth = 0.12f;
        public float markerBaseHalfHeight = 0.09f;

        [Header("Extrinsics (issue #9)")]
        [Tooltip("Apply per-device global_tr_colorCamera read from <extrinsicsRoot>/calibration/" +
                 "extrinsics.yaml to each spawned renderer GO. Off → renderers stay at the manager's " +
                 "local origin (legacy behavior).")]
        public bool applyExtrinsics = false;
        [Tooltip("Root directory for extrinsics.yaml lookup. Same root convention as " +
                 "SensorRecorder.folderPath. Empty → Application.persistentDataPath/Recordings/recording.")]
        public string extrinsicsRoot = string.Empty;

        [Header("World rebase (experience flow)")]
        [Tooltip("Re-base the calibration world when applying extrinsics: origin = floor " +
                 "projection of the 4-camera centroid, +X = camera1→2 (yaw + XZ only, floor " +
                 "height stays calibrated). Needs rigSerialOrder. Off → Dev mode unchanged.")]
        public bool applyWorldRebase = false;
        [Tooltip("FALLBACK ONLY — calibration/cameras.yaml (assign mode, machine-local) wins " +
                 "when it lists 4 serials, so each rig set resolves its own cameras. " +
                 "The rig's 4 camera serials in order 1..4 (defines the rebased axes: " +
                 "+X = camera1→2, +Z ≈ camera2→3). Rebase is skipped with a warning when " +
                 "these don't resolve against extrinsics.yaml.")]
        public string[] rigSerialOrder = new string[0];

        [Tooltip("Physical floor height in the CALIBRATION frame (m) — the rebase shifts " +
                 "Y so this becomes the new y=0 (the cameras sit ~1 m above the floor). " +
                 "0 = keep calibrated heights. OVERWRITTEN AT STARTUP by " +
                 "calibration/floor.yaml when that exists (written by the floor-tune " +
                 "mode), since the floor is a per-room measurement rather than a scene " +
                 "setting — delete the file to go back to this value.")]
        public float rebaseFloorY = 0f;

        [Tooltip("Load rebaseFloorY from calibration/floor.yaml at startup. Off = the " +
                 "Inspector value always wins.")]
        public bool loadFloorYFromCalibration = true;

        [Tooltip("Load the sensing box (BoundingVolume centre + size) from " +
                 "calibration/sensing_area.yaml at startup. The box is fitted to the " +
                 "camera rig, and the two exhibition sets stand their cameras " +
                 "differently, so it is machine-local like cameras.yaml — the scene " +
                 "value is only the fallback for a machine that has never fitted one. " +
                 "Off = the scene value always wins.")]
        public bool loadSensingAreaFromCalibration = true;

        /// <summary>
        /// World-space floor-levelling correction (tilt + height) produced by the
        /// interactive 3-point floor pick (CalibrationRuntimeUI floor tune). Applied
        /// AFTER the camera rebase so the picked floor becomes horizontal at y=0.
        /// Not serialized: it is a per-room measurement loaded from / saved to
        /// calibration/floor.yaml, exactly like <see cref="rebaseFloorY"/>.
        /// </summary>
        [System.NonSerialized]
        public Pose rebaseFloorLeveling = Pose.identity;

        [Header("Frame rate")]
        [Tooltip("Cap the application frame rate via Application.targetFrameRate. " +
                 "vSync is disabled (QualitySettings.vSyncCount = 0) so the cap takes " +
                 "effect — vSync would otherwise override targetFrameRate. Set to 0 or " +
                 "below to leave the frame rate uncapped.")]
        public int targetFrameRate = 30;

        [Header("Diagnostics")]
        public bool verboseLogging = true;

        public IReadOnlyList<PointCloudRenderer> Renderers => _renderers;

        private readonly List<PointCloudRenderer> _renderers = new List<PointCloudRenderer>();

        private void Awake()
        {
            if (view == null) view = FindFirstObjectByType<PointCloudView>();
            ApplyTargetFrameRate();
        }

        // vSync overrides Application.targetFrameRate, so it must be off for the cap
        // to take effect. Only touch these globals when a positive cap is requested.
        private void ApplyTargetFrameRate()
        {
            if (targetFrameRate <= 0) return;
            QualitySettings.vSyncCount = 0;
            Application.targetFrameRate = targetFrameRate;
            if (verboseLogging)
                Debug.Log($"[{nameof(SensorManager)}] targetFrameRate capped at {targetFrameRate} fps (vSync off).");
        }

        private bool _markerToggleInit;
        private bool _lastShowCameraMarkers;

        // Propagate the manager-level marker toggle to existing markers, but only when
        // it actually changes — so per-marker Inspector toggles aren't clobbered each frame.
        private void Update()
        {
            if (_markerToggleInit && _lastShowCameraMarkers == showCameraMarkers) return;
            _markerToggleInit = true;
            _lastShowCameraMarkers = showCameraMarkers;
            foreach (var r in _renderers)
            {
                if (r == null) continue;
                var m = r.GetComponent<CameraPoseMarker>();
                if (m != null) m.showVisualization = showCameraMarkers;
            }
        }

        private void Start()
        {
            global::Shared.StartupProfiler.Mark("SensorManager.Start");
            LoadFloorYFromCalibration();
            ApplySensingAreaFromCalibration();
            global::Shared.StartupProfiler.Mark("SensorManager: floor + sensing area loaded");
            if (playbackOnly)
            {
                if (verboseLogging)
                    Debug.Log($"[{nameof(SensorManager)}] playbackOnly=true; skipping device enumeration. " +
                              "SensorRecorder will drive playback from its folderPath.");
                return;
            }
            // Staged, NOT StartLive() straight through: opening the rig costs ~1.6s per
            // camera and every one of those seconds would land inside this first frame's
            // Start phase, so the player would present nothing at all until the whole rig
            // was up (measured: first frame at 8.9s). Yielding first lets the boot splash
            // reach the screen, and yielding between cameras keeps its counter moving.
            _startingLive = true;
            global::Shared.BootOverlay.Claim();
            StartCoroutine(StartLiveStaged());
        }

        // Boot-time StartLive: same work, spread over frames so the screen is alive
        // while it happens. StartLive() itself stays synchronous — SwitchToLive()
        // calls it mid-session, where there is no splash and no reason to wait.
        // True from the moment the staged coroutine is launched until it has finished (or
        // failed). The `_renderers.Count == 0` test alone cannot guard this: the staged path
        // yields for whole frames BEFORE it spawns anything, so during that window the list
        // is still empty and a second staged run — or a SwitchToLive()/StartLive() call —
        // sails past the check and enumerates the same USB devices again. Two opens of one
        // Femto Bolt is not a duplicate-and-recover situation; the second fails and the rig
        // comes up short a camera.
        private bool _startingLive;

        // Set the moment staged shutdown begins. Boot and teardown genuinely overlap: the
        // operator can quit while the rig is still coming up, and the staged start spends
        // seconds spawning cameras one per frame. Without this the teardown would snapshot
        // the renderer list as it stood, then boot would keep adding cameras behind it that
        // nothing ever stops — and _stoppedStaged has already switched off the
        // OnApplicationQuit fallback that used to catch them.
        private bool _shutdownRequested;

        [Tooltip("Staged startup: hard cap on waiting for a batch of device opens. Past " +
                 "this the boot carries on with whatever came up, so a single dead camera " +
                 "cannot hold the exhibition on the splash screen forever. Normal cost is " +
                 "~1.6s for the whole batch (they open concurrently).")]
        public float openTimeoutSeconds = 30f;

        // Polls a batch of device opens without blocking the main thread, reporting the
        // running count into the splash. Faults are logged per camera and do not stop the
        // rest of the rig coming up: three working cameras beat none.
        private System.Collections.IEnumerator WaitForOpens(
            System.Collections.Generic.List<System.Threading.Tasks.Task> opens,
            string label, System.Func<int, int> toOverallCount, int total)
        {
            if (opens.Count == 0) yield break;
            float deadline = Time.realtimeSinceStartup + openTimeoutSeconds;
            while (true)
            {
                int done = 0;
                for (int i = 0; i < opens.Count; i++) if (opens[i].IsCompleted) done++;
                global::Shared.BootOverlay.SetStatus(
                    $"カメラを起動しています…  {toOverallCount(done)} / {total}");
                if (done >= opens.Count) break;
                if (Time.realtimeSinceStartup > deadline)
                {
                    Debug.LogError($"[{nameof(SensorManager)}] {opens.Count - done} {label} camera " +
                                   $"open(s) did not finish within {openTimeoutSeconds:0}s — " +
                                   "continuing without them.", this);
                    break;
                }
                yield return null;
            }
            foreach (var t in opens)
                if (t.IsFaulted && t.Exception != null) Debug.LogException(t.Exception, this);
            global::Shared.StartupProfiler.Mark($"SensorManager: {label} open(s) joined");
        }

        private System.Collections.IEnumerator StartLiveStaged()
        {
            // try/finally around the whole claimed lifetime: an exception anywhere below
            // (context creation, enumeration, a camera that refuses to open) aborts the
            // coroutine on the spot, and without this the splash would stay up over every
            // display until its own safety timeout — an unattended installation showing a
            // boot screen for two minutes.
            try
            {
                global::Shared.BootOverlay.SetStatus("カメラを検出しています…");
                // End of frame, not just "next frame": the splash has to be PRESENTED
                // before we block, otherwise it is only ever a queued draw call.
                yield return new WaitForEndOfFrame();

                // Same re-check as in the spawn loop, for the same reason: a quit landing
                // during the yield above must not be followed by context creation and a
                // synchronous QueryDevices(). Both block the main thread, so a slow or
                // wedged enumeration would stall the very loop that drives the startup-join
                // deadline and the quit watchdog. Nothing has been opened yet at this
                // point, so there is nothing for the teardown to clean up.
                if (_shutdownRequested)
                {
                    Debug.Log($"[{nameof(SensorManager)}] staged startup cancelled before " +
                              "device enumeration — shutdown requested.");
                    yield break;
                }

                _renderers.RemoveAll(r => r == null);
                if (_renderers.Count == 0)
                {
                    var ctx = OrbbecRuntime.Context;
                    global::Shared.StartupProfiler.Mark("SensorManager: Orbbec context ready");
                    var devices = ctx.QueryDevices();
                    global::Shared.StartupProfiler.Mark($"SensorManager: QueryDevices -> {devices.Count}");
                    if (verboseLogging)
                        Debug.Log($"[{nameof(SensorManager)}] Found {devices.Count} device(s).");

                    // Spawn every renderer first with deferOpen, so none of them opens a
                    // device from its own Start(). The opens are then issued from here,
                    // concurrently — serialised they cost ~1.6s each (6.4s for four).
                    for (int i = 0; i < devices.Count; i++)
                    {
                        // Quit arrived mid-boot: stop spawning. Anything opened so far is
                        // already in _renderers and the teardown waiting on us will stop it;
                        // opening MORE cameras now would hand it a rig it has already
                        // enumerated past, leaving those pipelines running into the exit.
                        if (_shutdownRequested)
                        {
                            Debug.Log($"[{nameof(SensorManager)}] staged startup cancelled at " +
                                      $"{i}/{devices.Count} — shutdown requested.");
                            break;
                        }

                        var d = devices[i];
                        if (verboseLogging) Debug.Log($"  [{i}] {d}");
                        var r = SpawnRenderer(d, i, deferOpen: true);
                        _renderers.Add(r);
                        if (view != null && r != null)
                        {
                            var mr = r.GetComponent<MeshRenderer>();
                            if (mr != null) view.Register(mr);
                        }
                    }

                    int n = _renderers.Count;
                    if (n > 0)
                    {
                        global::Shared.BootOverlay.SetStatus($"カメラを起動しています…  0 / {n}");
                        yield return new WaitForEndOfFrame();   // present the count first

                        // Secondaries concurrently, Primary last — the same ordering the
                        // teardown uses, and here it is what the sync topology requires:
                        // ob_pipeline_start happens inside the open, so every Secondary has
                        // to be up and waiting before the Primary (index 0, see
                        // ResolveSyncMode) starts emitting triggers into the hub.
                        var opens = new System.Collections.Generic.List<System.Threading.Tasks.Task>(n);
                        for (int i = 1; i < n; i++)
                            if (_renderers[i] != null) opens.Add(_renderers[i].OpenDeviceAsync());

                        yield return WaitForOpens(opens, "secondary", done => done, n);

                        if (!_shutdownRequested && _renderers[0] != null)
                        {
                            var primaryOpen = _renderers[0].OpenDeviceAsync();
                            yield return WaitForOpens(
                                new System.Collections.Generic.List<System.Threading.Tasks.Task> { primaryOpen },
                                "primary", done => opens.Count + done, n);
                        }

                        // Mesh, transform and capture thread are Unity-side, so they land
                        // back here on the main thread. A camera whose open faulted is left
                        // alone — FinishOpen on a device that never opened would throw.
                        for (int i = 0; i < n; i++)
                        {
                            var r = _renderers[i];
                            if (r == null || !r.IsDeviceOpen) continue;
                            try { r.FinishOpen(); }
                            catch (Exception e) { Debug.LogException(e, r); r.StopCapture(); }
                        }
                    }

                    if (applyExtrinsics) ApplyExtrinsicsToLive();
                    global::Shared.StartupProfiler.Mark("SensorManager.StartLive done (staged)");
                }
            }
            finally
            {
                _startingLive = false;
                global::Shared.BootOverlay.Hide();
            }
        }

        // The floor height belongs to the room, not the scene: the floor-tune mode
        // writes it next to extrinsics.yaml so it survives Play mode and stays
        // machine-local (the two rig sets stand in different rooms).
        private void LoadFloorYFromCalibration()
        {
            if (!loadFloorYFromCalibration) return;
            if (!PointCloudRecording.TryReadFloor(ResolveExtrinsicsRoot(), out float y,
                                                  out var lp, out var lr))
                return;
            rebaseFloorLeveling = new Pose(new Vector3(lp[0], lp[1], lp[2]),
                                           new Quaternion(lr[0], lr[1], lr[2], lr[3]));
            if (Mathf.Approximately(y, rebaseFloorY)) return;
            if (verboseLogging)
                Debug.Log($"[{nameof(SensorManager)}] rebaseFloorY {rebaseFloorY} → {y} from calibration/floor.yaml.",
                          this);
            rebaseFloorY = y;
        }

        // The sensing box belongs to the rig, not the scene, for the same reason the
        // floor does: main.unity is shared between the two exhibition sets but their
        // cameras stand in different places. A box committed to the scene therefore
        // arrives at the other set subtly wrong, and a wrong-but-plausible box is
        // hard to notice. Missing file = keep whatever the scene authored; the
        // operator fits one with the calibration UI (or a Solve) and it is saved.
        private void ApplySensingAreaFromCalibration()
        {
            if (!loadSensingAreaFromCalibration || defaultBoundingBox == null) return;
            if (!PointCloudRecording.TryReadSensingArea(ResolveExtrinsicsRoot(),
                                                        out var c, out var sz))
                return;
            var center = new Vector3(c[0], c[1], c[2]);
            var size = new Vector3(sz[0], sz[1], sz[2]);
            var t = defaultBoundingBox.transform;
            t.position = center;
            t.rotation = Quaternion.identity;   // the fit is always axis-aligned
            t.localScale = size;
            if (verboseLogging)
                Debug.Log($"[{nameof(SensorManager)}] sensing area from calibration/sensing_area.yaml: " +
                          $"center {center:F3}, size {size:F3}.", this);
        }

        /// <summary>
        /// Enumerate connected Femto Bolts and spawn one PointCloudRenderer per
        /// device. Runs from Start() on live scenes, and again from
        /// SensorRecorder.SwitchToLive() after playback destroyed the renderers,
        /// so switching back to live no longer requires reloading the scene.
        /// No-op while live renderers already exist. Ignores playbackOnly — the
        /// flag only decides the startup mode, not explicit switches.
        /// </summary>
        public void StartLive()
        {
            // The staged boot path owns the rig until it finishes. It spends whole frames
            // yielding before the first spawn, so `_renderers.Count` is still 0 then and
            // would let this run enumerate and open the very same devices a second time.
            if (_startingLive)
            {
                if (verboseLogging)
                    Debug.Log($"[{nameof(SensorManager)}] StartLive ignored: staged startup already in progress.");
                return;
            }
            _renderers.RemoveAll(r => r == null);
            if (_renderers.Count > 0) return;

            var ctx = OrbbecRuntime.Context;
            global::Shared.StartupProfiler.Mark("SensorManager: Orbbec context ready");
            var devices = ctx.QueryDevices();
            global::Shared.StartupProfiler.Mark($"SensorManager: QueryDevices -> {devices.Count}");
            if (verboseLogging)
                Debug.Log($"[{nameof(SensorManager)}] Found {devices.Count} device(s).");

            for (int i = 0; i < devices.Count; i++)
            {
                var d = devices[i];
                if (verboseLogging)
                    Debug.Log($"  [{i}] {d}");
                var r = SpawnRenderer(d, i);
                _renderers.Add(r);
                if (view != null && r != null)
                {
                    var mr = r.GetComponent<MeshRenderer>();
                    if (mr != null) view.Register(mr);
                }
            }

            if (applyExtrinsics) ApplyExtrinsicsToLive();
            global::Shared.StartupProfiler.Mark("SensorManager.StartLive done");
        }

        // Quitting fires before the scene's GameObjects are destroyed, which is the
        // only place we still control the ORDER the rig stops in. Skipped when the
        // staged path already stopped the rig before Application.Quit() was called
        // (see StopStaged) — that is the normal route out of the build now, and this
        // stays as the fallback for a quit that bypasses it.
        private void OnApplicationQuit()
        {
            // Not a bare `if (_stoppedStaged) return`: the staged stop records how many
            // renderers it actually handled, and a boot that was still unwinding past the
            // join timeout can have added more behind it. Those are exactly the pipelines
            // nothing else will close, so the fallback has to run for them even though the
            // staged path "already stopped the rig".
            _renderers.RemoveAll(r => r == null);
            if (_stoppedStaged && _renderers.Count <= _stagedStopCount) return;
            if (_stoppedStaged)
                Debug.LogWarning($"[{nameof(SensorManager)}] {_renderers.Count - _stagedStopCount} " +
                                 "renderer(s) appeared after the staged stop — stopping them now.", this);
            StopCaptureSecondariesFirst();
        }

        private bool _stoppedStaged;
        // Renderer count the staged stop covered; see OnApplicationQuit.
        private int _stagedStopCount;

        [Tooltip("Staged shutdown: how long a secondary pipeline stop may take before " +
                 "the splash says it is taking a while. Waiting continues either way — " +
                 "the normal cost is ~1.3s per camera, all four in parallel.")]
        public float softStopTimeoutSeconds = 15f;

        [Tooltip("Staged shutdown: hard cap on waiting for pipeline stops. Past this the " +
                 "exit gives up on disposing the Orbbec context (a native stop is still " +
                 "running and disposing under it would be a use-after-free) and quits " +
                 "anyway — an installation that cannot be quit is worse than a leak.")]
        public float hardStopTimeoutSeconds = 60f;

        /// <summary>
        /// <see cref="StopCaptureSecondariesFirst"/> spread over frames: same order,
        /// same concurrency, but the main thread returns to the player loop between
        /// polls so the shutdown splash can show how many cameras are down. The
        /// blocking version stays for every non-quit caller (DestroyAllRenderers,
        /// mode switches) where there is no screen to keep alive.
        /// </summary>
        /// <summary>The quit watchdog gave up on this teardown. Whatever state the
        /// pipeline stops are in, the context must not be disposed behind them.</summary>
        public void AbandonForForcedExit()
        {
            OrbbecRuntime.SuppressShutdown("forced exit while the staged teardown was still running");
        }

        public System.Collections.IEnumerator StopStaged(System.Action<int, int> progress)
        {
            // Cancel and JOIN a staged startup before looking at _renderers. The list is
            // only a valid picture of the rig once boot has stopped adding to it; snapshot
            // it while the start coroutine is still running and every camera opened after
            // this point is one nothing will ever stop.
            _shutdownRequested = true;
            if (_startingLive)
            {
                global::Shared.ShutdownProfiler.Mark("SensorManager: waiting for staged startup to unwind");
                // Bounded: the start coroutine yields between cameras so it unwinds within
                // a frame or two of the flag, but a camera open blocks the main thread for
                // ~1.6s and a wedged one would never return. Proceeding with a partial rig
                // beats never quitting — the fallback below still catches stragglers.
                float deadline = Time.realtimeSinceStartup + hardStopTimeoutSeconds;
                while (_startingLive && Time.realtimeSinceStartup < deadline) yield return null;
                if (_startingLive)
                    Debug.LogError($"[{nameof(SensorManager)}] staged startup did not unwind within " +
                                   $"{hardStopTimeoutSeconds:0}s — stopping the rig as it stands.", this);
            }

            _stoppedStaged = true;
            _renderers.RemoveAll(r => r == null);
            int total = _renderers.Count;
            _stagedStopCount = total;
            if (total == 0) yield break;

            global::Shared.ShutdownProfiler.Mark(
                $"SensorManager: staged stop of {total} renderer(s), Secondaries first");

            // Secondaries concurrently, Primary last — the ordering rationale is in
            // StopCaptureSecondariesFirst and applies unchanged here.
            var tasks = new System.Collections.Generic.List<System.Threading.Tasks.Task>(total);
            for (int i = total - 1; i >= 1; i--)
            {
                var r = _renderers[i];
                if (r != null) tasks.Add(r.StopCaptureAsync());
            }
            // Hold the Primary by reference, not by index: this coroutine yields, and a
            // renderer can be destroyed while it waits (teardown, a health-monitor
            // restart), which shrinks the list. Indexing it afterwards threw
            // IndexOutOfRange and abandoned the rest of the shutdown. The Unity null
            // check below also covers the destroyed-but-still-referenced case.
            var primary = _renderers.Count > 0 ? _renderers[0] : null;

            // Poll instead of Task.WaitAll: waiting is exactly the part that used to
            // freeze the picture.
            //
            // Unlike the blocking version, a timeout here must NOT simply carry on to
            // the quit. These tasks are native ob_pipeline_stop calls; returning lets
            // the caller quit, and the exit path disposes the shared Orbbec context —
            // under a stop that is still running that is a use-after-free. So the soft
            // bound only changes what the splash says, the hard bound gives up on
            // disposing the context at all, and neither one abandons the wait quietly.
            float softDeadline = Time.realtimeSinceStartup + softStopTimeoutSeconds;
            float hardDeadline = Time.realtimeSinceStartup + hardStopTimeoutSeconds;
            bool warned = false, abandoned = false;
            int done = 0;
            while (true)
            {
                done = 0;
                for (int i = 0; i < tasks.Count; i++) if (tasks[i].IsCompleted) done++;
                progress?.Invoke(done, total);
                if (done >= tasks.Count) break;

                float now = Time.realtimeSinceStartup;
                if (!warned && now > softDeadline)
                {
                    warned = true;
                    Debug.LogWarning($"[SensorManager] {tasks.Count - done} secondary pipeline stop(s) " +
                                     $"still running after {softStopTimeoutSeconds:0}s — still waiting.", this);
                    global::Shared.BootOverlay.SetStatus("カメラの停止に時間がかかっています…");
                }
                if (now > hardDeadline)
                {
                    abandoned = true;
                    Debug.LogError($"[SensorManager] {tasks.Count - done} secondary pipeline stop(s) never " +
                                   $"returned within {hardStopTimeoutSeconds:0}s. Giving up on a clean exit.", this);
                    break;
                }
                yield return null;
            }
            foreach (var t in tasks)
                if (t.IsFaulted && t.Exception != null) Debug.LogException(t.Exception, this);
            // A stop that never returned owns native state we cannot account for, so the
            // context must outlive the process rather than be disposed out from under it.
            if (abandoned)
                OrbbecRuntime.SuppressShutdown($"{tasks.Count - done} pipeline stop(s) still in flight");
            global::Shared.ShutdownProfiler.Mark($"SensorManager: {tasks.Count} secondary stop(s) joined (staged)");

            // Let the "secondaries done" count reach the screen before the Primary goes.
            yield return new WaitForEndOfFrame();

            // The Primary goes through StopCaptureAsync too, even though nothing else is
            // waiting on it. A synchronous stop here blocks the main thread with no
            // bound, and the quit watchdog lives in Update() — so a wedged primary stop
            // would freeze the very loop that is supposed to rescue the exit, and the
            // app could never be closed. Off-thread it stays pollable.
            var primaryTask = primary != null ? primary.StopCaptureAsync() : null;
            if (primaryTask != null)
            {
                float primaryDeadline = Time.realtimeSinceStartup + hardStopTimeoutSeconds;
                while (!primaryTask.IsCompleted)
                {
                    if (Time.realtimeSinceStartup > primaryDeadline)
                    {
                        Debug.LogError("[SensorManager] the primary pipeline stop never returned " +
                                       $"within {hardStopTimeoutSeconds:0}s. Giving up on a clean exit.", this);
                        OrbbecRuntime.SuppressShutdown("primary pipeline stop still in flight");
                        break;
                    }
                    yield return null;
                }
                if (primaryTask.IsFaulted && primaryTask.Exception != null)
                    Debug.LogException(primaryTask.Exception, this);
            }
            progress?.Invoke(total, total);
            global::Shared.ShutdownProfiler.Mark("SensorManager: primary stopped (staged)");
        }

        /// <summary>
        /// Stop every renderer's pipeline, Secondaries first and the Primary (index 0,
        /// see <see cref="ResolveSyncMode"/>) last.
        ///
        /// Why the order matters: a Secondary's ob_pipeline_stop blocks until its
        /// in-flight frame wait times out, and with the Primary already stopped no
        /// more sync triggers arrive, so every Secondary pays the full timeout. Left
        /// to Unity's own teardown order (spawn order = Primary first) a 4-camera rig
        /// measured 18.2s of ob_pipeline_stop: 1.3s for the Primary, then 5.0 / 5.1 /
        /// 6.8s for the Secondaries. Stopping them while the trigger still runs keeps
        /// each stop on the fast path.
        /// </summary>
        public void StopCaptureSecondariesFirst()
        {
            if (_renderers.Count == 0) return;
            global::Shared.ShutdownProfiler.Mark(
                $"SensorManager: stopping {_renderers.Count} renderer(s), Secondaries first");

            // Secondaries (everything but index 0) stop concurrently: each
            // ob_pipeline_stop still costs ~1.3s of internal frame-thread wait even on
            // the fast path, and those waits have no reason to queue up behind each
            // other. The Primary is stopped last and on this thread — it keeps the
            // sync trigger running for the others until they are done.
            var tasks = new System.Collections.Generic.List<System.Threading.Tasks.Task>(_renderers.Count);
            for (int i = _renderers.Count - 1; i >= 1; i--)
            {
                var r = _renderers[i];
                if (r != null) tasks.Add(r.StopCaptureAsync());
            }
            if (tasks.Count > 0)
            {
                // Bounded: a wedged native stop must not hang the quit forever. On
                // timeout the pipelines are left to process teardown, same contract as
                // the in-flight-timer-sync leak inside PointCloudRenderer.
                if (!System.Threading.Tasks.Task.WaitAll(tasks.ToArray(), 15000))
                    Debug.LogWarning("[SensorManager] secondary pipeline stop did not finish within 15s.", this);
                global::Shared.ShutdownProfiler.Mark($"SensorManager: {tasks.Count} secondary stop(s) joined");
            }

            if (_renderers.Count > 0 && _renderers[0] != null) _renderers[0].StopCapture();
        }

        private void OnDestroy()
        {
            // Renderers Dispose themselves on OnDestroy; nothing to do here for them.
            // Tear down the shared context after children are gone.
            // Unity destroys children before parents in default scene teardown,
            // but to be safe we defer context shutdown to next frame via Application.quitting in OrbbecRuntime.
            global::Shared.ShutdownProfiler.Mark("SensorManager.OnDestroy");
            OrbbecRuntime.RequestShutdown();
        }

        /// <summary>
        /// Destroy every spawned PointCloudRenderer GameObject so its capture
        /// thread / OrbbecSDK pipeline tears down and releases the USB device.
        /// Called by SensorRecorder.Read so playback runs without the live
        /// cameras competing for USB bandwidth / GPU. Re-connect with
        /// <see cref="StartLive"/> (SensorRecorder's Switch Mode button).
        /// </summary>
        public void DestroyAllRenderers()
        {
            // Same Secondaries-first ordering as the quit path, and for the same
            // reason (see StopCaptureSecondariesFirst) — Destroy is deferred to the
            // end of frame, so stop the pipelines synchronously here first.
            StopCaptureSecondariesFirst();
            for (int i = 0; i < _renderers.Count; i++)
            {
                var r = _renderers[i];
                if (r == null) continue;
                // GameObject destruction triggers PointCloudRenderer.OnDestroy
                // which joins the capture thread and disposes the pipeline.
                UnityEngine.Object.Destroy(r.gameObject);
            }
            _renderers.Clear();
        }

        private PointCloudRenderer SpawnRenderer(OrbbecDeviceDescriptor desc, int index,
                                                 bool deferOpen = false)
        {
            var go = new GameObject($"PointCloud[{index}] {desc.Name} ({desc.Serial})");
            go.transform.SetParent(transform, worldPositionStays: false);
            go.AddComponent<MeshFilter>();
            go.AddComponent<MeshRenderer>();
            var pcr = go.AddComponent<PointCloudRenderer>();

            pcr.deferOpen = deferOpen;
            pcr.deviceSerial = desc.Serial;
            pcr.depthWidth = depthWidth;
            pcr.depthHeight = depthHeight;
            pcr.depthFps = depthFps;
            pcr.colorWidth = colorWidth;
            pcr.colorHeight = colorHeight;
            pcr.colorFps = colorFps;
            pcr.colorFormat = colorFormat;
            pcr.alignTargetStream = alignTargetStream;
            pcr.maxPoints = ResolveMaxPointsPerDevice();
            pcr.pointMaterial = defaultPointMaterial;
            pcr.boundingBox = defaultBoundingBox;
            pcr.decimater = defaultDecimater;
            pcr.cumulative = defaultCumulative;
            pcr.floorMask = defaultFloorMask;
            pcr.syncMode = ResolveSyncMode(index);
            pcr.applySyncConfig = applySyncConfig;
            pcr.trigger2ImageDelayUs = trigger2ImageDelayStepUs * index;
            pcr.timerSyncWithHost = enableTimerSyncWithHost;
            pcr.enableGlobalTimestamp = enableGlobalTimestamp;
            pcr.depthWorkMode = depthWorkMode;

            var marker = go.AddComponent<CameraPoseMarker>();
            marker.showVisualization = showCameraMarkers;
            marker.length = markerLength;
            marker.baseHalfWidth = markerBaseHalfWidth;
            marker.baseHalfHeight = markerBaseHalfHeight;
            // Golden-ratio hue spacing so each camera's frustum is a distinct color.
            marker.color = Color.HSVToRGB((index * 0.61803398875f) % 1f, 0.8f, 1f);

            return pcr;
        }

        /// <summary>
        /// Read <c><extrinsicsRoot>/calibration/extrinsics.yaml</c> and apply each
        /// device's <c>global_tr_colorCamera</c> to the matching spawned renderer GO
        /// (sets <c>localPosition</c> + <c>localRotation</c>; leaves <c>localScale</c>
        /// alone so the existing per-renderer Y flip stays). Devices without a
        /// matching yaml entry keep their default transform.
        /// </summary>
        public void ApplyExtrinsicsToLive()
        {
            string root = ResolveExtrinsicsRoot();
            string path = Path.Combine(PointCloudRecording.CalibrationDir(root), "extrinsics.yaml");
            if (!File.Exists(path))
            {
                if (verboseLogging)
                    Debug.Log($"[{nameof(SensorManager)}] applyExtrinsics: no file at {path}, skipping.");
                return;
            }

            IReadOnlyList<PointCloudRecording.DeviceCalibration> calibrations;
            try
            {
                calibrations = PointCloudRecording.ReadExtrinsicsYaml(root);
            }
            catch (Exception e)
            {
                Debug.LogWarning(
                    $"[{nameof(SensorManager)}] applyExtrinsics: parse failed for {path}: {e.Message}",
                    this);
                return;
            }

            // World rebase is a group-level decision: either every camera gets
            // the rebased pose or none does (a failed resolve must not leave a
            // mixed rig). Resolved once here, before any ApplyToTransform.
            Pose rebase = Pose.identity;
            bool useRebase = applyWorldRebase &&
                             TryResolveWorldRebase(calibrations, out rebase);

            // Fold the interactive floor levelling (3-point pick) onto the camera
            // rebase: it is a world-space correction applied AFTER the rebase, so it
            // left-composes just like the rebase left-composes after ToUnityLocal.
            if (useRebase && !Calibration.FloorPlaneMath.IsApproximatelyIdentity(rebaseFloorLeveling))
                rebase = Calibration.FloorPlaneMath.ComposeWorld(rebaseFloorLeveling, rebase);

            int applied = 0;
            foreach (var c in calibrations)
            {
                var renderer = FindRendererBySerial(c.Serial);
                if (renderer == null) continue;
                if (!c.GlobalTrColorCamera.HasValue) continue;
                if (useRebase)
                    ExtrinsicsApply.ApplyToTransform(renderer.transform, c.GlobalTrColorCamera.Value, rebase);
                else
                    ExtrinsicsApply.ApplyToTransform(renderer.transform, c.GlobalTrColorCamera.Value);
                applied++;
            }
            if (verboseLogging)
                Debug.Log(
                    $"[{nameof(SensorManager)}] applyExtrinsics: applied to {applied}/{_renderers.Count} renderer(s) from {path}" +
                    $"{(applyWorldRebase ? (useRebase ? " (world rebase ON)" : " (world rebase REQUESTED but skipped)") : "")}.");
        }

        /// <summary>
        /// Compute the world-rebase Pose for the whole rig (see WorldFrameRebase).
        /// False (with a warning) when the serials don't resolve, the layout is
        /// degenerate, or this manager's transform isn't identity — in which
        /// case the caller applies plain extrinsics to EVERY camera.
        /// </summary>
        private bool TryResolveWorldRebase(
            IReadOnlyList<PointCloudRecording.DeviceCalibration> calibrations, out Pose rebase)
        {
            rebase = Pose.identity;
            // localPosition/localRotation composition equals a world rebase only
            // under an identity parent (the renderers sit directly under us).
            if (!WorldFrameRebase.ParentIsIdentity(transform))
            {
                Debug.LogWarning($"[{nameof(SensorManager)}] world rebase skipped: this GameObject's " +
                                 "transform is not identity, so local-pose composition would not be a " +
                                 "world rebase. Reset the SensorManager transform.", this);
                return false;
            }
            var cams = new List<(string serial, Vector3 posUnity)>(calibrations.Count);
            foreach (var c in calibrations)
            {
                if (!c.GlobalTrColorCamera.HasValue) continue;
                ExtrinsicsApply.ToUnityLocal(c.GlobalTrColorCamera.Value, out var pos, out _);
                cams.Add((c.Serial, pos));
            }
            var rigOrder = PointCloudRecording.ResolveRigSerialOrder(
                ResolveExtrinsicsRoot(), rigSerialOrder, out string rigSource);
            if (WorldFrameRebase.TryComputeFromCalibrations(cams, rigOrder, out rebase, out string reason,
                                                            rebaseFloorY))
                return true;

            // cameras.yaml is an ID map, not necessarily a perimeter walk. When its
            // order fails the rebase gate, fall back to the scene's hand-ordered
            // list rather than losing the world rebase entirely.
            if (rigSerialOrder is { Length: 4 } && !ReferenceEquals(rigOrder, rigSerialOrder)
                && WorldFrameRebase.TryComputeFromCalibrations(cams, rigSerialOrder, out rebase, out _,
                                                              rebaseFloorY))
            {
                Debug.Log($"[{nameof(SensorManager)}] world rebase: {rigSource} order rejected ({reason}); " +
                          "used the scene's rigSerialOrder instead.", this);
                return true;
            }

            Debug.LogWarning($"[{nameof(SensorManager)}] world rebase skipped (order from {rigSource}): {reason}",
                             this);
            return false;
        }

        /// <summary>
        /// Resolves <see cref="extrinsicsRoot"/> to an absolute path: empty string
        /// defaults to <c>&lt;persistentDataPath&gt;/Recordings/recording</c>;
        /// non-empty relative paths are taken under <c>persistentDataPath</c>.
        /// Exposed so SensorRecorder.Save/Read can inherit the same calibration
        /// the live renderers used — recordings inherit the rig's current setup
        /// rather than starting from identity in a fresh folder.
        /// </summary>
        public string ResolveExtrinsicsRoot() =>
            PointCloudRecording.ResolveRecordingRoot(extrinsicsRoot);

        /// <summary>Show/hide every live renderer's mesh. Frame capture, BT feed
        /// and occupancy keep running — this is purely visual (the experience
        /// hides the raw clouds while the TSDF mesh is the star).</summary>
        public void SetLiveVisualsVisible(bool visible)
        {
            foreach (var r in _renderers)
            {
                if (r == null) continue;
                if (r.TryGetComponent(out MeshRenderer mr)) mr.enabled = visible;
            }
        }

        /// <summary>Include/exclude every live renderer from the sculpture
        /// sources (TSDF integration + curve seeding). Independent of visibility:
        /// attract = hidden AND suppressed, experience = hidden but ACTIVE.</summary>
        public void SetLiveSuppressedAsSource(bool suppressed)
        {
            foreach (var r in _renderers)
                if (r != null) r.suppressAsSource = suppressed;
        }

        private PointCloudRenderer FindRendererBySerial(string serial)
        {
            for (int i = 0; i < _renderers.Count; i++)
            {
                var r = _renderers[i];
                if (r != null && r.deviceSerial == serial) return r;
            }
            return null;
        }

        /// <summary>
        /// Derive the per-device point buffer size from the active align target
        /// stream — D2C aligns depth to color so the aligned cloud has
        /// colorWidth*colorHeight points; C2D would produce depthWidth*depthHeight.
        /// </summary>
        private int ResolveMaxPointsPerDevice()
        {
            if (alignTargetStream == ObStreamType.Depth)
                return checked((int)(depthWidth * depthHeight));
            return checked((int)(colorWidth * colorHeight));
        }

        private ObMultiDeviceSyncMode ResolveSyncMode(int index)
        {
            switch (syncTopology)
            {
                case SyncTopology.SyncHubPro:
                case SyncTopology.DaisyChain:
                    // Femto Bolt sync requires exactly one Primary even when a Sync Hub Pro
                    // is fanning the trigger pulses. cam0 is Primary so its VSYNC OUT seeds
                    // the chain / hub; the remaining cameras are Secondary receivers.
                    return index == 0 ? ObMultiDeviceSyncMode.Primary : ObMultiDeviceSyncMode.Secondary;
                default:
                    return ObMultiDeviceSyncMode.Standalone;
            }
        }
    }
}
