// Command-line argument parsing for k4abt_worker. All args are required
// except --bt-sdk-bin / --k4a-wrapper-bin which fall back to the canonical
// install paths in WorkerBootstrap.

using System;
using BodyTracking.Shared;

namespace K4abtWorker
{
    internal sealed class WorkerArgs
    {
        public Guid SessionGuid;
        public string Serial;
        public int ParentPid;
        public int DepthW;
        public int DepthH;
        public int IrW;
        public int IrH;
        public string K4aWrapperBin;
        public string BtSdkBin;

        public static bool TryParse(string[] args, out WorkerArgs parsed, out string error)
        {
            parsed = new WorkerArgs();
            error = null;

            for (int i = 0; i < args.Length; i++)
            {
                string a = args[i];
                if (!TrySplit(a, out string key, out string val))
                {
                    error = $"unrecognised argument: {a}";
                    return false;
                }

                switch (key)
                {
                    case "--session":
                        if (!Guid.TryParse(val, out parsed.SessionGuid))
                        {
                            error = $"--session: '{val}' is not a valid GUID";
                            return false;
                        }
                        break;
                    case "--serial":
                        parsed.Serial = val;
                        break;
                    case "--parent-pid":
                        if (!int.TryParse(val, out parsed.ParentPid) || parsed.ParentPid <= 0)
                        {
                            error = $"--parent-pid: '{val}' must be a positive integer";
                            return false;
                        }
                        break;
                    case "--depth-w":
                        if (!int.TryParse(val, out parsed.DepthW) || parsed.DepthW <= 0)
                        {
                            error = $"--depth-w: '{val}' must be > 0";
                            return false;
                        }
                        break;
                    case "--depth-h":
                        if (!int.TryParse(val, out parsed.DepthH) || parsed.DepthH <= 0)
                        {
                            error = $"--depth-h: '{val}' must be > 0";
                            return false;
                        }
                        break;
                    case "--ir-w":
                        if (!int.TryParse(val, out parsed.IrW) || parsed.IrW <= 0)
                        {
                            error = $"--ir-w: '{val}' must be > 0";
                            return false;
                        }
                        break;
                    case "--ir-h":
                        if (!int.TryParse(val, out parsed.IrH) || parsed.IrH <= 0)
                        {
                            error = $"--ir-h: '{val}' must be > 0";
                            return false;
                        }
                        break;
                    case "--k4a-wrapper-bin":
                        parsed.K4aWrapperBin = val;
                        break;
                    case "--bt-sdk-bin":
                        parsed.BtSdkBin = val;
                        break;
                    default:
                        error = $"unknown option: {key}";
                        return false;
                }
            }

            if (parsed.SessionGuid == Guid.Empty) { error = "--session is required"; return false; }
            if (string.IsNullOrEmpty(parsed.Serial)) { error = "--serial is required"; return false; }
            if (parsed.ParentPid <= 0) { error = "--parent-pid is required"; return false; }
            if (parsed.DepthW <= 0 || parsed.DepthH <= 0) { error = "--depth-w / --depth-h are required"; return false; }
            if (parsed.IrW <= 0 || parsed.IrH <= 0) { error = "--ir-w / --ir-h are required"; return false; }

            return true;
        }

        // Accepts both "--key=value" and "--key value" if val came as a separate arg, but
        // we keep the surface simple: only "--key=value".
        private static bool TrySplit(string raw, out string key, out string val)
        {
            int eq = raw.IndexOf('=');
            if (!raw.StartsWith("--") || eq <= 2)
            {
                key = null;
                val = null;
                return false;
            }
            key = raw.Substring(0, eq);
            val = raw.Substring(eq + 1);
            return true;
        }

        public static string Usage =>
            "Usage: k4abt_worker --session=<GUID> --serial=<S> --parent-pid=<PID> " +
            "--depth-w=<W> --depth-h=<H> --ir-w=<W> --ir-h=<H> " +
            "[--k4a-wrapper-bin=<path>] [--bt-sdk-bin=<path>]";
    }
}
