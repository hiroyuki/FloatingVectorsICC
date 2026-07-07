// Shared helpers for the TSDF compute pipeline (Phase 2 refactor):
//   TryLoad        — lazy Resources.Load<ComputeShader> with the unified
//                    "[Owner] Compute shader ... not found" error; callers keep
//                    their own failure action (disable / return false / fall through).
//   DispatchLinear — the linearised 64-thread 2D dispatch used by every
//                    volume-wide kernel: groups laid out as a (gx, gy) grid so
//                    the 65535 threadgroup-per-axis D3D limit is never exceeded
//                    (a 1x1m@1cm volume already needs >100k groups); kernels
//                    linearise via _DispatchWidth = gx * 64.

using UnityEngine;

namespace TSDF
{
    internal static class TSDFComputeUtil
    {
        /// <summary>Load <paramref name="resourceName"/> into <paramref name="cached"/>
        /// once (no-op when already loaded). Logs the unified error and returns false
        /// when the compute shader is missing from Resources.</summary>
        public static bool TryLoad(ref ComputeShader cached, string resourceName,
                                   string ownerTag, Object ctx)
        {
            if (cached != null) return true;
            cached = Resources.Load<ComputeShader>(resourceName);
            if (cached == null)
            {
                Debug.LogError($"[{ownerTag}] Compute shader \"Resources/{resourceName}.compute\" not found.", ctx);
                return false;
            }
            return true;
        }

        /// <summary>Dispatch <paramref name="kernel"/> over <paramref name="totalThreads"/>
        /// linear threads in 64-thread groups laid out as a 2D (gx, gy) grid, setting the
        /// _DispatchWidth uniform (= gx * 64) the kernels use to linearise the 2D group id.</summary>
        public static void DispatchLinear(ComputeShader cs, int kernel, int totalThreads)
        {
            int groups = Mathf.CeilToInt(totalThreads / 64f);
            int gx = Mathf.Max(1, Mathf.Min(groups, 65535));
            int gy = Mathf.Max(1, Mathf.CeilToInt(groups / (float)gx));
            cs.SetInt("_DispatchWidth", gx * 64);
            cs.Dispatch(kernel, gx, gy, 1);
        }
    }
}
