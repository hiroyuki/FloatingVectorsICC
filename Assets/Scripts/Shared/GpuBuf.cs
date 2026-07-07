// ComputeBuffer lifecycle helpers shared by PointCloud / BodyTracking / TSDF.
// Replaces the hand-rolled "if (buf == null || buf.count != n) { buf?.Release();
// buf = new ComputeBuffer(...); }" and "buf?.Release(); buf = null;" idioms that
// were repeated at ~50 allocation / ~53 release sites. Centralizing them makes
// the "reallocate without Release" VRAM-leak class structurally impossible at
// converted call sites.
//
// Ensure deliberately checks ONLY null + count (not stride/type), matching the
// pre-existing idiom exactly: converted call sites keep byte-identical realloc
// behavior. Callers that change stride at a fixed count must Release first.

using UnityEngine;

namespace Shared
{
    public static class GpuBuf
    {
        /// <summary>(Re)allocate <paramref name="buf"/> when it is null or its
        /// element count differs. No-op otherwise.</summary>
        public static void Ensure(ref ComputeBuffer buf, int count, int stride,
                                  ComputeBufferType type = ComputeBufferType.Default)
        {
            if (buf != null && buf.count == count) return;
            buf?.Release();
            buf = new ComputeBuffer(count, stride, type);
        }

        /// <summary>Release + null the field so a later Ensure can't double-free
        /// or leak.</summary>
        public static void Release(ref ComputeBuffer buf)
        {
            buf?.Release();
            buf = null;
        }
    }
}
