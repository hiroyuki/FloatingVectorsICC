// Ownership marker on the canvases VisitorMessageUI builds at runtime.
//
// A component survives an Editor domain reload even though the plain runtime
// bookkeeping that describes it (VisitorMessageUI's _uis / _builtDisplays) does
// not, which is exactly the case the orphan sweep has to recognise. Identifying
// the canvases by name instead would also match anything a scene author happens
// to name the same way, and delete it.

using UnityEngine;

namespace Experience
{
    [DisallowMultipleComponent]
    public sealed class VisitorUiCanvasTag : MonoBehaviour
    {
        [Tooltip("targetDisplay this canvas was built for (Unity's 0-based index).")]
        public int display;

        /// <summary>Already swept and awaiting the end-of-frame Destroy. Unity's
        /// Destroy is deferred, so the object is still reachable for the rest of the
        /// frame; this keeps a second sweep in the same frame from reporting it
        /// twice.</summary>
        [System.NonSerialized] public bool retired;
    }
}
