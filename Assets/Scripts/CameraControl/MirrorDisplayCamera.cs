// Mirrors a camera's image horizontally, for the visitor-facing displays.
//
// Why this exists: a camera pointed at the visitor renders them un-mirrored, so
// raising the right hand moves the figure on the screen's left — correct, but it
// reads as "the wrong arm moved" to someone watching themselves. Every mirror
// people have ever used flips left/right, so the installation should too.
//
// This is deliberately a DISPLAY-only transform. The obvious alternative —
// negating X somewhere in the point cloud / skeleton pipeline — would make the
// reconstructed world left-handed, and the calibration, the STL export and the
// body-tracking joint labels all assume it is not. Body tracking was verified
// (2026-07-21) to label left/right correctly, so there is nothing to fix in the
// data; only the presentation is being flipped.
//
// The flip is done by negating the projection matrix's X scale rather than by
// scaling the transform, so nothing in the scene graph moves and no other camera
// is affected. Negating one axis reverses triangle winding, so front faces would
// be culled as back faces — GL.invertCulling is toggled around this camera's
// render to compensate. Point clouds are unaffected by winding, but anything
// solid sharing the display would otherwise turn inside out.
//
// Screen Space - Overlay canvases do NOT go through a camera projection, so the
// visitor UI and QR code stay readable. World-space text on a mirrored display
// would come out reversed — there is none today; add an exception here if that
// changes.

using UnityEngine;
using UnityEngine.Rendering;

namespace CameraControl
{
    [RequireComponent(typeof(Camera))]
    [DisallowMultipleComponent]
    public class MirrorDisplayCamera : MonoBehaviour
    {
        [Tooltip("Flip this camera's image left/right, so visitors see themselves " +
                 "as in a mirror. Display-only: the world, the calibration and the " +
                 "skeleton are untouched.")]
        public bool mirrorHorizontally = true;

        private Camera _cam;
        private bool _hooked;

        private void OnEnable()
        {
            _cam = GetComponent<Camera>();
            if (!_hooked)
            {
                RenderPipelineManager.beginCameraRendering += OnBeginCameraRendering;
                RenderPipelineManager.endCameraRendering += OnEndCameraRendering;
                _hooked = true;
            }
        }

        private void OnDisable()
        {
            if (_hooked)
            {
                RenderPipelineManager.beginCameraRendering -= OnBeginCameraRendering;
                RenderPipelineManager.endCameraRendering -= OnEndCameraRendering;
                _hooked = false;
            }
            // Hand the camera back unmirrored: a stale custom projection would
            // otherwise survive this component being switched off, and it also
            // freezes out aspect-ratio updates.
            if (_cam != null) _cam.ResetProjectionMatrix();
            GL.invertCulling = false;
        }

        // Rebuilt every frame rather than once on enable, so a resolution or
        // aspect change still reaches the projection (ResetProjectionMatrix
        // recomputes from the camera's current fov/aspect/clip planes).
        private void LateUpdate()
        {
            if (_cam == null) return;
            _cam.ResetProjectionMatrix();
            if (!mirrorHorizontally) return;
            Matrix4x4 p = _cam.projectionMatrix;
            p.m00 = -p.m00;
            p.m01 = -p.m01;
            p.m02 = -p.m02;
            p.m03 = -p.m03;
            _cam.projectionMatrix = p;
        }

        private void OnBeginCameraRendering(ScriptableRenderContext ctx, Camera cam)
        {
            if (mirrorHorizontally && cam == _cam) GL.invertCulling = true;
        }

        private void OnEndCameraRendering(ScriptableRenderContext ctx, Camera cam)
        {
            if (cam == _cam) GL.invertCulling = false;
        }
    }
}
