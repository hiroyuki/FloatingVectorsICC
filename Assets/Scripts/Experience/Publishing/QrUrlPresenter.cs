// QR renderer (Phase 5): vendored Net.Codecrete.QrCodeGenerator (ThirdParty/,
// MIT, pinned SHA — see README-VENDORED.md) encodes the URL; the modules are
// written 1:1 into a point-filtered Texture2D that VisitorMessageUI's RawImage
// scales up crisply. A quiet-zone border keeps phone cameras happy.

using Net.Codecrete.QrCodeGenerator;
using UnityEngine;

namespace Experience.Publishing
{
    public sealed class QrUrlPresenter : IUrlPresenter
    {
        private readonly int _border;

        public QrUrlPresenter(int borderModules = 3)
        {
            _border = Mathf.Max(0, borderModules);
        }

        public Texture2D Present(string url)
        {
            if (string.IsNullOrEmpty(url)) return null;
            QrCode qr;
            try { qr = QrCode.EncodeText(url, QrCode.Ecc.Medium); }
            catch (System.Exception e)
            {
                Debug.LogWarning($"[QrUrlPresenter] QR encode failed for \"{url}\": {e.Message}");
                return null;
            }

            int size = qr.Size + _border * 2;
            var tex = new Texture2D(size, size, TextureFormat.RGB24, false)
            {
                filterMode = FilterMode.Point,
                wrapMode = TextureWrapMode.Clamp,
                name = "_QrUrl",
            };
            var px = new Color32[size * size];
            var black = new Color32(0, 0, 0, 255);
            var white = new Color32(255, 255, 255, 255);
            for (int y = 0; y < size; y++)
                for (int x = 0; x < size; x++)
                {
                    bool dark = x >= _border && y >= _border &&
                                x < size - _border && y < size - _border &&
                                qr.GetModule(x - _border, y - _border);
                    // QR row 0 is the top; texture row 0 is the bottom.
                    px[(size - 1 - y) * size + x] = dark ? black : white;
                }
            tex.SetPixels32(px);
            tex.Apply(false, true);
            return tex;
        }
    }
}
