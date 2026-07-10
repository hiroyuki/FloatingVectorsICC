// URL presentation seam (Phase 5): the director resolves the download URL and
// asks a presenter to make it visitor-visible. QrUrlPresenter renders a QR
// texture for VisitorMessageUI; a future OscUrlPresenter can push it to an
// external device instead (master plan).

using UnityEngine;

namespace Experience.Publishing
{
    public interface IUrlPresenter
    {
        /// <summary>Produce the visual for the URL (QR texture; may be null on
        /// failure — the caller falls back to showing the URL as text).</summary>
        Texture2D Present(string url);
    }
}
