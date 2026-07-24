// Publishing seam for the experience flow (Phase 5): the director hands the
// exported .glb/.usdz to a publisher and gets back download URLs for the QR.
// DryRunPublisher (below) is the Phase 5 default; LfksUploadPublisher
// (upload.ps1 + LFKS_TOKEN env) arrives in Phase 7 behind this same interface.

using System.Threading;
using System.Threading.Tasks;

namespace Experience.Publishing
{
    public struct PublishResult
    {
        public bool Success;
        public string GlbUrl;
        public string UsdzUrl;
        // Server-assigned file ids from the upload API (LfksResult.id). The QR's
        // viewer URL keys off these (?id=…), not the local file name. Empty from
        // the dry-run publisher, which has no server round-trip.
        public string GlbId;
        public string UsdzId;
        public string Error;
    }

    public interface ISculptureResultPublisher
    {
        /// <summary>Upload both files; resolve to download URLs. Must observe
        /// <paramref name="ct"/> (mode exit cancels). Never throws — failures
        /// come back as Success=false + Error (cancellation may throw
        /// OperationCanceledException, which the caller treats as abort).</summary>
        Task<PublishResult> PublishAsync(string glbPath, string usdzPath, CancellationToken ct);
    }
}
