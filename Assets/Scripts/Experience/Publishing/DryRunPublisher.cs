// Dry-run publisher (Phase 5): waits a configurable delay, then returns fake
// LFKS-shaped URLs derived from the file names. Lets the whole state machine
// (Exporting -> QrShow) run end-to-end with no network and no token.

using System;
using System.IO;
using System.Threading;
using System.Threading.Tasks;

namespace Experience.Publishing
{
    public sealed class DryRunPublisher : ISculptureResultPublisher
    {
        private readonly float _delaySeconds;

        public DryRunPublisher(float delaySeconds = 1f)
        {
            _delaySeconds = Math.Max(0f, delaySeconds);
        }

        public async Task<PublishResult> PublishAsync(string glbPath, string usdzPath, CancellationToken ct)
        {
            await Task.Delay(TimeSpan.FromSeconds(_delaySeconds), ct);
            return new PublishResult
            {
                Success = true,
                GlbUrl = $"https://dry.run/files/{Path.GetFileName(glbPath)}",
                UsdzUrl = $"https://dry.run/files/{Path.GetFileName(usdzPath)}",
                // No server id offline — stand in with the file-name stem so the
                // viewer URL is still well-formed (?id=exp_{stamp}) for dev checks.
                GlbId = Path.GetFileNameWithoutExtension(glbPath),
                UsdzId = Path.GetFileNameWithoutExtension(usdzPath),
            };
        }
    }
}
