// Real LFKS publisher (Phase 7, Plans/phase7-upload-plan.md): runs the PINNED
// local copy of upload.ps1 (Assets/StreamingAssets/lfks/, SHA-256 gated — a
// tampered script never executes) once per file and returns the downloadUrls
// for the QR.
//
// Token trust boundary: the token value goes into the child process's
// environment only. The command line contains the LITERAL string
// "$env:LFKS_TOKEN" (expanded inside the child PowerShell), so the token never
// appears in the OS process list, in config assets, or in logs. There is no
// wrapper script — the only code that sees the token is the SHA-verified
// upload.ps1 itself.
//
// Process hygiene: stdout/stderr are drained via ReadToEndAsync started
// immediately after Start() (pipe-full deadlocks are impossible), and a
// timeout/cancel kills the process, waits for exit, then awaits both drains.

using System;
using System.Diagnostics;
using System.IO;
using System.Security.Cryptography;
using System.Threading;
using System.Threading.Tasks;

namespace Experience.Publishing
{
    public sealed class LfksUploadPublisher : ISculptureResultPublisher
    {
        [Serializable]
        private class LfksResult
        {
            public string id;
            public string name;
            public string relativePath;
            public long size;
            public string downloadUrl;
            public string fs;
        }

        private readonly string _scriptPath;
        private readonly string _expectedSha256;
        private readonly string _token;
        private readonly string _remoteDirectory;
        private readonly string _apiUrl;
        private readonly float _timeoutSeconds;

        public LfksUploadPublisher(string uploadScriptPath, string expectedScriptSha256,
                                   string token, string remoteDirectory, float timeoutSeconds = 60f,
                                   string apiUrl = "")
        {
            _scriptPath = uploadScriptPath;
            _expectedSha256 = (expectedScriptSha256 ?? "").Trim();
            _token = (token ?? "").Trim();
            _remoteDirectory = remoteDirectory ?? "";
            _apiUrl = (apiUrl ?? "").Trim();
            _timeoutSeconds = timeoutSeconds > 0f && float.IsFinite(timeoutSeconds) ? timeoutSeconds : 60f;
        }

        public async Task<PublishResult> PublishAsync(string glbPath, string usdzPath, CancellationToken ct)
        {
            if (_token.Length == 0)
                return Fail("no LFKS token (persistentDataPath/lfks-token.txt or LFKS_TOKEN env)");
            // Before the token leaves the process, not per-file: an unknown origin is
            // a configuration mistake, and failing the whole publish makes it visible.
            if (!IsAllowedApiOrigin(_apiUrl))
                return Fail($"lfksApiUrl '{_apiUrl}' is not an allow-listed LFKS origin " +
                            $"({string.Join(", ", AllowedApiOrigins)}) — refusing to send the token");
            if (!VerifyScript(out string why))
                return Fail(why);

            var (glbUrl, _, glbErr) = await UploadWithRetryAsync(glbPath, ct);
            if (glbUrl == null) return Fail($"glb upload failed: {glbErr}");
            var (usdzUrl, _, usdzErr) = await UploadWithRetryAsync(usdzPath, ct);
            if (usdzUrl == null) return Fail($"usdz upload failed: {usdzErr}");

            // The viewer's ?id= is the fs file name, not the API's opaque id. We
            // upload without -Name, so the stored name is the local file name and
            // the sculpture id is that name without its extension. (The opaque id
            // is still what GlbUrl/UsdzUrl — /files/{id} — are built from.)
            return new PublishResult
            {
                Success = true, GlbUrl = glbUrl, UsdzUrl = usdzUrl,
                GlbId = Path.GetFileNameWithoutExtension(glbPath),
                UsdzId = Path.GetFileNameWithoutExtension(usdzPath),
            };
        }

        private static PublishResult Fail(string error) =>
            new PublishResult { Success = false, Error = error };

        // The SHA gate: the pinned script is the whole trust boundary — never
        // run it when the bytes changed (server rotation, tampering, disk rot).
        private bool VerifyScript(out string error)
        {
            error = null;
            if (!File.Exists(_scriptPath)) { error = $"upload script missing: {_scriptPath}"; return false; }
            if (_expectedSha256.Length != 64) { error = "uploadScriptSha256 is not configured"; return false; }
            using var sha = SHA256.Create();
            string actual = BitConverter.ToString(sha.ComputeHash(File.ReadAllBytes(_scriptPath)))
                                        .Replace("-", "").ToLowerInvariant();
            if (!string.Equals(actual, _expectedSha256.ToLowerInvariant(), StringComparison.Ordinal))
            {
                error = $"upload.ps1 SHA-256 mismatch — refusing to execute " +
                        $"(expected {_expectedSha256}, got {actual})";
                return false;
            }
            return true;
        }

        private async Task<(string url, string id, string error)> UploadWithRetryAsync(string filePath, CancellationToken ct)
        {
            var (url, id, error) = await UploadOnceAsync(filePath, ct);
            if (url != null) return (url, id, null);
            ct.ThrowIfCancellationRequested();
            UnityEngine.Debug.LogWarning($"[LfksUploadPublisher] retrying {Path.GetFileName(filePath)}: {error}");
            return await UploadOnceAsync(filePath, ct);
        }

        private async Task<(string url, string id, string error)> UploadOnceAsync(string filePath, CancellationToken ct)
        {
            if (!File.Exists(filePath)) return (null, null, $"file missing: {filePath}");
            // Injection guard for every value interpolated into the command:
            // a single quote would escape the PowerShell string, a DOUBLE quote
            // would terminate the outer Windows -Command argument itself, and
            // control characters have no business in a path. Our values never
            // contain any of these — a hostile config must not build a shell.
            if (!IsSafeArgument(_scriptPath) || !IsSafeArgument(filePath) || !IsSafeArgument(_remoteDirectory)
                || !IsSafeArgument(_apiUrl))
                return (null, null, "path, remote directory or api url contains a quote/control character");

            string dirArg = _remoteDirectory.Length > 0 ? $" -Directory '{_remoteDirectory}'" : "";
            // Empty = the script's own baked-in origin (the domain it was pinned
            // from). Set it when the token belongs to a different deployment —
            // a token issued for one origin is rejected by the other with a
            // misleading "invalid or expired token".
            string urlArg = _apiUrl.Length > 0 ? $" -Url '{_apiUrl}'" : "";
            var psi = new ProcessStartInfo
            {
                FileName = "powershell.exe",
                Arguments = "-NoProfile -ExecutionPolicy Bypass -Command " +
                            $"\"& '{_scriptPath}' -Token $env:LFKS_TOKEN -File '{filePath}'{dirArg}{urlArg} -Json\"",
                UseShellExecute = false,
                RedirectStandardOutput = true,
                RedirectStandardError = true,
                CreateNoWindow = true,
            };
            psi.EnvironmentVariables["LFKS_TOKEN"] = _token;

            using var proc = new Process { StartInfo = psi };
            try { proc.Start(); }
            catch (Exception e) { return (null, null, $"could not start powershell: {e.Message}"); }

            // Start draining BOTH pipes before waiting — a full pipe would
            // otherwise deadlock the child.
            var stdoutTask = proc.StandardOutput.ReadToEndAsync();
            var stderrTask = proc.StandardError.ReadToEndAsync();

            var sw = Stopwatch.StartNew();
            while (!proc.HasExited)
            {
                if (ct.IsCancellationRequested || sw.Elapsed.TotalSeconds > _timeoutSeconds)
                {
                    try { proc.Kill(); } catch { /* already gone */ }
                    try { proc.WaitForExit(3000); } catch { }
                    await Task.WhenAll(stdoutTask, stderrTask); // explicit post-kill drain
                    if (ct.IsCancellationRequested) ct.ThrowIfCancellationRequested();
                    return (null, null, $"timed out after {_timeoutSeconds:0}s: {Tail(stderrTask.Result)}");
                }
                await Task.Delay(100, CancellationToken.None);
            }

            string stdout = await stdoutTask;
            string stderr = await stderrTask;
            if (proc.ExitCode != 0)
                return (null, null, $"upload.ps1 exited {proc.ExitCode}: {Tail(stderr)}");

            // stdout carries exactly one JSON line, but progress/blank lines
            // must not break us — take the last {...}-shaped line.
            string json = null;
            foreach (var raw in stdout.Split('\n'))
            {
                string line = raw.Trim();
                if (line.StartsWith("{") && line.EndsWith("}")) json = line;
            }
            if (json == null) return (null, null, $"no JSON result in output: {Tail(stdout)}");

            LfksResult result;
            try { result = UnityEngine.JsonUtility.FromJson<LfksResult>(json); }
            catch (Exception e) { return (null, null, $"result parse failed: {e.Message}"); }
            if (result == null || string.IsNullOrEmpty(result.downloadUrl))
                return (null, null, $"no downloadUrl in result: {json}");
            return (result.downloadUrl, result.id, null);
        }

        /// <summary>
        /// The only origins the token may ever be sent to. `-Url` overrides where the
        /// SHA-pinned script POSTs `?token=...`, so without this an edit to a config
        /// asset — which changes no hashed bytes and passes the quote guard — would
        /// hand the upload token to any HTTPS host. Adding a deployment here is a
        /// deliberate, reviewable code change; pointing at one is not.
        /// </summary>
        private static readonly string[] AllowedApiOrigins =
        {
            "https://ntticc.lfks.app",                        // production
            "https://lfks-staging.circuit-lab.workers.dev",   // staging
        };

        /// <summary>Empty (= the script's own baked origin) or an exact allow-listed
        /// origin. Case-insensitive, trailing slash tolerated, nothing else.</summary>
        private static bool IsAllowedApiOrigin(string url)
        {
            if (string.IsNullOrEmpty(url)) return true;
            string trimmed = url.TrimEnd('/');
            foreach (string allowed in AllowedApiOrigins)
                if (string.Equals(trimmed, allowed, StringComparison.OrdinalIgnoreCase)) return true;
            return false;
        }

        private static bool IsSafeArgument(string s)
        {
            if (s == null) return false;
            foreach (char c in s)
                if (c == '\'' || c == '"' || c == '`' || char.IsControl(c)) return false;
            return true;
        }

        private static string Tail(string s)
        {
            if (string.IsNullOrEmpty(s)) return "(no output)";
            s = s.Trim();
            return s.Length <= 300 ? s : s.Substring(s.Length - 300);
        }
    }
}
