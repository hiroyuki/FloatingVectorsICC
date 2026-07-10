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
        private readonly float _timeoutSeconds;

        public LfksUploadPublisher(string uploadScriptPath, string expectedScriptSha256,
                                   string token, string remoteDirectory, float timeoutSeconds = 60f)
        {
            _scriptPath = uploadScriptPath;
            _expectedSha256 = (expectedScriptSha256 ?? "").Trim();
            _token = (token ?? "").Trim();
            _remoteDirectory = remoteDirectory ?? "";
            _timeoutSeconds = timeoutSeconds > 0f && float.IsFinite(timeoutSeconds) ? timeoutSeconds : 60f;
        }

        public async Task<PublishResult> PublishAsync(string glbPath, string usdzPath, CancellationToken ct)
        {
            if (_token.Length == 0)
                return Fail("no LFKS token (persistentDataPath/lfks-token.txt or LFKS_TOKEN env)");
            if (!VerifyScript(out string why))
                return Fail(why);

            var (glbUrl, glbErr) = await UploadWithRetryAsync(glbPath, ct);
            if (glbUrl == null) return Fail($"glb upload failed: {glbErr}");
            var (usdzUrl, usdzErr) = await UploadWithRetryAsync(usdzPath, ct);
            if (usdzUrl == null) return Fail($"usdz upload failed: {usdzErr}");

            return new PublishResult { Success = true, GlbUrl = glbUrl, UsdzUrl = usdzUrl };
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

        private async Task<(string url, string error)> UploadWithRetryAsync(string filePath, CancellationToken ct)
        {
            var (url, error) = await UploadOnceAsync(filePath, ct);
            if (url != null) return (url, null);
            ct.ThrowIfCancellationRequested();
            UnityEngine.Debug.LogWarning($"[LfksUploadPublisher] retrying {Path.GetFileName(filePath)}: {error}");
            return await UploadOnceAsync(filePath, ct);
        }

        private async Task<(string url, string error)> UploadOnceAsync(string filePath, CancellationToken ct)
        {
            if (!File.Exists(filePath)) return (null, $"file missing: {filePath}");
            // Injection guard for every value interpolated into the command:
            // a single quote would escape the PowerShell string, a DOUBLE quote
            // would terminate the outer Windows -Command argument itself, and
            // control characters have no business in a path. Our values never
            // contain any of these — a hostile config must not build a shell.
            if (!IsSafeArgument(_scriptPath) || !IsSafeArgument(filePath) || !IsSafeArgument(_remoteDirectory))
                return (null, "path or remote directory contains a quote/control character");

            string dirArg = _remoteDirectory.Length > 0 ? $" -Directory '{_remoteDirectory}'" : "";
            var psi = new ProcessStartInfo
            {
                FileName = "powershell.exe",
                Arguments = "-NoProfile -ExecutionPolicy Bypass -Command " +
                            $"\"& '{_scriptPath}' -Token $env:LFKS_TOKEN -File '{filePath}'{dirArg} -Json\"",
                UseShellExecute = false,
                RedirectStandardOutput = true,
                RedirectStandardError = true,
                CreateNoWindow = true,
            };
            psi.EnvironmentVariables["LFKS_TOKEN"] = _token;

            using var proc = new Process { StartInfo = psi };
            try { proc.Start(); }
            catch (Exception e) { return (null, $"could not start powershell: {e.Message}"); }

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
                    return (null, $"timed out after {_timeoutSeconds:0}s: {Tail(stderrTask.Result)}");
                }
                await Task.Delay(100, CancellationToken.None);
            }

            string stdout = await stdoutTask;
            string stderr = await stderrTask;
            if (proc.ExitCode != 0)
                return (null, $"upload.ps1 exited {proc.ExitCode}: {Tail(stderr)}");

            // stdout carries exactly one JSON line, but progress/blank lines
            // must not break us — take the last {...}-shaped line.
            string json = null;
            foreach (var raw in stdout.Split('\n'))
            {
                string line = raw.Trim();
                if (line.StartsWith("{") && line.EndsWith("}")) json = line;
            }
            if (json == null) return (null, $"no JSON result in output: {Tail(stdout)}");

            LfksResult result;
            try { result = UnityEngine.JsonUtility.FromJson<LfksResult>(json); }
            catch (Exception e) { return (null, $"result parse failed: {e.Message}"); }
            if (result == null || string.IsNullOrEmpty(result.downloadUrl))
                return (null, $"no downloadUrl in result: {json}");
            return (result.downloadUrl, null);
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
