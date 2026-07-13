// LFKS token resolution, shared by ExperienceDirector (visitor flow) and
// OperatorPublisher (F10 operator flow). The token lives OUTSIDE the
// repo/assets: a user-local file first, the environment second.

using System;
using System.IO;
using UnityEngine;

namespace Experience.Publishing
{
    internal static class LfksToken
    {
        /// <summary>Trimmed token, or null/empty when not configured.</summary>
        public static string Resolve()
        {
            try
            {
                string path = Path.Combine(Application.persistentDataPath, "lfks-token.txt");
                if (File.Exists(path))
                {
                    string fromFile = File.ReadAllText(path).Trim();
                    if (fromFile.Length > 0) return fromFile;
                }
            }
            catch (Exception e)
            {
                Debug.LogWarning($"[{nameof(LfksToken)}] token file read failed: {e.Message}");
            }
            return Environment.GetEnvironmentVariable("LFKS_TOKEN")?.Trim();
        }
    }
}
