// Activates additional OS displays so cameras with targetDisplay >= 1 show up
// in standalone builds. Display 1 (index 0) is always active; the rest are off
// until Display.displays[i].Activate() is called once.
//
// In the Editor this is a no-op: preview Display 2 by opening a second Game
// view and picking "Display 2" in its display dropdown.

using UnityEngine;

namespace CameraControl
{
    public class MultiDisplayActivator : MonoBehaviour
    {
        [Tooltip("Total displays to use, including the primary one. 3 = operator + " +
                 "two visitor displays (the experience-flow rig). Connected displays " +
                 "beyond this count stay off.")]
        public int displayCount = 3;

        [Tooltip("Seconds between activation retries while a wanted display is still " +
                 "missing or inactive. A monitor can enumerate after this component's " +
                 "Start (slow DisplayPort/HDMI handshake, a hub powering up, hot-plug), " +
                 "and a one-shot Activate would leave it dark for the whole run — " +
                 "VisitorMessageUI also waits for the activation before it builds that " +
                 "display's canvas. 0 disables the retry (one attempt only).")]
        [Min(0f)]
        public float retryIntervalSeconds = 1f;

        private float _nextAttempt;
        private int _reportedDisplayCount = -1;

        void Start()
        {
            ActivateWanted();
        }

        void Update()
        {
            if (retryIntervalSeconds <= 0f) return;
            if (AllWantedActive()) return;
            if (Time.unscaledTime < _nextAttempt) return;
            _nextAttempt = Time.unscaledTime + retryIntervalSeconds;
            ActivateWanted();
        }

        private bool AllWantedActive()
        {
            if (Display.displays.Length < displayCount) return false;
            for (int i = 1; i < displayCount; i++)
                if (!Display.displays[i].active) return false;
            return true;
        }

        private void ActivateWanted()
        {
            int count = Mathf.Min(displayCount, Display.displays.Length);
            for (int i = 1; i < count; i++)
            {
                if (!Display.displays[i].active)
                    Display.displays[i].Activate();
            }
            // Only when the connected count CHANGES, so the retry loop cannot spam
            // the log for a run that permanently has fewer monitors.
            if (Display.displays.Length != _reportedDisplayCount)
            {
                _reportedDisplayCount = Display.displays.Length;
                if (Display.displays.Length < displayCount)
                    Debug.LogWarning("[MultiDisplayActivator] Only " + Display.displays.Length +
                                     " display(s) connected; wanted " + displayCount +
                                     ". Retrying every " + retryIntervalSeconds + "s.");
                else
                    Debug.Log("[MultiDisplayActivator] " + Display.displays.Length +
                              " display(s) available; activated up to " + displayCount + ".");
            }
        }
    }
}
