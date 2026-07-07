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
        [Tooltip("Total displays to use, including the primary one. 2 = main + " +
                 "second display. Connected displays beyond this count stay off.")]
        public int displayCount = 2;

        void Start()
        {
            int count = Mathf.Min(displayCount, Display.displays.Length);
            for (int i = 1; i < count; i++)
            {
                if (!Display.displays[i].active)
                    Display.displays[i].Activate();
            }
            if (Display.displays.Length < displayCount)
            {
                Debug.LogWarning("[MultiDisplayActivator] Only " + Display.displays.Length +
                                 " display(s) connected; wanted " + displayCount + ".");
            }
        }
    }
}
