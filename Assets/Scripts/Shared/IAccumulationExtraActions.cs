// Optional companion to IAccumulationController: extra one-shot actions a
// component wants to expose on the central Control Panel besides Start/Stop/
// Clear (e.g. TSDFTrailBaker's "Resume live"). Kept index-based so the panel
// (Shared.Editor) needs no knowledge of the concrete component.

namespace Shared
{
    public interface IAccumulationExtraActions
    {
        int ExtraActionCount { get; }
        string ExtraActionLabel(int index);
        void RunExtraAction(int index);
    }
}
