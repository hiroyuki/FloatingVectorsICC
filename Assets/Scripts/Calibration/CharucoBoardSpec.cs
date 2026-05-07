using OpenCVForUnity.ObjdetectModule;
using UnityEngine;

namespace Calibration
{
    /// <summary>
    /// Defines a ChArUco board for multi-camera extrinsic calibration.
    /// Lengths are in meters (matches the OpenCV ArUco API and the project-wide
    /// transform convention defined in Plans/issue-9-multicam-extrinsic-calibration.md).
    /// </summary>
    [CreateAssetMenu(menuName = "Calibration/ChArUco Board Spec", fileName = "CharucoBoardSpec")]
    public class CharucoBoardSpec : ScriptableObject
    {
        public enum DictionaryPreset
        {
            Dict4x4_50,
            Dict4x4_100,
            Dict4x4_250,
            Dict4x4_1000,
            Dict5x5_50,
            Dict5x5_100,
            Dict5x5_250,
            Dict5x5_1000,
            Dict6x6_50,
            Dict6x6_100,
            Dict6x6_250,
            Dict6x6_1000,
            Dict7x7_50,
            Dict7x7_100,
            Dict7x7_250,
            Dict7x7_1000,
            DictArucoOriginal,
            DictArucoMip_36h12,
            DictAprilTag_16h5,
            DictAprilTag_25h9,
            DictAprilTag_36h10,
            DictAprilTag_36h11,
        }

        [Tooltip("Squares along X (chessboard columns).")]
        public int squaresX = 7;

        [Tooltip("Squares along Y (chessboard rows). Use a non-square count (e.g. 7x10) so pose is non-ambiguous.")]
        public int squaresY = 10;

        [Tooltip("Length of one chessboard square in METERS.")]
        public float squareLengthMeters = 0.04f;

        [Tooltip("Length of one ArUco marker side in METERS. Must be strictly less than squareLengthMeters.")]
        public float markerLengthMeters = 0.03f;

        [Tooltip("ArUco dictionary preset. Scanned Reality DIN-A3 PDF uses one of the DICT_6X6_* presets — confirm against the actual PDF.")]
        public DictionaryPreset dictionary = DictionaryPreset.Dict6x6_250;

        public int OpenCVDictionaryId
        {
            get
            {
                return dictionary switch
                {
                    DictionaryPreset.Dict4x4_50 => Objdetect.DICT_4X4_50,
                    DictionaryPreset.Dict4x4_100 => Objdetect.DICT_4X4_100,
                    DictionaryPreset.Dict4x4_250 => Objdetect.DICT_4X4_250,
                    DictionaryPreset.Dict4x4_1000 => Objdetect.DICT_4X4_1000,
                    DictionaryPreset.Dict5x5_50 => Objdetect.DICT_5X5_50,
                    DictionaryPreset.Dict5x5_100 => Objdetect.DICT_5X5_100,
                    DictionaryPreset.Dict5x5_250 => Objdetect.DICT_5X5_250,
                    DictionaryPreset.Dict5x5_1000 => Objdetect.DICT_5X5_1000,
                    DictionaryPreset.Dict6x6_50 => Objdetect.DICT_6X6_50,
                    DictionaryPreset.Dict6x6_100 => Objdetect.DICT_6X6_100,
                    DictionaryPreset.Dict6x6_250 => Objdetect.DICT_6X6_250,
                    DictionaryPreset.Dict6x6_1000 => Objdetect.DICT_6X6_1000,
                    DictionaryPreset.Dict7x7_50 => Objdetect.DICT_7X7_50,
                    DictionaryPreset.Dict7x7_100 => Objdetect.DICT_7X7_100,
                    DictionaryPreset.Dict7x7_250 => Objdetect.DICT_7X7_250,
                    DictionaryPreset.Dict7x7_1000 => Objdetect.DICT_7X7_1000,
                    DictionaryPreset.DictArucoOriginal => Objdetect.DICT_ARUCO_ORIGINAL,
                    DictionaryPreset.DictArucoMip_36h12 => Objdetect.DICT_ARUCO_MIP_36h12,
                    DictionaryPreset.DictAprilTag_16h5 => Objdetect.DICT_APRILTAG_16h5,
                    DictionaryPreset.DictAprilTag_25h9 => Objdetect.DICT_APRILTAG_25h9,
                    DictionaryPreset.DictAprilTag_36h10 => Objdetect.DICT_APRILTAG_36h10,
                    DictionaryPreset.DictAprilTag_36h11 => Objdetect.DICT_APRILTAG_36h11,
                    _ => Objdetect.DICT_6X6_250,
                };
            }
        }
    }
}
