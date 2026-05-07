using System.Collections.Generic;
using System.IO;
using NUnit.Framework;
using Orbbec;
using PointCloud;

namespace Calibration.Tests
{
    public class ExtrinsicsYamlRoundTripTests
    {
        private string _tempRoot;

        [SetUp]
        public void SetUp()
        {
            _tempRoot = Path.Combine(Path.GetTempPath(),
                "FvICC_extrinsics_test_" + System.Guid.NewGuid().ToString("N"));
            Directory.CreateDirectory(_tempRoot);
        }

        [TearDown]
        public void TearDown()
        {
            if (_tempRoot != null && Directory.Exists(_tempRoot))
                Directory.Delete(_tempRoot, recursive: true);
        }

        [Test]
        public void Write_Then_Read_RecoversAllFields()
        {
            var src = new[] { MakeCal("CAM-A", 0), MakeCal("CAM-B", 0.5f) };
            PointCloudRecording.WriteExtrinsicsYaml(_tempRoot, src);
            var dst = PointCloudRecording.ReadExtrinsicsYaml(_tempRoot);

            Assert.AreEqual(src.Length, dst.Count);
            for (int i = 0; i < src.Length; i++)
                AssertCalibrationEqual(src[i], dst[i]);
        }

        [Test]
        public void Read_MissingFile_Throws()
        {
            // No file written yet.
            Assert.Throws<FileNotFoundException>(() => PointCloudRecording.ReadExtrinsicsYaml(_tempRoot));
        }

        [Test]
        public void Read_MissingRequiredField_Throws()
        {
            string calDir = PointCloudRecording.CalibrationDir(_tempRoot);
            Directory.CreateDirectory(calDir);
            File.WriteAllText(Path.Combine(calDir, "extrinsics.yaml"),
                "cameras:\n  - device_serial: \"X\"\n    color_intrinsic:\n      fx: 1\n      fy: 2\n");
            Assert.Throws<InvalidDataException>(() => PointCloudRecording.ReadExtrinsicsYaml(_tempRoot));
        }

        [Test]
        public void Read_PathTraversalSerial_Throws()
        {
            // Build a minimal valid file with a malicious serial.
            var src = new[] { MakeCal("../evil", 0) };
            PointCloudRecording.WriteExtrinsicsYaml(_tempRoot, src);
            Assert.Throws<InvalidDataException>(() => PointCloudRecording.ReadExtrinsicsYaml(_tempRoot));
        }

        [Test]
        public void Read_TranslationsRoundTripInMillimeters()
        {
            var src = new[] { MakeCal("MM-CHECK", 0) };
            // SDK convention: Trans is mm. Write→read should preserve that.
            src[0].DepthToColor.Trans = new float[] { 32f, -15.5f, 7.25f };
            src[0].GlobalTrColorCamera = new ObExtrinsic
            {
                Rot = new float[] { 1, 0, 0, 0, 1, 0, 0, 0, 1 },
                Trans = new float[] { 1234.5f, -678.25f, 90f },
            };
            PointCloudRecording.WriteExtrinsicsYaml(_tempRoot, src);
            var dst = PointCloudRecording.ReadExtrinsicsYaml(_tempRoot);

            // Round-trip via meters has ~1e-6 relative error (single precision); allow 1e-2 mm.
            Assert.AreEqual(32f, dst[0].DepthToColor.Trans[0], 1e-2);
            Assert.AreEqual(-15.5f, dst[0].DepthToColor.Trans[1], 1e-2);
            Assert.AreEqual(7.25f, dst[0].DepthToColor.Trans[2], 1e-2);

            Assert.IsTrue(dst[0].GlobalTrColorCamera.HasValue);
            var g = dst[0].GlobalTrColorCamera.Value;
            Assert.AreEqual(1234.5f, g.Trans[0], 1e-1);
            Assert.AreEqual(-678.25f, g.Trans[1], 1e-1);
            Assert.AreEqual(90f, g.Trans[2], 1e-2);
        }

        // ========= helpers =========

        private static PointCloudRecording.DeviceCalibration MakeCal(string serial, float jitter)
        {
            return new PointCloudRecording.DeviceCalibration
            {
                Serial = serial,
                ColorIntrinsic = new ObCameraIntrinsic
                {
                    Fx = 1234.5f + jitter, Fy = 1234.6f + jitter,
                    Cx = 640f + jitter,    Cy = 360f + jitter,
                    Width = 1280, Height = 720,
                },
                DepthIntrinsic = new ObCameraIntrinsic
                {
                    Fx = 500f, Fy = 500f, Cx = 320f, Cy = 288f,
                    Width = 640, Height = 576,
                },
                ColorDistortion = new ObCameraDistortion
                {
                    K1 = 0.01f + jitter, K2 = -0.005f, K3 = 0, K4 = 0, K5 = 0, K6 = 0,
                    P1 = 0.0001f, P2 = -0.0002f,
                    Model = ObCameraDistortionModel.BrownConrady,
                },
                DepthDistortion = new ObCameraDistortion
                {
                    K1 = 0, K2 = 0, K3 = 0, K4 = 0, K5 = 0, K6 = 0, P1 = 0, P2 = 0,
                    Model = ObCameraDistortionModel.None,
                },
                DepthToColor = new ObExtrinsic
                {
                    Rot = new float[] { 1, 0, 0, 0, 1, 0, 0, 0, 1 },
                    Trans = new float[] { 32f, 0, 0 },
                },
                GlobalTrColorCamera = new ObExtrinsic
                {
                    Rot = new float[] { 1, 0, 0, 0, 1, 0, 0, 0, 1 },
                    Trans = new float[] { 0, 0, 0 },
                },
            };
        }

        private static void AssertCalibrationEqual(
            PointCloudRecording.DeviceCalibration a,
            PointCloudRecording.DeviceCalibration b)
        {
            Assert.AreEqual(a.Serial, b.Serial, "Serial");

            AssertIntrinsicEqual(a.ColorIntrinsic, b.ColorIntrinsic, "ColorIntrinsic");
            AssertIntrinsicEqual(a.DepthIntrinsic, b.DepthIntrinsic, "DepthIntrinsic");

            AssertDistortionEqual(a.ColorDistortion, b.ColorDistortion, "ColorDistortion");
            AssertDistortionEqual(a.DepthDistortion, b.DepthDistortion, "DepthDistortion");

            AssertExtrinsicEqual(a.DepthToColor, b.DepthToColor, "DepthToColor");

            Assert.IsTrue(b.GlobalTrColorCamera.HasValue, "GlobalTrColorCamera should be present");
            AssertExtrinsicEqual(a.GlobalTrColorCamera.Value, b.GlobalTrColorCamera.Value, "GlobalTrColorCamera");
        }

        private static void AssertIntrinsicEqual(ObCameraIntrinsic a, ObCameraIntrinsic b, string ctx)
        {
            Assert.AreEqual(a.Fx, b.Fx, 1e-3, $"{ctx}.Fx");
            Assert.AreEqual(a.Fy, b.Fy, 1e-3, $"{ctx}.Fy");
            Assert.AreEqual(a.Cx, b.Cx, 1e-3, $"{ctx}.Cx");
            Assert.AreEqual(a.Cy, b.Cy, 1e-3, $"{ctx}.Cy");
            Assert.AreEqual(a.Width, b.Width, $"{ctx}.Width");
            Assert.AreEqual(a.Height, b.Height, $"{ctx}.Height");
        }

        private static void AssertDistortionEqual(ObCameraDistortion a, ObCameraDistortion b, string ctx)
        {
            Assert.AreEqual(a.K1, b.K1, 1e-6, $"{ctx}.K1");
            Assert.AreEqual(a.K2, b.K2, 1e-6, $"{ctx}.K2");
            Assert.AreEqual(a.K3, b.K3, 1e-6, $"{ctx}.K3");
            Assert.AreEqual(a.K4, b.K4, 1e-6, $"{ctx}.K4");
            Assert.AreEqual(a.K5, b.K5, 1e-6, $"{ctx}.K5");
            Assert.AreEqual(a.K6, b.K6, 1e-6, $"{ctx}.K6");
            Assert.AreEqual(a.P1, b.P1, 1e-6, $"{ctx}.P1");
            Assert.AreEqual(a.P2, b.P2, 1e-6, $"{ctx}.P2");
            Assert.AreEqual(a.Model, b.Model, $"{ctx}.Model");
        }

        private static void AssertExtrinsicEqual(ObExtrinsic a, ObExtrinsic b, string ctx)
        {
            for (int i = 0; i < 9; i++)
                Assert.AreEqual(a.Rot[i], b.Rot[i], 1e-6, $"{ctx}.Rot[{i}]");
            for (int i = 0; i < 3; i++)
                Assert.AreEqual(a.Trans[i], b.Trans[i], 1e-2, $"{ctx}.Trans[{i}]");
        }
    }
}
