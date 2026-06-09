using NUnit.Framework;
using TSDF;

public class TsdfMorphCloseConfigTests
{
    [TestCase(0f, 0.01f, 0)]
    [TestCase(-1f, 0.01f, 0)]
    [TestCase(0.01f, 0.01f, 1)]
    [TestCase(0.06f, 0.01f, 3)]
    [TestCase(0.051f, 0.01f, 3)]
    [TestCase(0.2f, 0.05f, 2)]
    public void ComputeCloseRadiusVoxels_MapsGapToRadius(float gapMeters, float voxelSizeMeters, int expectedRadius)
    {
        int radius = TSDFVolume.ComputeCloseRadiusVoxels(gapMeters, voxelSizeMeters);
        Assert.That(radius, Is.EqualTo(expectedRadius));
    }
}
