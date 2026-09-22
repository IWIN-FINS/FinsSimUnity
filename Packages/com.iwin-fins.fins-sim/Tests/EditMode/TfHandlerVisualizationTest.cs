using NUnit.Framework;
using TestUtils;
using FinsSim.Networking;
using FinsSim.Visualization;
using UnityEngine;

public class TfHandlerVisualizationTest
{
    [SetUp]
    public void SetUp()
    {
        Utils.CreateEmptyScene();
    }

    [TearDown]
    public void TearDown()
    {
        Utils.CreateEmptyScene();
    }

    [Test]
    public void MapTransformIsHiddenWhenDisplayTfDisabled()
    {
        RosConnection.Instance.DisplayTf = false;

        var mapFrame = (Transform)Utils.GetNonpublicField(TfHandler.Instance, "_mapFrame");
        Utils.CallUpdate(Visualizer.Instance);

        Assert.NotNull(mapFrame);
        Assert.AreEqual(0, mapFrame.childCount,
            "DisplayTf=false should not draw the map frame axis geometry.");
    }

    [Test]
    public void MapTransformIsDrawnWhenDisplayTfEnabled()
    {
        RosConnection.Instance.DisplayTf = true;

        var mapFrame = (Transform)Utils.GetNonpublicField(TfHandler.Instance, "_mapFrame");
        Utils.CallUpdate(Visualizer.Instance);

        Assert.NotNull(mapFrame);
        Assert.AreEqual(3, mapFrame.childCount,
            "DisplayTf=true should draw the map frame XYZ axis geometry.");
    }
}
