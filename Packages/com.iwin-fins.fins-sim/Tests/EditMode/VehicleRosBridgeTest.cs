using FinsSim.Actuators;
using FinsSim.Networking;
using NUnit.Framework;
using TestUtils;
using UnityEngine;

public class VehicleRosBridgeTest
{
    GameObject _vehicle;

    [TearDown]
    public void TearDown()
    {
        if (_vehicle != null)
        {
            Object.DestroyImmediate(_vehicle);
            _vehicle = null;
        }
    }

    [Test]
    public void DefaultTopicsUseLowercaseBridgeName()
    {
        _vehicle = new GameObject("FinsROV");
        _vehicle.AddComponent<Rigidbody>();
        var bridge = _vehicle.AddComponent<VehicleRosBridge>();

        Utils.CallAwake(bridge);

        Assert.AreEqual("finsrov", bridge.bridgeName);
        Assert.AreEqual("/finsrov/thrusters_out", bridge.thrusterTopic);
        Assert.AreEqual("/finsrov/pose", bridge.poseTopic);
        Assert.AreEqual("/finsrov/imu_link", bridge.imuTopic);
        Assert.AreEqual("/finsrov/dvl_link", bridge.dvlTopic);
        Assert.AreEqual("/finsrov/depth_link", bridge.depthTopic);
        Assert.AreEqual("/finsrov/reset", bridge.resetTopic);
        Assert.AreEqual("/finsrov/controller/pose", bridge.controllerPoseTopic);
        Assert.AreEqual("/finsrov/controller/imu", bridge.controllerImuTopic);
        Assert.AreEqual("/finsrov/controller/dvl", bridge.controllerDvlTopic);
        Assert.AreEqual("/finsrov/controller/depth", bridge.controllerDepthTopic);
        Assert.AreEqual("controller_world", bridge.controllerWorldFrameId);
        Assert.AreEqual("controller_body", bridge.controllerBodyFrameId);
        Assert.AreEqual(VehicleRosStatePublishMode.SimTruth, bridge.statePublishMode);
    }

    [Test]
    public void ResolvesCurrentFinsRovThrusterNamesInCanonicalOrder()
    {
        var controller = CreateVehicleWithThrusters(
            "Horizontal4",
            "Vertical3",
            "Horizontal2",
            "Vertical1",
            "Horizontal1",
            "Vertical4",
            "Horizontal3",
            "Vertical2");

        var bridge = _vehicle.AddComponent<VehicleRosBridge>();
        bridge.thrusterControllerOverride = controller;

        Utils.CallAwake(bridge);
        var ordered = bridge.GetResolvedOrderedThrustersForTests();

        Assert.AreEqual("Vertical1", ordered[0].name);
        Assert.AreEqual("Vertical2", ordered[1].name);
        Assert.AreEqual("Vertical3", ordered[2].name);
        Assert.AreEqual("Vertical4", ordered[3].name);
        Assert.AreEqual("Horizontal1", ordered[4].name);
        Assert.AreEqual("Horizontal2", ordered[5].name);
        Assert.AreEqual("Horizontal3", ordered[6].name);
        Assert.AreEqual("Horizontal4", ordered[7].name);
    }

    [Test]
    public void ForceNModeAppliesDirectForcesWithoutNormalizedClamp()
    {
        var controller = CreateVehicleWithThrusters(
            "Vertical1",
            "Vertical2",
            "Vertical3",
            "Vertical4",
            "Horizontal1",
            "Horizontal2",
            "Horizontal3",
            "Horizontal4");
        var bridge = _vehicle.AddComponent<VehicleRosBridge>();
        bridge.thrusterControllerOverride = controller;
        bridge.thrusterCommandMode = VehicleRosThrusterCommandMode.ForceN;
        Utils.CallAwake(bridge);

        var command = new[] { 5f, -4f, 3f, -2f, 6f, -7f, 8f, -9f };
        Assert.IsTrue(bridge.ApplyReceivedThrusterValues(command));

        for (int i = 0; i < command.Length; i++)
        {
            Assert.AreEqual(command[i], controller.thrusters[i].TargetForceRequest, 1e-5f);
        }
    }

    [Test]
    public void NormalizedDirectModeClampsBeforeApplyInput()
    {
        var controller = CreateVehicleWithThrusters(
            "Vertical1",
            "Vertical2",
            "Vertical3",
            "Vertical4",
            "Horizontal1",
            "Horizontal2",
            "Horizontal3",
            "Horizontal4");
        var bridge = _vehicle.AddComponent<VehicleRosBridge>();
        bridge.thrusterControllerOverride = controller;
        bridge.thrusterCommandMode = VehicleRosThrusterCommandMode.NormalizedDirect;
        Utils.CallAwake(bridge);

        Assert.IsTrue(bridge.ApplyReceivedThrusterValues(new[] { 2f, -2f, 0.5f, -0.5f, 0f, 1f, -1f, 0.25f }));

        Assert.AreEqual(10f, controller.thrusters[0].TargetForceRequest, 1e-5f);
        Assert.AreEqual(-10f, controller.thrusters[1].TargetForceRequest, 1e-5f);
        Assert.AreEqual(5f, controller.thrusters[2].TargetForceRequest, 1e-5f);
        Assert.AreEqual(-5f, controller.thrusters[3].TargetForceRequest, 1e-5f);
    }

    ThrusterController CreateVehicleWithThrusters(params string[] names)
    {
        _vehicle = new GameObject("FinsROV");
        var rigidBody = _vehicle.AddComponent<Rigidbody>();
        var controller = _vehicle.AddComponent<ThrusterController>();

        foreach (var thrusterName in names)
        {
            var thrusterObject = new GameObject(thrusterName);
            thrusterObject.transform.SetParent(_vehicle.transform);
            var thruster = thrusterObject.AddComponent<Thruster>();
            thruster.TargetRigidbody = rigidBody;
            thruster.Model = Thruster.InputModel.NormalizedForce;
            thruster.MaxForwardForceN = 10f;
            thruster.MaxReverseForceN = -10f;
            Utils.CallNonpublicMethod(thruster, "Start");
            controller.thrusters.Add(thruster);
        }

        return controller;
    }
}
