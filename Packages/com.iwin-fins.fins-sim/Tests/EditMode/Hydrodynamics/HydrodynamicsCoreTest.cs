using FinsSim.Hydrodynamics;
using FinsSim.Core.Spatial;
using NUnit.Framework;
using UnityEngine;

public class HydrodynamicsCoreTest
{
    [Test]
    public void SurfaceGeometryHydroUsesClosedMeshVolumeForFullySubmergedBuoyancy()
    {
        var go = new GameObject("surface_mesh_full_submerge_test");
        var profile = ScriptableObject.CreateInstance<HydrodynamicsProfile>();
        var mesh = CreateUnitCubeMesh();
        try
        {
            var body = go.AddComponent<Rigidbody>();
            body.useGravity = false;
            profile.waterDensity = 1000f;
            profile.gravityMagnitude = 10f;
            profile.displacedVolume = 0.1f;
            profile.useMeshVolumeForBuoyancy = true;
            profile.maxSurfaceTriangles = 64;

            var context = new HydrodynamicsContext(
                body, go.transform, profile, null, WaterKinematicsSample.Flat(2f, Vector3.zero), 0.02f, mesh);
            HydrodynamicsWrench wrench = new SurfaceGeometryHydroBackend().Compute(context);

            Assert.That(wrench.SubmergedVolume, Is.EqualTo(1f).Within(1e-5f));
            Assert.That(wrench.WorldForce.y, Is.EqualTo(10000f).Within(1e-2f));
            Assert.That(wrench.WorldForce.x, Is.EqualTo(0f).Within(1e-5f));
            Assert.That(wrench.WorldForce.z, Is.EqualTo(0f).Within(1e-5f));
        }
        finally
        {
            Object.DestroyImmediate(mesh);
            Object.DestroyImmediate(profile);
            Object.DestroyImmediate(go);
        }
    }

    [Test]
    public void SurfaceGeometryHydroIntegratesHalfSubmergedClosedMeshVolume()
    {
        var go = new GameObject("surface_mesh_half_submerge_test");
        var profile = ScriptableObject.CreateInstance<HydrodynamicsProfile>();
        var mesh = CreateUnitCubeMesh();
        try
        {
            var body = go.AddComponent<Rigidbody>();
            body.useGravity = false;
            profile.waterDensity = 1000f;
            profile.gravityMagnitude = 10f;
            profile.useMeshVolumeForBuoyancy = true;
            profile.maxSurfaceTriangles = 64;

            var context = new HydrodynamicsContext(
                body, go.transform, profile, null, WaterKinematicsSample.Flat(0f, Vector3.zero), 0.02f, mesh);
            HydrodynamicsWrench wrench = new SurfaceGeometryHydroBackend().Compute(context);

            Assert.That(wrench.SubmergedVolume, Is.EqualTo(0.5f).Within(1e-5f));
            Assert.That(wrench.WorldForce.y, Is.EqualTo(5000f).Within(1e-2f));
        }
        finally
        {
            Object.DestroyImmediate(mesh);
            Object.DestroyImmediate(profile);
            Object.DestroyImmediate(go);
        }
    }

    [Test]
    public void UnityFossenVelocityMappingRoundTripForWrench()
    {
        var go = new GameObject("hydro_mapping_test");
        try
        {
            go.transform.rotation = Quaternion.Euler(0f, 90f, 0f);
            var bodyWrench = new SixDofVector(1f, 2f, 3f, 4f, 5f, 6f);

            HydroMath.FossenBodyWrenchToUnityWorld(go.transform, bodyWrench, out Vector3 force, out Vector3 torque);
            SixDofVector mappedBack = HydroMath.UnityWorldVelocityToFossen(go.transform, force, torque);

            Assert.That(mappedBack.u, Is.EqualTo(bodyWrench.u).Within(1e-5f));
            Assert.That(mappedBack.v, Is.EqualTo(bodyWrench.v).Within(1e-5f));
            Assert.That(mappedBack.w, Is.EqualTo(bodyWrench.w).Within(1e-5f));
            Assert.That(mappedBack.p, Is.EqualTo(bodyWrench.p).Within(1e-5f));
            Assert.That(mappedBack.q, Is.EqualTo(bodyWrench.q).Within(1e-5f));
            Assert.That(mappedBack.r, Is.EqualTo(bodyWrench.r).Within(1e-5f));
        }
        finally
        {
            Object.DestroyImmediate(go);
        }
    }

    [Test]
    public void DiagonalDampingDoesNotInjectEnergy()
    {
        var linear = new SixDofVector(1f, -2f, 3f, -4f, 5f, -6f);
        var quadratic = new SixDofVector(2f, 2f, 2f, 2f, 2f, 2f);
        var velocity = new SixDofVector(0.4f, -0.2f, 0.8f, -0.5f, 0.1f, 0.3f);

        SixDofVector damping = HydroMath.ComputeDiagonalDamping(linear, quadratic, SixDofVector.Zero, velocity);

        Assert.That(damping.Dot(velocity), Is.LessThanOrEqualTo(1e-6f));
    }

    [Test]
    public void LearningToSwimEquivalentBoxMatchesReferenceHydrostaticAndDragFormula()
    {
        var go = new GameObject("learning_to_swim_equivalent_box_test");
        var profile = ScriptableObject.CreateInstance<HydrodynamicsProfile>();
        try
        {
            var body = go.AddComponent<Rigidbody>();
            body.useGravity = true;
            profile.waterDensity = 1027f;
            profile.gravityMagnitude = 9.81f;
            profile.displacedVolume = 0.0118f;
            profile.centerOfMass = new Vector3(0f, -0.08f, 0f);
            profile.centerOfBuoyancy = new Vector3(0f, 0.03f, 0f);
            profile.equivalentBoxWaterDynamicViscosity = 0.001306f;
            profile.equivalentBoxMass = 12.11f;
            profile.equivalentBoxInertia = new Vector3(0.13154832f, 0.23185866f, 0.24651341f);
            body.linearVelocity = new Vector3(0.7f, 0f, 0.2f);
            body.angularVelocity = new Vector3(0.3f, 0.4f, 0.5f);

            var context = new HydrodynamicsContext(
                body,
                go.transform,
                profile,
                null,
                WaterKinematicsSample.Flat(0f, Vector3.zero),
                0.02f,
                null,
                BodyAxisConvention.FinsRovXForwardYUpZLeft);
            HydrodynamicsWrench wrench = new LearningToSwimEquivalentBoxBackend().Compute(context);

            Assert.That(wrench.SubmergedVolume, Is.EqualTo(0.0118f).Within(1e-6f));
            Assert.That(wrench.WorldForce.y, Is.EqualTo(1027f * 9.81f * 0.0118f).Within(0.1f));
            Assert.That(wrench.BodyWrench.u, Is.LessThan(0f));
            Assert.That(wrench.BodyWrench.v, Is.LessThan(0f));
            Assert.That(wrench.BodyWrench.w, Is.LessThan(0f));
            Assert.That(wrench.BodyWrench.p, Is.LessThan(0f));
            Assert.That(wrench.BodyWrench.q, Is.LessThan(0f));
            Assert.That(wrench.BodyWrench.r, Is.LessThan(0f));
        }
        finally
        {
            Object.DestroyImmediate(profile);
            Object.DestroyImmediate(go);
        }
    }

    [Test]
    public void RandomizationContextIsSeedReproducible()
    {
        var profile = ScriptableObject.CreateInstance<DomainRandomizationProfile>();
        try
        {
            profile.massScale = new FloatRange(0.8f, 1.2f);
            profile.initialPositionOffset = new Vector3Range(new Vector3(-1f, -2f, -3f), new Vector3(1f, 2f, 3f));
            profile.waveComponentCount = new IntRange(1, 3);
            profile.linearDampingAxisScaleMin = new SixDofVector(0.5f, 0.6f, 0.7f, 0.8f, 0.9f, 1.0f);
            profile.linearDampingAxisScaleMax = new SixDofVector(1.5f, 1.6f, 1.7f, 1.8f, 1.9f, 2.0f);

            var a = new RandomizationContext(profile, DomainRandomizationMode.Train, 42, 0);
            var b = new RandomizationContext(profile, DomainRandomizationMode.Train, 42, 0);

            Assert.That(a.Range(profile.massScale), Is.EqualTo(b.Range(profile.massScale)).Within(1e-6f));
            Assert.That(a.Range(profile.initialPositionOffset), Is.EqualTo(b.Range(profile.initialPositionOffset)));
            Assert.That(a.Range(profile.waveComponentCount), Is.EqualTo(b.Range(profile.waveComponentCount)));
            SixDofVector sixDofA = a.Range(profile.linearDampingAxisScaleMin, profile.linearDampingAxisScaleMax);
            SixDofVector sixDofB = b.Range(profile.linearDampingAxisScaleMin, profile.linearDampingAxisScaleMax);
            Assert.That(sixDofA.u, Is.EqualTo(sixDofB.u).Within(1e-6f));
            Assert.That(sixDofA.v, Is.EqualTo(sixDofB.v).Within(1e-6f));
            Assert.That(sixDofA.w, Is.EqualTo(sixDofB.w).Within(1e-6f));
            Assert.That(sixDofA.p, Is.EqualTo(sixDofB.p).Within(1e-6f));
            Assert.That(sixDofA.q, Is.EqualTo(sixDofB.q).Within(1e-6f));
            Assert.That(sixDofA.r, Is.EqualTo(sixDofB.r).Within(1e-6f));
        }
        finally
        {
            Object.DestroyImmediate(profile);
        }
    }

    [Test]
    public void KeyedRandomizationIsStableAndIndependentOfSequentialSamples()
    {
        var profile = ScriptableObject.CreateInstance<DomainRandomizationProfile>();
        try
        {
            var first = new RandomizationContext(profile, DomainRandomizationMode.Train, 42, 3);
            var second = new RandomizationContext(profile, DomainRandomizationMode.Train, 42, 3);
            var differentSeed = new RandomizationContext(profile, DomainRandomizationMode.Train, 43, 3);
            FloatRange range = new FloatRange(0.8f, 1.2f);

            float firstMassScale = first.Range(range, "body.mass-scale");
            first.Value();
            first.Range(range);
            float secondMassScale = second.Range(range, "body.mass-scale");

            Assert.That(secondMassScale, Is.EqualTo(firstMassScale).Within(1e-6f));
            Assert.That(differentSeed.Range(range, "body.mass-scale"), Is.Not.EqualTo(firstMassScale));
        }
        finally
        {
            Object.DestroyImmediate(profile);
        }
    }

    [Test]
    public void HydrodynamicsProfileAppliesAxisDampingScales()
    {
        var profile = ScriptableObject.CreateInstance<HydrodynamicsProfile>();
        try
        {
            profile.addedMassDiagonal = new SixDofVector(1f, 2f, 3f, 4f, 5f, 6f);
            profile.linearDamping = new SixDofVector(10f, 20f, 30f, 40f, 50f, 60f);
            profile.quadraticDamping = new SixDofVector(100f, 200f, 300f, 400f, 500f, 600f);

            profile.ScaleHydrodynamicAxisCoefficients(
                new SixDofVector(1f, 1f, 1f, 1f, 1f, 0.5f),
                new SixDofVector(1f, 1f, 1f, 1f, 1f, 2f),
                new SixDofVector(1f, 1f, 1f, 1f, 1f, 0.25f));

            Assert.That(profile.addedMassDiagonal.u, Is.EqualTo(1f).Within(1e-6f));
            Assert.That(profile.addedMassDiagonal.r, Is.EqualTo(3f).Within(1e-6f));
            Assert.That(profile.linearDamping.u, Is.EqualTo(10f).Within(1e-6f));
            Assert.That(profile.linearDamping.r, Is.EqualTo(120f).Within(1e-6f));
            Assert.That(profile.quadraticDamping.u, Is.EqualTo(100f).Within(1e-6f));
            Assert.That(profile.quadraticDamping.r, Is.EqualTo(150f).Within(1e-6f));
        }
        finally
        {
            Object.DestroyImmediate(profile);
        }
    }

    [Test]
    public void YawDampingCompensationInjectsSameSignTorqueAndClamps()
    {
        float positive = YawDampingCompensationTarget.ComputeCompensationTorque(
            yawRateRadPerSec: 2f,
            linearNmPerRadSec: 0.4f,
            quadraticNmPerRadSec2: 0.2f,
            deadbandRadPerSec: 0.01f,
            maxTorqueNm: 10f);

        float negative = YawDampingCompensationTarget.ComputeCompensationTorque(
            yawRateRadPerSec: -2f,
            linearNmPerRadSec: 0.4f,
            quadraticNmPerRadSec2: 0.2f,
            deadbandRadPerSec: 0.01f,
            maxTorqueNm: 10f);

        float clamped = YawDampingCompensationTarget.ComputeCompensationTorque(
            yawRateRadPerSec: 10f,
            linearNmPerRadSec: 10f,
            quadraticNmPerRadSec2: 10f,
            deadbandRadPerSec: 0.01f,
            maxTorqueNm: 3f);

        Assert.That(positive, Is.GreaterThan(0f));
        Assert.That(negative, Is.LessThan(0f));
        Assert.That(Mathf.Abs(clamped), Is.EqualTo(3f).Within(1e-6f));
    }

    [Test]
    public void YawDampingCompensationRespectsDeadband()
    {
        float torque = YawDampingCompensationTarget.ComputeCompensationTorque(
            yawRateRadPerSec: 0.01f,
            linearNmPerRadSec: 100f,
            quadraticNmPerRadSec2: 100f,
            deadbandRadPerSec: 0.02f,
            maxTorqueNm: 10f);

        Assert.That(torque, Is.EqualTo(0f).Within(1e-6f));
    }

    [Test]
    public void CompositeWaterProviderSumsFlowAndKeepsFirstHeight()
    {
        var root = new GameObject("water_root");
        var childA = new GameObject("water_a");
        var childB = new GameObject("water_b");
        try
        {
            childA.transform.SetParent(root.transform);
            childB.transform.SetParent(root.transform);

            var composite = root.AddComponent<CompositeWaterKinematicsProvider>();
            var a = childA.AddComponent<FlatWaterKinematicsProvider>();
            var b = childB.AddComponent<FlatWaterKinematicsProvider>();
            a.waterHeight = 3f;
            a.currentVelocity = new Vector3(1f, 0f, 0f);
            b.waterHeight = 5f;
            b.currentVelocity = new Vector3(0f, 0f, 2f);

            WaterKinematicsSample sample = composite.Sample(Vector3.zero);

            Assert.That(sample.Height, Is.EqualTo(3f).Within(1e-6f));
            Assert.That(sample.FlowVelocity, Is.EqualTo(new Vector3(1f, 0f, 2f)));
        }
        finally
        {
            Object.DestroyImmediate(root);
        }
    }

    static Mesh CreateUnitCubeMesh()
    {
        var mesh = new Mesh { name = "unit_cube_hydro_test" };
        mesh.vertices = new[]
        {
            new Vector3(-0.5f, -0.5f, -0.5f), new Vector3(0.5f, -0.5f, -0.5f),
            new Vector3(0.5f, 0.5f, -0.5f), new Vector3(-0.5f, 0.5f, -0.5f),
            new Vector3(-0.5f, -0.5f, 0.5f), new Vector3(0.5f, -0.5f, 0.5f),
            new Vector3(0.5f, 0.5f, 0.5f), new Vector3(-0.5f, 0.5f, 0.5f),
        };
        mesh.triangles = new[]
        {
            0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7,
            0, 1, 5, 0, 5, 4, 3, 7, 6, 3, 6, 2,
            0, 4, 7, 0, 7, 3, 1, 2, 6, 1, 6, 5,
        };
        return mesh;
    }
}
