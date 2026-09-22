using UnityEngine;

using FinsSim.Core.Spatial;

namespace FinsSim.Hydrodynamics
{
    public enum HydrodynamicsMode
    {
        Off,
        HydrostaticOnly,
        EmpiricalDrag6Dof,
        Fossen6Dof,
        SurfaceGeometryHydro,
        FossenPlusSurfaceResidual,
        // Append-only: existing scenes serialize this enum as an integer.
        LearningToSwimEquivalentBox
    }

    public enum SurfaceHydroMode
    {
        Standalone,
        Residual
    }

    public enum BodyAxisConvention
    {
        LegacyHydroMathZForwardXRightYUp,
        FinsRovXForwardYUpZLeft
    }

    public struct WaterKinematicsSample
    {
        public bool IsValid;
        public float Height;
        public Vector3 Normal;
        public Vector3 FlowVelocity;
        public Vector3 AngularFlowVelocity;

        public static WaterKinematicsSample Flat(float waterHeight, Vector3 flowVelocity)
        {
            return new WaterKinematicsSample
            {
                IsValid = true,
                Height = waterHeight,
                Normal = Vector3.up,
                FlowVelocity = flowVelocity,
                AngularFlowVelocity = Vector3.zero,
            };
        }
    }

    public struct HydrodynamicsWrench
    {
        public Vector3 WorldForce;
        public Vector3 WorldTorque;
        public SixDofVector BodyWrench;
        public float SubmergedVolume;
        public float WettedArea;

        public static HydrodynamicsWrench Zero => new HydrodynamicsWrench
        {
            WorldForce = Vector3.zero,
            WorldTorque = Vector3.zero,
            BodyWrench = SixDofVector.Zero,
            SubmergedVolume = 0f,
            WettedArea = 0f,
        };

        public static HydrodynamicsWrench operator +(HydrodynamicsWrench a, HydrodynamicsWrench b)
        {
            return new HydrodynamicsWrench
            {
                WorldForce = a.WorldForce + b.WorldForce,
                WorldTorque = a.WorldTorque + b.WorldTorque,
                BodyWrench = a.BodyWrench + b.BodyWrench,
                SubmergedVolume = a.SubmergedVolume + b.SubmergedVolume,
                WettedArea = a.WettedArea + b.WettedArea,
            };
        }
    }

    /// <summary>
    /// Runtime breakdown of the Fossen 6DOF wrench. Terms are expressed in the
    /// configured Fossen body frame and represent the exact contributions used
    /// for the most recent physics step.
    /// </summary>
    public struct Fossen6DofDiagnostics
    {
        public HydrodynamicsWrench HydrostaticWrench;
        public SixDofVector DampingBodyWrench;
        public SixDofVector AddedMassAccelerationBodyWrench;
        public SixDofVector AddedMassCoriolisBodyWrench;
        public SixDofVector DynamicBodyWrench;
        public SixDofVector RelativeVelocity;
        public SixDofVector FilteredRelativeAcceleration;

        public static Fossen6DofDiagnostics Zero => new Fossen6DofDiagnostics
        {
            HydrostaticWrench = HydrodynamicsWrench.Zero,
            DampingBodyWrench = SixDofVector.Zero,
            AddedMassAccelerationBodyWrench = SixDofVector.Zero,
            AddedMassCoriolisBodyWrench = SixDofVector.Zero,
            DynamicBodyWrench = SixDofVector.Zero,
            RelativeVelocity = SixDofVector.Zero,
            FilteredRelativeAcceleration = SixDofVector.Zero,
        };
    }

    public readonly struct HydrodynamicsContext
    {
        public readonly Rigidbody Body;
        public readonly Transform Transform;
        public readonly HydrodynamicsProfile Profile;
        public readonly IWaterKinematicsProvider WaterProvider;
        public readonly WaterKinematicsSample WaterSample;
        public readonly float DeltaTime;
        public readonly Vector3 WorldCenterOfMass;
        public readonly Vector3 WorldLinearVelocity;
        public readonly Vector3 WorldAngularVelocity;
        public readonly Vector3 WorldWaterVelocity;
        public readonly Vector3 WorldWaterAngularVelocity;
        public readonly SixDofVector BodyVelocity;
        public readonly SixDofVector BodyWaterVelocity;
        public readonly SixDofVector RelativeVelocity;
        public readonly Mesh SurfaceMesh;
        public readonly BodyAxisConvention AxisConvention;

        public HydrodynamicsContext(
            Rigidbody body,
            Transform transform,
            HydrodynamicsProfile profile,
            IWaterKinematicsProvider waterProvider,
            WaterKinematicsSample waterSample,
            float deltaTime,
            Mesh surfaceMesh,
            BodyAxisConvention axisConvention = BodyAxisConvention.LegacyHydroMathZForwardXRightYUp)
        {
            Body = body;
            Transform = transform;
            Profile = profile;
            WaterProvider = waterProvider;
            WaterSample = waterSample;
            DeltaTime = deltaTime;
            WorldCenterOfMass = body != null ? body.worldCenterOfMass : transform.position;
            WorldLinearVelocity = body != null ? body.linearVelocity : Vector3.zero;
            WorldAngularVelocity = body != null ? body.angularVelocity : Vector3.zero;
            WorldWaterVelocity = waterSample.FlowVelocity;
            WorldWaterAngularVelocity = waterSample.AngularFlowVelocity;
            AxisConvention = axisConvention;
            BodyVelocity = HydroMath.UnityWorldVelocityToFossen(transform, axisConvention, WorldLinearVelocity, WorldAngularVelocity);
            BodyWaterVelocity = HydroMath.UnityWorldVelocityToFossen(transform, axisConvention, WorldWaterVelocity, WorldWaterAngularVelocity);
            RelativeVelocity = HydroMath.UnityWorldVelocityToFossen(
                transform,
                axisConvention,
                WorldLinearVelocity - WorldWaterVelocity,
                WorldAngularVelocity - WorldWaterAngularVelocity);
            SurfaceMesh = surfaceMesh;
        }
    }

    public interface IHydrodynamicsBackend
    {
        void Initialize(HydrodynamicsController controller);
        void ResetState();
        HydrodynamicsWrench Compute(in HydrodynamicsContext context);
    }

    public interface IWaterKinematicsProvider
    {
        WaterKinematicsSample Sample(Vector3 worldPoint);
    }

    /// <summary>Optional area-local water-height control used by replicated training scenes.</summary>
    public interface IAreaWaterKinematicsProvider : IWaterKinematicsProvider
    {
        void SetAreaWaterHeight(float height);
    }

    public interface IEpisodeRandomizable
    {
        void RandomizeForEpisode(RandomizationContext context);
    }

    /// <summary>
    /// Marks a plugin component that owns rigidbody mass/buoyancy randomization.
    /// The public setup code uses this instead of naming a specific integration.
    /// </summary>
    public interface IRigidbodyRandomizationOwner
    {
    }
}
