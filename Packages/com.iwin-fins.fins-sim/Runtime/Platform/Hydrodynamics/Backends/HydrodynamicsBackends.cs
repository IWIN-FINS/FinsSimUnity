using UnityEngine;

using FinsSim.Core.Spatial;

namespace FinsSim.Hydrodynamics
{
    public abstract class HydrodynamicsBackendBase : IHydrodynamicsBackend
    {
        protected HydrodynamicsController Controller { get; private set; }

        public virtual void Initialize(HydrodynamicsController controller)
        {
            Controller = controller;
        }

        public virtual void ResetState()
        {
        }

        public abstract HydrodynamicsWrench Compute(in HydrodynamicsContext context);

        protected static HydrodynamicsWrench ComputeHydrostatic(in HydrodynamicsContext context)
        {
            HydrodynamicsProfile profile = context.Profile;
            if (profile == null || context.Body == null)
            {
                return HydrodynamicsWrench.Zero;
            }

            float submergence = profile.ComputeSubmergence(context.WorldCenterOfMass, context.WaterSample);
            if (submergence <= 0f)
            {
                return HydrodynamicsWrench.Zero;
            }

            float volume = profile.displacedVolume * submergence;
            Vector3 cobWorld = context.Transform.TransformPoint(profile.centerOfBuoyancy);
            return ComputeHydrostatic(context, volume, cobWorld);
        }

        protected static HydrodynamicsWrench ComputeHydrostatic(
            in HydrodynamicsContext context,
            float displacedVolume,
            Vector3 centerOfBuoyancyWorld)
        {
            HydrodynamicsProfile profile = context.Profile;
            if (profile == null || context.Body == null || displacedVolume <= 0f)
            {
                return HydrodynamicsWrench.Zero;
            }

            Vector3 buoyancy = Vector3.up * (profile.waterDensity * profile.gravityMagnitude * displacedVolume);
            Vector3 torque = Vector3.Cross(centerOfBuoyancyWorld - context.WorldCenterOfMass, buoyancy);
            Vector3 force = buoyancy;

            if (!context.Body.useGravity && profile.applyGravityWhenRigidbodyGravityDisabled)
            {
                Vector3 gravity = Vector3.down * (context.Body.mass * profile.gravityMagnitude);
                Vector3 cogWorld = context.Transform.TransformPoint(profile.centerOfMass);
                force += gravity;
                torque += Vector3.Cross(cogWorld - context.WorldCenterOfMass, gravity);
            }

            return new HydrodynamicsWrench
            {
                WorldForce = force,
                WorldTorque = torque,
                BodyWrench = HydroMath.UnityWorldVelocityToFossen(context.Transform, context.AxisConvention, force, torque),
                SubmergedVolume = displacedVolume,
                WettedArea = 0f,
            };
        }

        protected static HydrodynamicsWrench BodyWrenchToWorld(in HydrodynamicsContext context, SixDofVector bodyWrench)
        {
            HydroMath.FossenBodyWrenchToUnityWorld(
                context.Transform,
                context.AxisConvention,
                bodyWrench,
                out Vector3 force,
                out Vector3 torque);
            return new HydrodynamicsWrench
            {
                WorldForce = force,
                WorldTorque = torque,
                BodyWrench = bodyWrench,
                SubmergedVolume = 0f,
                WettedArea = 0f,
            };
        }
    }

    public sealed class HydrostaticOnlyBackend : HydrodynamicsBackendBase
    {
        public override HydrodynamicsWrench Compute(in HydrodynamicsContext context)
        {
            return ComputeHydrostatic(context);
        }
    }

    /// <summary>
    /// Water model used by the Learning to Swim IsaacLab task.
    ///
    /// The vehicle is treated as a fully submerged, inertia-equivalent box.
    /// It contributes hydrostatic buoyancy, quadratic box drag and linear
    /// Stokes drag only.  In particular, it intentionally does not reuse the
    /// Fossen added-mass, Coriolis, or profile damping terms.
    /// </summary>
    public sealed class LearningToSwimEquivalentBoxBackend : HydrodynamicsBackendBase
    {
        public override HydrodynamicsWrench Compute(in HydrodynamicsContext context)
        {
            HydrodynamicsProfile profile = context.Profile;
            if (profile == null || context.Body == null)
            {
                return HydrodynamicsWrench.Zero;
            }

            // The Isaac environment models an always-submerged vehicle.  Do
            // not attenuate this force at Unity's visible water surface.
            Vector3 cobWorld = context.Transform.TransformPoint(profile.centerOfBuoyancy);
            HydrodynamicsWrench hydrostatic = ComputeHydrostatic(
                context,
                Mathf.Max(0f, profile.displacedVolume),
                cobWorld);

            Vector3 halfDimensions = InferHalfDimensions(profile);
            SixDofVector relativeVelocity = context.RelativeVelocity;
            Vector3 linearVelocity = relativeVelocity.Linear;
            Vector3 angularVelocity = relativeVelocity.Angular;

            Vector3 rj = new Vector3(halfDimensions.z, halfDimensions.x, halfDimensions.y);
            Vector3 rk = new Vector3(halfDimensions.y, halfDimensions.z, halfDimensions.x);
            float waterDensity = Mathf.Max(0f, profile.waterDensity);
            float dynamicViscosity = Mathf.Max(0f, profile.equivalentBoxWaterDynamicViscosity);

            Vector3 quadraticForce = -2f * waterDensity * Vector3.Scale(
                Vector3.Scale(rj, rk),
                Vector3.Scale(Abs(linearVelocity), linearVelocity));
            Vector3 quadraticTorque = -0.5f * waterDensity * Vector3.Scale(
                Vector3.Scale(halfDimensions, FourthPower(rj) + FourthPower(rk)),
                Vector3.Scale(Abs(angularVelocity), angularVelocity));

            float equivalentRadius = (halfDimensions.x + halfDimensions.y + halfDimensions.z) / 3f;
            Vector3 viscousForce = -6f * dynamicViscosity * Mathf.PI * equivalentRadius * linearVelocity;
            Vector3 viscousTorque = -8f * dynamicViscosity * Mathf.PI * equivalentRadius * equivalentRadius * equivalentRadius * angularVelocity;

            SixDofVector drag = new SixDofVector(
                quadraticForce.x + viscousForce.x,
                quadraticForce.y + viscousForce.y,
                quadraticForce.z + viscousForce.z,
                quadraticTorque.x + viscousTorque.x,
                quadraticTorque.y + viscousTorque.y,
                quadraticTorque.z + viscousTorque.z);
            return hydrostatic + BodyWrenchToWorld(context, drag);
        }

        static Vector3 InferHalfDimensions(HydrodynamicsProfile profile)
        {
            float mass = Mathf.Max(1e-6f, profile.equivalentBoxMass);
            Vector3 inertia = new Vector3(
                Mathf.Max(0f, profile.equivalentBoxInertia.x),
                Mathf.Max(0f, profile.equivalentBoxInertia.y),
                Mathf.Max(0f, profile.equivalentBoxInertia.z));
            float scale = 3f / (2f * mass);
            return new Vector3(
                Mathf.Sqrt(Mathf.Max(0f, scale * (inertia.y + inertia.z - inertia.x))),
                Mathf.Sqrt(Mathf.Max(0f, scale * (inertia.z + inertia.x - inertia.y))),
                Mathf.Sqrt(Mathf.Max(0f, scale * (inertia.x + inertia.y - inertia.z))));
        }

        static Vector3 Abs(Vector3 value)
        {
            return new Vector3(Mathf.Abs(value.x), Mathf.Abs(value.y), Mathf.Abs(value.z));
        }

        static Vector3 FourthPower(Vector3 value)
        {
            return new Vector3(
                value.x * value.x * value.x * value.x,
                value.y * value.y * value.y * value.y,
                value.z * value.z * value.z * value.z);
        }
    }

    public class EmpiricalDrag6DofBackend : HydrodynamicsBackendBase
    {
        public override HydrodynamicsWrench Compute(in HydrodynamicsContext context)
        {
            HydrodynamicsProfile profile = context.Profile;
            if (profile == null)
            {
                return HydrodynamicsWrench.Zero;
            }

            SixDofVector damping = HydroMath.ComputeDiagonalDamping(
                profile.linearDamping,
                profile.quadraticDamping,
                profile.forwardSpeedDamping,
                context.RelativeVelocity);
            return ComputeHydrostatic(context) + BodyWrenchToWorld(context, damping);
        }
    }

    public sealed class Fossen6DofBackend : EmpiricalDrag6DofBackend
    {
        SixDofVector _previousRelativeVelocity;
        SixDofVector _filteredRelativeAcceleration;
        bool _hasPreviousVelocity;

        public Fossen6DofDiagnostics LastDiagnostics { get; private set; }

        public override void ResetState()
        {
            _previousRelativeVelocity = SixDofVector.Zero;
            _filteredRelativeAcceleration = SixDofVector.Zero;
            _hasPreviousVelocity = false;
            LastDiagnostics = Fossen6DofDiagnostics.Zero;
        }

        public override HydrodynamicsWrench Compute(in HydrodynamicsContext context)
        {
            HydrodynamicsProfile profile = context.Profile;
            if (profile == null)
            {
                return HydrodynamicsWrench.Zero;
            }

            HydrodynamicsWrench hydrostatic = ComputeHydrostatic(context);
            SixDofVector damping = HydroMath.ComputeDiagonalDamping(
                profile.linearDamping,
                profile.quadraticDamping,
                profile.forwardSpeedDamping,
                context.RelativeVelocity);

            SixDofMatrix addedMass = profile.GetPositiveAddedMassMatrix();
            SixDofVector acceleration = EstimateRelativeAcceleration(context, profile);
            SixDofVector addedMassAcceleration = SixDofVector.Zero;
            SixDofVector addedMassCoriolis = SixDofVector.Zero;

            if (profile.enableAddedMassForce)
            {
                addedMassAcceleration = -addedMass.Multiply(acceleration);
            }

            if (profile.enableAddedMassCoriolis)
            {
                addedMassCoriolis = -HydroMath.BuildFullAddedMassCoriolisProduct(addedMass, context.RelativeVelocity);
            }

            SixDofVector dynamic = damping + addedMassAcceleration + addedMassCoriolis;
            LastDiagnostics = new Fossen6DofDiagnostics
            {
                HydrostaticWrench = hydrostatic,
                DampingBodyWrench = damping,
                AddedMassAccelerationBodyWrench = addedMassAcceleration,
                AddedMassCoriolisBodyWrench = addedMassCoriolis,
                DynamicBodyWrench = dynamic,
                RelativeVelocity = context.RelativeVelocity,
                FilteredRelativeAcceleration = acceleration,
            };

            return hydrostatic + BodyWrenchToWorld(context, dynamic);
        }

        SixDofVector EstimateRelativeAcceleration(in HydrodynamicsContext context, HydrodynamicsProfile profile)
        {
            if (!_hasPreviousVelocity || context.DeltaTime <= 1e-6f)
            {
                _previousRelativeVelocity = context.RelativeVelocity;
                _filteredRelativeAcceleration = SixDofVector.Zero;
                _hasPreviousVelocity = true;
                return SixDofVector.Zero;
            }

            SixDofVector raw = (context.RelativeVelocity - _previousRelativeVelocity) / context.DeltaTime;
            float alpha = Mathf.Clamp01(profile.accelerationFilterAlpha);
            _filteredRelativeAcceleration = _filteredRelativeAcceleration * (1f - alpha) + raw * alpha;
            if (profile.limitExplicitAddedMassAcceleration)
            {
                _filteredRelativeAcceleration = ClampPerAxis(
                    _filteredRelativeAcceleration,
                    profile.maxExplicitAddedMassAcceleration);
            }

            _previousRelativeVelocity = context.RelativeVelocity;
            return _filteredRelativeAcceleration;
        }

        static SixDofVector ClampPerAxis(SixDofVector value, SixDofVector limits)
        {
            for (int axis = 0; axis < 6; axis++)
            {
                float limit = Mathf.Max(0f, limits[axis]);
                if (limit > 0f)
                {
                    value[axis] = Mathf.Clamp(value[axis], -limit, limit);
                }
            }

            return value;
        }
    }

    public sealed class CompositeHydrodynamicsBackend : HydrodynamicsBackendBase
    {
        readonly IHydrodynamicsBackend _primary;
        readonly IHydrodynamicsBackend _secondary;

        public CompositeHydrodynamicsBackend(IHydrodynamicsBackend primary, IHydrodynamicsBackend secondary)
        {
            _primary = primary;
            _secondary = secondary;
        }

        public override void Initialize(HydrodynamicsController controller)
        {
            base.Initialize(controller);
            _primary?.Initialize(controller);
            _secondary?.Initialize(controller);
        }

        public override void ResetState()
        {
            _primary?.ResetState();
            _secondary?.ResetState();
        }

        public override HydrodynamicsWrench Compute(in HydrodynamicsContext context)
        {
            HydrodynamicsWrench primary = _primary != null ? _primary.Compute(context) : HydrodynamicsWrench.Zero;
            HydrodynamicsWrench secondary = _secondary != null ? _secondary.Compute(context) : HydrodynamicsWrench.Zero;

            if (context.Profile != null && context.Profile.surfaceHydroMode == SurfaceHydroMode.Residual)
            {
                secondary.WorldForce *= context.Profile.surfaceResidualScale;
                secondary.WorldTorque *= context.Profile.surfaceResidualScale;
                secondary.BodyWrench *= context.Profile.surfaceResidualScale;
            }

            return primary + secondary;
        }
    }
}
