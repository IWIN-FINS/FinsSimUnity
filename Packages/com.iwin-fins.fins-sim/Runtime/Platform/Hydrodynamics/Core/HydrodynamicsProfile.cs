using System;
using UnityEngine;

using FinsSim.Core.Spatial;

namespace FinsSim.Hydrodynamics
{
    [CreateAssetMenu(menuName = "Marus/Hydrodynamics/Profile", fileName = "HydrodynamicsProfile")]
    public class HydrodynamicsProfile : ScriptableObject
    {
        [Header("Fluid")]
        [Min(0f)] public float waterDensity = 1027f;
        [Min(0f)] public float displacedVolume = 0.03f;
        [Min(0f)] public float gravityMagnitude = 9.81f;
        public bool assumeFullySubmerged = true;
        [Min(0.001f)] public float surfaceTransitionHalfHeight = 0.5f;

        [Header("Centers (Unity local frame)")]
        public bool applyCenterOfMassToRigidbody = true;
        public Vector3 centerOfMass = Vector3.zero;
        public Vector3 centerOfBuoyancy = new Vector3(0f, 0.03f, 0f);
        public bool applyGravityWhenRigidbodyGravityDisabled = true;

        [Header("Learning-to-Swim Equivalent Box")]
        [Tooltip("Dynamic viscosity beta [Pa s] used by LearningToSwimEquivalentBox. The reference Isaac task uses 0.001306.")]
        [Min(0f)] public float equivalentBoxWaterDynamicViscosity = 0.001306f;
        [Tooltip("Mass [kg] used to infer the equivalent rectangular body's half-dimensions. This is deliberately independent of the Rigidbody mass so it matches the IsaacLab task configuration.")]
        [Min(0.0001f)] public float equivalentBoxMass = 12.11f;
        [Tooltip("Principal inertia [kg m^2] in the configured Fossen body frame: [surge-roll, sway-pitch, heave-yaw].")]
        public Vector3 equivalentBoxInertia = new Vector3(0.13154832f, 0.23185866f, 0.24651341f);

        [Header("6DOF Damping [u, v, w, p, q, r]")]
        public SixDofVector linearDamping = new SixDofVector(20f, 30f, 40f, 1f, 1f, 4f);
        public SixDofVector quadraticDamping = new SixDofVector(40f, 80f, 100f, 2f, 2f, 8f);
        public SixDofVector forwardSpeedDamping = SixDofVector.Zero;

        [Header("Added Mass [u, v, w, p, q, r]")]
        public bool useFullAddedMassMatrix;
        public SixDofVector addedMassDiagonal = new SixDofVector(5f, 12f, 14f, 0.1f, 0.1f, 0.2f);
        public SixDofMatrix addedMassFull = SixDofMatrix.Zero;
        public bool enableAddedMassForce = true;
        public bool enableAddedMassCoriolis = true;
        [Range(0f, 1f)] public float accelerationFilterAlpha = 0.3f;
        [Header("Explicit Added-Mass Stabilization")]
        [Tooltip("Caps only the filtered finite-difference acceleration used by the explicit -M_A * nu_dot term. Each positive entry is a per-axis limit; zero leaves that axis unlimited. This does not change damping, hydrostatics, propulsion, or the added-mass Coriolis term.")]
        public bool limitExplicitAddedMassAcceleration;
        [Tooltip("Per-axis absolute acceleration limits in [m/s^2, m/s^2, m/s^2, rad/s^2, rad/s^2, rad/s^2] for [u, v, w, p, q, r].")]
        public SixDofVector maxExplicitAddedMassAcceleration = SixDofVector.Zero;

        [Header("Surface Geometry Hydro")]
        public Mesh surfaceHydroMesh;
        public SurfaceHydroMode surfaceHydroMode = SurfaceHydroMode.Residual;
        [Tooltip("Use the processed DWP2 simulation mesh on surfaceHydroMeshFilter when available. This is normally a better closed physics hull than the render mesh.")]
        public bool preferDwp2SimulationMesh = true;
        [Tooltip("Use the closed physics mesh volume and centroid for mesh-backend buoyancy. Disable only when the mesh is open or its volume is known to be invalid.")]
        public bool useMeshVolumeForBuoyancy = true;
        [Tooltip("Scales mesh-derived displaced volume. Keep at 1 for a physical hull; use mass / (rho * mesh volume) only to correct a deliberately simplified hull.")]
        [Min(0f)] public float meshBuoyancyVolumeScale = 1f;
        [Min(1)] public int maxSurfaceTriangles = 2500;
        [Tooltip("Legacy compatibility setting. The Stonefish-style mesh backend always clips triangles at the water surface.")]
        public bool clipTrianglesAtWaterSurface = true;
        [Tooltip("Legacy name retained for existing profiles. It now scales mesh-derived displaced volume rather than per-face pressure.")]
        [Min(0f)] public float pressureBuoyancyScale = 1f;
        [Min(0f)] public float formDragCoefficient = 1.0f;
        [Min(0f)] public float skinDragCoefficient = 0.02f;
        [Tooltip("Body-axis multiplier for form drag force: surge, sway, heave.")]
        public Vector3 formDragAxisScale = Vector3.one;
        [Tooltip("Body-axis multiplier for form drag torque: roll, pitch, yaw.")]
        public Vector3 formDragTorqueAxisScale = Vector3.one;
        [Tooltip("Body-axis multiplier for skin-friction force: surge, sway, heave.")]
        public Vector3 skinDragAxisScale = Vector3.one;
        [Tooltip("Body-axis multiplier for skin-friction torque: roll, pitch, yaw.")]
        public Vector3 skinDragTorqueAxisScale = Vector3.one;
        [Min(0f)] public float surfaceResidualScale = 0.25f;
        [Min(0f)] public float minTriangleArea = 1e-6f;

        [Header("Safety Clamps")]
        [Min(0f)] public float maxForceMagnitude = 10000f;
        [Min(0f)] public float maxTorqueMagnitude = 10000f;

        public HydrodynamicsProfile CloneRuntimeProfile()
        {
            var clone = Instantiate(this);
            clone.name = $"{name}_Runtime";
            if (addedMassFull != null)
            {
                clone.addedMassFull = addedMassFull.Clone();
            }

            return clone;
        }

        public float ComputeSubmergence(Vector3 worldPosition, WaterKinematicsSample waterSample)
        {
            if (assumeFullySubmerged)
            {
                return 1f;
            }

            float halfHeight = Mathf.Max(0.001f, surfaceTransitionHalfHeight);
            return Mathf.Clamp01((waterSample.Height - worldPosition.y) / (2f * halfHeight) + 0.5f);
        }

        public SixDofMatrix GetPositiveAddedMassMatrix()
        {
            if (useFullAddedMassMatrix && addedMassFull != null)
            {
                return addedMassFull.Abs();
            }

            return SixDofMatrix.Diagonal(addedMassDiagonal.Abs());
        }

        public void ScaleHydrodynamicCoefficients(
            float addedMassScale,
            float linearDampingScale,
            float quadraticDampingScale)
        {
            addedMassDiagonal *= Mathf.Max(0f, addedMassScale);
            linearDamping *= Mathf.Max(0f, linearDampingScale);
            quadraticDamping *= Mathf.Max(0f, quadraticDampingScale);

            if (addedMassFull != null)
            {
                for (int row = 0; row < 6; row++)
                {
                    for (int column = 0; column < 6; column++)
                    {
                        addedMassFull[row, column] *= Mathf.Max(0f, addedMassScale);
                    }
                }
            }
        }

        public void ScaleHydrodynamicAxisCoefficients(
            SixDofVector addedMassScale,
            SixDofVector linearDampingScale,
            SixDofVector quadraticDampingScale)
        {
            SixDofVector addedMass = ClampNonNegative(addedMassScale);
            SixDofVector linear = ClampNonNegative(linearDampingScale);
            SixDofVector quadratic = ClampNonNegative(quadraticDampingScale);

            addedMassDiagonal = SixDofVector.Scale(addedMassDiagonal, addedMass);
            linearDamping = SixDofVector.Scale(linearDamping, linear);
            quadraticDamping = SixDofVector.Scale(quadraticDamping, quadratic);

            if (addedMassFull != null)
            {
                for (int row = 0; row < 6; row++)
                {
                    for (int column = 0; column < 6; column++)
                    {
                        float rowScale = addedMass[row];
                        float columnScale = addedMass[column];
                        addedMassFull[row, column] *= Mathf.Sqrt(Mathf.Max(0f, rowScale * columnScale));
                    }
                }
            }
        }

        public void ApplyGazeboSnameDiagonal(
            GazeboSnameHydrodynamicsCoefficients coefficients,
            bool overwriteAddedMass = true,
            bool overwriteDamping = true)
        {
            if (coefficients == null)
            {
                return;
            }

            if (overwriteAddedMass)
            {
                addedMassDiagonal = new SixDofVector(
                    coefficients.xDotU,
                    coefficients.yDotV,
                    coefficients.zDotW,
                    coefficients.kDotP,
                    coefficients.mDotQ,
                    coefficients.nDotR).Abs();
                useFullAddedMassMatrix = false;
            }

            if (overwriteDamping)
            {
                linearDamping = new SixDofVector(
                    coefficients.xU,
                    coefficients.yV,
                    coefficients.zW,
                    coefficients.kP,
                    coefficients.mQ,
                    coefficients.nR).Abs();
                quadraticDamping = new SixDofVector(
                    coefficients.xUabsU,
                    coefficients.yVabsV,
                    coefficients.zWabsW,
                    coefficients.kPabsP,
                    coefficients.mQabsQ,
                    coefficients.nRabsR).Abs();
            }
        }

        static SixDofVector ClampNonNegative(SixDofVector value)
        {
            return new SixDofVector(
                Mathf.Max(0f, value.u),
                Mathf.Max(0f, value.v),
                Mathf.Max(0f, value.w),
                Mathf.Max(0f, value.p),
                Mathf.Max(0f, value.q),
                Mathf.Max(0f, value.r));
        }
    }

    [Serializable]
    public class GazeboSnameHydrodynamicsCoefficients
    {
        public float xDotU;
        public float yDotV;
        public float zDotW;
        public float kDotP;
        public float mDotQ;
        public float nDotR;

        public float xU;
        public float yV;
        public float zW;
        public float kP;
        public float mQ;
        public float nR;

        public float xUabsU;
        public float yVabsV;
        public float zWabsW;
        public float kPabsP;
        public float mQabsQ;
        public float nRabsR;
    }
}
