using UnityEngine;

using FinsSim.Core.Spatial;

namespace FinsSim.Hydrodynamics
{
    [DefaultExecutionOrder(20)]
    [DisallowMultipleComponent]
    [RequireComponent(typeof(Rigidbody))]
    public class HydrodynamicsController : MonoBehaviour, IEpisodeRandomizable
    {
        [Header("Mode")]
        public HydrodynamicsMode mode = HydrodynamicsMode.Fossen6Dof;
        public BodyAxisConvention bodyAxisConvention = BodyAxisConvention.LegacyHydroMathZForwardXRightYUp;
        public HydrodynamicsProfile profile;
        public bool createRuntimeDefaultProfileWhenMissing = true;

        [Header("Water")]
        [Tooltip("Optional MonoBehaviour implementing IWaterKinematicsProvider.")]
        public MonoBehaviour waterProviderBehaviour;
        public bool autoFindWaterProvider = true;
        public float fallbackWaterHeight = 0f;
        public Vector3 fallbackCurrentVelocity = Vector3.zero;

        [Header("Surface Geometry")]
        public Mesh surfaceHydroMeshOverride;
        public MeshFilter surfaceHydroMeshFilter;

        [Header("Runtime Debug")]
        [SerializeField] HydrodynamicsMode activeMode;
        [SerializeField] Vector3 lastHydroForce;
        [SerializeField] Vector3 lastHydroTorque;
        [SerializeField] Vector3 lastWaterFlow;
        [SerializeField] float lastWaterHeight;
        [SerializeField] float lastSubmergedVolume;

        Rigidbody _body;
        HydrodynamicsProfile _runtimeProfile;
        HydrodynamicsProfile _runtimeProfileSource;
        HydrodynamicsProfile _runtimeDefaultProfile;
        IWaterKinematicsProvider _waterProvider;
        IHydrodynamicsBackend _backend;
        HydrodynamicsMode _backendMode;
        bool _warnedMissingWaterProvider;

        public HydrodynamicsWrench LastWrench { get; private set; }
        public SixDofVector LastRelativeVelocity6Dof { get; private set; }
        public WaterKinematicsSample LastWaterSample { get; private set; }
        public HydrodynamicsProfile RuntimeProfile => EnsureRuntimeProfile();
        public Transform SurfaceHydroTransform => surfaceHydroMeshFilter != null ? surfaceHydroMeshFilter.transform : transform;

        public bool TryGetFossenDiagnostics(out Fossen6DofDiagnostics diagnostics)
        {
            if (_backend is Fossen6DofBackend fossenBackend)
            {
                diagnostics = fossenBackend.LastDiagnostics;
                return true;
            }

            diagnostics = Fossen6DofDiagnostics.Zero;
            return false;
        }

        void Awake()
        {
            _body = GetComponent<Rigidbody>();
            ResolveWaterProvider();
            EnsureBackend();
            ApplyProfileToRigidbody();
        }

        void OnEnable()
        {
            ResetBackendState();
        }

        void OnValidate()
        {
            activeMode = mode;
        }

        void FixedUpdate()
        {
            long finsSimProfileStart = FinsSimRuntimeProfiler.Begin();
            UnityEngine.Profiling.Profiler.BeginSample("FinsSim.HydrodynamicsController.FixedUpdate");
            try
            {
                if (_body == null)
                {
                    _body = GetComponent<Rigidbody>();
                }

                HydrodynamicsProfile activeProfile = EnsureRuntimeProfile();
                if (mode == HydrodynamicsMode.Off || activeProfile == null)
                {
                    LastWrench = HydrodynamicsWrench.Zero;
                    return;
                }

                EnsureBackend();
                ApplyProfileToRigidbody();
                WaterKinematicsSample waterSample = SampleWater(_body.worldCenterOfMass);
                Mesh surfaceMesh = ResolveSurfaceMesh(activeProfile);
                var context = new HydrodynamicsContext(
                    _body,
                    transform,
                    activeProfile,
                    _waterProvider,
                    waterSample,
                    Time.fixedDeltaTime,
                    surfaceMesh,
                    bodyAxisConvention);

                HydrodynamicsWrench wrench = _backend != null
                    ? _backend.Compute(context)
                    : HydrodynamicsWrench.Zero;
                wrench = HydroMath.ClampWrench(
                    wrench,
                    activeProfile.maxForceMagnitude,
                    activeProfile.maxTorqueMagnitude);

                if (wrench.WorldForce.sqrMagnitude > 0f)
                {
                    _body.AddForce(wrench.WorldForce, ForceMode.Force);
                }

                if (wrench.WorldTorque.sqrMagnitude > 0f)
                {
                    _body.AddTorque(wrench.WorldTorque, ForceMode.Force);
                }

                LastWrench = wrench;
                LastRelativeVelocity6Dof = context.RelativeVelocity;
                LastWaterSample = waterSample;
                lastHydroForce = wrench.WorldForce;
                lastHydroTorque = wrench.WorldTorque;
                lastWaterFlow = waterSample.FlowVelocity;
                lastWaterHeight = waterSample.Height;
                lastSubmergedVolume = wrench.SubmergedVolume;
                activeMode = mode;
            }
            finally
            {
                UnityEngine.Profiling.Profiler.EndSample();
                FinsSimRuntimeProfiler.End("FinsSim.HydrodynamicsController.FixedUpdate", finsSimProfileStart);
            }
        }

        public void ResetRuntimeProfileFromSource()
        {
            if (_runtimeProfile != null)
            {
                Destroy(_runtimeProfile);
                _runtimeProfile = null;
            }

            _runtimeProfileSource = null;
            EnsureRuntimeProfile();
            ResetBackendState();
        }

        public void ResetBackendState()
        {
            _backend?.ResetState();
        }

        public void RandomizeForEpisode(RandomizationContext context)
        {
            if (context.Profile == null)
            {
                return;
            }

            ResetRuntimeProfileFromSource();
            HydrodynamicsProfile runtime = EnsureRuntimeProfile();
            if (runtime == null)
            {
                return;
            }

            if (context.Profile.randomizeBody)
            {
                float volumeScale = context.Profile.coupleMassAndVolumeForNeutralBuoyancy
                    ? context.Range(context.Profile.massScale, "body.mass-scale") *
                      context.Range(context.Profile.buoyancyTrimScale, "body.buoyancy-trim")
                    : context.Range(context.Profile.volumeScale, "body.volume-scale");
                runtime.displacedVolume *= Mathf.Max(0f, volumeScale);
                runtime.centerOfBuoyancy += context.Range(context.Profile.centerOfBuoyancyOffset);
                runtime.centerOfMass += context.Range(context.Profile.centerOfMassOffset);
            }

            if (context.Profile.randomizeHydrodynamics)
            {
                runtime.ScaleHydrodynamicCoefficients(
                    context.Range(context.Profile.addedMassScale),
                    context.Range(context.Profile.linearDampingScale),
                    context.Range(context.Profile.quadraticDampingScale));

                if (context.Profile.randomizeHydrodynamicAxisScales)
                {
                    runtime.ScaleHydrodynamicAxisCoefficients(
                        context.Range(context.Profile.addedMassAxisScaleMin, context.Profile.addedMassAxisScaleMax),
                        context.Range(context.Profile.linearDampingAxisScaleMin, context.Profile.linearDampingAxisScaleMax),
                        context.Range(context.Profile.quadraticDampingAxisScaleMin, context.Profile.quadraticDampingAxisScaleMax));
                }
            }

            ApplyProfileToRigidbody();
            ResetBackendState();
        }

        HydrodynamicsProfile EnsureRuntimeProfile()
        {
            if (profile == null)
            {
                if (!Application.isPlaying || !createRuntimeDefaultProfileWhenMissing)
                {
                    return null;
                }

                if (_runtimeDefaultProfile == null)
                {
                    _runtimeDefaultProfile = ScriptableObject.CreateInstance<HydrodynamicsProfile>();
                    _runtimeDefaultProfile.name = "RuntimeDefaultHydrodynamicsProfile";
                }

                return _runtimeDefaultProfile;
            }

            if (!Application.isPlaying)
            {
                return profile;
            }

            if (_runtimeProfile == null || _runtimeProfileSource != profile)
            {
                if (_runtimeProfile != null)
                {
                    Destroy(_runtimeProfile);
                }

                _runtimeProfile = profile.CloneRuntimeProfile();
                _runtimeProfileSource = profile;
            }

            return _runtimeProfile;
        }

        void EnsureBackend()
        {
            if (_backend != null && _backendMode == mode)
            {
                return;
            }

            _backend = CreateBackend(mode);
            _backendMode = mode;
            _backend?.Initialize(this);
            activeMode = mode;
        }

        static IHydrodynamicsBackend CreateBackend(HydrodynamicsMode requestedMode)
        {
            switch (requestedMode)
            {
                case HydrodynamicsMode.HydrostaticOnly:
                    return new HydrostaticOnlyBackend();
                case HydrodynamicsMode.EmpiricalDrag6Dof:
                    return new EmpiricalDrag6DofBackend();
                case HydrodynamicsMode.Fossen6Dof:
                    return new Fossen6DofBackend();
                case HydrodynamicsMode.LearningToSwimEquivalentBox:
                    return new LearningToSwimEquivalentBoxBackend();
                case HydrodynamicsMode.SurfaceGeometryHydro:
                    return new SurfaceGeometryHydroBackend();
                case HydrodynamicsMode.FossenPlusSurfaceResidual:
                    // Fossen owns static buoyancy in the composite. The mesh backend contributes drag only.
                    return new CompositeHydrodynamicsBackend(new Fossen6DofBackend(), new SurfaceGeometryHydroBackend(includeHydrostatics: false));
                case HydrodynamicsMode.Off:
                default:
                    return null;
            }
        }

        void ApplyProfileToRigidbody()
        {
            HydrodynamicsProfile activeProfile = EnsureRuntimeProfile();
            if (_body == null || activeProfile == null || !activeProfile.applyCenterOfMassToRigidbody)
            {
                return;
            }

            _body.centerOfMass = activeProfile.centerOfMass;
        }

        void ResolveWaterProvider()
        {
            _waterProvider = waterProviderBehaviour as IWaterKinematicsProvider;
            if (_waterProvider != null || !autoFindWaterProvider)
            {
                return;
            }

            foreach (MonoBehaviour behaviour in FindObjectsByType<MonoBehaviour>(FindObjectsSortMode.None))
            {
                if (behaviour is IWaterKinematicsProvider provider)
                {
                    _waterProvider = provider;
                    waterProviderBehaviour = behaviour;
                    return;
                }
            }
        }

        WaterKinematicsSample SampleWater(Vector3 worldPoint)
        {
            if (_waterProvider == null)
            {
                ResolveWaterProvider();
            }

            if (_waterProvider != null)
            {
                return _waterProvider.Sample(worldPoint);
            }

            if (!_warnedMissingWaterProvider)
            {
                Debug.LogWarning(
                    $"[{nameof(HydrodynamicsController)}] No IWaterKinematicsProvider found. Using flat fallback water.",
                    this);
                _warnedMissingWaterProvider = true;
            }

            return WaterKinematicsSample.Flat(fallbackWaterHeight, fallbackCurrentVelocity);
        }

        Mesh ResolveSurfaceMesh(HydrodynamicsProfile activeProfile)
        {
            if (surfaceHydroMeshOverride != null)
            {
                return surfaceHydroMeshOverride;
            }

            if (surfaceHydroMeshFilter != null && surfaceHydroMeshFilter.sharedMesh != null)
            {
                return surfaceHydroMeshFilter.sharedMesh;
            }

            return activeProfile != null ? activeProfile.surfaceHydroMesh : null;
        }

    }
}
