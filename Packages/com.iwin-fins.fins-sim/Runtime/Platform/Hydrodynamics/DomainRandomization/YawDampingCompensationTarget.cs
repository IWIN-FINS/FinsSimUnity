using UnityEngine;

using FinsSim.Core.Spatial;

namespace FinsSim.Hydrodynamics
{
    [DisallowMultipleComponent]
    [RequireComponent(typeof(Rigidbody))]
    public sealed class YawDampingCompensationTarget : MonoBehaviour, IEpisodeRandomizable
    {
        [Header("References")]
        public Rigidbody targetRigidbody;
        public Transform bodyFrame;
        [Tooltip("Optional MonoBehaviour implementing IWaterKinematicsProvider. Used only to subtract angular water flow if available.")]
        public MonoBehaviour waterProviderBehaviour;
        public BodyAxisConvention bodyAxisConvention = BodyAxisConvention.FinsRovXForwardYUpZLeft;

        [Header("Yaw Compensation")]
        [Tooltip("Anti-damping compensation torque: tau_r = linear * r + quadratic * |r| * r. Positive values reduce excessive yaw damping.")]
        [Min(0f)] public float linearCompensationNmPerRadSec;
        [Tooltip("Anti-damping compensation torque: tau_r = linear * r + quadratic * |r| * r. Positive values reduce excessive yaw damping.")]
        [Min(0f)] public float quadraticCompensationNmPerRadSec2;
        [Min(0f)] public float yawRateDeadbandRadPerSec = 0.02f;
        [Tooltip("Hard safety cap for the injected yaw torque. Set conservatively.")]
        [Min(0f)] public float maxCompensationTorqueNm = 0.5f;
        public bool compensateRelativeToWaterAngularFlow = true;

        [Header("Domain Randomization")]
        public bool randomizeInTrainMode = true;
        public bool randomizeInEvaluateMode;
        public FloatRange linearCompensationScale = new FloatRange(0.0f, 1.0f);
        public FloatRange quadraticCompensationScale = new FloatRange(0.0f, 1.0f);
        public FloatRange torqueLimitScale = new FloatRange(0.8f, 1.2f);

        [Header("Runtime Debug")]
        [SerializeField] float activeLinearScale = 1f;
        [SerializeField] float activeQuadraticScale = 1f;
        [SerializeField] float activeTorqueLimitScale = 1f;
        [SerializeField] float lastYawRateRadPerSec;
        [SerializeField] float lastBodyYawTorqueNm;
        [SerializeField] Vector3 lastWorldTorqueNm;

        IWaterKinematicsProvider _waterProvider;

        void Reset()
        {
            ResolveReferences();
        }

        void Awake()
        {
            ResolveReferences();
        }

        void OnValidate()
        {
            ResolveReferences();
            linearCompensationNmPerRadSec = Mathf.Max(0f, linearCompensationNmPerRadSec);
            quadraticCompensationNmPerRadSec2 = Mathf.Max(0f, quadraticCompensationNmPerRadSec2);
            yawRateDeadbandRadPerSec = Mathf.Max(0f, yawRateDeadbandRadPerSec);
            maxCompensationTorqueNm = Mathf.Max(0f, maxCompensationTorqueNm);
        }

        public void ResolveReferences()
        {
            if (targetRigidbody == null)
            {
                targetRigidbody = GetComponent<Rigidbody>();
            }

            if (bodyFrame == null)
            {
                bodyFrame = transform;
            }

            _waterProvider = waterProviderBehaviour as IWaterKinematicsProvider;
            if (_waterProvider == null)
            {
                foreach (MonoBehaviour behaviour in GetComponentsInChildren<MonoBehaviour>(true))
                {
                    if (behaviour is IWaterKinematicsProvider provider)
                    {
                        waterProviderBehaviour = behaviour;
                        _waterProvider = provider;
                        break;
                    }
                }
            }
        }

        public void RandomizeForEpisode(RandomizationContext context)
        {
            bool shouldRandomize =
                context.Mode == DomainRandomizationMode.Train && randomizeInTrainMode ||
                context.Mode == DomainRandomizationMode.Evaluate && randomizeInEvaluateMode;

            if (!shouldRandomize)
            {
                activeLinearScale = 1f;
                activeQuadraticScale = 1f;
                activeTorqueLimitScale = 1f;
                return;
            }

            activeLinearScale = context.Range(linearCompensationScale);
            activeQuadraticScale = context.Range(quadraticCompensationScale);
            activeTorqueLimitScale = context.Range(torqueLimitScale);
        }

        void FixedUpdate()
        {
            if (targetRigidbody == null)
            {
                targetRigidbody = GetComponent<Rigidbody>();
            }

            if (targetRigidbody == null)
            {
                return;
            }

            Vector3 worldAngularVelocity = targetRigidbody.angularVelocity;
            if (compensateRelativeToWaterAngularFlow && _waterProvider != null)
            {
                WaterKinematicsSample sample = _waterProvider.Sample(targetRigidbody.worldCenterOfMass);
                if (sample.IsValid)
                {
                    worldAngularVelocity -= sample.AngularFlowVelocity;
                }
            }

            SixDofVector bodyVelocity = HydroMath.UnityWorldVelocityToFossen(
                bodyFrame != null ? bodyFrame : transform,
                bodyAxisConvention,
                Vector3.zero,
                worldAngularVelocity);

            float yawRate = bodyVelocity.r;
            float torque = ComputeCompensationTorque(
                yawRate,
                linearCompensationNmPerRadSec * activeLinearScale,
                quadraticCompensationNmPerRadSec2 * activeQuadraticScale,
                yawRateDeadbandRadPerSec,
                maxCompensationTorqueNm * activeTorqueLimitScale);

            lastYawRateRadPerSec = yawRate;
            lastBodyYawTorqueNm = torque;

            if (Mathf.Abs(torque) <= 1e-6f)
            {
                lastWorldTorqueNm = Vector3.zero;
                return;
            }

            var bodyWrench = new SixDofVector(0f, 0f, 0f, 0f, 0f, torque);
            HydroMath.FossenBodyWrenchToUnityWorld(
                bodyFrame != null ? bodyFrame : transform,
                bodyAxisConvention,
                bodyWrench,
                out _,
                out Vector3 worldTorque);

            lastWorldTorqueNm = worldTorque;
            targetRigidbody.AddTorque(worldTorque, ForceMode.Force);
        }

        public static float ComputeCompensationTorque(
            float yawRateRadPerSec,
            float linearNmPerRadSec,
            float quadraticNmPerRadSec2,
            float deadbandRadPerSec,
            float maxTorqueNm)
        {
            if (Mathf.Abs(yawRateRadPerSec) <= Mathf.Max(0f, deadbandRadPerSec))
            {
                return 0f;
            }

            float torque =
                Mathf.Max(0f, linearNmPerRadSec) * yawRateRadPerSec +
                Mathf.Max(0f, quadraticNmPerRadSec2) * Mathf.Abs(yawRateRadPerSec) * yawRateRadPerSec;

            if (maxTorqueNm > 0f)
            {
                torque = Mathf.Clamp(torque, -maxTorqueNm, maxTorqueNm);
            }

            return HydroMath.ClampFinite(torque);
        }
    }
}
