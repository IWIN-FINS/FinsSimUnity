using System;
using System.Collections.Generic;
using NWH.DWP2.WaterObjects;
using UnityEngine;

using FinsSim.Core.Spatial;

namespace FinsSim.Hydrodynamics
{
    [DefaultExecutionOrder(120)]
    [DisallowMultipleComponent]
    [RequireComponent(typeof(Rigidbody))]
    [Obsolete("Use DWP2 WaterObject hydrodynamicTorqueAxisScale or Dwp2AxisHydrodynamicScaleRandomizer instead. Do not enable both.")]
    public sealed class Dwp2BodyYawTorqueScaler : MonoBehaviour, IEpisodeRandomizable
    {
        [Header("Deprecated Compatibility")]
        [Tooltip("Deprecated. Leave disabled when using DWP2 WaterObject hydrodynamicTorqueAxisScale.")]
        public bool applyDeprecatedCorrection;

        [Header("References")]
        public Rigidbody targetRigidbody;
        public Transform bodyFrame;
        public BodyAxisConvention bodyAxisConvention = BodyAxisConvention.FinsRovXForwardYUpZLeft;

        [Header("DWP2 WaterObjects")]
        public Transform waterObjectSearchRoot;
        public bool autoRefreshWaterObjects = true;
        public bool includeInactiveWaterObjects = true;
        public bool requireMatchingTargetRigidbody = true;
        public List<WaterObject> waterObjects = new List<WaterObject>();

        [Header("Yaw Torque Scaling")]
        [Min(0f)] public float yawTorqueScale = 1f;
        [Tooltip("0 disables clamping. Use this as a safety guard when randomizing very low scales with large DWP2 torques.")]
        [Min(0f)] public float maxAbsDeltaYawTorqueNm;

        [Header("Domain Randomization")]
        public DomainRandomizationMode standaloneMode = DomainRandomizationMode.Train;
        public int standaloneBaseSeed = 32017;
        public bool incrementStandaloneSeedPerEpisode = true;
        public bool randomizeOnStart = true;
        public bool randomizeInTrainMode = true;
        public bool randomizeInEvaluateMode;
        public FloatRange yawTorqueScaleRange = new FloatRange(0.3f, 1.2f);

        [Header("Runtime Debug")]
        [SerializeField] float activeYawTorqueScale = 1f;
        [SerializeField] int episodeIndex;
        [SerializeField] int lastRandomizationSeed;
        [SerializeField] int lastWaterObjectCount;
        [SerializeField] Vector3 lastOriginalWorldTorqueNm;
        [SerializeField] float lastOriginalBodyYawTorqueNm;
        [SerializeField] float lastDeltaBodyYawTorqueNm;
        [SerializeField] Vector3 lastDeltaWorldTorqueNm;

        public float ActiveYawTorqueScale => activeYawTorqueScale;
        public int EpisodeIndex => episodeIndex;
        public int LastRandomizationSeed => lastRandomizationSeed;
        public int LastWaterObjectCount => lastWaterObjectCount;
        public float LastOriginalBodyYawTorqueNm => lastOriginalBodyYawTorqueNm;
        public float LastDeltaBodyYawTorqueNm => lastDeltaBodyYawTorqueNm;

        void Reset()
        {
            ResolveReferences();
            RefreshWaterObjects();
            activeYawTorqueScale = Mathf.Max(0f, yawTorqueScale);
        }

        void Awake()
        {
            ResolveReferences();
            if (autoRefreshWaterObjects || waterObjects == null || waterObjects.Count == 0)
            {
                RefreshWaterObjects();
            }

            activeYawTorqueScale = Mathf.Max(0f, yawTorqueScale);
        }

        void Start()
        {
            if (randomizeOnStart)
            {
                RandomizeStandaloneEpisode();
            }
        }

        void OnValidate()
        {
            yawTorqueScale = Mathf.Max(0f, yawTorqueScale);
            maxAbsDeltaYawTorqueNm = Mathf.Max(0f, maxAbsDeltaYawTorqueNm);
            activeYawTorqueScale = Mathf.Max(0f, activeYawTorqueScale);
            ResolveReferences();
        }

        public void RandomizeForEpisode(RandomizationContext context)
        {
            bool shouldRandomize =
                context.Mode == DomainRandomizationMode.Train && randomizeInTrainMode ||
                context.Mode == DomainRandomizationMode.Evaluate && randomizeInEvaluateMode;

            activeYawTorqueScale = shouldRandomize
                ? Mathf.Max(0f, context.Range(yawTorqueScaleRange))
                : Mathf.Max(0f, yawTorqueScale);
        }

        [ContextMenu("Randomize Standalone Episode")]
        public void RandomizeStandaloneEpisode()
        {
            if (standaloneMode == DomainRandomizationMode.Disabled)
            {
                activeYawTorqueScale = Mathf.Max(0f, yawTorqueScale);
                return;
            }

            int seed = incrementStandaloneSeedPerEpisode
                ? standaloneBaseSeed + episodeIndex
                : standaloneBaseSeed;

            var context = new RandomizationContext(null, standaloneMode, seed, episodeIndex);
            RandomizeForEpisode(context);
            lastRandomizationSeed = seed;
            episodeIndex++;
        }

        [ContextMenu("Refresh DWP2 WaterObjects")]
        public void RefreshWaterObjects()
        {
            ResolveReferences();
            if (waterObjects == null)
            {
                waterObjects = new List<WaterObject>();
            }

            waterObjects.Clear();
            Transform root = waterObjectSearchRoot != null ? waterObjectSearchRoot : transform;
            foreach (WaterObject waterObject in root.GetComponentsInChildren<WaterObject>(includeInactiveWaterObjects))
            {
                if (waterObject != null && !waterObjects.Contains(waterObject))
                {
                    waterObjects.Add(waterObject);
                }
            }
        }

        void FixedUpdate()
        {
            if (!applyDeprecatedCorrection)
            {
                ClearRuntimeDebug();
                return;
            }

            ResolveReferences();
            if (targetRigidbody == null)
            {
                ClearRuntimeDebug();
                return;
            }

            if (autoRefreshWaterObjects && (waterObjects == null || waterObjects.Count == 0))
            {
                RefreshWaterObjects();
            }

            Vector3 originalWorldTorque = SumDwp2WorldTorque(out int contributingWaterObjects);
            lastWaterObjectCount = contributingWaterObjects;
            lastOriginalWorldTorqueNm = originalWorldTorque;

            if (contributingWaterObjects == 0)
            {
                lastOriginalBodyYawTorqueNm = 0f;
                lastDeltaBodyYawTorqueNm = 0f;
                lastDeltaWorldTorqueNm = Vector3.zero;
                return;
            }

            Vector3 deltaWorldTorque = ComputeScaledYawDeltaWorldTorque(
                bodyFrame != null ? bodyFrame : transform,
                bodyAxisConvention,
                originalWorldTorque,
                activeYawTorqueScale,
                maxAbsDeltaYawTorqueNm,
                out float originalBodyYawTorque,
                out float deltaBodyYawTorque);

            lastOriginalBodyYawTorqueNm = originalBodyYawTorque;
            lastDeltaBodyYawTorqueNm = deltaBodyYawTorque;
            lastDeltaWorldTorqueNm = deltaWorldTorque;

            if (deltaWorldTorque.sqrMagnitude > 1e-12f)
            {
                targetRigidbody.AddTorque(deltaWorldTorque, ForceMode.Force);
            }
        }

        public static Vector3 ComputeScaledYawDeltaWorldTorque(
            Transform bodyFrame,
            BodyAxisConvention axisConvention,
            Vector3 originalWorldTorque,
            float yawScale,
            float maxAbsDeltaYawTorqueNm,
            out float originalBodyYawTorque,
            out float deltaBodyYawTorque)
        {
            SixDofVector bodyTorque = HydroMath.UnityWorldVelocityToFossen(
                bodyFrame,
                axisConvention,
                Vector3.zero,
                originalWorldTorque);

            originalBodyYawTorque = bodyTorque.r;
            deltaBodyYawTorque = (Mathf.Max(0f, yawScale) - 1f) * originalBodyYawTorque;
            if (maxAbsDeltaYawTorqueNm > 0f)
            {
                deltaBodyYawTorque = Mathf.Clamp(
                    deltaBodyYawTorque,
                    -maxAbsDeltaYawTorqueNm,
                    maxAbsDeltaYawTorqueNm);
            }

            var deltaBodyWrench = new SixDofVector(0f, 0f, 0f, 0f, 0f, deltaBodyYawTorque);
            HydroMath.FossenBodyWrenchToUnityWorld(
                bodyFrame,
                axisConvention,
                deltaBodyWrench,
                out _,
                out Vector3 deltaWorldTorque);

            return HydroMath.ClampFinite(deltaWorldTorque);
        }

        void ResolveReferences()
        {
            if (targetRigidbody == null)
            {
                targetRigidbody = GetComponent<Rigidbody>();
            }

            if (bodyFrame == null)
            {
                bodyFrame = transform;
            }
        }

        Vector3 SumDwp2WorldTorque(out int contributingWaterObjects)
        {
            contributingWaterObjects = 0;
            Vector3 total = Vector3.zero;
            if (waterObjects == null)
            {
                return total;
            }

            for (int i = 0; i < waterObjects.Count; i++)
            {
                WaterObject waterObject = waterObjects[i];
                if (waterObject == null || !waterObject.isActiveAndEnabled)
                {
                    continue;
                }

                if (requireMatchingTargetRigidbody &&
                    waterObject.targetRigidbody != null &&
                    targetRigidbody != null &&
                    waterObject.targetRigidbody != targetRigidbody)
                {
                    continue;
                }

                total += waterObject.ResultTorque;
                contributingWaterObjects++;
            }

            return HydroMath.ClampFinite(total);
        }

        void ClearRuntimeDebug()
        {
            lastWaterObjectCount = 0;
            lastOriginalWorldTorqueNm = Vector3.zero;
            lastOriginalBodyYawTorqueNm = 0f;
            lastDeltaBodyYawTorqueNm = 0f;
            lastDeltaWorldTorqueNm = Vector3.zero;
        }
    }
}
