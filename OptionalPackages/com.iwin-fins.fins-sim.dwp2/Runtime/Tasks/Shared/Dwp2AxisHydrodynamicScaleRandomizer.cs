using System.Collections.Generic;
using FinsSim.Hydrodynamics;
using NWH.DWP2.WaterObjects;
using UnityEngine;

[DisallowMultipleComponent]
public sealed class Dwp2AxisHydrodynamicScaleRandomizer : MonoBehaviour, IEpisodeRandomizable
{
    [Header("Targets")]
    [Tooltip("Root used to find DWP2 WaterObject components. If empty, this GameObject is used.")]
    public Transform targetRoot;
    public bool includeInactiveWaterObjects = true;
    public bool autoRefreshTargets = true;
    public List<WaterObject> waterObjects = new List<WaterObject>();

    [Header("Baseline")]
    public bool enableAxisScalingOnTargets = true;
    public HydrodynamicAxisConvention axisConvention = HydrodynamicAxisConvention.FinsRovXForwardYUpZLeft;
    public HydrodynamicYawDampingMode yawDampingMode = HydrodynamicYawDampingMode.MatchRealYawDampingCurve;
    [Tooltip("Baseline body force scales. FinsROV convention: x=surge, y=sway, z=heave.")]
    public Vector3 baselineForceAxisScale = Vector3.one;
    [Tooltip("Baseline body torque scales. FinsROV convention: x=roll, y=pitch, z=yaw.")]
    public Vector3 baselineTorqueAxisScale = new Vector3(1f, 1f, 0.25f);
    [Min(0f)] public float yawLinearDamping = 0.11116476f;
    [Min(0f)] public float yawQuadraticDamping = 0.17463507f;
    [Range(0f, 1f)] public float yawDampingBlend = 1f;
    [Min(0f)] public float yawRateDeadbandRadPerSec = 0.02f;

    [Header("Randomized Axes")]
    public bool randomizeSurge;
    public bool randomizeSway;
    public bool randomizeHeave;
    public bool randomizeRoll;
    public bool randomizePitch;
    public bool randomizeYaw = true;

    [Header("Force Scale Ranges")]
    public FloatRange surgeScaleRange = new FloatRange(1f, 1f);
    public FloatRange swayScaleRange = new FloatRange(1f, 1f);
    public FloatRange heaveScaleRange = new FloatRange(1f, 1f);

    [Header("Torque Scale Ranges")]
    public FloatRange rollScaleRange = new FloatRange(1f, 1f);
    public FloatRange pitchScaleRange = new FloatRange(1f, 1f);
    public FloatRange yawScaleRange = new FloatRange(0.18f, 0.40f);

    [Header("Randomization")]
    [Tooltip("When enabled, writes the baseline scale/damping values to all target WaterObjects during Awake. Keep disabled for diagnostic scenes that should use WaterObject values directly.")]
    public bool applyBaselineOnAwake;
    public bool randomizeOnStart;
    public bool randomizePerEpisode;

    [Header("Debug")]
    public bool logRandomizedValue;
    [SerializeField] Vector3 lastForceAxisScale = Vector3.one;
    [SerializeField] Vector3 lastTorqueAxisScale = new Vector3(1f, 1f, 0.25f);
    [SerializeField] int lastTargetCount;

    public Vector3 LastForceAxisScale => lastForceAxisScale;
    public Vector3 LastTorqueAxisScale => lastTorqueAxisScale;
    public int LastTargetCount => lastTargetCount;

    void Reset()
    {
        targetRoot = transform;
        ResolveTargets();
    }

    void Awake()
    {
        ResolveTargets();
        if (applyBaselineOnAwake)
        {
            ApplyScales(baselineForceAxisScale, baselineTorqueAxisScale);
        }
    }

    void Start()
    {
        if (randomizeOnStart)
        {
            RandomizeNow();
        }
    }

    void OnValidate()
    {
        baselineForceAxisScale = ClampNonNegative(baselineForceAxisScale);
        baselineTorqueAxisScale = ClampNonNegative(baselineTorqueAxisScale);
        yawLinearDamping = Mathf.Max(0f, yawLinearDamping);
        yawQuadraticDamping = Mathf.Max(0f, yawQuadraticDamping);
        yawDampingBlend = Mathf.Clamp01(yawDampingBlend);
        yawRateDeadbandRadPerSec = Mathf.Max(0f, yawRateDeadbandRadPerSec);
        lastForceAxisScale = ClampNonNegative(lastForceAxisScale);
        lastTorqueAxisScale = ClampNonNegative(lastTorqueAxisScale);
    }

    [ContextMenu("Resolve Targets")]
    public void ResolveTargets()
    {
        if (targetRoot == null)
        {
            targetRoot = transform;
        }

        if (!autoRefreshTargets && waterObjects.Count > 0)
        {
            RemoveMissingTargets();
            lastTargetCount = waterObjects.Count;
            return;
        }

        waterObjects.Clear();
        if (targetRoot == null)
        {
            lastTargetCount = 0;
            return;
        }

        WaterObject[] found = targetRoot.GetComponentsInChildren<WaterObject>(includeInactiveWaterObjects);
        for (int i = 0; i < found.Length; i++)
        {
            if (found[i] != null && !waterObjects.Contains(found[i]))
            {
                waterObjects.Add(found[i]);
            }
        }

        lastTargetCount = waterObjects.Count;
    }

    [ContextMenu("Apply Baseline Scales")]
    public void ApplyBaselineScales()
    {
        ApplyScales(baselineForceAxisScale, baselineTorqueAxisScale);
    }

    [ContextMenu("Randomize Now")]
    public void RandomizeNow()
    {
        ApplyRandomizedScales(null);
    }

    public void RandomizeForEpisode(RandomizationContext context)
    {
        if (!randomizePerEpisode)
        {
            return;
        }

        ApplyRandomizedScales(context);
    }

    void ApplyRandomizedScales(RandomizationContext? context)
    {
        Vector3 forceScale = baselineForceAxisScale;
        Vector3 torqueScale = baselineTorqueAxisScale;

        if (randomizeSurge) forceScale.x = Sample(surgeScaleRange, context);
        if (randomizeSway) forceScale.y = Sample(swayScaleRange, context);
        if (randomizeHeave) forceScale.z = Sample(heaveScaleRange, context);
        if (randomizeRoll) torqueScale.x = Sample(rollScaleRange, context);
        if (randomizePitch) torqueScale.y = Sample(pitchScaleRange, context);
        if (randomizeYaw) torqueScale.z = Sample(yawScaleRange, context);

        ApplyScales(forceScale, torqueScale);
    }

    void ApplyScales(Vector3 forceScale, Vector3 torqueScale)
    {
        ResolveTargets();

        forceScale = ClampNonNegative(forceScale);
        torqueScale = ClampNonNegative(torqueScale);

        for (int i = 0; i < waterObjects.Count; i++)
        {
            WaterObject waterObject = waterObjects[i];
            if (waterObject == null)
            {
                continue;
            }

            waterObject.hydrodynamicAxisScalingEnabled = enableAxisScalingOnTargets;
            waterObject.hydrodynamicAxisConvention = axisConvention;
            waterObject.hydrodynamicForceAxisScale = forceScale;
            waterObject.hydrodynamicTorqueAxisScale = torqueScale;
            waterObject.hydrodynamicYawDampingMode = yawDampingMode;
            waterObject.hydrodynamicYawLinearDamping = Mathf.Max(0f, yawLinearDamping);
            waterObject.hydrodynamicYawQuadraticDamping = Mathf.Max(0f, yawQuadraticDamping);
            waterObject.hydrodynamicYawDampingBlend = Mathf.Clamp01(yawDampingBlend);
            waterObject.hydrodynamicYawRateDeadbandRadPerSec = Mathf.Max(0f, yawRateDeadbandRadPerSec);
        }

        lastForceAxisScale = forceScale;
        lastTorqueAxisScale = torqueScale;
        lastTargetCount = waterObjects.Count;

        if (logRandomizedValue)
        {
            Debug.Log(
                $"[{nameof(Dwp2AxisHydrodynamicScaleRandomizer)}] forceScale={forceScale}, torqueScale={torqueScale}, targets={lastTargetCount}",
                this);
        }
    }

    static float Sample(FloatRange range, RandomizationContext? context)
    {
        float t = context.HasValue ? context.Value.Value() : UnityEngine.Random.value;
        return Mathf.Max(0f, range.ClampSample(t));
    }

    static Vector3 ClampNonNegative(Vector3 value)
    {
        return new Vector3(
            Mathf.Max(0f, value.x),
            Mathf.Max(0f, value.y),
            Mathf.Max(0f, value.z));
    }

    void RemoveMissingTargets()
    {
        for (int i = waterObjects.Count - 1; i >= 0; i--)
        {
            if (waterObjects[i] == null)
            {
                waterObjects.RemoveAt(i);
            }
        }
    }
}
