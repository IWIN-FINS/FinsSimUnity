using System.Collections.Generic;
using FinsSim.Hydrodynamics;
using NWH.Common.CoM;
using NWH.DWP2.WaterObjects;
using UnityEngine;

[DisallowMultipleComponent]
public sealed class Dwp2MassBuoyancyRandomizer : MonoBehaviour, IEpisodeRandomizable, IRigidbodyRandomizationOwner
{
    [Header("Targets")]
    public Rigidbody targetRigidbody;
    [Tooltip("Root used to find DWP2 WaterObject components. If empty, the target Rigidbody transform is used.")]
    public Transform waterObjectRoot;
    public bool includeInactiveWaterObjects = true;
    public bool autoRefreshTargets = true;
    public List<WaterObject> buoyancyWaterObjects = new List<WaterObject>();

    [Header("Mass Randomization")]
    [Min(0.001f)] public float minMassScale = 0.8f;
    [Min(0.001f)] public float maxMassScale = 1.2f;
    public bool randomizeOnStart;
    public bool randomizePerEpisode = true;
    public bool scaleInertiaTensorWithMass = true;

    [Header("Buoyancy Trim")]
    [Tooltip("Extra buoyancy coefficient margin above the mass scale. 0.02 means 2% light/positive buoyancy relative to the baseline trim.")]
    [Range(0f, 0.2f)] public float positiveBuoyancyMargin = 0.02f;
    [Min(0f)] public float minBuoyantForceCoefficient = 0.01f;
    [Min(0f)] public float maxBuoyantForceCoefficient = 2f;
    public bool updateVariableCenterOfMassBaseMass = true;

    [Header("Debug")]
    public bool logRandomizedValue;
    [SerializeField] float baselineMassKg;
    [SerializeField] float lastMassScale = 1f;
    [SerializeField] float lastMassKg;
    [SerializeField] float lastBuoyancyCoefficientScale = 1f;
    [SerializeField] int lastTargetCount;

    readonly Dictionary<WaterObject, float> _baselineBuoyancyCoefficients = new Dictionary<WaterObject, float>();
    Vector3 _baselineInertiaTensor = Vector3.one;
    VariableCenterOfMass _variableCenterOfMass;
    bool _capturedBaseline;

    public float LastMassScale => lastMassScale;
    public float LastMassKg => lastMassKg;
    public float LastBuoyancyCoefficientScale => lastBuoyancyCoefficientScale;
    public int LastTargetCount => lastTargetCount;

    void Reset()
    {
        targetRigidbody = GetComponent<Rigidbody>();
        waterObjectRoot = targetRigidbody != null ? targetRigidbody.transform : transform;
        ResolveTargets();
    }

    void Awake()
    {
        ResolveReferences();
        ResolveTargets();
        CaptureBaseline();
    }

    void Start()
    {
        if (randomizeOnStart)
        {
            RandomizeNow();
        }
    }

    [ContextMenu("Resolve Targets")]
    public void ResolveTargets()
    {
        ResolveReferences();

        if (!autoRefreshTargets && buoyancyWaterObjects.Count > 0)
        {
            RemoveMissingTargets();
            CaptureNewWaterObjectBaselines();
            lastTargetCount = buoyancyWaterObjects.Count;
            return;
        }

        buoyancyWaterObjects.Clear();
        Transform root = waterObjectRoot != null ? waterObjectRoot : transform;
        WaterObject[] found = root.GetComponentsInChildren<WaterObject>(includeInactiveWaterObjects);
        for (int i = 0; i < found.Length; i++)
        {
            if (found[i] != null && !buoyancyWaterObjects.Contains(found[i]))
            {
                buoyancyWaterObjects.Add(found[i]);
            }
        }

        CaptureNewWaterObjectBaselines();
        lastTargetCount = buoyancyWaterObjects.Count;
    }

    [ContextMenu("Capture Baseline")]
    public void CaptureBaseline()
    {
        ResolveReferences();

        if (targetRigidbody == null)
        {
            _capturedBaseline = false;
            baselineMassKg = 0f;
            lastMassKg = 0f;
            return;
        }

        baselineMassKg = Mathf.Max(0.001f, targetRigidbody.mass);
        _baselineInertiaTensor = targetRigidbody.inertiaTensor;
        lastMassKg = baselineMassKg;
        _capturedBaseline = true;

        CaptureNewWaterObjectBaselines();
    }

    [ContextMenu("Randomize Now")]
    public void RandomizeNow()
    {
        ApplySample(UnityEngine.Random.value);
    }

    public void RandomizeForEpisode(RandomizationContext context)
    {
        if (!randomizePerEpisode)
        {
            return;
        }

        ApplySample(context.Value());
    }

    void ApplySample(float unitSample)
    {
        ResolveTargets();

        if (!_capturedBaseline)
        {
            CaptureBaseline();
        }

        if (targetRigidbody == null || baselineMassKg <= 0f)
        {
            return;
        }

        float lower = Mathf.Min(minMassScale, maxMassScale);
        float upper = Mathf.Max(minMassScale, maxMassScale);
        float massScale = Mathf.Lerp(lower, upper, Mathf.Clamp01(unitSample));
        float massKg = Mathf.Max(0.001f, baselineMassKg * massScale);
        float buoyancyScale = massScale * (1f + Mathf.Max(0f, positiveBuoyancyMargin));

        targetRigidbody.mass = massKg;
        if (scaleInertiaTensorWithMass)
        {
            targetRigidbody.inertiaTensor = _baselineInertiaTensor * Mathf.Max(0.001f, massScale);
        }

        if (updateVariableCenterOfMassBaseMass && _variableCenterOfMass != null)
        {
            _variableCenterOfMass.baseMass = massKg;
            _variableCenterOfMass.combinedMass = massKg;
            _variableCenterOfMass.MarkDirty();
        }

        float minBfc = Mathf.Min(minBuoyantForceCoefficient, maxBuoyantForceCoefficient);
        float maxBfc = Mathf.Max(minBuoyantForceCoefficient, maxBuoyantForceCoefficient);
        for (int i = 0; i < buoyancyWaterObjects.Count; i++)
        {
            WaterObject waterObject = buoyancyWaterObjects[i];
            if (waterObject == null)
            {
                continue;
            }

            if (!_baselineBuoyancyCoefficients.TryGetValue(waterObject, out float baselineBfc))
            {
                baselineBfc = waterObject.buoyantForceCoefficient;
                _baselineBuoyancyCoefficients[waterObject] = baselineBfc;
            }

            waterObject.buoyantForceCoefficient = Mathf.Clamp(baselineBfc * buoyancyScale, minBfc, maxBfc);
        }

        lastMassScale = massScale;
        lastMassKg = massKg;
        lastBuoyancyCoefficientScale = buoyancyScale;
        lastTargetCount = buoyancyWaterObjects.Count;

        if (logRandomizedValue)
        {
            Debug.Log(
                $"[{nameof(Dwp2MassBuoyancyRandomizer)}] mass={massKg:F3} kg, massScale={massScale:F3}, buoyancyScale={buoyancyScale:F3}, targets={lastTargetCount}",
                this);
        }
    }

    void ResolveReferences()
    {
        if (targetRigidbody == null)
        {
            targetRigidbody = GetComponent<Rigidbody>();
        }

        if (waterObjectRoot == null)
        {
            waterObjectRoot = targetRigidbody != null ? targetRigidbody.transform : transform;
        }

        if (_variableCenterOfMass == null && targetRigidbody != null)
        {
            _variableCenterOfMass = targetRigidbody.GetComponent<VariableCenterOfMass>();
        }
    }

    void CaptureNewWaterObjectBaselines()
    {
        for (int i = 0; i < buoyancyWaterObjects.Count; i++)
        {
            WaterObject waterObject = buoyancyWaterObjects[i];
            if (waterObject != null && !_baselineBuoyancyCoefficients.ContainsKey(waterObject))
            {
                _baselineBuoyancyCoefficients[waterObject] = waterObject.buoyantForceCoefficient;
            }
        }
    }

    void RemoveMissingTargets()
    {
        for (int i = buoyancyWaterObjects.Count - 1; i >= 0; i--)
        {
            if (buoyancyWaterObjects[i] == null)
            {
                buoyancyWaterObjects.RemoveAt(i);
            }
        }
    }
}
