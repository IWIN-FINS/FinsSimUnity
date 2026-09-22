using System.Collections.Generic;
using FinsSim.Hydrodynamics;
using NWH.DWP2.WaterObjects;
using UnityEngine;

[DisallowMultipleComponent]
public sealed class Dwp2HydrodynamicForceCoefficientRandomizer : MonoBehaviour, IEpisodeRandomizable
{
    [Header("Targets")]
    [Tooltip("Root used to find DWP2 WaterObject components. If empty, this GameObject is used.")]
    public Transform targetRoot;
    public bool includeInactiveWaterObjects = true;
    public bool autoRefreshTargets = true;
    public List<WaterObject> waterObjects = new List<WaterObject>();

    [Header("Randomization")]
    [Min(0f)] public float minHydrodynamicForceCoefficient = 0.1f;
    [Min(0f)] public float maxHydrodynamicForceCoefficient = 0.52f;
    public bool randomizeOnStart;
    public bool randomizePerEpisode = true;

    [Header("Debug")]
    public bool logRandomizedValue;
    [SerializeField] float lastHydrodynamicForceCoefficient = 1f;
    [SerializeField] int lastTargetCount;

    public float LastHydrodynamicForceCoefficient => lastHydrodynamicForceCoefficient;
    public int LastTargetCount => lastTargetCount;

    void Reset()
    {
        targetRoot = transform;
        ResolveTargets();
    }

    void Awake()
    {
        ResolveTargets();
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

        float lower = Mathf.Min(minHydrodynamicForceCoefficient, maxHydrodynamicForceCoefficient);
        float upper = Mathf.Max(minHydrodynamicForceCoefficient, maxHydrodynamicForceCoefficient);
        float coefficient = Mathf.Lerp(lower, upper, Mathf.Clamp01(unitSample));

        for (int i = 0; i < waterObjects.Count; i++)
        {
            WaterObject waterObject = waterObjects[i];
            if (waterObject == null)
            {
                continue;
            }

            waterObject.hydrodynamicForceCoefficient = coefficient;
        }

        lastHydrodynamicForceCoefficient = coefficient;
        lastTargetCount = waterObjects.Count;

        if (logRandomizedValue)
        {
            Debug.Log(
                $"[{nameof(Dwp2HydrodynamicForceCoefficientRandomizer)}] hydrodynamicForceCoefficient={coefficient:F3}, targets={lastTargetCount}",
                this);
        }
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
