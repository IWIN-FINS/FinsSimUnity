using System.Collections.Generic;
using FinsSim.Actuators;
using FinsSim.Hydrodynamics;
using UnityEngine;

[DisallowMultipleComponent]
public sealed class HorizontalThrusterAmplitudeRandomizer : MonoBehaviour, IEpisodeRandomizable
{
    [Header("Targets")]
    public Transform targetRoot;
    public bool includeInactiveThrusters = true;
    public bool autoRefreshTargets = true;
    [Tooltip("Thrusters with abs(LocalForceDirection.y) <= this threshold are treated as horizontal.")]
    [Range(0f, 1f)] public float horizontalLocalYAbsThreshold = 0.35f;
    public List<Thruster> horizontalThrusters = new List<Thruster>();

    [Header("Randomization")]
    [Min(0f)] public float minAmplitudeScale = 0.85f;
    [Min(0f)] public float maxAmplitudeScale = 1.15f;
    [Tooltip("Use one shared scale for all horizontal thrusters. Disable to model per-thruster mismatch.")]
    public bool useCommonScale;
    public bool randomizeOnStart;
    public bool randomizePerEpisode = true;

    [Header("Optional Curve Scale")]
    [Tooltip("Also scale C1Forward/C1Reverse. Usually leave false when thrusters use NormalizedForce.")]
    public bool scaleForceConstants;

    [Header("Debug")]
    public bool logRandomizedValue;
    [SerializeField] float lastMinScale = 1f;
    [SerializeField] float lastMaxScale = 1f;
    [SerializeField] int lastTargetCount;

    readonly Dictionary<Thruster, ThrusterBaseline> _baselines = new Dictionary<Thruster, ThrusterBaseline>();

    struct ThrusterBaseline
    {
        public float MaxForwardForceN;
        public float MaxReverseForceN;
        public float C1Forward;
        public float C1Reverse;
    }

    public int LastTargetCount => lastTargetCount;
    public float LastMinScale => lastMinScale;
    public float LastMaxScale => lastMaxScale;

    void Reset()
    {
        targetRoot = transform;
        ResolveTargets();
    }

    void Awake()
    {
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
        if (targetRoot == null)
        {
            targetRoot = transform;
        }

        if (!autoRefreshTargets && horizontalThrusters.Count > 0)
        {
            RemoveMissingTargets();
            lastTargetCount = horizontalThrusters.Count;
            return;
        }

        horizontalThrusters.Clear();
        if (targetRoot == null)
        {
            lastTargetCount = 0;
            return;
        }

        Thruster[] found = targetRoot.GetComponentsInChildren<Thruster>(includeInactiveThrusters);
        for (int i = 0; i < found.Length; i++)
        {
            Thruster thruster = found[i];
            if (thruster != null && IsHorizontalThruster(thruster) && !horizontalThrusters.Contains(thruster))
            {
                horizontalThrusters.Add(thruster);
            }
        }

        lastTargetCount = horizontalThrusters.Count;
    }

    [ContextMenu("Capture Baseline")]
    public void CaptureBaseline()
    {
        ResolveTargets();
        RemoveMissingBaselines();

        for (int i = 0; i < horizontalThrusters.Count; i++)
        {
            Thruster thruster = horizontalThrusters[i];
            if (thruster == null || _baselines.ContainsKey(thruster))
            {
                continue;
            }

            _baselines[thruster] = new ThrusterBaseline
            {
                MaxForwardForceN = thruster.MaxForwardForceN,
                MaxReverseForceN = thruster.MaxReverseForceN,
                C1Forward = thruster.C1Forward,
                C1Reverse = thruster.C1Reverse,
            };
        }
    }

    [ContextMenu("Randomize Now")]
    public void RandomizeNow()
    {
        ApplyRandomization(UnityEngine.Random.value, () => UnityEngine.Random.value);
    }

    public void RandomizeForEpisode(RandomizationContext context)
    {
        if (!randomizePerEpisode)
        {
            return;
        }

        ApplyRandomization(context.Value(), context.Value);
    }

    void ApplyRandomization(float commonUnitSample, System.Func<float> nextUnitSample)
    {
        CaptureBaseline();

        float lower = Mathf.Min(minAmplitudeScale, maxAmplitudeScale);
        float upper = Mathf.Max(minAmplitudeScale, maxAmplitudeScale);
        float commonScale = Mathf.Lerp(lower, upper, Mathf.Clamp01(commonUnitSample));
        lastMinScale = float.PositiveInfinity;
        lastMaxScale = 0f;

        for (int i = 0; i < horizontalThrusters.Count; i++)
        {
            Thruster thruster = horizontalThrusters[i];
            if (thruster == null || !_baselines.TryGetValue(thruster, out ThrusterBaseline baseline))
            {
                continue;
            }

            float scale = useCommonScale
                ? commonScale
                : Mathf.Lerp(lower, upper, Mathf.Clamp01(nextUnitSample()));

            thruster.MaxForwardForceN = baseline.MaxForwardForceN * scale;
            thruster.MaxReverseForceN = baseline.MaxReverseForceN * scale;

            if (scaleForceConstants)
            {
                thruster.C1Forward = baseline.C1Forward * scale;
                thruster.C1Reverse = baseline.C1Reverse * scale;
            }

            lastMinScale = Mathf.Min(lastMinScale, scale);
            lastMaxScale = Mathf.Max(lastMaxScale, scale);
        }

        lastTargetCount = horizontalThrusters.Count;
        if (lastTargetCount == 0)
        {
            lastMinScale = 1f;
            lastMaxScale = 1f;
        }

        if (logRandomizedValue)
        {
            Debug.Log(
                $"[{nameof(HorizontalThrusterAmplitudeRandomizer)}] scale={lastMinScale:F3}..{lastMaxScale:F3}, targets={lastTargetCount}",
                this);
        }
    }

    bool IsHorizontalThruster(Thruster thruster)
    {
        Vector3 direction = thruster.LocalForceDirection.sqrMagnitude > 1e-8f
            ? thruster.LocalForceDirection.normalized
            : Vector3.forward;
        return Mathf.Abs(direction.y) <= horizontalLocalYAbsThreshold;
    }

    void RemoveMissingTargets()
    {
        for (int i = horizontalThrusters.Count - 1; i >= 0; i--)
        {
            if (horizontalThrusters[i] == null)
            {
                horizontalThrusters.RemoveAt(i);
            }
        }
    }

    void RemoveMissingBaselines()
    {
        s_removeKeys.Clear();
        foreach (Thruster thruster in _baselines.Keys)
        {
            if (thruster == null || !horizontalThrusters.Contains(thruster))
            {
                s_removeKeys.Add(thruster);
            }
        }

        for (int i = 0; i < s_removeKeys.Count; i++)
        {
            _baselines.Remove(s_removeKeys[i]);
        }
        s_removeKeys.Clear();
    }

    static readonly List<Thruster> s_removeKeys = new List<Thruster>();
}
