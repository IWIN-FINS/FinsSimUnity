using FinsSim.Hydrodynamics;
using UnityEngine;

[DisallowMultipleComponent]
public sealed class AngularStabilityRewardRandomizer : MonoBehaviour, IEpisodeRandomizable
{
    [Header("Target")]
    public ControlForPosition_AngularStabilityReward rewardTarget;
    public bool autoResolveTarget = true;

    [Header("Randomization")]
    [Min(0f)] public float minGlobalAngularPenaltyScale = 0.8f;
    [Min(0f)] public float maxGlobalAngularPenaltyScale = 1.4f;
    [Min(0f)] public float minYawPenaltyScale = 0.9f;
    [Min(0f)] public float maxYawPenaltyScale = 1.8f;
    [Min(0f)] public float minNearTargetPenaltyScale = 1.0f;
    [Min(0f)] public float maxNearTargetPenaltyScale = 2.0f;
    public bool randomizeOnStart;
    public bool randomizePerEpisode = true;

    [Header("Debug")]
    public bool logRandomizedValue;
    [SerializeField] float lastGlobalAngularPenaltyScale = 1f;
    [SerializeField] float lastYawPenaltyScale = 1f;
    [SerializeField] float lastNearTargetPenaltyScale = 1f;

    public float LastGlobalAngularPenaltyScale => lastGlobalAngularPenaltyScale;
    public float LastYawPenaltyScale => lastYawPenaltyScale;
    public float LastNearTargetPenaltyScale => lastNearTargetPenaltyScale;

    void Reset()
    {
        ResolveTarget();
    }

    void Awake()
    {
        ResolveTarget();
        rewardTarget?.CaptureAngularPenaltyBaseline();
    }

    void Start()
    {
        if (randomizeOnStart)
        {
            RandomizeNow();
        }
    }

    [ContextMenu("Resolve Target")]
    public void ResolveTarget()
    {
        if (!autoResolveTarget || rewardTarget != null)
        {
            return;
        }

        rewardTarget = GetComponent<ControlForPosition_AngularStabilityReward>();
        if (rewardTarget == null)
        {
            rewardTarget = GetComponentInChildren<ControlForPosition_AngularStabilityReward>(true);
        }
    }

    [ContextMenu("Randomize Now")]
    public void RandomizeNow()
    {
        ApplySample(UnityEngine.Random.value, UnityEngine.Random.value, UnityEngine.Random.value);
    }

    public void RandomizeForEpisode(RandomizationContext context)
    {
        if (!randomizePerEpisode)
        {
            return;
        }

        ApplySample(context.Value(), context.Value(), context.Value());
    }

    void ApplySample(float globalUnitSample, float yawUnitSample, float nearTargetUnitSample)
    {
        ResolveTarget();
        if (rewardTarget == null)
        {
            return;
        }

        lastGlobalAngularPenaltyScale = Sample(minGlobalAngularPenaltyScale, maxGlobalAngularPenaltyScale, globalUnitSample);
        lastYawPenaltyScale = Sample(minYawPenaltyScale, maxYawPenaltyScale, yawUnitSample);
        lastNearTargetPenaltyScale = Sample(minNearTargetPenaltyScale, maxNearTargetPenaltyScale, nearTargetUnitSample);

        rewardTarget.ApplyAngularPenaltyMultipliers(
            lastGlobalAngularPenaltyScale,
            lastYawPenaltyScale,
            lastNearTargetPenaltyScale);

        if (logRandomizedValue)
        {
            Debug.Log(
                $"[{nameof(AngularStabilityRewardRandomizer)}] global={lastGlobalAngularPenaltyScale:F3}, yaw={lastYawPenaltyScale:F3}, near={lastNearTargetPenaltyScale:F3}",
                this);
        }
    }

    static float Sample(float min, float max, float unitSample)
    {
        float lower = Mathf.Min(min, max);
        float upper = Mathf.Max(min, max);
        return Mathf.Lerp(lower, upper, Mathf.Clamp01(unitSample));
    }
}
