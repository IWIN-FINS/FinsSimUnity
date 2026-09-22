using Unity.MLAgents;
using UnityEngine;

[DefaultExecutionOrder(-100)]
public sealed class ThreeChaseOneCurriculumController : MonoBehaviour
{
    private const string Prefix = "finsim_3c1.";

    private const string LessonIdKey = Prefix + "lesson_id";
    private const string CaptureCriterionKey = Prefix + "capture.criterion";
    private const string CaptureNetSurfaceDistanceKey = Prefix + "capture.net_surface_distance";
    private const string CaptureNetSurfaceHoldTimeKey = Prefix + "capture.net_surface_hold_time";
    private const string CaptureUuvDistanceKey = Prefix + "capture.uuv_distance";
    private const string ChaserRewardModeKey = Prefix + "reward.mode";
    private const string ChaserCaptureRewardKey = Prefix + "reward.chaser_capture";
    private const string FishCaptureRewardKey = Prefix + "reward.fish_capture";
    private const string ChaserStepPenaltyKey = Prefix + "reward.chaser.step_penalty";
    private const string ChaserSimpleChaseDistanceKey = Prefix + "reward.chaser.simple_chase_distance";
    private const string ChaserSimpleChaseProgressKey = Prefix + "reward.chaser.simple_chase_progress";
    private const string ChaserSimpleChaseDistanceRangeKey = Prefix + "reward.chaser.simple_chase_distance_range";
    private const string ChaserFishClosingKey = Prefix + "reward.chaser.fish_closing";
    private const string ChaserNetterSpacingKey = Prefix + "reward.chaser.netter_spacing";
    private const string ChaserNetterSpacingProgressKey = Prefix + "reward.chaser.netter_spacing_progress";
    private const string ChaserHerderFishClosingKey = Prefix + "reward.chaser.herder_fish_closing";
    private const string ChaserHerdingProgressKey = Prefix + "reward.chaser.herding_progress";
    private const string ChaserAngularVelocityPenaltyKey = Prefix + "reward.chaser.angular_velocity_penalty";
    private const string ChaserLinearVelocityPenaltyKey = Prefix + "reward.chaser.linear_velocity_penalty";
    private const string ChaserActionMagnitudePenaltyKey = Prefix + "reward.chaser.action_magnitude_penalty";
    private const string ChaserActionDeltaPenaltyKey = Prefix + "reward.chaser.action_delta_penalty";
    private const string PreySurvivalKey = Prefix + "reward.prey.survival";
    private const string PreyAverageSeparationKey = Prefix + "reward.prey.average_separation_progress";
    private const string PreyNearestThreatKey = Prefix + "reward.prey.nearest_threat_progress";
    private const string PreyMoveSpeedKey = Prefix + "prey.move_speed";
    private const string PreyVerticalSpeedScaleKey = Prefix + "prey.vertical_speed_scale";

    [SerializeField] private CatchAreaManager areaManager;
    [SerializeField] private bool logLessonChanges = true;
    [SerializeField] private bool recordStats = true;

    private bool registered;
    private bool initialApplied;
    private int currentLessonId = int.MinValue;

    private ChaserAgent[] Chasers
    {
        get
        {
            ResolveAreaManager();
            if (areaManager == null)
            {
                return new ChaserAgent[0];
            }

            return new[]
            {
                areaManager.netter1,
                areaManager.netter2,
                areaManager.herder
            };
        }
    }

    private PreyAgent Prey
    {
        get
        {
            ResolveAreaManager();
            return areaManager != null ? areaManager.fish : null;
        }
    }

    private void Awake()
    {
        ResolveAreaManager();
        RegisterCallbacksOnce();
        ApplyCurrentParameters();
    }

    private void Start()
    {
        ResolveAreaManager();
        ApplyCurrentParameters();
    }

    private void ResolveAreaManager()
    {
        if (areaManager != null)
        {
            return;
        }

        areaManager = GetComponent<CatchAreaManager>();
        if (areaManager == null)
        {
            areaManager = FindFirstObjectByType<CatchAreaManager>();
        }
    }

    private void RegisterCallbacksOnce()
    {
        if (registered)
        {
            return;
        }

        Academy.Instance.EnvironmentParameters.RegisterCallback(LessonIdKey, ApplyLessonId);
        Academy.Instance.EnvironmentParameters.RegisterCallback(CaptureCriterionKey, ApplyCaptureCriterion);
        Academy.Instance.EnvironmentParameters.RegisterCallback(CaptureNetSurfaceDistanceKey, value => WithManager(m => m.netSurfaceCaptureDistance = Mathf.Max(0f, value)));
        Academy.Instance.EnvironmentParameters.RegisterCallback(CaptureNetSurfaceHoldTimeKey, value => WithManager(m => m.netSurfaceCaptureHoldTime = Mathf.Max(0f, value)));
        Academy.Instance.EnvironmentParameters.RegisterCallback(CaptureUuvDistanceKey, value => WithManager(m => m.uuvCaptureDistance = Mathf.Max(0f, value)));
        Academy.Instance.EnvironmentParameters.RegisterCallback(ChaserRewardModeKey, ApplyChaserRewardMode);
        Academy.Instance.EnvironmentParameters.RegisterCallback(ChaserCaptureRewardKey, value => WithManager(m => m.chaserCaptureReward = Mathf.Max(0f, value)));
        Academy.Instance.EnvironmentParameters.RegisterCallback(FishCaptureRewardKey, value => WithManager(m => m.fishCapturePenalty = value));
        Academy.Instance.EnvironmentParameters.RegisterCallback(ChaserStepPenaltyKey, value => WithChasers(c => c.stepPenalty = value));
        Academy.Instance.EnvironmentParameters.RegisterCallback(ChaserSimpleChaseDistanceKey, value => WithChasers(c => c.simpleChaseDistanceRewardScale = value));
        Academy.Instance.EnvironmentParameters.RegisterCallback(ChaserSimpleChaseProgressKey, value => WithChasers(c => c.simpleChaseProgressRewardScale = value));
        Academy.Instance.EnvironmentParameters.RegisterCallback(ChaserSimpleChaseDistanceRangeKey, value => WithChasers(c => c.simpleChaseDistanceRewardRange = Mathf.Max(0.1f, value)));
        Academy.Instance.EnvironmentParameters.RegisterCallback(ChaserFishClosingKey, value => WithChasers(c => c.fishClosingRewardScale = value));
        Academy.Instance.EnvironmentParameters.RegisterCallback(ChaserNetterSpacingKey, value => WithChasers(c => c.netterSpacingRewardScale = value));
        Academy.Instance.EnvironmentParameters.RegisterCallback(ChaserNetterSpacingProgressKey, value => WithChasers(c => c.netterSpacingProgressRewardScale = value));
        Academy.Instance.EnvironmentParameters.RegisterCallback(ChaserHerderFishClosingKey, value => WithChasers(c => c.herderFishClosingRewardScale = value));
        Academy.Instance.EnvironmentParameters.RegisterCallback(ChaserHerdingProgressKey, value => WithChasers(c => c.herdingProgressRewardScale = value));
        Academy.Instance.EnvironmentParameters.RegisterCallback(ChaserAngularVelocityPenaltyKey, value => WithChasers(c => c.angularVelocityPenaltyScale = Mathf.Max(0f, value)));
        Academy.Instance.EnvironmentParameters.RegisterCallback(ChaserLinearVelocityPenaltyKey, value => WithChasers(c => c.linearVelocityPenaltyScale = Mathf.Max(0f, value)));
        Academy.Instance.EnvironmentParameters.RegisterCallback(ChaserActionMagnitudePenaltyKey, value => WithChasers(c => c.actionMagnitudePenaltyScale = Mathf.Max(0f, value)));
        Academy.Instance.EnvironmentParameters.RegisterCallback(ChaserActionDeltaPenaltyKey, value => WithChasers(c => c.actionDeltaPenaltyScale = Mathf.Max(0f, value)));
        Academy.Instance.EnvironmentParameters.RegisterCallback(PreySurvivalKey, value => WithPrey(p => p.survivalReward = value));
        Academy.Instance.EnvironmentParameters.RegisterCallback(PreyAverageSeparationKey, value => WithPrey(p => p.averageSeparationProgressRewardScale = value));
        Academy.Instance.EnvironmentParameters.RegisterCallback(PreyNearestThreatKey, value => WithPrey(p => p.nearestThreatProgressRewardScale = value));
        Academy.Instance.EnvironmentParameters.RegisterCallback(PreyMoveSpeedKey, value => WithPrey(p => p.moveSpeed = Mathf.Max(0f, value)));
        Academy.Instance.EnvironmentParameters.RegisterCallback(PreyVerticalSpeedScaleKey, value => WithPrey(p => p.verticalSpeedScale = Mathf.Clamp01(value)));

        registered = true;
    }

    private void ApplyCurrentParameters()
    {
        if (areaManager == null)
        {
            return;
        }

        ApplyLessonId(Get(LessonIdKey, currentLessonId == int.MinValue ? 0f : currentLessonId));
        ApplyCaptureCriterion(Get(CaptureCriterionKey, (float)areaManager.captureCriterion));
        areaManager.netSurfaceCaptureDistance = Mathf.Max(0f, Get(CaptureNetSurfaceDistanceKey, areaManager.netSurfaceCaptureDistance));
        areaManager.netSurfaceCaptureHoldTime = Mathf.Max(0f, Get(CaptureNetSurfaceHoldTimeKey, areaManager.netSurfaceCaptureHoldTime));
        areaManager.uuvCaptureDistance = Mathf.Max(0f, Get(CaptureUuvDistanceKey, areaManager.uuvCaptureDistance));
        areaManager.chaserCaptureReward = Mathf.Max(0f, Get(ChaserCaptureRewardKey, areaManager.chaserCaptureReward));
        areaManager.fishCapturePenalty = Get(FishCaptureRewardKey, areaManager.fishCapturePenalty);

        foreach (ChaserAgent chaser in Chasers)
        {
            if (chaser == null)
            {
                continue;
            }

            ApplyChaserRewardMode(Get(ChaserRewardModeKey, (float)chaser.rewardMode));
            chaser.stepPenalty = Get(ChaserStepPenaltyKey, chaser.stepPenalty);
            chaser.simpleChaseDistanceRewardScale = Get(ChaserSimpleChaseDistanceKey, chaser.simpleChaseDistanceRewardScale);
            chaser.simpleChaseProgressRewardScale = Get(ChaserSimpleChaseProgressKey, chaser.simpleChaseProgressRewardScale);
            chaser.simpleChaseDistanceRewardRange = Mathf.Max(0.1f, Get(ChaserSimpleChaseDistanceRangeKey, chaser.simpleChaseDistanceRewardRange));
            chaser.fishClosingRewardScale = Get(ChaserFishClosingKey, chaser.fishClosingRewardScale);
            chaser.netterSpacingRewardScale = Get(ChaserNetterSpacingKey, chaser.netterSpacingRewardScale);
            chaser.netterSpacingProgressRewardScale = Get(ChaserNetterSpacingProgressKey, chaser.netterSpacingProgressRewardScale);
            chaser.herderFishClosingRewardScale = Get(ChaserHerderFishClosingKey, chaser.herderFishClosingRewardScale);
            chaser.herdingProgressRewardScale = Get(ChaserHerdingProgressKey, chaser.herdingProgressRewardScale);
            chaser.angularVelocityPenaltyScale = Mathf.Max(0f, Get(ChaserAngularVelocityPenaltyKey, chaser.angularVelocityPenaltyScale));
            chaser.linearVelocityPenaltyScale = Mathf.Max(0f, Get(ChaserLinearVelocityPenaltyKey, chaser.linearVelocityPenaltyScale));
            chaser.actionMagnitudePenaltyScale = Mathf.Max(0f, Get(ChaserActionMagnitudePenaltyKey, chaser.actionMagnitudePenaltyScale));
            chaser.actionDeltaPenaltyScale = Mathf.Max(0f, Get(ChaserActionDeltaPenaltyKey, chaser.actionDeltaPenaltyScale));
        }

        PreyAgent prey = Prey;
        if (prey != null)
        {
            prey.survivalReward = Get(PreySurvivalKey, prey.survivalReward);
            prey.averageSeparationProgressRewardScale = Get(PreyAverageSeparationKey, prey.averageSeparationProgressRewardScale);
            prey.nearestThreatProgressRewardScale = Get(PreyNearestThreatKey, prey.nearestThreatProgressRewardScale);
            prey.moveSpeed = Mathf.Max(0f, Get(PreyMoveSpeedKey, prey.moveSpeed));
            prey.verticalSpeedScale = Mathf.Clamp01(Get(PreyVerticalSpeedScaleKey, prey.verticalSpeedScale));
        }

        if (!initialApplied)
        {
            initialApplied = true;
            Debug.Log("[ThreeChaseOneCurriculumController] Environment parameters initialized.");
        }
    }

    private float Get(string key, float defaultValue)
    {
        return Academy.Instance.EnvironmentParameters.GetWithDefault(key, defaultValue);
    }

    private void ApplyLessonId(float value)
    {
        int lessonId = Mathf.RoundToInt(value);
        bool changed = lessonId != currentLessonId;
        currentLessonId = lessonId;

        if (recordStats)
        {
            Academy.Instance.StatsRecorder.Add(
                "FinsSim/3Chase1/lesson_id",
                currentLessonId,
                StatAggregationMethod.MostRecent);
        }

        if (logLessonChanges && changed)
        {
            Debug.Log($"[ThreeChaseOneCurriculumController] lesson_id={currentLessonId}");
        }
    }

    private void ApplyCaptureCriterion(float value)
    {
        int criterion = Mathf.Clamp(
            Mathf.RoundToInt(value),
            0,
            (int)CatchAreaManager.CaptureCriterion.NetCollisionOrUuvProximity);
        WithManager(m => m.captureCriterion = (CatchAreaManager.CaptureCriterion)criterion);
    }

    private void ApplyChaserRewardMode(float value)
    {
        int mode = Mathf.Clamp(
            Mathf.RoundToInt(value),
            0,
            (int)ChaserRewardMode.SimpleChasePrey);
        WithChasers(c => c.rewardMode = (ChaserRewardMode)mode);
    }

    private void WithManager(System.Action<CatchAreaManager> action)
    {
        ResolveAreaManager();
        if (areaManager != null)
        {
            action(areaManager);
        }
    }

    private void WithChasers(System.Action<ChaserAgent> action)
    {
        foreach (ChaserAgent chaser in Chasers)
        {
            if (chaser != null)
            {
                action(chaser);
            }
        }
    }

    private void WithPrey(System.Action<PreyAgent> action)
    {
        PreyAgent prey = Prey;
        if (prey != null)
        {
            action(prey);
        }
    }
}
