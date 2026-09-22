using UnityEngine;
using System.Collections.Generic;
using MarusThruster = FinsSim.Actuators.Thruster;
using MarusThrusterController = FinsSim.Actuators.ThrusterController;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using NWH.DWP2.ShipController; // 引入动态水物理的船舶控制器命名空间

public enum AgentRole { Netter, Herder }

public enum ChaserRewardMode
{
    HerdingNet = 0,
    SimpleChasePrey = 1
}

public class ChaserAgent : Agent
{
    public const int VectorObservationSize = 30;

    public AgentRole role;
    public Transform fish;
    public ChaserAgent partnerNetter; // 仅拉网者需要引用另一个拉网者
    public CatchAreaManager areaManager;
    public Transform selfTransform;

    private Rigidbody rb;

    [Tooltip("推进效率参考速度，单位 m/s。它会参与推进器推力曲线归一化，不是 Rigidbody 的硬限速。")]
    public float maxSpeed = 4f;

    [Header("Thruster Tuning")]
    [Tooltip("当前 UUV 竖直推进器统一使用的 maxThrust。通常由 CatchAreaManager 统一覆写。")]
    [SerializeField] private float verticalThrusterMaxThrust = 150f;
    [Tooltip("当前 UUV 水平推进器统一使用的 maxThrust。通常由 CatchAreaManager 统一覆写。")]
    [SerializeField] private float horizontalThrusterMaxThrust = 400f;

    [Tooltip("推进器效率曲线：横轴为当前速度 / maxSpeed，纵轴为剩余推力比例。")]
    public AnimationCurve engineThrustCurve = CreateDefaultEngineThrustCurve();
    [Header("Reward Tuning")]
    [Tooltip("选择追方的单步 shaping reward。HerdingNet 为原始协同收网 reward；SimpleChasePrey 为直接追近 prey 的简单 reward。")]
    public ChaserRewardMode rewardMode = ChaserRewardMode.HerdingNet;
    [Tooltip("每步轻微时间惩罚，鼓励更快收网。")]
    public float stepPenalty = -0.0002f;
    [Tooltip("SimpleChasePrey: 距离 prey 越近得到越高的静态 reward。")]
    public float simpleChaseDistanceRewardScale = 0.05f;
    [Tooltip("SimpleChasePrey: 相邻 step 到 prey 的距离缩小时得到的进度 reward。")]
    public float simpleChaseProgressRewardScale = 0.30f;
    [Tooltip("SimpleChasePrey: 静态距离 reward 的归一化距离范围。")]
    public float simpleChaseDistanceRewardRange = 8f;
    [Tooltip("拉网者接近鱼的进度奖励系数。")]
    public float fishClosingRewardScale = 0.20f;
    [Tooltip("拉网者维持合理网宽的静态奖励系数。")]
    public float netterSpacingRewardScale = 0.005f;
    [Tooltip("拉网者把网宽调回目标值时的进度奖励系数。")]
    public float netterSpacingProgressRewardScale = 0.02f;
    public float desiredNetterSpacing = 6f;
    public float netterSpacingTolerance = 2.5f;
    [Tooltip("赶鱼者接近鱼的进度奖励系数。")]
    public float herderFishClosingRewardScale = 0.08f;
    [Tooltip("赶鱼者让鱼更接近网中心时的进度奖励系数。")]
    public float herdingProgressRewardScale = 0.20f;
    [Tooltip("角速度惩罚系数，抑制原地高速旋转。")]
    public float angularVelocityPenaltyScale = 0.002f;
    [Tooltip("线速度惩罚系数，避免用高速乱冲换取偶然进度奖励。")]
    public float linearVelocityPenaltyScale = 0.0005f;
    [Tooltip("推进器动作幅值惩罚系数，抑制长期满推力。")]
    public float actionMagnitudePenaltyScale = 0.001f;
    [Tooltip("推进器动作变化惩罚系数，抑制相邻决策间的动作抖动。")]
    public float actionDeltaPenaltyScale = 0.002f;

    AdvancedShipController advancedShipController;

    List<Engine> engines;
    MarusThrusterController finsRovThrusterController;
    readonly MarusThruster[] orderedFinsRovThrusters = new MarusThruster[FinsROVAgentRuntime.DefaultThrusterOrder.Length];

    [Header("Debug Heuristic Input")]
    [Tooltip("仅用于手动调试 UUV。训练/相机漫游时应保持关闭，避免 WASD 被 Heuristic 读成推进器动作。")]
    [SerializeField] private bool enableKeyboardHeuristicInput = false;

    [Header("FinsROV Thruster Control")]
    [Tooltip("开启后使用 FinsROV prefab 上的 Marus ThrusterController/Thruster，而不是 DWP2 AdvancedShipController/Engine。")]
    [SerializeField] private bool useFinsRovThrusters = false;
    [SerializeField] private MarusThrusterController thrusterControllerOverride;
    [SerializeField] private ThrusterCommandMode thrusterCommandMode = ThrusterCommandMode.ScaledForceRequest;
    [ShowWhenThrusterCommandMode(ThrusterCommandMode.ScaledForceRequest)]
    [SerializeField] private float actionForceScaleN = 7f;
    [SerializeField] private bool autoResolveThrustersFromChildren = true;
    [SerializeField] private bool logThrusterResolution = true;
    [Tooltip("开启后把 CatchAreaManager 的 vertical/horizontal maxThrust 迁移到 FinsROV Marus Thruster 的 force limits 和 actionForceScaleN。")]
    [SerializeField] private bool allowUnifiedMaxThrustToOverrideFinsRovForceScale = false;

    [Header("Runtime Action Audit")]
    [Tooltip("仅用于排查 Python ML-Agents 动作是否在 Unity 端被改写。")]
    [SerializeField] private bool logActionAudit;
    [SerializeField, Min(1)] private int actionAuditInterval = 20;

    public ShipInputActions shipInputActions;
    private float previousDistanceToFish;
    private float previousPartnerSpacingError;
    private float previousFishToNetCenterDistance;
    private readonly float[] baselineOverrideActions = new float[8];
    private readonly float[] appliedContinuousActions = new float[8];
    private readonly float[] previousAppliedContinuousActions = new float[8];
    private readonly float[] receivedContinuousActions = new float[8];
    private int actionAuditStep;
    private string lastActionSource = "PythonMLAgents";

    public float LastStepReward { get; private set; }
    public float LastStepPenaltyReward { get; private set; }
    public float LastSimpleChaseReward { get; private set; }
    public float LastFishClosingReward { get; private set; }
    public float LastNetterSpacingReward { get; private set; }
    public float LastNetterSpacingProgressReward { get; private set; }
    public float LastHerderFishClosingReward { get; private set; }
    public float LastHerdingProgressReward { get; private set; }
    public float LastAngularVelocityPenaltyReward { get; private set; }
    public float LastLinearVelocityPenaltyReward { get; private set; }
    public float LastActionMagnitudePenaltyReward { get; private set; }
    public float LastActionDeltaPenaltyReward { get; private set; }

    /// <summary>Editor-only diagnostics use this without changing control semantics.</summary>
    public void SetActionAuditEnabled(bool enabled)
    {
        logActionAudit = enabled;
        actionAuditStep = 0;
    }

    private Transform PoseTransform => selfTransform != null ? selfTransform : transform;

    public float VerticalThrusterMaxThrust
    {
        get => verticalThrusterMaxThrust;
        set
        {
            verticalThrusterMaxThrust = Mathf.Max(0f, value);
            ApplyEngineSettings();
        }
    }

    public float HorizontalThrusterMaxThrust
    {
        get => horizontalThrusterMaxThrust;
        set
        {
            horizontalThrusterMaxThrust = Mathf.Max(0f, value);
            ApplyEngineSettings();
        }
    }

    private static AnimationCurve CreateDefaultEngineThrustCurve()
    {
        AnimationCurve curve = new AnimationCurve( // 模拟的非线性衰减推力曲线（当然只是一个比较好的近似）
            new Keyframe(0f, 1f),
            new Keyframe(0.4f, 0.9f),
            new Keyframe(0.7f, 0.6f),
            new Keyframe(1f, 0.15f),
            new Keyframe(1.2f, 0f)
        );
        curve.preWrapMode = WrapMode.ClampForever;
        curve.postWrapMode = WrapMode.ClampForever;
        return curve;
    }

    private Engine GetEngineByName(string engineName)
    {
        return engines.Find(engine => engine.name == engineName);
    }

    private static float GetBoundedProgress(float previousValue, float currentValue)
    {
        return Mathf.Clamp(previousValue - currentValue, -1f, 1f);
    }

    private void ResetRewardTracking()
    {
        ClearRewardDebugBreakdown();

        Vector3 posePosition = PoseTransform.position;
        previousDistanceToFish = fish != null ? Vector3.Distance(posePosition, fish.position) : 0f;

        NetSurfaceModel netModel = areaManager != null ? areaManager.NetSurfaceModel : null;

        if (partnerNetter != null)
        {
            float partnerDistance = netModel != null && netModel.IsReady()
                ? netModel.GetNetWidth()
                : Vector3.Distance(posePosition, partnerNetter.PoseTransform.position);
            float targetWidth = netModel != null ? netModel.TargetNetWidth : desiredNetterSpacing;
            previousPartnerSpacingError = Mathf.Abs(partnerDistance - targetWidth);
        }
        else
        {
            previousPartnerSpacingError = 0f;
        }

        previousFishToNetCenterDistance = areaManager != null && fish != null
            ? Vector3.Distance(fish.position, areaManager.GetNetCenter())
            : 0f;
    }

    private void ClearRewardDebugBreakdown()
    {
        LastStepReward = 0f;
        LastStepPenaltyReward = 0f;
        LastSimpleChaseReward = 0f;
        LastFishClosingReward = 0f;
        LastNetterSpacingReward = 0f;
        LastNetterSpacingProgressReward = 0f;
        LastHerderFishClosingReward = 0f;
        LastHerdingProgressReward = 0f;
        LastAngularVelocityPenaltyReward = 0f;
        LastLinearVelocityPenaltyReward = 0f;
        LastActionMagnitudePenaltyReward = 0f;
        LastActionDeltaPenaltyReward = 0f;
    }

    private void EnsureShipControllerReferences()
    {
        if (advancedShipController == null)
        {
            advancedShipController = GetComponent<AdvancedShipController>();
        }

        if (advancedShipController == null)
        {
            engines = null;
            return;
        }

        if (engines == null || engines.Count == 0)
        {
            engines = advancedShipController.engines;
        }
    }

    private static bool IsVerticalEngine(Engine engine)
    {
        return engine != null
               && !string.IsNullOrEmpty(engine.name)
               && engine.name.StartsWith("Vertical", System.StringComparison.OrdinalIgnoreCase);
    }

    private static bool IsHorizontalEngine(Engine engine)
    {
        return engine != null
               && !string.IsNullOrEmpty(engine.name)
               && engine.name.StartsWith("Horizontal", System.StringComparison.OrdinalIgnoreCase);
    }

    private static bool IsVerticalThruster(MarusThruster thruster)
    {
        return thruster != null
               && !string.IsNullOrEmpty(thruster.name)
               && thruster.name.StartsWith("Vertical", System.StringComparison.OrdinalIgnoreCase);
    }

    private static bool IsHorizontalThruster(MarusThruster thruster)
    {
        return thruster != null
               && !string.IsNullOrEmpty(thruster.name)
               && thruster.name.StartsWith("Horizontal", System.StringComparison.OrdinalIgnoreCase);
    }

    private bool ShouldUseFinsRovThrusters()
    {
        if (useFinsRovThrusters)
        {
            return true;
        }

        if (thrusterControllerOverride != null || finsRovThrusterController != null)
        {
            return true;
        }

        return GetComponent<MarusThrusterController>() != null ||
               GetComponentInChildren<MarusThrusterController>(true) != null;
    }

    private void ApplyEngineSettings()
    {
        if (ShouldUseFinsRovThrusters())
        {
            ResolveFinsRovThrusterReferences();
            ApplyFinsRovThrusterForceLimitsIfEnabled();
            return;
        }

        EnsureShipControllerReferences();
        if (engines == null)
        {
            return;
        }

        AnimationCurve thrustCurveToApply = engineThrustCurve != null && engineThrustCurve.length > 0
            ? new AnimationCurve(engineThrustCurve.keys)
            : CreateDefaultEngineThrustCurve();
        thrustCurveToApply.preWrapMode = WrapMode.ClampForever;
        thrustCurveToApply.postWrapMode = WrapMode.ClampForever;

        foreach (Engine engine in engines)
        {
            if (engine == null)
            {
                continue;
            }

            engine.useExternalThrottleInput = true;
            engine.maxSpeed = maxSpeed;
            engine.thrustCurve = thrustCurveToApply;

            if (IsVerticalEngine(engine))
            {
                engine.maxThrust = verticalThrusterMaxThrust;
            }
            else if (IsHorizontalEngine(engine))
            {
                engine.maxThrust = horizontalThrusterMaxThrust;
            }
        }
    }

    public void ApplyUnifiedThrusterSettings(float verticalMaxThrust, float horizontalMaxThrust)
    {
        verticalThrusterMaxThrust = Mathf.Max(0f, verticalMaxThrust);
        horizontalThrusterMaxThrust = Mathf.Max(0f, horizontalMaxThrust);

        if (ShouldUseFinsRovThrusters())
        {
            if (allowUnifiedMaxThrustToOverrideFinsRovForceScale)
            {
                actionForceScaleN = Mathf.Max(0f, Mathf.Max(verticalThrusterMaxThrust, horizontalThrusterMaxThrust));
            }

            ResolveFinsRovThrusterReferences();
            ApplyFinsRovThrusterForceLimitsIfEnabled();
            return;
        }

        ApplyEngineSettings();
    }

    private void ApplyFinsRovThrusterForceLimitsIfEnabled()
    {
        if (!allowUnifiedMaxThrustToOverrideFinsRovForceScale)
        {
            return;
        }

        for (int i = 0; i < orderedFinsRovThrusters.Length; i++)
        {
            MarusThruster thruster = orderedFinsRovThrusters[i];
            if (thruster == null)
            {
                continue;
            }

            float maxForceN;
            if (IsVerticalThruster(thruster))
            {
                maxForceN = verticalThrusterMaxThrust;
            }
            else if (IsHorizontalThruster(thruster))
            {
                maxForceN = horizontalThrusterMaxThrust;
            }
            else
            {
                maxForceN = Mathf.Max(verticalThrusterMaxThrust, horizontalThrusterMaxThrust);
            }

            maxForceN = Mathf.Max(0f, maxForceN);
            thruster.MaxForwardForceN = maxForceN;
            thruster.MaxReverseForceN = -maxForceN;
        }
    }

    public void ApplyUnifiedThrusterSettings(float maxThrust)
    {
        ApplyUnifiedThrusterSettings(maxThrust, maxThrust);
    }

    public override void Initialize()
    {
        rb = GetComponent<Rigidbody>();

        if (enableKeyboardHeuristicInput)
        {
            EnsureShipInputActionsEnabled();
        }

        ApplyEngineSettings();
        areaManager?.ApplyThrusterSettingsToChaser(this);
    }

    protected override void OnDisable()
    {
        base.OnDisable();
        shipInputActions?.Disable();
    }

    private void OnDestroy()
    {
        shipInputActions?.Dispose();
        shipInputActions = null;
    }

    public override void OnEpisodeBegin()
    {
        areaManager?.RandomizeDomainForEpisode();
        areaManager.ResetArea();
        for (int i = 0; i < previousAppliedContinuousActions.Length; i++)
        {
            previousAppliedContinuousActions[i] = 0f;
            appliedContinuousActions[i] = 0f;
        }
        ZeroThrusterOutput();
        ResetRewardTracking();
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        Transform pose = PoseTransform;
        Rigidbody fishRb = fish != null ? fish.GetComponent<Rigidbody>() : null;

        // 30D actor observation, entirely expressed in controller-body coordinates.
        Vector3 selfLinearVelocityBody = ControllerBodyFrame.WorldLinearVelocityToBody(pose, rb.linearVelocity);
        Vector3 selfAngularVelocityBody = ControllerBodyFrame.WorldAngularVelocityToBody(pose, rb.angularVelocity);
        Vector3 fishRelativePositionBody = Vector3.zero;
        Vector3 fishRelativeVelocityBody = Vector3.zero;
        if (fish != null)
        {
            fishRelativePositionBody = ControllerBodyFrame.WorldToBodyPositionDelta(pose, fish.position - pose.position);
            if (fishRb != null)
            {
                fishRelativeVelocityBody = ControllerBodyFrame.WorldLinearVelocityToBody(
                    pose,
                    fishRb.linearVelocity - rb.linearVelocity
                );
            }
        }
        float distanceToFish = fishRelativePositionBody.magnitude;
        float bearingToFish = ControllerBodyFrame.HorizontalBearingRad(fishRelativePositionBody);

        sensor.AddObservation(selfLinearVelocityBody);
        sensor.AddObservation(selfAngularVelocityBody);
        sensor.AddObservation(fishRelativePositionBody);
        sensor.AddObservation(fishRelativeVelocityBody);
        sensor.AddObservation(distanceToFish);
        sensor.AddObservation(bearingToFish);

        ChaserAgent[] teammates = GetOrderedTeammates();
        for (int i = 0; i < 2; i++)
        {
            ChaserAgent teammate = i < teammates.Length ? teammates[i] : null;
            AddTeammateObservation(sensor, teammate);
        }
    }

    private void FixedUpdate()
    {
        long finsSimProfileStart = FinsSimRuntimeProfiler.Begin();
        UnityEngine.Profiling.Profiler.BeginSample("FinsSim.ChaserAgent.FixedUpdate");
        try
        {
            if (!Application.isPlaying)
            {
                return;
            }

            NetSeparationStressTest separationStressTest =
                NetSeparationStressTest.ResolveActive(areaManager);
            if (separationStressTest != null && separationStressTest.IsProvidingActions)
            {
                RequestDecision();
                return;
            }
        }
        finally
        {
            UnityEngine.Profiling.Profiler.EndSample();
            FinsSimRuntimeProfiler.End("FinsSim.ChaserAgent.FixedUpdate", finsSimProfileStart);
        }
    }

    private ChaserAgent[] GetOrderedTeammates()
    {
        if (areaManager == null)
        {
            return new ChaserAgent[0];
        }

        if (role == AgentRole.Herder)
        {
            return new ChaserAgent[] { areaManager.netter1, areaManager.netter2 };
        }

        ChaserAgent otherNetter = partnerNetter;
        if (otherNetter == null)
        {
            if (areaManager.netter1 != null && areaManager.netter1 != this)
            {
                otherNetter = areaManager.netter1;
            }
            else if (areaManager.netter2 != null && areaManager.netter2 != this)
            {
                otherNetter = areaManager.netter2;
            }
        }

        return new ChaserAgent[] { areaManager.herder, otherNetter };
    }

    private void AddTeammateObservation(VectorSensor sensor, ChaserAgent teammate)
    {
        if (teammate == null)
        {
            sensor.AddObservation(0f);
            sensor.AddObservation(0f);
            sensor.AddObservation(Vector3.zero);
            sensor.AddObservation(Vector3.zero);
            return;
        }

        bool teammateIsHerder = teammate.role == AgentRole.Herder;
        sensor.AddObservation(teammateIsHerder ? 1f : 0f);
        sensor.AddObservation(teammateIsHerder ? 0f : 1f);

        Transform pose = PoseTransform;
        Transform teammatePose = teammate.PoseTransform;
        Vector3 relativePositionBody = ControllerBodyFrame.WorldToBodyPositionDelta(
            pose,
            teammatePose.position - pose.position
        );

        Rigidbody teammateRb = teammate.GetComponent<Rigidbody>();
        Vector3 relativeVelocityBody = Vector3.zero;
        if (teammateRb != null)
        {
            relativeVelocityBody = ControllerBodyFrame.WorldLinearVelocityToBody(
                pose,
                teammateRb.linearVelocity - rb.linearVelocity
            );
        }

        sensor.AddObservation(relativePositionBody);
        sensor.AddObservation(relativeVelocityBody);
    }

    private void AddHomogeneousTeammateObservation(VectorSensor sensor, ChaserAgent teammate)
    {
        if (teammate == null)
        {
            sensor.AddObservation(Vector3.zero);
            sensor.AddObservation(Vector3.zero);
            return;
        }

        Transform pose = PoseTransform;
        Transform teammatePose = teammate.PoseTransform;
        Vector3 relativePositionBody = ControllerBodyFrame.WorldToBodyPositionDelta(
            pose,
            teammatePose.position - pose.position
        );

        Rigidbody teammateRb = teammate.GetComponent<Rigidbody>();
        Vector3 relativeVelocityBody = Vector3.zero;
        if (teammateRb != null)
        {
            relativeVelocityBody = ControllerBodyFrame.WorldLinearVelocityToBody(
                pose,
                teammateRb.linearVelocity - rb.linearVelocity
            );
        }

        sensor.AddObservation(relativePositionBody);
        sensor.AddObservation(relativeVelocityBody);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        if (areaManager != null && areaManager.TryResolveCapture())
        {
            return;
        }

        ResolveAppliedContinuousActions(actions);
        LogActionAuditIfEnabled();

        ApplyThrusterOutput();

        // float moveStepX = actions.ContinuousActions[0];
        // float moveStepZ = actions.ContinuousActions[1];
        // rb.linearVelocity = new Vector3(moveStepX, 0, moveStepZ) * moveSpeed;

        // --- 奖励逻辑 ---
        float reward = stepPenalty;
        ClearRewardDebugBreakdown();
        LastStepPenaltyReward = stepPenalty;

        float actionMagnitude = 0f;
        float actionDelta = 0f;
        for (int i = 0; i < appliedContinuousActions.Length; i++)
        {
            actionMagnitude += appliedContinuousActions[i] * appliedContinuousActions[i];
            actionDelta += Mathf.Abs(appliedContinuousActions[i] - previousAppliedContinuousActions[i]);
        }
        actionMagnitude /= appliedContinuousActions.Length;
        actionDelta /= appliedContinuousActions.Length;

        float angularVelocityPenalty = -angularVelocityPenaltyScale * rb.angularVelocity.magnitude;
        float linearVelocityPenalty = -linearVelocityPenaltyScale * rb.linearVelocity.magnitude;
        float actionMagnitudePenalty = -actionMagnitudePenaltyScale * actionMagnitude;
        float actionDeltaPenalty = -actionDeltaPenaltyScale * actionDelta;
        reward += angularVelocityPenalty;
        reward += linearVelocityPenalty;
        reward += actionMagnitudePenalty;
        reward += actionDeltaPenalty;
        LastAngularVelocityPenaltyReward = angularVelocityPenalty;
        LastLinearVelocityPenaltyReward = linearVelocityPenalty;
        LastActionMagnitudePenaltyReward = actionMagnitudePenalty;
        LastActionDeltaPenaltyReward = actionDeltaPenalty;

        Vector3 posePosition = PoseTransform.position;

        if (rewardMode == ChaserRewardMode.SimpleChasePrey)
        {
            float distToFish = fish != null ? Vector3.Distance(posePosition, fish.position) : previousDistanceToFish;
            float distanceReward = simpleChaseDistanceRewardScale
                                   * Mathf.Clamp01(1f - distToFish / Mathf.Max(0.1f, simpleChaseDistanceRewardRange));
            float progressReward = simpleChaseProgressRewardScale
                                   * GetBoundedProgress(previousDistanceToFish, distToFish);
            float simpleChaseReward = distanceReward + progressReward;
            reward += simpleChaseReward;
            LastSimpleChaseReward = simpleChaseReward;
            LastFishClosingReward = progressReward;
            previousDistanceToFish = distToFish;
        }
        else if (role == AgentRole.Netter)
        {
            float distToFish = fish != null ? Vector3.Distance(posePosition, fish.position) : previousDistanceToFish;
            float fishClosingReward = fishClosingRewardScale * GetBoundedProgress(previousDistanceToFish, distToFish);
            reward += fishClosingReward; //单步靠近奖励
            LastFishClosingReward = fishClosingReward;
            previousDistanceToFish = distToFish;

            // 如果是拉网者，要与另一个拉网者保持同步且靠近
            if (partnerNetter != null)
            {
                NetSurfaceModel netModel = areaManager != null ? areaManager.NetSurfaceModel : null;
                float currentWidth = netModel != null && netModel.IsReady()
                    ? netModel.GetNetWidth()
                    : Vector3.Distance(posePosition, partnerNetter.PoseTransform.position);
                float targetWidth = netModel != null ? netModel.TargetNetWidth : desiredNetterSpacing;
                float tolerance = netModel != null ? Mathf.Max(0.1f, netModel.WidthTolerance) : netterSpacingTolerance;
                float spacingError = Mathf.Abs(currentWidth - targetWidth);
                float distToFishForGate = fish != null ? Vector3.Distance(posePosition, fish.position) : previousDistanceToFish;
                float spacingTaskGate = Mathf.Clamp01(1f - Mathf.Max(0f, distToFishForGate - targetWidth) / Mathf.Max(0.1f, targetWidth));
                float spacingReward = spacingTaskGate * netterSpacingRewardScale * Mathf.Clamp01(1f - spacingError / tolerance);
                float spacingProgressReward =
                    netterSpacingProgressRewardScale * GetBoundedProgress(previousPartnerSpacingError, spacingError);

                if (netModel != null && netModel.IsReady())
                {
                    float heightError = netModel.GetHeightError();
                    float heightReward = 0.5f * netterSpacingRewardScale
                                         * Mathf.Clamp01(1f - heightError / Mathf.Max(0.1f, netModel.HeightTolerance));
                    spacingReward += spacingTaskGate * heightReward;

                    float fishToNetCenterDist = fish != null
                        ? Vector3.Distance(fish.position, netModel.GetNetCenter())
                        : previousFishToNetCenterDistance;
                    float fishCenterProgress = 0.35f * netterSpacingProgressRewardScale
                                               * GetBoundedProgress(previousFishToNetCenterDistance, fishToNetCenterDist);
                    spacingProgressReward += fishCenterProgress;
                    previousFishToNetCenterDistance = fishToNetCenterDist;
                }

                spacingReward = Mathf.Clamp(spacingReward, -0.2f, 0.2f);
                spacingProgressReward = Mathf.Clamp(spacingProgressReward, -0.2f, 0.2f);
                reward += spacingReward;
                reward += spacingProgressReward;
                LastNetterSpacingReward = spacingReward;
                LastNetterSpacingProgressReward = spacingProgressReward;
                previousPartnerSpacingError = spacingError;
            }
        }
        else if (role == AgentRole.Herder)
        {
            float distToFish = fish != null ? Vector3.Distance(posePosition, fish.position) : previousDistanceToFish;
            float herderFishClosingReward =
                herderFishClosingRewardScale * GetBoundedProgress(previousDistanceToFish, distToFish);
            reward += herderFishClosingReward;
            LastHerderFishClosingReward = herderFishClosingReward;
            previousDistanceToFish = distToFish;

            // 如果是赶鱼者，引导鱼走向两个拉网者的中点 (NetCenter)
            Vector3 netCenter = areaManager.GetNetCenter();
            float fishToCenterDist = fish != null ? Vector3.Distance(fish.position, netCenter) : previousFishToNetCenterDistance;
            float herdingProgressReward =
                herdingProgressRewardScale * GetBoundedProgress(previousFishToNetCenterDistance, fishToCenterDist);
            reward += herdingProgressReward;
            LastHerdingProgressReward = herdingProgressReward;
            previousFishToNetCenterDistance = fishToCenterDist;
        }

        SetReward(reward);
        LastStepReward = reward;

        if (role == AgentRole.Netter)
        {
            // Debug.Log("Role: " + role + ", Step reward: " + reward);
        }
        else if (role == AgentRole.Herder)
        {
            // Debug.Log("Role: " + role + ", Step reward: " + reward);
        }

        for (int i = 0; i < previousAppliedContinuousActions.Length; i++)
        {
            previousAppliedContinuousActions[i] = appliedContinuousActions[i];
        }


        // 碰撞墙壁惩罚
        // if (transform.localPosition.x > 10f || transform.localPosition.x < -10f ||
        //     transform.localPosition.z > 10f || transform.localPosition.z < -10f)
        // {
        //     AddReward(-0.1f);
        //     EndEpisode();
        // }
    }

    private void ApplyThrusterOutput()
    {
        if (ShouldUseFinsRovThrusters())
        {
            ResolveFinsRovThrusterReferences();
            FinsROVAgentRuntime.ApplyThrusterActions(
                orderedFinsRovThrusters,
                appliedContinuousActions,
                thrusterCommandMode,
                actionForceScaleN);
            return;
        }

        SetEngineThrottle("Vertical1", appliedContinuousActions[0]);
        SetEngineThrottle("Vertical2", appliedContinuousActions[1]);
        SetEngineThrottle("Vertical3", appliedContinuousActions[2]);
        SetEngineThrottle("Vertical4", appliedContinuousActions[3]);
        SetEngineThrottle("Horizontal1", appliedContinuousActions[4]);
        SetEngineThrottle("Horizontal2", appliedContinuousActions[5]);
        SetEngineThrottle("Horizontal3", appliedContinuousActions[6]);
        SetEngineThrottle("Horizontal4", appliedContinuousActions[7]);
    }

    private void ZeroThrusterOutput()
    {
        if (ShouldUseFinsRovThrusters())
        {
            ResolveFinsRovThrusterReferences();
            FinsROVAgentRuntime.ZeroThrusters(orderedFinsRovThrusters, thrusterCommandMode, actionForceScaleN);
            return;
        }

        SetEngineThrottle("Vertical1", 0f);
        SetEngineThrottle("Vertical2", 0f);
        SetEngineThrottle("Vertical3", 0f);
        SetEngineThrottle("Vertical4", 0f);
        SetEngineThrottle("Horizontal1", 0f);
        SetEngineThrottle("Horizontal2", 0f);
        SetEngineThrottle("Horizontal3", 0f);
        SetEngineThrottle("Horizontal4", 0f);
    }

    private void SetEngineThrottle(string engineName, float throttle)
    {
        Engine engine = GetEngineByName(engineName);
        if (engine != null)
        {
            engine.externalThrottleInput = Mathf.Clamp(throttle, -1f, 1f);
        }
    }

    private void ResolveFinsRovThrusterReferences()
    {
        if (finsRovThrusterController == null)
        {
            finsRovThrusterController = thrusterControllerOverride != null
                ? thrusterControllerOverride
                : GetComponent<MarusThrusterController>();
        }

        if (FinsROVAgentRuntime.TryResolveOrderedThrusters(
            this,
            finsRovThrusterController,
            autoResolveThrustersFromChildren,
            orderedFinsRovThrusters,
            out string statusMessage))
        {
            FinsROVAgentRuntime.EnsureThrusterControllerOrder(finsRovThrusterController, orderedFinsRovThrusters);
            if (logThrusterResolution)
            {
                Debug.Log($"[{nameof(ChaserAgent)}] {statusMessage}", this);
                logThrusterResolution = false;
            }
        }
        else if (logThrusterResolution)
        {
            Debug.LogError($"[{nameof(ChaserAgent)}] {statusMessage}", this);
        }
    }

    private void ResolveAppliedContinuousActions(ActionBuffers actions)
    {
        for (int i = 0; i < appliedContinuousActions.Length; i++)
        {
            float receivedAction = i < actions.ContinuousActions.Length
                ? Mathf.Clamp(actions.ContinuousActions[i], -1f, 1f)
                : 0f;
            receivedContinuousActions[i] = receivedAction;
            appliedContinuousActions[i] = receivedAction;
        }

        lastActionSource = "PythonMLAgents";

        NetSeparationStressTest separationStressTest =
            NetSeparationStressTest.ResolveActive(areaManager);
        if (separationStressTest != null
            && separationStressTest.TryGetChaserOverrideActions(this, baselineOverrideActions))
        {
            for (int i = 0; i < appliedContinuousActions.Length; i++)
            {
                appliedContinuousActions[i] = Mathf.Clamp(baselineOverrideActions[i], -1f, 1f);
            }

            lastActionSource = nameof(NetSeparationStressTest);

            return;
        }

        ThreeChaseOneBaselineController baselineController =
            ThreeChaseOneBaselineController.ResolveActive(areaManager);
        if (baselineController == null
            || !baselineController.ShouldOverrideChaserActions()
            || !baselineController.TryGetChaserOverrideActions(this, baselineOverrideActions))
        {
            return;
        }

        for (int i = 0; i < appliedContinuousActions.Length; i++)
        {
            appliedContinuousActions[i] = Mathf.Clamp(baselineOverrideActions[i], -1f, 1f);
        }

        lastActionSource = nameof(ThreeChaseOneBaselineController);
    }

    private void LogActionAuditIfEnabled()
    {
        if (!logActionAudit)
        {
            return;
        }

        actionAuditStep++;
        if (actionAuditStep % Mathf.Max(1, actionAuditInterval) != 0)
        {
            return;
        }

        Debug.Log(
            $"[ChaserActionAudit] agent={name} source={lastActionSource} " +
            $"received=[{string.Join(", ", receivedContinuousActions)}] " +
            $"applied=[{string.Join(", ", appliedContinuousActions)}] " +
            $"commandMode={thrusterCommandMode}",
            this);
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActionsOut = actionsOut.ContinuousActions;
        ClearContinuousActions(continuousActionsOut);

        ThreeChaseOneBaselineController baselineController =
            ThreeChaseOneBaselineController.ResolveActive(areaManager);
        if (baselineController != null
            && baselineController.TryFillChaserHeuristic(this, continuousActionsOut))
        {
            return;
        }

        if (FreeCameraDragController.IsInputActive || !enableKeyboardHeuristicInput)
        {
            shipInputActions?.Disable();
            return;
        }

        EnsureShipInputActionsEnabled();
        if (shipInputActions == null)
        {
            return;
        }

        var inputTestActions = new float[8] { 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f };
        //下面的键盘输入仍然使用老的shipoInputActions的键位映射获取，仅仅为了方便调试。实际上continuousActions的输出并非如此控制
        inputTestActions[0] = shipInputActions.ShipControls.Throttle.ReadValue<float>(); // SW
        inputTestActions[1] = shipInputActions.ShipControls.Throttle2.ReadValue<float>(); // 56
        inputTestActions[2] = shipInputActions.ShipControls.Throttle3.ReadValue<float>(); // 78
        inputTestActions[3] = shipInputActions.ShipControls.Throttle4.ReadValue<float>(); // 90
        inputTestActions[4] = shipInputActions.ShipControls.Steering.ReadValue<float>(); // AD

        // Debug.Log("Heuristic input actions: " + string.Join(", ", inputTestActions));

        // 下潜/上浮
        continuousActionsOut[0] += Mathf.Clamp(inputTestActions[1], -1f, 1f);
        continuousActionsOut[1] += Mathf.Clamp(inputTestActions[1], -1f, 1f);
        continuousActionsOut[2] += Mathf.Clamp(inputTestActions[1], -1f, 1f);
        continuousActionsOut[3] += Mathf.Clamp(inputTestActions[1], -1f, 1f);

        // 前进/后退
        continuousActionsOut[4] += Mathf.Clamp(inputTestActions[0], -1f, 1f);
        continuousActionsOut[7] += -Mathf.Clamp(inputTestActions[0], -1f, 1f);
        continuousActionsOut[5] += Mathf.Clamp(inputTestActions[0], -1f, 1f);
        continuousActionsOut[6] += -Mathf.Clamp(inputTestActions[0], -1f, 1f);

        //转向
        continuousActionsOut[4] += Mathf.Clamp(inputTestActions[4], -0.1f, 0.1f);
        continuousActionsOut[5] += Mathf.Clamp(inputTestActions[4], -0.1f, 0.1f);
        continuousActionsOut[6] += Mathf.Clamp(inputTestActions[4], -0.1f, 0.1f);
        continuousActionsOut[7] += Mathf.Clamp(inputTestActions[4], -0.1f, 0.1f);
    }

    private void EnsureShipInputActionsEnabled()
    {
        if (shipInputActions == null)
        {
            shipInputActions = new ShipInputActions();
        }

        shipInputActions.Enable();
    }

    private static void ClearContinuousActions(ActionSegment<float> continuousActions)
    {
        for (int i = 0; i < continuousActions.Length; i++)
        {
            continuousActions[i] = 0f;
        }
    }
}
