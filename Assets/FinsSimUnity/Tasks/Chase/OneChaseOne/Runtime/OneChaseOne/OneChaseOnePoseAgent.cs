using System;
using System.Text;
using FinsSim.Actuators;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;
using Random = UnityEngine.Random;

[DisallowMultipleComponent]
[RequireComponent(typeof(Rigidbody))]
public class OneChaseOnePoseAgent : Agent
{
    private const string RewardProfileIdKey = "finsim_1chase1_reward_profile";
    private const string RewardLabelArgName = "-fins-1chase1-reward-label";
    private const string RewardPrefix = "finsim_1c1.reward.";
    private const int ExpectedRewardProtocolVersion = 1;
    private const string RewardProtocolVersionKey = RewardPrefix + "protocol_version";
    private const string RewardModeKey = RewardPrefix + "mode";
    private const string RewardPerStepPenaltyKey = RewardPrefix + "per_step_penalty";
    private const string RewardDistanceProgressScaleKey = RewardPrefix + "distance_progress_scale";
    private const string RewardDistanceScaleKey = RewardPrefix + "distance_scale";
    private const string RewardDistanceRangeKey = RewardPrefix + "distance_range";
    private const string RewardHeadingAlignmentScaleKey = RewardPrefix + "heading_alignment_scale";
    private const string RewardApproachVelocityScaleKey = RewardPrefix + "approach_velocity_scale";
    private const string RewardAngularVelocityPenaltyScaleKey = RewardPrefix + "angular_velocity_penalty_scale";
    private const string RewardActionEnergyPenaltyScaleKey = RewardPrefix + "action_energy_penalty_scale";
    private const string RewardActionDeltaPenaltyScaleKey = RewardPrefix + "action_delta_penalty_scale";
    private const string RewardNearCaptureDistanceKey = RewardPrefix + "near_capture_distance";
    private const string RewardNearCaptureBonusScaleKey = RewardPrefix + "near_capture_bonus_scale";
    private const string RewardSuccessRewardKey = RewardPrefix + "success_reward";
    private const string RewardTimeoutPenaltyKey = RewardPrefix + "timeout_penalty";
    private const string RewardOutOfBoundsPenaltyKey = RewardPrefix + "out_of_bounds_penalty";
    private const string RewardCaptureDistanceKey = RewardPrefix + "capture_distance";
    private const string RewardStableCaptureRelativeSpeedKey = RewardPrefix + "stable_capture_relative_speed";
    private const string RewardStableCaptureStepsRequiredKey = RewardPrefix + "stable_capture_steps_required";
    private const string RewardSubgoalToPreyScaleKey = RewardPrefix + "subgoal_to_prey_scale";
    private const string RewardSubgoalToPreyRangeKey = RewardPrefix + "subgoal_to_prey_range";
    private const string RewardSubgoalStaleTimeoutSecKey = RewardPrefix + "subgoal_stale_timeout_sec";

    public enum ObservationMode
    {
        // 1Chase1 的标准高层观测接口：14D controller_body 局部追踪观测
        DirectLocal14 = 0,
    }

    public enum OneChaseOneRewardMode
    {
        DenseChase = 0,
        DistanceOnly = 1,
        DistancePlusSubgoal = 2,
        SparseCapture = 3,
    }

    private Rigidbody rigidBody;
    private ThrusterController thrusterController;
    private Transform referenceTransform;

    private readonly Thruster[] orderedThrusters = new Thruster[FinsROVAgentRuntime.DefaultThrusterOrder.Length];
    private readonly float[] currentActions = new float[FinsROVAgentRuntime.DefaultThrusterOrder.Length];
    private readonly float[] previousActions = new float[FinsROVAgentRuntime.DefaultThrusterOrder.Length];
    private readonly StringBuilder rewardSummaryBuilder = new StringBuilder(2048);

    [Header("References")]
    public Transform selfTransform;
    public Transform preyTransform;
    public OneChaseOnePreyController preyController;
    [SerializeField] private ObservationMode observationMode = ObservationMode.DirectLocal14;

    [Header("Thruster Control")]
    [SerializeField] private ThrusterController thrusterControllerOverride;
    [SerializeField] private ThrusterCommandMode thrusterCommandMode = ThrusterCommandMode.NormalizedMaxForceRequest;
    [SerializeField] private bool autoResolveThrustersFromChildren = true;
    [SerializeField] private bool logThrusterResolution = true;
    [SerializeField] private bool autoRequestDecisionWhenNoDecisionRequester = true;
    [SerializeField] private float heuristicYawMixScale = 1.0f;
    [SerializeField] private float heuristicAuxMixScale = 1.0f;

    [Header("Episode Reset")]
    [SerializeField] private int episodeMaxSteps = 3000;
    [SerializeField] private float horizontalSpawnRadius = 2.5f;
    [SerializeField] private float verticalSpawnJitter = 0.2f;
    [SerializeField] private float minInitialPreyDistance = 2.0f;
    [SerializeField] private float maxInitialPreyDistance = 5.0f;
    [SerializeField] private bool randomizeAgentYawOnReset = true;

    [Header("Reward")]
    [SerializeField] private int rewardProtocolVersion = ExpectedRewardProtocolVersion;
    [SerializeField] private OneChaseOneRewardMode rewardMode = OneChaseOneRewardMode.DenseChase;
    [SerializeField] private float perStepPenalty = -0.0005f;
    [SerializeField] private float distanceProgressRewardScale = 0.9f;
    [SerializeField] private float distanceRewardScale = 0.08f;
    [SerializeField] private float distanceRewardRange = 8.0f;
    [SerializeField] private float headingAlignmentRewardScale = 0.01f;
    [SerializeField] private float approachVelocityRewardScale = 0.08f;
    [SerializeField] private float angularVelocityPenaltyScale = 0.002f;
    [SerializeField] private float actionEnergyPenaltyScale = 0.001f;
    [SerializeField] private float actionDeltaPenaltyScale = 0.0015f;
    [SerializeField] private float nearCaptureDistance = 1.2f;
    [SerializeField] private float nearCaptureBonusScale = 0.03f;
    [SerializeField] private float successReward = 50.0f;
    [SerializeField] private float timeoutPenalty = -5.0f;
    [SerializeField] private float subgoalToPreyRewardScale = 0.0f;
    [SerializeField] private float subgoalToPreyRewardRange = 1.5f;
    [SerializeField] private float subgoalStaleTimeoutSeconds = 0.5f;

    [Header("Capture")]
    [SerializeField] private float captureDistance = 0.8f;
    [SerializeField] private float stableCaptureRelativeSpeed = 0.35f;
    [SerializeField] private int stableCaptureStepsRequired = 5;

    [Header("Safety")]
    [SerializeField] private float maxDistanceFromAnchor = 12.0f;
    [SerializeField] private float surfaceY = 0.0f;
    [SerializeField] private float maxHeightAboveSurface = 0.3f;
    [SerializeField] private float outOfBoundsPenalty = -1.0f;

    [Header("Diagnostics")]
    [SerializeField] private bool enableStatsRecorder = true;
    [SerializeField] private bool showRewardDebugHud = true;
    [SerializeField] private KeyCode toggleRewardDebugHudKey = KeyCode.F8;
    [SerializeField] private Vector2 rewardDebugPanelPosition = new Vector2(16f, 16f);
    [SerializeField] private Vector2 rewardDebugPanelSize = new Vector2(520f, 520f);
    [SerializeField] private int rewardDebugFontSize = 14;
    [SerializeField] private string rewardProfileDisplayName = "scene_default_dense_chase";
    [SerializeField] private bool logRewardConfigChanges = true;
    [SerializeField, TextArea(10, 28)] private string rewardRuntimeSummary = "";
    [SerializeField] private bool highLevelTargetDebugLive = false;
    [SerializeField] private int highLevelTargetDebugStep = -1;
    [SerializeField] private float highLevelTargetDebugAgeSeconds = -1f;
    [SerializeField] private Vector3 highLevelTargetDebugWorld = Vector3.zero;
    [SerializeField] private Vector3 highLevelTargetDebugLocalDelta = Vector3.zero;
    [SerializeField] private Vector3 highLevelTargetDebugCurrentWorld = Vector3.zero;

    private Vector3 initialAgentPosition;
    private Vector3 initialPreyPosition;
    private bool anchorsCaptured;
    private float previousDistanceToPrey;
    private float currentDistanceToPrey;
    private int stableCaptureSteps;
    private float lastStepReward;
    private float lastProgressReward;
    private float lastDistanceReward;
    private float lastHeadingAlignmentReward;
    private float lastApproachVelocityReward;
    private float lastAngularVelocityPenalty;
    private float lastActionEnergyPenalty;
    private float lastActionDeltaPenalty;
    private float lastNearCaptureBonus;
    private float lastSubgoalToPreyReward;
    private float lastTerminalReward;
    private float lastDistanceProgress;
    private float lastDistanceToCapture;
    private float lastRelativeSpeed;
    private float lastRovSpeed;
    private float lastPreySpeed;
    private float lastApproachSpeed;
    private float lastHeadingAlignment;
    private float lastActionEnergy;
    private float lastActionDelta;
    private Vector3 lastRovVelocityWorld;
    private Vector3 lastPreyVelocityWorld;
    private Vector3 lastRovVelocityBody;
    private Vector3 lastPreyVelocityBody;
    private bool lastCapturePoseSatisfied;
    private bool lastOutOfBounds;
    private bool lastSubgoalFresh;
    private float lastSubgoalError;
    private bool rewardCallbacksRegistered;
    private Vector2 rewardHudScrollPosition;
    private GUIStyle rewardHudPanelStyle;
    private GUIStyle rewardHudTitleStyle;
    private GUIStyle rewardHudSectionStyle;
    private GUIStyle rewardHudBodyStyle;
    private GUIStyle rewardHudHelpStyle;
    private Texture2D rewardHudBackgroundTexture;

    public float LastStepReward => lastStepReward;
    public float CurrentDistanceToPrey => currentDistanceToPrey;
    public ObservationMode CurrentObservationMode => observationMode;

    private Transform ReferenceTransform
    {
        get
        {
            if (referenceTransform == null)
            {
                referenceTransform = FinsROVAgentRuntime.ResolveReferenceTransform(transform, selfTransform);
            }

            return referenceTransform;
        }
    }

    private Rigidbody RigidBody
    {
        get
        {
            if (rigidBody == null)
            {
                rigidBody = GetComponent<Rigidbody>();
            }

            return rigidBody;
        }
    }

    private void Awake()
    {
        rigidBody = RigidBody;
        MaxStep = Mathf.Max(1, episodeMaxSteps);
        RegisterRewardCallbacksOnce();
        ApplyCurrentRewardParameters();
        ResolveRewardProfileDisplayName();
        RefreshRewardRuntimeSummary();
    }

    private void Start()
    {
        ResolveRuntimeReferences();
        CaptureEpisodeAnchors();
        RegisterRewardCallbacksOnce();
        ApplyCurrentRewardParameters();
        ResolveRewardProfileDisplayName();
        RefreshRewardRuntimeSummary();
    }

    private void FixedUpdate()
    {
        if (autoRequestDecisionWhenNoDecisionRequester)
        {
            RequestDecision();
        }
    }

    private void Update()
    {
        if (Input.GetKeyDown(toggleRewardDebugHudKey))
        {
            showRewardDebugHud = !showRewardDebugHud;
        }
    }

    private void OnDisable()
    {
        FinsROVAgentRuntime.ZeroThrusters(orderedThrusters, thrusterCommandMode);
    }

    public override void OnEpisodeBegin()
    {
        ResolveRuntimeReferences();
        CaptureEpisodeAnchors();
        MaxStep = Mathf.Max(1, episodeMaxSteps);

        // The parallel-area runtime places one coordinator on each FinsROV
        // scene instance. This is a no-op for the existing single-area scene.
        FinsROVAgentRuntime.RandomizeEpisodeIfPresent(this);

        RigidBody.angularVelocity = Vector3.zero;
        RigidBody.linearVelocity = Vector3.zero;
        FinsROVAgentRuntime.ZeroThrusters(orderedThrusters, thrusterCommandMode);

        Vector3 spawnPosition = SampleAgentSpawnPosition();
        transform.position = spawnPosition;
        if (randomizeAgentYawOnReset)
        {
            transform.rotation = Quaternion.Euler(0f, Random.Range(0f, 360f), 0f);
        }

        Vector3 preySpawnPosition = SamplePreySpawnPosition(spawnPosition);
        if (preyController != null)
        {
            preyController.ResetForEpisode(preySpawnPosition, Quaternion.identity);
        }
        else if (preyTransform != null)
        {
            preyTransform.position = preySpawnPosition;
            preyTransform.rotation = Quaternion.identity;
            if (preyTransform.TryGetComponent(out Rigidbody preyRb))
            {
                preyRb.linearVelocity = Vector3.zero;
                preyRb.angularVelocity = Vector3.zero;
            }
        }

        currentDistanceToPrey = GetDistanceToPrey();
        previousDistanceToPrey = currentDistanceToPrey;
        stableCaptureSteps = 0;
        lastStepReward = 0f;
        lastProgressReward = 0f;
        lastDistanceReward = 0f;
        lastHeadingAlignmentReward = 0f;
        lastApproachVelocityReward = 0f;
        lastAngularVelocityPenalty = 0f;
        lastActionEnergyPenalty = 0f;
        lastActionDeltaPenalty = 0f;
        lastNearCaptureBonus = 0f;
        lastSubgoalToPreyReward = 0f;
        lastTerminalReward = 0f;
        lastDistanceProgress = 0f;
        lastDistanceToCapture = currentDistanceToPrey;
        lastRelativeSpeed = 0f;
        lastRovSpeed = 0f;
        lastPreySpeed = 0f;
        lastApproachSpeed = 0f;
        lastHeadingAlignment = 0f;
        lastActionEnergy = 0f;
        lastActionDelta = 0f;
        lastRovVelocityWorld = Vector3.zero;
        lastPreyVelocityWorld = Vector3.zero;
        lastRovVelocityBody = Vector3.zero;
        lastPreyVelocityBody = Vector3.zero;
        lastCapturePoseSatisfied = false;
        lastOutOfBounds = false;
        lastSubgoalFresh = false;
        lastSubgoalError = 0f;
        Array.Clear(currentActions, 0, currentActions.Length);
        Array.Clear(previousActions, 0, previousActions.Length);
        RefreshRewardRuntimeSummary();

        if (autoRequestDecisionWhenNoDecisionRequester)
        {
            RequestDecision();
        }
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        Transform reference = ReferenceTransform;
        // Standard 1Chase1 observation contract (14D, controller_body):
        // [0:3]   self_linear_velocity_body
        // [3:6]   self_angular_velocity_body
        // [6:9]   fish_relative_position_body
        // [9:12]  fish_relative_velocity_body
        // [12]    distance_to_fish
        // [13]    bearing_to_fish
        Vector3 selfLinearVelocityBody = ControllerBodyFrame.WorldLinearVelocityToBody(reference, RigidBody.linearVelocity);
        Vector3 selfAngularVelocityBody = ControllerBodyFrame.WorldAngularVelocityToBody(reference, RigidBody.angularVelocity);
        Vector3 fishRelativePositionBody = Vector3.zero;
        Vector3 fishRelativeVelocityBody = Vector3.zero;

        if (preyTransform != null)
        {
            fishRelativePositionBody = ControllerBodyFrame.WorldToBodyPositionDelta(
                reference,
                preyTransform.position - reference.position
            );

            Vector3 preyVelocity = preyController != null ? preyController.CurrentVelocity : Vector3.zero;
            fishRelativeVelocityBody = ControllerBodyFrame.WorldLinearVelocityToBody(
                reference,
                preyVelocity - RigidBody.linearVelocity
            );
        }

        float distanceToFish = fishRelativePositionBody.magnitude;
        float bearingToFish = ControllerBodyFrame.HorizontalBearingRad(fishRelativePositionBody);

        sensor.AddObservation(selfLinearVelocityBody);
        sensor.AddObservation(selfAngularVelocityBody);
        sensor.AddObservation(fishRelativePositionBody);
        sensor.AddObservation(fishRelativeVelocityBody);
        sensor.AddObservation(distanceToFish);
        sensor.AddObservation(bearingToFish);
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        ResolveRuntimeReferences();

        int actionCount = Mathf.Min(currentActions.Length, actionBuffers.ContinuousActions.Length);
        for (int i = 0; i < currentActions.Length; i++)
        {
            currentActions[i] = i < actionCount
                ? Mathf.Clamp(actionBuffers.ContinuousActions[i], -1f, 1f)
                : 0f;
        }

        FinsROVAgentRuntime.ApplyThrusterActions(orderedThrusters, currentActions, thrusterCommandMode);

        Vector3 currentPosition = ReferenceTransform.position;
        Vector3 preyPosition = preyTransform != null ? preyTransform.position : currentPosition;
        Vector3 toPrey = preyPosition - currentPosition;
        float distanceToPrey = toPrey.magnitude;
        Vector3 directionToPrey = toPrey.sqrMagnitude > 1e-6f ? toPrey / distanceToPrey : ReferenceTransform.forward;

        Vector3 preyVelocity = preyController != null ? preyController.CurrentVelocity : Vector3.zero;
        Vector3 rovVelocity = RigidBody.linearVelocity;
        Vector3 relativeVelocity = rovVelocity - preyVelocity;
        float relativeSpeed = relativeVelocity.magnitude;
        float progress = previousDistanceToPrey - distanceToPrey;
        float approachSpeed = Vector3.Dot(relativeVelocity, directionToPrey);
        float headingAlignment = Mathf.Clamp01(Vector3.Dot(ReferenceTransform.forward.normalized, directionToPrey));

        float actionEnergy = 0f;
        float actionDelta = 0f;
        for (int i = 0; i < currentActions.Length; i++)
        {
            actionEnergy += Mathf.Abs(currentActions[i]);
            actionDelta += Mathf.Abs(currentActions[i] - previousActions[i]);
            previousActions[i] = currentActions[i];
        }

        actionEnergy /= currentActions.Length;
        actionDelta /= currentActions.Length;

        float progressReward = distanceProgressRewardScale * progress;
        float distanceReward = distanceRewardScale *
            Mathf.Clamp01(1f - distanceToPrey / Mathf.Max(0.1f, distanceRewardRange));
        float headingReward = headingAlignmentRewardScale * headingAlignment;
        float approachReward = approachVelocityRewardScale * Mathf.Clamp(approachSpeed, -1f, 1f);
        float angularVelocityPenalty = angularVelocityPenaltyScale * RigidBody.angularVelocity.magnitude;
        float actionEnergyPenalty = actionEnergyPenaltyScale * actionEnergy;
        float actionDeltaPenalty = actionDeltaPenaltyScale * actionDelta;

        float nearCaptureBonus = 0f;
        if (distanceToPrey <= Mathf.Max(captureDistance, nearCaptureDistance))
        {
            float nearCaptureRatio = 1f - distanceToPrey / Mathf.Max(0.1f, Mathf.Max(captureDistance, nearCaptureDistance));
            nearCaptureBonus = nearCaptureBonusScale * Mathf.Clamp01(nearCaptureRatio);
        }

        Vector3 fishRelativePositionBody = ControllerBodyFrame.WorldToBodyPositionDelta(ReferenceTransform, toPrey);
        float subgoalReward = ComputeSubgoalToPreyReward(fishRelativePositionBody);
        float commonPenalty =
            angularVelocityPenalty +
            actionEnergyPenalty +
            actionDeltaPenalty;

        float reward = perStepPenalty;
        switch (rewardMode)
        {
            case OneChaseOneRewardMode.DistanceOnly:
                progressReward = 0f;
                headingReward = 0f;
                approachReward = 0f;
                nearCaptureBonus = 0f;
                reward += distanceReward - commonPenalty;
                break;
            case OneChaseOneRewardMode.DistancePlusSubgoal:
                progressReward = 0f;
                headingReward = 0f;
                approachReward = 0f;
                nearCaptureBonus = 0f;
                reward += distanceReward + subgoalReward - commonPenalty;
                break;
            case OneChaseOneRewardMode.SparseCapture:
                progressReward = 0f;
                distanceReward = 0f;
                headingReward = 0f;
                approachReward = 0f;
                nearCaptureBonus = 0f;
                subgoalReward = 0f;
                reward -= commonPenalty;
                break;
            case OneChaseOneRewardMode.DenseChase:
            default:
                reward +=
                    progressReward +
                    distanceReward +
                    headingReward +
                    approachReward +
                    nearCaptureBonus -
                    commonPenalty;
                break;
        }

        AddReward(reward);
        lastStepReward = reward;
        currentDistanceToPrey = distanceToPrey;
        previousDistanceToPrey = distanceToPrey;
        lastProgressReward = progressReward;
        lastDistanceReward = distanceReward;
        lastHeadingAlignmentReward = headingReward;
        lastApproachVelocityReward = approachReward;
        lastAngularVelocityPenalty = angularVelocityPenalty;
        lastActionEnergyPenalty = actionEnergyPenalty;
        lastActionDeltaPenalty = actionDeltaPenalty;
        lastNearCaptureBonus = nearCaptureBonus;
        lastSubgoalToPreyReward = subgoalReward;
        lastTerminalReward = 0f;
        lastDistanceProgress = progress;
        lastDistanceToCapture = distanceToPrey;
        lastRelativeSpeed = relativeSpeed;
        lastRovSpeed = rovVelocity.magnitude;
        lastPreySpeed = preyVelocity.magnitude;
        lastApproachSpeed = approachSpeed;
        lastHeadingAlignment = headingAlignment;
        lastActionEnergy = actionEnergy;
        lastActionDelta = actionDelta;
        lastRovVelocityWorld = rovVelocity;
        lastPreyVelocityWorld = preyVelocity;
        lastRovVelocityBody = ControllerBodyFrame.WorldLinearVelocityToBody(ReferenceTransform, rovVelocity);
        lastPreyVelocityBody = ControllerBodyFrame.WorldLinearVelocityToBody(ReferenceTransform, preyVelocity);
        lastOutOfBounds = false;

        bool capturePoseSatisfied = distanceToPrey <= captureDistance && relativeSpeed <= stableCaptureRelativeSpeed;
        stableCaptureSteps = capturePoseSatisfied ? stableCaptureSteps + 1 : 0;
        lastCapturePoseSatisfied = capturePoseSatisfied;
        if (stableCaptureSteps >= Mathf.Max(1, stableCaptureStepsRequired))
        {
            AddReward(successReward);
            lastStepReward += successReward;
            lastTerminalReward = successReward;
            RecordTerminalStats(1);
            RefreshRewardRuntimeSummary();
            EndEpisode();
            return;
        }

        bool outOfBounds =
            Vector3.Distance(currentPosition, initialAgentPosition) > maxDistanceFromAnchor ||
            currentPosition.y > surfaceY + maxHeightAboveSurface;

        if (outOfBounds)
        {
            AddReward(outOfBoundsPenalty);
            lastStepReward += outOfBoundsPenalty;
            lastOutOfBounds = true;
            lastTerminalReward = outOfBoundsPenalty;
            RecordTerminalStats(2);
            RefreshRewardRuntimeSummary();
            EndEpisode();
            return;
        }

        if (StepCount >= MaxStep - 1)
        {
            AddReward(timeoutPenalty);
            lastStepReward += timeoutPenalty;
            lastTerminalReward = timeoutPenalty;
            RecordTerminalStats(3);
        }

        RefreshRewardRuntimeSummary();

        if (enableStatsRecorder)
        {
            RecordStepStats(distanceToPrey, relativeSpeed, headingAlignment, actionEnergy, actionDelta);
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        FinsROVAgentRuntime.WriteHeuristicActionsFromManualInput(
            actionsOut.ContinuousActions,
            currentActions,
            heuristicYawMixScale,
            heuristicAuxMixScale);
    }

    private float ComputeSubgoalToPreyReward(Vector3 fishRelativePositionBody)
    {
        lastSubgoalFresh =
            highLevelTargetDebugLive &&
            highLevelTargetDebugAgeSeconds >= 0f &&
            highLevelTargetDebugAgeSeconds <= Mathf.Max(0.01f, subgoalStaleTimeoutSeconds);
        if (!lastSubgoalFresh || subgoalToPreyRewardScale == 0f)
        {
            lastSubgoalError = 0f;
            return 0f;
        }

        float range = Mathf.Max(0.1f, subgoalToPreyRewardRange);
        Vector3 desiredDeltaBody = Vector3.ClampMagnitude(fishRelativePositionBody, range);
        lastSubgoalError = (highLevelTargetDebugLocalDelta - desiredDeltaBody).magnitude;
        return subgoalToPreyRewardScale * Mathf.Clamp01(1f - lastSubgoalError / range);
    }

    private void RecordStepStats(
        float distanceToPrey,
        float relativeSpeed,
        float headingAlignment,
        float actionEnergy,
        float actionDelta)
    {
        Academy.Instance.StatsRecorder.Add("OneChaseOne/distance_to_prey", distanceToPrey);
        Academy.Instance.StatsRecorder.Add("OneChaseOne/rov_speed", lastRovSpeed);
        Academy.Instance.StatsRecorder.Add("OneChaseOne/prey_speed", lastPreySpeed);
        Academy.Instance.StatsRecorder.Add("OneChaseOne/relative_speed", relativeSpeed);
        Academy.Instance.StatsRecorder.Add("OneChaseOne/stable_capture_steps", stableCaptureSteps);
        Academy.Instance.StatsRecorder.Add("OneChaseOne/heading_alignment", headingAlignment);
        Academy.Instance.StatsRecorder.Add("OneChaseOne/action_abs_mean", actionEnergy);
        Academy.Instance.StatsRecorder.Add("OneChaseOne/action_delta_mean", actionDelta);
    }

    // terminal_reason: 1=success, 2=out_of_bounds, 3=timeout.
    private void RecordTerminalStats(int terminalReason)
    {
        if (!enableStatsRecorder)
        {
            return;
        }

        Academy.Instance.StatsRecorder.Add("OneChaseOne/terminal_reason", terminalReason);
        Academy.Instance.StatsRecorder.Add("OneChaseOne/terminal_success", terminalReason == 1 ? 1f : 0f);
        Academy.Instance.StatsRecorder.Add("OneChaseOne/terminal_out_of_bounds", terminalReason == 2 ? 1f : 0f);
        Academy.Instance.StatsRecorder.Add("OneChaseOne/terminal_timeout", terminalReason == 3 ? 1f : 0f);
        Academy.Instance.StatsRecorder.Add("OneChaseOne/terminal_distance_to_prey", currentDistanceToPrey);
        Academy.Instance.StatsRecorder.Add("OneChaseOne/terminal_relative_speed", lastRelativeSpeed);
    }

    private void RegisterRewardCallbacksOnce()
    {
        if (rewardCallbacksRegistered)
        {
            return;
        }

        Academy.Instance.EnvironmentParameters.RegisterCallback(RewardProtocolVersionKey, ApplyRewardProtocolVersion);
        Academy.Instance.EnvironmentParameters.RegisterCallback(RewardModeKey, ApplyRewardMode);
        Academy.Instance.EnvironmentParameters.RegisterCallback(RewardPerStepPenaltyKey, value => perStepPenalty = value);
        Academy.Instance.EnvironmentParameters.RegisterCallback(RewardDistanceProgressScaleKey, value => distanceProgressRewardScale = value);
        Academy.Instance.EnvironmentParameters.RegisterCallback(RewardDistanceScaleKey, value => distanceRewardScale = value);
        Academy.Instance.EnvironmentParameters.RegisterCallback(RewardDistanceRangeKey, value => distanceRewardRange = Mathf.Max(0.1f, value));
        Academy.Instance.EnvironmentParameters.RegisterCallback(RewardHeadingAlignmentScaleKey, value => headingAlignmentRewardScale = value);
        Academy.Instance.EnvironmentParameters.RegisterCallback(RewardApproachVelocityScaleKey, value => approachVelocityRewardScale = value);
        Academy.Instance.EnvironmentParameters.RegisterCallback(RewardAngularVelocityPenaltyScaleKey, value => angularVelocityPenaltyScale = Mathf.Max(0f, value));
        Academy.Instance.EnvironmentParameters.RegisterCallback(RewardActionEnergyPenaltyScaleKey, value => actionEnergyPenaltyScale = Mathf.Max(0f, value));
        Academy.Instance.EnvironmentParameters.RegisterCallback(RewardActionDeltaPenaltyScaleKey, value => actionDeltaPenaltyScale = Mathf.Max(0f, value));
        Academy.Instance.EnvironmentParameters.RegisterCallback(RewardNearCaptureDistanceKey, value => nearCaptureDistance = Mathf.Max(0f, value));
        Academy.Instance.EnvironmentParameters.RegisterCallback(RewardNearCaptureBonusScaleKey, value => nearCaptureBonusScale = value);
        Academy.Instance.EnvironmentParameters.RegisterCallback(RewardSuccessRewardKey, value => successReward = value);
        Academy.Instance.EnvironmentParameters.RegisterCallback(RewardTimeoutPenaltyKey, value => timeoutPenalty = value);
        Academy.Instance.EnvironmentParameters.RegisterCallback(RewardOutOfBoundsPenaltyKey, value => outOfBoundsPenalty = value);
        Academy.Instance.EnvironmentParameters.RegisterCallback(RewardCaptureDistanceKey, value => captureDistance = Mathf.Max(0f, value));
        Academy.Instance.EnvironmentParameters.RegisterCallback(RewardStableCaptureRelativeSpeedKey, value => stableCaptureRelativeSpeed = Mathf.Max(0f, value));
        Academy.Instance.EnvironmentParameters.RegisterCallback(RewardStableCaptureStepsRequiredKey, value => stableCaptureStepsRequired = Mathf.Max(1, Mathf.RoundToInt(value)));
        Academy.Instance.EnvironmentParameters.RegisterCallback(RewardSubgoalToPreyScaleKey, value => subgoalToPreyRewardScale = value);
        Academy.Instance.EnvironmentParameters.RegisterCallback(RewardSubgoalToPreyRangeKey, value => subgoalToPreyRewardRange = Mathf.Max(0.1f, value));
        Academy.Instance.EnvironmentParameters.RegisterCallback(RewardSubgoalStaleTimeoutSecKey, value => subgoalStaleTimeoutSeconds = Mathf.Max(0.01f, value));

        rewardCallbacksRegistered = true;
    }

    private void ApplyCurrentRewardParameters()
    {
        ApplyRewardProtocolVersion(GetRewardParameter(RewardProtocolVersionKey, rewardProtocolVersion));
        ApplyRewardMode(GetRewardParameter(RewardModeKey, (float)rewardMode));
        perStepPenalty = GetRewardParameter(RewardPerStepPenaltyKey, perStepPenalty);
        distanceProgressRewardScale = GetRewardParameter(RewardDistanceProgressScaleKey, distanceProgressRewardScale);
        distanceRewardScale = GetRewardParameter(RewardDistanceScaleKey, distanceRewardScale);
        distanceRewardRange = Mathf.Max(0.1f, GetRewardParameter(RewardDistanceRangeKey, distanceRewardRange));
        headingAlignmentRewardScale = GetRewardParameter(RewardHeadingAlignmentScaleKey, headingAlignmentRewardScale);
        approachVelocityRewardScale = GetRewardParameter(RewardApproachVelocityScaleKey, approachVelocityRewardScale);
        angularVelocityPenaltyScale = Mathf.Max(0f, GetRewardParameter(RewardAngularVelocityPenaltyScaleKey, angularVelocityPenaltyScale));
        actionEnergyPenaltyScale = Mathf.Max(0f, GetRewardParameter(RewardActionEnergyPenaltyScaleKey, actionEnergyPenaltyScale));
        actionDeltaPenaltyScale = Mathf.Max(0f, GetRewardParameter(RewardActionDeltaPenaltyScaleKey, actionDeltaPenaltyScale));
        nearCaptureDistance = Mathf.Max(0f, GetRewardParameter(RewardNearCaptureDistanceKey, nearCaptureDistance));
        nearCaptureBonusScale = GetRewardParameter(RewardNearCaptureBonusScaleKey, nearCaptureBonusScale);
        successReward = GetRewardParameter(RewardSuccessRewardKey, successReward);
        timeoutPenalty = GetRewardParameter(RewardTimeoutPenaltyKey, timeoutPenalty);
        outOfBoundsPenalty = GetRewardParameter(RewardOutOfBoundsPenaltyKey, outOfBoundsPenalty);
        captureDistance = Mathf.Max(0f, GetRewardParameter(RewardCaptureDistanceKey, captureDistance));
        stableCaptureRelativeSpeed = Mathf.Max(0f, GetRewardParameter(RewardStableCaptureRelativeSpeedKey, stableCaptureRelativeSpeed));
        stableCaptureStepsRequired = Mathf.Max(1, Mathf.RoundToInt(GetRewardParameter(RewardStableCaptureStepsRequiredKey, stableCaptureStepsRequired)));
        subgoalToPreyRewardScale = GetRewardParameter(RewardSubgoalToPreyScaleKey, subgoalToPreyRewardScale);
        subgoalToPreyRewardRange = Mathf.Max(0.1f, GetRewardParameter(RewardSubgoalToPreyRangeKey, subgoalToPreyRewardRange));
        subgoalStaleTimeoutSeconds = Mathf.Max(0.01f, GetRewardParameter(RewardSubgoalStaleTimeoutSecKey, subgoalStaleTimeoutSeconds));

        if (logRewardConfigChanges)
        {
            Debug.Log(
                $"[{nameof(OneChaseOnePoseAgent)}] reward protocol={rewardProtocolVersion}, "
                + $"mode={rewardMode} ({(int)rewardMode})",
                this);
        }
    }

    private float GetRewardParameter(string key, float defaultValue)
    {
        return Academy.Instance.EnvironmentParameters.GetWithDefault(key, defaultValue);
    }

    private void ApplyRewardProtocolVersion(float value)
    {
        rewardProtocolVersion = Mathf.RoundToInt(value);
        if (rewardProtocolVersion != ExpectedRewardProtocolVersion)
        {
            Debug.LogWarning(
                $"[{nameof(OneChaseOnePoseAgent)}] Unsupported reward protocol version {rewardProtocolVersion}; "
                + $"expected {ExpectedRewardProtocolVersion}.",
                this);
        }
    }

    private void ApplyRewardMode(float value)
    {
        int modeId = Mathf.Clamp(
            Mathf.RoundToInt(value),
            0,
            (int)OneChaseOneRewardMode.SparseCapture);
        OneChaseOneRewardMode newMode = (OneChaseOneRewardMode)modeId;
        bool changed = newMode != rewardMode;
        rewardMode = newMode;
        rewardProfileDisplayName = GetRewardModeDisplayName(rewardMode);
        if (logRewardConfigChanges && changed)
        {
            Debug.Log($"[{nameof(OneChaseOnePoseAgent)}] reward mode={rewardMode} ({modeId})", this);
        }
    }

    private void ResolveRuntimeReferences()
    {
        referenceTransform = FinsROVAgentRuntime.ResolveReferenceTransform(transform, selfTransform);

        if (thrusterController == null)
        {
            thrusterController = thrusterControllerOverride != null
                ? thrusterControllerOverride
                : GetComponent<ThrusterController>();
        }

        if (FinsROVAgentRuntime.TryResolveOrderedThrusters(
            this,
            thrusterController,
            autoResolveThrustersFromChildren,
            orderedThrusters,
            out string statusMessage))
        {
            FinsROVAgentRuntime.EnsureThrusterControllerOrder(thrusterController, orderedThrusters);
            if (logThrusterResolution)
            {
                Debug.Log($"[{nameof(OneChaseOnePoseAgent)}] {statusMessage}", this);
                logThrusterResolution = false;
            }
        }
        else
        {
            Debug.LogError($"[{nameof(OneChaseOnePoseAgent)}] {statusMessage}", this);
        }
    }

    private void CaptureEpisodeAnchors()
    {
        if (anchorsCaptured)
        {
            return;
        }

        initialAgentPosition = transform.position;
        initialPreyPosition = preyTransform != null ? preyTransform.position : transform.position + transform.forward * 3f;
        anchorsCaptured = true;
    }

    public void SetObservationMode(ObservationMode mode)
    {
        observationMode = mode;
        RefreshRewardRuntimeSummary();
    }

    /// <summary>Rebinds scene-only references after TrainingAreaReplicator clones an area.</summary>
    public void ConfigureForParallelTrainingArea(
        Transform areaTransform,
        Transform areaPrey,
        OneChaseOnePreyController areaPreyController,
        float areaSurfaceY)
    {
        preyTransform = areaPrey;
        preyController = areaPreyController;
        surfaceY = areaSurfaceY;
        initialAgentPosition = transform.position;
        initialPreyPosition = areaPrey != null ? areaPrey.position : transform.position + transform.forward * 3f;
        anchorsCaptured = true;
        enableStatsRecorder = false;
        showRewardDebugHud = false;
        logRewardConfigChanges = false;
    }

    public void SetHighLevelTargetDebugState(
        bool live,
        int step,
        float ageSeconds,
        Vector3 targetWorld,
        Vector3 localDelta,
        Vector3 currentWorld)
    {
        bool changed =
            highLevelTargetDebugLive != live ||
            highLevelTargetDebugStep != step ||
            Mathf.Abs(highLevelTargetDebugAgeSeconds - ageSeconds) > 0.2f ||
            Vector3.SqrMagnitude(highLevelTargetDebugWorld - targetWorld) > 1e-8f ||
            Vector3.SqrMagnitude(highLevelTargetDebugLocalDelta - localDelta) > 1e-8f ||
            Vector3.SqrMagnitude(highLevelTargetDebugCurrentWorld - currentWorld) > 1e-8f;

        highLevelTargetDebugLive = live;
        highLevelTargetDebugStep = step;
        highLevelTargetDebugAgeSeconds = ageSeconds;
        highLevelTargetDebugWorld = targetWorld;
        highLevelTargetDebugLocalDelta = localDelta;
        highLevelTargetDebugCurrentWorld = currentWorld;

        if (changed)
        {
            RefreshRewardRuntimeSummary();
        }
    }

    private void OnGUI()
    {
        if (!showRewardDebugHud)
        {
            return;
        }

        EnsureRewardHudStyles();

        Rect panelRect = new Rect(
            rewardDebugPanelPosition.x,
            rewardDebugPanelPosition.y,
            rewardDebugPanelSize.x,
            rewardDebugPanelSize.y);

        GUI.Box(panelRect, GUIContent.none, rewardHudPanelStyle);

        Rect contentRect = new Rect(
            panelRect.x + 12f,
            panelRect.y + 12f,
            panelRect.width - 24f,
            panelRect.height - 24f);

        GUILayout.BeginArea(contentRect);
        GUILayout.Label("1Chase1 Reward Debug", rewardHudTitleStyle);
        GUILayout.Label(
            $"Profile: {rewardProfileDisplayName}   Mode: {observationMode}   Toggle: {toggleRewardDebugHudKey}",
            rewardHudHelpStyle);

        rewardHudScrollPosition = GUILayout.BeginScrollView(
            rewardHudScrollPosition,
            false,
            true,
            GUILayout.ExpandHeight(true));

        GUILayout.Label("Effective Reward Summary", rewardHudSectionStyle);
        GUILayout.TextArea(rewardRuntimeSummary, rewardHudBodyStyle, GUILayout.ExpandHeight(true));

        GUILayout.EndScrollView();
        GUILayout.EndArea();
    }

    private float GetDistanceToPrey()
    {
        return preyTransform != null
            ? Vector3.Distance(ReferenceTransform.position, preyTransform.position)
            : 0f;
    }

    private Vector3 SampleAgentSpawnPosition()
    {
        Vector2 planar = Random.insideUnitCircle * Mathf.Max(0f, horizontalSpawnRadius);
        float vertical = Random.Range(-Mathf.Abs(verticalSpawnJitter), Mathf.Abs(verticalSpawnJitter));
        return new Vector3(
            initialAgentPosition.x + planar.x,
            initialAgentPosition.y + vertical,
            initialAgentPosition.z + planar.y
        );
    }

    private Vector3 SamplePreySpawnPosition(Vector3 agentSpawnPosition)
    {
        float minDistance = Mathf.Max(0.1f, minInitialPreyDistance);
        float maxDistance = Mathf.Max(minDistance, maxInitialPreyDistance);
        Vector2 direction2D = Random.insideUnitCircle.normalized;
        if (direction2D.sqrMagnitude < 1e-6f)
        {
            direction2D = Vector2.right;
        }

        float radius = Random.Range(minDistance, maxDistance);
        return new Vector3(
            agentSpawnPosition.x + direction2D.x * radius,
            initialPreyPosition.y + Random.Range(-Mathf.Abs(verticalSpawnJitter), Mathf.Abs(verticalSpawnJitter)),
            agentSpawnPosition.z + direction2D.y * radius
        );
    }

    private void ResolveRewardProfileDisplayName()
    {
        int profileId = ResolveRewardProfileIdFromEnvironmentParameters();
        if (profileId >= 0)
        {
            rewardProfileDisplayName = GetRewardProfileName(profileId);
            return;
        }

        string[] args = Environment.GetCommandLineArgs();
        for (int i = 0; i < args.Length - 1; i++)
        {
            if (!string.Equals(args[i], RewardLabelArgName, StringComparison.OrdinalIgnoreCase))
            {
                continue;
            }

            if (!string.IsNullOrWhiteSpace(args[i + 1]))
            {
                rewardProfileDisplayName = args[i + 1].Trim();
                return;
            }
        }
    }

    private static int ResolveRewardProfileIdFromEnvironmentParameters()
    {
        try
        {
            float rawValue = Academy.Instance.EnvironmentParameters.GetWithDefault(RewardProfileIdKey, -1f);
            if (rawValue < 0f)
            {
                return -1;
            }

            return Mathf.RoundToInt(rawValue);
        }
        catch
        {
            return -1;
        }
    }

    private static string GetRewardProfileName(int profileId)
    {
        switch (profileId)
        {
            case 0:
                return "scene_default_dense_chase";
            case 1:
                return "hierarchy_chase_direct14";
            case 2:
                return "hybrid_pid_legacy_dense";
            case 3:
                return "direct_local14_dense";
            default:
                return $"custom_profile_{profileId}";
        }
    }

    private static string GetRewardModeDisplayName(OneChaseOneRewardMode mode)
    {
        switch (mode)
        {
            case OneChaseOneRewardMode.DistanceOnly:
                return "distance_only";
            case OneChaseOneRewardMode.DistancePlusSubgoal:
                return "distance_plus_subgoal";
            case OneChaseOneRewardMode.SparseCapture:
                return "sparse_capture";
            case OneChaseOneRewardMode.DenseChase:
            default:
                return "dense_chase";
        }
    }

    private void RefreshRewardRuntimeSummary()
    {
        rewardSummaryBuilder.Clear();
        rewardSummaryBuilder.AppendLine($"Reward profile: {rewardProfileDisplayName}");
        rewardSummaryBuilder.AppendLine($"Reward protocol: {rewardProtocolVersion}");
        rewardSummaryBuilder.AppendLine($"Reward mode: {rewardMode} ({(int)rewardMode})");
        rewardSummaryBuilder.AppendLine($"Observation mode: {observationMode}");
        rewardSummaryBuilder.AppendLine($"Step: {StepCount}/{Mathf.Max(1, MaxStep)}");
        rewardSummaryBuilder.AppendLine();

        rewardSummaryBuilder.AppendLine("High-level target debug");
        rewardSummaryBuilder.AppendLine($"  live            = {highLevelTargetDebugLive}");
        rewardSummaryBuilder.AppendLine($"  step            = {highLevelTargetDebugStep}");
        rewardSummaryBuilder.AppendLine($"  age_seconds     = {highLevelTargetDebugAgeSeconds:F3}");
        rewardSummaryBuilder.AppendLine(
            $"  target_world    = [{highLevelTargetDebugWorld.x:+0.000;-0.000;0.000}, {highLevelTargetDebugWorld.y:+0.000;-0.000;0.000}, {highLevelTargetDebugWorld.z:+0.000;-0.000;0.000}]");
        rewardSummaryBuilder.AppendLine(
            $"  local_delta     = [{highLevelTargetDebugLocalDelta.x:+0.000;-0.000;0.000}, {highLevelTargetDebugLocalDelta.y:+0.000;-0.000;0.000}, {highLevelTargetDebugLocalDelta.z:+0.000;-0.000;0.000}]");
        rewardSummaryBuilder.AppendLine(
            $"  current_world   = [{highLevelTargetDebugCurrentWorld.x:+0.000;-0.000;0.000}, {highLevelTargetDebugCurrentWorld.y:+0.000;-0.000;0.000}, {highLevelTargetDebugCurrentWorld.z:+0.000;-0.000;0.000}]");
        rewardSummaryBuilder.AppendLine();

        rewardSummaryBuilder.AppendLine("Current state");
        rewardSummaryBuilder.AppendLine($"  distance_to_prey = {lastDistanceToCapture:F3}");
        rewardSummaryBuilder.AppendLine($"  rov_speed_world  = {lastRovSpeed:F3} m/s  [{lastRovVelocityWorld.x:+0.000;-0.000;0.000}, {lastRovVelocityWorld.y:+0.000;-0.000;0.000}, {lastRovVelocityWorld.z:+0.000;-0.000;0.000}]");
        rewardSummaryBuilder.AppendLine($"  prey_speed_world = {lastPreySpeed:F3} m/s  [{lastPreyVelocityWorld.x:+0.000;-0.000;0.000}, {lastPreyVelocityWorld.y:+0.000;-0.000;0.000}, {lastPreyVelocityWorld.z:+0.000;-0.000;0.000}]");
        rewardSummaryBuilder.AppendLine($"  rov_vel_body     = [{lastRovVelocityBody.x:+0.000;-0.000;0.000}, {lastRovVelocityBody.y:+0.000;-0.000;0.000}, {lastRovVelocityBody.z:+0.000;-0.000;0.000}]");
        rewardSummaryBuilder.AppendLine($"  prey_vel_body    = [{lastPreyVelocityBody.x:+0.000;-0.000;0.000}, {lastPreyVelocityBody.y:+0.000;-0.000;0.000}, {lastPreyVelocityBody.z:+0.000;-0.000;0.000}]");
        rewardSummaryBuilder.AppendLine($"  relative_speed   = {lastRelativeSpeed:F3}");
        rewardSummaryBuilder.AppendLine($"  distance_delta   = {lastDistanceProgress:F4}");
        rewardSummaryBuilder.AppendLine($"  approach_speed   = {lastApproachSpeed:F4}");
        rewardSummaryBuilder.AppendLine($"  heading_align    = {lastHeadingAlignment:F4}");
        rewardSummaryBuilder.AppendLine($"  action_energy    = {lastActionEnergy:F4}");
        rewardSummaryBuilder.AppendLine($"  action_delta     = {lastActionDelta:F4}");
        rewardSummaryBuilder.AppendLine($"  stable_capture_steps = {stableCaptureSteps}/{Mathf.Max(1, stableCaptureStepsRequired)}");
        rewardSummaryBuilder.AppendLine($"  capture_pose_satisfied = {lastCapturePoseSatisfied}");
        rewardSummaryBuilder.AppendLine($"  out_of_bounds = {lastOutOfBounds}");
        rewardSummaryBuilder.AppendLine($"  subgoal_fresh = {lastSubgoalFresh}");
        rewardSummaryBuilder.AppendLine($"  subgoal_error = {lastSubgoalError:F4}");
        rewardSummaryBuilder.AppendLine();

        rewardSummaryBuilder.AppendLine("Last step reward breakdown");
        rewardSummaryBuilder.AppendLine($"  base_step_penalty      = {perStepPenalty:+0.0000;-0.0000;0.0000}");
        rewardSummaryBuilder.AppendLine($"  progress_reward        = {lastProgressReward:+0.0000;-0.0000;0.0000}");
        rewardSummaryBuilder.AppendLine($"  distance_reward        = {lastDistanceReward:+0.0000;-0.0000;0.0000}");
        rewardSummaryBuilder.AppendLine($"  heading_reward         = {lastHeadingAlignmentReward:+0.0000;-0.0000;0.0000}");
        rewardSummaryBuilder.AppendLine($"  approach_reward        = {lastApproachVelocityReward:+0.0000;-0.0000;0.0000}");
        rewardSummaryBuilder.AppendLine($"  near_capture_bonus     = {lastNearCaptureBonus:+0.0000;-0.0000;0.0000}");
        rewardSummaryBuilder.AppendLine($"  subgoal_to_prey_reward = {lastSubgoalToPreyReward:+0.0000;-0.0000;0.0000}");
        rewardSummaryBuilder.AppendLine($"  angular_vel_penalty    = {-lastAngularVelocityPenalty:+0.0000;-0.0000;0.0000}");
        rewardSummaryBuilder.AppendLine($"  action_energy_penalty  = {-lastActionEnergyPenalty:+0.0000;-0.0000;0.0000}");
        rewardSummaryBuilder.AppendLine($"  action_delta_penalty   = {-lastActionDeltaPenalty:+0.0000;-0.0000;0.0000}");
        rewardSummaryBuilder.AppendLine($"  terminal_reward        = {lastTerminalReward:+0.0000;-0.0000;0.0000}");
        rewardSummaryBuilder.AppendLine($"  total_last_step_reward = {lastStepReward:+0.0000;-0.0000;0.0000}");
        rewardSummaryBuilder.AppendLine();

        rewardSummaryBuilder.AppendLine("Configured reward weights");
        rewardSummaryBuilder.AppendLine($"  per_step_penalty               = {perStepPenalty}");
        rewardSummaryBuilder.AppendLine($"  distance_progress_scale        = {distanceProgressRewardScale}");
        rewardSummaryBuilder.AppendLine($"  distance_reward_scale          = {distanceRewardScale}");
        rewardSummaryBuilder.AppendLine($"  distance_reward_range          = {distanceRewardRange}");
        rewardSummaryBuilder.AppendLine($"  heading_alignment_scale        = {headingAlignmentRewardScale}");
        rewardSummaryBuilder.AppendLine($"  approach_velocity_scale        = {approachVelocityRewardScale}");
        rewardSummaryBuilder.AppendLine($"  angular_velocity_penalty_scale = {angularVelocityPenaltyScale}");
        rewardSummaryBuilder.AppendLine($"  action_energy_penalty_scale    = {actionEnergyPenaltyScale}");
        rewardSummaryBuilder.AppendLine($"  action_delta_penalty_scale     = {actionDeltaPenaltyScale}");
        rewardSummaryBuilder.AppendLine($"  near_capture_distance          = {nearCaptureDistance}");
        rewardSummaryBuilder.AppendLine($"  near_capture_bonus_scale       = {nearCaptureBonusScale}");
        rewardSummaryBuilder.AppendLine($"  success_reward                 = {successReward}");
        rewardSummaryBuilder.AppendLine($"  timeout_penalty                = {timeoutPenalty}");
        rewardSummaryBuilder.AppendLine($"  out_of_bounds_penalty          = {outOfBoundsPenalty}");
        rewardSummaryBuilder.AppendLine($"  subgoal_to_prey_scale          = {subgoalToPreyRewardScale}");
        rewardSummaryBuilder.AppendLine($"  subgoal_to_prey_range          = {subgoalToPreyRewardRange}");
        rewardSummaryBuilder.AppendLine($"  subgoal_stale_timeout_sec      = {subgoalStaleTimeoutSeconds}");
        rewardSummaryBuilder.AppendLine();

        rewardSummaryBuilder.AppendLine("Capture thresholds");
        rewardSummaryBuilder.AppendLine($"  capture_distance               = {captureDistance}");
        rewardSummaryBuilder.AppendLine($"  stable_capture_relative_speed  = {stableCaptureRelativeSpeed}");
        rewardSummaryBuilder.AppendLine($"  stable_capture_steps_required  = {stableCaptureStepsRequired}");

        rewardRuntimeSummary = rewardSummaryBuilder.ToString();
    }

    private void EnsureRewardHudStyles()
    {
        if (rewardHudPanelStyle != null && rewardHudBodyStyle != null)
        {
            return;
        }

        rewardHudBackgroundTexture = new Texture2D(1, 1);
        rewardHudBackgroundTexture.SetPixel(0, 0, new Color(0.06f, 0.08f, 0.10f, 0.90f));
        rewardHudBackgroundTexture.Apply();

        rewardHudPanelStyle = new GUIStyle(GUI.skin.box);
        rewardHudPanelStyle.padding = new RectOffset(12, 12, 12, 12);
        rewardHudPanelStyle.normal.background = rewardHudBackgroundTexture;

        rewardHudTitleStyle = new GUIStyle(GUI.skin.label);
        rewardHudTitleStyle.fontSize = rewardDebugFontSize + 2;
        rewardHudTitleStyle.fontStyle = FontStyle.Bold;
        rewardHudTitleStyle.wordWrap = true;
        rewardHudTitleStyle.normal.textColor = new Color(0.93f, 0.96f, 1.0f);

        rewardHudSectionStyle = new GUIStyle(GUI.skin.label);
        rewardHudSectionStyle.fontSize = rewardDebugFontSize;
        rewardHudSectionStyle.fontStyle = FontStyle.Bold;
        rewardHudSectionStyle.wordWrap = true;
        rewardHudSectionStyle.normal.textColor = new Color(0.67f, 0.85f, 1.0f);

        rewardHudHelpStyle = new GUIStyle(GUI.skin.label);
        rewardHudHelpStyle.fontSize = rewardDebugFontSize - 1;
        rewardHudHelpStyle.wordWrap = true;
        rewardHudHelpStyle.normal.textColor = new Color(0.78f, 0.82f, 0.87f);

        rewardHudBodyStyle = new GUIStyle(GUI.skin.textArea);
        rewardHudBodyStyle.fontSize = rewardDebugFontSize;
        rewardHudBodyStyle.wordWrap = false;
        rewardHudBodyStyle.richText = false;
        rewardHudBodyStyle.alignment = TextAnchor.UpperLeft;
        rewardHudBodyStyle.normal.textColor = new Color(0.92f, 0.95f, 1.0f);
    }
}
