using FinsSim.Actuators;
using FinsSim.Hydrodynamics;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

[DisallowMultipleComponent]
[RequireComponent(typeof(Rigidbody))]
public class HoldForPosition : Agent
{
    public const int VectorObservationSize = 16;
    public const int ContinuousActionSize = 8;
    const string RewardParameterPrefix = "finsim_hold.reward.";
    const string DebugParameterPrefix = "finsim_hold.debug.";

    public enum InitialLinearVelocityRandomizationMode
    {
        IndependentAxes,
        SingleRandomAxis,
    }

    public enum DenseRewardMode
    {
        InspectorDefault = 0,
        // FinsSim's extended Learn-to-Swim goal-hold reward. It adds explicit
        // tracking/smoothness penalties, near-goal angular damping, and a
        // stable-hold bonus to the Isaac-style exponential terms.
        LearnToSwimGoalHold = 1,
        // Isaac WarpAUV-style position-hold reward: position, attitude, and
        // action-energy exponentials only. Unity evaluates action energy over
        // its eight thruster actions rather than Isaac's six actions.
        IsaacPositionHold = 2,
        // Isaac WarpAUV-style reward plus near-goal action-energy and angular
        // velocity costs. Unlike LearnToSwimGoalHold, it has no tracking,
        // action-delta, or stable-hold additions.
        IsaacPositionHoldNearGoalDamping = 3,
    }

    Rigidbody rigidBody;
    ThrusterController thrusterController;
    HydrodynamicsController hydrodynamicsController;
    Transform referenceTransform;

    readonly Thruster[] orderedThrusters = new Thruster[FinsROVAgentRuntime.DefaultThrusterOrder.Length];
    readonly float[] currentActions = new float[FinsROVAgentRuntime.DefaultThrusterOrder.Length];
    readonly float[] previousActions = new float[FinsROVAgentRuntime.DefaultThrusterOrder.Length];

    [Header("Reference Frame")]
    [Tooltip("If empty, uses child MarkPosition; otherwise uses this Transform.")]
    public Transform selfTransform;
    public Transform targetTransform;

    [Header("Thruster Control")]
    [SerializeField] ThrusterController thrusterControllerOverride;
    [SerializeField] ThrusterCommandMode thrusterCommandMode = ThrusterCommandMode.NormalizedMaxForceRequest;
    [ShowWhenThrusterCommandMode(ThrusterCommandMode.ScaledForceRequest)]
    [SerializeField] float actionForceScaleN = 7f;
    [Tooltip("Absolute limit applied to every normalized thruster action before it is converted to a force request. Keep one for the nominal task.")]
    [Range(0f, 1f)]
    [SerializeField] float normalizedThrusterActionLimit = 1f;
    [SerializeField] bool autoResolveThrustersFromChildren = true;
    [SerializeField] bool logThrusterResolution = true;
    [SerializeField] float heuristicYawMixScale = 1f;
    [SerializeField] float heuristicAuxMixScale = 1f;
    [SerializeField] bool autoRequestDecisionWhenNoDecisionRequester = true;
    [SerializeField] bool requestDecisionOnEpisodeBegin;
    Transform parallelAreaTransform;
    System.Random parallelEpisodeRandom;
    int parallelAreaSeedOffset;
    int parallelEpisodeIndex;

    [Header("Episode Reset")]
    [Tooltip("The existing DecisionPeriod=5 and fixed timestep=0.02 require MaxStep=3000 for a 60 second episode.")]
    [SerializeField] int recommendedMaxStep = 3000;
    [SerializeField] float spawnPositionRange = 2f;
    [SerializeField] float spawnDepthOffset = -2f;
    [SerializeField] float targetSpawnRadius = 3f;
    [SerializeField] float surfaceY = 0f;
    [SerializeField] float initialRollPitchRangeDeg = 20f;
    [Tooltip("IndependentAxes samples all three local velocity axes. SingleRandomAxis samples one local axis per episode, limiting total initial speed to this range.")]
    [SerializeField] InitialLinearVelocityRandomizationMode initialLinearVelocityRandomizationMode = InitialLinearVelocityRandomizationMode.SingleRandomAxis;
    [SerializeField] float initialLinearVelocityRange = 0.30f;
    [SerializeField] float initialAngularVelocityRangeRadPerSec = 0.35f;
    [Tooltip("Logs the sampled spawn/target geometry at every reset. Keep disabled for training; an evaluator may enable it through finsim_hold.debug.log_reset_target.")]
    [SerializeField] bool logResetTargetSampling;

    [Header("Observation Normalization")]
    [SerializeField] float linearVelocityObservationScale = 1f;
    [SerializeField] float angularVelocityObservationScale = 1f;
    [SerializeField] float normalizedVelocityObservationClip = 2f;

    [Header("Dense Reward Mode")]
    [SerializeField] DenseRewardMode denseRewardMode = DenseRewardMode.InspectorDefault;
    [SerializeField] float positionRewardScale = 0.2f;
    [SerializeField] float attitudeRewardScale = 0.5f;
    [SerializeField] float actionRewardScale = 0.2f;

    [Header("Learn To Swim Goal-Hold Reward")]
    [Tooltip("Read float overrides from ML-Agents EnvironmentParametersChannel at every episode reset.")]
    [SerializeField] bool enableRewardParameterOverrides = true;
    [SerializeField] float learnToSwimPositionErrorExponent = 1f;
    [SerializeField] float learnToSwimAttitudeErrorExponent = 1f;
    [SerializeField] float learnToSwimActionEnergyExponent = 1f;
    [Tooltip("Absolute distance penalty supplies a non-vanishing gradient and removes static position error.")]
    [SerializeField] float distancePenaltyScale = 1.0f;
    [SerializeField] float attitudePenaltyScale = 0.05f;
    [Tooltip("Uses mean squared change between consecutive eight-thruster actions.")]
    [SerializeField] float actionDeltaPenaltyScale = 0.02f;
    [SerializeField] float nearGoalRadius = 0.75f;
    [Tooltip("Within nearGoalRadius, subtracts this scale times the mean squared thruster action. This makes sustained high thrust unprofitable while holding position.")]
    [SerializeField] float nearGoalActionEnergyPenaltyScale = 0.60f;
    [Tooltip("Within nearGoalRadius, subtracts this scale times the mean squared change between consecutive thruster actions. This targets high-frequency oscillation without penalizing a steady hover trim.")]
    [SerializeField] float nearGoalActionDeltaPenaltyScale = 0.05f;
    [SerializeField] float nearGoalAngularVelocityPenaltyScale = 0.03f;
    [SerializeField] float nearGoalAngularVelocityPenaltyMaxRadPerSec = 5f;
    [SerializeField] float holdRewardScale = 0.10f;
    [SerializeField] float holdAngularVelocityThresholdRadPerSec = 0.25f;

    [Header("Episode Termination")]
    [SerializeField] float maxDistanceFromSpawn = 8f;
    [SerializeField] float maxHeightAboveSurface = 0.3f;
    [SerializeField] float outOfBoundsPenalty = -1f;

    [Header("Hold Diagnostics")]
    [SerializeField] float holdPositionThreshold = 0.05f;
    [SerializeField] float holdAttitudeThresholdDeg = 10f;

    [Header("Runtime Diagnostics")]
    [Tooltip("Also emits aggregated train/eval TensorBoard metrics when running a batch player.")]
    [SerializeField] bool enableStatsRecorder;
    [SerializeField] float currentPositionError;
    [SerializeField] float currentAttitudeErrorDeg;
    [SerializeField] Vector3 currentLocalLinearVelocity;
    [SerializeField] Vector3 currentLocalAngularVelocity;
    [SerializeField] float currentActionRms;
    [SerializeField] float currentPositionReward;
    [SerializeField] float currentAttitudeReward;
    [SerializeField] float currentActionReward;
    [SerializeField] float currentDistancePenalty;
    [SerializeField] float currentAttitudePenalty;
    [SerializeField] float currentActionDeltaPenalty;
    [SerializeField] float currentNearGoalActionEnergyPenalty;
    [SerializeField] float currentNearGoalActionDeltaPenalty;
    [SerializeField] float currentNearGoalAngularVelocityPenalty;
    [SerializeField] float currentHoldReward;
    [SerializeField] float currentActionDeltaRms;
    [SerializeField] float currentNearGoalWeight;
    [SerializeField] float currentTargetHoldWindow;
    [SerializeField] float lastStepReward;
    [SerializeField] DenseRewardMode activeDenseRewardMode;

    Vector3 episodeSpawnPosition;
    bool hasDecisionRequester;
    float activePositionRewardScale;
    float activeAttitudeRewardScale;
    float activeActionRewardScale;
    float activePositionErrorExponent;
    float activeAttitudeErrorExponent;
    float activeActionEnergyExponent;
    float activeDistancePenaltyScale;
    float activeAttitudePenaltyScale;
    float activeActionDeltaPenaltyScale;
    float activeNearGoalRadius;
    float activeNearGoalActionEnergyPenaltyScale;
    float activeNearGoalActionDeltaPenaltyScale;
    float activeNearGoalAngularVelocityPenaltyScale;
    float activeNearGoalAngularVelocityPenaltyMaxRadPerSec;
    float activeHoldRewardScale;
    float activeHoldPositionThreshold;
    float activeHoldAttitudeThresholdDeg;
    float activeHoldAngularVelocityThresholdRadPerSec;

    public float LastStepReward => lastStepReward;
    public float CurrentPositionError => currentPositionError;
    public float CurrentAttitudeErrorDeg => currentAttitudeErrorDeg;
    public Vector3 CurrentLocalLinearVelocity => currentLocalLinearVelocity;
    public Vector3 CurrentLocalAngularVelocity => currentLocalAngularVelocity;
    public float CurrentActionRms => currentActionRms;
    public bool IsInsideTargetHoldWindow => currentTargetHoldWindow > 0.5f;
    public int RecommendedMaxStep => recommendedMaxStep;
    public ThrusterCommandMode CurrentThrusterCommandMode => thrusterCommandMode;
    public float NormalizedThrusterActionLimit => normalizedThrusterActionLimit;
    public DenseRewardMode CurrentDenseRewardMode => denseRewardMode;

    Transform ReferenceTransform
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

    Rigidbody RigidBody
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

    void Start()
    {
        rigidBody = RigidBody;
        thrusterController = thrusterControllerOverride != null
            ? thrusterControllerOverride
            : GetComponent<ThrusterController>();
        hydrodynamicsController = GetComponent<HydrodynamicsController>();
        hasDecisionRequester = GetComponent("DecisionRequester") != null;
        ResolveRuntimeReferences();
        ResolveDenseRewardConfiguration();
    }

    void FixedUpdate()
    {
        if (autoRequestDecisionWhenNoDecisionRequester && !hasDecisionRequester)
        {
            RequestDecision();
        }
    }

    public override void OnEpisodeBegin()
    {
        ResolveRuntimeReferences();
        ResolveDenseRewardConfiguration();
        FinsROVAgentRuntime.RandomizeEpisodeIfPresent(this);
        parallelEpisodeRandom = CreateParallelEpisodeRandom();

        FinsROVAgentRuntime.ZeroThrusters(orderedThrusters, thrusterCommandMode, actionForceScaleN);
        RigidBody.linearVelocity = Vector3.zero;
        RigidBody.angularVelocity = Vector3.zero;

        RigidBody.position = ParallelAreaOrigin + new Vector3(
            RandomRange(0f, spawnPositionRange),
            RandomRange(spawnDepthOffset, spawnDepthOffset + spawnPositionRange),
            RandomRange(0f, spawnPositionRange));
        RigidBody.rotation = Quaternion.Euler(
            RandomRange(-initialRollPitchRangeDeg, initialRollPitchRangeDeg),
            RandomRange(0f, 360f),
            RandomRange(-initialRollPitchRangeDeg, initialRollPitchRangeDeg));
        Physics.SyncTransforms();
        episodeSpawnPosition = ReferenceTransform.position;

        if (targetTransform != null)
        {
            Vector3 targetPosition = GenerateRandomTargetPosition(
                targetSpawnRadius,
                episodeSpawnPosition,
                ParallelAreaOrigin.y + surfaceY,
                out Vector3 rawTargetPosition,
                out bool surfaceClamped);
            targetTransform.position = targetPosition;
            targetTransform.rotation = Quaternion.Euler(0f, RandomRange(0f, 360f), 0f);
            LogResetTargetSampleIfEnabled(
                episodeSpawnPosition,
                rawTargetPosition,
                targetPosition,
                ParallelAreaOrigin.y + surfaceY,
                surfaceClamped);
        }

        Vector3 initialLocalLinearVelocity = SampleInitialLocalLinearVelocity();
        Vector3 initialLocalAngularVelocity = RandomInsideBox(initialAngularVelocityRangeRadPerSec);
        RigidBody.linearVelocity = FinsROVAgentRuntime.GetWorldLinearVelocity(ReferenceTransform, initialLocalLinearVelocity);
        RigidBody.angularVelocity = FinsROVAgentRuntime.GetWorldAngularVelocity(ReferenceTransform, initialLocalAngularVelocity);
        hydrodynamicsController?.ResetBackendState();
        RigidBody.WakeUp();

        System.Array.Clear(currentActions, 0, currentActions.Length);
        System.Array.Clear(previousActions, 0, previousActions.Length);
        ResetDiagnostics();

        if (requestDecisionOnEpisodeBegin)
        {
            RequestDecision();
        }
    }

    /// <summary>Configure the agent for a TrainingAreaReplicator clone.</summary>
    public void ConfigureForParallelTrainingArea(
        Transform areaTransform,
        Transform areaTarget,
        int areaSeedOffset)
    {
        parallelAreaTransform = areaTransform;
        targetTransform = areaTarget;
        parallelAreaSeedOffset = areaSeedOffset;
        parallelEpisodeIndex = 0;
        requestDecisionOnEpisodeBegin = true;
        enableStatsRecorder = false;
    }

    /// <summary>Sets the ML-Agents physics-step episode limit for a generated training scene.</summary>
    public void ConfigureEpisodeMaxStep(int maxPhysicsSteps)
    {
        MaxStep = Mathf.Max(1, maxPhysicsSteps);
        recommendedMaxStep = MaxStep;
    }

    /// <summary>
    /// Configures the policy action ABI for a scene instance.  In
    /// NormalizedMaxForceRequest mode, a signed policy action maps to this
    /// specific thruster's current calibrated forward/reverse force limit.
    /// </summary>
    public void ConfigureThrusterCommandMode(ThrusterCommandMode commandMode)
    {
        thrusterCommandMode = commandMode;
    }

    /// <summary>
    /// Selects the serialized dense-reward family for a generated scene.
    /// Environment-parameter overrides, when enabled for an experiment, may
    /// still refine its weights without changing the scene's declared default.
    /// </summary>
    public void ConfigureDenseRewardMode(DenseRewardMode mode)
    {
        denseRewardMode = mode;
        activeDenseRewardMode = mode;
    }

    /// <summary>
    /// Sets the symmetric direct-thruster policy envelope. The limit is applied
    /// before each normalized command is mapped to that thruster's calibrated
    /// forward/reverse force capacity.
    /// </summary>
    public void ConfigureNormalizedThrusterActionLimit(float limit)
    {
        normalizedThrusterActionLimit = Mathf.Clamp01(limit);
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        Transform reference = ReferenceTransform;
        float positionScale = Mathf.Max(targetSpawnRadius, 0.01f);
        Vector3 localTargetOffset = targetTransform != null
            ? reference.InverseTransformPoint(targetTransform.position)
            : Vector3.zero;
        Quaternion relativeTargetRotation = targetTransform != null
            ? Quaternion.Inverse(reference.rotation) * targetTransform.rotation
            : Quaternion.identity;

        sensor.AddObservation(localTargetOffset / positionScale);
        AddRotation6DObservation(sensor, relativeTargetRotation);
        AddNormalizedVectorObservation(sensor, GetLocalLinearVelocity(), linearVelocityObservationScale);
        AddNormalizedVectorObservation(sensor, GetLocalAngularVelocity(), angularVelocityObservationScale);
        sensor.AddObservation(Mathf.Clamp01(localTargetOffset.magnitude / positionScale));
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        ResolveRuntimeReferences();
        ReadAndApplyActions(actionBuffers.ContinuousActions);

        float positionError = targetTransform != null
            ? Vector3.Distance(ReferenceTransform.position, targetTransform.position)
            : 0f;
        float attitudeErrorRadians = targetTransform != null
            ? Quaternion.Angle(ReferenceTransform.rotation, targetTransform.rotation) * Mathf.Deg2Rad
            : 0f;
        float meanSquaredAction = ComputeMeanSquaredAction();
        float actionEnergy = ComputeActionEnergy();
        float meanSquaredActionDelta = ComputeMeanSquaredActionDelta();
        Vector3 localAngularVelocity = GetLocalAngularVelocity();
        float angularVelocityEnergy = localAngularVelocity.sqrMagnitude;
        float angularSpeed = localAngularVelocity.magnitude;

        float positionReward;
        float attitudeReward;
        float actionReward;
        float distancePenalty;
        float attitudePenalty;
        float actionDeltaPenalty;
        float nearGoalActionEnergyPenalty;
        float nearGoalActionDeltaPenalty;
        float nearGoalAngularVelocityPenalty;
        float holdReward;
        float nearGoalWeight;
        bool insideTargetHoldWindow;
        if (activeDenseRewardMode == DenseRewardMode.LearnToSwimGoalHold)
        {
            // WarpAUV's dense exponentials keep reward smooth. The signed
            // tracking penalties retain gradient at large error and prevent
            // action-energy reward from creating a static offset.
            positionReward = activePositionRewardScale * Mathf.Exp(
                -activePositionErrorExponent * positionError * positionError);
            attitudeReward = activeAttitudeRewardScale * Mathf.Exp(
                -activeAttitudeErrorExponent * attitudeErrorRadians);
            actionReward = activeActionRewardScale * Mathf.Exp(
                -activeActionEnergyExponent * actionEnergy);
            distancePenalty = -activeDistancePenaltyScale * positionError;
            attitudePenalty = -activeAttitudePenaltyScale * attitudeErrorRadians;
            actionDeltaPenalty = -activeActionDeltaPenaltyScale * meanSquaredActionDelta;
            nearGoalWeight = ComputeNearGoalWeight(positionError, activeNearGoalRadius);
            // The exponential action term only removes a small positive reward
            // at full thrust.  Use an explicit near-goal cost so an agent
            // cannot turn high-frequency, saturated thrust into a profitable
            // position-hold oscillation. Mean squared action is in [0, 1],
            // making the scale independent of the eight-thruster action count.
            nearGoalActionEnergyPenalty = -activeNearGoalActionEnergyPenaltyScale * nearGoalWeight * meanSquaredAction;
            nearGoalActionDeltaPenalty = 0f;
            float cappedAngularVelocityEnergy = Mathf.Min(
                angularVelocityEnergy,
                activeNearGoalAngularVelocityPenaltyMaxRadPerSec * activeNearGoalAngularVelocityPenaltyMaxRadPerSec);
            nearGoalAngularVelocityPenalty = -activeNearGoalAngularVelocityPenaltyScale * nearGoalWeight * cappedAngularVelocityEnergy;
            insideTargetHoldWindow = positionError <= activeHoldPositionThreshold &&
                                      attitudeErrorRadians <= activeHoldAttitudeThresholdDeg * Mathf.Deg2Rad &&
                                      angularSpeed <= activeHoldAngularVelocityThresholdRadPerSec;
            holdReward = insideTargetHoldWindow ? activeHoldRewardScale : 0f;
        }
        else if (activeDenseRewardMode == DenseRewardMode.IsaacPositionHold ||
                 activeDenseRewardMode == DenseRewardMode.IsaacPositionHoldNearGoalDamping)
        {
            // Keep the Isaac WarpAUV position-hold reward structure exactly
            // for mode 2. Mode 3 starts with that same base reward and adds
            // only the near-goal damping costs below.
            // 0.2 exp(-||position_error||^2) + 0.5 exp(-attitude_error)
            // + 0.2 exp(-||action||^2) + 0.0 angular-velocity reward.
            // The Unity task exposes eight thruster actions, so ||action||^2
            // is evaluated over those eight values instead of Isaac's six.
            positionReward = 0.2f * Mathf.Exp(-positionError * positionError);
            attitudeReward = 0.5f * Mathf.Exp(-attitudeErrorRadians);
            actionReward = 0.2f * Mathf.Exp(-actionEnergy);
            distancePenalty = 0f;
            attitudePenalty = 0f;
            actionDeltaPenalty = 0f;
            holdReward = 0f;
            insideTargetHoldWindow = false;
            if (activeDenseRewardMode == DenseRewardMode.IsaacPositionHoldNearGoalDamping)
            {
                nearGoalWeight = ComputeNearGoalWeight(positionError, activeNearGoalRadius);
                // Keep the direct energy cost normalized by action count so
                // the mode-3 coefficient has the same [0, 1] interpretation
                // as in mode 1.
                nearGoalActionEnergyPenalty = -activeNearGoalActionEnergyPenaltyScale * nearGoalWeight * meanSquaredAction;
                // Do not penalize the steady trim needed to counter buoyancy or
                // other persistent biases. Penalize only changes in thrust so
                // a high-frequency switching policy is unattractive.
                nearGoalActionDeltaPenalty = -activeNearGoalActionDeltaPenaltyScale * nearGoalWeight * meanSquaredActionDelta;
                float cappedAngularVelocityEnergy = Mathf.Min(
                    angularVelocityEnergy,
                    activeNearGoalAngularVelocityPenaltyMaxRadPerSec * activeNearGoalAngularVelocityPenaltyMaxRadPerSec);
                nearGoalAngularVelocityPenalty = -activeNearGoalAngularVelocityPenaltyScale * nearGoalWeight * cappedAngularVelocityEnergy;
            }
            else
            {
                nearGoalActionEnergyPenalty = 0f;
                nearGoalActionDeltaPenalty = 0f;
                nearGoalAngularVelocityPenalty = 0f;
                nearGoalWeight = 0f;
            }
        }
        else
        {
            positionReward = activePositionRewardScale * Mathf.Exp(-positionError * positionError);
            attitudeReward = activeAttitudeRewardScale * Mathf.Exp(-attitudeErrorRadians);
            actionReward = activeActionRewardScale * Mathf.Exp(-meanSquaredAction);
            distancePenalty = 0f;
            attitudePenalty = 0f;
            actionDeltaPenalty = 0f;
            nearGoalActionEnergyPenalty = 0f;
            nearGoalActionDeltaPenalty = 0f;
            nearGoalAngularVelocityPenalty = 0f;
            holdReward = 0f;
            nearGoalWeight = 0f;
            insideTargetHoldWindow = positionError <= activeHoldPositionThreshold &&
                                      attitudeErrorRadians <= activeHoldAttitudeThresholdDeg * Mathf.Deg2Rad;
        }

        float reward = positionReward + attitudeReward + actionReward + distancePenalty + attitudePenalty +
                       actionDeltaPenalty + nearGoalActionEnergyPenalty + nearGoalActionDeltaPenalty +
                       nearGoalAngularVelocityPenalty + holdReward;

        AddReward(reward);
        UpdateDiagnostics(
            positionError,
            attitudeErrorRadians,
            meanSquaredAction,
            positionReward,
            attitudeReward,
            actionReward,
            distancePenalty,
            attitudePenalty,
            actionDeltaPenalty,
            nearGoalActionEnergyPenalty,
            nearGoalActionDeltaPenalty,
            nearGoalAngularVelocityPenalty,
            holdReward,
            meanSquaredActionDelta,
            nearGoalWeight,
            insideTargetHoldWindow,
            reward);
        RememberCurrentActions();

        if (Application.isBatchMode || enableStatsRecorder)
        {
            HoldForPositionTrainingMetrics.Record(
                currentPositionError,
                currentAttitudeErrorDeg,
                currentLocalAngularVelocity,
                currentActionDeltaRms,
                insideTargetHoldWindow,
                activeNearGoalRadius,
                orderedThrusters,
                currentActions,
                thrusterController);
        }

        if (IsOutOfBounds())
        {
            AddReward(outOfBoundsPenalty);
            lastStepReward += outOfBoundsPenalty;
            EndEpisode();
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

    void ResolveRuntimeReferences()
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
                Debug.Log($"[{nameof(HoldForPosition)}] {statusMessage}", this);
                logThrusterResolution = false;
            }
        }
        else
        {
            Debug.LogError($"[{nameof(HoldForPosition)}] {statusMessage}", this);
        }
    }

    void ReadAndApplyActions(ActionSegment<float> continuousActions)
    {
        int actionCount = Mathf.Min(currentActions.Length, continuousActions.Length);
        float actionLimit = Mathf.Clamp01(normalizedThrusterActionLimit);
        for (int i = 0; i < currentActions.Length; i++)
        {
            currentActions[i] = i < actionCount
                ? Mathf.Clamp(continuousActions[i], -actionLimit, actionLimit)
                : 0f;
        }

        FinsROVAgentRuntime.ApplyThrusterActions(orderedThrusters, currentActions, thrusterCommandMode, actionForceScaleN);
    }

    float ComputeMeanSquaredAction()
    {
        float sum = 0f;
        for (int i = 0; i < currentActions.Length; i++)
        {
            sum += currentActions[i] * currentActions[i];
        }

        return sum / currentActions.Length;
    }

    float ComputeActionEnergy()
    {
        float sum = 0f;
        for (int i = 0; i < currentActions.Length; i++)
        {
            sum += currentActions[i] * currentActions[i];
        }

        return sum;
    }

    float ComputeMeanSquaredActionDelta()
    {
        float sum = 0f;
        for (int i = 0; i < currentActions.Length; i++)
        {
            float delta = currentActions[i] - previousActions[i];
            sum += delta * delta;
        }

        return sum / currentActions.Length;
    }

    void RememberCurrentActions()
    {
        System.Array.Copy(currentActions, previousActions, currentActions.Length);
    }

    static float ComputeNearGoalWeight(float positionError, float radius)
    {
        float safeRadius = Mathf.Max(0.001f, radius);
        return Mathf.Clamp01(1f - positionError / safeRadius);
    }

    void ResolveDenseRewardConfiguration()
    {
        activeDenseRewardMode = denseRewardMode;
        activePositionRewardScale = Mathf.Max(0f, positionRewardScale);
        activeAttitudeRewardScale = Mathf.Max(0f, attitudeRewardScale);
        activeActionRewardScale = Mathf.Max(0f, actionRewardScale);
        activePositionErrorExponent = Mathf.Max(0f, learnToSwimPositionErrorExponent);
        activeAttitudeErrorExponent = Mathf.Max(0f, learnToSwimAttitudeErrorExponent);
        activeActionEnergyExponent = Mathf.Max(0f, learnToSwimActionEnergyExponent);
        activeDistancePenaltyScale = Mathf.Max(0f, distancePenaltyScale);
        activeAttitudePenaltyScale = Mathf.Max(0f, attitudePenaltyScale);
        activeActionDeltaPenaltyScale = Mathf.Max(0f, actionDeltaPenaltyScale);
        activeNearGoalRadius = Mathf.Max(0.001f, nearGoalRadius);
        activeNearGoalActionEnergyPenaltyScale = Mathf.Max(0f, nearGoalActionEnergyPenaltyScale);
        activeNearGoalActionDeltaPenaltyScale = Mathf.Max(0f, nearGoalActionDeltaPenaltyScale);
        activeNearGoalAngularVelocityPenaltyScale = Mathf.Max(0f, nearGoalAngularVelocityPenaltyScale);
        activeNearGoalAngularVelocityPenaltyMaxRadPerSec = Mathf.Max(0f, nearGoalAngularVelocityPenaltyMaxRadPerSec);
        activeHoldRewardScale = Mathf.Max(0f, holdRewardScale);
        activeHoldPositionThreshold = Mathf.Max(0f, holdPositionThreshold);
        activeHoldAttitudeThresholdDeg = Mathf.Max(0f, holdAttitudeThresholdDeg);
        activeHoldAngularVelocityThresholdRadPerSec = Mathf.Max(0f, holdAngularVelocityThresholdRadPerSec);

        if (!enableRewardParameterOverrides || Academy.Instance == null)
        {
            return;
        }

        EnvironmentParameters parameters = Academy.Instance.EnvironmentParameters;
        activeDenseRewardMode = (DenseRewardMode)Mathf.Clamp(
            Mathf.RoundToInt(parameters.GetWithDefault(RewardParameterPrefix + "mode", (float)activeDenseRewardMode)),
            (int)DenseRewardMode.InspectorDefault,
            (int)DenseRewardMode.IsaacPositionHoldNearGoalDamping);
        activePositionRewardScale = NonNegativeParameter(parameters, "position_scale", activePositionRewardScale);
        activeAttitudeRewardScale = NonNegativeParameter(parameters, "attitude_scale", activeAttitudeRewardScale);
        activeActionRewardScale = NonNegativeParameter(parameters, "action_scale", activeActionRewardScale);
        activePositionErrorExponent = NonNegativeParameter(parameters, "position_error_exponent", activePositionErrorExponent);
        activeAttitudeErrorExponent = NonNegativeParameter(parameters, "attitude_error_exponent", activeAttitudeErrorExponent);
        activeActionEnergyExponent = NonNegativeParameter(parameters, "action_energy_exponent", activeActionEnergyExponent);
        activeDistancePenaltyScale = NonNegativeParameter(parameters, "distance_penalty_scale", activeDistancePenaltyScale);
        activeAttitudePenaltyScale = NonNegativeParameter(parameters, "attitude_penalty_scale", activeAttitudePenaltyScale);
        activeActionDeltaPenaltyScale = NonNegativeParameter(parameters, "action_delta_penalty_scale", activeActionDeltaPenaltyScale);
        activeNearGoalRadius = PositiveParameter(parameters, "near_goal_radius", activeNearGoalRadius);
        activeNearGoalActionEnergyPenaltyScale = NonNegativeParameter(
            parameters,
            "near_goal_action_energy_penalty_scale",
            activeNearGoalActionEnergyPenaltyScale);
        activeNearGoalActionDeltaPenaltyScale = NonNegativeParameter(
            parameters,
            "near_goal_action_delta_penalty_scale",
            activeNearGoalActionDeltaPenaltyScale);
        activeNearGoalAngularVelocityPenaltyScale = NonNegativeParameter(
            parameters,
            "near_goal_angular_velocity_penalty_scale",
            activeNearGoalAngularVelocityPenaltyScale);
        activeNearGoalAngularVelocityPenaltyMaxRadPerSec = NonNegativeParameter(
            parameters,
            "near_goal_angular_velocity_penalty_max_radps",
            activeNearGoalAngularVelocityPenaltyMaxRadPerSec);
        activeHoldRewardScale = NonNegativeParameter(parameters, "hold_reward_scale", activeHoldRewardScale);
        activeHoldPositionThreshold = NonNegativeParameter(parameters, "hold_position_threshold", activeHoldPositionThreshold);
        activeHoldAttitudeThresholdDeg = NonNegativeParameter(parameters, "hold_attitude_threshold_deg", activeHoldAttitudeThresholdDeg);
        activeHoldAngularVelocityThresholdRadPerSec = NonNegativeParameter(
            parameters,
            "hold_angular_velocity_threshold_radps",
            activeHoldAngularVelocityThresholdRadPerSec);
    }

    static float NonNegativeParameter(EnvironmentParameters parameters, string name, float fallback)
    {
        return Mathf.Max(0f, parameters.GetWithDefault(RewardParameterPrefix + name, fallback));
    }

    static float PositiveParameter(EnvironmentParameters parameters, string name, float fallback)
    {
        return Mathf.Max(0.001f, parameters.GetWithDefault(RewardParameterPrefix + name, fallback));
    }

    bool IsOutOfBounds()
    {
        return Vector3.Distance(ReferenceTransform.position, episodeSpawnPosition) > maxDistanceFromSpawn ||
               ReferenceTransform.position.y > ParallelAreaOrigin.y + surfaceY + maxHeightAboveSurface;
    }

    Vector3 GetLocalLinearVelocity()
    {
        return FinsROVAgentRuntime.GetLocalLinearVelocity(ReferenceTransform, RigidBody);
    }

    Vector3 GetLocalAngularVelocity()
    {
        return FinsROVAgentRuntime.GetLocalAngularVelocity(ReferenceTransform, RigidBody);
    }

    System.Random CreateParallelEpisodeRandom()
    {
        DomainRandomizationCoordinator coordinator = GetComponentInParent<DomainRandomizationCoordinator>();
        if (coordinator == null)
        {
            coordinator = UnityEngine.Object.FindFirstObjectByType<DomainRandomizationCoordinator>();
        }
        // The DR coordinator provides an area- and episode-unique seed while
        // DR is active.  Do not use its serialized LastSeed when it is
        // disabled: that value remains zero, which otherwise makes every
        // replicated area and every episode sample exactly the same task.
        if (coordinator != null &&
            coordinator.isActiveAndEnabled &&
            coordinator.mode != DomainRandomizationMode.Disabled)
        {
            // Randomization is applied immediately before this method.
            // LastSeed is therefore area-unique and episode-unique after the
            // runtime's grid-derived offset has been applied.
            unchecked
            {
                return new System.Random(coordinator.LastSeed * 1_664_525 + 1_013_904_223);
            }
        }

        // Physical domain randomization is optional; task-state sampling is
        // not. The launcher gives every multi-binary worker a distinct
        // -fins-dr-seed argument, while multi-area workers share that launcher
        // seed and differ through parallelAreaSeedOffset. This keeps initial
        // poses and targets independent without enabling physical DR.
        unchecked
        {
            int taskSeed = LauncherTaskSeed();
            taskSeed = taskSeed * 486_187_739 + parallelAreaSeedOffset;
            taskSeed = taskSeed * 16_777_619 + parallelEpisodeIndex;
            parallelEpisodeIndex++;
            return new System.Random(taskSeed);
        }
    }

    static int LauncherTaskSeed()
    {
        string[] args = System.Environment.GetCommandLineArgs();
        for (int i = 0; i < args.Length; i++)
        {
            string argument = args[i];
            if (string.Equals(argument, "-fins-dr-seed", System.StringComparison.OrdinalIgnoreCase) &&
                i + 1 < args.Length && int.TryParse(args[i + 1], out int nextValue))
            {
                return nextValue;
            }

            const string prefix = "-fins-dr-seed=";
            if (argument.StartsWith(prefix, System.StringComparison.OrdinalIgnoreCase) &&
                int.TryParse(argument.Substring(prefix.Length), out int inlineValue))
            {
                return inlineValue;
            }
        }

        // Editor/manual runs without a launcher argument remain deterministic.
        return 19_001;
    }

    float RandomRange(float minInclusive, float maxInclusive)
    {
        if (parallelEpisodeRandom == null)
        {
            return UnityEngine.Random.Range(minInclusive, maxInclusive);
        }
        return Mathf.Lerp(minInclusive, maxInclusive, (float)parallelEpisodeRandom.NextDouble());
    }

    int RandomRange(int minInclusive, int maxExclusive)
    {
        if (parallelEpisodeRandom == null)
        {
            return UnityEngine.Random.Range(minInclusive, maxExclusive);
        }
        return parallelEpisodeRandom.Next(minInclusive, maxExclusive);
    }

    Vector3 RandomInsideBox(float range)
    {
        float clampedRange = Mathf.Max(0f, range);
        return new Vector3(
            RandomRange(-clampedRange, clampedRange),
            RandomRange(-clampedRange, clampedRange),
            RandomRange(-clampedRange, clampedRange));
    }

    Vector3 SampleInitialLocalLinearVelocity()
    {
        float range = Mathf.Max(0f, initialLinearVelocityRange);
        if (initialLinearVelocityRandomizationMode == InitialLinearVelocityRandomizationMode.IndependentAxes)
        {
            return RandomInsideBox(range);
        }

        int axis = RandomRange(0, 3);
        var velocity = Vector3.zero;
        velocity[axis] = RandomRange(-range, range);
        return velocity;
    }

    static void AddRotation6DObservation(VectorSensor sensor, Quaternion rotation)
    {
        Matrix4x4 matrix = Matrix4x4.Rotate(rotation.normalized);
        sensor.AddObservation(new Vector3(matrix.m00, matrix.m10, matrix.m20));
        sensor.AddObservation(new Vector3(matrix.m01, matrix.m11, matrix.m21));
    }

    void AddNormalizedVectorObservation(VectorSensor sensor, Vector3 value, float scale)
    {
        float safeScale = Mathf.Max(Mathf.Abs(scale), 1e-6f);
        float clip = Mathf.Max(Mathf.Abs(normalizedVelocityObservationClip), 1e-6f);
        Vector3 normalized = value / safeScale;
        sensor.AddObservation(new Vector3(
            Mathf.Clamp(normalized.x, -clip, clip),
            Mathf.Clamp(normalized.y, -clip, clip),
            Mathf.Clamp(normalized.z, -clip, clip)));
    }

    Vector3 GenerateRandomTargetPosition(
        float radius,
        Vector3 centerPosition,
        float effectiveSurfaceY,
        out Vector3 rawTargetPosition,
        out bool surfaceClamped)
    {
        Vector3 randomDirection = RandomUnitDirection();
        Vector3 randomPoint = randomDirection.sqrMagnitude > 1e-6f
            ? randomDirection * RandomRange(0f, Mathf.Max(0f, radius))
            : Vector3.zero;
        rawTargetPosition = centerPosition + randomPoint;
        surfaceClamped = rawTargetPosition.y > effectiveSurfaceY;
        Vector3 target = rawTargetPosition;
        target.y = Mathf.Min(rawTargetPosition.y, effectiveSurfaceY);
        return target;
    }

    void LogResetTargetSampleIfEnabled(
        Vector3 spawnPosition,
        Vector3 rawTargetPosition,
        Vector3 targetPosition,
        float effectiveSurfaceY,
        bool surfaceClamped)
    {
        bool logEnabled = logResetTargetSampling;
        if (!logEnabled && Academy.Instance != null)
        {
            logEnabled = Academy.Instance.EnvironmentParameters.GetWithDefault(
                DebugParameterPrefix + "log_reset_target", 0f) > 0.5f;
        }

        if (!logEnabled)
        {
            return;
        }

        DomainRandomizationCoordinator coordinator = GetComponentInParent<DomainRandomizationCoordinator>();
        int episode = coordinator != null ? coordinator.EpisodeIndex - 1 : parallelEpisodeIndex - 1;
        int seed = coordinator != null ? coordinator.LastSeed : LauncherTaskSeed();
        Debug.Log(
            $"[HoldForPosition.ResetTarget] episode={episode} seed={seed} " +
            $"spawn={spawnPosition:F3} raw_target={rawTargetPosition:F3} " +
            $"target={targetPosition:F3} surface_y={effectiveSurfaceY:F3} " +
            $"surface_clamped={surfaceClamped}");
    }

    Vector3 RandomUnitDirection()
    {
        if (parallelEpisodeRandom == null)
        {
            Vector3 direction = UnityEngine.Random.insideUnitSphere;
            return direction.sqrMagnitude > 1e-6f ? direction.normalized : Vector3.zero;
        }

        // Rejection sampling gives an isotropic direction without consuming
        // UnityEngine.Random's process-global stream shared by all areas.
        for (int attempt = 0; attempt < 32; attempt++)
        {
            Vector3 direction = new Vector3(
                RandomRange(-1f, 1f),
                RandomRange(-1f, 1f),
                RandomRange(-1f, 1f));
            float magnitude = direction.sqrMagnitude;
            if (magnitude > 1e-6f && magnitude <= 1f)
            {
                return direction / Mathf.Sqrt(magnitude);
            }
        }

        return Vector3.forward;
    }

    Vector3 ParallelAreaOrigin => parallelAreaTransform != null ? parallelAreaTransform.position : Vector3.zero;

    void ResetDiagnostics()
    {
        currentPositionError = 0f;
        currentAttitudeErrorDeg = 0f;
        currentLocalLinearVelocity = Vector3.zero;
        currentLocalAngularVelocity = Vector3.zero;
        currentActionRms = 0f;
        currentPositionReward = 0f;
        currentAttitudeReward = 0f;
        currentActionReward = 0f;
        currentDistancePenalty = 0f;
        currentAttitudePenalty = 0f;
        currentActionDeltaPenalty = 0f;
        currentNearGoalActionEnergyPenalty = 0f;
        currentNearGoalActionDeltaPenalty = 0f;
        currentNearGoalAngularVelocityPenalty = 0f;
        currentHoldReward = 0f;
        currentActionDeltaRms = 0f;
        currentNearGoalWeight = 0f;
        currentTargetHoldWindow = 0f;
        lastStepReward = 0f;
    }

    void UpdateDiagnostics(
        float positionError,
        float attitudeErrorRadians,
        float meanSquaredAction,
        float positionReward,
        float attitudeReward,
        float actionReward,
        float distancePenalty,
        float attitudePenalty,
        float actionDeltaPenalty,
        float nearGoalActionEnergyPenalty,
        float nearGoalActionDeltaPenalty,
        float nearGoalAngularVelocityPenalty,
        float holdReward,
        float meanSquaredActionDelta,
        float nearGoalWeight,
        bool insideTargetHoldWindow,
        float reward)
    {
        currentPositionError = positionError;
        currentAttitudeErrorDeg = attitudeErrorRadians * Mathf.Rad2Deg;
        currentLocalLinearVelocity = GetLocalLinearVelocity();
        currentLocalAngularVelocity = GetLocalAngularVelocity();
        currentActionRms = Mathf.Sqrt(meanSquaredAction);
        currentPositionReward = positionReward;
        currentAttitudeReward = attitudeReward;
        currentActionReward = actionReward;
        currentDistancePenalty = distancePenalty;
        currentAttitudePenalty = attitudePenalty;
        currentActionDeltaPenalty = actionDeltaPenalty;
        currentNearGoalActionEnergyPenalty = nearGoalActionEnergyPenalty;
        currentNearGoalActionDeltaPenalty = nearGoalActionDeltaPenalty;
        currentNearGoalAngularVelocityPenalty = nearGoalAngularVelocityPenalty;
        currentHoldReward = holdReward;
        currentActionDeltaRms = Mathf.Sqrt(meanSquaredActionDelta);
        currentNearGoalWeight = nearGoalWeight;
        currentTargetHoldWindow = insideTargetHoldWindow ? 1f : 0f;
        lastStepReward = reward;
    }
}
