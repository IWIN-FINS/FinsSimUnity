using FinsSim.Actuators;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

[DisallowMultipleComponent]
[RequireComponent(typeof(Rigidbody))]
public class ControlForPosition_GoalRefreshLowYawTask : Agent
{
    Rigidbody rigidBody;
    ThrusterController thrusterController;
    Transform referenceTransform;
    PositionDropoutHoldRandomizer positionDropout;

    readonly Thruster[] orderedThrusters = new Thruster[FinsROVAgentRuntime.DefaultThrusterOrder.Length];
    readonly float[] previousActions = new float[FinsROVAgentRuntime.DefaultThrusterOrder.Length];
    readonly float[] currentActions = new float[FinsROVAgentRuntime.DefaultThrusterOrder.Length];

    [Header("Reference Frame")]
    [Tooltip("Prefer this reference point. If empty, a child named MarkPosition is used; otherwise the root Transform is used.")]
    public Transform selfTransform;
    public Transform targetTransform;

    [Header("Thruster Control")]
    [SerializeField] ThrusterController thrusterControllerOverride;
    [SerializeField] ThrusterCommandMode thrusterCommandMode = ThrusterCommandMode.ScaledForceRequest;
    [ShowWhenThrusterCommandMode(ThrusterCommandMode.ScaledForceRequest)]
    [SerializeField] float actionForceScaleN = 7f;
    [SerializeField] bool autoResolveThrustersFromChildren = true;
    [SerializeField] bool logThrusterResolution = true;
    [SerializeField] float heuristicYawMixScale = 1.0f;
    [SerializeField] float heuristicAuxMixScale = 1.0f;
    [SerializeField] bool autoRequestDecisionWhenNoDecisionRequester = true;

    [Header("Dropout Randomization")]
    [SerializeField] PositionDropoutHoldRandomizer positionDropoutOverride;
    [SerializeField] bool autoResolvePositionDropout = true;

    [Header("Runtime Diagnostics")]
    [SerializeField] bool enableStatsRecorder = false;

    [Header("Episode Reset")]
    [SerializeField] float spawnPositionRange = 2f;
    [SerializeField] float spawnDepthOffset = -2f;
    [SerializeField] float targetSpawnRadius = 3f;
    [SerializeField] float surfaceY = 0f;
    [SerializeField] bool randomizeAgentYawOnReset = true;

    [Header("Goal Refresh")]
    [Min(0)] [SerializeField] int targetRefreshesPerEpisode = 2;
    [SerializeField] float targetRefreshRadius = 3f;
    [SerializeField] float minTargetRefreshDistance = 0.4f;
    [SerializeField] bool randomizeTargetYawOnRefresh = true;
    [SerializeField] bool waitForFreshPositionBeforeRefresh = true;

    [Header("Observation Normalization")]
    [SerializeField] float linearVelocityObservationScale = 1f;
    [SerializeField] float angularVelocityObservationScale = 1f;
    [SerializeField] float normalizedVelocityObservationClip = 2f;

    [Header("Reward")]
    [SerializeField] LowYawPoseReward rewardConfig = new LowYawPoseReward();

    [Header("Safety Limit")]
    [SerializeField] float maxDistanceFromSpawn = 8f;
    [SerializeField] float maxHeightAboveSurface = 0.3f;

    Vector3 episodeSpawnPosition;
    float previousDistanceToTarget;
    float previousHeadingError01;
    float currentDistanceToTarget;
    float currentHeadingError01;
    float currentDirectionAlignment;
    float currentForwardApproachSpeed;
    float currentNearTargetRatio;
    float currentLocalAngularSpeed;
    float currentLocalYawRate;
    float currentNearTargetAngularPenalty;
    float lastStepReward;
    int stableSuccessStepCount;
    int targetRefreshesCompleted;
    bool currentActionHeldByDropout;
    bool hasDecisionRequester;

    public float LastStepReward => lastStepReward;
    public float CurrentDistanceToTarget => currentDistanceToTarget;
    public float CurrentHeadingError01 => currentHeadingError01;
    public float CurrentHeadingErrorDeg => currentHeadingError01 * 180f;
    public float CurrentDirectionAlignment => currentDirectionAlignment;
    public float CurrentForwardApproachSpeed => currentForwardApproachSpeed;
    public float CurrentNearTargetRatio => currentNearTargetRatio;
    public float CurrentLocalAngularSpeed => currentLocalAngularSpeed;
    public float CurrentLocalYawRate => currentLocalYawRate;
    public float CurrentNearTargetAngularPenalty => currentNearTargetAngularPenalty;
    public int StableSuccessStepCount => stableSuccessStepCount;
    public int TargetRefreshesCompleted => targetRefreshesCompleted;
    public bool CurrentActionHeldByDropout => currentActionHeldByDropout;
    public bool PositionDropoutActive => positionDropout != null && positionDropout.IsDropoutActive;
    public float SuccessDistance => RewardConfig.successDistance;
    public float SuccessHeadingAngleDeg => RewardConfig.successHeadingAngleDeg;
    public Vector3 CurrentLocalLinearVelocity => GetLocalLinearVelocity();
    public Vector3 CurrentLocalAngularVelocity => GetLocalAngularVelocity();

    LowYawPoseReward RewardConfig
    {
        get
        {
            if (rewardConfig == null)
            {
                rewardConfig = new LowYawPoseReward();
            }

            return rewardConfig;
        }
    }

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
        hasDecisionRequester = GetComponent("DecisionRequester") != null;

        ResolveRuntimeReferences();
    }

    void FixedUpdate()
    {
        ResolvePositionDropout();
        positionDropout?.Tick(Time.fixedDeltaTime > 0f ? Time.fixedDeltaTime : Time.deltaTime);

        if (autoRequestDecisionWhenNoDecisionRequester && !hasDecisionRequester)
        {
            RequestDecision();
        }
    }

    public override void OnEpisodeBegin()
    {
        ResolveRuntimeReferences();
        FinsROVAgentRuntime.RandomizeEpisodeIfPresent(this);
        ResolvePositionDropout();
        positionDropout?.BeginEpisode();

        RigidBody.angularVelocity = Vector3.zero;
        RigidBody.linearVelocity = Vector3.zero;
        FinsROVAgentRuntime.ZeroThrusters(orderedThrusters, thrusterCommandMode, actionForceScaleN);

        transform.position = new Vector3(
            Random.Range(0f, spawnPositionRange),
            Random.Range(spawnDepthOffset, spawnDepthOffset + spawnPositionRange),
            Random.Range(0f, spawnPositionRange)
        );

        if (randomizeAgentYawOnReset)
        {
            transform.rotation = Quaternion.Euler(0f, Random.Range(0f, 360f), 0f);
        }

        episodeSpawnPosition = ReferenceTransform.position;

        if (targetTransform != null)
        {
            targetTransform.position = GenerateRandomTargetPosition(targetSpawnRadius, 0f, episodeSpawnPosition);
            targetTransform.rotation = GenerateRandomHeading();
        }

        previousDistanceToTarget = GetDistanceToTarget();
        previousHeadingError01 = GetHeadingError01();
        currentDistanceToTarget = previousDistanceToTarget;
        currentHeadingError01 = previousHeadingError01;
        currentDirectionAlignment = 0f;
        currentForwardApproachSpeed = 0f;
        currentNearTargetRatio = 0f;
        currentLocalAngularSpeed = 0f;
        currentLocalYawRate = 0f;
        currentNearTargetAngularPenalty = 0f;
        lastStepReward = 0f;
        stableSuccessStepCount = 0;
        targetRefreshesCompleted = 0;
        currentActionHeldByDropout = false;
        System.Array.Clear(previousActions, 0, previousActions.Length);
        System.Array.Clear(currentActions, 0, currentActions.Length);

        positionDropout?.CaptureFreshPose(ReferenceTransform, targetTransform);
        positionDropout?.CaptureThrusterActions(currentActions);
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        Transform reference = ReferenceTransform;
        float positionScale = Mathf.Max(Mathf.Max(targetSpawnRadius, targetRefreshRadius), 0.01f);
        ResolveObservedPose(
            reference,
            out Vector3 observedReferencePosition,
            out Quaternion observedReferenceRotation,
            out Vector3 observedTargetPosition,
            out Quaternion observedTargetRotation);

        Quaternion inverseObservedReferenceRotation = Quaternion.Inverse(observedReferenceRotation);
        Vector3 localTargetOffset = inverseObservedReferenceRotation * (observedTargetPosition - observedReferencePosition);
        Quaternion relativeTargetRotation = inverseObservedReferenceRotation * observedTargetRotation;
        float distanceToTarget = localTargetOffset.magnitude;

        sensor.AddObservation(localTargetOffset / positionScale);
        AddRotation6DObservation(sensor, relativeTargetRotation);
        AddNormalizedVectorObservation(sensor, GetLocalLinearVelocity(), linearVelocityObservationScale);
        AddNormalizedVectorObservation(sensor, GetLocalAngularVelocity(), angularVelocityObservationScale);
        sensor.AddObservation(Mathf.Clamp01(distanceToTarget / positionScale));
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

        currentActionHeldByDropout = positionDropout != null && positionDropout.TryWriteHeldThrusterActions(currentActions);
        FinsROVAgentRuntime.ApplyThrusterActions(orderedThrusters, currentActions, thrusterCommandMode, actionForceScaleN);
        if (!currentActionHeldByDropout)
        {
            positionDropout?.CaptureThrusterActions(currentActions);
        }

        float distanceToTarget = GetDistanceToTarget();
        float headingError01 = GetHeadingError01();

        Transform reference = ReferenceTransform;
        Vector3 toTarget = targetTransform != null ? targetTransform.position - reference.position : Vector3.zero;
        Vector3 directionToTarget = toTarget.sqrMagnitude > 1e-6f ? toTarget.normalized : Vector3.zero;
        Vector3 localLinearVelocity = GetLocalLinearVelocity();
        Vector3 localAngularVelocity = GetLocalAngularVelocity();

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

        LowYawPoseRewardResult rewardResult = RewardConfig.Evaluate(
            previousDistanceToTarget,
            previousHeadingError01,
            distanceToTarget,
            headingError01,
            directionToTarget,
            reference.forward,
            RigidBody.linearVelocity,
            localLinearVelocity,
            localAngularVelocity,
            actionEnergy,
            actionDelta);

        stableSuccessStepCount = rewardResult.InSuccessPose && rewardResult.StableForSuccess
            ? stableSuccessStepCount + 1
            : 0;

        AddReward(rewardResult.Reward);
        lastStepReward = rewardResult.Reward;
        currentDistanceToTarget = distanceToTarget;
        currentHeadingError01 = headingError01;
        currentDirectionAlignment = rewardResult.DirectionAlignment;
        currentForwardApproachSpeed = rewardResult.ForwardApproachSpeed;
        currentNearTargetRatio = rewardResult.NearTargetRatio;
        currentLocalAngularSpeed = rewardResult.AngularSpeed;
        currentLocalYawRate = rewardResult.YawRate;
        currentNearTargetAngularPenalty = rewardResult.NearTargetAngularPenalty;

        if (enableStatsRecorder)
        {
            Academy.Instance.StatsRecorder.Add("FinsROV/low_yaw_distance", distanceToTarget);
            Academy.Instance.StatsRecorder.Add("FinsROV/low_yaw_heading_error_deg", CurrentHeadingErrorDeg);
            Academy.Instance.StatsRecorder.Add("FinsROV/low_yaw_near_target_ratio", rewardResult.NearTargetRatio);
            Academy.Instance.StatsRecorder.Add("FinsROV/low_yaw_yaw_rate", rewardResult.YawRate);
            Academy.Instance.StatsRecorder.Add("FinsROV/low_yaw_near_target_angular_penalty", rewardResult.NearTargetAngularPenalty);
            Academy.Instance.StatsRecorder.Add("FinsROV/low_yaw_stable_success_steps", stableSuccessStepCount);
            Academy.Instance.StatsRecorder.Add("FinsROV/low_yaw_target_refreshes", targetRefreshesCompleted);
            Academy.Instance.StatsRecorder.Add("FinsROV/position_dropout_active", PositionDropoutActive ? 1f : 0f);
            Academy.Instance.StatsRecorder.Add("FinsROV/thruster_hold_active", currentActionHeldByDropout ? 1f : 0f);
        }

        previousDistanceToTarget = distanceToTarget;
        previousHeadingError01 = headingError01;

        bool canResolveReachedGoal = !waitForFreshPositionBeforeRefresh || !PositionDropoutActive;
        if (canResolveReachedGoal && stableSuccessStepCount >= Mathf.Max(1, RewardConfig.stableSuccessStepsRequired))
        {
            AddReward(RewardConfig.successReward);
            lastStepReward += RewardConfig.successReward;

            if (TryRefreshTarget())
            {
                return;
            }

            EndEpisode();
            return;
        }

        bool outOfBounds =
            Vector3.Distance(ReferenceTransform.position, episodeSpawnPosition) > maxDistanceFromSpawn ||
            ReferenceTransform.position.y > surfaceY + maxHeightAboveSurface;

        if (outOfBounds)
        {
            AddReward(RewardConfig.outOfBoundsPenalty);
            lastStepReward += RewardConfig.outOfBoundsPenalty;
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
        ResolvePositionDropout();

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
                Debug.Log($"[{nameof(ControlForPosition_GoalRefreshLowYawTask)}] {statusMessage}", this);
                logThrusterResolution = false;
            }
        }
        else
        {
            Debug.LogError($"[{nameof(ControlForPosition_GoalRefreshLowYawTask)}] {statusMessage}", this);
        }
    }

    void ResolvePositionDropout()
    {
        if (positionDropout != null)
        {
            return;
        }

        if (positionDropoutOverride != null)
        {
            positionDropout = positionDropoutOverride;
            return;
        }

        if (!autoResolvePositionDropout)
        {
            return;
        }

        positionDropout = GetComponent<PositionDropoutHoldRandomizer>();
        if (positionDropout == null)
        {
            positionDropout = GetComponentInChildren<PositionDropoutHoldRandomizer>(true);
        }
        if (positionDropout == null)
        {
            positionDropout = GetComponentInParent<PositionDropoutHoldRandomizer>();
        }
    }

    void ResolveObservedPose(
        Transform reference,
        out Vector3 observedReferencePosition,
        out Quaternion observedReferenceRotation,
        out Vector3 observedTargetPosition,
        out Quaternion observedTargetRotation)
    {
        if (positionDropout != null)
        {
            positionDropout.ResolveObservedPose(
                reference,
                targetTransform,
                out observedReferencePosition,
                out observedReferenceRotation,
                out observedTargetPosition,
                out observedTargetRotation);
            return;
        }

        observedReferencePosition = reference.position;
        observedReferenceRotation = reference.rotation;
        observedTargetPosition = targetTransform != null ? targetTransform.position : reference.position;
        observedTargetRotation = targetTransform != null ? targetTransform.rotation : Quaternion.identity;
    }

    bool TryRefreshTarget()
    {
        if (targetTransform == null || targetRefreshesCompleted >= Mathf.Max(0, targetRefreshesPerEpisode))
        {
            return false;
        }

        targetRefreshesCompleted++;
        targetTransform.position = GenerateRandomTargetPosition(
            Mathf.Max(0f, targetRefreshRadius),
            Mathf.Max(0f, minTargetRefreshDistance),
            ReferenceTransform.position);
        if (randomizeTargetYawOnRefresh)
        {
            targetTransform.rotation = GenerateRandomHeading();
        }

        stableSuccessStepCount = 0;
        previousDistanceToTarget = GetDistanceToTarget();
        previousHeadingError01 = GetHeadingError01();
        currentDistanceToTarget = previousDistanceToTarget;
        currentHeadingError01 = previousHeadingError01;
        positionDropout?.CaptureFreshPose(ReferenceTransform, targetTransform);

        if (enableStatsRecorder)
        {
            Academy.Instance.StatsRecorder.Add("FinsROV/low_yaw_goal_refreshed", 1f);
        }

        return true;
    }

    float GetDistanceToTarget()
    {
        return targetTransform != null
            ? Vector3.Distance(ReferenceTransform.position, targetTransform.position)
            : 0f;
    }

    float GetHeadingError01()
    {
        return targetTransform != null
            ? Quaternion.Angle(ReferenceTransform.rotation, targetTransform.rotation) / 180f
            : 0f;
    }

    Vector3 GetLocalLinearVelocity()
    {
        return FinsROVAgentRuntime.GetLocalLinearVelocity(ReferenceTransform, RigidBody);
    }

    Vector3 GetLocalAngularVelocity()
    {
        return FinsROVAgentRuntime.GetLocalAngularVelocity(ReferenceTransform, RigidBody);
    }

    void AddRotation6DObservation(VectorSensor sensor, Quaternion rotation)
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
            Mathf.Clamp(normalized.z, -clip, clip)
        ));
    }

    Vector3 GenerateRandomTargetPosition(float radius, float minDistance, Vector3 centerPosition)
    {
        float safeRadius = Mathf.Max(radius, 0f);
        float safeMinDistance = Mathf.Min(Mathf.Max(minDistance, 0f), safeRadius);
        Vector3 randomPoint = Vector3.zero;

        for (int attempt = 0; attempt < 16; attempt++)
        {
            Vector3 randomDirection = Random.insideUnitSphere;
            if (randomDirection.sqrMagnitude < 1e-6f)
            {
                randomDirection = Vector3.forward;
            }

            randomPoint = randomDirection.normalized * Random.Range(safeMinDistance, safeRadius);
            if (randomPoint.magnitude >= safeMinDistance || safeMinDistance <= 1e-6f)
            {
                break;
            }
        }

        return new Vector3(
            centerPosition.x + randomPoint.x,
            centerPosition.y + Mathf.Min(randomPoint.y, surfaceY - centerPosition.y),
            centerPosition.z + randomPoint.z
        );
    }

    Quaternion GenerateRandomHeading()
    {
        return Quaternion.Euler(0f, Random.Range(0f, 360f), 0f);
    }
}
