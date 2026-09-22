using FinsSim.Actuators;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

[DisallowMultipleComponent]
[RequireComponent(typeof(Rigidbody))]
public class ControlForPosition_GentleReward : Agent
{
    Rigidbody rigidBody;
    ThrusterController thrusterController;
    Transform referenceTransform;

    readonly Thruster[] orderedThrusters = new Thruster[FinsROVAgentRuntime.DefaultThrusterOrder.Length];
    readonly float[] previousActions = new float[FinsROVAgentRuntime.DefaultThrusterOrder.Length];
    readonly float[] currentActions = new float[FinsROVAgentRuntime.DefaultThrusterOrder.Length];

    [Header("Reference Frame")]
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

    [Header("Runtime Diagnostics")]
    [SerializeField] bool enableStatsRecorder = false;

    [Header("Episode Reset")]
    [SerializeField] float spawnPositionRange = 2f;
    [SerializeField] float spawnDepthOffset = -2f;
    [SerializeField] float targetSpawnRadius = 3f;
    [SerializeField] float surfaceY = 0f;
    [SerializeField] bool randomizeAgentYawOnReset = true;

    [Header("Observation Normalization")]
    [SerializeField] float linearVelocityObservationScale = 1f;
    [SerializeField] float angularVelocityObservationScale = 1f;
    [SerializeField] float normalizedVelocityObservationClip = 2f;

    [Header("Success Threshold")]
    [SerializeField] float successDistance = 0.35f;
    [SerializeField] bool requireHeadingForSuccess = false;
    [SerializeField] float successHeadingAngleDeg = 60f;
    [SerializeField] bool requireStableVelocityForSuccess = false;
    [SerializeField] float stableSuccessLinearVelocity = 0.25f;
    [SerializeField] float stableSuccessAngularVelocity = 0.8f;
    [SerializeField] int stableSuccessStepsRequired = 3;
    [SerializeField] float nearTargetDistance = 1.0f;

    [Header("Reward Weights")]
    [SerializeField] float perStepPenalty = -0.0005f;
    [SerializeField] float distanceProgressRewardScale = 1.0f;
    [SerializeField] float distanceShapingRewardScale = 0.02f;
    [SerializeField] float nearTargetRewardScale = 0.12f;
    [SerializeField] float headingProgressRewardScale = 0.05f;
    [SerializeField] float directionAlignmentRewardScale = 0.015f;
    [SerializeField] float approachVelocityRewardScale = 0.03f;
    [SerializeField] float stableHoldRewardScale = 0.01f;

    [Header("Soft Motion Penalties")]
    [SerializeField] float angularPenaltyDeadbandDegPerSec = 60f;
    [SerializeField] float yawPenaltyDeadbandDegPerSec = 80f;
    [SerializeField] float angularVelocityPenaltyScale = 0.002f;
    [SerializeField] float yawRatePenaltyScale = 0.003f;
    [SerializeField] float nearTargetAngularPenaltyScale = 0.006f;
    [SerializeField] float speedNearTargetPenaltyScale = 0.01f;
    [SerializeField] float actionEnergyPenaltyScale = 0.0008f;
    [SerializeField] float actionChangePenaltyScale = 0.0006f;

    [Header("Terminal Rewards")]
    [SerializeField] float successReward = 1.5f;
    [SerializeField] float outOfBoundsPenalty = -1f;

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
    float currentAngularPenalty;
    float lastStepReward;
    int stableSuccessStepCount;
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
    public float CurrentAngularPenalty => currentAngularPenalty;
    public int StableSuccessStepCount => stableSuccessStepCount;
    public float SuccessDistance => successDistance;
    public float SuccessHeadingAngleDeg => successHeadingAngleDeg;
    public Vector3 CurrentLocalLinearVelocity => GetLocalLinearVelocity();
    public Vector3 CurrentLocalAngularVelocity => GetLocalAngularVelocity();

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
        if (autoRequestDecisionWhenNoDecisionRequester && !hasDecisionRequester)
        {
            RequestDecision();
        }
    }

    public override void OnEpisodeBegin()
    {
        ResolveRuntimeReferences();
        FinsROVAgentRuntime.RandomizeEpisodeIfPresent(this);

        RigidBody.angularVelocity = Vector3.zero;
        RigidBody.linearVelocity = Vector3.zero;
        FinsROVAgentRuntime.ZeroThrusters(orderedThrusters, thrusterCommandMode, actionForceScaleN);

        transform.position = new Vector3(
            Random.Range(0f, spawnPositionRange),
            Random.Range(spawnDepthOffset, spawnDepthOffset + spawnPositionRange),
            Random.Range(0f, spawnPositionRange));

        transform.rotation = randomizeAgentYawOnReset
            ? Quaternion.Euler(0f, Random.Range(0f, 360f), 0f)
            : Quaternion.identity;

        episodeSpawnPosition = ReferenceTransform.position;

        if (targetTransform != null)
        {
            targetTransform.position = GenerateRandomTargetPosition(targetSpawnRadius, episodeSpawnPosition);
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
        currentAngularPenalty = 0f;
        lastStepReward = 0f;
        stableSuccessStepCount = 0;
        System.Array.Clear(previousActions, 0, previousActions.Length);
        System.Array.Clear(currentActions, 0, currentActions.Length);
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
        float distanceToTarget = targetTransform != null ? localTargetOffset.magnitude : 0f;

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

        FinsROVAgentRuntime.ApplyThrusterActions(orderedThrusters, currentActions, thrusterCommandMode, actionForceScaleN);

        float distanceToTarget = GetDistanceToTarget();
        float headingError01 = GetHeadingError01();

        Transform reference = ReferenceTransform;
        Vector3 toTarget = targetTransform != null ? targetTransform.position - reference.position : Vector3.zero;
        Vector3 directionToTarget = toTarget.sqrMagnitude > 1e-6f ? toTarget.normalized : reference.forward;
        Vector3 localLinearVelocity = GetLocalLinearVelocity();
        Vector3 localAngularVelocity = GetLocalAngularVelocity();

        float distanceProgress = previousDistanceToTarget - distanceToTarget;
        float headingProgress = previousHeadingError01 - headingError01;

        Vector3 referenceForwardFlat = new Vector3(reference.forward.x, 0f, reference.forward.z);
        Vector3 targetDirectionFlat = new Vector3(directionToTarget.x, 0f, directionToTarget.z);
        float directionAlignment = 0f;
        if (referenceForwardFlat.sqrMagnitude > 1e-6f && targetDirectionFlat.sqrMagnitude > 1e-6f)
        {
            directionAlignment = Vector3.Dot(referenceForwardFlat.normalized, targetDirectionFlat.normalized);
        }

        float forwardApproachSpeed = Vector3.Dot(RigidBody.linearVelocity, directionToTarget);
        float nearTargetRatio = 1f - Mathf.Clamp01(distanceToTarget / Mathf.Max(nearTargetDistance, 0.01f));
        float nearTargetGain = nearTargetRatio * nearTargetRatio * (3f - 2f * nearTargetRatio);
        float angularSpeed = localAngularVelocity.magnitude;
        float yawRate = Mathf.Abs(localAngularVelocity.y);
        float angularExcess = Mathf.Max(0f, angularSpeed - angularPenaltyDeadbandDegPerSec * Mathf.Deg2Rad);
        float yawExcess = Mathf.Max(0f, yawRate - yawPenaltyDeadbandDegPerSec * Mathf.Deg2Rad);
        float angularPenalty =
            angularVelocityPenaltyScale * angularExcess * angularExcess +
            yawRatePenaltyScale * yawExcess * yawExcess +
            nearTargetAngularPenaltyScale * nearTargetGain * (angularExcess * angularExcess + yawExcess * yawExcess);

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

        bool headingOk = !requireHeadingForSuccess || headingError01 < successHeadingAngleDeg / 180f;
        bool velocityOk = !requireStableVelocityForSuccess ||
            (localLinearVelocity.magnitude < stableSuccessLinearVelocity && angularSpeed < stableSuccessAngularVelocity);
        bool inSuccessWindow = distanceToTarget < successDistance && headingOk && velocityOk;

        stableSuccessStepCount = inSuccessWindow
            ? stableSuccessStepCount + 1
            : 0;

        float distanceShaping = 1f - Mathf.Clamp01(distanceToTarget / Mathf.Max(targetSpawnRadius, 0.01f));
        float stableHoldReward = inSuccessWindow ? stableHoldRewardScale * nearTargetGain : 0f;
        float reward =
            perStepPenalty +
            distanceProgressRewardScale * distanceProgress +
            distanceShapingRewardScale * distanceShaping +
            nearTargetRewardScale * nearTargetGain +
            headingProgressRewardScale * headingProgress +
            directionAlignmentRewardScale * directionAlignment +
            approachVelocityRewardScale * Mathf.Clamp(forwardApproachSpeed, -1f, 1f) +
            stableHoldReward -
            angularPenalty -
            speedNearTargetPenaltyScale * nearTargetGain * localLinearVelocity.magnitude -
            actionEnergyPenaltyScale * actionEnergy -
            actionChangePenaltyScale * actionDelta;

        AddReward(reward);
        lastStepReward = reward;
        currentDistanceToTarget = distanceToTarget;
        currentHeadingError01 = headingError01;
        currentDirectionAlignment = directionAlignment;
        currentForwardApproachSpeed = forwardApproachSpeed;
        currentNearTargetRatio = nearTargetRatio;
        currentLocalAngularSpeed = angularSpeed;
        currentLocalYawRate = yawRate;
        currentAngularPenalty = angularPenalty;

        if (enableStatsRecorder)
        {
            Academy.Instance.StatsRecorder.Add("FinsROV/gentle_distance", distanceToTarget);
            Academy.Instance.StatsRecorder.Add("FinsROV/gentle_near_target_ratio", nearTargetRatio);
            Academy.Instance.StatsRecorder.Add("FinsROV/gentle_local_angular_speed", angularSpeed);
            Academy.Instance.StatsRecorder.Add("FinsROV/gentle_local_yaw_rate", yawRate);
            Academy.Instance.StatsRecorder.Add("FinsROV/gentle_angular_penalty", angularPenalty);
            Academy.Instance.StatsRecorder.Add("FinsROV/gentle_stable_success_steps", stableSuccessStepCount);
        }

        previousDistanceToTarget = distanceToTarget;
        previousHeadingError01 = headingError01;

        if (stableSuccessStepCount >= Mathf.Max(1, stableSuccessStepsRequired))
        {
            AddReward(successReward);
            lastStepReward += successReward;
            EndEpisode();
            return;
        }

        bool outOfBounds =
            Vector3.Distance(ReferenceTransform.position, episodeSpawnPosition) > maxDistanceFromSpawn ||
            ReferenceTransform.position.y > surfaceY + maxHeightAboveSurface;

        if (outOfBounds)
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
                Debug.Log($"[{nameof(ControlForPosition_GentleReward)}] {statusMessage}", this);
                logThrusterResolution = false;
            }
        }
        else
        {
            Debug.LogError($"[{nameof(ControlForPosition_GentleReward)}] {statusMessage}", this);
        }
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
            Mathf.Clamp(normalized.z, -clip, clip)));
    }

    Vector3 GenerateRandomTargetPosition(float radius, Vector3 centerPosition)
    {
        Vector3 randomDirection = Random.insideUnitSphere;
        Vector3 randomPoint = randomDirection.normalized * Random.value * radius;

        return new Vector3(
            centerPosition.x + randomPoint.x,
            centerPosition.y + Mathf.Min(randomPoint.y, surfaceY - centerPosition.y),
            centerPosition.z + randomPoint.z);
    }

    Quaternion GenerateRandomHeading()
    {
        return Quaternion.Euler(0f, Random.Range(0f, 360f), 0f);
    }
}
