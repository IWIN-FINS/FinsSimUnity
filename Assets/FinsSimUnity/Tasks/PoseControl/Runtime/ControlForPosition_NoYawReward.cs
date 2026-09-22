using FinsSim.Actuators;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

[DisallowMultipleComponent]
[RequireComponent(typeof(Rigidbody))]
public class ControlForPosition_NoYawReward : Agent
{
    Rigidbody rigidBody;
    ThrusterController thrusterController;
    Transform referenceTransform;

    readonly Thruster[] orderedThrusters = new Thruster[FinsROVAgentRuntime.DefaultThrusterOrder.Length];
    readonly float[] previousActions = new float[FinsROVAgentRuntime.DefaultThrusterOrder.Length];
    readonly float[] currentActions = new float[FinsROVAgentRuntime.DefaultThrusterOrder.Length];

    [Header("Reference Frame")]
    [Tooltip("Prefer this reference point. If empty, a child named MarkPosition is used; otherwise the root transform is used.")]
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
    [SerializeField] float successDistance = 0.2f;
    [SerializeField] float nearTargetDistance = 0.6f;
    [SerializeField] float stableSuccessLinearVelocity = 0.1f;
    [SerializeField] float stableSuccessAngularVelocity = 0.3f;
    [SerializeField] int stableSuccessStepsRequired = 10;

    [Header("Reward Weights")]
    [SerializeField] float perStepPenalty = -0.001f;
    [SerializeField] float distanceProgressRewardScale = 1.4f;
    [SerializeField] float approachVelocityRewardScale = 0.05f;
    [SerializeField] float stationKeepingRewardScale = 0.10f;
    [SerializeField] float stableHoldRewardScale = 0.05f;
    [SerializeField] float angularPenaltyDeadbandDegPerSec = 20f;
    [SerializeField] float angularVelocityPenaltyScale = 0.02f;
    [SerializeField] float nearTargetAngularVelocityPenaltyScale = 0.12f;
    [SerializeField] float speedNearTargetPenaltyScale = 0.06f;
    [SerializeField] float tangentialVelocityPenaltyScale = 0.03f;
    [SerializeField] float actionEnergyPenaltyScale = 0.0015f;
    [SerializeField] float actionChangePenaltyScale = 0.0015f;
    [SerializeField] float successReward = 2f;
    [SerializeField] float outOfBoundsPenalty = -1f;

    [Header("Safety Limit")]
    [SerializeField] float maxDistanceFromSpawn = 8f;
    [SerializeField] float maxHeightAboveSurface = 0.3f;

    Vector3 episodeSpawnPosition;
    float previousDistanceToTarget;
    float currentDistanceToTarget;
    float currentForwardApproachSpeed;
    float currentNearTargetRatio;
    float currentLocalAngularSpeed;
    float currentTangentialSpeed;
    float currentNearTargetAngularPenalty;
    float lastStepReward;
    int stableSuccessStepCount;
    bool hasDecisionRequester;

    public float LastStepReward => lastStepReward;
    public float CurrentDistanceToTarget => currentDistanceToTarget;
    public float CurrentHeadingError01 => 0f;
    public float CurrentHeadingErrorDeg => 0f;
    public float CurrentDirectionAlignment => 0f;
    public float CurrentForwardApproachSpeed => currentForwardApproachSpeed;
    public float CurrentNearTargetRatio => currentNearTargetRatio;
    public float CurrentLocalAngularSpeed => currentLocalAngularSpeed;
    public float CurrentLocalYawRate => Mathf.Abs(GetLocalAngularVelocity().y);
    public float CurrentTangentialSpeed => currentTangentialSpeed;
    public float CurrentNearTargetAngularPenalty => currentNearTargetAngularPenalty;
    public int StableSuccessStepCount => stableSuccessStepCount;
    public float SuccessDistance => successDistance;
    public float SuccessHeadingAngleDeg => 180f;
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
            Random.Range(0f, spawnPositionRange)
        );

        if (randomizeAgentYawOnReset)
        {
            transform.rotation = Quaternion.Euler(0f, Random.Range(0f, 360f), 0f);
        }

        episodeSpawnPosition = ReferenceTransform.position;

        if (targetTransform != null)
        {
            targetTransform.position = GenerateRandomTargetPosition(targetSpawnRadius, episodeSpawnPosition);
            targetTransform.rotation = Quaternion.identity;
        }

        previousDistanceToTarget = GetDistanceToTarget();
        currentDistanceToTarget = previousDistanceToTarget;
        currentForwardApproachSpeed = 0f;
        currentNearTargetRatio = 0f;
        currentLocalAngularSpeed = 0f;
        currentTangentialSpeed = 0f;
        currentNearTargetAngularPenalty = 0f;
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
        float distanceToTarget = targetTransform != null
            ? localTargetOffset.magnitude
            : 0f;

        sensor.AddObservation(localTargetOffset / positionScale);
        AddRotation6DObservation(sensor, Quaternion.identity);
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
        Transform reference = ReferenceTransform;
        Vector3 toTarget = targetTransform != null ? targetTransform.position - reference.position : Vector3.zero;
        Vector3 directionToTarget = toTarget.sqrMagnitude > 1e-6f ? toTarget.normalized : Vector3.zero;
        Vector3 localLinearVelocity = GetLocalLinearVelocity();
        Vector3 localAngularVelocity = GetLocalAngularVelocity();
        Vector3 worldLinearVelocity = RigidBody.linearVelocity;

        float distanceProgress = previousDistanceToTarget - distanceToTarget;
        float forwardApproachSpeed = directionToTarget.sqrMagnitude > 1e-6f
            ? Vector3.Dot(worldLinearVelocity, directionToTarget)
            : 0f;
        Vector3 radialVelocity = directionToTarget * forwardApproachSpeed;
        float tangentialSpeed = directionToTarget.sqrMagnitude > 1e-6f
            ? (worldLinearVelocity - radialVelocity).magnitude
            : 0f;

        float nearTargetRatio = 1f - Mathf.Clamp01(distanceToTarget / Mathf.Max(nearTargetDistance, 0.01f));
        float nearTargetGain = nearTargetRatio * nearTargetRatio * (3f - 2f * nearTargetRatio);
        float angularSpeed = localAngularVelocity.magnitude;
        float angularPenaltyDeadband = angularPenaltyDeadbandDegPerSec * Mathf.Deg2Rad;
        float angularSpeedExcess = Mathf.Max(0f, angularSpeed - angularPenaltyDeadband);
        float nearTargetAngularPenalty =
            nearTargetGain * nearTargetAngularVelocityPenaltyScale * angularSpeedExcess * angularSpeedExcess;
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

        bool inSuccessPosition = distanceToTarget < successDistance;
        bool stableForSuccess =
            localLinearVelocity.magnitude < stableSuccessLinearVelocity &&
            angularSpeed < stableSuccessAngularVelocity;

        if (inSuccessPosition && stableForSuccess)
        {
            stableSuccessStepCount++;
        }
        else
        {
            stableSuccessStepCount = 0;
        }

        float reward =
            perStepPenalty +
            distanceProgressRewardScale * distanceProgress +
            approachVelocityRewardScale * Mathf.Clamp(forwardApproachSpeed, -1f, 1f) +
            stationKeepingRewardScale * nearTargetRatio +
            stableHoldRewardScale * nearTargetGain * (stableForSuccess ? 1f : 0f) -
            angularVelocityPenaltyScale * angularSpeedExcess * angularSpeedExcess -
            nearTargetAngularPenalty -
            speedNearTargetPenaltyScale * nearTargetRatio * localLinearVelocity.magnitude -
            tangentialVelocityPenaltyScale * nearTargetGain * tangentialSpeed -
            actionEnergyPenaltyScale * actionEnergy -
            actionChangePenaltyScale * actionDelta;

        AddReward(reward);
        lastStepReward = reward;
        currentDistanceToTarget = distanceToTarget;
        currentForwardApproachSpeed = forwardApproachSpeed;
        currentNearTargetRatio = nearTargetRatio;
        currentLocalAngularSpeed = angularSpeed;
        currentTangentialSpeed = tangentialSpeed;
        currentNearTargetAngularPenalty = nearTargetAngularPenalty;

        if (enableStatsRecorder)
        {
            Academy.Instance.StatsRecorder.Add("FinsROV/no_yaw_distance", distanceToTarget);
            Academy.Instance.StatsRecorder.Add("FinsROV/no_yaw_near_target_ratio", nearTargetRatio);
            Academy.Instance.StatsRecorder.Add("FinsROV/no_yaw_approach_speed", forwardApproachSpeed);
            Academy.Instance.StatsRecorder.Add("FinsROV/no_yaw_tangential_speed", tangentialSpeed);
            Academy.Instance.StatsRecorder.Add("FinsROV/no_yaw_local_angular_speed", angularSpeed);
            Academy.Instance.StatsRecorder.Add("FinsROV/no_yaw_stable_success_steps", stableSuccessStepCount);
        }

        previousDistanceToTarget = distanceToTarget;

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
                Debug.Log($"[{nameof(ControlForPosition_NoYawReward)}] {statusMessage}", this);
                logThrusterResolution = false;
            }
        }
        else
        {
            Debug.LogError($"[{nameof(ControlForPosition_NoYawReward)}] {statusMessage}", this);
        }
    }

    float GetDistanceToTarget()
    {
        return targetTransform != null
            ? Vector3.Distance(ReferenceTransform.position, targetTransform.position)
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

    Vector3 GenerateRandomTargetPosition(float radius, Vector3 centerPosition)
    {
        Vector3 randomDirection = Random.insideUnitSphere;
        Vector3 randomPoint = randomDirection.normalized * Random.value * radius;

        return new Vector3(
            centerPosition.x + randomPoint.x,
            centerPosition.y + Mathf.Min(randomPoint.y, surfaceY - centerPosition.y),
            centerPosition.z + randomPoint.z
        );
    }
}
