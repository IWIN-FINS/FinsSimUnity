using FinsSim.Actuators;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

[DisallowMultipleComponent]
[RequireComponent(typeof(Rigidbody))]
public class ControlForMovingTargetReward : Agent
{
    Rigidbody rigidBody;
    ThrusterController thrusterController;
    Transform referenceTransform;
    SmoothRandomMovingTarget targetMotion;
    Rigidbody targetRigidbody;

    readonly Thruster[] orderedThrusters = new Thruster[FinsROVAgentRuntime.DefaultThrusterOrder.Length];
    readonly float[] previousActions = new float[FinsROVAgentRuntime.DefaultThrusterOrder.Length];
    readonly float[] currentActions = new float[FinsROVAgentRuntime.DefaultThrusterOrder.Length];

    [Header("Reference Frame")]
    [Tooltip("优先使用这个参考点。为空时自动查找名为 MarkPosition 的子物体；如果不存在，则退回模型自身 Transform。")]
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
    [Tooltip("Maximum initial distance between the agent reference point and target at episode reset.")]
    [SerializeField] float maxInitialTargetDistance = 2f;
    [SerializeField] float surfaceY = 0f;
    [SerializeField] bool randomizeAgentYawOnReset = true;
    [SerializeField] bool resetMovingTargetOnEpisodeBegin = true;

    [Header("Observation Normalization")]
    [SerializeField] float linearVelocityObservationScale = 1f;
    [SerializeField] float angularVelocityObservationScale = 1f;
    [SerializeField] float targetVelocityObservationScale = 0.2f;
    [SerializeField] float normalizedVelocityObservationClip = 2f;

    [Header("Success Threshold")]
    [SerializeField] float successDistance = 0.25f;
    [SerializeField] float nearTargetDistance = 0.8f;
    [SerializeField] float stableSuccessRelativeSpeed = 0.12f;
    [SerializeField] float stableSuccessAngularVelocity = 0.3f;
    [SerializeField] int stableSuccessStepsRequired = 10;

    [Header("Reward Weights")]
    [SerializeField] float perStepPenalty = -0.001f;
    [SerializeField] float distanceProgressRewardScale = 1.2f;
    [SerializeField] float directionAlignmentRewardScale = 0.03f;
    [SerializeField] float approachVelocityRewardScale = 0.05f;
    [SerializeField] float nearTargetRewardScale = 0.08f;
    [SerializeField] float relativeVelocityRewardScale = 0.06f;
    [SerializeField] float relativeVelocityMatchSpeed = 0.25f;
    [SerializeField] float angularVelocityPenaltyScale = 0.01f;
    [SerializeField] float speedNearTargetPenaltyScale = 0.04f;
    [SerializeField] float actionEnergyPenaltyScale = 0.0015f;
    [SerializeField] float actionChangePenaltyScale = 0.001f;
    [SerializeField] float stableHoldRewardScale = 0.04f;
    [SerializeField] float successReward = 2f;
    [SerializeField] float outOfBoundsPenalty = -1f;

    [Header("Safety Limit")]
    [SerializeField] float maxDistanceFromSpawn = 8f;
    [SerializeField] float maxHeightAboveSurface = 0.3f;

    Vector3 episodeSpawnPosition;
    Vector3 previousTargetPosition;
    Vector3 estimatedTargetVelocity;
    float previousDistanceToTarget;
    float currentDistanceToTarget;
    float currentDirectionAlignment;
    float currentForwardApproachSpeed;
    float currentNearTargetRatio;
    float currentRelativeTargetSpeed;
    float currentLocalAngularSpeed;
    float lastStepReward;
    int stableSuccessStepCount;
    bool hasTargetPositionSample;

    public float LastStepReward => lastStepReward;
    public float CurrentDistanceToTarget => currentDistanceToTarget;
    public float CurrentDirectionAlignment => currentDirectionAlignment;
    public float CurrentForwardApproachSpeed => currentForwardApproachSpeed;
    public float CurrentNearTargetRatio => currentNearTargetRatio;
    public Vector3 CurrentTargetVelocity => GetTargetVelocity();
    public float CurrentRelativeTargetSpeed => currentRelativeTargetSpeed;
    public float CurrentLocalAngularSpeed => currentLocalAngularSpeed;
    public int StableSuccessStepCount => stableSuccessStepCount;
    public float SuccessDistance => successDistance;
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
        ResolveRuntimeReferences();
    }

    void FixedUpdate()
    {
        if (autoRequestDecisionWhenNoDecisionRequester)
        {
            // The FinsROV prefab can keep a DecisionRequester bound to a disabled legacy Agent.
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
            targetTransform.position = GenerateRandomTargetPosition(GetInitialTargetSpawnRadius(), episodeSpawnPosition);
            targetTransform.rotation = Quaternion.identity;
            ResetMovingTarget();
        }

        CaptureTargetPositionSample();
        previousDistanceToTarget = GetDistanceToTarget();
        currentDistanceToTarget = previousDistanceToTarget;
        currentDirectionAlignment = 0f;
        currentForwardApproachSpeed = 0f;
        currentNearTargetRatio = 0f;
        currentRelativeTargetSpeed = 0f;
        currentLocalAngularSpeed = 0f;
        lastStepReward = 0f;
        stableSuccessStepCount = 0;
        System.Array.Clear(previousActions, 0, previousActions.Length);
        System.Array.Clear(currentActions, 0, currentActions.Length);

        if (autoRequestDecisionWhenNoDecisionRequester)
        {
            RequestDecision();
        }
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        Transform reference = ReferenceTransform;
        float positionScale = Mathf.Max(targetSpawnRadius, 0.01f);
        Vector3 localTargetOffset = targetTransform != null
            ? reference.InverseTransformPoint(targetTransform.position)
            : Vector3.zero;
        Vector3 localTargetVelocity = reference.InverseTransformDirection(GetTargetVelocity());
        float distanceToTarget = targetTransform != null
            ? localTargetOffset.magnitude
            : 0f;

        sensor.AddObservation(localTargetOffset / positionScale);
        AddNormalizedVectorObservation(sensor, localTargetVelocity, targetVelocityObservationScale);
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
        UpdateTargetVelocityEstimate();

        Transform reference = ReferenceTransform;
        float distanceToTarget = GetDistanceToTarget();
        Vector3 toTarget = targetTransform != null ? targetTransform.position - reference.position : Vector3.zero;
        Vector3 directionToTarget = toTarget.sqrMagnitude > 1e-6f ? toTarget.normalized : reference.forward;
        Vector3 localAngularVelocity = GetLocalAngularVelocity();
        Vector3 targetVelocity = GetTargetVelocity();
        Vector3 relativeTargetVelocity = RigidBody.linearVelocity - targetVelocity;

        float distanceProgress = previousDistanceToTarget - distanceToTarget;
        float directionAlignment = 0f;
        if (relativeTargetVelocity.sqrMagnitude > 1e-6f && toTarget.sqrMagnitude > 1e-6f)
        {
            directionAlignment = Vector3.Dot(relativeTargetVelocity.normalized, directionToTarget);
        }

        float forwardApproachSpeed = Vector3.Dot(relativeTargetVelocity, directionToTarget);
        float nearTargetRatio = 1f - Mathf.Clamp01(distanceToTarget / Mathf.Max(nearTargetDistance, 0.01f));
        float nearTargetGain = nearTargetRatio * nearTargetRatio;
        float angularSpeed = localAngularVelocity.magnitude;
        float relativeTargetSpeed = relativeTargetVelocity.magnitude;
        float velocityMatchReward = nearTargetRatio * (
            1f - Mathf.Clamp01(relativeTargetSpeed / Mathf.Max(relativeVelocityMatchSpeed, 0.01f))
        );
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

        bool inSuccessPose = distanceToTarget < successDistance;
        bool stableForSuccess =
            relativeTargetSpeed < stableSuccessRelativeSpeed &&
            angularSpeed < stableSuccessAngularVelocity;

        stableSuccessStepCount = inSuccessPose && stableForSuccess
            ? stableSuccessStepCount + 1
            : 0;

        float reward =
            perStepPenalty +
            distanceProgressRewardScale * distanceProgress +
            directionAlignmentRewardScale * directionAlignment +
            approachVelocityRewardScale * Mathf.Clamp(forwardApproachSpeed, -1f, 1f) +
            nearTargetRewardScale * nearTargetRatio +
            relativeVelocityRewardScale * velocityMatchReward +
            stableHoldRewardScale * nearTargetGain * (stableForSuccess ? 1f : 0f) -
            angularVelocityPenaltyScale * angularSpeed -
            speedNearTargetPenaltyScale * nearTargetRatio * relativeTargetSpeed -
            actionEnergyPenaltyScale * actionEnergy -
            actionChangePenaltyScale * actionDelta;

        AddReward(reward);
        lastStepReward = reward;
        currentDistanceToTarget = distanceToTarget;
        currentDirectionAlignment = directionAlignment;
        currentForwardApproachSpeed = forwardApproachSpeed;
        currentNearTargetRatio = nearTargetRatio;
        currentRelativeTargetSpeed = relativeTargetSpeed;
        currentLocalAngularSpeed = angularSpeed;

        if (enableStatsRecorder)
        {
            Academy.Instance.StatsRecorder.Add("FinsROV/moving_target_distance", distanceToTarget);
            Academy.Instance.StatsRecorder.Add("FinsROV/moving_target_speed", targetVelocity.magnitude);
            Academy.Instance.StatsRecorder.Add("FinsROV/relative_target_speed", relativeTargetSpeed);
            Academy.Instance.StatsRecorder.Add("FinsROV/moving_target_near_ratio", nearTargetRatio);
            Academy.Instance.StatsRecorder.Add("FinsROV/moving_target_stable_steps", stableSuccessStepCount);
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
        ResolveTargetMotionReferences();

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
                Debug.Log($"[{nameof(ControlForMovingTargetReward)}] {statusMessage}", this);
                logThrusterResolution = false;
            }
        }
        else
        {
            Debug.LogError($"[{nameof(ControlForMovingTargetReward)}] {statusMessage}", this);
        }
    }

    void ResolveTargetMotionReferences()
    {
        if (targetTransform == null)
        {
            targetMotion = null;
            targetRigidbody = null;
            return;
        }

        if (targetMotion == null || targetMotion.transform != targetTransform)
        {
            targetMotion = targetTransform.GetComponent<SmoothRandomMovingTarget>();
        }

        if (targetRigidbody == null || targetRigidbody.transform != targetTransform)
        {
            targetRigidbody = targetTransform.GetComponent<Rigidbody>();
        }
    }

    void ResetMovingTarget()
    {
        ResolveTargetMotionReferences();
        if (resetMovingTargetOnEpisodeBegin && targetMotion != null)
        {
            targetMotion.ResetMotionAtCurrentPosition();
        }
    }

    void CaptureTargetPositionSample()
    {
        if (targetTransform == null)
        {
            previousTargetPosition = Vector3.zero;
            estimatedTargetVelocity = Vector3.zero;
            hasTargetPositionSample = false;
            return;
        }

        previousTargetPosition = targetTransform.position;
        estimatedTargetVelocity = GetTargetVelocity();
        hasTargetPositionSample = true;
    }

    void UpdateTargetVelocityEstimate()
    {
        if (targetTransform == null)
        {
            estimatedTargetVelocity = Vector3.zero;
            hasTargetPositionSample = false;
            return;
        }

        if (targetMotion != null)
        {
            estimatedTargetVelocity = targetMotion.CurrentVelocity;
        }
        else if (targetRigidbody != null)
        {
            estimatedTargetVelocity = targetRigidbody.linearVelocity;
        }
        else if (hasTargetPositionSample)
        {
            float deltaTime = Time.fixedDeltaTime > 0f ? Time.fixedDeltaTime : Time.deltaTime;
            estimatedTargetVelocity = deltaTime > 0f
                ? (targetTransform.position - previousTargetPosition) / deltaTime
                : Vector3.zero;
        }
        else
        {
            estimatedTargetVelocity = Vector3.zero;
        }

        previousTargetPosition = targetTransform.position;
        hasTargetPositionSample = true;
    }

    Vector3 GetTargetVelocity()
    {
        if (targetMotion != null)
        {
            return targetMotion.CurrentVelocity;
        }

        if (targetRigidbody != null)
        {
            return targetRigidbody.linearVelocity;
        }

        return estimatedTargetVelocity;
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

    float GetInitialTargetSpawnRadius()
    {
        return Mathf.Min(
            Mathf.Max(0f, targetSpawnRadius),
            Mathf.Max(0f, maxInitialTargetDistance));
    }
}
