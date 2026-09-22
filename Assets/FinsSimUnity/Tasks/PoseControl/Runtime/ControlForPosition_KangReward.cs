using FinsSim.Actuators;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

[DisallowMultipleComponent]
[RequireComponent(typeof(Rigidbody))]
public class ControlForPosition_KangReward : Agent
{
    Rigidbody rigidBody;
    ThrusterController thrusterController;
    Transform referenceTransform;

    readonly Thruster[] orderedThrusters = new Thruster[FinsROVAgentRuntime.DefaultThrusterOrder.Length];
    readonly float[] previousActions = new float[FinsROVAgentRuntime.DefaultThrusterOrder.Length];
    readonly float[] currentActions = new float[FinsROVAgentRuntime.DefaultThrusterOrder.Length];

    [Header("Reference Frame")]
    [Tooltip("ä¼˜å…ˆä½¿ç”¨è¿™ä¸ªå‚è€ƒç‚¹ã€‚ä¸ºç©ºæ—¶è‡ªåŠ¨æŸ¥æ‰¾åä¸º MarkPosition çš„å­ç‰©ä½“ï¼›å¦‚æžœä¸å­˜åœ¨ï¼Œåˆ™é€€å›žæ¨¡åž‹è‡ªèº« Transformã€‚")]
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
    [SerializeField] float episodeDurationSeconds = 3f;
    [SerializeField] float spawnPositionRange = 2f;
    [SerializeField] float spawnDepthOffset = -2f;
    [SerializeField] float targetSpawnRadius = 2f;
    [SerializeField] float surfaceY = 0f;
    [SerializeField] bool randomizeAgentYawOnReset = true;

    [Header("Reward Weights")]
    [SerializeField] float positionRewardWeight = 2f;
    [SerializeField] float orientationRewardWeight = 1f;
    [SerializeField] float energyRewardWeight = 1f;

    [Header("Observation Normalization")]
    [SerializeField] float linearVelocityObservationScale = 1f;
    [SerializeField] float angularVelocityObservationScale = 1f;
    [SerializeField] float normalizedVelocityObservationClip = 2f;

    [Header("Success Threshold")]
    [SerializeField] float successDistance = 0.2f;
    [SerializeField] float successHeadingAngleDeg = 25f;
    [SerializeField] float nearTargetDistance = 0.6f;
    [SerializeField] float stableSuccessLinearVelocity = 0.1f;
    [SerializeField] float stableSuccessAngularVelocity = 0.2f;
    [SerializeField] int stableSuccessStepsRequired = 10;

    [Header("Safety Limit")]
    [SerializeField] float maxDistanceFromSpawn = 8f;
    [SerializeField] float maxHeightAboveSurface = 0.3f;

    Vector3 episodeSpawnPosition;
    float episodeStartTime;
    float previousDistanceToTarget;
    float previousHeadingError01;
    float currentDistanceToTarget;
    float currentHeadingError01;
    float currentHeadingErrorRad;
    float currentDirectionAlignment;
    float currentForwardApproachSpeed;
    float currentNearTargetRatio;
    float currentLocalAngularSpeed;
    float currentLocalYawRate;
    float currentPositionReward;
    float currentOrientationReward;
    float currentEnergyReward;
    float lastStepReward;
    int stableSuccessStepCount;
    bool hasDecisionRequester;

    public float LastStepReward => lastStepReward;
    public float CurrentDistanceToTarget => currentDistanceToTarget;
    public float CurrentHeadingError01 => currentHeadingError01;
    public float CurrentHeadingErrorDeg => currentHeadingError01 * 180f;
    public float CurrentHeadingErrorRad => currentHeadingErrorRad;
    public float CurrentDirectionAlignment => currentDirectionAlignment;
    public float CurrentForwardApproachSpeed => currentForwardApproachSpeed;
    public float CurrentNearTargetRatio => currentNearTargetRatio;
    public float CurrentLocalAngularSpeed => currentLocalAngularSpeed;
    public float CurrentLocalYawRate => currentLocalYawRate;
    public float CurrentPositionReward => currentPositionReward;
    public float CurrentOrientationReward => currentOrientationReward;
    public float CurrentEnergyReward => currentEnergyReward;
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
        // Episode length is controlled by episodeDurationSeconds, not MaxStep.
        MaxStep = 0;
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
        MaxStep = 0;
        ResolveRuntimeReferences();
        FinsROVAgentRuntime.RandomizeEpisodeIfPresent(this);

        RigidBody.angularVelocity = Vector3.zero;
        RigidBody.linearVelocity = Vector3.zero;
        FinsROVAgentRuntime.ZeroThrusters(orderedThrusters, thrusterCommandMode, actionForceScaleN);
        episodeStartTime = Time.time;

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
            targetTransform.rotation = GenerateRandomHeading();
        }

        previousDistanceToTarget = GetDistanceToTarget();
        previousHeadingError01 = GetHeadingError01();
        currentDistanceToTarget = previousDistanceToTarget;
        currentHeadingError01 = previousHeadingError01;
        currentHeadingErrorRad = previousHeadingError01 * Mathf.PI;
        currentDirectionAlignment = 0f;
        currentForwardApproachSpeed = 0f;
        currentNearTargetRatio = 0f;
        currentLocalAngularSpeed = 0f;
        currentLocalYawRate = 0f;
        currentPositionReward = 0f;
        currentOrientationReward = 0f;
        currentEnergyReward = 0f;
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
        float distanceToTarget = targetTransform != null
            ? localTargetOffset.magnitude
            : 0f;

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
        float headingErrorRad = headingError01 * Mathf.PI;

        Transform reference = ReferenceTransform;
        Vector3 toTarget = targetTransform != null ? targetTransform.position - reference.position : Vector3.zero;
        Vector3 directionToTarget = toTarget.sqrMagnitude > 1e-6f ? toTarget.normalized : reference.forward;
        Vector3 localLinearVelocity = GetLocalLinearVelocity();
        Vector3 localAngularVelocity = GetLocalAngularVelocity();

        Vector3 referenceForwardFlat = new Vector3(reference.forward.x, 0f, reference.forward.z);
        Vector3 targetDirectionFlat = new Vector3(directionToTarget.x, 0f, directionToTarget.z);
        float directionAlignment = 0f;
        if (referenceForwardFlat.sqrMagnitude > 1e-6f && targetDirectionFlat.sqrMagnitude > 1e-6f)
        {
            directionAlignment = Vector3.Dot(referenceForwardFlat.normalized, targetDirectionFlat.normalized);
        }

        float forwardApproachSpeed = Vector3.Dot(RigidBody.linearVelocity, directionToTarget);
        float nearTargetRatio = 1f - Mathf.Clamp01(distanceToTarget / Mathf.Max(nearTargetDistance, 0.01f));
        float angularSpeed = localAngularVelocity.magnitude;
        float yawRate = Mathf.Abs(localAngularVelocity.y);

        float actionNormSq = 0f;
        for (int i = 0; i < currentActions.Length; i++)
        {
            actionNormSq += currentActions[i] * currentActions[i];
            previousActions[i] = currentActions[i];
        }

        // Reward = w_p * Position + w_o * Orientation + w_e * Energy
        // Position:    exp(-distance^2)
        // Orientation: exp(-angle)  with angle in radians
        // Energy:      exp(-||action||^2)
        float positionReward = positionRewardWeight * Mathf.Exp(-(distanceToTarget * distanceToTarget));
        float orientationReward = orientationRewardWeight * Mathf.Exp(-headingErrorRad);
        float energyReward = energyRewardWeight * Mathf.Exp(-actionNormSq);
        float reward = positionReward + orientationReward + energyReward;

        bool inSuccessPose =
            distanceToTarget < successDistance &&
            headingError01 < successHeadingAngleDeg / 180f;
        bool stableForSuccess =
            localLinearVelocity.magnitude < stableSuccessLinearVelocity &&
            angularSpeed < stableSuccessAngularVelocity;

        if (inSuccessPose && stableForSuccess)
        {
            stableSuccessStepCount++;
        }
        else
        {
            stableSuccessStepCount = 0;
        }

        AddReward(reward);
        lastStepReward = reward;
        currentDistanceToTarget = distanceToTarget;
        currentHeadingError01 = headingError01;
        currentHeadingErrorRad = headingErrorRad;
        currentDirectionAlignment = directionAlignment;
        currentForwardApproachSpeed = forwardApproachSpeed;
        currentNearTargetRatio = nearTargetRatio;
        currentLocalAngularSpeed = angularSpeed;
        currentLocalYawRate = yawRate;
        currentPositionReward = positionReward;
        currentOrientationReward = orientationReward;
        currentEnergyReward = energyReward;

        if (enableStatsRecorder)
        {
            Academy.Instance.StatsRecorder.Add("FinsROV/position_reward", positionReward);
            Academy.Instance.StatsRecorder.Add("FinsROV/orientation_reward", orientationReward);
            Academy.Instance.StatsRecorder.Add("FinsROV/energy_reward", energyReward);
            Academy.Instance.StatsRecorder.Add("FinsROV/stable_success_steps", stableSuccessStepCount);
        }

        previousDistanceToTarget = distanceToTarget;
        previousHeadingError01 = headingError01;

        bool outOfBounds =
            Vector3.Distance(ReferenceTransform.position, episodeSpawnPosition) > maxDistanceFromSpawn ||
            ReferenceTransform.position.y > surfaceY + maxHeightAboveSurface;

        if (outOfBounds)
        {
            EndEpisode();
            return;
        }

        if (Time.time - episodeStartTime >= episodeDurationSeconds)
        {
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
                Debug.Log($"[{nameof(ControlForPosition_KangReward)}] {statusMessage}", this);
                logThrusterResolution = false;
            }
        }
        else
        {
            Debug.LogError($"[{nameof(ControlForPosition_KangReward)}] {statusMessage}", this);
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

    Quaternion GenerateRandomHeading()
    {
        return Quaternion.Euler(0f, Random.Range(0f, 360f), 0f);
    }
}
