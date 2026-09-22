using FinsSim.Actuators;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

/// <summary>
/// Single-goal position-and-yaw task for the standalone FinsSim AIRL/GAIL
/// backend.  Its vector observation and eight-thruster action ABI are kept
/// deliberately identical to the current single-agent RL backend:
/// [target_local(3), relative_rotation_6d(6), local_velocity(3),
/// local_angular_velocity(3), normalized_distance(1)] and the canonical
/// FinsROVAgentRuntime thruster order.
/// </summary>
[DisallowMultipleComponent]
[RequireComponent(typeof(Rigidbody))]
public sealed class GoalYawIrlAgent : Agent
{
    public const int VectorObservationSize = 16;
    public const int ContinuousActionSize = 8;
    const string StratifiedResetParameter = "goal_yaw_stratified_resets";
    const int DistanceStrata = 3;
    const int YawStrata = 4;
    const int VerticalStrata = 2;
    // The Python PID collector uses this deliberately separated terminal
    // signal to retain successful demonstrations only.  It is a diagnostic
    // signal: AIRL/GAIL still replace Unity's native reward during training.
    public const float SuccessfulTerminalReward = 10f;

    readonly Thruster[] orderedThrusters = new Thruster[ContinuousActionSize];
    readonly float[] currentActions = new float[ContinuousActionSize];
    readonly float[] previousActions = new float[ContinuousActionSize];

    [Header("Scene References")]
    [SerializeField] Transform selfTransform;
    [SerializeField] Transform targetTransform;

    [Header("Thruster Control")]
    [SerializeField] ThrusterController thrusterControllerOverride;
    // Each signed action uses this thruster's current forward/reverse
    // calibrated force limit.  The scene may therefore expose asymmetric
    // per-thruster force envelopes.
    [SerializeField] ThrusterCommandMode thrusterCommandMode = ThrusterCommandMode.NormalizedMaxForceRequest;
    [ShowWhenThrusterCommandMode(ThrusterCommandMode.ScaledForceRequest)]
    [SerializeField] float actionForceScaleN = 7f;
    [SerializeField] bool autoResolveThrustersFromChildren = true;
    [SerializeField] bool logThrusterResolution = true;
    [SerializeField] float heuristicYawMixScale = 1f;
    [SerializeField] float heuristicAuxMixScale = 1f;
    [SerializeField] bool autoRequestDecisionWhenNoDecisionRequester = true;

    [Header("GoalYaw Episode")]
    [SerializeField] float spawnPositionRange = 2f;
    [SerializeField] float spawnDepthOffset = -2f;
    [SerializeField] float targetSpawnRadius = 3f;
    [SerializeField] float minimumTargetDistance = 0.5f;
    [SerializeField] float surfaceY = 0f;
    [SerializeField] bool randomizeAgentYawOnReset = true;
    [SerializeField] float successDistanceM = 0.15f;
    [SerializeField] float successYawErrorDeg = 10f;
    [SerializeField] float successLinearSpeedMps = 0.2f;
    [SerializeField] float successAngularSpeedDegPerSec = 20f;
    [SerializeField, Min(0.02f)] float stableSuccessDurationSecondsRequired = 5f;
    [SerializeField] float maxDistanceFromSpawn = 8f;
    [SerializeField] float maxHeightAboveSurface = 0.3f;

    [Header("Observation Normalization")]
    [SerializeField] float linearVelocityObservationScale = 1f;
    [SerializeField] float angularVelocityObservationScale = 1f;
    [SerializeField] float normalizedVelocityObservationClip = 2f;

    [Header("Native Task Diagnostics")]
    [SerializeField] bool enableStatsRecorder = true;

    Rigidbody rigidBody;
    ThrusterController thrusterController;
    Transform referenceTransform;
    Vector3 episodeSpawnPosition;
    float previousDistanceM;
    float previousYawErrorDeg;
    float stableSuccessDurationSeconds;
    int resetIndex;
    int episodeStratumIndex = -1;
    bool hasDecisionRequester;

    public Transform TargetTransform => targetTransform;
    public ThrusterCommandMode CurrentThrusterCommandMode => thrusterCommandMode;
    public Transform ReferenceTransform => referenceTransform != null
        ? referenceTransform
        : FinsROVAgentRuntime.ResolveReferenceTransform(transform, selfTransform);
    public float CurrentDistanceM { get; private set; }
    public float CurrentYawErrorDeg { get; private set; }
    public float CurrentLinearSpeedMps { get; private set; }
    public float CurrentAngularSpeedDegPerSec { get; private set; }
    public bool CurrentSuccess =>
        CurrentDistanceM <= successDistanceM &&
        CurrentYawErrorDeg <= successYawErrorDeg &&
        CurrentLinearSpeedMps <= successLinearSpeedMps &&
        CurrentAngularSpeedDegPerSec <= successAngularSpeedDegPerSec;

    /// <summary>Called only by the generated IRL scene builder.</summary>
    public void ConfigureScene(Transform reference, Transform target, int episodeMaxStep)
    {
        selfTransform = reference;
        targetTransform = target;
        MaxStep = episodeMaxStep;
        // Preserve the normalized-max-force ABI used by the direct-thruster
        // RL scenes: each action is scaled by the target thruster's current
        // forward/reverse force bound.
        thrusterCommandMode = ThrusterCommandMode.NormalizedMaxForceRequest;
        // The generated scene is an artifact.  Set the task criteria here so
        // rebuilding it updates existing serialized components as well.
        successDistanceM = 0.15f;
        stableSuccessDurationSecondsRequired = 5f;
    }

    /// <summary>Sets the serialized action-to-thruster mapping for the generated scene.</summary>
    public void ConfigureThrusterCommandMode(ThrusterCommandMode commandMode)
    {
        thrusterCommandMode = commandMode;
    }

    void Start()
    {
        ResolveRuntimeReferences();
        hasDecisionRequester = GetComponent("DecisionRequester") != null;
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

        rigidBody.angularVelocity = Vector3.zero;
        rigidBody.linearVelocity = Vector3.zero;
        FinsROVAgentRuntime.ZeroThrusters(orderedThrusters, thrusterCommandMode, actionForceScaleN);

        episodeStratumIndex = UseStratifiedResets()
            ? resetIndex++ % (DistanceStrata * YawStrata * VerticalStrata)
            : -1;
        transform.position = new Vector3(
            Random.Range(0f, spawnPositionRange),
            Random.Range(spawnDepthOffset, spawnDepthOffset + spawnPositionRange),
            Random.Range(0f, spawnPositionRange));
        if (randomizeAgentYawOnReset)
        {
            transform.rotation = Quaternion.Euler(0f, Random.Range(0f, 360f), 0f);
        }

        episodeSpawnPosition = ReferenceTransform.position;
        if (targetTransform == null)
        {
            throw new MissingReferenceException("GoalYawIrlAgent requires a target Transform.");
        }
        if (episodeStratumIndex >= 0)
        {
            GenerateStratifiedGoal(episodeSpawnPosition, episodeStratumIndex, out Vector3 targetPosition, out float targetYawDeg);
            targetTransform.position = targetPosition;
            targetTransform.rotation = Quaternion.Euler(0f, targetYawDeg, 0f);
        }
        else
        {
            targetTransform.position = GenerateGoalPosition(episodeSpawnPosition);
            targetTransform.rotation = Quaternion.Euler(0f, Random.Range(0f, 360f), 0f);
        }

        CurrentDistanceM = DistanceToTarget();
        CurrentYawErrorDeg = YawErrorDeg();
        UpdateSpeedMetrics();
        previousDistanceM = CurrentDistanceM;
        previousYawErrorDeg = CurrentYawErrorDeg;
        stableSuccessDurationSeconds = 0f;
        System.Array.Clear(currentActions, 0, currentActions.Length);
        System.Array.Clear(previousActions, 0, previousActions.Length);
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        Transform reference = ReferenceTransform;
        Vector3 targetPosition = targetTransform != null ? targetTransform.position : reference.position;
        Quaternion targetRotation = targetTransform != null ? targetTransform.rotation : Quaternion.identity;
        Quaternion inverseReferenceRotation = Quaternion.Inverse(reference.rotation);
        Vector3 localTargetOffset = inverseReferenceRotation * (targetPosition - reference.position);
        Quaternion relativeTargetRotation = inverseReferenceRotation * targetRotation;
        float positionScale = Mathf.Max(targetSpawnRadius, 0.01f);

        sensor.AddObservation(localTargetOffset / positionScale);
        AddRotation6DObservation(sensor, relativeTargetRotation);
        AddNormalizedVectorObservation(sensor, LocalLinearVelocity(), linearVelocityObservationScale);
        AddNormalizedVectorObservation(sensor, LocalAngularVelocity(), angularVelocityObservationScale);
        sensor.AddObservation(Mathf.Clamp01(localTargetOffset.magnitude / positionScale));
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        ResolveRuntimeReferences();
        int actionCount = Mathf.Min(ContinuousActionSize, actionBuffers.ContinuousActions.Length);
        for (int i = 0; i < ContinuousActionSize; i++)
        {
            currentActions[i] = i < actionCount ? Mathf.Clamp(actionBuffers.ContinuousActions[i], -1f, 1f) : 0f;
        }
        FinsROVAgentRuntime.ApplyThrusterActions(orderedThrusters, currentActions, thrusterCommandMode, actionForceScaleN);

        CurrentDistanceM = DistanceToTarget();
        CurrentYawErrorDeg = YawErrorDeg();
        UpdateSpeedMetrics();
        float actionEnergy = 0f;
        float actionDelta = 0f;
        for (int i = 0; i < ContinuousActionSize; i++)
        {
            actionEnergy += Mathf.Abs(currentActions[i]);
            actionDelta += Mathf.Abs(currentActions[i] - previousActions[i]);
            previousActions[i] = currentActions[i];
        }
        actionEnergy /= ContinuousActionSize;
        actionDelta /= ContinuousActionSize;

        // Native reward is intentionally only a diagnostic/evaluation metric.
        // GAIL/AIRL replace it with their learned discriminator reward at
        // training time through the Python adversarial trainer.
        float distanceProgress = Mathf.Clamp(previousDistanceM - CurrentDistanceM, -0.25f, 0.25f);
        float yawProgress = Mathf.Clamp((previousYawErrorDeg - CurrentYawErrorDeg) / 180f, -0.25f, 0.25f);
        AddReward(-0.001f + distanceProgress + 0.25f * yawProgress - 0.002f * actionEnergy - 0.001f * actionDelta);

        bool inSuccessPose = CurrentSuccess;
        // OnActionReceived executes once per physics action application (not
        // merely once per ML-Agents decision request).  Accumulate simulated
        // physics time so this criterion remains exactly five simulation
        // seconds with the 10 Hz DecisionRequester and with any time_scale.
        stableSuccessDurationSeconds = inSuccessPose
            ? stableSuccessDurationSeconds + Time.fixedDeltaTime
            : 0f;
        RecordStepStats();
        previousDistanceM = CurrentDistanceM;
        previousYawErrorDeg = CurrentYawErrorDeg;

        if (stableSuccessDurationSeconds >= stableSuccessDurationSecondsRequired)
        {
            SetReward(SuccessfulTerminalReward);
            RecordTerminalStats(true, "success");
            EndEpisode();
            return;
        }

        bool outOfBounds = Vector3.Distance(ReferenceTransform.position, episodeSpawnPosition) > maxDistanceFromSpawn ||
            ReferenceTransform.position.y > surfaceY + maxHeightAboveSurface;
        if (outOfBounds)
        {
            AddReward(-1f);
            RecordTerminalStats(false, "out_of_bounds");
            EndEpisode();
            return;
        }

        // End explicitly one decision before ML-Agents reaches MaxStep so the
        // success/failure statistic is present for evaluation in every episode.
        if (MaxStep > 0 && StepCount >= MaxStep - 1)
        {
            RecordTerminalStats(false, "time_limit");
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
        rigidBody = rigidBody != null ? rigidBody : GetComponent<Rigidbody>();
        referenceTransform = FinsROVAgentRuntime.ResolveReferenceTransform(transform, selfTransform);
        if (thrusterController == null)
        {
            thrusterController = thrusterControllerOverride != null
                ? thrusterControllerOverride
                : GetComponent<ThrusterController>();
        }
        if (FinsROVAgentRuntime.TryResolveOrderedThrusters(
            this, thrusterController, autoResolveThrustersFromChildren, orderedThrusters, out string statusMessage))
        {
            FinsROVAgentRuntime.EnsureThrusterControllerOrder(thrusterController, orderedThrusters);
            if (logThrusterResolution)
            {
                Debug.Log($"[{nameof(GoalYawIrlAgent)}] {statusMessage}", this);
                logThrusterResolution = false;
            }
        }
        else
        {
            Debug.LogError($"[{nameof(GoalYawIrlAgent)}] {statusMessage}", this);
        }
    }

    float DistanceToTarget()
    {
        return targetTransform == null ? 0f : Vector3.Distance(ReferenceTransform.position, targetTransform.position);
    }

    float YawErrorDeg()
    {
        if (targetTransform == null)
        {
            return 0f;
        }
        Vector3 referenceForward = Vector3.ProjectOnPlane(ReferenceTransform.forward, Vector3.up);
        Vector3 targetForward = Vector3.ProjectOnPlane(targetTransform.forward, Vector3.up);
        if (referenceForward.sqrMagnitude < 1e-6f || targetForward.sqrMagnitude < 1e-6f)
        {
            return 180f;
        }
        return Vector3.Angle(referenceForward, targetForward);
    }

    Vector3 LocalLinearVelocity() => FinsROVAgentRuntime.GetLocalLinearVelocity(ReferenceTransform, rigidBody);

    Vector3 LocalAngularVelocity() => FinsROVAgentRuntime.GetLocalAngularVelocity(ReferenceTransform, rigidBody);

    void UpdateSpeedMetrics()
    {
        CurrentLinearSpeedMps = LocalLinearVelocity().magnitude;
        CurrentAngularSpeedDegPerSec = LocalAngularVelocity().magnitude * Mathf.Rad2Deg;
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

    Vector3 GenerateGoalPosition(Vector3 center)
    {
        float minRadius = Mathf.Clamp(minimumTargetDistance, 0f, targetSpawnRadius);
        Vector3 direction = Random.insideUnitSphere;
        if (direction.sqrMagnitude < 1e-6f)
        {
            direction = Vector3.forward;
        }
        Vector3 offset = direction.normalized * Random.Range(minRadius, targetSpawnRadius);
        return new Vector3(center.x + offset.x, center.y + Mathf.Min(offset.y, surfaceY - center.y), center.z + offset.z);
    }

    bool UseStratifiedResets() =>
        Academy.Instance.EnvironmentParameters.GetWithDefault(StratifiedResetParameter, 0f) >= 0.5f;

    void GenerateStratifiedGoal(Vector3 center, int stratumIndex, out Vector3 position, out float targetYawDeg)
    {
        int distanceIndex = stratumIndex % DistanceStrata;
        int yawIndex = (stratumIndex / DistanceStrata) % YawStrata;
        int verticalIndex = (stratumIndex / (DistanceStrata * YawStrata)) % VerticalStrata;

        // With the current 0.5--3 m task range, these fractions yield the
        // explicit 0.5--1, 1--2, and 2--3 m strata.  Expressing them as
        // fractions keeps the partition valid if that range is retuned.
        float[] distanceFractions = { 0f, 0.20f, 0.60f, 1f };
        float distanceMin = Mathf.Lerp(minimumTargetDistance, targetSpawnRadius, distanceFractions[distanceIndex]);
        float distanceMax = Mathf.Lerp(minimumTargetDistance, targetSpawnRadius, distanceFractions[distanceIndex + 1]);
        float distance = Random.Range(distanceMin, distanceMax);

        // Level and non-level targets are balanced independently.  The
        // resulting 3D goal distance remains in the selected distance bin.
        float verticalMagnitude = verticalIndex == 0
            ? Random.Range(0f, Mathf.Min(0.30f, 0.5f * distance))
            : Random.Range(Mathf.Min(0.40f, 0.8f * distance), 0.8f * distance);
        float verticalSign = Random.value < 0.5f ? -1f : 1f;
        float verticalOffset = verticalSign * verticalMagnitude;
        if (center.y + verticalOffset > surfaceY)
        {
            verticalOffset = -verticalMagnitude;
        }
        float horizontalMagnitude = Mathf.Sqrt(Mathf.Max(0f, distance * distance - verticalOffset * verticalOffset));
        float heading = Random.Range(0f, 360f) * Mathf.Deg2Rad;
        position = center + new Vector3(
            Mathf.Cos(heading) * horizontalMagnitude,
            verticalOffset,
            Mathf.Sin(heading) * horizontalMagnitude);

        float yawBinWidth = 180f / YawStrata;
        float yawErrorMagnitude = Random.Range(yawIndex * yawBinWidth, (yawIndex + 1) * yawBinWidth);
        float yawSign = Random.value < 0.5f ? -1f : 1f;
        targetYawDeg = ReferenceTransform.eulerAngles.y + yawSign * yawErrorMagnitude;
    }

    void RecordStepStats()
    {
        if (!enableStatsRecorder)
        {
            return;
        }
        Academy.Instance.StatsRecorder.Add("FinsROV/goal_yaw_position_error_m", CurrentDistanceM);
        Academy.Instance.StatsRecorder.Add("FinsROV/goal_yaw_yaw_error_deg", CurrentYawErrorDeg);
        Academy.Instance.StatsRecorder.Add("FinsROV/goal_yaw_linear_speed_mps", CurrentLinearSpeedMps);
        Academy.Instance.StatsRecorder.Add("FinsROV/goal_yaw_angular_speed_degps", CurrentAngularSpeedDegPerSec);
        Academy.Instance.StatsRecorder.Add("FinsROV/goal_yaw_stable_success_duration_sec", stableSuccessDurationSeconds);
        if (episodeStratumIndex >= 0)
        {
            Academy.Instance.StatsRecorder.Add("FinsROV/goal_yaw_reset_stratum", episodeStratumIndex);
        }
    }

    void RecordTerminalStats(bool success, string terminalReason)
    {
        if (!enableStatsRecorder)
        {
            return;
        }
        Academy.Instance.StatsRecorder.Add("FinsROV/goal_yaw_episode_success", success ? 1f : 0f);
        Academy.Instance.StatsRecorder.Add("FinsROV/goal_yaw_terminal_position_error_m", CurrentDistanceM);
        Academy.Instance.StatsRecorder.Add("FinsROV/goal_yaw_terminal_yaw_error_deg", CurrentYawErrorDeg);
        Academy.Instance.StatsRecorder.Add("FinsROV/goal_yaw_terminal_linear_speed_mps", CurrentLinearSpeedMps);
        Academy.Instance.StatsRecorder.Add("FinsROV/goal_yaw_terminal_angular_speed_degps", CurrentAngularSpeedDegPerSec);
        Academy.Instance.StatsRecorder.Add("FinsROV/goal_yaw_terminal_reason_code", terminalReason == "success" ? 1f : terminalReason == "out_of_bounds" ? 2f : 3f);
    }
}
