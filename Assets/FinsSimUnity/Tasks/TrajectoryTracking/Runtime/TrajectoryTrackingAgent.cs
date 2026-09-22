using System;
using FinsSim.Actuators;
using FinsSim.Hydrodynamics;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

[DisallowMultipleComponent]
[RequireComponent(typeof(Rigidbody))]
public sealed class TrajectoryTrackingAgent : Agent
{
    public const int VectorObservationSize = 30;
    public const int ThrusterActionSize = 8;
    const string CurriculumStageEnvironmentParameter = "finsim_trajectory_curriculum_stage";
    const string CurriculumStageCommandLineArgument = "-fins-trajectory-curriculum-stage";

    [Header("References")]
    public Transform selfTransform;
    public Transform trajectoryReference;
    [SerializeField] ThrusterController thrusterControllerOverride;
    [SerializeField] ThrusterCommandMode directThrusterCommandMode = ThrusterCommandMode.ScaledForceRequest;
    // The calibrated FinsROV thrusters are limited to +/-7 N. Keep the
    // policy's normalized endpoint aligned with that physical limit.
    [SerializeField] float directActionForceScaleN = 7f;
    [SerializeField] bool autoResolveThrustersFromChildren = true;

    [Header("Trajectory")]
    [SerializeField] TrajectoryTrackingMath.Profile profile = TrajectoryTrackingMath.Profile.Curriculum;
    [SerializeField, Range(0, 2)] int curriculumStage;
    [SerializeField] float previewDecisionStride = 5f;
    [SerializeField] float initialDepth = -0.5f;
    [SerializeField] float initialPositionJitter = 0.15f;
    [SerializeField] float initialRollPitchRangeDeg = 20f;
    [Tooltip("World/controller yaw range in degrees. Keep at zero for an asymmetric physical pool whose long side is +X.")]
    [SerializeField] Vector2 trajectoryYawRangeDeg = Vector2.zero;
    [SerializeField] Vector2 trajectoryScaleXRange = new Vector2(0.5f, 2.5f);
    [SerializeField] Vector2 trajectoryScaleYRange = new Vector2(0.04f, 0.12f);
    [SerializeField] Vector2 trajectoryScaleZRange = new Vector2(0.25f, 1.25f);
    [Tooltip("Requested phase cycles over one episode. A trajectory may complete fewer cycles when the physical speed limits require it.")]
    [SerializeField, Min(0.01f)] float requestedTrajectoryCycles = 1f;
    [Header("Reference Speed Limits (m/s)")]
    [Tooltip("60% of the measured 0.4998 m/s full-wrench surge steady-state limit. This reserve is needed for tracking, turns, response lag, and DR disturbances.")]
    [SerializeField, Min(0.01f)] float maxReferenceSurgeSpeedMps = 0.30f;
    [Tooltip("57% of the measured 0.3520 m/s full-wrench sway steady-state limit. Keep a tracking reserve instead of commanding the steady-state boundary.")]
    [SerializeField, Min(0.01f)] float maxReferenceSwaySpeedMps = 0.20f;
    [Tooltip("Conservative vertical reference limit. The measured 0.2196 m/s downward result has no reserve, and the upward full-wrench test reached the surface.")]
    [SerializeField, Min(0.01f)] float maxReferenceHeaveSpeedMps = 0.10f;
    [SerializeField] Vector2 lemniscateCRange = new Vector2(-0.6f, 0.6f);

    [Header("Observation")]
    [SerializeField] float previewOffsetScale = 3f;
    [SerializeField] float linearVelocityScale = 1f;
    [SerializeField] float angularVelocityScale = 1f;
    [SerializeField] float observationClip = 2f;

    [Header("Reward")]
    [SerializeField] float poseDistanceScale = 1.6f;
    [SerializeField] float effortScale = 0.1f;
    [SerializeField] float smoothnessScale;

    [Header("Termination")]
    [SerializeField] float maxPathError = 4f;
    [SerializeField] float maxHeightAboveWater = 0.3f;
    [SerializeField] float maxDistanceFromArea = 12f;
    [SerializeField] float outOfBoundsPenalty = -1f;

    [Header("Runtime Diagnostics")]
    [SerializeField] float trackingError;
    [SerializeField] float trajectoryProgress;
    [SerializeField] Vector3 currentReferencePosition;
    [SerializeField] Vector3 currentReferenceVelocityBody;
    [SerializeField] Vector3 currentLinearVelocityBody;
    [SerializeField] Vector3 currentAngularVelocityBody;
    [SerializeField] float poseReward;
    [SerializeField] float upReward;
    [SerializeField] float spinReward;
    [SerializeField] float effortReward;
    [SerializeField] float smoothReward;
    [SerializeField] float lastReward;
    [SerializeField] float selectedAngularSpeedRadPerSec;
    [SerializeField] float selectedTrajectoryCycles;
    [SerializeField] int activeCurriculumStage;
    [SerializeField] string activeCurriculumStageSource = "scene";
    [SerializeField] float[] lastPolicyAction = new float[ThrusterActionSize];
    [SerializeField] float[] lastThrusterAction = new float[ThrusterActionSize];

    Rigidbody rigidBody;
    HydrodynamicsController hydrodynamics;
    ThrusterController thrusterController;
    Transform referenceTransform;
    Transform parallelAreaTransform;
    readonly Thruster[] orderedThrusters = new Thruster[ThrusterActionSize];
    readonly float[] previousPolicyAction = new float[ThrusterActionSize];
    TrajectoryTrackingMath.Parameters trajectory;
    System.Random episodeRandom;
    int episodeIndex;
    bool includeProcessSeedInEpisodeRandom;
    int sceneCurriculumStage;
    int? commandLineCurriculumStage;
    bool loggedCurriculumStage;

    public int ExpectedActionSize => ThrusterActionSize;
    public int PolicyActionCount => ThrusterActionSize;
    public Thruster[] OrderedThrusters => orderedThrusters;
    public float EpisodeDurationSec => Mathf.Max(MaxStep, 1) * Time.fixedDeltaTime;

    Transform ReferenceTransform => referenceTransform == null
        ? referenceTransform = FinsROVAgentRuntime.ResolveReferenceTransform(transform, selfTransform)
        : referenceTransform;

    protected override void Awake()
    {
        // Agent.Awake performs the ML-Agents registration and initializes the
        // internal sensor/action machinery. Hiding it leaves a Player able to
        // establish a gRPC communicator but unable to emit its first decision.
        base.Awake();
        sceneCurriculumStage = Mathf.Clamp(curriculumStage, 0, 2);
        activeCurriculumStage = sceneCurriculumStage;
        commandLineCurriculumStage = ReadCommandLineCurriculumStage();
    }

    void Start()
    {
        rigidBody = GetComponent<Rigidbody>();
        hydrodynamics = GetComponent<HydrodynamicsController>();
        ResolveThrusters();
    }

    public void ConfigureForParallelTrainingArea(Transform area, Transform reference)
    {
        parallelAreaTransform = area;
        trajectoryReference = reference;
    }

    /// <summary>
    /// Enables a deterministic per-Unity-Player episode offset. The DWP2 T2
    /// fixture uses one vehicle per Player rather than replicated areas.
    /// </summary>
    public void ConfigureProcessSeedOffset(bool enabled)
    {
        includeProcessSeedInEpisodeRandom = enabled;
    }

    public void ConfigureEpisodeMaxStep(int maxPhysicsSteps)
    {
        MaxStep = Mathf.Max(1, maxPhysicsSteps);
    }

    public override void OnEpisodeBegin()
    {
        // Evaluation episodes can end before the regular aggregation window.
        // Publish the preceding window before clearing this area's state.
        TrajectoryTrackingMetrics.FlushPending();
        ResolveThrusters();
        ResolveCurriculumStage();
        FinsROVAgentRuntime.RandomizeEpisodeIfPresent(this);
        episodeRandom = CreateEpisodeRandom();
        trajectory = CreateTrajectory(episodeRandom);
        TrajectoryTrackingMath.Evaluate(trajectory, 0f, out Vector3 startPosition, out _);
        Vector3 jitter = new Vector3(RandomRange(-initialPositionJitter, initialPositionJitter), RandomRange(-initialPositionJitter, initialPositionJitter), RandomRange(-initialPositionJitter, initialPositionJitter));

        FinsROVAgentRuntime.ZeroThrusters(orderedThrusters, directThrusterCommandMode, directActionForceScaleN);
        rigidBody.linearVelocity = Vector3.zero;
        rigidBody.angularVelocity = Vector3.zero;
        rigidBody.position = startPosition + jitter;
        rigidBody.rotation = Quaternion.Euler(
            RandomRange(-initialRollPitchRangeDeg, initialRollPitchRangeDeg),
            RandomRange(0f, 360f),
            RandomRange(-initialRollPitchRangeDeg, initialRollPitchRangeDeg));
        Physics.SyncTransforms();
        hydrodynamics?.ResetBackendState();
        rigidBody.WakeUp();

        Array.Clear(previousPolicyAction, 0, previousPolicyAction.Length);
        Array.Clear(lastPolicyAction, 0, lastPolicyAction.Length);
        Array.Clear(lastThrusterAction, 0, lastThrusterAction.Length);
        episodeIndex++;
        UpdateReferenceVisual();
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        float now = StepCount * Time.fixedDeltaTime;
        Transform reference = ReferenceTransform;
        for (int index = 0; index < 4; index++)
        {
            float previewTime = now + index * previewDecisionStride * Time.fixedDeltaTime;
            TrajectoryTrackingMath.Evaluate(trajectory, previewTime, out Vector3 point, out _);
            sensor.AddObservation(Clip(reference.InverseTransformPoint(point) / Mathf.Max(previewOffsetScale, 1e-4f)));
        }
        TrajectoryTrackingMath.Evaluate(trajectory, now, out _, out Vector3 referenceVelocityWorld);
        Vector3 referenceVelocityBody = reference.InverseTransformDirection(referenceVelocityWorld);
        currentReferenceVelocityBody = referenceVelocityBody;
        currentLinearVelocityBody = FinsROVAgentRuntime.GetLocalLinearVelocity(reference, rigidBody);
        currentAngularVelocityBody = FinsROVAgentRuntime.GetLocalAngularVelocity(reference, rigidBody);
        sensor.AddObservation(Clip(referenceVelocityBody / Mathf.Max(linearVelocityScale, 1e-4f)));
        sensor.AddObservation(Clip(currentLinearVelocityBody / Mathf.Max(linearVelocityScale, 1e-4f)));
        sensor.AddObservation(Clip(currentAngularVelocityBody / Mathf.Max(angularVelocityScale, 1e-4f)));
        sensor.AddObservation(reference.InverseTransformDirection(Vector3.up));
        float tangentYaw = Mathf.Atan2(referenceVelocityBody.z, referenceVelocityBody.x);
        sensor.AddObservation(Mathf.Sin(tangentYaw));
        sensor.AddObservation(Mathf.Cos(tangentYaw));
        float progress = MaxStep > 0 ? Mathf.Clamp01((float)StepCount / MaxStep) : 0f;
        sensor.AddObservation(Mathf.Sin(2f * Mathf.PI * progress));
        sensor.AddObservation(Mathf.Cos(2f * Mathf.PI * progress));
        sensor.AddObservation(Mathf.Sin(4f * Mathf.PI * progress));
        sensor.AddObservation(Mathf.Cos(4f * Mathf.PI * progress));
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        ResolveThrusters();
        ReadAndApplyAction(actions.ContinuousActions);
        float now = StepCount * Time.fixedDeltaTime;
        TrajectoryTrackingMath.Evaluate(trajectory, now, out currentReferencePosition, out _);
        trackingError = Vector3.Distance(ReferenceTransform.position, currentReferencePosition);
        trajectoryProgress = MaxStep > 0 ? Mathf.Clamp01((float)StepCount / MaxStep) : 0f;
        currentLinearVelocityBody = FinsROVAgentRuntime.GetLocalLinearVelocity(ReferenceTransform, rigidBody);
        currentAngularVelocityBody = FinsROVAgentRuntime.GetLocalAngularVelocity(ReferenceTransform, rigidBody);

        poseReward = 0.5f * Mathf.Exp(-poseDistanceScale * trackingError);
        float tiltError = Mathf.Abs(1f - Vector3.Dot(ReferenceTransform.up, Vector3.up));
        upReward = 0.5f / (1f + tiltError * tiltError);
        float yawRate = currentAngularVelocityBody.y;
        float yawRateFourth = yawRate * yawRate * yawRate * yawRate;
        spinReward = 0.5f / (1f + yawRateFourth);
        effortReward = effortScale * Mathf.Exp(-MeanAbs(lastPolicyAction, PolicyActionCount));
        smoothReward = smoothnessScale * Mathf.Exp(-MeanSquaredDelta(lastPolicyAction, previousPolicyAction, PolicyActionCount));
        lastReward = poseReward + poseReward * (upReward + spinReward) + effortReward + smoothReward;
        AddReward(lastReward);
        TrajectoryTrackingMetrics.Record(
            this,
            trackingError,
            currentAngularVelocityBody,
            lastPolicyAction,
            previousPolicyAction,
            lastThrusterAction);
        Array.Copy(lastPolicyAction, previousPolicyAction, previousPolicyAction.Length);
        UpdateReferenceVisual();

        bool invalid = !IsFinite(rigidBody.position) || !IsFinite(rigidBody.linearVelocity) || !IsFinite(rigidBody.angularVelocity);
        Vector3 areaOrigin = parallelAreaTransform != null ? parallelAreaTransform.position : Vector3.zero;
        if (invalid || trackingError > maxPathError || rigidBody.position.y > areaOrigin.y + maxHeightAboveWater || Vector3.Distance(rigidBody.position, areaOrigin) > maxDistanceFromArea)
        {
            AddReward(outOfBoundsPenalty);
            // EndEpisode schedules the terminal transition for the next
            // communicator exchange. Flush before ending so short eval
            // episodes publish their partial Unity stats window as part of
            // that terminal transition.
            TrajectoryTrackingMetrics.FlushPending();
            EndEpisode();
        }
    }

    void ReadAndApplyAction(ActionSegment<float> continuousActions)
    {
        Array.Clear(lastPolicyAction, 0, lastPolicyAction.Length);
        Array.Clear(lastThrusterAction, 0, lastThrusterAction.Length);
        for (int i = 0; i < ThrusterActionSize; i++)
        {
            float action = i < continuousActions.Length ? Mathf.Clamp(continuousActions[i], -1f, 1f) : 0f;
            lastPolicyAction[i] = action;
            lastThrusterAction[i] = action;
        }
        FinsROVAgentRuntime.ApplyThrusterActions(orderedThrusters, lastThrusterAction, directThrusterCommandMode, directActionForceScaleN);
    }

    void ResolveThrusters()
    {
        thrusterController = thrusterControllerOverride != null ? thrusterControllerOverride : GetComponent<ThrusterController>();
        FinsROVAgentRuntime.TryResolveOrderedThrusters(this, thrusterController, autoResolveThrustersFromChildren, orderedThrusters, out _);
        FinsROVAgentRuntime.EnsureThrusterControllerOrder(thrusterController, orderedThrusters);
    }

    TrajectoryTrackingMath.Parameters CreateTrajectory(System.Random random)
    {
        var activeProfile = profile == TrajectoryTrackingMath.Profile.Curriculum
            ? TrajectoryTrackingMath.ResolveCurriculumProfile(activeCurriculumStage, random)
            : profile;
        var parameters = new TrajectoryTrackingMath.Parameters
        {
            profile = activeProfile,
            origin = (parallelAreaTransform != null ? parallelAreaTransform.position : Vector3.zero) + new Vector3(0f, initialDepth, 0f),
            yawDeg = RandomRange(trajectoryYawRangeDeg.x, trajectoryYawRangeDeg.y),
            scale = new Vector3(RandomRange(trajectoryScaleXRange.x, trajectoryScaleXRange.y), RandomRange(trajectoryScaleYRange.x, trajectoryScaleYRange.y), RandomRange(trajectoryScaleZRange.x, trajectoryScaleZRange.y)),
            phase = RandomRange(0f, 2f * Mathf.PI),
            lemniscateC = RandomRange(lemniscateCRange.x, lemniscateCRange.y),
            durationSec = EpisodeDurationSec,
        };
        if (parameters.profile == TrajectoryTrackingMath.Profile.StraightLine)
        {
            // A finite line spans 2 * scale.x in one episode.  Trim only its
            // traversed segment when a requested large line exceeds the
            // measured, derated surge capability; do not create an infeasible
            // reference just to reach a geometric endpoint.
            float maxHalfSpan = 0.5f * maxReferenceSurgeSpeedMps * parameters.durationSec;
            parameters.scale.x = Mathf.Min(parameters.scale.x, Mathf.Max(maxHalfSpan, 1e-4f));
        }
        parameters.angularSpeed = ResolveAngularSpeed(parameters, random.Next(0, 2) == 0 ? -1f : 1f);
        selectedAngularSpeedRadPerSec = parameters.angularSpeed;
        selectedTrajectoryCycles = Mathf.Abs(parameters.angularSpeed) * parameters.durationSec / (2f * Mathf.PI);
        return parameters;
    }

    float ResolveAngularSpeed(in TrajectoryTrackingMath.Parameters parameters, float direction)
    {
        if (parameters.profile == TrajectoryTrackingMath.Profile.StraightLine)
        {
            return 0f;
        }

        float duration = Mathf.Max(parameters.durationSec, 1e-4f);
        float requestedSpeed = 2f * Mathf.PI * requestedTrajectoryCycles / duration;
        float feasibleSpeed = EstimateFeasibleAngularSpeed(parameters);
        return direction * Mathf.Min(requestedSpeed, feasibleSpeed);
    }

    float EstimateFeasibleAngularSpeed(in TrajectoryTrackingMath.Parameters parameters)
    {
        const int phaseSamples = 96;
        float feasibleSpeed = float.PositiveInfinity;
        var unitSpeedParameters = parameters;
        unitSpeedParameters.angularSpeed = 1f;
        for (int index = 0; index < phaseSamples; index++)
        {
            unitSpeedParameters.phase = 2f * Mathf.PI * index / phaseSamples;
            TrajectoryTrackingMath.Evaluate(unitSpeedParameters, 0f, out _, out Vector3 unitVelocity);
            LimitAngularSpeed(ref feasibleSpeed, maxReferenceSurgeSpeedMps, Mathf.Abs(unitVelocity.x));
            LimitAngularSpeed(ref feasibleSpeed, maxReferenceHeaveSpeedMps, Mathf.Abs(unitVelocity.y));
            LimitAngularSpeed(ref feasibleSpeed, maxReferenceSwaySpeedMps, Mathf.Abs(unitVelocity.z));
        }
        return float.IsFinite(feasibleSpeed) ? Mathf.Max(feasibleSpeed, 1e-4f) : 0f;
    }

    static void LimitAngularSpeed(ref float feasibleSpeed, float componentSpeedLimit, float unitVelocityComponent)
    {
        if (unitVelocityComponent > 1e-5f)
        {
            feasibleSpeed = Mathf.Min(feasibleSpeed, Mathf.Max(componentSpeedLimit, 1e-4f) / unitVelocityComponent);
        }
    }

    void ResolveCurriculumStage()
    {
        activeCurriculumStage = sceneCurriculumStage;
        activeCurriculumStageSource = "scene";

        try
        {
            float environmentValue = Academy.Instance.EnvironmentParameters.GetWithDefault(CurriculumStageEnvironmentParameter, -1f);
            if (environmentValue >= 0f)
            {
                activeCurriculumStage = Mathf.Clamp(Mathf.RoundToInt(environmentValue), 0, 2);
                activeCurriculumStageSource = "mlagents_parameter";
            }
        }
        catch
        {
            // The scene value remains usable for editor/manual play without an Academy connection.
        }

        if (commandLineCurriculumStage.HasValue)
        {
            activeCurriculumStage = commandLineCurriculumStage.Value;
            activeCurriculumStageSource = "command_line";
        }

        if (!loggedCurriculumStage)
        {
            Debug.Log($"[TrajectoryTrackingAgent] curriculum stage={activeCurriculumStage} source={activeCurriculumStageSource} " +
                $"(scene={sceneCurriculumStage}, arg={CurriculumStageCommandLineArgument}).");
            loggedCurriculumStage = true;
        }
    }

    static int? ReadCommandLineCurriculumStage()
    {
        string[] arguments = Environment.GetCommandLineArgs();
        for (int index = 0; index < arguments.Length; index++)
        {
            string argument = arguments[index];
            string value = null;
            if (string.Equals(argument, CurriculumStageCommandLineArgument, StringComparison.OrdinalIgnoreCase) && index + 1 < arguments.Length)
            {
                value = arguments[index + 1];
            }
            else if (argument.StartsWith(CurriculumStageCommandLineArgument + "=", StringComparison.OrdinalIgnoreCase))
            {
                value = argument.Substring(CurriculumStageCommandLineArgument.Length + 1);
            }

            if (value == null)
            {
                continue;
            }
            if (int.TryParse(value, out int stage))
            {
                return Mathf.Clamp(stage, 0, 2);
            }

            Debug.LogWarning($"[TrajectoryTrackingAgent] Ignoring invalid {CurriculumStageCommandLineArgument} value '{value}'; expected 0, 1, or 2.");
            return null;
        }
        return null;
    }

    System.Random CreateEpisodeRandom()
    {
        Vector3 origin = parallelAreaTransform != null ? parallelAreaTransform.position : transform.position;
        int seed = 19001 + episodeIndex * 997 + Mathf.RoundToInt(origin.x) * 73856093 ^ Mathf.RoundToInt(origin.y) * 19349663 ^ Mathf.RoundToInt(origin.z) * 83492791;
        if (includeProcessSeedInEpisodeRandom)
        {
            seed ^= ReadProcessSeed();
        }
        return new System.Random(seed);
    }

    static int ReadProcessSeed()
    {
        string[] args = Environment.GetCommandLineArgs();
        for (int index = 0; index < args.Length; index++)
        {
            string arg = args[index];
            if (arg.Equals("-fins-dr-seed", StringComparison.OrdinalIgnoreCase) &&
                index + 1 < args.Length && int.TryParse(args[index + 1], out int separateValue))
            {
                return separateValue;
            }
            const string prefix = "-fins-dr-seed=";
            if (arg.StartsWith(prefix, StringComparison.OrdinalIgnoreCase) &&
                int.TryParse(arg.Substring(prefix.Length), out int assignedValue))
            {
                return assignedValue;
            }
        }
        return 0;
    }

    float RandomRange(float min, float max)
    {
        return Mathf.Lerp(min, max, (float)(episodeRandom ??= new System.Random(19001)).NextDouble());
    }

    void UpdateReferenceVisual()
    {
        if (trajectoryReference == null) return;
        TrajectoryTrackingMath.Evaluate(trajectory, StepCount * Time.fixedDeltaTime, out Vector3 position, out _);
        trajectoryReference.position = position;
    }

    public void EvaluateReference(float timeSec, out Vector3 position, out Vector3 velocity)
    {
        TrajectoryTrackingMath.Evaluate(trajectory, timeSec, out position, out velocity);
    }

    Vector3 Clip(Vector3 value) => new Vector3(Mathf.Clamp(value.x, -observationClip, observationClip), Mathf.Clamp(value.y, -observationClip, observationClip), Mathf.Clamp(value.z, -observationClip, observationClip));
    static float MeanAbs(float[] values, int count) { float sum = 0f; for (int i = 0; i < count; i++) sum += Mathf.Abs(values[i]); return sum / Mathf.Max(count, 1); }
    static float MeanSquaredDelta(float[] current, float[] previous, int count) { float sum = 0f; for (int i = 0; i < count; i++) { float d = current[i] - previous[i]; sum += d * d; } return sum / Mathf.Max(count, 1); }
    static bool IsFinite(Vector3 value) => float.IsFinite(value.x) && float.IsFinite(value.y) && float.IsFinite(value.z);
}
