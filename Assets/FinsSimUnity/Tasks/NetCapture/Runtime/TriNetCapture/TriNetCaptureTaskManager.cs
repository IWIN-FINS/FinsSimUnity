using System;
using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using UnityEngine;

/// <summary>
/// Three-ROV triangular-net transport task. This owns reward and lifecycle only
/// when attached to a TriNetCapture training area; legacy 3Chase1 stays unchanged.
/// </summary>
[DisallowMultipleComponent]
public sealed class TriNetCaptureTaskManager : MonoBehaviour
{
    [Header("Scene References")]
    [SerializeField] private Transform poolFrame;
    [SerializeField] private TriNetCaptureAgent left;
    [SerializeField] private TriNetCaptureAgent top;
    [SerializeField] private TriNetCaptureAgent right;
    [SerializeField] private TriNetTarget target;
    [SerializeField] private TriNetRecoveryGoal recoveryGoal;
    [SerializeField] private HabradorTriangleTowDriver xpbdNet;

    [Header("Pool And Spawn")]
    [SerializeField] private Vector3 poolMinLocal = new Vector3(-2f, -1f, -1f);
    [SerializeField] private Vector3 poolMaxLocal = new Vector3(2f, 0f, 1f);
    [SerializeField] private float wallClearance = 0.18f;
    [SerializeField] private float rovClearance = 0.45f;
    [SerializeField] private float goalClearance = 0.35f;
    [SerializeField] private int spawnAttempts = 80;
    [Tooltip("Enable per-episode random sampling for Target's initial pool position.")]
    [SerializeField] private bool randomizeTargetInitialPosition = true;
    [Tooltip("Pool-local Target position used when initial-position randomization is disabled.")]
    [SerializeField] private Vector3 fixedTargetInitialLocalPosition = new Vector3(1.20f, -0.40f, 0f);

    [Header("Net Hold")]
    [Tooltip("Maximum collider-to-XPBD-cloth surface separation that still counts as captured.")]
    [SerializeField] private float targetNetDistance = 0.10f;
    [SerializeField] private float successHoldSeconds = 1.0f;
    [SerializeField] private float desiredSideLength = 0.60f;
    [SerializeField] private float invalidTriangleArea = 0.002f;
    [SerializeField] private float invalidNetGraceSeconds = 1.0f;

    [Header("Reward")]
    [SerializeField] private float timePenalty = -0.0005f;
    [SerializeField] private float netTargetProgressScale = 0.20f;
    [SerializeField] private float targetGoalProgressScale = 0.45f;
    [SerializeField] private float shapeProgressScale = 0.10f;
    [SerializeField] private float holdProgressScale = 0.03f;
    [SerializeField] private float successReward = 5.0f;
    [SerializeField] private float failurePenalty = -2.0f;
    [SerializeField] private float collisionPenalty = -0.02f;

    private readonly HashSet<TriNetCaptureAgent> begunAgents = new HashSet<TriNetCaptureAgent>();
    private Vector3[] initialLocalPositions;
    private Quaternion[] initialLocalRotations;
    private float previousNetTargetDistance;
    private float previousTargetGoalDistance;
    private float previousShapeError;
    private float holdSeconds;
    private float initialTargetGoalDistance;
    private float minimumTargetGoalDistance;
    private float invalidNetSeconds;
    private bool terminal;
    private string terminalReason = "running";

    public TriNetTarget Target => target;
    public TriNetRecoveryGoal RecoveryGoal => recoveryGoal;
    public TriNetCaptureAgent Left => left;
    public TriNetCaptureAgent Top => top;
    public TriNetCaptureAgent Right => right;
    public string TerminalReason => terminalReason;
    public float HoldSeconds => holdSeconds;
    public float DesiredSideLength => desiredSideLength;

    private TriNetCaptureAgent[] Chasers => new[] { left, top, right };

    /// <summary>Returns the two other homogeneous ROVs in a stable scene order.</summary>
    public TriNetCaptureAgent[] GetOtherAgents(TriNetCaptureAgent self)
    {
        List<TriNetCaptureAgent> result = new List<TriNetCaptureAgent>(2);
        foreach (TriNetCaptureAgent chaser in Chasers)
        {
            if (chaser != null && chaser != self)
            {
                result.Add(chaser);
            }
        }
        return result.ToArray();
    }

    public void Configure(
        Transform configuredPoolFrame,
        TriNetCaptureAgent configuredLeft,
        TriNetCaptureAgent configuredTop,
        TriNetCaptureAgent configuredRight,
        TriNetTarget configuredTarget,
        TriNetRecoveryGoal configuredGoal,
        HabradorTriangleTowDriver configuredXpbdNet)
    {
        poolFrame = configuredPoolFrame;
        left = configuredLeft;
        top = configuredTop;
        right = configuredRight;
        target = configuredTarget;
        recoveryGoal = configuredGoal;
        xpbdNet = configuredXpbdNet;
        if (xpbdNet != null && xpbdNet.TryGetTriangleVertices(out Vector3 a, out Vector3 b, out Vector3 c))
        {
            desiredSideLength = (Vector3.Distance(a, b) + Vector3.Distance(b, c) + Vector3.Distance(c, a)) / 3f;
        }
        CaptureInitialChaserStates();
    }

    private void Awake()
    {
        ResolveReferences();
        CaptureInitialChaserStates();
    }

    private void OnValidate()
    {
        wallClearance = Mathf.Max(0.01f, wallClearance);
        rovClearance = Mathf.Max(0.01f, rovClearance);
        goalClearance = Mathf.Max(0.01f, goalClearance);
        targetNetDistance = Mathf.Max(0f, targetNetDistance);
        successHoldSeconds = Mathf.Max(0f, successHoldSeconds);
        desiredSideLength = Mathf.Max(0.01f, desiredSideLength);
    }

    private void FixedUpdate()
    {
        if (terminal || target == null || xpbdNet == null)
        {
            return;
        }

        if (!TryGetTaskState(out TaskState state))
        {
            // Give the XPBD cloth a short reset grace period before treating
            // a genuinely collapsed or missing net as terminal.
            invalidNetSeconds += Time.fixedDeltaTime;
            if (invalidNetSeconds >= invalidNetGraceSeconds)
            {
                Finish(false, "invalid_net");
            }
            return;
        }
        invalidNetSeconds = 0f;

        float netProgress = Mathf.Clamp(previousNetTargetDistance - state.netTargetDistance, -0.10f, 0.10f);
        float shapeProgress = Mathf.Clamp(previousShapeError - state.shapeError, -0.10f, 0.10f);
        float goalProgress = state.netHeld
            ? Mathf.Clamp(previousTargetGoalDistance - state.targetGoalDistance, -0.10f, 0.10f)
            : 0f;
        float holdProgress = state.goalContains && state.netHeld ? Time.fixedDeltaTime : 0f;
        if (state.goalContains && state.netHeld)
        {
            holdSeconds += Time.fixedDeltaTime;
        }
        else
        {
            holdSeconds = 0f;
        }

        float reward = timePenalty + netTargetProgressScale * netProgress +
            targetGoalProgressScale * goalProgress + shapeProgressScale * shapeProgress +
            holdProgressScale * holdProgress;
        AddSharedReward(reward);

        previousNetTargetDistance = state.netTargetDistance;
        previousTargetGoalDistance = state.targetGoalDistance;
        previousShapeError = state.shapeError;
        minimumTargetGoalDistance = Mathf.Min(minimumTargetGoalDistance, state.targetGoalDistance);
        RecordStepStats(state);

        if (!IsInsideSafePool(target.transform.position))
        {
            Finish(false, "target_out_of_bounds");
        }
        else if (holdSeconds >= successHoldSeconds)
        {
            Finish(true, "success");
        }
    }

    public void BeginEpisode(TriNetCaptureAgent agent)
    {
        if (agent == null)
        {
            return;
        }
        if (begunAgents.Count == 0)
        {
            ResetTask();
        }
        begunAgents.Add(agent);
    }

    public void NotifyActionApplied(TriNetCaptureAgent agent)
    {
        // Kept as a lifecycle hook so action-level diagnostics can stay task-local.
    }

    public void NotifyCollision(TriNetCaptureAgent agent, Collider other)
    {
        if (terminal || agent == null || other == null || IsTaskInteraction(other.transform))
        {
            return;
        }

        agent.AddReward(collisionPenalty);
    }

    public void CollectTaskObservations(TriNetCaptureAgent agent, VectorSensor sensor)
    {
        Vector3 goalRelativeBody = Vector3.zero;
        if (agent != null && recoveryGoal != null)
        {
            Transform pose = agent.PoseTransform;
            goalRelativeBody = ControllerBodyFrame.WorldToBodyPositionDelta(
                pose, recoveryGoal.GuidancePoint - pose.position);
        }
        sensor.AddObservation(goalRelativeBody);

        NetObservationSummary netSummary = GetNetObservationSummary();
        sensor.AddObservation(netSummary.sortedEdgeErrors);
        sensor.AddObservation(netSummary.signedSurfaceDistance);
        sensor.AddObservation(netSummary.minimumEdgeMargin);

        AddPrivilegedCriticState(sensor, netSummary);
    }

    /// <summary>Pool-local vertical coordinate for the scripted lift stage.</summary>
    public float GetPoolLocalY(Vector3 worldPosition) => WorldToLocal(worldPosition).y;

    /// <summary>
    /// Signed controller-frame yaw error that returns this ROV to pool yaw 0.
    /// A positive value means the pool +X axis lies on the controller's +Z
    /// side, matching ControllerBodyFrame.HorizontalBearingRad's convention.
    /// </summary>
    public float GetYawErrorToHoldPoolYawZeroDegrees(Transform pose)
    {
        if (pose == null)
        {
            return 0f;
        }

        Vector3 poolPositiveX = poolFrame != null ? poolFrame.right : Vector3.right;
        Vector3 poolXAxisInBody = ControllerBodyFrame.WorldToBodyVector(pose, poolPositiveX);
        return ControllerBodyFrame.HorizontalBearingRad(poolXAxisInBody) * Mathf.Rad2Deg;
    }

    private bool IsTaskInteraction(Transform other)
    {
        if (other == null)
        {
            return false;
        }
        if (target != null && (other == target.transform || other.IsChildOf(target.transform)))
        {
            return true;
        }
        Transform netRoot = xpbdNet != null ? xpbdNet.transform : null;
        return netRoot != null && (other == netRoot || other.IsChildOf(netRoot));
    }

    private NetObservationSummary GetNetObservationSummary()
    {
        NetObservationSummary summary = default;
        if (target == null || xpbdNet == null || !xpbdNet.TryGetTriangleVertices(
                out Vector3 a, out Vector3 b, out Vector3 c))
        {
            return summary;
        }

        float[] edgeErrors =
        {
            (Vector3.Distance(a, b) - desiredSideLength) / desiredSideLength,
            (Vector3.Distance(b, c) - desiredSideLength) / desiredSideLength,
            (Vector3.Distance(c, a) - desiredSideLength) / desiredSideLength,
        };
        Array.Sort(edgeErrors);
        summary.sortedEdgeErrors = new Vector3(
            Mathf.Clamp(edgeErrors[0], -1f, 1f),
            Mathf.Clamp(edgeErrors[1], -1f, 1f),
            Mathf.Clamp(edgeErrors[2], -1f, 1f));

        Vector3 capturePoint = target.GetCapturePoint(xpbdNet.NetCenter);
        if (TryGetTriangularSurfaceMetrics(a, b, c,
                capturePoint,
                out _, out _, out float signedDistance, out _, out float edgeMargin))
        {
            summary.signedSurfaceDistance = Mathf.Clamp(signedDistance / desiredSideLength, -1f, 1f);
            summary.minimumEdgeMargin = Mathf.Clamp(edgeMargin / desiredSideLength, -1f, 1f);
        }
        return summary;
    }

    // Appended after the 34D deployable actor observation. The wrapper strips
    // this 57D state before actor inference and exposes it only to the critic.
    private void AddPrivilegedCriticState(VectorSensor sensor, NetObservationSummary netSummary)
    {
        Vector3 goalPosition = recoveryGoal != null ? recoveryGoal.GuidancePoint : Vector3.zero;
        foreach (TriNetCaptureAgent chaser in Chasers)
        {
            AddRovPrivilegedToken(sensor, chaser, goalPosition);
        }

        Rigidbody targetRigidbody = target != null ? target.GetComponent<Rigidbody>() : null;
        sensor.AddObservation(WorldToPoolVector((target != null ? target.transform.position : Vector3.zero) - goalPosition));
        sensor.AddObservation(WorldToPoolDirection(targetRigidbody != null ? targetRigidbody.linearVelocity : Vector3.zero));
        sensor.AddObservation(netSummary.sortedEdgeErrors);
        sensor.AddObservation(netSummary.signedSurfaceDistance);
        sensor.AddObservation(netSummary.minimumEdgeMargin);
        sensor.AddObservation(successHoldSeconds > 1e-5f ? Mathf.Clamp01(holdSeconds / successHoldSeconds) : 0f);
    }

    private void AddRovPrivilegedToken(VectorSensor sensor, TriNetCaptureAgent chaser, Vector3 goalPosition)
    {
        if (chaser == null)
        {
            sensor.AddObservation(Vector3.zero);
            sensor.AddObservation(Vector3.zero);
            sensor.AddObservation(Vector3.zero);
            sensor.AddObservation(Vector3.zero);
            sensor.AddObservation(Vector3.zero);
            return;
        }

        Rigidbody rigidbody = chaser.GetComponent<Rigidbody>();
        Transform pose = chaser.PoseTransform;
        sensor.AddObservation(WorldToPoolVector(pose.position - goalPosition));
        sensor.AddObservation(WorldToPoolDirection(rigidbody != null ? rigidbody.linearVelocity : Vector3.zero));
        sensor.AddObservation(WorldToPoolDirection(rigidbody != null ? rigidbody.angularVelocity : Vector3.zero));
        sensor.AddObservation(WorldToPoolDirection(pose.right));
        sensor.AddObservation(WorldToPoolDirection(pose.up));
    }

    private Vector3 WorldToPoolVector(Vector3 worldVector) => WorldToPoolDirection(worldVector);
    private Vector3 WorldToPoolDirection(Vector3 worldVector) =>
        poolFrame != null ? poolFrame.InverseTransformDirection(worldVector) : worldVector;

    private void ResetTask()
    {
        ResolveReferences();
        terminal = false;
        terminalReason = "running";
        holdSeconds = 0f;
        invalidNetSeconds = 0f;
        ResetChasers();
        target?.ResetTo(ResolveTargetInitialPosition(), Quaternion.identity);
        xpbdNet?.ResetSimulation();

        if (TryGetTaskState(out TaskState state))
        {
            previousNetTargetDistance = state.netTargetDistance;
            previousTargetGoalDistance = state.targetGoalDistance;
            previousShapeError = state.shapeError;
            initialTargetGoalDistance = state.targetGoalDistance;
            minimumTargetGoalDistance = state.targetGoalDistance;
        }
        else
        {
            previousNetTargetDistance = 0f;
            previousTargetGoalDistance = 0f;
            previousShapeError = 0f;
            initialTargetGoalDistance = 0f;
            minimumTargetGoalDistance = 0f;
        }
    }

    private void Finish(bool success, string reason)
    {
        if (terminal)
        {
            return;
        }
        terminal = true;
        terminalReason = reason;
        AddSharedReward(success ? successReward : failurePenalty);
        Academy.Instance.StatsRecorder.Add("TriNetCapture/success", success ? 1f : 0f);
        Academy.Instance.StatsRecorder.Add("TriNetCapture/hold_seconds", holdSeconds);
        Academy.Instance.StatsRecorder.Add("TriNetCapture/target_goal_initial_m", initialTargetGoalDistance);
        Academy.Instance.StatsRecorder.Add("TriNetCapture/target_goal_minimum_m", minimumTargetGoalDistance);
        Academy.Instance.StatsRecorder.Add("TriNetCapture/target_goal_final_m", previousTargetGoalDistance);
        foreach (TriNetCaptureAgent chaser in Chasers)
        {
            if (chaser != null)
            {
                chaser.EndEpisode();
            }
        }
        begunAgents.Clear();
    }

    private void AddSharedReward(float reward)
    {
        foreach (TriNetCaptureAgent chaser in Chasers)
        {
            if (chaser != null)
            {
                chaser.AddReward(reward);
            }
        }
    }

    private bool TryGetTaskState(out TaskState state)
    {
        state = default;
        if (target == null || recoveryGoal == null || xpbdNet == null)
        {
            return false;
        }
        if (!xpbdNet.TryGetTriangleVertices(out Vector3 a, out Vector3 b, out Vector3 c))
        {
            return false;
        }
        Vector3 capturePoint = target.GetCapturePoint(xpbdNet.NetCenter);
        if (!xpbdNet.TryGetClosestSurfaceDistance(target.Collider, out float clothSurfaceDistance))
        {
            return false;
        }
        float doubleArea = Vector3.Cross(b - a, c - a).magnitude;
        if (doubleArea * 0.5f < invalidTriangleArea)
        {
            return false;
        }
        state.netTargetDistance = clothSurfaceDistance;
        state.targetGoalDistance = Vector3.Distance(target.transform.position, recoveryGoal.GuidancePoint);
        state.shapeError = (
            Mathf.Abs(Vector3.Distance(a, b) - desiredSideLength) +
            Mathf.Abs(Vector3.Distance(b, c) - desiredSideLength) +
            Mathf.Abs(Vector3.Distance(c, a) - desiredSideLength)) / 3f;
        state.goalContains = recoveryGoal.Contains(capturePoint);
        // Capture is defined by the actual deformed XPBD cloth, not the ideal
        // plane through the three ROV anchors. This remains a surface-distance
        // threshold so the Target cannot count as held after it has slipped away.
        state.netHeld = clothSurfaceDistance <= targetNetDistance;
        return true;
    }

    private void RecordStepStats(TaskState state)
    {
        Academy.Instance.StatsRecorder.Add("TriNetCapture/target_goal_distance_m", state.targetGoalDistance);
        Academy.Instance.StatsRecorder.Add("TriNetCapture/net_held", state.netHeld ? 1f : 0f);
        Academy.Instance.StatsRecorder.Add("TriNetCapture/inside_goal", state.goalContains ? 1f : 0f);
        Academy.Instance.StatsRecorder.Add("TriNetCapture/hold_seconds", holdSeconds);
    }

    private static bool TryGetTriangularSurfaceMetrics(
        Vector3 a, Vector3 b, Vector3 c, Vector3 worldPoint,
        out Vector3 projectedPoint, out float surfaceDistance, out float signedSurfaceDistance,
        out Vector3 barycentric, out float minimumEdgeMargin)
    {
        projectedPoint = Vector3.zero;
        surfaceDistance = 0f;
        signedSurfaceDistance = 0f;
        barycentric = Vector3.zero;
        minimumEdgeMargin = 0f;

        Vector3 normal = Vector3.Cross(b - a, c - a);
        float twiceArea = normal.magnitude;
        if (twiceArea < 1e-6f)
            return false;

        normal /= twiceArea;
        signedSurfaceDistance = Vector3.Dot(worldPoint - a, normal);
        projectedPoint = worldPoint - signedSurfaceDistance * normal;
        surfaceDistance = Mathf.Abs(signedSurfaceDistance);

        Vector3 v0 = b - a;
        Vector3 v1 = c - a;
        Vector3 v2 = projectedPoint - a;
        float d00 = Vector3.Dot(v0, v0);
        float d01 = Vector3.Dot(v0, v1);
        float d11 = Vector3.Dot(v1, v1);
        float d20 = Vector3.Dot(v2, v0);
        float d21 = Vector3.Dot(v2, v1);
        float denominator = d00 * d11 - d01 * d01;
        if (Mathf.Abs(denominator) < 1e-8f)
            return false;

        float baryB = (d11 * d20 - d01 * d21) / denominator;
        float baryC = (d00 * d21 - d01 * d20) / denominator;
        float baryA = 1f - baryB - baryC;
        barycentric = new Vector3(baryA, baryB, baryC);

        float altitudeA = twiceArea / Vector3.Distance(b, c);
        float altitudeB = twiceArea / Vector3.Distance(c, a);
        float altitudeC = twiceArea / Vector3.Distance(a, b);
        minimumEdgeMargin = Mathf.Min(baryA * altitudeA, baryB * altitudeB, baryC * altitudeC);
        return true;
    }

    private Vector3 SampleTargetPosition()
    {
        for (int attempt = 0; attempt < spawnAttempts; attempt++)
        {
            Vector3 local = new Vector3(
                UnityEngine.Random.Range(poolMinLocal.x + wallClearance, poolMaxLocal.x - wallClearance),
                UnityEngine.Random.Range(poolMinLocal.y + wallClearance, poolMaxLocal.y - wallClearance),
                UnityEngine.Random.Range(poolMinLocal.z + wallClearance, poolMaxLocal.z - wallClearance));
            Vector3 candidate = LocalToWorld(local);
            if (IsValidSpawn(candidate))
            {
                return candidate;
            }
        }
        return LocalToWorld(new Vector3(1.20f, -0.40f, 0f));
    }

    private Vector3 ResolveTargetInitialPosition()
    {
        return randomizeTargetInitialPosition
            ? SampleTargetPosition()
            : LocalToWorld(fixedTargetInitialLocalPosition);
    }

    private bool IsValidSpawn(Vector3 candidate)
    {
        if (recoveryGoal != null && Vector3.Distance(candidate, recoveryGoal.GuidancePoint) < goalClearance)
        {
            return false;
        }
        foreach (TriNetCaptureAgent chaser in Chasers)
        {
            if (chaser != null && Vector3.Distance(candidate, chaser.transform.position) < rovClearance)
            {
                return false;
            }
        }
        return true;
    }

    private bool IsInsideSafePool(Vector3 worldPoint)
    {
        Vector3 local = WorldToLocal(worldPoint);
        return local.x >= poolMinLocal.x + wallClearance && local.x <= poolMaxLocal.x - wallClearance &&
            local.y >= poolMinLocal.y + wallClearance && local.y <= poolMaxLocal.y + wallClearance &&
            local.z >= poolMinLocal.z + wallClearance && local.z <= poolMaxLocal.z - wallClearance;
    }

    private void ResolveReferences()
    {
        if (poolFrame == null)
        {
            poolFrame = transform.parent != null ? transform.parent : transform;
        }
        if (xpbdNet == null)
        {
            xpbdNet = GetComponentInChildren<HabradorTriangleTowDriver>(true);
        }
    }

    private void CaptureInitialChaserStates()
    {
        TriNetCaptureAgent[] chasers = Chasers;
        initialLocalPositions = new Vector3[chasers.Length];
        initialLocalRotations = new Quaternion[chasers.Length];
        for (int index = 0; index < chasers.Length; index++)
        {
            if (chasers[index] == null)
            {
                continue;
            }
            initialLocalPositions[index] = WorldToLocal(chasers[index].transform.position);
            initialLocalRotations[index] = Quaternion.Inverse(poolFrame.rotation) * chasers[index].transform.rotation;
        }
    }

    private void ResetChasers()
    {
        if (initialLocalPositions == null || initialLocalPositions.Length != Chasers.Length)
        {
            CaptureInitialChaserStates();
        }
        TriNetCaptureAgent[] chasers = Chasers;
        for (int index = 0; index < chasers.Length; index++)
        {
            TriNetCaptureAgent chaser = chasers[index];
            if (chaser == null)
            {
                continue;
            }
            chaser.transform.SetPositionAndRotation(LocalToWorld(initialLocalPositions[index]), poolFrame.rotation * initialLocalRotations[index]);
            Rigidbody rigidbody = chaser.GetComponent<Rigidbody>();
            if (rigidbody != null)
            {
                rigidbody.linearVelocity = Vector3.zero;
                rigidbody.angularVelocity = Vector3.zero;
            }
        }
    }

    private Vector3 LocalToWorld(Vector3 local) => poolFrame != null ? poolFrame.TransformPoint(local) : local;
    private Vector3 WorldToLocal(Vector3 world) => poolFrame != null ? poolFrame.InverseTransformPoint(world) : world;

    private struct TaskState
    {
        public float netTargetDistance;
        public float targetGoalDistance;
        public float shapeError;
        public bool netHeld;
        public bool goalContains;
    }

    private struct NetObservationSummary
    {
        public Vector3 sortedEdgeErrors;
        public float signedSurfaceDistance;
        public float minimumEdgeMargin;
    }
}
