using MarusThruster = FinsSim.Actuators.Thruster;
using MarusThrusterController = FinsSim.Actuators.ThrusterController;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

/// <summary>
/// Dedicated ML-Agents endpoint for TriNetCapture.
/// </summary>
[DisallowMultipleComponent]
public sealed class TriNetCaptureAgent : Agent
{
    public const int ActorObservationSize = 34;
    public const int PrivilegedStateSize = 57;
    public const int VectorObservationSize = ActorObservationSize + PrivilegedStateSize;

    [SerializeField] private Transform selfTransform;
    [SerializeField] private MarusThrusterController thrusterControllerOverride;
    [SerializeField] private bool autoResolveThrustersFromChildren = true;

    private readonly float[] appliedActions = new float[8];
    private readonly MarusThruster[] orderedThrusters = new MarusThruster[FinsROVAgentRuntime.DefaultThrusterOrder.Length];
    private Rigidbody rigidbodyComponent;
    private MarusThrusterController thrusterController;
    private TriNetCaptureTaskManager task;

    public Transform PoseTransform => selfTransform != null ? selfTransform : transform;

    public override void Initialize()
    {
        rigidbodyComponent = GetComponent<Rigidbody>();
        task = GetComponentInParent<TriNetCaptureTaskManager>();
        ResolveThrusters();
    }

    public override void OnEpisodeBegin()
    {
        task ??= GetComponentInParent<TriNetCaptureTaskManager>();
        task?.BeginEpisode(this);
        ZeroThrusters();
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        task ??= GetComponentInParent<TriNetCaptureTaskManager>();
        Transform pose = PoseTransform;
        Rigidbody targetRigidbody = task != null && task.Target != null ? task.Target.GetComponent<Rigidbody>() : null;
        Vector3 targetPosition = task != null && task.Target != null ? task.Target.transform.position : pose.position;
        Vector3 selfVelocity = rigidbodyComponent != null ? rigidbodyComponent.linearVelocity : Vector3.zero;
        Vector3 selfAngularVelocity = rigidbodyComponent != null ? rigidbodyComponent.angularVelocity : Vector3.zero;
        Vector3 targetVelocity = targetRigidbody != null ? targetRigidbody.linearVelocity : Vector3.zero;
        Vector3 targetRelative = ControllerBodyFrame.WorldToBodyPositionDelta(pose, targetPosition - pose.position);

        sensor.AddObservation(ControllerBodyFrame.WorldLinearVelocityToBody(pose, selfVelocity));
        sensor.AddObservation(ControllerBodyFrame.WorldAngularVelocityToBody(pose, selfAngularVelocity));
        sensor.AddObservation(targetRelative);
        sensor.AddObservation(ControllerBodyFrame.WorldLinearVelocityToBody(pose, targetVelocity - selfVelocity));
        // Keep the 34D actor interface stable. Target distance is redundant
        // (it is the norm of targetRelative); expose own pool-local Y here so
        // the scripted lift stage can record a real height at phase entry.
        sensor.AddObservation(task != null ? task.GetPoolLocalY(pose.position) : pose.position.y);
        // Slot 13 is reserved for the scripted baseline's desired heading:
        // the signed error required to hold pool/world yaw at zero degrees.
        sensor.AddObservation(task != null
            ? task.GetYawErrorToHoldPoolYawZeroDegrees(pose)
            : ControllerBodyFrame.HorizontalBearingRad(
                ControllerBodyFrame.WorldToBodyVector(pose, Vector3.right)) * Mathf.Rad2Deg);

        TriNetCaptureAgent[] teammates = task != null ? task.GetOtherAgents(this) : new TriNetCaptureAgent[0];
        for (int index = 0; index < 2; index++)
        {
            TriNetCaptureAgent teammate = index < teammates.Length ? teammates[index] : null;
            AddTeammateObservation(sensor, teammate);
        }
        task?.CollectTaskObservations(this, sensor);
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        for (int index = 0; index < appliedActions.Length; index++)
        {
            appliedActions[index] = index < actions.ContinuousActions.Length
                ? Mathf.Clamp(actions.ContinuousActions[index], -1f, 1f)
                : 0f;
        }
        ResolveThrusters();
        // The Python baseline already turns its body-frame PID wrench into
        // eight normalized thruster commands. Preserve that allocation here:
        // each action is applied against the corresponding thruster's own
        // configured force limit, not against a shared force-scale gain.
        FinsROVAgentRuntime.ApplyThrusterActions(
            orderedThrusters,
            appliedActions,
            ThrusterCommandMode.NormalizedMaxForceRequest);
        task ??= GetComponentInParent<TriNetCaptureTaskManager>();
        task?.NotifyActionApplied(this);
    }

    private void OnCollisionEnter(Collision collision)
    {
        task ??= GetComponentInParent<TriNetCaptureTaskManager>();
        task?.NotifyCollision(this, collision.collider);
    }

    private void AddTeammateObservation(VectorSensor sensor, TriNetCaptureAgent teammate)
    {
        if (teammate == null)
        {
            sensor.AddObservation(Vector3.zero);
            sensor.AddObservation(Vector3.zero);
            return;
        }
        Transform pose = PoseTransform;
        Rigidbody teammateRigidbody = teammate.GetComponent<Rigidbody>();
        Vector3 teammateVelocity = teammateRigidbody != null ? teammateRigidbody.linearVelocity : Vector3.zero;
        Vector3 selfVelocity = rigidbodyComponent != null ? rigidbodyComponent.linearVelocity : Vector3.zero;
        sensor.AddObservation(ControllerBodyFrame.WorldToBodyPositionDelta(
            pose, teammate.PoseTransform.position - pose.position));
        sensor.AddObservation(ControllerBodyFrame.WorldLinearVelocityToBody(pose, teammateVelocity - selfVelocity));
    }

    private void ResolveThrusters()
    {
        thrusterController ??= thrusterControllerOverride != null
            ? thrusterControllerOverride
            : GetComponent<MarusThrusterController>();
        if (!FinsROVAgentRuntime.TryResolveOrderedThrusters(
                this, thrusterController, autoResolveThrustersFromChildren, orderedThrusters, out string status))
        {
            Debug.LogError($"[{nameof(TriNetCaptureAgent)}] {status}", this);
            return;
        }
        FinsROVAgentRuntime.EnsureThrusterControllerOrder(thrusterController, orderedThrusters);
    }

    private void ZeroThrusters()
    {
        ResolveThrusters();
        FinsROVAgentRuntime.ZeroThrusters(
            orderedThrusters,
            ThrusterCommandMode.NormalizedMaxForceRequest);
    }
}
