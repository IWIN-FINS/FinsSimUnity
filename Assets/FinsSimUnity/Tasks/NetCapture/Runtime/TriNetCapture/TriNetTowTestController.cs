using UnityEngine;

/// <summary>
/// Deterministic physical towing check: three kinematic FinsROVs preserve a
/// triangular net formation and translate it along +X. The Target remains a
/// dynamic Rigidbody, so any forward motion must come from rope contact.
/// </summary>
[DisallowMultipleComponent]
public sealed class TriNetTowTestController : MonoBehaviour
{
    [SerializeField] private TriNetCaptureTaskManager task;
    [SerializeField] private Vector3 leftStart = new(-0.50f, -0.80f, -0.50f);
    [SerializeField] private Vector3 topStart = new(-0.50f, -0.25f, 0f);
    [SerializeField] private Vector3 rightStart = new(-0.50f, -0.80f, 0.50f);
    [SerializeField] private float startDelaySeconds = 2f;
    [SerializeField] private float towSpeedMetersPerSecond = 0.10f;
    [SerializeField] private float towDurationSeconds = 12f;

    private Rigidbody[] rovBodies;
    private Vector3[] startPositions;
    private float startedAt;

    private void OnValidate()
    {
        if (task == null)
        {
            task = GetComponent<TriNetCaptureTaskManager>();
        }
    }

    public void Configure(
        TriNetCaptureTaskManager configuredTask,
        Vector3 configuredLeftStart,
        Vector3 configuredTopStart,
        Vector3 configuredRightStart)
    {
        task = configuredTask;
        leftStart = configuredLeftStart;
        topStart = configuredTopStart;
        rightStart = configuredRightStart;
    }

    private void Awake()
    {
        if (task == null)
        {
            task = GetComponent<TriNetCaptureTaskManager>();
        }

        // This scene is a contact test, not an ML episode. Keep the actual
        // Target physics active while removing policy/reset ownership.
        if (task != null)
        {
            task.enabled = false;
        }

        TriNetCaptureAgent[] agents = task != null
            ? new[] { task.Left, task.Top, task.Right }
            : new TriNetCaptureAgent[0];
        rovBodies = new Rigidbody[agents.Length];
        startPositions = new[] { leftStart, topStart, rightStart };
        for (int index = 0; index < agents.Length; index++)
        {
            TriNetCaptureAgent agent = agents[index];
            if (agent == null)
            {
                continue;
            }

            agent.enabled = false;
            Rigidbody body = agent.GetComponent<Rigidbody>();
            rovBodies[index] = body;
            if (body != null)
            {
                body.isKinematic = true;
                body.position = startPositions[index];
                body.rotation = Quaternion.identity;
            }
            else
            {
                agent.transform.SetPositionAndRotation(startPositions[index], Quaternion.identity);
            }
        }

        startedAt = Time.time;
    }

    private void FixedUpdate()
    {
        float elapsed = Time.time - startedAt - startDelaySeconds;
        if (elapsed <= 0f)
        {
            return;
        }

        float distance = Mathf.Min(elapsed, towDurationSeconds) * towSpeedMetersPerSecond;
        for (int index = 0; index < rovBodies.Length; index++)
        {
            Rigidbody body = rovBodies[index];
            if (body == null)
            {
                continue;
            }

            body.MovePosition(startPositions[index] + Vector3.right * distance);
            body.MoveRotation(Quaternion.identity);
        }
    }
}
