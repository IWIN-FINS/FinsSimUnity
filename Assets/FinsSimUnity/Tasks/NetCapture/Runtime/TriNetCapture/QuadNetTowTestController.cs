using UnityEngine;

/// <summary>Moves four kinematic FinsROVs and their vertical quadrilateral net along +X.</summary>
[DisallowMultipleComponent]
public sealed class QuadNetTowTestController : MonoBehaviour
{
    [SerializeField] private Transform[] rovs;
    [SerializeField] private Vector3[] startPositions;
    [SerializeField] private float startDelaySeconds = 2f;
    [SerializeField] private float towSpeedMetersPerSecond = 0.10f;
    [SerializeField] private float towDurationSeconds = 12f;

    private Rigidbody[] bodies;
    private float startedAt;

    public void Configure(Transform[] configuredRovs, Vector3[] configuredStarts)
    {
        rovs = configuredRovs;
        startPositions = configuredStarts;
    }

    private void Awake()
    {
        bodies = new Rigidbody[rovs != null ? rovs.Length : 0];
        for (int index = 0; index < bodies.Length; index++)
        {
            Transform rov = rovs[index];
            if (rov == null)
            {
                continue;
            }

            TriNetCaptureAgent agent = rov.GetComponent<TriNetCaptureAgent>();
            if (agent != null)
            {
                agent.enabled = false;
            }

            Rigidbody body = rov.GetComponent<Rigidbody>();
            bodies[index] = body;
            Vector3 start = startPositions[index];
            if (body != null)
            {
                body.isKinematic = true;
                body.position = start;
                body.rotation = Quaternion.identity;
            }
            else
            {
                rov.SetPositionAndRotation(start, Quaternion.identity);
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
        for (int index = 0; index < bodies.Length; index++)
        {
            if (bodies[index] == null)
            {
                continue;
            }

            // Fossen/DWP initialization can write Rigidbody flags after Awake.
            // This deterministic contact test owns the four towing anchors.
            bodies[index].MovePosition(startPositions[index] + Vector3.right * distance);
            bodies[index].MoveRotation(Quaternion.identity);
        }
    }
}
