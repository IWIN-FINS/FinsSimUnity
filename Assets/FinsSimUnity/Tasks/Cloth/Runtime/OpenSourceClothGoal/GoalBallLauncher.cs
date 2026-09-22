using UnityEngine;

/// <summary>Repeatedly fires one Rigidbody ball straight at a goal net.</summary>
[RequireComponent(typeof(Rigidbody))]
public sealed class GoalBallLauncher : MonoBehaviour
{
    public Vector3 launchPosition = new(0f, 1.7f, -7f);
    public Vector3 launchVelocity = new(0f, 0f, 9f);
    [Min(0f)] public float launchDelaySeconds = 1f;
    [Min(0.1f)] public float resetAfterSeconds = 7f;

    private Rigidbody body;
    private float elapsed;
    private bool launched;

    private void Awake()
    {
        body = GetComponent<Rigidbody>();
    }

    private void OnEnable()
    {
        elapsed = 0f;
        launched = false;
        ResetBall();
    }

    private void FixedUpdate()
    {
        elapsed += Time.fixedDeltaTime;
        if (elapsed >= resetAfterSeconds)
        {
            elapsed = 0f;
            launched = false;
            ResetBall();
        }
        else if (!launched && elapsed >= launchDelaySeconds)
        {
            body.linearVelocity = launchVelocity;
            launched = true;
        }
    }

    private void ResetBall()
    {
        body.position = launchPosition;
        body.rotation = Quaternion.identity;
        body.linearVelocity = Vector3.zero;
        body.angularVelocity = Vector3.zero;
    }
}
