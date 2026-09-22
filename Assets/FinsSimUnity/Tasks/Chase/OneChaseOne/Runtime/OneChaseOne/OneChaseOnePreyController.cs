using UnityEngine;

[DisallowMultipleComponent]
[RequireComponent(typeof(Rigidbody))]
public class OneChaseOnePreyController : MonoBehaviour
{
    public enum MotionMode
    {
        Random,
        RandomStraightLine,
        HybridEscape,
        LineOfSightEscape
    }

    private Rigidbody rigidBody;
    private Vector3 homePosition;
    private bool homeCaptured;
    private Vector3 smoothedEscapeDirection = Vector3.forward;
    private Vector3 randomWanderDirection = Vector3.forward;
    private Vector3 randomStraightLineDirection = Vector3.forward;
    private float nextRandomWanderDirectionTime;

    [Header("References")]
    public Transform chaserTransform;

    [Header("Motion")]
    [SerializeField] private MotionMode motionMode = MotionMode.HybridEscape;
    [SerializeField] private float moveSpeed = 2.0f;
    [SerializeField] private float verticalSpeedScale = 0.15f;
    [SerializeField] private float directionSmoothing = 0.35f;
    [SerializeField] private float randomDirectionHoldSeconds = 1.5f;
    [SerializeField] private float noiseWeight = 0.10f;
    [SerializeField] private float centerRepulsionWeight = 0.25f;
    [SerializeField] private float threatWeight = 1.0f;
    [SerializeField] private bool faceMotionDirection = true;

    [Header("Bounds")]
    [SerializeField] private bool constrainToWaterColumn = true;
    [SerializeField] private float waterSurfaceY = 0f;
    [SerializeField] private float surfaceClearance = 0f;
    [SerializeField] private float minWaterY = -6f;
    [SerializeField] private float verticalBoundaryBuffer = 0.35f;
    [SerializeField] private float verticalBoundaryCorrectionGain = 4f;
    [SerializeField] private float horizontalPatrolRadius = 8f;

    public Vector3 CurrentVelocity => rigidBody != null ? rigidBody.linearVelocity : Vector3.zero;

    private Rigidbody RigidBody
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

    private void Awake()
    {
        rigidBody = RigidBody;
        CaptureHomePosition();
        ResetMotionDirections();
    }

    private void FixedUpdate()
    {
        CaptureHomePosition();

        Vector3 desiredDirection = ComputeDesiredDirection();
        if (desiredDirection.sqrMagnitude < 1e-6f)
        {
            desiredDirection = transform.forward.sqrMagnitude > 1e-6f ? transform.forward : Vector3.forward;
        }

        Vector3 smoothedDirection = Vector3.Lerp(desiredDirection.normalized, smoothedEscapeDirection, Mathf.Clamp01(directionSmoothing));
        smoothedEscapeDirection = smoothedDirection.sqrMagnitude > 1e-6f ? smoothedDirection.normalized : desiredDirection.normalized;

        Vector3 desiredVelocity = new Vector3(
            smoothedEscapeDirection.x,
            smoothedEscapeDirection.y * verticalSpeedScale,
            smoothedEscapeDirection.z
        ).normalized * moveSpeed;

        desiredVelocity = constrainToWaterColumn ? ApplyWaterColumnConstraint(desiredVelocity) : desiredVelocity;
        RigidBody.linearVelocity = desiredVelocity;

        if (faceMotionDirection)
        {
            Vector3 planarVelocity = Vector3.ProjectOnPlane(desiredVelocity, Vector3.up);
            if (planarVelocity.sqrMagnitude > 1e-6f)
            {
                transform.rotation = Quaternion.LookRotation(planarVelocity.normalized, Vector3.up);
            }
        }
    }

    public void ResetForEpisode(Vector3 spawnPosition, Quaternion spawnRotation)
    {
        transform.position = spawnPosition;
        transform.rotation = spawnRotation;
        RigidBody.linearVelocity = Vector3.zero;
        RigidBody.angularVelocity = Vector3.zero;
        smoothedEscapeDirection = transform.forward.sqrMagnitude > 1e-6f ? transform.forward.normalized : Vector3.forward;
        CaptureHomePosition(force: true);
        ResetMotionDirections();
    }

    /// <summary>Sets area-local references and water-column limits for a replicated area.</summary>
    public void ConfigureForParallelTrainingArea(
        Transform areaTransform,
        Transform areaChaser,
        float localWaterSurfaceY,
        float localMinWaterY)
    {
        chaserTransform = areaChaser;
        float areaY = areaTransform != null ? areaTransform.position.y : 0f;
        waterSurfaceY = areaY + localWaterSurfaceY;
        minWaterY = areaY + localMinWaterY;
        homeCaptured = false;
        CaptureHomePosition(force: true);
    }

    private void CaptureHomePosition(bool force = false)
    {
        if (homeCaptured && !force)
        {
            return;
        }

        homePosition = transform.position;
        homeCaptured = true;
    }

    private Vector3 ComputeDesiredDirection()
    {
        switch (motionMode)
        {
            case MotionMode.Random:
                return GetRandomWanderDirection();
            case MotionMode.RandomStraightLine:
                return randomStraightLineDirection;
            case MotionMode.LineOfSightEscape:
                return ComputeLineOfSightEscapeDirection();
            case MotionMode.HybridEscape:
            default:
                return ComputeHybridEscapeDirection();
        }
    }

    private Vector3 ComputeHybridEscapeDirection()
    {
        Vector3 escapeDirection = Vector3.zero;
        Vector3 lineOfSightEscapeDirection = ComputeLineOfSightEscapeDirection();
        if (lineOfSightEscapeDirection.sqrMagnitude > 1e-6f)
        {
            escapeDirection += threatWeight * lineOfSightEscapeDirection;
        }

        Vector3 centerOffset = transform.position - homePosition;
        Vector3 centerCorrection = centerOffset.sqrMagnitude > 1e-6f
            ? -centerOffset.normalized * Mathf.Clamp01(centerOffset.magnitude / Mathf.Max(0.1f, horizontalPatrolRadius))
            : Vector3.zero;
        escapeDirection += centerRepulsionWeight * centerCorrection;

        Vector3 noiseDirection = new Vector3(
            Mathf.PerlinNoise(Time.time * 0.31f, transform.position.z * 0.07f) * 2f - 1f,
            Mathf.PerlinNoise(transform.position.x * 0.05f, Time.time * 0.23f) * 2f - 1f,
            Mathf.PerlinNoise(Time.time * 0.17f, transform.position.x * 0.09f) * 2f - 1f
        );
        escapeDirection += noiseWeight * noiseDirection;

        return escapeDirection.sqrMagnitude > 1e-6f ? escapeDirection.normalized : Vector3.zero;
    }

    private Vector3 ComputeLineOfSightEscapeDirection()
    {
        if (chaserTransform == null)
        {
            return Vector3.zero;
        }

        Vector3 awayFromChaser = transform.position - chaserTransform.position;
        return awayFromChaser.sqrMagnitude > 1e-6f ? awayFromChaser.normalized : Vector3.zero;
    }

    private Vector3 GetRandomWanderDirection()
    {
        if (Time.time >= nextRandomWanderDirectionTime)
        {
            randomWanderDirection = SampleRandomDirection();
            nextRandomWanderDirectionTime = Time.time + Mathf.Max(0.02f, randomDirectionHoldSeconds);
        }

        return randomWanderDirection;
    }

    private void ResetMotionDirections()
    {
        randomWanderDirection = SampleRandomDirection();
        randomStraightLineDirection = SampleRandomDirection();
        nextRandomWanderDirectionTime = Time.time + Mathf.Max(0.02f, randomDirectionHoldSeconds);
    }

    private static Vector3 SampleRandomDirection()
    {
        Vector3 direction = Random.insideUnitSphere;
        return direction.sqrMagnitude > 1e-6f ? direction.normalized : Vector3.forward;
    }

    private Vector3 ApplyWaterColumnConstraint(Vector3 desiredVelocity)
    {
        float upperY = waterSurfaceY - Mathf.Max(0f, surfaceClearance);
        float lowerY = Mathf.Min(minWaterY, upperY);
        float buffer = Mathf.Max(0f, verticalBoundaryBuffer);
        float currentY = transform.position.y;

        if (buffer > 1e-4f)
        {
            if (desiredVelocity.y > 0f && currentY > upperY - buffer)
            {
                desiredVelocity.y *= Mathf.InverseLerp(upperY, upperY - buffer, currentY);
            }
            else if (desiredVelocity.y < 0f && currentY < lowerY + buffer)
            {
                desiredVelocity.y *= Mathf.InverseLerp(lowerY, lowerY + buffer, currentY);
            }
        }

        if (currentY > upperY)
        {
            desiredVelocity.y = Mathf.Min(desiredVelocity.y, -(currentY - upperY) * verticalBoundaryCorrectionGain);
        }
        else if (currentY < lowerY)
        {
            desiredVelocity.y = Mathf.Max(desiredVelocity.y, (lowerY - currentY) * verticalBoundaryCorrectionGain);
        }

        return desiredVelocity;
    }
}
