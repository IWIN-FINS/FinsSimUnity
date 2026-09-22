using UnityEngine;

[DisallowMultipleComponent]
public class SmoothRandomMovingTarget : MonoBehaviour
{
    [Header("Motion Bounds")]
    [Tooltip("Movement extents around the reset center, in meters.")]
    [SerializeField] Vector3 movementExtents = new Vector3(2.5f, 0.8f, 2.5f);
    [Tooltip("World-space water surface height. The target will not move above this Y value when clamping is enabled.")]
    [SerializeField] float surfaceY = 0f;
    [Tooltip("Keep the target at or below the configured water surface.")]
    [SerializeField] bool clampBelowSurface = true;

    [Header("Smooth Random Motion")]
    [SerializeField] bool moveInFixedUpdate = true;
    [SerializeField] float minSpeed = 0.00f;
    [SerializeField] float maxSpeed = 0.10f;
    [SerializeField] float maxAcceleration = 0.08f;
    [SerializeField] float maxTurnRateDegPerSecond = 25f;
    [SerializeField] float maxPlannedTurnAngleDeg = 45f;
    [SerializeField] float verticalDirectionScale = 0.4f;
    [SerializeField] float retargetIntervalMin = 1.5f;
    [SerializeField] float retargetIntervalMax = 4.0f;
    [SerializeField] float boundarySteeringStartRatio = 0.65f;

    [Header("Visual")]
    [SerializeField] bool drawDebugBounds = true;

    Vector3 centerPosition;
    Vector3 velocity;
    Vector3 desiredVelocity;
    bool initialized;
    float nextRetargetTime;

    public Vector3 CurrentVelocity => velocity;
    public Vector3 CenterPosition => centerPosition;
    public Vector3 MovementExtents => GetSafeExtents();
    public float MaxSpeed => Mathf.Max(0f, Mathf.Max(minSpeed, maxSpeed));
    public float SurfaceY => surfaceY;
    public bool ClampBelowSurface => clampBelowSurface;

    void OnEnable()
    {
        ResetMotionAtCurrentPosition();
    }

    void Update()
    {
        if (!moveInFixedUpdate)
        {
            Step(Time.deltaTime);
        }
    }

    void FixedUpdate()
    {
        if (moveInFixedUpdate)
        {
            Step(Time.fixedDeltaTime);
        }
    }

    public void ResetMotionAtCurrentPosition()
    {
        ResetMotion(transform.position, true);
    }

    public void ResetMotion(Vector3 position, bool usePositionAsCenter)
    {
        if (clampBelowSurface && position.y > surfaceY)
        {
            position.y = surfaceY;
        }

        transform.position = position;
        if (usePositionAsCenter || !initialized)
        {
            centerPosition = position;
        }

        Vector3 initialDirection = GenerateRandomDirection(Vector3.forward, 180f);
        float initialSpeed = Random.Range(GetSafeMinSpeed(), GetSafeMaxSpeed());
        velocity = initialDirection * initialSpeed;
        desiredVelocity = velocity;
        initialized = true;
        ScheduleNextRetarget();
    }

    public void SetCenterPosition(Vector3 center)
    {
        if (clampBelowSurface && center.y > surfaceY)
        {
            center.y = surfaceY;
        }

        centerPosition = center;
        initialized = true;
    }

    void Step(float deltaTime)
    {
        if (deltaTime <= 0f)
        {
            return;
        }

        if (!initialized)
        {
            ResetMotionAtCurrentPosition();
        }

        if (Time.time >= nextRetargetTime || desiredVelocity.sqrMagnitude < 1e-6f)
        {
            PickNewDesiredVelocity();
        }

        ApplyBoundarySteering();

        Vector3 previousPosition = transform.position;
        Vector3 targetDirection = desiredVelocity.sqrMagnitude > 1e-6f
            ? desiredVelocity.normalized
            : GenerateRandomDirection(Vector3.forward, 180f);
        Vector3 currentDirection = velocity.sqrMagnitude > 1e-6f
            ? velocity.normalized
            : targetDirection;

        float maxRadiansDelta = Mathf.Deg2Rad * Mathf.Max(0f, maxTurnRateDegPerSecond) * deltaTime;
        Vector3 nextDirection = Vector3.RotateTowards(currentDirection, targetDirection, maxRadiansDelta, 0f);
        float nextSpeed = Mathf.MoveTowards(
            velocity.magnitude,
            Mathf.Clamp(desiredVelocity.magnitude, GetSafeMinSpeed(), GetSafeMaxSpeed()),
            Mathf.Max(0f, maxAcceleration) * deltaTime);

        Vector3 candidatePosition = previousPosition + nextDirection * nextSpeed * deltaTime;
        Vector3 clampedPosition = ClampToBounds(candidatePosition);
        transform.position = clampedPosition;
        velocity = (clampedPosition - previousPosition) / deltaTime;
    }

    void PickNewDesiredVelocity()
    {
        Vector3 baseDirection = velocity.sqrMagnitude > 1e-6f
            ? velocity.normalized
            : GenerateRandomDirection(Vector3.forward, 180f);
        Vector3 direction = GenerateRandomDirection(baseDirection, Mathf.Max(0f, maxPlannedTurnAngleDeg));
        desiredVelocity = direction * Random.Range(GetSafeMinSpeed(), GetSafeMaxSpeed());
        ScheduleNextRetarget();
    }

    void ApplyBoundarySteering()
    {
        Vector3 extents = GetSafeExtents();
        Vector3 offset = transform.position - centerPosition;
        float xRatio = Mathf.Abs(offset.x) / extents.x;
        float yRatio = Mathf.Abs(offset.y) / extents.y;
        float zRatio = Mathf.Abs(offset.z) / extents.z;
        float ratio = Mathf.Max(xRatio, Mathf.Max(yRatio, zRatio));

        if (clampBelowSurface && transform.position.y > surfaceY - extents.y * 0.25f)
        {
            ratio = Mathf.Max(ratio, boundarySteeringStartRatio);
        }

        if (ratio < boundarySteeringStartRatio)
        {
            return;
        }

        Vector3 inward = new Vector3(
            -offset.x / extents.x,
            -offset.y / extents.y,
            -offset.z / extents.z);

        if (clampBelowSurface && transform.position.y > surfaceY - extents.y * 0.25f)
        {
            inward.y -= 1f;
        }

        if (inward.sqrMagnitude < 1e-6f)
        {
            inward = Vector3.down;
        }

        float steerWeight = Mathf.InverseLerp(boundarySteeringStartRatio, 1f, ratio);
        Vector3 currentDirection = desiredVelocity.sqrMagnitude > 1e-6f
            ? desiredVelocity.normalized
            : velocity.normalized;
        Vector3 steeredDirection = Vector3.Slerp(currentDirection, inward.normalized, Mathf.Clamp01(steerWeight));
        desiredVelocity = steeredDirection.normalized * Mathf.Clamp(
            desiredVelocity.magnitude,
            GetSafeMinSpeed(),
            GetSafeMaxSpeed());
    }

    Vector3 ClampToBounds(Vector3 position)
    {
        GetClampedMotionBounds(out Vector3 min, out Vector3 max);

        return new Vector3(
            Mathf.Clamp(position.x, min.x, max.x),
            Mathf.Clamp(position.y, min.y, max.y),
            Mathf.Clamp(position.z, min.z, max.z));
    }

    void GetClampedMotionBounds(out Vector3 min, out Vector3 max)
    {
        Vector3 extents = GetSafeExtents();
        min = centerPosition - extents;
        max = centerPosition + extents;
        if (clampBelowSurface)
        {
            max.y = Mathf.Min(max.y, surfaceY);
        }
    }

    Vector3 GenerateRandomDirection(Vector3 baseDirection, float maxTurnAngleDeg)
    {
        Vector3 randomDirection = Random.insideUnitSphere;
        randomDirection.y *= Mathf.Clamp01(verticalDirectionScale);
        if (randomDirection.sqrMagnitude < 1e-6f)
        {
            randomDirection = Vector3.forward;
        }

        randomDirection.Normalize();
        if (baseDirection.sqrMagnitude < 1e-6f || maxTurnAngleDeg >= 179.9f)
        {
            return randomDirection;
        }

        return Vector3.RotateTowards(
            baseDirection.normalized,
            randomDirection,
            Mathf.Deg2Rad * Mathf.Max(0f, maxTurnAngleDeg),
            0f).normalized;
    }

    void ScheduleNextRetarget()
    {
        float minInterval = Mathf.Max(0.02f, retargetIntervalMin);
        float maxInterval = Mathf.Max(minInterval, retargetIntervalMax);
        nextRetargetTime = Time.time + Random.Range(minInterval, maxInterval);
    }

    Vector3 GetSafeExtents()
    {
        return new Vector3(
            Mathf.Max(0.01f, Mathf.Abs(movementExtents.x)),
            Mathf.Max(0.01f, Mathf.Abs(movementExtents.y)),
            Mathf.Max(0.01f, Mathf.Abs(movementExtents.z)));
    }

    float GetSafeMinSpeed()
    {
        return Mathf.Max(0f, Mathf.Min(minSpeed, maxSpeed));
    }

    float GetSafeMaxSpeed()
    {
        return Mathf.Max(0f, Mathf.Max(minSpeed, maxSpeed));
    }

    void OnDrawGizmosSelected()
    {
        if (!drawDebugBounds)
        {
            return;
        }

        Gizmos.color = new Color(0.2f, 0.8f, 1f, 0.35f);
        Vector3 gizmoCenterPosition = Application.isPlaying && initialized ? centerPosition : transform.position;
        if (clampBelowSurface && gizmoCenterPosition.y > surfaceY)
        {
            gizmoCenterPosition.y = surfaceY;
        }

        Vector3 previousCenter = centerPosition;
        centerPosition = gizmoCenterPosition;
        GetClampedMotionBounds(out Vector3 min, out Vector3 max);
        centerPosition = previousCenter;

        Gizmos.DrawWireCube((min + max) * 0.5f, max - min);
    }
}
