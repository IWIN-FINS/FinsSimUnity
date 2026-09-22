using System.Collections.Generic;
using FinsSim.Hydrodynamics;
using UnityEngine;

/// <summary>
/// Applies a randomized water-current disturbance to one or more rigidbodies.
/// Attach this script to an empty scene object or directly to the vehicle.
/// </summary>
public class WaterCurrentDomainRandomizer : MonoBehaviour, IWaterKinematicsProvider, IEpisodeRandomizable
{
    public enum DirectionSamplingMode
    {
        Horizontal,
        Full3D,
        AroundMean
    }

    public enum CurrentForceMode
    {
        Force,
        Acceleration,
        VelocityChange
    }

    [Header("Targets")]
    [Tooltip("Rigidbodies affected by the randomized current. If empty, the Rigidbody on this GameObject is used.")]
    public List<Rigidbody> targetRigidbodies = new List<Rigidbody>();

    [Header("Randomization")]
    public bool randomizeOnStart = true;
    public bool randomizePeriodically = false;
    [Min(0.02f)] public float randomizationIntervalSeconds = 30f;
    public DirectionSamplingMode directionSampling = DirectionSamplingMode.Horizontal;
    public Vector3 meanCurrentVelocity = Vector3.zero;
    [Min(0f)] public float minCurrentSpeed = 0f;
    [Min(0f)] public float maxCurrentSpeed = 0.35f;
    [Range(0f, 180f)] public float maxAngleFromMeanDegrees = 45f;
    [Tooltip("Optional vertical current range used only in Full3D mode.")]
    public Vector2 verticalVelocityRange = new Vector2(-0.03f, 0.03f);

    [Header("Force Model")]
    [Tooltip("Legacy mode: directly applies current disturbance forces to target rigidbodies. Disable when HydrodynamicsController consumes this component as a water provider.")]
    public bool applyForcesToRigidbodies = true;
    public CurrentForceMode forceMode = CurrentForceMode.Force;
    [Tooltip("Acceleration gain that drives each target's velocity toward the sampled water current.")]
    [Min(0f)] public float accelerationGain = 0.8f;
    [Tooltip("Quadratic drag-like gain applied along relative velocity.")]
    [Min(0f)] public float quadraticDragGain = 0.15f;
    [Tooltip("Clamp disturbance acceleration before multiplying by mass.")]
    [Min(0f)] public float maxAcceleration = 2f;
    public bool includeVerticalForce = false;
    public bool wakeTargetsBeforeApplyingForce = true;

    [Header("Debug")]
    public bool drawDebug = true;
    public bool logRandomizedCurrent = true;
    public bool logAppliedForce = false;
    [Min(0.1f)] public float appliedForceLogIntervalSeconds = 2f;
    public Color debugColor = Color.cyan;
    [Min(0.1f)] public float debugScale = 3f;

    public Vector3 CurrentVelocity { get; private set; }
    public Vector3 LastAppliedAcceleration { get; private set; }
    public Vector3 LastAppliedForce { get; private set; }

    float _nextRandomizationTime;
    float _nextAppliedForceLogTime;

    void Reset()
    {
        var rb = GetComponent<Rigidbody>();
        if (rb != null && targetRigidbodies.Count == 0)
        {
            targetRigidbodies.Add(rb);
        }
    }

    void Awake()
    {
        RemoveMissingTargets();

        if (targetRigidbodies.Count == 0)
        {
            var rb = GetComponent<Rigidbody>();
            if (rb != null)
            {
                targetRigidbodies.Add(rb);
            }
        }
    }

    void Start()
    {
        if (randomizeOnStart)
        {
            RandomizeCurrent();
        }
        else
        {
            CurrentVelocity = meanCurrentVelocity;
        }

        _nextRandomizationTime = Time.time + randomizationIntervalSeconds;
        _nextAppliedForceLogTime = Time.time + appliedForceLogIntervalSeconds;

        if (logAppliedForce)
        {
            Debug.Log($"[{nameof(WaterCurrentDomainRandomizer)}] targets={targetRigidbodies.Count}", this);
        }
    }

    void FixedUpdate()
    {
        if (randomizePeriodically && Time.time >= _nextRandomizationTime)
        {
            RandomizeCurrent();
            _nextRandomizationTime = Time.time + randomizationIntervalSeconds;
        }

        if (applyForcesToRigidbodies)
        {
            ApplyCurrentForces();
        }
    }

    void OnDrawGizmosSelected()
    {
        if (!drawDebug)
        {
            return;
        }

        Gizmos.color = debugColor;
        Gizmos.DrawLine(transform.position, transform.position + CurrentVelocity * debugScale);
        Gizmos.DrawSphere(transform.position + CurrentVelocity * debugScale, 0.08f);
    }

    public void RandomizeCurrent()
    {
        float speed = Random.Range(Mathf.Min(minCurrentSpeed, maxCurrentSpeed), Mathf.Max(minCurrentSpeed, maxCurrentSpeed));
        Vector3 direction = SampleDirection();
        CurrentVelocity = direction * speed;

        if (!includeVerticalForce)
        {
            CurrentVelocity = new Vector3(CurrentVelocity.x, 0f, CurrentVelocity.z);
        }

        if (logRandomizedCurrent)
        {
            Debug.Log($"[{nameof(WaterCurrentDomainRandomizer)}] current={CurrentVelocity:F3} m/s, speed={CurrentVelocity.magnitude:F3} m/s", this);
        }
    }

    public void SetCurrent(Vector3 worldVelocity)
    {
        CurrentVelocity = includeVerticalForce ? worldVelocity : new Vector3(worldVelocity.x, 0f, worldVelocity.z);
    }

    public WaterKinematicsSample Sample(Vector3 worldPoint)
    {
        return new WaterKinematicsSample
        {
            IsValid = true,
            Height = 0f,
            Normal = Vector3.up,
            FlowVelocity = CurrentVelocity,
            AngularFlowVelocity = Vector3.zero,
        };
    }

    public void RandomizeForEpisode(RandomizationContext context)
    {
        if (context.Profile != null && context.Profile.randomizeWater)
        {
            minCurrentSpeed = context.Profile.currentSpeed.min;
            maxCurrentSpeed = context.Profile.currentSpeed.max;
            meanCurrentVelocity = context.Profile.meanCurrent;
            maxAngleFromMeanDegrees = context.Profile.maxAngleFromMeanDegrees;
            includeVerticalForce = context.Profile.allowVerticalCurrent;
            directionSampling = meanCurrentVelocity.sqrMagnitude > 1e-8f
                ? DirectionSamplingMode.AroundMean
                : (includeVerticalForce ? DirectionSamplingMode.Full3D : DirectionSamplingMode.Horizontal);

            float speed = Mathf.Max(0f, context.Range(context.Profile.currentSpeed));
            CurrentVelocity = context.CurrentDirection() * speed;
            if (!includeVerticalForce)
            {
                CurrentVelocity = new Vector3(CurrentVelocity.x, 0f, CurrentVelocity.z);
            }

            if (logRandomizedCurrent)
            {
                Debug.Log($"[{nameof(WaterCurrentDomainRandomizer)}] current={CurrentVelocity:F3} m/s, speed={CurrentVelocity.magnitude:F3} m/s", this);
            }

            return;
        }

        RandomizeCurrent();
    }

    void ApplyCurrentForces()
    {
        RemoveMissingTargets();

        for (int i = 0; i < targetRigidbodies.Count; i++)
        {
            Rigidbody rb = targetRigidbodies[i];
            Vector3 targetVelocity = includeVerticalForce ? CurrentVelocity : new Vector3(CurrentVelocity.x, rb.linearVelocity.y, CurrentVelocity.z);
            Vector3 relativeVelocity = targetVelocity - rb.linearVelocity;

            if (!includeVerticalForce)
            {
                relativeVelocity.y = 0f;
            }

            Vector3 acceleration = relativeVelocity * accelerationGain;
            if (quadraticDragGain > 0f)
            {
                acceleration += relativeVelocity * relativeVelocity.magnitude * quadraticDragGain;
            }

            if (maxAcceleration > 0f)
            {
                acceleration = Vector3.ClampMagnitude(acceleration, maxAcceleration);
            }

            Vector3 force = acceleration * rb.mass;
            LastAppliedAcceleration = acceleration;
            LastAppliedForce = force;

            if (wakeTargetsBeforeApplyingForce)
            {
                rb.WakeUp();
            }

            switch (forceMode)
            {
                case CurrentForceMode.Acceleration:
                    rb.AddForce(acceleration, ForceMode.Acceleration);
                    break;
                case CurrentForceMode.VelocityChange:
                    rb.AddForce(acceleration * Time.fixedDeltaTime, ForceMode.VelocityChange);
                    break;
                default:
                    rb.AddForce(force, ForceMode.Force);
                    break;
            }
        }

        if (logAppliedForce && Time.time >= _nextAppliedForceLogTime)
        {
            Debug.Log(
                $"[{nameof(WaterCurrentDomainRandomizer)}] targets={targetRigidbodies.Count}, "
                + $"current={CurrentVelocity:F3}, "
                + $"last_acc={LastAppliedAcceleration:F3} m/s^2, "
                + $"last_force={LastAppliedForce:F3} N",
                this
            );
            _nextAppliedForceLogTime = Time.time + appliedForceLogIntervalSeconds;
        }
    }

    Vector3 SampleDirection()
    {
        switch (directionSampling)
        {
            case DirectionSamplingMode.Full3D:
                return SampleFull3DDirection();
            case DirectionSamplingMode.AroundMean:
                return SampleDirectionAroundMean();
            default:
                return SampleHorizontalDirection();
        }
    }

    Vector3 SampleHorizontalDirection()
    {
        float angle = Random.Range(0f, Mathf.PI * 2f);
        return new Vector3(Mathf.Cos(angle), 0f, Mathf.Sin(angle)).normalized;
    }

    Vector3 SampleFull3DDirection()
    {
        Vector3 horizontal = SampleHorizontalDirection() * Random.Range(0f, 1f);
        float vertical = Random.Range(verticalVelocityRange.x, verticalVelocityRange.y);
        Vector3 direction = new Vector3(horizontal.x, vertical, horizontal.z);
        return direction.sqrMagnitude > 1e-6f ? direction.normalized : Vector3.forward;
    }

    Vector3 SampleDirectionAroundMean()
    {
        Vector3 baseDirection = meanCurrentVelocity.sqrMagnitude > 1e-6f
            ? meanCurrentVelocity.normalized
            : Vector3.forward;

        Vector3 axis = includeVerticalForce ? Random.onUnitSphere : Vector3.up;
        Quaternion perturbation = Quaternion.AngleAxis(Random.Range(-maxAngleFromMeanDegrees, maxAngleFromMeanDegrees), axis);
        Vector3 direction = perturbation * baseDirection;

        if (!includeVerticalForce)
        {
            direction.y = 0f;
        }

        return direction.sqrMagnitude > 1e-6f ? direction.normalized : Vector3.forward;
    }

    void RemoveMissingTargets()
    {
        for (int i = targetRigidbodies.Count - 1; i >= 0; i--)
        {
            if (targetRigidbodies[i] == null)
            {
                targetRigidbodies.RemoveAt(i);
            }
        }
    }
}
