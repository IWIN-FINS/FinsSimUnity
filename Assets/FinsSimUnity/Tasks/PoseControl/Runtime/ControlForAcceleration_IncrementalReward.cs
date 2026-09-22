using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;
using NWH.DWP2.ShipController;

public class ControlForAcceleration_IncrementalReward : Agent
{
    Rigidbody rigidBody;
    AdvancedShipController advancedShipController;
    List<Engine> engines;
    readonly Dictionary<string, Engine> engineMap = new Dictionary<string, Engine>();

    [Tooltip("调试时可绑定潜器中心点")]
    public Transform selfTransform;

    [Header("目标加速度范围 (线加速度)")]
    [SerializeField] float maxLinearAccelerationX = 1.2f;
    [SerializeField] float maxLinearAccelerationY = 1.2f;
    [SerializeField] float maxLinearAccelerationZ = 1.5f;

    [Header("目标加速度范围 (角加速度)")]
    [SerializeField] float maxAngularAccelerationX = 1.5f;
    [SerializeField] float maxAngularAccelerationY = 1.5f;
    [SerializeField] float maxAngularAccelerationZ = 1.5f;

    [Header("初始速度范围")]
    [SerializeField] float initialLinearVelocityRange = 0.5f;
    [SerializeField] float initialAngularVelocityRange = 0.4f;

    [Header("奖励参数")]
    [SerializeField] float perStepPenalty = -0.002f;
    [SerializeField] float linearImprovementRewardScale = 0.9f;
    [SerializeField] float angularImprovementRewardScale = 0.7f;
    [SerializeField] float linearTrackingRewardScale = 0.08f;
    [SerializeField] float angularTrackingRewardScale = 0.06f;
    [SerializeField] float velocityDampingRewardScale = 0.03f;
    [SerializeField] float actionEnergyPenaltyScale = 0.0015f;
    [SerializeField] float actionChangePenaltyScale = 0.001f;
    [SerializeField] float accelerationSpikePenaltyScale = 0.025f;
    [SerializeField] float successReward = 1f;

    [Header("终止条件")]
    [SerializeField] float linearSuccessError01 = 0.1f;
    [SerializeField] float angularSuccessError01 = 0.1f;
    [SerializeField] int requiredStableSteps = 8;
    [SerializeField] float maxDistanceFromOrigin = 10f;
    [SerializeField] float maxHeightAboveSurface = 0.3f;
    [SerializeField] float surfaceY = 0f;
    [SerializeField] float resetDepthMin = -5f;
    [SerializeField] float resetDepthMax = -4f;

    Vector3 targetLinearAcceleration;
    Vector3 targetAngularAcceleration;
    Vector3 currentLinearAcceleration;
    Vector3 currentAngularAcceleration;
    Vector3 previousLocalLinearVelocity;
    Vector3 previousLocalAngularVelocity;
    Vector3 resetPosition;

    float previousLinearError01;
    float previousAngularError01;
    int stableTrackingSteps;
    float lastStepReward;

    readonly float[] currentActions = new float[8];
    readonly float[] previousActions = new float[8];

    public float LastStepReward => lastStepReward;
    public Vector3 TargetLinearAcceleration => targetLinearAcceleration;
    public Vector3 TargetAngularAcceleration => targetAngularAcceleration;
    public Vector3 CurrentLinearAcceleration => currentLinearAcceleration;
    public Vector3 CurrentAngularAcceleration => currentAngularAcceleration;
    public Vector3 CurrentLinearVelocity => GetLocalLinearVelocity(rigidBody.linearVelocity);
    public Vector3 CurrentAngularVelocity => GetLocalAngularVelocity(rigidBody.angularVelocity);
    public float CurrentLinearError01 => previousLinearError01;
    public float CurrentAngularError01 => previousAngularError01;
    public int StableTrackingSteps => stableTrackingSteps;
    public int RequiredStableSteps => requiredStableSteps;
    public float LinearSuccessError01 => linearSuccessError01;
    public float AngularSuccessError01 => angularSuccessError01;

    void Start()
    {
        rigidBody = GetComponent<Rigidbody>();
        advancedShipController = GetComponent<AdvancedShipController>();

        engines = advancedShipController.engines;
        engineMap.Clear();
        foreach (Engine engine in engines)
        {
            engine.useExternalThrottleInput = true;
            if (!engineMap.ContainsKey(engine.name))
            {
                engineMap.Add(engine.name, engine);
            }
        }
    }

    void FixedUpdate()
    {
        Vector3 currentLocalLinearVelocity = GetLocalLinearVelocity(rigidBody.linearVelocity);
        Vector3 currentLocalAngularVelocity = GetLocalAngularVelocity(rigidBody.angularVelocity);
        float dt = Mathf.Max(Time.fixedDeltaTime, 1e-4f);

        currentLinearAcceleration = (currentLocalLinearVelocity - previousLocalLinearVelocity) / dt;
        currentAngularAcceleration = (currentLocalAngularVelocity - previousLocalAngularVelocity) / dt;

        previousLocalLinearVelocity = currentLocalLinearVelocity;
        previousLocalAngularVelocity = currentLocalAngularVelocity;
    }

    public override void OnEpisodeBegin()
    {
        FinsROVAgentRuntime.RandomizeEpisodeIfPresent(this);

        resetPosition = new Vector3(
            Random.Range(0f, 2f),
            Random.Range(resetDepthMin, resetDepthMax),
            Random.Range(0f, 2f)
        );
        transform.position = resetPosition;
        transform.rotation = Random.rotationUniform;

        Vector3 initialLocalLinearVelocity = new Vector3(
            Random.Range(-initialLinearVelocityRange, initialLinearVelocityRange),
            Random.Range(-initialLinearVelocityRange, initialLinearVelocityRange),
            Random.Range(-initialLinearVelocityRange, initialLinearVelocityRange)
        );
        Vector3 initialLocalAngularVelocity = new Vector3(
            Random.Range(-initialAngularVelocityRange, initialAngularVelocityRange),
            Random.Range(-initialAngularVelocityRange, initialAngularVelocityRange),
            Random.Range(-initialAngularVelocityRange, initialAngularVelocityRange)
        );

        rigidBody.linearVelocity = GetWorldLinearVelocity(initialLocalLinearVelocity);
        rigidBody.angularVelocity = GetWorldAngularVelocity(initialLocalAngularVelocity);

        targetLinearAcceleration = new Vector3(
            Random.Range(-maxLinearAccelerationX, maxLinearAccelerationX),
            Random.Range(-maxLinearAccelerationY, maxLinearAccelerationY),
            Random.Range(-maxLinearAccelerationZ, maxLinearAccelerationZ)
        );
        targetAngularAcceleration = new Vector3(
            Random.Range(-maxAngularAccelerationX, maxAngularAccelerationX),
            Random.Range(-maxAngularAccelerationY, maxAngularAccelerationY),
            Random.Range(-maxAngularAccelerationZ, maxAngularAccelerationZ)
        );

        previousLocalLinearVelocity = initialLocalLinearVelocity;
        previousLocalAngularVelocity = initialLocalAngularVelocity;
        currentLinearAcceleration = Vector3.zero;
        currentAngularAcceleration = Vector3.zero;
        previousLinearError01 = GetLinearError01();
        previousAngularError01 = GetAngularError01();
        stableTrackingSteps = 0;
        lastStepReward = 0f;
        System.Array.Clear(currentActions, 0, currentActions.Length);
        System.Array.Clear(previousActions, 0, previousActions.Length);
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        Vector3 currentLocalLinearVelocity = GetLocalLinearVelocity(rigidBody.linearVelocity);
        Vector3 currentLocalAngularVelocity = GetLocalAngularVelocity(rigidBody.angularVelocity);

        sensor.AddObservation(NormalizeLinearAcceleration(targetLinearAcceleration));
        sensor.AddObservation(NormalizeAngularAcceleration(targetAngularAcceleration));
        sensor.AddObservation(NormalizeLinearAcceleration(currentLinearAcceleration));
        sensor.AddObservation(NormalizeAngularAcceleration(currentAngularAcceleration));
        sensor.AddObservation(NormalizeLinearVelocity(currentLocalLinearVelocity));
        sensor.AddObservation(NormalizeAngularVelocity(currentLocalAngularVelocity));
        sensor.AddObservation(previousLinearError01);
        sensor.AddObservation(previousAngularError01);
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        ApplyActions(actionBuffers.ContinuousActions);

        float linearError01 = GetLinearError01();
        float angularError01 = GetAngularError01();

        float linearImprovement = previousLinearError01 - linearError01;
        float angularImprovement = previousAngularError01 - angularError01;

        Vector3 currentLocalLinearVelocity = GetLocalLinearVelocity(rigidBody.linearVelocity);
        Vector3 currentLocalAngularVelocity = GetLocalAngularVelocity(rigidBody.angularVelocity);

        float velocityDampingReward =
            0.5f * (1f - Mathf.Clamp01(NormalizeLinearVelocity(currentLocalLinearVelocity).magnitude / Mathf.Sqrt(3f))) +
            0.5f * (1f - Mathf.Clamp01(NormalizeAngularVelocity(currentLocalAngularVelocity).magnitude / Mathf.Sqrt(3f)));

        float actionEnergy = 0f;
        float actionDelta = 0f;
        for (int i = 0; i < currentActions.Length; i++)
        {
            actionEnergy += Mathf.Abs(currentActions[i]);
            actionDelta += Mathf.Abs(currentActions[i] - previousActions[i]);
            previousActions[i] = currentActions[i];
        }

        actionEnergy /= currentActions.Length;
        actionDelta /= currentActions.Length;

        float accelerationSpikePenalty =
            Mathf.Clamp01(NormalizeLinearAcceleration(currentLinearAcceleration).magnitude / Mathf.Sqrt(3f)) +
            Mathf.Clamp01(NormalizeAngularAcceleration(currentAngularAcceleration).magnitude / Mathf.Sqrt(3f));

        float reward =
            perStepPenalty +
            linearImprovementRewardScale * linearImprovement +
            angularImprovementRewardScale * angularImprovement +
            linearTrackingRewardScale * (1f - linearError01) +
            angularTrackingRewardScale * (1f - angularError01) +
            velocityDampingRewardScale * velocityDampingReward -
            actionEnergyPenaltyScale * actionEnergy -
            actionChangePenaltyScale * actionDelta -
            accelerationSpikePenaltyScale * accelerationSpikePenalty * 0.5f;

        AddReward(reward);
        lastStepReward = reward;

        previousLinearError01 = linearError01;
        previousAngularError01 = angularError01;

        if (linearError01 < linearSuccessError01 && angularError01 < angularSuccessError01)
        {
            stableTrackingSteps++;
            AddReward(0.02f);
            lastStepReward += 0.02f;
        }
        else
        {
            stableTrackingSteps = 0;
        }

        if (stableTrackingSteps >= requiredStableSteps)
        {
            AddReward(successReward);
            lastStepReward += successReward;
            EndEpisode();
            return;
        }

        bool outOfBounds =
            Vector3.Distance(transform.position, resetPosition) > maxDistanceFromOrigin ||
            transform.position.y > surfaceY + maxHeightAboveSurface;

        if (outOfBounds)
        {
            AddReward(-1f);
            lastStepReward += -1f;
            EndEpisode();
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        FinsROVAgentRuntime.WriteHeuristicActionsFromManualInput(
            actionsOut.ContinuousActions,
            currentActions);
    }

    void ApplyActions(ActionSegment<float> actions)
    {
        ResetAllEngines();

        for (int i = 0; i < currentActions.Length; i++)
        {
            currentActions[i] = Mathf.Clamp(actions[i], -1f, 1f);
        }

        ApplyEngineInput("Vertical1", currentActions[0]);
        ApplyEngineInput("Vertical2", currentActions[1]);
        ApplyEngineInput("Vertical3", currentActions[2]);
        ApplyEngineInput("Vertical4", currentActions[3]);
        ApplyEngineInput("Horizontal1", currentActions[4]);
        ApplyEngineInput("Horizontal2", currentActions[5]);
        ApplyEngineInput("Horizontal3", currentActions[6]);
        ApplyEngineInput("Horizontal4", currentActions[7]);
    }

    float GetLinearError01()
    {
        Vector3 diff = NormalizeLinearAcceleration(currentLinearAcceleration - targetLinearAcceleration);
        return Mathf.Clamp01(diff.magnitude / Mathf.Sqrt(3f));
    }

    float GetAngularError01()
    {
        Vector3 diff = NormalizeAngularAcceleration(currentAngularAcceleration - targetAngularAcceleration);
        return Mathf.Clamp01(diff.magnitude / Mathf.Sqrt(3f));
    }

    Vector3 NormalizeLinearAcceleration(Vector3 value)
    {
        return new Vector3(
            SafeDivide(value.x, maxLinearAccelerationX),
            SafeDivide(value.y, maxLinearAccelerationY),
            SafeDivide(value.z, maxLinearAccelerationZ)
        );
    }

    Vector3 NormalizeAngularAcceleration(Vector3 value)
    {
        return new Vector3(
            SafeDivide(value.x, maxAngularAccelerationX),
            SafeDivide(value.y, maxAngularAccelerationY),
            SafeDivide(value.z, maxAngularAccelerationZ)
        );
    }

    Vector3 NormalizeLinearVelocity(Vector3 value)
    {
        return value / Mathf.Max(initialLinearVelocityRange, 0.01f);
    }

    Vector3 NormalizeAngularVelocity(Vector3 value)
    {
        return value / Mathf.Max(initialAngularVelocityRange, 0.01f);
    }

    static float SafeDivide(float value, float denominator)
    {
        return Mathf.Approximately(denominator, 0f) ? 0f : value / denominator;
    }

    Vector3 GetLocalLinearVelocity(Vector3 worldLinearVelocity)
    {
        return transform.InverseTransformDirection(worldLinearVelocity);
    }

    Vector3 GetLocalAngularVelocity(Vector3 worldAngularVelocity)
    {
        return transform.InverseTransformDirection(worldAngularVelocity);
    }

    Vector3 GetWorldLinearVelocity(Vector3 localLinearVelocity)
    {
        return transform.TransformDirection(localLinearVelocity);
    }

    Vector3 GetWorldAngularVelocity(Vector3 localAngularVelocity)
    {
        return transform.TransformDirection(localAngularVelocity);
    }

    void ResetAllEngines()
    {
        foreach (Engine engine in engines)
        {
            engine.externalThrottleInput = 0f;
        }
    }

    void ApplyEngineInput(string engineName, float input)
    {
        if (engineMap.TryGetValue(engineName, out Engine engine))
        {
            engine.externalThrottleInput = input;
        }
    }
}
