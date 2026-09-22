using FinsSim.Actuators;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

[DisallowMultipleComponent]
[RequireComponent(typeof(Rigidbody))]
public class ControlForVelocity : Agent
{
    Rigidbody rigidBody;
    ThrusterController thrusterController;
    Transform referenceTransform;

    readonly Thruster[] orderedThrusters = new Thruster[FinsROVAgentRuntime.DefaultThrusterOrder.Length];
    readonly float[] currentActions = new float[FinsROVAgentRuntime.DefaultThrusterOrder.Length];

    [Header("Reference Frame")]
    [Tooltip("优先使用这个参考点。为空时自动查找名为 MarkPosition 的子物体；如果不存在，则退回模型自身 Transform。")]
    public Transform selfTransform;

    [Header("Thruster Control")]
    [SerializeField] ThrusterController thrusterControllerOverride;
    [SerializeField] ThrusterCommandMode thrusterCommandMode = ThrusterCommandMode.ScaledForceRequest;
    [ShowWhenThrusterCommandMode(ThrusterCommandMode.ScaledForceRequest)]
    [SerializeField] float actionForceScaleN = 7f;
    [SerializeField] bool autoResolveThrustersFromChildren = true;
    [SerializeField] bool logThrusterResolution = true;
    [SerializeField] float heuristicYawMixScale = 1.0f;
    [SerializeField] float heuristicAuxMixScale = 1.0f;

    [Header("目标速度范围 (线性)")]
    public float maxLinearVelocityX = 0.5641f;
    public float maxLinearVelocityY = 0.4965f;
    public float maxLinearVelocityZ = 0.887f;

    [Header("目标速度范围 (角速度)")]
    public float maxAngularVelocityX = 0.6163f;
    public float maxAngularVelocityY = 0.6f;
    public float maxAngularVelocityZ = 0.6621f;

    [Header("初始速度范围")]
    [Tooltip("初始速度范围 = 目标速度范围 * 该倍率")]
    public float initialVelocityMultiplier = 1f;

    [Header("域随机化")]
    [Tooltip("可选：每个 episode 开始时重新采样水流。")]
    public WaterCurrentDomainRandomizer waterCurrentRandomizer;

    Vector3 targetLinearVelocity;
    Vector3 targetAngularVelocity;
    VelocityDebugDisplay debugDisplay;

    Transform ReferenceTransform
    {
        get
        {
            if (referenceTransform == null)
            {
                referenceTransform = FinsROVAgentRuntime.ResolveReferenceTransform(transform, selfTransform);
            }

            return referenceTransform;
        }
    }

    void Start()
    {
        rigidBody = GetComponent<Rigidbody>();
        thrusterController = thrusterControllerOverride != null
            ? thrusterControllerOverride
            : GetComponent<ThrusterController>();

        debugDisplay = GetComponent<VelocityDebugDisplay>();
        if (debugDisplay == null)
        {
            debugDisplay = FindObjectOfType<VelocityDebugDisplay>();
        }

        if (waterCurrentRandomizer == null)
        {
            waterCurrentRandomizer = FindObjectOfType<WaterCurrentDomainRandomizer>();
        }

        ResolveRuntimeReferences();
    }

    void UpdateDebugDisplay()
    {
        if (debugDisplay != null)
        {
            debugDisplay.targetLinearVelocity = targetLinearVelocity;
            debugDisplay.targetAngularVelocity = targetAngularVelocity;
            debugDisplay.currentLinearVelocity = GetLocalLinearVelocity(rigidBody.linearVelocity);
            debugDisplay.currentAngularVelocity = GetLocalAngularVelocity(rigidBody.angularVelocity);
            debugDisplay.currentReward = GetCumulativeReward();
            debugDisplay.velocitySpace = VelocityDebugDisplay.VelocitySpace.Local;
            debugDisplay.targetTransform = ReferenceTransform;
        }
    }

    static float SafeDivide(float value, float denominator)
    {
        return Mathf.Approximately(denominator, 0f) ? 0f : value / denominator;
    }

    Vector3 NormalizeLinearVelocity(Vector3 velocity)
    {
        return new Vector3(
            SafeDivide(velocity.x, maxLinearVelocityX),
            SafeDivide(velocity.y, maxLinearVelocityY),
            SafeDivide(velocity.z, maxLinearVelocityZ)
        );
    }

    Vector3 NormalizeAngularVelocity(Vector3 velocity)
    {
        return new Vector3(
            SafeDivide(velocity.x, maxAngularVelocityX),
            SafeDivide(velocity.y, maxAngularVelocityY),
            SafeDivide(velocity.z, maxAngularVelocityZ)
        );
    }

    Vector3 GetLocalLinearVelocity(Vector3 worldLinearVelocity)
    {
        return ReferenceTransform.InverseTransformDirection(worldLinearVelocity);
    }

    Vector3 GetLocalAngularVelocity(Vector3 worldAngularVelocity)
    {
        return ReferenceTransform.InverseTransformDirection(worldAngularVelocity);
    }

    Vector3 GetWorldLinearVelocity(Vector3 localLinearVelocity)
    {
        return ReferenceTransform.TransformDirection(localLinearVelocity);
    }

    Vector3 GetWorldAngularVelocity(Vector3 localAngularVelocity)
    {
        return ReferenceTransform.TransformDirection(localAngularVelocity);
    }

    public override void OnEpisodeBegin()
    {
        ResolveRuntimeReferences();
        FinsROVAgentRuntime.RandomizeEpisodeIfPresent(this);

        if (waterCurrentRandomizer != null)
        {
            waterCurrentRandomizer.RandomizeCurrent();
        }

        transform.position = new Vector3(
            Random.value * 2f,
            Random.value - 5f,
            Random.value * 2f
        );
        transform.rotation = Random.rotationUniform;

        FinsROVAgentRuntime.ZeroThrusters(orderedThrusters, thrusterCommandMode, actionForceScaleN);

        targetLinearVelocity = new Vector3(
            Random.Range(-maxLinearVelocityX, maxLinearVelocityX),
            Random.Range(-maxLinearVelocityY, maxLinearVelocityY),
            Random.Range(-maxLinearVelocityZ, maxLinearVelocityZ)
        );
        targetAngularVelocity = new Vector3(
            Random.Range(-maxAngularVelocityX, maxAngularVelocityX),
            Random.Range(-maxAngularVelocityY, maxAngularVelocityY),
            Random.Range(-maxAngularVelocityZ, maxAngularVelocityZ)
        );

        float initialVelocityScale = Mathf.Max(0f, initialVelocityMultiplier);
        Vector3 initialLocalLinearVelocity = new Vector3(
            Random.Range(-maxLinearVelocityX * initialVelocityScale, maxLinearVelocityX * initialVelocityScale),
            Random.Range(-maxLinearVelocityY * initialVelocityScale, maxLinearVelocityY * initialVelocityScale),
            Random.Range(-maxLinearVelocityZ * initialVelocityScale, maxLinearVelocityZ * initialVelocityScale)
        );
        Vector3 initialLocalAngularVelocity = new Vector3(
            Random.Range(-maxAngularVelocityX * initialVelocityScale, maxAngularVelocityX * initialVelocityScale),
            Random.Range(-maxAngularVelocityY * initialVelocityScale, maxAngularVelocityY * initialVelocityScale),
            Random.Range(-maxAngularVelocityZ * initialVelocityScale, maxAngularVelocityZ * initialVelocityScale)
        );

        rigidBody.linearVelocity = GetWorldLinearVelocity(initialLocalLinearVelocity);
        rigidBody.angularVelocity = GetWorldAngularVelocity(initialLocalAngularVelocity);
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        Vector3 localCurrentLinearVelocity = GetLocalLinearVelocity(rigidBody.linearVelocity);
        Vector3 localCurrentAngularVelocity = GetLocalAngularVelocity(rigidBody.angularVelocity);

        sensor.AddObservation(NormalizeLinearVelocity(targetLinearVelocity));
        sensor.AddObservation(NormalizeAngularVelocity(targetAngularVelocity));
        sensor.AddObservation(NormalizeLinearVelocity(localCurrentLinearVelocity));
        sensor.AddObservation(NormalizeAngularVelocity(localCurrentAngularVelocity));
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        ResolveRuntimeReferences();

        int actionCount = Mathf.Min(currentActions.Length, actionBuffers.ContinuousActions.Length);
        for (int i = 0; i < currentActions.Length; i++)
        {
            currentActions[i] = i < actionCount
                ? Mathf.Clamp(actionBuffers.ContinuousActions[i], -1f, 1f)
                : 0f;
        }

        FinsROVAgentRuntime.ApplyThrusterActions(orderedThrusters, currentActions, thrusterCommandMode, actionForceScaleN);

        Vector3 currentLinearVelocity = GetLocalLinearVelocity(rigidBody.linearVelocity);
        Vector3 currentAngularVelocity = GetLocalAngularVelocity(rigidBody.angularVelocity);

        Vector3 normalizedLinearDiff = NormalizeLinearVelocity(currentLinearVelocity - targetLinearVelocity);
        Vector3 normalizedAngularDiff = NormalizeAngularVelocity(currentAngularVelocity - targetAngularVelocity);

        float linearError01 = Mathf.Clamp01(normalizedLinearDiff.magnitude / Mathf.Sqrt(3f));
        float angularError01 = Mathf.Clamp01(normalizedAngularDiff.magnitude / Mathf.Sqrt(3f));

        float linearScore = 1f - linearError01;
        float angularScore = 1f - angularError01;

        float linearSpeedPenalty = Mathf.Clamp01(NormalizeLinearVelocity(currentLinearVelocity).magnitude / Mathf.Sqrt(3f));
        float angularSpeedPenalty = Mathf.Clamp01(NormalizeAngularVelocity(currentAngularVelocity).magnitude / Mathf.Sqrt(3f));
        float timePenalty = 0.02f;

        float reward01 =
            0.5f * linearScore +
            0.4f * angularScore -
            timePenalty -
            0.05f * linearSpeedPenalty -
            0.05f * angularSpeedPenalty;

        float reward = Mathf.Clamp(reward01 * 2f - 1f, -1f, 1f);
        SetReward(reward);

        UpdateDebugDisplay();

        if (linearError01 < 0.1f && angularError01 < 0.1f)
        {
            AddReward(0.5f);
            EndEpisode();
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        FinsROVAgentRuntime.WriteHeuristicActionsFromManualInput(
            actionsOut.ContinuousActions,
            currentActions,
            heuristicYawMixScale,
            heuristicAuxMixScale);
    }

    void ResolveRuntimeReferences()
    {
        referenceTransform = FinsROVAgentRuntime.ResolveReferenceTransform(transform, selfTransform);

        if (thrusterController == null)
        {
            thrusterController = thrusterControllerOverride != null
                ? thrusterControllerOverride
                : GetComponent<ThrusterController>();
        }

        if (FinsROVAgentRuntime.TryResolveOrderedThrusters(
            this,
            thrusterController,
            autoResolveThrustersFromChildren,
            orderedThrusters,
            out string statusMessage))
        {
            FinsROVAgentRuntime.EnsureThrusterControllerOrder(thrusterController, orderedThrusters);
            if (logThrusterResolution)
            {
                Debug.Log($"[{nameof(ControlForVelocity)}] {statusMessage}", this);
                logThrusterResolution = false;
            }
        }
        else
        {
            Debug.LogError($"[{nameof(ControlForVelocity)}] {statusMessage}", this);
        }
    }
}
