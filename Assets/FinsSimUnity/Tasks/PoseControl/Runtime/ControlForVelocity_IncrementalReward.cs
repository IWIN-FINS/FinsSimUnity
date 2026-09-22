using FinsSim.Actuators;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
using UnityEngine;

[DisallowMultipleComponent]
[RequireComponent(typeof(Rigidbody))]
public class ControlForVelocity_IncrementalReward : Agent
{
    Rigidbody rigidBody;
    ThrusterController thrusterController;
    Transform referenceTransform;

    readonly Thruster[] orderedThrusters = new Thruster[FinsROVAgentRuntime.DefaultThrusterOrder.Length];
    readonly float[] currentActions = new float[FinsROVAgentRuntime.DefaultThrusterOrder.Length];
    readonly float[] previousActions = new float[FinsROVAgentRuntime.DefaultThrusterOrder.Length];

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
    [SerializeField] bool autoRequestDecisionWhenNoDecisionRequester = true;

    [Header("目标速度范围 (线性)")]
    [SerializeField] float maxLinearVelocityX = 0.3f;
    [SerializeField] float maxLinearVelocityY = 0.3f;
    [SerializeField] float maxLinearVelocityZ = 0.3f;

    [Header("目标速度范围 (角速度)")]
    [SerializeField] float maxAngularVelocityX = 0f;
    [SerializeField] float maxAngularVelocityY = 0f;
    [SerializeField] float maxAngularVelocityZ = 0f;

    [Header("初始速度范围")]
    [SerializeField] float initialVelocityMultiplier = 1f;

    [Header("奖励参数")]
    [SerializeField] float perStepPenalty = -0.002f;
    [SerializeField] float linearImprovementRewardScale = 0.8f;
    [SerializeField] float angularImprovementRewardScale = 0.6f;
    [SerializeField] float linearTrackingRewardScale = 0.08f;
    [SerializeField] float angularTrackingRewardScale = 0.06f;
    [SerializeField] float targetDirectionRewardScale = 0.05f;
    [SerializeField] float actionEnergyPenaltyScale = 0.0015f;
    [SerializeField] float actionChangePenaltyScale = 0.001f;
    [SerializeField] float overspeedPenaltyScale = 0.03f;
    [SerializeField] float successReward = 1f;

    [Header("终止条件")]
    [SerializeField] float linearSuccessError01 = 0.08f;
    [SerializeField] float angularSuccessError01 = 0.08f;
    [SerializeField] int requiredStableSteps = 8;
    [SerializeField] float maxDistanceFromOrigin = 10f;
    [SerializeField] float maxHeightAboveSurface = 0.3f;
    [SerializeField] float surfaceY = 0f;
    [SerializeField] float resetDepthMin = -5f;
    [SerializeField] float resetDepthMax = -4f;

    Vector3 targetLinearVelocity;
    Vector3 targetAngularVelocity;
    Vector3 resetPosition;
    Vector3 resetReferencePosition;

    float previousLinearError01;
    float previousAngularError01;
    int stableTrackingSteps;

    VelocityDebugDisplay debugDisplay;
    float lastStepReward;
    float currentLinearError01;
    float currentAngularError01;
    bool hasDecisionRequester;

    public float LastStepReward => lastStepReward;
    public Vector3 TargetLinearVelocity => targetLinearVelocity;
    public Vector3 TargetAngularVelocity => targetAngularVelocity;
    public Vector3 CurrentLinearVelocity => GetLocalLinearVelocity();
    public Vector3 CurrentAngularVelocity => GetLocalAngularVelocity();
    public float CurrentLinearError01 => currentLinearError01;
    public float CurrentAngularError01 => currentAngularError01;
    public int StableTrackingSteps => stableTrackingSteps;
    public int RequiredStableSteps => requiredStableSteps;
    public float LinearSuccessError01 => linearSuccessError01;
    public float AngularSuccessError01 => angularSuccessError01;

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

    Rigidbody RigidBody
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

    void Start()
    {
        rigidBody = RigidBody;
        thrusterController = thrusterControllerOverride != null
            ? thrusterControllerOverride
            : GetComponent<ThrusterController>();
        hasDecisionRequester = GetComponent("DecisionRequester") != null;

        debugDisplay = GetComponent<VelocityDebugDisplay>();
        if (debugDisplay == null)
        {
            debugDisplay = FindObjectOfType<VelocityDebugDisplay>();
        }

        ResolveRuntimeReferences();
    }

    void FixedUpdate()
    {
        if (autoRequestDecisionWhenNoDecisionRequester && !hasDecisionRequester)
        {
            RequestDecision();
        }
    }

    public override void OnEpisodeBegin()
    {
        ResolveRuntimeReferences();
        FinsROVAgentRuntime.RandomizeEpisodeIfPresent(this);

        resetPosition = new Vector3(
            Random.Range(0f, 2f),
            Random.Range(resetDepthMin, resetDepthMax),
            Random.Range(0f, 2f)
        );
        transform.position = resetPosition;
        transform.rotation = Random.rotationUniform;
        resetReferencePosition = ReferenceTransform.position;

        RigidBody.linearVelocity = Vector3.zero;
        RigidBody.angularVelocity = Vector3.zero;
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

        RigidBody.linearVelocity = FinsROVAgentRuntime.GetWorldLinearVelocity(ReferenceTransform, initialLocalLinearVelocity);
        RigidBody.angularVelocity = FinsROVAgentRuntime.GetWorldAngularVelocity(ReferenceTransform, initialLocalAngularVelocity);

        previousLinearError01 = GetLinearError01();
        previousAngularError01 = GetAngularError01();
        currentLinearError01 = previousLinearError01;
        currentAngularError01 = previousAngularError01;
        stableTrackingSteps = 0;
        lastStepReward = 0f;
        System.Array.Clear(currentActions, 0, currentActions.Length);
        System.Array.Clear(previousActions, 0, previousActions.Length);
        UpdateDebugDisplay();
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        Vector3 currentLinearVelocity = GetLocalLinearVelocity();
        Vector3 currentAngularVelocity = GetLocalAngularVelocity();

        sensor.AddObservation(NormalizeLinearVector(targetLinearVelocity));
        sensor.AddObservation(NormalizeAngularVector(targetAngularVelocity));
        sensor.AddObservation(NormalizeLinearVector(currentLinearVelocity));
        sensor.AddObservation(NormalizeAngularVector(currentAngularVelocity));
        sensor.AddObservation(NormalizeLinearVector(targetLinearVelocity - currentLinearVelocity));
        sensor.AddObservation(NormalizeAngularVector(targetAngularVelocity - currentAngularVelocity));
        sensor.AddObservation(previousLinearError01);
        sensor.AddObservation(previousAngularError01);
    }

    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        ResolveRuntimeReferences();

        ApplyActions(actionBuffers.ContinuousActions);

        float linearError01 = GetLinearError01();
        float angularError01 = GetAngularError01();

        float linearImprovement = previousLinearError01 - linearError01;
        float angularImprovement = previousAngularError01 - angularError01;

        Vector3 currentLinearVelocity = GetLocalLinearVelocity();
        Vector3 normalizedTargetLinear = NormalizeLinearVector(targetLinearVelocity);
        Vector3 normalizedCurrentLinear = NormalizeLinearVector(currentLinearVelocity);

        float targetDirectionAlignment = 0f;
        if (normalizedTargetLinear.sqrMagnitude > 1e-6f && normalizedCurrentLinear.sqrMagnitude > 1e-6f)
        {
            targetDirectionAlignment = Vector3.Dot(
                normalizedTargetLinear.normalized,
                normalizedCurrentLinear.normalized
            );
        }

        float trackingScore =
            linearTrackingRewardScale * (1f - linearError01) +
            angularTrackingRewardScale * (1f - angularError01);

        float overspeedPenalty = Mathf.Max(
            0f,
            NormalizeLinearVector(currentLinearVelocity).magnitude - NormalizeLinearVector(targetLinearVelocity).magnitude
        );

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

        float reward =
            perStepPenalty +
            linearImprovementRewardScale * linearImprovement +
            angularImprovementRewardScale * angularImprovement +
            trackingScore +
            targetDirectionRewardScale * targetDirectionAlignment -
            overspeedPenaltyScale * overspeedPenalty -
            actionEnergyPenaltyScale * actionEnergy -
            actionChangePenaltyScale * actionDelta;

        AddReward(reward);
        lastStepReward = reward;
        currentLinearError01 = linearError01;
        currentAngularError01 = angularError01;

        previousLinearError01 = linearError01;
        previousAngularError01 = angularError01;
        UpdateDebugDisplay();

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
            Vector3.Distance(ReferenceTransform.position, resetReferencePosition) > maxDistanceFromOrigin ||
            ReferenceTransform.position.y > surfaceY + maxHeightAboveSurface;

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
            currentActions,
            heuristicYawMixScale,
            heuristicAuxMixScale);
    }

    void ApplyActions(ActionSegment<float> actions)
    {
        int actionCount = Mathf.Min(currentActions.Length, actions.Length);
        for (int i = 0; i < currentActions.Length; i++)
        {
            currentActions[i] = i < actionCount ? Mathf.Clamp(actions[i], -1f, 1f) : 0f;
        }

        FinsROVAgentRuntime.ApplyThrusterActions(orderedThrusters, currentActions, thrusterCommandMode, actionForceScaleN);
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
                Debug.Log($"[{nameof(ControlForVelocity_IncrementalReward)}] {statusMessage}", this);
                logThrusterResolution = false;
            }
        }
        else
        {
            Debug.LogError($"[{nameof(ControlForVelocity_IncrementalReward)}] {statusMessage}", this);
        }
    }

    void UpdateDebugDisplay()
    {
        if (debugDisplay == null)
        {
            return;
        }

        debugDisplay.targetLinearVelocity = targetLinearVelocity;
        debugDisplay.targetAngularVelocity = targetAngularVelocity;
        debugDisplay.currentLinearVelocity = GetLocalLinearVelocity();
        debugDisplay.currentAngularVelocity = GetLocalAngularVelocity();
        debugDisplay.currentReward = GetCumulativeReward();
        debugDisplay.stepReward = lastStepReward;
        debugDisplay.velocitySpace = VelocityDebugDisplay.VelocitySpace.Local;
        debugDisplay.targetTransform = ReferenceTransform;
    }

    float GetLinearError01()
    {
        Vector3 diff = NormalizeLinearVector(GetLocalLinearVelocity() - targetLinearVelocity);
        return Mathf.Clamp01(diff.magnitude / Mathf.Sqrt(3f));
    }

    float GetAngularError01()
    {
        Vector3 diff = NormalizeAngularVector(GetLocalAngularVelocity() - targetAngularVelocity);
        return Mathf.Clamp01(diff.magnitude / Mathf.Sqrt(3f));
    }

    Vector3 NormalizeLinearVector(Vector3 value)
    {
        return new Vector3(
            SafeDivide(value.x, maxLinearVelocityX),
            SafeDivide(value.y, maxLinearVelocityY),
            SafeDivide(value.z, maxLinearVelocityZ)
        );
    }

    Vector3 NormalizeAngularVector(Vector3 value)
    {
        return new Vector3(
            SafeDivide(value.x, maxAngularVelocityX),
            SafeDivide(value.y, maxAngularVelocityY),
            SafeDivide(value.z, maxAngularVelocityZ)
        );
    }

    static float SafeDivide(float value, float denominator)
    {
        return Mathf.Approximately(denominator, 0f) ? 0f : value / denominator;
    }

    Vector3 GetLocalLinearVelocity()
    {
        return FinsROVAgentRuntime.GetLocalLinearVelocity(ReferenceTransform, RigidBody);
    }

    Vector3 GetLocalAngularVelocity()
    {
        return FinsROVAgentRuntime.GetLocalAngularVelocity(ReferenceTransform, RigidBody);
    }
}
