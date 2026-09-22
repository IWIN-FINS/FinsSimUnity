using System.Collections.Generic;
using System.Text;
using NWH.Common.SceneManagement;
using NWH.Common.Vehicles;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Policies;
using UnityEngine;
using UnityEngine.Serialization;

[DisallowMultipleComponent]
public class ThreeChaseOneBaselineController : MonoBehaviour
{
    private sealed class ChaserHeuristicSnapshot
    {
        public int frame;
        public Vector3 targetPosition;
        public Vector3 desiredOffsetWorld;
        public Vector3 desiredOffsetLocal;
        public float verticalCommand;
        public float forwardCommand;
        public float steeringCommand;
        public bool arrived;
        public readonly float[] actions = new float[8];
    }

    private sealed class PreyHeuristicSnapshot
    {
        public int frame;
        public Vector3 escapeDirection;
        public readonly float[] actions = new float[3];
    }

    public enum ChaserBaselineMode
    {
        DirectChase,
        SplitNettersAndHerder
    }

    public enum PreyBaselineMode
    {
        Idle,
        Random,
        WrapperNearestThreat,
        HybridEscape
    }

    [Header("Runtime Switch")]
    [Tooltip("是否启用这套基线追逃控制。")]
    public bool baselineEnabled = true;
    [Tooltip("保留的兼容开关。当前基线测试已改为 OnActionReceived 覆写动作，通常不需要再切 BehaviorType。")]
    public bool autoSwitchBehaviorType = false;
    [Tooltip("运行时切换基线控制启停的按键。")]
    public KeyCode toggleBaselineKey = KeyCode.F7;
    [Tooltip("当前场景实际要控制的 CatchAreaManager。若留空会自动查找。")]
    public CatchAreaManager areaManager;
    [Tooltip("基线测试时是否自动关闭 VehicleChanger 对非当前载具的休眠。")]
    public bool keepAllVehiclesAwakeForBaseline = true;

    [Header("Strategy")]
    [Tooltip("是否由 Unity baseline 覆写追方动作。")]
    public bool baselineControlsChasers = true;
    [Tooltip("是否由 Unity baseline 覆写鱼的动作。")]
    public bool baselineControlsPrey = true;
    [Tooltip("追方的简单基线策略。")]
    public ChaserBaselineMode chaserMode = ChaserBaselineMode.SplitNettersAndHerder;
    [Tooltip("鱼的简单逃逸策略。")]
    public PreyBaselineMode preyMode = PreyBaselineMode.HybridEscape;

    [Header("Chaser Tuning")]
    [Tooltip("网两侧目标间距；若 <= 0 则回退到 netter1 的 desiredNetterSpacing。")]
    public float desiredNetterSpacingOverride = 0f;
    [Tooltip("SplitNetters 模式下，两个 netter 相对鱼在前方的超前距离。")]
    public float netterForwardOffset = 2.0f;
    [Tooltip("SplitNetters 模式下，herder 相对鱼在后方的追赶距离。")]
    public float herderBehindOffset = 3.0f;
    [Tooltip("与目标的高度差达到该值时给满 vertical throttle。")]
    public float verticalDistanceForFullThrust = 2.5f;
    [Tooltip("与目标的前后差达到该值时给满 forward throttle。")]
    public float forwardDistanceForFullThrust = 8f;
    [Tooltip("偏航误差达到该角度时给满 steering。")]
    public float yawErrorForFullSteerDeg = 60f;
    [Tooltip("水平自动 steering 的最大幅度。原始手动 Heuristic 里实际只用了 +/-0.1。")]
    [Range(0f, 1f)]
    public float maxHorizontalSteeringCommand = 0.1f;
    [Tooltip("转向越大，前进推力衰减越明显。1 表示满转时完全抑制前进。")]
    [Range(0f, 1.5f)]
    public float forwardSuppressionBySteering = 0.7f;
    [Tooltip("是否沿用原始手动 Heuristic 的水平混合方式：转向时不额外压低前进推力。")]
    public bool mimicLegacyHorizontalHeuristic = true;
    [Tooltip("距离目标很近时，认为追方已经到位。")]
    public float chaserArrivalDistance = 1.2f;

    [Header("Prey Tuning")]
    [Tooltip("最近威胁逃逸方向的权重。")]
    public float preyNearestThreatWeight = 1f;
    [Tooltip("远离所有追方平均位置方向的权重。")]
    public float preyAverageThreatWeight = 0.35f;
    [Tooltip("远离网中心方向的权重。")]
    public float preyNetCenterWeight = 0.2f;
    [Tooltip("轻微随机扰动权重，用于减少纯对称局面卡死。")]
    public float preyNoiseWeight = 0.08f;
    [Tooltip("生成随机扰动的时间尺度。")]
    public float preyNoiseFrequency = 0.35f;
    [FormerlySerializedAs("randomPreyUsesHorizontalPlane")]
    [Tooltip("Random 鱼策略是否避免把鱼继续随机到水面外。开启后仍保留 y 随机分量，只会在接近/超过水面时抑制向上随机。")]
    public bool randomPreyAvoidLeavingSurface = true;
    [Tooltip("鱼的逃逸方向中，垂直分量的额外缩放。")]
    public float preyVerticalBias = 0.6f;
    [Tooltip("鱼基线逃逸方向的低通平滑系数。0 表示不平滑，越接近 1 越保留上一帧方向。")]
    [Range(0f, 0.95f)]
    public float preyEscapeDirectionSmoothing = 0.35f;

    [Header("Debug")]
    [Tooltip("是否绘制基线目标点和连线。")]
    public bool drawTargetGizmos = true;
    [Tooltip("目标点 gizmo 半径。")]
    public float gizmoSphereRadius = 0.18f;

    private readonly Dictionary<BehaviorParameters, BehaviorType> originalBehaviorTypes =
        new Dictionary<BehaviorParameters, BehaviorType>();
    private readonly Dictionary<ChaserAgent, ChaserHeuristicSnapshot> chaserSnapshots =
        new Dictionary<ChaserAgent, ChaserHeuristicSnapshot>();
    private readonly Dictionary<PreyAgent, PreyHeuristicSnapshot> preySnapshots =
        new Dictionary<PreyAgent, PreyHeuristicSnapshot>();
    private readonly StringBuilder debugStringBuilder = new StringBuilder(256);
    private VehicleChanger cachedVehicleChanger;
    private bool? originalPutOtherVehiclesToSleep;
    [SerializeField, HideInInspector] private bool baselineRoleControlDefaultsInitialized;
    public static ThreeChaseOneBaselineController ActiveInstance { get; private set; }

    private void Awake()
    {
        EnsureBaselineRoleControlDefaults();
        ActiveInstance = this;
        ResolveAreaManager();
    }

    private void Start()
    {
        ResolveAreaManager();
        SyncVehicleChangerSleepPolicy();
    }

    private void OnEnable()
    {
        EnsureBaselineRoleControlDefaults();
        ActiveInstance = this;
        ResolveAreaManager();
        SyncVehicleChangerSleepPolicy();
    }

    private void OnValidate()
    {
        EnsureBaselineRoleControlDefaults();
    }

    private void OnDisable()
    {
        if (ActiveInstance == this)
        {
            ActiveInstance = null;
        }
        RestoreVehicleChangerSleepPolicy();
    }

    private void Update()
    {
        if (!Application.isPlaying)
        {
            return;
        }

        if (InputSystemKeyBridge.WasPressedThisFrame(toggleBaselineKey))
        {
            baselineEnabled = !baselineEnabled;
            SyncVehicleChangerSleepPolicy();
        }
    }

    public bool TryFillChaserHeuristic(ChaserAgent agent, ActionSegment<float> continuousActionsOut)
    {
        float[] actionBuffer = new float[continuousActionsOut.Length];
        if (!TryGetChaserOverrideActions(agent, actionBuffer))
        {
            return false;
        }

        CopyActions(actionBuffer, continuousActionsOut);
        return true;
    }

    public bool TryFillPreyHeuristic(PreyAgent agent, ActionSegment<float> continuousActionsOut)
    {
        float[] actionBuffer = new float[continuousActionsOut.Length];
        if (!TryGetPreyOverrideActions(agent, actionBuffer))
        {
            return false;
        }

        CopyActions(actionBuffer, continuousActionsOut);
        return true;
    }

    public bool TryGetChaserOverrideActions(ChaserAgent agent, float[] actionsOut)
    {
        CatchAreaManager manager = ResolveAreaManager(agent != null ? agent.areaManager : null);
        if (!ShouldOverrideChaserActions() || agent == null || manager == null || manager.fish == null || actionsOut == null)
        {
            return false;
        }

        ClearActions(actionsOut);

        Transform controlTransform = agent.selfTransform != null ? agent.selfTransform : agent.transform;
        Vector3 targetPosition = ComputeChaserTarget(agent, manager);
        Vector3 desiredOffsetWorld = targetPosition - controlTransform.position;
        Vector3 desiredOffsetLocal = ControllerBodyFrame.WorldToBodyPositionDelta(controlTransform, desiredOffsetWorld);
        WriteChaserContinuousActions(
            desiredOffsetWorld,
            desiredOffsetLocal,
            actionsOut,
            out float verticalCommand,
            out float forwardCommand,
            out float steeringCommand,
            out bool arrived);
        RecordChaserSnapshot(
            agent,
            targetPosition,
            desiredOffsetWorld,
            desiredOffsetLocal,
            verticalCommand,
            forwardCommand,
            steeringCommand,
            arrived,
            actionsOut);
        return true;
    }

    public bool TryGetPreyOverrideActions(PreyAgent agent, float[] actionsOut)
    {
        if (!ShouldOverridePreyActions() || agent == null || actionsOut == null)
        {
            return false;
        }

        ClearActions(actionsOut);

        if (preyMode == PreyBaselineMode.Idle)
        {
            RecordPreySnapshot(agent, Vector3.zero, actionsOut);
            return true;
        }

        Vector3 escapeDirection = ComputePreyEscapeDirection(agent);
        if (escapeDirection.sqrMagnitude < 1e-6f)
        {
            RecordPreySnapshot(agent, Vector3.zero, actionsOut);
            return true;
        }

        escapeDirection.Normalize();
        escapeDirection = SmoothPreyEscapeDirection(agent, escapeDirection);
        if (actionsOut.Length > 0) actionsOut[0] = Mathf.Clamp(escapeDirection.x, -1f, 1f);
        if (actionsOut.Length > 1) actionsOut[1] = Mathf.Clamp(escapeDirection.y, -1f, 1f);
        if (actionsOut.Length > 2) actionsOut[2] = Mathf.Clamp(escapeDirection.z, -1f, 1f);
        RecordPreySnapshot(agent, escapeDirection, actionsOut);
        return true;
    }

    public bool ShouldOverrideChaserActions()
    {
        return baselineEnabled && baselineControlsChasers;
    }

    public bool ShouldOverridePreyActions()
    {
        return baselineEnabled && baselineControlsPrey;
    }

    public bool IsAnyBaselineOverrideEnabled()
    {
        return ShouldOverrideChaserActions() || ShouldOverridePreyActions();
    }

    private void EnsureBaselineRoleControlDefaults()
    {
        if (baselineRoleControlDefaultsInitialized)
        {
            return;
        }

        baselineControlsChasers = true;
        baselineControlsPrey = true;
        baselineRoleControlDefaultsInitialized = true;
    }

    public string GetStatusSummary(CatchAreaManager expectedAreaManager = null)
    {
        CatchAreaManager manager = ResolveAreaManager(expectedAreaManager);
        string linkedArea = manager != null ? manager.name : "null";
        string expectedState = expectedAreaManager == null
            ? "No expected area manager was supplied."
            : (manager == expectedAreaManager
                ? "The current link matches the expected area manager."
                : "The current link does not match the expected area manager.");
        return
            $"Input pipeline uses OnActionReceived override. " +
            $"Linked area manager: {linkedArea}. " +
            $"Area link validation: {expectedState} " +
            $"Chaser override active: {ShouldOverrideChaserActions()}. " +
            $"Prey override active: {ShouldOverridePreyActions()}. " +
            $"Host object: {name}. " +
            $"Vehicle changer: {GetVehicleChangerSummary()}.";
    }

    private void SyncVehicleChangerSleepPolicy()
    {
        if (!keepAllVehiclesAwakeForBaseline)
        {
            return;
        }

        VehicleChanger vehicleChanger = ResolveVehicleChanger();
        if (vehicleChanger == null)
        {
            return;
        }

        if (!originalPutOtherVehiclesToSleep.HasValue)
        {
            originalPutOtherVehiclesToSleep = vehicleChanger.putOtherVehiclesToSleep;
        }

        if (IsAnyBaselineOverrideEnabled())
        {
            vehicleChanger.putOtherVehiclesToSleep = false;
            WakeAllManagedVehicles(vehicleChanger);
        }
        else if (originalPutOtherVehiclesToSleep.HasValue)
        {
            vehicleChanger.putOtherVehiclesToSleep = originalPutOtherVehiclesToSleep.Value;
        }
    }

    private void RestoreVehicleChangerSleepPolicy()
    {
        if (!keepAllVehiclesAwakeForBaseline)
        {
            return;
        }

        VehicleChanger vehicleChanger = ResolveVehicleChanger();
        if (vehicleChanger != null && originalPutOtherVehiclesToSleep.HasValue)
        {
            vehicleChanger.putOtherVehiclesToSleep = originalPutOtherVehiclesToSleep.Value;
        }
    }

    private VehicleChanger ResolveVehicleChanger()
    {
        if (cachedVehicleChanger != null)
        {
            return cachedVehicleChanger;
        }

        cachedVehicleChanger = FindFirstObjectByType<VehicleChanger>();
        return cachedVehicleChanger;
    }

    private static void WakeAllManagedVehicles(VehicleChanger vehicleChanger)
    {
        if (vehicleChanger == null || vehicleChanger.vehicles == null)
        {
            return;
        }

        for (int i = 0; i < vehicleChanger.vehicles.Count; i++)
        {
            Vehicle vehicle = vehicleChanger.vehicles[i];
            if (vehicle != null)
            {
                vehicle.enabled = true;
            }
        }
    }

    private string GetVehicleChangerSummary()
    {
        VehicleChanger vehicleChanger = ResolveVehicleChanger();
        if (vehicleChanger == null)
        {
            return "not found";
        }

        int enabledCount = 0;
        int totalCount = 0;
        if (vehicleChanger.vehicles != null)
        {
            for (int i = 0; i < vehicleChanger.vehicles.Count; i++)
            {
                Vehicle vehicle = vehicleChanger.vehicles[i];
                if (vehicle == null)
                {
                    continue;
                }

                totalCount++;
                if (vehicle.enabled)
                {
                    enabledCount++;
                }
            }
        }

        return
            $"sleep for non-current vehicles is {(vehicleChanger.putOtherVehiclesToSleep ? "enabled" : "disabled")}, " +
            $"active vehicles: {enabledCount}/{totalCount}";
    }

    public string GetChaserHeuristicSummary(ChaserAgent agent)
    {
        if (agent == null)
        {
            return "No chaser reference was provided.";
        }

        if (!chaserSnapshots.TryGetValue(agent, out ChaserHeuristicSnapshot snapshot))
        {
            return "No heuristic snapshot has been recorded for this chaser yet.";
        }

        debugStringBuilder.Clear();
        debugStringBuilder.Append(snapshot.arrived
            ? "Target state: arrived. "
            : "Target state: driving toward target. ");
        debugStringBuilder.Append("Snapshot age: ");
        debugStringBuilder.Append(Time.frameCount - snapshot.frame);
        debugStringBuilder.Append(" frames. Vertical command: ");
        debugStringBuilder.Append(snapshot.verticalCommand.ToString("F2"));
        debugStringBuilder.Append(". Forward command: ");
        debugStringBuilder.Append(snapshot.forwardCommand.ToString("F2"));
        debugStringBuilder.Append(". Steering command: ");
        debugStringBuilder.Append(snapshot.steeringCommand.ToString("F2"));
        debugStringBuilder.Append(". Horizontal thruster actions: [");
        AppendAction(debugStringBuilder, snapshot.actions, 4);
        debugStringBuilder.Append(", ");
        AppendAction(debugStringBuilder, snapshot.actions, 5);
        debugStringBuilder.Append(", ");
        AppendAction(debugStringBuilder, snapshot.actions, 6);
        debugStringBuilder.Append(", ");
        AppendAction(debugStringBuilder, snapshot.actions, 7);
        debugStringBuilder.Append("]. Target position: ");
        AppendVector3(debugStringBuilder, snapshot.targetPosition);
        return debugStringBuilder.ToString();
    }

    public string GetPreyHeuristicSummary(PreyAgent agent)
    {
        if (agent == null)
        {
            return "No prey reference was provided.";
        }

        if (!preySnapshots.TryGetValue(agent, out PreyHeuristicSnapshot snapshot))
        {
            return "No heuristic snapshot has been recorded for this prey yet.";
        }

        debugStringBuilder.Clear();
        debugStringBuilder.Append("Snapshot age: ");
        debugStringBuilder.Append(Time.frameCount - snapshot.frame);
        debugStringBuilder.Append(" frames. Movement actions (x, y, z): [");
        AppendAction(debugStringBuilder, snapshot.actions, 0);
        debugStringBuilder.Append(", ");
        AppendAction(debugStringBuilder, snapshot.actions, 1);
        debugStringBuilder.Append(", ");
        AppendAction(debugStringBuilder, snapshot.actions, 2);
        debugStringBuilder.Append("]. Escape direction: ");
        AppendVector3(debugStringBuilder, snapshot.escapeDirection);
        return debugStringBuilder.ToString();
    }

    public static ThreeChaseOneBaselineController ResolveActive(CatchAreaManager preferredAreaManager = null)
    {
        if (ActiveInstance != null)
        {
            ActiveInstance.ResolveAreaManager(preferredAreaManager);
            return ActiveInstance;
        }

        ThreeChaseOneBaselineController controller = FindFirstObjectByType<ThreeChaseOneBaselineController>();
        if (controller != null)
        {
            ActiveInstance = controller;
            controller.ResolveAreaManager(preferredAreaManager);
        }

        return controller;
    }

    private void ApplyHeuristicOnlyBehaviorType()
    {
        foreach (BehaviorParameters parameters in EnumerateBehaviorParameters())
        {
            if (parameters == null)
            {
                continue;
            }

            if (!originalBehaviorTypes.ContainsKey(parameters))
            {
                originalBehaviorTypes[parameters] = parameters.BehaviorType;
            }

            parameters.BehaviorType = BehaviorType.HeuristicOnly;
        }
    }

    private void RestoreBehaviorTypes()
    {
        foreach (KeyValuePair<BehaviorParameters, BehaviorType> pair in originalBehaviorTypes)
        {
            if (pair.Key != null)
            {
                pair.Key.BehaviorType = pair.Value;
            }
        }
    }

    private IEnumerable<BehaviorParameters> EnumerateBehaviorParameters()
    {
        CatchAreaManager manager = ResolveAreaManager();
        if (manager == null)
        {
            yield break;
        }

        if (manager.netter1 != null && manager.netter1.TryGetComponent(out BehaviorParameters netter1Behavior))
        {
            yield return netter1Behavior;
        }

        if (manager.netter2 != null && manager.netter2.TryGetComponent(out BehaviorParameters netter2Behavior))
        {
            yield return netter2Behavior;
        }

        if (manager.herder != null && manager.herder.TryGetComponent(out BehaviorParameters herderBehavior))
        {
            yield return herderBehavior;
        }

        if (manager.fish != null && manager.fish.TryGetComponent(out BehaviorParameters fishBehavior))
        {
            yield return fishBehavior;
        }
    }

    private CatchAreaManager ResolveAreaManager(CatchAreaManager preferredAreaManager = null)
    {
        if (preferredAreaManager != null)
        {
            areaManager = preferredAreaManager;
            return areaManager;
        }

        if (areaManager != null)
        {
            return areaManager;
        }

        areaManager = GetComponent<CatchAreaManager>();
        if (areaManager != null)
        {
            return areaManager;
        }

        areaManager = FindFirstObjectByType<CatchAreaManager>();
        return areaManager;
    }

    private static void ClearActions(ActionSegment<float> continuousActionsOut)
    {
        for (int i = 0; i < continuousActionsOut.Length; i++)
        {
            continuousActionsOut[i] = 0f;
        }
    }

    private static void ClearActions(float[] actionsOut)
    {
        for (int i = 0; i < actionsOut.Length; i++)
        {
            actionsOut[i] = 0f;
        }
    }

    private Vector3 ComputeChaserTarget(ChaserAgent agent, CatchAreaManager manager)
    {
        Vector3 fishPosition = manager.fish.transform.position;
        if (chaserMode == ChaserBaselineMode.DirectChase)
        {
            return fishPosition;
        }

        float targetSpacing = desiredNetterSpacingOverride > 0f
            ? desiredNetterSpacingOverride
            : (manager.netter1 != null ? manager.netter1.desiredNetterSpacing : 6f);
        float halfSpacing = targetSpacing * 0.5f;

        Vector3 netterMidpoint = GetNetterMidpoint(manager, fishPosition);
        Vector3 herderPosition = manager.herder != null ? manager.herder.transform.position : fishPosition - Vector3.forward;
        Vector3 driveAxis = Vector3.ProjectOnPlane(netterMidpoint - fishPosition, Vector3.up);
        if (driveAxis.sqrMagnitude < 1e-4f)
        {
            driveAxis = Vector3.ProjectOnPlane(fishPosition - herderPosition, Vector3.up);
        }
        if (driveAxis.sqrMagnitude < 1e-4f)
        {
            driveAxis = Vector3.forward;
        }
        driveAxis.Normalize();

        Vector3 lateralAxis = Vector3.Cross(Vector3.up, driveAxis);
        if (lateralAxis.sqrMagnitude < 1e-4f)
        {
            lateralAxis = Vector3.right;
        }
        lateralAxis.Normalize();

        if (agent == manager.herder)
        {
            return fishPosition - driveAxis * herderBehindOffset;
        }

        float lateralSign = DetermineNetterLateralSign(agent, manager, netterMidpoint, lateralAxis);
        return fishPosition + driveAxis * netterForwardOffset + lateralAxis * (lateralSign * halfSpacing);
    }

    private static float DetermineNetterLateralSign(
        ChaserAgent agent,
        CatchAreaManager manager,
        Vector3 netterMidpoint,
        Vector3 lateralAxis)
    {
        if (agent == null)
        {
            return 0f;
        }

        Transform pose = agent.selfTransform != null ? agent.selfTransform : agent.transform;
        float currentSide = Vector3.Dot(pose.position - netterMidpoint, lateralAxis);
        if (Mathf.Abs(currentSide) > 1e-3f)
        {
            return Mathf.Sign(currentSide);
        }

        ChaserAgent otherNetter = agent == manager.netter1 ? manager.netter2 : manager.netter1;
        if (otherNetter != null)
        {
            Transform otherPose = otherNetter.selfTransform != null ? otherNetter.selfTransform : otherNetter.transform;
            float otherSide = Vector3.Dot(otherPose.position - netterMidpoint, lateralAxis);
            if (Mathf.Abs(otherSide) > 1e-3f)
            {
                return -Mathf.Sign(otherSide);
            }
        }

        return agent == manager.netter1 ? -1f : 1f;
    }

    private void WriteChaserContinuousActions(
        Vector3 desiredOffsetWorld,
        Vector3 desiredOffsetLocal,
        float[] actionsOut,
        out float verticalCommand,
        out float forwardCommand,
        out float steeringCommand,
        out bool arrived)
    {
        verticalCommand = 0f;
        forwardCommand = 0f;
        steeringCommand = 0f;
        arrived = desiredOffsetWorld.sqrMagnitude < chaserArrivalDistance * chaserArrivalDistance;
        if (arrived)
        {
            return;
        }

        verticalCommand = Mathf.Clamp(
            desiredOffsetLocal.y / Mathf.Max(0.01f, verticalDistanceForFullThrust),
            -1f,
            1f);

        // Controller body frame convention:
        //   x = forward, y = up, z = left
        // Horizontal yaw error therefore uses atan2(left, forward).
        Vector2 planarLocal = new Vector2(desiredOffsetLocal.x, desiredOffsetLocal.z);
        float yawErrorDeg = planarLocal.sqrMagnitude > 1e-6f
            ? Mathf.Atan2(planarLocal.y, planarLocal.x) * Mathf.Rad2Deg
            : 0f;
        steeringCommand = Mathf.Clamp(
            yawErrorDeg / Mathf.Max(1f, yawErrorForFullSteerDeg),
            -1f,
            1f) * maxHorizontalSteeringCommand;

        forwardCommand = Mathf.Clamp(desiredOffsetLocal.x / Mathf.Max(0.01f, forwardDistanceForFullThrust), -1f, 1f);
        if (!mimicLegacyHorizontalHeuristic)
        {
            float normalizedSteering = maxHorizontalSteeringCommand > 1e-6f
                ? Mathf.Abs(steeringCommand) / maxHorizontalSteeringCommand
                : 0f;
            float steeringSuppression = Mathf.Clamp01(1f - normalizedSteering * forwardSuppressionBySteering);
            float headingAlignment = Mathf.Clamp01(Mathf.Cos(yawErrorDeg * Mathf.Deg2Rad));
            forwardCommand *= Mathf.Min(steeringSuppression, headingAlignment);
        }

        if (actionsOut.Length > 0) actionsOut[0] = verticalCommand;
        if (actionsOut.Length > 1) actionsOut[1] = verticalCommand;
        if (actionsOut.Length > 2) actionsOut[2] = verticalCommand;
        if (actionsOut.Length > 3) actionsOut[3] = verticalCommand;

        if (actionsOut.Length > 4) actionsOut[4] = Mathf.Clamp(forwardCommand + steeringCommand, -1f, 1f);
        if (actionsOut.Length > 5) actionsOut[5] = Mathf.Clamp(forwardCommand + steeringCommand, -1f, 1f);
        if (actionsOut.Length > 6) actionsOut[6] = Mathf.Clamp(-forwardCommand + steeringCommand, -1f, 1f);
        if (actionsOut.Length > 7) actionsOut[7] = Mathf.Clamp(-forwardCommand + steeringCommand, -1f, 1f);
    }

    private void RecordChaserSnapshot(
        ChaserAgent agent,
        Vector3 targetPosition,
        Vector3 desiredOffsetWorld,
        Vector3 desiredOffsetLocal,
        float verticalCommand,
        float forwardCommand,
        float steeringCommand,
        bool arrived,
        float[] actionsOut)
    {
        if (agent == null)
        {
            return;
        }

        if (!chaserSnapshots.TryGetValue(agent, out ChaserHeuristicSnapshot snapshot))
        {
            snapshot = new ChaserHeuristicSnapshot();
            chaserSnapshots[agent] = snapshot;
        }

        snapshot.frame = Time.frameCount;
        snapshot.targetPosition = targetPosition;
        snapshot.desiredOffsetWorld = desiredOffsetWorld;
        snapshot.desiredOffsetLocal = desiredOffsetLocal;
        snapshot.verticalCommand = verticalCommand;
        snapshot.forwardCommand = forwardCommand;
        snapshot.steeringCommand = steeringCommand;
        snapshot.arrived = arrived;
        CopyActions(actionsOut, snapshot.actions);
    }

    private void RecordPreySnapshot(
        PreyAgent agent,
        Vector3 escapeDirection,
        float[] actionsOut)
    {
        if (agent == null)
        {
            return;
        }

        if (!preySnapshots.TryGetValue(agent, out PreyHeuristicSnapshot snapshot))
        {
            snapshot = new PreyHeuristicSnapshot();
            preySnapshots[agent] = snapshot;
        }

        snapshot.frame = Time.frameCount;
        snapshot.escapeDirection = escapeDirection;
        CopyActions(actionsOut, snapshot.actions);
    }

    private Vector3 SmoothPreyEscapeDirection(PreyAgent agent, Vector3 rawEscapeDirection)
    {
        float keepPrevious = Mathf.Clamp01(preyEscapeDirectionSmoothing);
        if (keepPrevious <= 1e-4f || agent == null)
        {
            return rawEscapeDirection;
        }

        if (!preySnapshots.TryGetValue(agent, out PreyHeuristicSnapshot snapshot) ||
            snapshot.escapeDirection.sqrMagnitude < 1e-6f)
        {
            return rawEscapeDirection;
        }

        Vector3 smoothed = Vector3.Lerp(rawEscapeDirection, snapshot.escapeDirection.normalized, keepPrevious);
        return smoothed.sqrMagnitude > 1e-6f
            ? smoothed.normalized
            : rawEscapeDirection;
    }

    private static void CopyActions(ActionSegment<float> source, float[] target)
    {
        if (target == null)
        {
            return;
        }

        for (int i = 0; i < target.Length; i++)
        {
            target[i] = i < source.Length ? source[i] : 0f;
        }
    }

    private static void CopyActions(float[] source, ActionSegment<float> target)
    {
        for (int i = 0; i < target.Length; i++)
        {
            target[i] = i < source.Length ? source[i] : 0f;
        }
    }

    private static void CopyActions(float[] source, float[] target)
    {
        if (target == null)
        {
            return;
        }

        for (int i = 0; i < target.Length; i++)
        {
            target[i] = i < source.Length ? source[i] : 0f;
        }
    }

    private static void AppendAction(StringBuilder builder, float[] actions, int index)
    {
        builder.Append(index >= 0 && index < actions.Length ? actions[index].ToString("F2") : "0.00");
    }

    private static void AppendVector3(StringBuilder builder, Vector3 value)
    {
        builder.Append("(");
        builder.Append(value.x.ToString("F1"));
        builder.Append(", ");
        builder.Append(value.y.ToString("F1"));
        builder.Append(", ");
        builder.Append(value.z.ToString("F1"));
        builder.Append(")");
    }

    private static Vector3 GetNetterMidpoint(CatchAreaManager manager, Vector3 fallbackPosition)
    {
        if (manager == null)
        {
            return fallbackPosition;
        }

        int count = 0;
        Vector3 sum = Vector3.zero;
        if (manager.netter1 != null)
        {
            sum += manager.netter1.transform.position;
            count++;
        }

        if (manager.netter2 != null)
        {
            sum += manager.netter2.transform.position;
            count++;
        }

        return count > 0 ? sum / count : fallbackPosition;
    }

    private Vector3 ComputePreyEscapeDirection(PreyAgent agent)
    {
        if (preyMode == PreyBaselineMode.Random)
        {
            return ComputeRandomPreyDirection(agent);
        }

        CatchAreaManager manager = ResolveAreaManager(agent != null ? agent.areaManager : null);
        Vector3 wrapperNearestThreatDirection = ComputeWrapperNearestThreatDirection(agent);
        if (preyMode == PreyBaselineMode.WrapperNearestThreat)
        {
            return wrapperNearestThreatDirection;
        }

        Vector3 averageThreatDirection = ComputeAverageThreatDirection(agent);
        Vector3 netCenterDirection = manager != null
            ? agent.transform.position - manager.GetNetCenter()
            : Vector3.zero;
        Vector3 noiseDirection = ComputeNoiseDirection(agent.transform.position);

        Vector3 blendedDirection =
            wrapperNearestThreatDirection * preyNearestThreatWeight
            + averageThreatDirection * preyAverageThreatWeight
            + netCenterDirection.normalized * preyNetCenterWeight
            + noiseDirection * preyNoiseWeight;

        if (blendedDirection.sqrMagnitude < 1e-6f)
        {
            blendedDirection = wrapperNearestThreatDirection.sqrMagnitude > 1e-6f
                ? wrapperNearestThreatDirection
                : Random.onUnitSphere;
        }

        blendedDirection.y *= preyVerticalBias;
        return blendedDirection.normalized;
    }

    private Vector3 ComputeRandomPreyDirection(PreyAgent agent)
    {
        Vector3 referencePosition = agent != null ? agent.transform.position : Vector3.zero;
        Vector3 randomDirection = ComputeNoiseDirection(referencePosition);
        randomDirection.y *= preyVerticalBias;

        if (randomPreyAvoidLeavingSurface && agent != null && randomDirection.y > 0f)
        {
            float upperY = agent.waterSurfaceY - Mathf.Max(0f, agent.surfaceClearance);
            float buffer = Mathf.Max(0f, agent.verticalBoundaryBuffer);

            if (referencePosition.y >= upperY)
            {
                randomDirection.y = 0f;
            }
            else if (buffer > 1e-4f && referencePosition.y > upperY - buffer)
            {
                float upwardScale = Mathf.InverseLerp(upperY, upperY - buffer, referencePosition.y);
                randomDirection.y *= upwardScale;
            }
        }

        return randomDirection.sqrMagnitude > 1e-6f
            ? randomDirection.normalized
            : Vector3.forward;
    }

    private static Vector3 ComputeWrapperNearestThreatDirection(PreyAgent agent)
    {
        if (agent.chasers == null || agent.chasers.Length == 0)
        {
            return Vector3.zero;
        }

        Vector3 preyPosition = agent.transform.position;
        float minDistance = float.PositiveInfinity;
        Vector3 escapeDirection = Vector3.zero;

        foreach (Transform chaser in agent.chasers)
        {
            if (chaser == null)
            {
                continue;
            }

            Vector3 direction = preyPosition - chaser.position;
            float distance = direction.magnitude;
            if (distance > 0.1f && distance < minDistance)
            {
                minDistance = distance;
                escapeDirection = direction / distance;
            }
        }

        return escapeDirection;
    }

    private static Vector3 ComputeAverageThreatDirection(PreyAgent agent)
    {
        if (agent.chasers == null || agent.chasers.Length == 0)
        {
            return Vector3.zero;
        }

        Vector3 averageThreatPosition = Vector3.zero;
        int validCount = 0;
        foreach (Transform chaser in agent.chasers)
        {
            if (chaser == null)
            {
                continue;
            }

            averageThreatPosition += chaser.position;
            validCount++;
        }

        if (validCount == 0)
        {
            return Vector3.zero;
        }

        averageThreatPosition /= validCount;
        return (agent.transform.position - averageThreatPosition).normalized;
    }

    private Vector3 ComputeNoiseDirection(Vector3 referencePosition)
    {
        float t = Time.time * preyNoiseFrequency;
        float x = Mathf.PerlinNoise(referencePosition.x * 0.13f, t) * 2f - 1f;
        float y = Mathf.PerlinNoise(referencePosition.y * 0.17f + 31.7f, t + 7.1f) * 2f - 1f;
        float z = Mathf.PerlinNoise(referencePosition.z * 0.11f + 63.2f, t + 13.4f) * 2f - 1f;
        return new Vector3(x, y, z).normalized;
    }

    private void OnDrawGizmos()
    {
        if (!drawTargetGizmos)
        {
            return;
        }

        CatchAreaManager manager = ResolveAreaManager();
        if (manager == null || manager.fish == null)
        {
            return;
        }

        DrawAgentTargetGizmo(manager.netter1, Color.cyan, manager);
        DrawAgentTargetGizmo(manager.netter2, Color.green, manager);
        DrawAgentTargetGizmo(manager.herder, Color.yellow, manager);
    }

    private void DrawAgentTargetGizmo(ChaserAgent agent, Color color, CatchAreaManager manager)
    {
        if (agent == null || manager.fish == null)
        {
            return;
        }

        Color previousColor = Gizmos.color;
        Gizmos.color = color;
        Vector3 target = ComputeChaserTarget(agent, manager);
        Gizmos.DrawLine(agent.transform.position, target);
        Gizmos.DrawSphere(target, gizmoSphereRadius);
        Gizmos.color = previousColor;
    }
}
