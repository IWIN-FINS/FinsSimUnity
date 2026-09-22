using UnityEngine;

[DisallowMultipleComponent]
public class NetSeparationStressTest : MonoBehaviour
{
    [Header("Scene References")]
    [Tooltip("留空时会自动查找 CatchAreaManager。")]
    public CatchAreaManager areaManager;

    [Header("Runtime Control")]
    [Tooltip("总开关。关闭后不会覆写任何 UUV 动作。")]
    public bool stressTestEnabled = true;
    [Tooltip("进入 Play 后自动开始该分离测试。")]
    public bool autoStartOnPlay = true;
    [Tooltip("开始测试前是否先调用一次 ResetArea。")]
    public bool resetAreaOnStart = true;
    [Tooltip("开始施加分离动作前的等待时间，便于场景先稳定下来。")]
    public float startDelaySeconds = 1f;
    [Tooltip("运行时切换开始 / 停止测试的按键。")]
    public KeyCode toggleTestKey = KeyCode.F9;
    [Tooltip("运行时重置场景并重新开始测试的按键。")]
    public KeyCode resetAndRestartKey = KeyCode.F10;

    [Header("Separation Command")]
    [Tooltip("每个 netter 都会朝远离对方的方向施加外推；关闭持续模式时，也把它作为停止分离的目标间距。")]
    public float outwardPushDistance = 12f;
    [Tooltip("与目标高度差达到该值时给满 vertical throttle。")]
    public float verticalDistanceForFullThrust = 2.5f;
    [Tooltip("与目标前后差达到该值时给满 forward throttle。")]
    public float forwardDistanceForFullThrust = 8f;
    [Tooltip("偏航误差达到该角度时给满 steering。")]
    public float yawErrorForFullSteerDeg = 60f;
    [Tooltip("水平自动 steering 的最大幅度。")]
    [Range(0f, 1f)]
    public float maxHorizontalSteeringCommand = 0.1f;
    [Tooltip("默认关闭。这样测试只看水平分离，不会因为艇体俯仰/横滚把世界水平向量误算成下潜动作。")]
    public bool allowVerticalSeparationMotion = false;
    [Tooltip("始终持续向外推。如果关闭，则两个 netter 间距达到 outwardPushDistance 后停下。")]
    public bool pushContinuously = true;
    [Tooltip("关闭时只有两个 netter 被覆写；打开时 herder 也会被强制给零动作。")]
    public bool zeroHerderActions = true;

    [Header("Isolation")]
    [Tooltip("测试期间持续把 herder 刚体速度清零，避免它干扰拉网压力测试。")]
    public bool immobilizeHerderRigidbody = true;
    [Tooltip("测试期间持续把鱼的刚体速度清零，避免鱼碰网或逃逸干扰观察。")]
    public bool immobilizeFishRigidbody = true;

    [Header("Debug")]
    [Tooltip("是否按固定间隔输出网宽、网高、最大网体速度等信息。")]
    public bool enablePeriodicLogging = true;
    [Tooltip("日志输出间隔。")]
    public float logIntervalSeconds = 1f;
    [Tooltip("当网任意刚体速度超过该阈值时使用 Warning 输出。")]
    public float warningNetBodySpeed = 8f;

    private float activationTime = float.PositiveInfinity;
    private float nextLogTime;
    private string lastStatusMessage = "Stress test idle.";
    private float lastStatusRealtime;
    private string lastSnapshotMessage = "No stress snapshot recorded yet.";
    private float lastSnapshotRealtime;

    public static NetSeparationStressTest ActiveInstance { get; private set; }

    public bool IsProvidingActions =>
        stressTestEnabled && Application.isPlaying && Time.time >= activationTime;
    public string LastStatusMessage => lastStatusMessage;
    public float LastStatusAgeSeconds => Mathf.Max(0f, Time.realtimeSinceStartup - lastStatusRealtime);
    public string LastSnapshotMessage => lastSnapshotMessage;
    public float LastSnapshotAgeSeconds => Mathf.Max(0f, Time.realtimeSinceStartup - lastSnapshotRealtime);

    private void Awake()
    {
        ResolveAreaManager();
        lastStatusRealtime = Time.realtimeSinceStartup;
        lastSnapshotRealtime = Time.realtimeSinceStartup;
    }

    private void Start()
    {
        ResolveAreaManager();
        if (Application.isPlaying && autoStartOnPlay && stressTestEnabled)
        {
            StartTest();
        }
    }

    private void OnEnable()
    {
        ActiveInstance = this;
        ResolveAreaManager();
    }

    private void OnDisable()
    {
        if (ActiveInstance == this)
        {
            ActiveInstance = null;
        }
    }

    private void Update()
    {
        if (!Application.isPlaying)
        {
            return;
        }

        if (InputSystemKeyBridge.WasPressedThisFrame(toggleTestKey))
        {
            if (IsProvidingActions)
            {
                StopTest();
            }
            else
            {
                StartTest();
            }
        }

        if (InputSystemKeyBridge.WasPressedThisFrame(resetAndRestartKey))
        {
            ResetAndRestart();
        }
    }

    private void FixedUpdate()
    {
        if (!IsProvidingActions)
        {
            return;
        }

        ResolveAreaManager();
        HoldNonTestBodiesStill();

        if (!enablePeriodicLogging || Time.time < nextLogTime)
        {
            return;
        }

        nextLogTime = Time.time + Mathf.Max(0.1f, logIntervalSeconds);
        LogStressSnapshot();
    }

    [ContextMenu("Start Stress Test")]
    public void StartTest()
    {
        ResolveAreaManager();
        if (areaManager == null)
        {
            RecordStatus(
                $"{nameof(NetSeparationStressTest)} on {name} could not find a CatchAreaManager.",
                true);
            return;
        }

        if (resetAreaOnStart)
        {
            areaManager.ResetArea();
        }

        activationTime = Time.time + Mathf.Max(0f, startDelaySeconds);
        nextLogTime = activationTime;
        RecordStatus(
            $"{nameof(NetSeparationStressTest)} armed on {name}. " +
            $"Netters {GetAgentName(areaManager.netter1)} / {GetAgentName(areaManager.netter2)} " +
            $"will start separating at t={activationTime:F2}s.");
    }

    [ContextMenu("Stop Stress Test")]
    public void StopTest()
    {
        activationTime = float.PositiveInfinity;
        RecordStatus($"{nameof(NetSeparationStressTest)} stopped on {name}.");
    }

    [ContextMenu("Reset And Restart Stress Test")]
    public void ResetAndRestart()
    {
        StopTest();
        StartTest();
    }

    public bool TryGetChaserOverrideActions(ChaserAgent agent, float[] actionsOut)
    {
        ResolveAreaManager(agent != null ? agent.areaManager : null);
        if (!IsProvidingActions || agent == null || actionsOut == null || areaManager == null)
        {
            return false;
        }

        ClearActions(actionsOut);

        if (agent == areaManager.netter1)
        {
            return TryBuildNetterSeparationActions(agent, areaManager.netter2, actionsOut);
        }

        if (agent == areaManager.netter2)
        {
            return TryBuildNetterSeparationActions(agent, areaManager.netter1, actionsOut);
        }

        return agent == areaManager.herder && zeroHerderActions;
    }

    public static NetSeparationStressTest ResolveActive(CatchAreaManager preferredAreaManager = null)
    {
        if (ActiveInstance != null)
        {
            ActiveInstance.ResolveAreaManager(preferredAreaManager);
            return ActiveInstance;
        }

        NetSeparationStressTest instance = FindFirstObjectByType<NetSeparationStressTest>();
        if (instance != null)
        {
            ActiveInstance = instance;
            instance.ResolveAreaManager(preferredAreaManager);
        }

        return instance;
    }

    private bool TryBuildNetterSeparationActions(
        ChaserAgent agent,
        ChaserAgent otherNetter,
        float[] actionsOut)
    {
        if (agent == null || otherNetter == null)
        {
            return false;
        }

        Transform agentTransform = agent.selfTransform != null ? agent.selfTransform : agent.transform;
        Transform otherTransform = otherNetter.selfTransform != null ? otherNetter.selfTransform : otherNetter.transform;
        float currentNetterDistance = Vector3.Distance(agentTransform.position, otherTransform.position);

        if (!pushContinuously && currentNetterDistance >= outwardPushDistance)
        {
            return true;
        }

        Vector3 outwardDirection = Vector3.ProjectOnPlane(
            agentTransform.position - otherTransform.position,
            Vector3.up);

        if (outwardDirection.sqrMagnitude < 1e-4f
            && areaManager.NetSurfaceModel != null
            && areaManager.NetSurfaceModel.TryGetFrame(
                out _,
                out Vector3 netRight,
                out _,
                out _,
                out _,
                out _))
        {
            outwardDirection = agent == areaManager.netter1 ? -netRight : netRight;
        }

        if (outwardDirection.sqrMagnitude < 1e-4f)
        {
            outwardDirection = agentTransform.right;
        }

        outwardDirection.Normalize();
        Vector3 desiredOffsetWorld = outwardDirection * Mathf.Max(0.1f, outwardPushDistance);
        Vector3 desiredOffsetLocal = GetHorizontalControlOffset(agentTransform, desiredOffsetWorld);

        WriteChaserContinuousActions(desiredOffsetWorld, desiredOffsetLocal, actionsOut);
        return true;
    }

    private void WriteChaserContinuousActions(
        Vector3 desiredOffsetWorld,
        Vector3 desiredOffsetLocal,
        float[] actionsOut)
    {
        if (desiredOffsetWorld.sqrMagnitude < 0.01f)
        {
            return;
        }

        float verticalCommand = allowVerticalSeparationMotion
            ? Mathf.Clamp(
                desiredOffsetLocal.y / Mathf.Max(0.01f, verticalDistanceForFullThrust),
                -1f,
                1f)
            : 0f;

        Vector2 planarLocal = new Vector2(desiredOffsetLocal.x, desiredOffsetLocal.z);
        float yawErrorDeg = planarLocal.sqrMagnitude > 1e-6f
            ? Mathf.Atan2(-planarLocal.y, planarLocal.x) * Mathf.Rad2Deg
            : 0f;
        float steeringCommand = Mathf.Clamp(
            yawErrorDeg / Mathf.Max(1f, yawErrorForFullSteerDeg),
            -1f,
            1f) * maxHorizontalSteeringCommand;

        float forwardCommand = Mathf.Clamp(
            desiredOffsetLocal.x / Mathf.Max(0.01f, forwardDistanceForFullThrust),
            -1f,
            1f);

        if (actionsOut.Length > 0) actionsOut[0] = verticalCommand;
        if (actionsOut.Length > 1) actionsOut[1] = verticalCommand;
        if (actionsOut.Length > 2) actionsOut[2] = verticalCommand;
        if (actionsOut.Length > 3) actionsOut[3] = verticalCommand;
        if (actionsOut.Length > 4) actionsOut[4] = Mathf.Clamp(forwardCommand + steeringCommand, -1f, 1f);
        if (actionsOut.Length > 5) actionsOut[5] = Mathf.Clamp(forwardCommand + steeringCommand, -1f, 1f);
        if (actionsOut.Length > 6) actionsOut[6] = Mathf.Clamp(-forwardCommand + steeringCommand, -1f, 1f);
        if (actionsOut.Length > 7) actionsOut[7] = Mathf.Clamp(-forwardCommand + steeringCommand, -1f, 1f);
    }

    private static Vector3 GetHorizontalControlOffset(Transform agentTransform, Vector3 desiredOffsetWorld)
    {
        Vector3 surgeAxis = Vector3.ProjectOnPlane(agentTransform.right, Vector3.up);
        if (surgeAxis.sqrMagnitude < 1e-4f)
        {
            surgeAxis = Vector3.right;
        }
        surgeAxis.Normalize();

        Vector3 yawAxis = Vector3.ProjectOnPlane(agentTransform.forward, Vector3.up);
        if (yawAxis.sqrMagnitude < 1e-4f)
        {
            yawAxis = Vector3.forward;
        }
        yawAxis.Normalize();

        float localX = Vector3.Dot(desiredOffsetWorld, surgeAxis);
        float localZ = Vector3.Dot(desiredOffsetWorld, yawAxis);
        float localY = Vector3.Dot(desiredOffsetWorld, Vector3.up);
        return new Vector3(localX, localY, localZ);
    }

    private void HoldNonTestBodiesStill()
    {
        if (immobilizeHerderRigidbody && areaManager != null && areaManager.herder != null)
        {
            ZeroRigidbody(areaManager.herder.GetComponent<Rigidbody>());
        }

        if (immobilizeFishRigidbody && areaManager != null && areaManager.fish != null)
        {
            ZeroRigidbody(areaManager.fish.GetComponent<Rigidbody>());
        }
    }

    private void LogStressSnapshot()
    {
        if (areaManager == null || areaManager.netter1 == null || areaManager.netter2 == null)
        {
            return;
        }

        float netterDistance = Vector3.Distance(
            areaManager.netter1.transform.position,
            areaManager.netter2.transform.position);
        float netWidth = areaManager.NetSurfaceModel != null && areaManager.NetSurfaceModel.IsReady()
            ? areaManager.NetSurfaceModel.GetNetWidth()
            : netterDistance;
        float netHeight = areaManager.NetSurfaceModel != null && areaManager.NetSurfaceModel.IsReady()
            ? areaManager.NetSurfaceModel.GetNetHeight()
            : 0f;
        float maxNetBodySpeed = GetMaxNetBodySpeed();

        string message =
            $"{nameof(NetSeparationStressTest)} snapshot: " +
            $"distance={netterDistance:F2}m, " +
            $"netWidth={netWidth:F2}m, " +
            $"netHeight={netHeight:F2}m, " +
            $"maxNetBodySpeed={maxNetBodySpeed:F2}m/s.";

        RecordSnapshot(message, maxNetBodySpeed >= warningNetBodySpeed);
    }

    private float GetMaxNetBodySpeed()
    {
        if (areaManager == null || areaManager.net == null)
        {
            return 0f;
        }

        float maxSpeed = 0f;
        foreach (Rigidbody rb in areaManager.net.GetComponentsInChildren<Rigidbody>())
        {
            if (rb != null)
            {
                maxSpeed = Mathf.Max(maxSpeed, rb.linearVelocity.magnitude);
            }
        }

        return maxSpeed;
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

    private static void ZeroRigidbody(Rigidbody rb)
    {
        if (rb == null)
        {
            return;
        }

        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
    }

    private static void ClearActions(float[] actionsOut)
    {
        for (int i = 0; i < actionsOut.Length; i++)
        {
            actionsOut[i] = 0f;
        }
    }

    private static string GetAgentName(Component component)
    {
        return component != null ? component.name : "null";
    }

    private void RecordStatus(string message, bool warning = false)
    {
        lastStatusMessage = message;
        lastStatusRealtime = Time.realtimeSinceStartup;

        if (warning)
        {
            ThreeChaseOneRuntimeLog.Warning(message, this);
            return;
        }

        ThreeChaseOneRuntimeLog.Info(message, this);
    }

    private void RecordSnapshot(string message, bool warning = false)
    {
        lastSnapshotMessage = message;
        lastSnapshotRealtime = Time.realtimeSinceStartup;

        if (warning)
        {
            ThreeChaseOneRuntimeLog.Warning(message, this);
            return;
        }

        ThreeChaseOneRuntimeLog.Info(message, this);
    }
}
