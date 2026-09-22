using System.Collections.Generic;
using Obi;
using FinsSim.Hydrodynamics;
using UnityEngine;

public class CatchAreaManager : MonoBehaviour
{
    public enum CaptureCriterion
    {
        NetSurfaceDistance,
        NetCollision,
        UuvProximity,
        NetCollisionOrUuvProximity
    }

    private static readonly int ObiCollideEverythingFilter =
        ObiUtils.MakeFilter(ObiUtils.CollideWithEverything, 0);
    private static readonly int ObiCollideNothingFilter =
        ObiUtils.MakeFilter(ObiUtils.CollideWithNothing, 0);
    private static readonly int FallbackPreyObiFilter =
        ObiUtils.MakeFilter(1 << 0, 1);

    public ChaserAgent netter1;
    public ChaserAgent netter2;
    public ChaserAgent herder;
    public PreyAgent fish;
    [Header("Unified UUV Thruster Settings")]
    [Tooltip("是否由 CatchAreaManager 统一覆写全部 UUV 推进器的 maxThrust。")]
    [SerializeField] private bool useUnifiedThrusterMaxThrust = true;
    [Tooltip("全部 UUV 竖直推进器统一使用的 maxThrust。")]
    [SerializeField] private float unifiedVerticalThrusterMaxThrust = 150f;
    [Tooltip("全部 UUV 水平推进器统一使用的 maxThrust。")]
    [SerializeField] private float unifiedHorizontalThrusterMaxThrust = 400f;
    [Header("Terminal Rewards")]
    [Tooltip("追方成功收网后的终局奖励。应该明显大于单步 shaping 奖励。")]
    public float chaserCaptureReward = 25f;
    [Tooltip("鱼被捕获后的终局惩罚。绝对值通常与追方终局奖励同量级。")]
    public float fishCapturePenalty = -25f;
    [Header("Capture Conditions")]
    [Tooltip("选择捕获判定方式：网面距离、网碰撞、UUV接近，或碰撞/UUV接近二者之一。")]
    public CaptureCriterion captureCriterion = CaptureCriterion.NetSurfaceDistance;
    [Tooltip("鱼到实际变形网面的最近距离小于该值时，认为满足网面距离捕获条件。")]
    public float netSurfaceCaptureDistance = 0.8f;
    [Tooltip("网面距离捕获条件需要连续满足的时间。设为0可立即触发。")]
    public float netSurfaceCaptureHoldTime = 0.25f;
    [Tooltip("开启后使用鱼的Collider表面到网面的距离；关闭后使用鱼Transform位置到网面的距离。")]
    public bool useFishColliderForNetSurfaceDistance = true;
    [Tooltip("UUV接近判定的距离阈值，鱼到任一Netter小于该距离即可触发。")]
    public float uuvCaptureDistance = 1.0f;
    [Tooltip("开启后由CatchAreaManager在FixedUpdate中自动结算捕获，不依赖Agent动作回调。")]
    public bool autoResolveCaptureInFixedUpdate = true;
    [Tooltip("开启后自动给Prey的Collider添加ObiCollider，并监听渔网ObiSolver的Prey接触。")]
    public bool useObiNetCollisionDetection = true;
    [Tooltip("Prey缺少ObiCollider时自动添加到每个Prey Collider所在物体。")]
    public bool addMissingObiCollidersToFish = true;
    [Tooltip("Obi contact距离小于该值时认为Prey和渔网发生碰撞。0附近是接触，负数是穿透。")]
    public float obiNetCollisionDistanceThreshold = 0.02f;
    [Header("Reward Debug HUD")]
    [Tooltip("是否在屏幕左上角显示 reward 调试信息。")]
    public bool showRewardDebug = false;
    [Tooltip("是否显示 reward 相关的常驻屏幕 overlay。")]
    public bool showRewardDebugPersistentOverlays = false;
    [Tooltip("运行时切换 reward 调试显示的按键。")]
    public KeyCode toggleRewardDebugKey = KeyCode.F8;
    [Tooltip("调试面板左上角位置。")]
    public Vector2 rewardDebugPanelPosition = new Vector2(16f, 16f);
    [Tooltip("调试面板大小。内容过多时会自动启用滚动区域。")]
    public Vector2 rewardDebugPanelSize = new Vector2(980f, 440f);
    [Tooltip("调试面板字体大小。")]
    public int rewardDebugFontSize = 14;

    public Transform net; // 网根物体，包含所有网的子rigidbody
    private bool hasNetCollision = false; // 追踪网与鱼的碰撞状态
    private bool captureResolved = false;
    private readonly CatchAreaRewardDebugPanel rewardDebugPanel = new CatchAreaRewardDebugPanel();
    private string lastRewardEventMessage = "Waiting for runtime data...";
    private float lastRewardEventRealtime;
    private NetSurfaceModel netSurfaceModel;
    private FishNetGenerator fishNetGenerator;
    private float netSurfaceCaptureProgressSeconds;
    private bool netSurfaceCaptureQualified;
    private bool fishNearNetSurface;
    private float fishNetSurfaceDistance = float.PositiveInfinity;
    private Vector3 closestNetSurfacePoint;
    private Vector3 closestFishSurfacePoint;
    private Collider[] fishColliders = new Collider[0];
    private string netSurfaceCaptureDebug = "Not evaluated yet.";
    private ObiSolver observedObiSolver;
    private readonly HashSet<ObiColliderBase> fishObiColliders = new HashSet<ObiColliderBase>();
    private bool hasObiNetCollision;
    private string obiNetCollisionDebug = "Not evaluated yet.";
    private bool netActorCollisionFiltersApplied;
    private int lastDomainRandomizationFrame = -1;

    public bool HasNetCollision => hasNetCollision;
    public bool CaptureResolved => captureResolved;
    public string LastRewardEventMessage => lastRewardEventMessage;
    public float LastRewardEventAgeSeconds => Mathf.Max(0f, Time.realtimeSinceStartup - lastRewardEventRealtime);
    public NetSurfaceModel NetSurfaceModel => netSurfaceModel;
    public float CaptureProgress01 => netSurfaceCaptureHoldTime <= 0f
        ? (netSurfaceCaptureQualified ? 1f : 0f)
        : Mathf.Clamp01(netSurfaceCaptureProgressSeconds / netSurfaceCaptureHoldTime);
    public CaptureCriterion ActiveCaptureCriterion => captureCriterion;
    public bool NetSurfaceCaptureQualified => netSurfaceCaptureQualified;
    public bool FishNearNetSurface => fishNearNetSurface;
    public float FishNetSurfaceDistance => fishNetSurfaceDistance;
    public Vector3 ClosestNetSurfacePoint => closestNetSurfacePoint;
    public Vector3 ClosestFishSurfacePoint => closestFishSurfacePoint;
    public string NetSurfaceCaptureDebug => netSurfaceCaptureDebug;
    public bool HasObiNetCollision => hasObiNetCollision;
    public string ObiNetCollisionDebug => obiNetCollisionDebug;
    public float NearestNetterFishDistance => GetNearestNetterFishDistance();
    public bool FishWithinUuvCaptureDistance => GetNearestNetterFishDistance() <= Mathf.Max(0f, uuvCaptureDistance);
    public bool NetCollisionOrUuvProximityQualified => hasNetCollision || FishWithinUuvCaptureDistance;

    public bool UseUnifiedThrusterMaxThrust
    {
        get => useUnifiedThrusterMaxThrust;
        set
        {
            useUnifiedThrusterMaxThrust = value;
            ApplyThrusterSettingsToAllChasers();
        }
    }

    public float UnifiedVerticalThrusterMaxThrust
    {
        get => unifiedVerticalThrusterMaxThrust;
        set
        {
            unifiedVerticalThrusterMaxThrust = Mathf.Max(0f, value);
            ApplyThrusterSettingsToAllChasers();
        }
    }

    public float UnifiedHorizontalThrusterMaxThrust
    {
        get => unifiedHorizontalThrusterMaxThrust;
        set
        {
            unifiedHorizontalThrusterMaxThrust = Mathf.Max(0f, value);
            ApplyThrusterSettingsToAllChasers();
        }
    }

    // 存储初始状态的字段
    private Vector3 netter1InitialPos;
    private Quaternion netter1InitialRot;
    private Vector3 netter2InitialPos;
    private Quaternion netter2InitialRot;
    private Vector3 herderInitialPos;
    private Quaternion herderInitialRot;
    private Vector3 fishInitialPos;
    private Quaternion fishInitialRot;
    private Vector3 netInitialPos;
    private Quaternion netInitialRot;

    // 存储网的所有子物体的初始状态
    private struct RigidbodyInitialState
    {
        public Vector3 position;
        public Quaternion rotation;
    }
    private RigidbodyInitialState[] netChildrenInitialStates;

    private void Awake()
    {
        ResolveFishNetGenerator();
        if (fishNetGenerator != null)
        {
            fishNetGenerator.areaManager = this;
            fishNetGenerator.EnsureStaticGeneratedNetReadyForRuntime();
            if (fishNetGenerator.generateOnStart)
            {
                fishNetGenerator.GenerateNet();
            }

            if (fishNetGenerator.NetRoot != null)
            {
                net = fishNetGenerator.NetRoot;
            }
        }

        ResolveNetControllers();
        // 保存所有物体的初始状态
        SaveInitialStates();
        RecordRewardEvent("Area initialized.");
    }

    private void OnValidate()
    {
        chaserCaptureReward = Mathf.Max(0f, chaserCaptureReward);
        netSurfaceCaptureDistance = Mathf.Max(0f, netSurfaceCaptureDistance);
        netSurfaceCaptureHoldTime = Mathf.Max(0f, netSurfaceCaptureHoldTime);
        uuvCaptureDistance = Mathf.Max(0f, uuvCaptureDistance);
        obiNetCollisionDistanceThreshold = Mathf.Max(0f, obiNetCollisionDistanceThreshold);
    }

    private void OnDisable()
    {
        UnsubscribeObiSolverCollision();
    }

    private void Start()
    {
        ResolveFishNetGenerator();
        if (fishNetGenerator != null)
        {
            fishNetGenerator.EnsureStaticGeneratedNetReadyForRuntime();
            if (fishNetGenerator.generateOnStart && fishNetGenerator.NetRoot == null)
            {
                fishNetGenerator.areaManager = this;
                fishNetGenerator.GenerateNet();
            }

            if (fishNetGenerator.NetRoot != null)
            {
                net = fishNetGenerator.NetRoot;
            }
        }

        ResolveNetControllers();
        ApplyThrusterSettingsToAllChasers();
    }

    private void ResolveFishNetGenerator()
    {
        if (fishNetGenerator != null)
        {
            return;
        }

        fishNetGenerator = GetComponent<FishNetGenerator>();
        if (fishNetGenerator == null)
        {
            fishNetGenerator = FindFirstObjectByType<FishNetGenerator>();
        }
    }

    private void ResolveNetControllers()
    {
        if (net == null)
        {
            netSurfaceModel = null;
            UnsubscribeObiSolverCollision();
            return;
        }

        netSurfaceModel = net.GetComponent<NetSurfaceModel>();
        if (netSurfaceModel == null)
        {
            netSurfaceModel = net.gameObject.AddComponent<NetSurfaceModel>();
        }

        netSurfaceModel.ResolveHierarchy();
        ConfigureObiNetCollisionDetection();
    }

    private void Update()
    {
        if (InputSystemKeyBridge.WasPressedThisFrame(toggleRewardDebugKey))
        {
            showRewardDebug = !showRewardDebug;
            RecordRewardEvent($"Reward HUD {(showRewardDebug ? "enabled" : "disabled")}.");
        }
    }

    private void FixedUpdate()
    {
        long finsSimProfileStart = FinsSimRuntimeProfiler.Begin();
        UnityEngine.Profiling.Profiler.BeginSample("FinsSim.CatchAreaManager.FixedUpdate");
        try
        {
            if (useObiNetCollisionDetection)
            {
                long ensureFiltersProfileStart = FinsSimRuntimeProfiler.Begin();
                UnityEngine.Profiling.Profiler.BeginSample("FinsSim.CatchAreaManager.EnsureNetActorCollisionFilters");
                try
                {
                    EnsureNetActorCollisionFilters();
                }
                finally
                {
                    UnityEngine.Profiling.Profiler.EndSample();
                    FinsSimRuntimeProfiler.End("FinsSim.CatchAreaManager.EnsureNetActorCollisionFilters", ensureFiltersProfileStart);
                }
            }

            long updateNetSurfaceProfileStart = FinsSimRuntimeProfiler.Begin();
            UnityEngine.Profiling.Profiler.BeginSample("FinsSim.CatchAreaManager.UpdateNetSurfaceCaptureState");
            try
            {
                UpdateNetSurfaceCaptureState(Time.fixedDeltaTime);
            }
            finally
            {
                UnityEngine.Profiling.Profiler.EndSample();
                FinsSimRuntimeProfiler.End("FinsSim.CatchAreaManager.UpdateNetSurfaceCaptureState", updateNetSurfaceProfileStart);
            }

            if (autoResolveCaptureInFixedUpdate)
            {
                long resolveCaptureProfileStart = FinsSimRuntimeProfiler.Begin();
                UnityEngine.Profiling.Profiler.BeginSample("FinsSim.CatchAreaManager.TryResolveCapture");
                try
                {
                    TryResolveCapture();
                }
                finally
                {
                    UnityEngine.Profiling.Profiler.EndSample();
                    FinsSimRuntimeProfiler.End("FinsSim.CatchAreaManager.TryResolveCapture", resolveCaptureProfileStart);
                }
            }
        }
        finally
        {
            UnityEngine.Profiling.Profiler.EndSample();
            FinsSimRuntimeProfiler.End("FinsSim.CatchAreaManager.FixedUpdate", finsSimProfileStart);
        }
    }

    private void SaveInitialStates()
    {
        // 保存各个Agent的初始状态
        netter1InitialPos = netter1.transform.localPosition;
        netter1InitialRot = netter1.transform.localRotation;

        netter2InitialPos = netter2.transform.localPosition;
        netter2InitialRot = netter2.transform.localRotation;

        herderInitialPos = herder.transform.localPosition;
        herderInitialRot = herder.transform.localRotation;

        fishInitialPos = fish.transform.localPosition;
        fishInitialRot = fish.transform.localRotation;

        if (net == null)
        {
            netChildrenInitialStates = new RigidbodyInitialState[0];
            return;
        }

        // 保存网的所有子rigidbody的初始状态
        netInitialPos = net.localPosition;
        netInitialRot = net.localRotation;
        Rigidbody[] netChildRigidbodies = net.GetComponentsInChildren<Rigidbody>(true);
        netChildrenInitialStates = new RigidbodyInitialState[netChildRigidbodies.Length];
        for (int i = 0; i < netChildRigidbodies.Length; i++)
        {
            netChildrenInitialStates[i].position = netChildRigidbodies[i].transform.localPosition;
            netChildrenInitialStates[i].rotation = netChildRigidbodies[i].transform.localRotation;
        }
    }

    public void SetUnifiedThrusterMaxThrust(float verticalMaxThrust, float horizontalMaxThrust)
    {
        unifiedVerticalThrusterMaxThrust = Mathf.Max(0f, verticalMaxThrust);
        unifiedHorizontalThrusterMaxThrust = Mathf.Max(0f, horizontalMaxThrust);
        ApplyThrusterSettingsToAllChasers();
    }

    public void SetUnifiedThrusterMaxThrust(float maxThrust)
    {
        SetUnifiedThrusterMaxThrust(maxThrust, maxThrust);
    }

    public void ApplyThrusterSettingsToAllChasers()
    {
        if (!useUnifiedThrusterMaxThrust)
        {
            return;
        }

        ApplyThrusterSettingsToChaser(netter1);
        ApplyThrusterSettingsToChaser(netter2);
        ApplyThrusterSettingsToChaser(herder);
    }

    public void ApplyThrusterSettingsToChaser(ChaserAgent agent)
    {
        if (!useUnifiedThrusterMaxThrust || agent == null)
        {
            return;
        }

        agent.ApplyUnifiedThrusterSettings(
            unifiedVerticalThrusterMaxThrust,
            unifiedHorizontalThrusterMaxThrust);
    }

    public void ResetArea()
    {
        // Debug.Log("Resetting area and agents to initial positions."); //由于三个Chaser都调用！所以一次重置发送三条reset信息！

        // 恢复 netter1
        netter1.transform.localPosition = netter1InitialPos;
        netter1.transform.localRotation = netter1InitialRot;
        netter1.GetComponent<Rigidbody>().linearVelocity = Vector3.zero;
        netter1.GetComponent<Rigidbody>().angularVelocity = Vector3.zero;

        // 恢复 netter2
        netter2.transform.localPosition = netter2InitialPos;
        netter2.transform.localRotation = netter2InitialRot;
        netter2.GetComponent<Rigidbody>().linearVelocity = Vector3.zero;
        netter2.GetComponent<Rigidbody>().angularVelocity = Vector3.zero;

        // 恢复 herder
        herder.transform.localPosition = herderInitialPos;
        herder.transform.localRotation = herderInitialRot;
        herder.GetComponent<Rigidbody>().linearVelocity = Vector3.zero;
        herder.GetComponent<Rigidbody>().angularVelocity = Vector3.zero;

        // 恢复 fish
        fish.transform.localPosition = fishInitialPos;
        fish.transform.localRotation = fishInitialRot;
        fish.GetComponent<Rigidbody>().linearVelocity = Vector3.zero;
        fish.GetComponent<Rigidbody>().angularVelocity = Vector3.zero;

        if (fishNetGenerator != null && fishNetGenerator.ResetNetForEpisode())
        {
            net = fishNetGenerator.NetRoot;
        }
        else if (net != null)
        {
            // 恢复 net
            net.localPosition = netInitialPos;
            net.localRotation = netInitialRot;
            Rigidbody netRb = net.GetComponent<Rigidbody>();
            if (netRb != null)
            {
                netRb.linearVelocity = Vector3.zero;
                netRb.angularVelocity = Vector3.zero;
            }

            // 恢复网的所有子rigidbody
            Rigidbody[] netChildRigidbodies = net.GetComponentsInChildren<Rigidbody>(true);
            for (int i = 0; i < netChildRigidbodies.Length && i < netChildrenInitialStates.Length; i++)
            {
                netChildRigidbodies[i].transform.localPosition = netChildrenInitialStates[i].position;
                netChildRigidbodies[i].transform.localRotation = netChildrenInitialStates[i].rotation;
                netChildRigidbodies[i].linearVelocity = Vector3.zero;
                netChildRigidbodies[i].angularVelocity = Vector3.zero;
            }
        }

        // 重置碰撞状态
        hasNetCollision = false;
        hasObiNetCollision = false;
        obiNetCollisionDebug = "Episode reset.";
        captureResolved = false;
        ResolveNetControllers();
        ResetNetSurfaceCaptureState();
        RecordRewardEvent("Episode reset.");
    }

    /// <summary>
    /// Each of the three ML-Agents calls <see cref="ResetArea"/> at the start
    /// of its episode. Keep scene-local DR to one sample per physics frame so
    /// all actors in a replicated training area share the same parameters.
    /// </summary>
    public void RandomizeDomainForEpisode()
    {
        if (lastDomainRandomizationFrame == Time.frameCount)
        {
            return;
        }

        DomainRandomizationCoordinator coordinator =
            GetComponentInParent<DomainRandomizationCoordinator>();
        if (coordinator == null)
        {
            return;
        }

        lastDomainRandomizationFrame = Time.frameCount;
        coordinator.RandomizeForEpisode();
    }

    public Vector3 GetNetCenter()
    {
        if (netSurfaceModel != null && netSurfaceModel.IsReady())
        {
            return netSurfaceModel.GetNetCenter();
        }

        // 返回两个拉网者在世界坐标中的几何中点，这就是“鱼网”的中心
        if (netter1 == null && netter2 == null)
        {
            return transform.position;
        }

        if (netter1 == null)
        {
            return netter2.transform.position;
        }

        if (netter2 == null)
        {
            return netter1.transform.position;
        }

        return (netter1.transform.position + netter2.transform.position) * 0.5f;
    }

    public bool CheckCapture()
    {
        ResolveNetControllers();
        UpdateNetSurfaceCaptureState(0f);
        return EvaluateCaptureDetected();
    }

    public bool TryResolveCapture()
    {
        ResolveNetControllers();
        UpdateNetSurfaceCaptureState(0f);
        bool captureDetected = EvaluateCaptureDetected();
        if (!captureDetected || captureResolved)
        {
            return false;
        }

        captureResolved = true;

        if (netter1 != null) netter1.SetReward(chaserCaptureReward);
        if (netter2 != null) netter2.SetReward(chaserCaptureReward);
        if (herder != null) herder.SetReward(chaserCaptureReward);
        if (fish != null) fish.SetReward(fishCapturePenalty);

        RecordRewardEvent(
            $"Capture resolved. Chasers {chaserCaptureReward:F3} each, fish {fishCapturePenalty:F3}.");

        if (netter1 != null) netter1.EndEpisode();
        if (netter2 != null) netter2.EndEpisode();
        if (herder != null) herder.EndEpisode();
        if (fish != null) fish.EndEpisode();

        return true;
    }

    private bool EvaluateCaptureDetected()
    {
        switch (captureCriterion)
        {
            case CaptureCriterion.NetCollision:
                return hasNetCollision;

            case CaptureCriterion.UuvProximity:
                return FishWithinUuvCaptureDistance;

            case CaptureCriterion.NetCollisionOrUuvProximity:
                return hasNetCollision || FishWithinUuvCaptureDistance;

            case CaptureCriterion.NetSurfaceDistance:
            default:
                return netSurfaceCaptureQualified;
        }
    }

    private void UpdateNetSurfaceCaptureState(float deltaTime)
    {
        if (captureResolved)
        {
            return;
        }

        if (netSurfaceModel == null)
        {
            ResolveNetControllers();
        }

        if (netSurfaceModel == null || fish == null)
        {
            ResetNetSurfaceCaptureState(netSurfaceModel == null ? "No NetSurfaceModel on current net." : "No fish reference.");
            return;
        }

        if (!TryMeasureFishToNetSurfaceDistance(out float distance, out Vector3 netPoint, out Vector3 fishPoint))
        {
            ResetNetSurfaceCaptureState(
                $"Could not measure net surface distance. Grid {netSurfaceModel.SurfaceColumnCount}x{netSurfaceModel.SurfaceRowCount}, ready={netSurfaceModel.IsReady()}.");
            return;
        }

        fishNetSurfaceDistance = distance;
        closestNetSurfacePoint = netPoint;
        closestFishSurfacePoint = fishPoint;
        fishNearNetSurface = fishNetSurfaceDistance <= Mathf.Max(0f, netSurfaceCaptureDistance);

        if (!fishNearNetSurface)
        {
            netSurfaceCaptureProgressSeconds = 0f;
            netSurfaceCaptureQualified = false;
            netSurfaceCaptureDebug =
                $"Too far from net surface: {fishNetSurfaceDistance:F3} > {Mathf.Max(0f, netSurfaceCaptureDistance):F3}. Grid {netSurfaceModel.SurfaceColumnCount}x{netSurfaceModel.SurfaceRowCount}.";
            return;
        }

        netSurfaceCaptureProgressSeconds += Mathf.Max(0f, deltaTime);
        netSurfaceCaptureDebug =
            $"Near net surface: {fishNetSurfaceDistance:F3} <= {Mathf.Max(0f, netSurfaceCaptureDistance):F3}. Hold {netSurfaceCaptureProgressSeconds:F3}/{Mathf.Max(0f, netSurfaceCaptureHoldTime):F3}. Grid {netSurfaceModel.SurfaceColumnCount}x{netSurfaceModel.SurfaceRowCount}.";
        if (netSurfaceCaptureHoldTime <= 0f || netSurfaceCaptureProgressSeconds >= netSurfaceCaptureHoldTime)
        {
            netSurfaceCaptureQualified = true;
        }
    }

    private bool TryMeasureFishToNetSurfaceDistance(out float distance, out Vector3 netPoint, out Vector3 fishPoint)
    {
        distance = 0f;
        netPoint = Vector3.zero;
        fishPoint = Vector3.zero;

        Vector3 referencePoint = fish.transform.position;
        fishPoint = referencePoint;

        for (int i = 0; i < 3; ++i)
        {
            if (!netSurfaceModel.TryGetClosestSurfacePoint(fishPoint, out netPoint, out distance))
            {
                return false;
            }

            if (!useFishColliderForNetSurfaceDistance)
            {
                return true;
            }

            Collider collider = GetClosestFishCollider(netPoint);
            if (collider == null)
            {
                fishPoint = referencePoint;
                distance = Vector3.Distance(fishPoint, netPoint);
                return true;
            }

            Vector3 nextFishPoint = collider.ClosestPoint(netPoint);
            if ((nextFishPoint - fishPoint).sqrMagnitude < 0.000001f)
            {
                fishPoint = nextFishPoint;
                distance = Vector3.Distance(fishPoint, netPoint);
                return true;
            }

            fishPoint = nextFishPoint;
        }

        if (!netSurfaceModel.TryGetClosestSurfacePoint(fishPoint, out netPoint, out distance))
        {
            return false;
        }

        if (useFishColliderForNetSurfaceDistance)
        {
            Collider collider = GetClosestFishCollider(netPoint);
            if (collider != null)
            {
                fishPoint = collider.ClosestPoint(netPoint);
                distance = Vector3.Distance(fishPoint, netPoint);
            }
        }

        return true;
    }

    private Collider GetClosestFishCollider(Vector3 worldPoint)
    {
        if (fish == null)
        {
            return null;
        }

        if (fishColliders == null || fishColliders.Length == 0)
        {
            fishColliders = fish.GetComponentsInChildren<Collider>();
        }

        Collider closestCollider = null;
        float closestSqrDistance = float.PositiveInfinity;
        for (int i = 0; i < fishColliders.Length; ++i)
        {
            Collider collider = fishColliders[i];
            if (collider == null || !collider.enabled)
            {
                continue;
            }

            Vector3 closestPoint = collider.ClosestPoint(worldPoint);
            float sqrDistance = (closestPoint - worldPoint).sqrMagnitude;
            if (sqrDistance < closestSqrDistance)
            {
                closestSqrDistance = sqrDistance;
                closestCollider = collider;
            }
        }

        return closestCollider;
    }

    private void ResetNetSurfaceCaptureState(string debugReason = "Reset.")
    {
        netSurfaceCaptureProgressSeconds = 0f;
        netSurfaceCaptureQualified = false;
        fishNearNetSurface = false;
        fishNetSurfaceDistance = float.PositiveInfinity;
        closestNetSurfacePoint = Vector3.zero;
        closestFishSurfacePoint = Vector3.zero;
        fishColliders = new Collider[0];
        netSurfaceCaptureDebug = debugReason;
    }

    private float GetNearestNetterFishDistance()
    {
        if (fish == null)
        {
            return float.PositiveInfinity;
        }

        float nearestDistance = float.PositiveInfinity;
        Vector3 fishPosition = fish.transform.position;

        if (netter1 != null)
        {
            nearestDistance = Mathf.Min(nearestDistance, Vector3.Distance(fishPosition, netter1.transform.position));
        }

        if (netter2 != null)
        {
            nearestDistance = Mathf.Min(nearestDistance, Vector3.Distance(fishPosition, netter2.transform.position));
        }

        return nearestDistance;
    }

    // 当鱼与网发生碰撞时调用
    public void OnNetCollisionWithFish()
    {
        hasNetCollision = true;
        RecordRewardEvent("Net collision with fish detected (debug collision only).");
    }

    private void ConfigureObiNetCollisionDetection()
    {
        if (!useObiNetCollisionDetection || net == null)
        {
            UnsubscribeObiSolverCollision();
            return;
        }

        EnsureFishObiColliders();

        ObiSolver solver = net.GetComponentInChildren<ObiSolver>(true);
        if (solver == null)
        {
            UnsubscribeObiSolverCollision();
            obiNetCollisionDebug = "No ObiSolver found under current net.";
            return;
        }

        solver.collisionConstraintParameters.enabled = true;
        solver.PushSolverParameters();

        if (observedObiSolver == solver)
        {
            return;
        }

        UnsubscribeObiSolverCollision();
        observedObiSolver = solver;
        netActorCollisionFiltersApplied = false;
        observedObiSolver.OnCollision += Solver_OnObiCollision;
        obiNetCollisionDebug = "Listening for Obi net collisions.";
    }

    private void UnsubscribeObiSolverCollision()
    {
        if (observedObiSolver != null)
        {
            observedObiSolver.OnCollision -= Solver_OnObiCollision;
            observedObiSolver = null;
        }

        netActorCollisionFiltersApplied = false;
    }

    private void EnsureNetActorCollisionFilters()
    {
        if (netActorCollisionFiltersApplied || net == null)
        {
            return;
        }

        ObiActor[] actors = net.GetComponentsInChildren<ObiActor>(true);
        bool generatorCollisionSettingsReady = true;
        int ropeFilter = ObiCollideEverythingFilter;
        int netPinFilter = ObiCollideNothingFilter;
        ResolveFishNetGenerator();
        if (fishNetGenerator != null)
        {
            generatorCollisionSettingsReady = fishNetGenerator.ApplyRuntimeCollisionSettingsToCurrentNet();
            ropeFilter = fishNetGenerator.GetRopeParticleFilter();
            netPinFilter = fishNetGenerator.GetNetPinColliderFilter();
        }

        ObiColliderBase[] netPinColliders = net.GetComponentsInChildren<ObiColliderBase>(true);
        for (int i = 0; i < netPinColliders.Length; ++i)
        {
            if (netPinColliders[i] != null)
            {
                netPinColliders[i].Filter = netPinFilter;
            }
        }

        if (actors.Length == 0)
        {
            return;
        }

        bool allLoaded = true;
        for (int i = 0; i < actors.Length; ++i)
        {
            ObiActor actor = actors[i];
            if (actor == null || !actor.isLoaded)
            {
                allLoaded = false;
                continue;
            }

            actor.SetFilterCategory(ObiUtils.GetCategoryFromFilter(ropeFilter));
            actor.SetFilterMask(ObiUtils.GetMaskFromFilter(ropeFilter));
        }

        netActorCollisionFiltersApplied = allLoaded && generatorCollisionSettingsReady;
    }

    private void EnsureFishObiColliders()
    {
        fishObiColliders.Clear();
        if (fish == null)
        {
            obiNetCollisionDebug = "No fish reference for Obi collision detection.";
            return;
        }

        ResolveFishNetGenerator();
        Collider[] colliders = fish.GetComponentsInChildren<Collider>(true);
        for (int i = 0; i < colliders.Length; ++i)
        {
            Collider unityCollider = colliders[i];
            if (unityCollider == null || !unityCollider.enabled)
            {
                continue;
            }

            ObiCollider obiCollider = unityCollider.GetComponent<ObiCollider>();
            if (obiCollider == null && addMissingObiCollidersToFish)
            {
                obiCollider = unityCollider.gameObject.AddComponent<ObiCollider>();
                obiCollider.sourceCollider = unityCollider;
            }

            if (obiCollider == null)
            {
                continue;
            }

            obiCollider.Filter = fishNetGenerator != null
                ? fishNetGenerator.GetPreyColliderFilter()
                : FallbackPreyObiFilter;
            fishObiColliders.Add(obiCollider);
        }

        obiNetCollisionDebug = fishObiColliders.Count > 0
            ? $"Registered {fishObiColliders.Count} Prey ObiCollider(s)."
            : "No Prey ObiCollider found. Enable Add Missing Obi Colliders To Fish or add them manually.";
    }

    private void Solver_OnObiCollision(ObiSolver solver, ObiNativeContactList contacts)
    {
        if (!useObiNetCollisionDetection || fishObiColliders.Count == 0)
        {
            return;
        }

        ObiColliderWorld world = ObiColliderWorld.GetInstance();
        if (world == null || world.colliderHandles == null)
        {
            obiNetCollisionDebug = "ObiColliderWorld is not ready yet.";
            return;
        }

        int contactCount = contacts.count;
        for (int i = 0; i < contactCount; ++i)
        {
            Oni.Contact contact = contacts[i];
            if (contact.distance > obiNetCollisionDistanceThreshold)
            {
                continue;
            }

            if (contact.bodyB < 0 || contact.bodyB >= world.colliderHandles.Count)
            {
                continue;
            }

            ObiColliderHandle handle = world.colliderHandles[contact.bodyB];
            ObiColliderBase contactCollider = handle != null ? handle.owner : null;
            if (contactCollider != null && fishObiColliders.Contains(contactCollider))
            {
                hasObiNetCollision = true;
                hasNetCollision = true;
                obiNetCollisionDebug =
                    $"Obi net collision with {contactCollider.name}, distance={contact.distance:F4}, contacts={contactCount}.";
                RecordRewardEvent("Obi net collision with fish detected.");
                return;
            }
        }

        obiNetCollisionDebug = $"No Prey Obi contact this frame. Contacts={contactCount}, prey colliders={fishObiColliders.Count}.";
    }

    // 可选：重置碰撞状态的方法
    public void ResetNetCollision()
    {
        hasNetCollision = false;
        hasObiNetCollision = false;
        obiNetCollisionDebug = "Net collision flag reset.";
        captureResolved = false;
        ResetNetSurfaceCaptureState();
        RecordRewardEvent("Net collision flag reset.");
    }

    private void RecordRewardEvent(string message)
    {
        lastRewardEventMessage = message;
        lastRewardEventRealtime = Time.realtimeSinceStartup;
    }

    private void OnGUI()
    {
        if (showRewardDebug)
        {
            rewardDebugPanel.Draw(this);
        }

        if (showRewardDebugPersistentOverlays)
        {
            rewardDebugPanel.DrawPersistentOverlays(this);
        }
    }
}
