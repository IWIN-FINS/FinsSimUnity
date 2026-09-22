using System;
using System.Collections.Generic;
using NWH.DWP2.ShipController;
using UnityEngine;

[DisallowMultipleComponent]
[RequireComponent(typeof(Rigidbody))]
public class UuvDirectionalSpeedTester : MonoBehaviour
{
    private enum TestDirection
    {
        Idle,
        Forward,
        Backward,
        Left,
        Right,
        Up,
        Down
    }

    private struct DirectionStats
    {
        public float maxDirectionalSpeed;
        public float maxTotalSpeed;
    }

    [Header("Runtime Control")]
    [Tooltip("启用组件后自动开始测试。")]
    public bool autoStart = true;
    [Tooltip("是否自动循环测试前后左右上下六个方向。")]
    public bool autoCycleDirections = true;
    [Tooltip("运行时切换开始 / 停止测试。")]
    public KeyCode toggleRunKey = KeyCode.F5;
    [Tooltip("运行时切换屏幕 HUD 显示。")]
    public KeyCode toggleHudKey = KeyCode.F6;
    [Tooltip("手动切到下一个测试方向。")]
    public KeyCode nextDirectionKey = KeyCode.Period;
    [Tooltip("手动切到上一个测试方向。")]
    public KeyCode previousDirectionKey = KeyCode.Comma;
    [Tooltip("运行时重置所有最大速度统计。")]
    public KeyCode resetStatsKey = KeyCode.F4;

    [Header("Sequence")]
    [Tooltip("每个方向持续施加推力的时间。")]
    public float directionDurationSeconds = 5f;
    [Tooltip("两个方向之间的空挡时间，用于让速度衰减。")]
    public float neutralDurationSeconds = 1.5f;
    [Tooltip("切换方向时是否把线速度清零，方便测单方向最大速度。")]
    public bool zeroLinearVelocityOnSwitch = true;
    [Tooltip("切换方向时是否把角速度清零，减少耦合干扰。")]
    public bool zeroAngularVelocityOnSwitch = true;

    [Header("Throttle")]
    [Range(0f, 1f)]
    [Tooltip("前后左右测试使用的水平推进器推力幅值。")]
    public float horizontalThrottle = 1f;
    [Range(0f, 1f)]
    [Tooltip("上下测试使用的竖直推进器推力幅值。")]
    public float verticalThrottle = 1f;

    [Header("Ownership")]
    [Tooltip("启用本脚本时，暂时禁用同物体上的其他已知控制脚本，避免抢写推进器。")]
    public bool disableOtherControlScriptsWhileActive = true;

    [Header("HUD")]
    [Tooltip("是否在屏幕左上角显示测试状态和最大速度统计。")]
    public bool showHud = true;
    [Tooltip("是否持续统计各方向最大速度。关闭后只下发测试推力，不维护统计量。")]
    public bool collectDirectionStats = true;
    public Vector2 hudPosition = new Vector2(20f, 20f);
    public Vector2 hudSize = new Vector2(520f, 360f);
    public int hudFontSize = 14;

    private static readonly TestDirection[] Sequence =
    {
        TestDirection.Forward,
        TestDirection.Backward,
        TestDirection.Left,
        TestDirection.Right,
        TestDirection.Up,
        TestDirection.Down
    };

    private readonly Dictionary<TestDirection, DirectionStats> directionStats =
        new Dictionary<TestDirection, DirectionStats>();
    private readonly List<MonoBehaviour> disabledControllers = new List<MonoBehaviour>();

    private Rigidbody rb;
    private AdvancedShipController advancedShipController;
    private List<Engine> engines;
    private GUIStyle panelStyle;
    private GUIStyle titleStyle;
    private GUIStyle bodyStyle;
    private Texture2D panelTexture;
    private bool isRunning;
    private bool isNeutralPhase;
    private int sequenceIndex;
    private float phaseStartTime;
    private TestDirection currentDirection = TestDirection.Idle;

    private void Awake()
    {
        rb = GetComponent<Rigidbody>();
        advancedShipController = GetComponent<AdvancedShipController>();
        engines = advancedShipController != null ? advancedShipController.engines : null;
        EnsureExternalThrottleMode();
        InitializeStats();
    }

    private void OnEnable()
    {
        EnsureExternalThrottleMode();
        if (Application.isPlaying && disableOtherControlScriptsWhileActive)
        {
            DisableKnownControllers();
        }

        if (Application.isPlaying && autoStart)
        {
            StartTest();
        }
    }

    private void OnDisable()
    {
        StopTest();
        RestoreDisabledControllers();
    }

    private void Update()
    {
        if (!Application.isPlaying)
        {
            return;
        }

        if (InputSystemKeyBridge.WasPressedThisFrame(toggleRunKey))
        {
            if (isRunning)
            {
                StopTest();
            }
            else
            {
                StartTest();
            }
        }

        if (InputSystemKeyBridge.WasPressedThisFrame(toggleHudKey))
        {
            showHud = !showHud;
        }

        if (InputSystemKeyBridge.WasPressedThisFrame(resetStatsKey))
        {
            ResetStats();
        }

        if (!isRunning)
        {
            return;
        }

        if (autoCycleDirections)
        {
            AdvanceSequenceIfNeeded();
            return;
        }

        if (InputSystemKeyBridge.WasPressedThisFrame(nextDirectionKey))
        {
            MoveToSequenceOffset(+1);
        }

        if (InputSystemKeyBridge.WasPressedThisFrame(previousDirectionKey))
        {
            MoveToSequenceOffset(-1);
        }
    }

    private void FixedUpdate()
    {
        if (!Application.isPlaying)
        {
            return;
        }

        EnsureExternalThrottleMode();
        ApplyCurrentDirectionThrust();
        if (collectDirectionStats)
        {
            UpdateStats();
        }
    }

    [ContextMenu("Start Directional Speed Test")]
    public void StartTest()
    {
        isRunning = true;
        isNeutralPhase = false;
        sequenceIndex = 0;
        SwitchToDirection(Sequence[sequenceIndex]);
    }

    [ContextMenu("Stop Directional Speed Test")]
    public void StopTest()
    {
        isRunning = false;
        currentDirection = TestDirection.Idle;
        isNeutralPhase = false;
        ClearAllThrusters();
    }

    [ContextMenu("Reset Directional Speed Stats")]
    public void ResetStats()
    {
        InitializeStats();
    }

    private void InitializeStats()
    {
        directionStats.Clear();
        for (int i = 0; i < Sequence.Length; i++)
        {
            directionStats[Sequence[i]] = default;
        }
    }

    private void AdvanceSequenceIfNeeded()
    {
        float phaseDuration = isNeutralPhase ? neutralDurationSeconds : directionDurationSeconds;
        if (Time.time - phaseStartTime < Mathf.Max(0.01f, phaseDuration))
        {
            return;
        }

        if (!isNeutralPhase)
        {
            isNeutralPhase = true;
            currentDirection = TestDirection.Idle;
            phaseStartTime = Time.time;
            ClearAllThrusters();
            if (zeroLinearVelocityOnSwitch)
            {
                rb.linearVelocity = Vector3.zero;
            }

            if (zeroAngularVelocityOnSwitch)
            {
                rb.angularVelocity = Vector3.zero;
            }

            return;
        }

        isNeutralPhase = false;
        sequenceIndex = (sequenceIndex + 1) % Sequence.Length;
        SwitchToDirection(Sequence[sequenceIndex]);
    }

    private void MoveToSequenceOffset(int offset)
    {
        int nextIndex = sequenceIndex + offset;
        if (nextIndex < 0)
        {
            nextIndex += Sequence.Length;
        }

        sequenceIndex = nextIndex % Sequence.Length;
        isNeutralPhase = false;
        SwitchToDirection(Sequence[sequenceIndex]);
    }

    private void SwitchToDirection(TestDirection direction)
    {
        currentDirection = direction;
        phaseStartTime = Time.time;
        ClearAllThrusters();

        if (zeroLinearVelocityOnSwitch)
        {
            rb.linearVelocity = Vector3.zero;
        }

        if (zeroAngularVelocityOnSwitch)
        {
            rb.angularVelocity = Vector3.zero;
        }
    }

    private void EnsureExternalThrottleMode()
    {
        if (engines == null)
        {
            return;
        }

        foreach (Engine engine in engines)
        {
            if (engine != null)
            {
                engine.useExternalThrottleInput = true;
            }
        }
    }

    private void ApplyCurrentDirectionThrust()
    {
        ClearAllThrusters();
        if (!isRunning || currentDirection == TestDirection.Idle)
        {
            return;
        }

        float h = Mathf.Clamp(horizontalThrottle, 0f, 1f);
        float v = Mathf.Clamp(verticalThrottle, 0f, 1f);

        switch (currentDirection)
        {
            case TestDirection.Forward:
                SetHorizontalPattern(+h, +h, -h, -h);
                break;
            case TestDirection.Backward:
                SetHorizontalPattern(-h, -h, +h, +h);
                break;
            case TestDirection.Left:
                SetHorizontalPattern(-h, +h, -h, +h);
                break;
            case TestDirection.Right:
                SetHorizontalPattern(+h, -h, +h, -h);
                break;
            case TestDirection.Up:
                SetVerticalPattern(+v, +v, +v, +v);
                break;
            case TestDirection.Down:
                SetVerticalPattern(-v, -v, -v, -v);
                break;
        }
    }

    private void SetVerticalPattern(float v1, float v2, float v3, float v4)
    {
        SetEngineInput("Vertical1", v1);
        SetEngineInput("Vertical2", v2);
        SetEngineInput("Vertical3", v3);
        SetEngineInput("Vertical4", v4);
    }

    private void SetHorizontalPattern(float h1, float h2, float h3, float h4)
    {
        SetEngineInput("Horizontal1", h1);
        SetEngineInput("Horizontal2", h2);
        SetEngineInput("Horizontal3", h3);
        SetEngineInput("Horizontal4", h4);
    }

    private void SetEngineInput(string engineName, float value)
    {
        Engine engine = GetEngineByName(engineName);
        if (engine != null)
        {
            engine.externalThrottleInput = Mathf.Clamp(value, -1f, 1f);
        }
    }

    private Engine GetEngineByName(string engineName)
    {
        if (engines == null)
        {
            return null;
        }

        return engines.Find(engine => engine != null && engine.name == engineName);
    }

    private void ClearAllThrusters()
    {
        if (engines == null)
        {
            return;
        }

        foreach (Engine engine in engines)
        {
            if (engine != null)
            {
                engine.externalThrottleInput = 0f;
            }
        }
    }

    private void UpdateStats()
    {
        if (!isRunning || currentDirection == TestDirection.Idle)
        {
            return;
        }

        Vector3 velocity = rb.linearVelocity;
        float directionalSpeed = Mathf.Max(0f, Vector3.Dot(velocity, GetMeasurementAxis(currentDirection)));
        float totalSpeed = velocity.magnitude;

        DirectionStats stats = directionStats[currentDirection];
        stats.maxDirectionalSpeed = Mathf.Max(stats.maxDirectionalSpeed, directionalSpeed);
        stats.maxTotalSpeed = Mathf.Max(stats.maxTotalSpeed, totalSpeed);
        directionStats[currentDirection] = stats;
    }

    private Vector3 GetMeasurementAxis(TestDirection direction)
    {
        Vector3 surgeAxis = Vector3.ProjectOnPlane(transform.right, Vector3.up);
        if (surgeAxis.sqrMagnitude < 1e-4f)
        {
            surgeAxis = Vector3.right;
        }
        surgeAxis.Normalize();

        Vector3 swayAxis = Vector3.ProjectOnPlane(transform.forward, Vector3.up);
        if (swayAxis.sqrMagnitude < 1e-4f)
        {
            swayAxis = Vector3.forward;
        }
        swayAxis.Normalize();

        switch (direction)
        {
            case TestDirection.Forward: return surgeAxis;
            case TestDirection.Backward: return -surgeAxis;
            case TestDirection.Left: return -swayAxis;
            case TestDirection.Right: return swayAxis;
            case TestDirection.Up: return Vector3.up;
            case TestDirection.Down: return Vector3.down;
            default: return Vector3.zero;
        }
    }

    private float GetCurrentDirectionalSpeed()
    {
        if (currentDirection == TestDirection.Idle)
        {
            return 0f;
        }

        return Mathf.Max(0f, Vector3.Dot(rb.linearVelocity, GetMeasurementAxis(currentDirection)));
    }

    private void DisableKnownControllers()
    {
        disabledControllers.Clear();
        DisableControllerIfNeeded(GetComponent<ChaserAgent>());
        DisableControllerIfNeeded(GetComponent<Pursuer>());
        DisableControllerIfNeeded(GetComponent<RollerAgent>());
    }

    private void DisableControllerIfNeeded(MonoBehaviour controller)
    {
        if (controller == null || !controller.enabled)
        {
            return;
        }

        controller.enabled = false;
        disabledControllers.Add(controller);
    }

    private void RestoreDisabledControllers()
    {
        for (int i = 0; i < disabledControllers.Count; i++)
        {
            if (disabledControllers[i] != null)
            {
                disabledControllers[i].enabled = true;
            }
        }

        disabledControllers.Clear();
    }

    private void OnGUI()
    {
        if (!showHud)
        {
            return;
        }

        EnsureHudStyles();
        Rect rect = new Rect(hudPosition.x, hudPosition.y, hudSize.x, hudSize.y);
        GUI.Box(rect, GUIContent.none, panelStyle);

        GUILayout.BeginArea(new Rect(rect.x + 12f, rect.y + 12f, rect.width - 24f, rect.height - 24f));
        GUILayout.Label("UUV Directional Speed Tester", titleStyle);
        GUILayout.Label(
            $"Running: {(isRunning ? "Yes" : "No")}    Mode: {currentDirection}    Auto cycle: {(autoCycleDirections ? "Yes" : "No")}",
            bodyStyle);
        GUILayout.Label(
            $"Keys: Run {toggleRunKey}    HUD {toggleHudKey}    Reset {resetStatsKey}    Prev {previousDirectionKey}    Next {nextDirectionKey}",
            bodyStyle);
        GUILayout.Label(
            $"Current directional speed: {GetCurrentDirectionalSpeed():F2} m/s    Total speed: {rb.linearVelocity.magnitude:F2} m/s",
            bodyStyle);
        GUILayout.Space(8f);

        for (int i = 0; i < Sequence.Length; i++)
        {
            TestDirection direction = Sequence[i];
            DirectionStats stats = directionStats[direction];
            GUILayout.Label(
                $"{direction,-8} max directional = {stats.maxDirectionalSpeed:F2} m/s    max total = {stats.maxTotalSpeed:F2} m/s",
                bodyStyle);
        }

        GUILayout.Space(8f);
        GUILayout.Label(
            "Note: Left / Right uses an empirical horizontal-thruster pattern for sway testing. If the hull / thruster geometry cannot produce pure lateral motion, the measured side-speed will naturally stay low.",
            bodyStyle);
        GUILayout.EndArea();
    }

    private void EnsureHudStyles()
    {
        if (panelTexture == null)
        {
            panelTexture = new Texture2D(1, 1, TextureFormat.RGBA32, false);
            panelTexture.hideFlags = HideFlags.HideAndDontSave;
            panelTexture.SetPixel(0, 0, new Color(0.08f, 0.1f, 0.14f, 0.92f));
            panelTexture.Apply();
        }

        if (panelStyle == null)
        {
            panelStyle = new GUIStyle(GUI.skin.box);
            panelStyle.normal.background = panelTexture;
            panelStyle.padding = new RectOffset(12, 12, 12, 12);
        }

        if (titleStyle == null)
        {
            titleStyle = new GUIStyle(GUI.skin.label);
            titleStyle.fontStyle = FontStyle.Bold;
            titleStyle.normal.textColor = Color.white;
            titleStyle.wordWrap = true;
        }

        if (bodyStyle == null)
        {
            bodyStyle = new GUIStyle(GUI.skin.label);
            bodyStyle.normal.textColor = Color.white;
            bodyStyle.wordWrap = true;
        }

        titleStyle.fontSize = hudFontSize + 3;
        bodyStyle.fontSize = hudFontSize;
    }
}
