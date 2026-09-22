using UnityEngine;
using System.Collections.Generic;
using NWH.DWP2.ShipController;

/// <summary>
/// 用于测试潜器在各方向上能达到的最大速度
/// 支持多个 Engine 同时测试
/// </summary>
public class MaxSpeedTester : MonoBehaviour
{
    [Header("测试设置")]
    public float testDuration = 10f;      // 每个方向测试持续时间
    public float settleTime = 3f;         // 等待速度收敛的时间
    public float convergenceThreshold = 0.02f; // 收敛阈值 (m/s)

    [Header("调试")]
    public bool autoRun = false;          // 启动时自动开始测试
    public bool verbose = true;           // 详细输出

    AdvancedShipController sc;
    Rigidbody rb;
    List<Engine> engines = new List<Engine>();
    Vector3 originalPosition;
    Quaternion originalRotation;

    // 保存原始配置
    private List<float> originalThrottleInputs = new List<float>();

    // 测试结果
    [System.Serializable]
    public class MaxSpeedResult
    {
        public float maxVelocityX;
        public float maxVelocityY;
        public float maxVelocityZ;
        public float maxVelocityMagnitude;
        public float equilibriumLinearSpeedX;
        public float equilibriumLinearSpeedY;
        public float equilibriumLinearSpeedZ;

        public float maxAngularVelocityX;
        public float maxAngularVelocityY;
        public float maxAngularVelocityZ;
        public float maxAngularVelocityMagnitude;
        public float equilibriumAngularVelocityX;
        public float equilibriumAngularVelocityY;
        public float equilibriumAngularVelocityZ;
    }

    public MaxSpeedResult result = new();

    /// <summary>
    /// 测试组合结构体
    /// </summary>
    [System.Serializable]
    public class ThrustCombo
    {
        public string name;
        public float[] throttles; // 8个元素: {V1, V2, V3, V4, H1, H2, H3, H4}

        public ThrustCombo(string n, float[] t)
        {
            name = n;
            throttles = t;
        }
    }

    // 预定义测试组合
    // 索引: engines[0-3] = Vertical, engines[4-7] = Horizontal
    private ThrustCombo[] testCombinations = new ThrustCombo[]
    {
        new ThrustCombo("All Zero",       new float[]{0,0,0,0, 0,0,0,0}),
        new ThrustCombo("V All +1",       new float[]{1,1,1,1, 0,0,0,0}),  // 纯 Y+
        new ThrustCombo("V All -1",       new float[]{-1,-1,-1,-1, 0,0,0,0}), // 纯 Y-
        new ThrustCombo("H All +1",       new float[]{0,0,0,0, 1,1,1,1}),  // 预期纯 X+
        new ThrustCombo("H All -1",       new float[]{0,0,0,0, -1,-1,-1,-1}), // 预期纯 X-
        new ThrustCombo("X+ Dir",          new float[]{0,0,0,0, -1,-1,1,1}),  // 纯 X+
        new ThrustCombo("X- Dir",          new float[]{0,0,0,0, 1,1,-1,-1}),  // 纯 X-
        new ThrustCombo("H2+H3 (Z+?)",    new float[]{0,0,0,0, -1,1,1,-1}), // 待验证: 纯 Z+ ?
        new ThrustCombo("H1+H4 (Z-?)",   new float[]{0,0,0,0, 1,-1,-1,1}), // 待验证: 纯 Z- ?
        new ThrustCombo("All +1",         new float[]{1,1,1,1, 1,1,1,1}),  // 最大合力
        new ThrustCombo("All -1",         new float[]{-1,-1,-1,-1, -1,-1,-1,-1}), // 最大反向
    };

    void Start()
    {
        sc = GetComponent<AdvancedShipController>();
        rb = GetComponent<Rigidbody>();

        if (sc == null || rb == null)
        {
            Debug.LogError("MaxSpeedTester: 需要挂载在带有 AdvancedShipController 和 Rigidbody 的对象上");
            return;
        }

        if (sc.engines.Count == 0)
        {
            Debug.LogError("MaxSpeedTester: AdvancedShipController 没有配置引擎");
            return;
        }

        if (sc.engines.Count != 8)
        {
            Debug.LogWarning($"[MaxSpeedTester] 期望8个引擎,当前有 {sc.engines.Count} 个");
        }

        engines.AddRange(sc.engines);

        // 启用所有引擎的外部控制
        foreach (var engine in engines)
        {
            engine.useExternalThrottleInput = true;
        }

        originalPosition = transform.position;
        originalRotation = transform.rotation;

        // 验证引擎名称顺序
        if (verbose)
        {
            Debug.Log($"[MaxSpeedTester] 找到 {engines.Count} 个引擎:");
            for (int i = 0; i < engines.Count; i++)
            {
                var e = engines[i];
                Debug.Log($"  [{i}] {e.name}: thrustDir={e.thrustDirection}, thrustPos={e.thrustPosition}, maxThrust={e.maxThrust}");
            }
        }

        if (autoRun)
        {
            RunAllTests();
        }
    }

    /// <summary>
    /// 保存所有引擎的当前油门配置
    /// </summary>
    void SaveThrottleConfigs()
    {
        originalThrottleInputs.Clear();
        foreach (var engine in engines)
        {
            originalThrottleInputs.Add(engine.externalThrottleInput);
        }
    }

    /// <summary>
    /// 恢复所有引擎的油门配置
    /// </summary>
    void RestoreThrottleConfigs()
    {
        for (int i = 0; i < engines.Count && i < originalThrottleInputs.Count; i++)
        {
            engines[i].externalThrottleInput = originalThrottleInputs[i];
        }
    }

    /// <summary>
    /// 重置潜器状态
    /// </summary>
    void ResetState()
    {
        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
        transform.position = originalPosition;
        transform.rotation = originalRotation;

        foreach (var engine in engines)
        {
            engine.externalThrottleInput = 0f;
        }
    }

    /// <summary>
    /// 应用油门组合
    /// </summary>
    void ApplyThrottleCombo(ThrustCombo combo)
    {
        for (int i = 0; i < engines.Count && i < combo.throttles.Length; i++)
        {
            engines[i].externalThrottleInput = combo.throttles[i];
            if (combo.throttles[i] != 0)
            {
                engines[i].StartEngine();
            }
        }
    }

    /// <summary>
    /// 测试单个组合
    /// </summary>
    System.Collections.IEnumerator TestCombo(ThrustCombo combo)
    {
        if (verbose) Debug.Log($"\n=== 测试: {combo.name} ===");

        SaveThrottleConfigs();
        ResetState();
        yield return new WaitForSeconds(0.5f);

        ApplyThrottleCombo(combo);

        float elapsed = 0f;
        float updateInterval = 0.5f;
        float lastLogTime = 0f;

        Vector3 maxVelocity = Vector3.zero;
        Vector3 equilibriumVelocity = Vector3.zero;
        Vector3 maxAngularVelocity = Vector3.zero;
        Vector3 equilibriumAngularVelocity = Vector3.zero;
        bool hasSettled = false;
        float settleStartTime = 0f;

        while (elapsed < testDuration)
        {
            elapsed += Time.deltaTime;

            Vector3 currentVel = rb.linearVelocity;
            Vector3 currentAngVel = rb.angularVelocity;

            if (currentVel.magnitude > maxVelocity.magnitude)
                maxVelocity = currentVel;

            if (currentAngVel.magnitude > maxAngularVelocity.magnitude)
                maxAngularVelocity = currentAngVel;

            // 检测收敛
            if (!hasSettled && elapsed > 3f)
            {
                if (Mathf.Abs(currentVel.magnitude) > 0.5f)
                {
                    // 检查速度是否趋于稳定 (变化小于阈值)
                    if (Mathf.Abs(currentVel.x) < 0.02f && Mathf.Abs(currentVel.y) < 0.02f && Mathf.Abs(currentVel.z) < 0.02f
                    && Mathf.Abs(currentAngVel.x) < 0.02f && Mathf.Abs(currentAngVel.y) < 0.02f && Mathf.Abs(currentAngVel.z) < 0.02f)
                    {
                        if (settleStartTime == 0f)
                            settleStartTime = elapsed;
                        else if (elapsed - settleStartTime > 1f)
                        {
                            equilibriumVelocity = currentVel;
                            equilibriumAngularVelocity = currentAngVel;
                            hasSettled = true;
                        }
                    }
                    else
                    {
                        settleStartTime = 0f;
                    }
                }
            }

            if (verbose && Time.time - lastLogTime > updateInterval)
            {
                Debug.Log($"[{combo.name}] t={elapsed:F1}s | Vel: ({currentVel.x:F3}, {currentVel.y:F3}, {currentVel.z:F3}) | |V|={currentVel.magnitude:F3} | AngVel: ({currentAngVel.x:F3}, {currentAngVel.y:F3}, {currentAngVel.z:F3})");
                lastLogTime = Time.time;
            }

            yield return null;
        }

        // 如果没有检测到收敛，使用最后的速度作为稳态
        if (!hasSettled)
        {
            equilibriumVelocity = rb.linearVelocity;
            equilibriumAngularVelocity = rb.angularVelocity;
        }

        // 停止所有引擎
        foreach (var engine in engines)
        {
            engine.externalThrottleInput = 0f;
        }

        if (verbose)
        {
            Debug.Log($"--- {combo.name} 结果 ---");
            Debug.Log($"  最大线速度: ({maxVelocity.x:F3}, {maxVelocity.y:F3}, {maxVelocity.z:F3}) | |V|={maxVelocity.magnitude:F3}");
            Debug.Log($"  稳态线速度: ({equilibriumVelocity.x:F3}, {equilibriumVelocity.y:F3}, {equilibriumVelocity.z:F3}) | |V|={equilibriumVelocity.magnitude:F3}");
            Debug.Log($"  最大角速度: ({maxAngularVelocity.x:F3}, {maxAngularVelocity.y:F3}, {maxAngularVelocity.z:F3}) | |W|={maxAngularVelocity.magnitude:F3}");
            Debug.Log($"  稳态角速度: ({equilibriumAngularVelocity.x:F3}, {equilibriumAngularVelocity.y:F3}, {equilibriumAngularVelocity.z:F3}) | |W|={equilibriumAngularVelocity.magnitude:F3}");
        }

        RestoreThrottleConfigs();

        // 更新结果
        result.maxVelocityX = Mathf.Max(result.maxVelocityX, Mathf.Abs(maxVelocity.x));
        result.maxVelocityY = Mathf.Max(result.maxVelocityY, Mathf.Abs(maxVelocity.y));
        result.maxVelocityZ = Mathf.Max(result.maxVelocityZ, Mathf.Abs(maxVelocity.z));
        result.maxVelocityMagnitude = Mathf.Max(result.maxVelocityMagnitude, maxVelocity.magnitude);

        // 记录各方向的稳态速度（取绝对值最大的）
        if (Mathf.Abs(equilibriumVelocity.x) > 0.01f)
            result.equilibriumLinearSpeedX = Mathf.Abs(equilibriumVelocity.x);
        if (Mathf.Abs(equilibriumVelocity.y) > 0.01f)
            result.equilibriumLinearSpeedY = Mathf.Abs(equilibriumVelocity.y);
        if (Mathf.Abs(equilibriumVelocity.z) > 0.01f)
            result.equilibriumLinearSpeedZ = Mathf.Abs(equilibriumVelocity.z);

        // 更新角速度结果
        result.maxAngularVelocityX = Mathf.Max(result.maxAngularVelocityX, Mathf.Abs(maxAngularVelocity.x));
        result.maxAngularVelocityY = Mathf.Max(result.maxAngularVelocityY, Mathf.Abs(maxAngularVelocity.y));
        result.maxAngularVelocityZ = Mathf.Max(result.maxAngularVelocityZ, Mathf.Abs(maxAngularVelocity.z));
        result.maxAngularVelocityMagnitude = Mathf.Max(result.maxAngularVelocityMagnitude, maxAngularVelocity.magnitude);

        if (Mathf.Abs(equilibriumAngularVelocity.x) > 0.01f)
            result.equilibriumAngularVelocityX = Mathf.Abs(equilibriumAngularVelocity.x);
        if (Mathf.Abs(equilibriumAngularVelocity.y) > 0.01f)
            result.equilibriumAngularVelocityY = Mathf.Abs(equilibriumAngularVelocity.y);
        if (Mathf.Abs(equilibriumAngularVelocity.z) > 0.01f)
            result.equilibriumAngularVelocityZ = Mathf.Abs(equilibriumAngularVelocity.z);
    }

    /// <summary>
    /// 运行所有测试
    /// </summary>
    public void RunAllTests()
    {
        StartCoroutine(RunAllTestsCoroutine());
    }

    System.Collections.IEnumerator RunAllTestsCoroutine()
    {
        if (verbose)
        {
            Debug.Log("========== 开始最大速度测试 ==========");
            Debug.Log($"Rigidbody Mass: {rb.mass} kg");
            Debug.Log($"Rigidbody Linear Drag: {rb.linearDamping}");
            Debug.Log($"Rigidbody Angular Drag: {rb.angularDamping}");
            Debug.Log($"活跃引擎数量: {engines.Count}");
            foreach (var engine in engines)
            {
                Debug.Log($"  - {engine.name}: maxThrust={engine.maxThrust}N, maxSpeed={engine.maxSpeed}m/s");
            }
        }

        // 重置结果
        result = new MaxSpeedResult();

        // 测试每个组合
        foreach (var combo in testCombinations)
        {
            // 检查是否有足够的引擎
            if (combo.throttles.Length > engines.Count)
            {
                Debug.LogWarning($"[MaxSpeedTester] 组合 {combo.name} 需要 {combo.throttles.Length} 个引擎,当前只有 {engines.Count} 个,跳过");
                continue;
            }
            yield return TestCombo(combo);
        }

        // 输出最终总结
        PrintSummary();
    }

    void PrintSummary()
    {
        Debug.Log("\n============================================");
        Debug.Log("========== 最大速度测试结果总结 ==========");
        Debug.Log("============================================");
        Debug.Log("--- 线速度 ---");
        Debug.Log($"最大线速度 X分量: {result.maxVelocityX:F4} m/s");
        Debug.Log($"最大线速度 Y分量: {result.maxVelocityY:F4} m/s");
        Debug.Log($"最大线速度 Z分量: {result.maxVelocityZ:F4} m/s");
        Debug.Log($"最大线速度 (模): {result.maxVelocityMagnitude:F4} m/s");
        Debug.Log("");
        Debug.Log($"稳态线速度 X: {result.equilibriumLinearSpeedX:F4} m/s");
        Debug.Log($"稳态线速度 Y: {result.equilibriumLinearSpeedY:F4} m/s");
        Debug.Log($"稳态线速度 Z: {result.equilibriumLinearSpeedZ:F4} m/s");
        Debug.Log("");
        Debug.Log("--- 角速度 ---");
        Debug.Log($"最大角速度 X分量: {result.maxAngularVelocityX:F4} rad/s");
        Debug.Log($"最大角速度 Y分量: {result.maxAngularVelocityY:F4} rad/s");
        Debug.Log($"最大角速度 Z分量: {result.maxAngularVelocityZ:F4} rad/s");
        Debug.Log($"最大角速度 (模): {result.maxAngularVelocityMagnitude:F4} rad/s");
        Debug.Log("");
        Debug.Log($"稳态角速度 X: {result.equilibriumAngularVelocityX:F4} rad/s");
        Debug.Log($"稳态角速度 Y: {result.equilibriumAngularVelocityY:F4} rad/s");
        Debug.Log($"稳态角速度 Z: {result.equilibriumAngularVelocityZ:F4} rad/s");
        Debug.Log("");
        Debug.Log("--- 用于 RL 归一化的建议值 ---");
        Debug.Log($"maxVelocityX = {result.maxVelocityX:F2}m/s");
        Debug.Log($"maxVelocityY = {result.maxVelocityY:F2}m/s");
        Debug.Log($"maxVelocityZ = {result.maxVelocityZ:F2}m/s");
        Debug.Log($"maxAngularVelocityX = {result.maxAngularVelocityX:F2}rad/s");
        Debug.Log($"maxAngularVelocityY = {result.maxAngularVelocityY:F2}rad/s");
        Debug.Log($"maxAngularVelocityZ = {result.maxAngularVelocityZ:F2}rad/s");
        Debug.Log("============================================");
    }

    /// <summary>
    /// 在 Unity Editor 中手动调用测试
    /// </summary>
    [ContextMenu("Run Max Speed Tests")]
    void ContextMenuRunTests()
    {
        RunAllTests();
    }
}
