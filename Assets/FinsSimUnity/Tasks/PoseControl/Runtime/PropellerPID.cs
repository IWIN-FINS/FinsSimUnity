using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Sensors;
using Unity.MLAgents.Actuators;
using NWH.DWP2.ShipController;

// PID控制器类（保留核心逻辑，移除PWM相关）
[System.Serializable]
public class PIDRegulator
{
    public float Kp;          // 比例系数
    public float Ki;          // 积分系数
    public float Kd;          // 微分系数
    public float MaxOutput;   // 最大输出
    public float MinOutput;   // 最小输出
    public float MaxIntegral; // 积分上限
    public float MinIntegral; // 积分下限

    private float integral;   // 积分值
    private float lastError;  // 上一次误差
    private float lastTime;   // 上一次计算时间

    // 初始化PID参数
    public PIDRegulator(float kp, float ki, float kd, float maxOut, float minOut, float maxInt, float minInt)
    {
        Kp = kp;
        Ki = ki;
        Kd = kd;
        MaxOutput = maxOut;
        MinOutput = minOut;
        MaxIntegral = maxInt;
        MinIntegral = minInt;
        integral = 0;
        lastError = 0;
        lastTime = Time.time;
    }

    // PID计算核心方法
    public float PIDCalc(float target, float current)
    {
        float currentTime = Time.time;
        float dt = currentTime - lastTime;
        if (dt < 0.001f) dt = 0.001f;

        // 计算误差
        float error = target - current;

        // 比例项
        float pTerm = Kp * error;

        // 积分项（限幅）
        integral += error * dt * Ki;
        integral = Mathf.Clamp(integral, MinIntegral, MaxIntegral);
        float iTerm = integral;

        // 微分项（避免除0）
        float dTerm = Kd * (error - lastError) / dt;

        // 总输出（限幅）
        float output = pTerm + iTerm + dTerm;
        output = Mathf.Clamp(output, MinOutput, MaxOutput);

        // 更新状态
        lastError = error;
        lastTime = currentTime;

        return output;
    }

    // 重置PID状态
    public void Reset()
    {
        integral = 0;
        lastError = 0;
        lastTime = Time.time;
    }
}

// 推力分量结构体（直接输出油门分量，而非PWM）
public struct ThrottleComponent
{
    public float Depth;   // 深度油门分量
    public float Roll;    // 横滚油门分量
    public float Pitch;   // 俯仰油门分量
    public float Yaw;     // 偏航油门分量
}

public class PropellerPID : Agent
{
    // 核心组件
    private Rigidbody rigidBody;
    private AdvancedShipController advancedShipController;
    private List<Engine> engines;
    
    // 目标与状态
    public Transform Target;
    private float targetDepth = 30f;       // 目标深度（Unity中Y轴向下为正，取-Y）
    private float targetRoll = 0f;         // 目标横滚角（°）
    private float targetPitch = 0f;        // 目标俯仰角（°）
    private float targetYaw = 0f;          // 目标偏航角（°）
    private float currentDepth;            // 当前深度（-transform.position.y）
    private Vector3 currentEulerAngles;    // 当前欧拉角（°）
    private Vector3 currentAngularVelocity;// 当前角速度（°/s）

    // 推进器配置（仅保留正反逻辑，移除PWM）
    private float[] Sign = { 1, -1, 1, 1, -1, -1, 1, -1 }; // 正反桨（影响油门方向）
    private int[] VerticalEnginesIdx = { 0, 1, 2, 3 };     // 垂直推进器索引（对应Vertical1-4）
    private int[] HorizontalEnginesIdx = { 4, 5, 6, 7 };   // 水平推进器索引（对应Horizontal1-4）

    // PID控制器（双环结构，输出直接映射到[-1,1]）
    private PIDRegulator DepthPID;
    private PIDRegulator RollOutPID;   // 横滚外环（角度→角速度）
    private PIDRegulator RollInPID;    // 横滚内环（角速度→油门）
    private PIDRegulator PitchOutPID;  // 俯仰外环
    private PIDRegulator PitchInPID;   // 俯仰内环
    private PIDRegulator YawOutPID;    // 偏航外环
    private PIDRegulator YawInPID;     // 偏航内环
    private ThrottleComponent throttleComponent; // 油门分量

    // 控制标志
    private bool isControlEnabled = true;     // 控制使能
    private int loopCounter = 0;              // 双环频率控制计数器
    private bool useHeuristicMode = true;     // 强制Heuristic模式（用于本地调试）

    // 碰撞相关
    private bool hasCollisionThisFrame = false;
    private float collisionPenalty = -0.5f;

    // 初始化
    void Start()
    {
        // 组件初始化
        rigidBody = GetComponent<Rigidbody>();
        advancedShipController = GetComponent<AdvancedShipController>();
        engines = advancedShipController.engines;
        
        // 启用外部油门控制
        foreach (var engine in engines)
        {
            engine.useExternalThrottleInput = true;
        }

        // 初始化默认PID参数（输出范围直接为[-1,1]）
        InitPID(
            0.1f, 0.001f, 0.05f,  // DepthPID (Kp,Ki,Kd)
            0.05f, 0.001f, 0.02f, // RollOutPID
            0.1f, 0.001f, 0.05f,  // RollInPID
            0.05f, 0.001f, 0.02f, // PitchOutPID
            0.1f, 0.001f, 0.05f,  // PitchInPID
            0.05f, 0.001f, 0.02f, // YawOutPID
            0.1f, 0.001f, 0.05f   // YawInPID
        );
        
        // 改为false进行RL训练，true进行手动测试
        useHeuristicMode = false;
    }



    // 初始化/更新PID参数（核心：RL输出的参数传入这里）
    private void InitPID(float depthKp, float depthKi, float depthKd,
                         float rollOutKp, float rollOutKi, float rollOutKd,
                         float rollInKp, float rollInKi, float rollInKd,
                         float pitchOutKp, float pitchOutKi, float pitchOutKd,
                         float pitchInKp, float pitchInKi, float pitchInKd,
                         float yawOutKp, float yawOutKi, float yawOutKd,
                         float yawInKp, float yawInKi, float yawInKd)
    {
        // 所有PID输出范围直接设为[-1,1]，适配Unity油门
        float pidMax = 1f;
        float pidMin = -1f;
        float intMax = 0.5f;
        float intMin = -0.5f;

        // 深度PID（单环）
        DepthPID = new PIDRegulator(depthKp, depthKi, depthKd, pidMax, pidMin, intMax, intMin);


        // 横滚双环
        RollOutPID = new PIDRegulator(rollOutKp, rollOutKi, rollOutKd, pidMax, pidMin, intMax, intMin);
        RollInPID = new PIDRegulator(rollInKp, rollInKi, rollInKd, pidMax, pidMin, intMax, intMin);

        // 俯仰双环
        PitchOutPID = new PIDRegulator(pitchOutKp, pitchOutKi, pitchOutKd, pidMax, pidMin, intMax, intMin);
        PitchInPID = new PIDRegulator(pitchInKp, pitchInKi, pitchInKd, pidMax, pidMin, intMax, intMin);

        // 偏航双环
        YawOutPID = new PIDRegulator(yawOutKp, yawOutKi, yawOutKd, pidMax, pidMin, intMax, intMin);
        YawInPID = new PIDRegulator(yawInKp, yawInKi, yawInKd, pidMax, pidMin, intMax, intMin);
    }

    // 回合开始重置
    public override void OnEpisodeBegin()
    {
        FinsROVAgentRuntime.RandomizeEpisodeIfPresent(this);

        // 物理状态重置
        rigidBody.angularVelocity = Vector3.zero;
        rigidBody.linearVelocity = Vector3.zero;
        transform.localPosition = new Vector3( Random.value * 2, 0, Random.value * 2);

        // 状态重置
        targetDepth = 30f;
        targetRoll = 0f;
        targetPitch = 0f;
        targetYaw = transform.eulerAngles.y;
        hasCollisionThisFrame = false;
        loopCounter = 0;

        // PID状态重置
        DepthPID?.Reset();
        RollOutPID?.Reset();
        RollInPID?.Reset();
        PitchOutPID?.Reset();
        PitchInPID?.Reset();
        YawOutPID?.Reset();
        YawInPID?.Reset();

        // 推进器油门归零
        foreach (var engine in engines)
        {
            engine.externalThrottleInput = 0f;
        }
    }

    // 收集观察数据（供RL决策）
    public override void CollectObservations(VectorSensor sensor)
    {
        // 1. 目标状态（深度+姿态）
        sensor.AddObservation(targetDepth);
        sensor.AddObservation(targetRoll);
        sensor.AddObservation(targetPitch);
        sensor.AddObservation(targetYaw);

        // 2. 当前状态（Unity仿真直接读取）
        currentDepth = -transform.position.y; // Y轴向上，深度取-Y
        currentEulerAngles = transform.eulerAngles;
        currentAngularVelocity = rigidBody.angularVelocity * Mathf.Rad2Deg; // 转°/s

        // 欧拉角归一化到[-180, 180]
        float roll = NormalizeAngle(currentEulerAngles.x);
        float pitch = NormalizeAngle(currentEulerAngles.y);
        float yaw = NormalizeAngle(currentEulerAngles.z);

        sensor.AddObservation(currentDepth);    // 当前深度
        sensor.AddObservation(roll);            // 当前横滚（°）
        sensor.AddObservation(pitch);           // 当前俯仰（°）
        sensor.AddObservation(yaw);             // 当前偏航（°）
        sensor.AddObservation(currentAngularVelocity.x); // 横滚角速度（°/s）
        sensor.AddObservation(currentAngularVelocity.y); // 俯仰角速度（°/s）
        sensor.AddObservation(currentAngularVelocity.z); // 偏航角速度（°/s）

        // 3. 误差（RL核心感知项）
        sensor.AddObservation(targetDepth - currentDepth);
        sensor.AddObservation(NormalizeAngle(targetRoll - roll));
        sensor.AddObservation(NormalizeAngle(targetPitch - pitch));
        sensor.AddObservation(NormalizeAngle(targetYaw - yaw));

        // 总观察维度：4(目标)+7(当前)+4(误差) = 15维
    }

    // 处理RL动作（核心：RL输出30维PID参数，直接映射到合理范围）
    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        if (!isControlEnabled) return;
        
        // 如果在Heuristic模式下，跳过（由Update中的HandleHeuristicControl处理）
        if (useHeuristicMode) return;

        // RL模式：解析RL输出的30维连续动作
        var actions = actionBuffers.ContinuousActions;
        float[] pidParams = new float[21]; // 7组PID×3参数
        for (int i = 0; i < 21 && i < actions.Length; i++)
        {
            pidParams[i] = MapActionToPID(actions[i], i);
        }

        // 更新PID参数
        InitPID(
            pidParams[0], pidParams[1], pidParams[2],    // DepthPID Kp/Ki/Kd
            pidParams[3], pidParams[4], pidParams[5],    // RollOutPID
            pidParams[6], pidParams[7], pidParams[8],    // RollInPID
            pidParams[9], pidParams[10], pidParams[11],  // PitchOutPID
            pidParams[12], pidParams[13], pidParams[14], // PitchInPID
            pidParams[15], pidParams[16], pidParams[17], // YawOutPID
            pidParams[18], pidParams[19], pidParams[20]  // YawInPID
        );

        // 双环PID计算油门分量
        CalculatePIDThrottle();

        // 油门分配到8个推进器
        AllocateThrottleToEngines();

        // 计算奖励
        CalculateReward();
    }

    // RL动作值映射到PID参数范围（适配Unity仿真的PID参数范围）
    private float MapActionToPID(float action, int paramIndex)
    {
        // 按参数类型划分范围（避免PID参数过大/过小）
        //这里后续可以考虑引入更复杂的映射策略，例如非线性映射或分段映射，以更好地适应不同参数的敏感度和作用范围
        //或者采用极点配置的方式，保证极点在左半平面，提高系统稳定性和响应速度
        switch (paramIndex % 3)
        {
            case 0: // Kp：映射到[0, 0.5]（仿真环境PID增益无需太大）
                return (action + 1) * 0.25f;
            case 1: // Ki：映射到[0, 0.01]（积分项避免过大）
                return (action + 1) * 0.005f;
            case 2: // Kd：映射到[0, 0.2]（微分项避免震荡）
                return (action + 1) * 0.1f;
            default:
                return 0;
        }
    }

    // 双环PID计算油门分量（适配Unity仿真）
    private void CalculatePIDThrottle()
    {
        // 深度PID计算（单环，输出[-1,1]油门）
        throttleComponent.Depth = DepthPID.PIDCalc(targetDepth, currentDepth);

        // 双环频率控制（3:1，外环每3帧计算一次）
        loopCounter++;
        bool isOuterLoop = loopCounter % 3 == 0;
        float targetRollV = 0, targetPitchV = 0, targetYawV = 0;

        // 欧拉角归一化到[-180, 180]
        float roll = NormalizeAngle(currentEulerAngles.x);
        float pitch = NormalizeAngle(currentEulerAngles.y);
        float yaw = NormalizeAngle(currentEulerAngles.z);

        // 外环（角度→角速度目标，仅每3帧计算）
        if (isOuterLoop)
        {
            targetRollV = RollOutPID.PIDCalc(targetRoll, roll);
            targetPitchV = PitchOutPID.PIDCalc(targetPitch, pitch);
            float yawError = NormalizeAngle(targetYaw - yaw);
            targetYawV = YawOutPID.PIDCalc(0, yawError);
        }

        // 内环（角速度→油门，每帧计算）
        throttleComponent.Roll = RollInPID.PIDCalc(targetRollV, currentAngularVelocity.x);
        throttleComponent.Pitch = PitchInPID.PIDCalc(targetPitchV, currentAngularVelocity.y);
        throttleComponent.Yaw = YawInPID.PIDCalc(targetYawV, currentAngularVelocity.z);
    }

    // 油门分配到8个推进器（直接输出[-1,1]，无PWM转换）
    private void AllocateThrottleToEngines()
    {
        // 1. 垂直推进器（Vertical1-4）油门分配
        float[,] verticalWeights = {
            {-1, -1, -1},  // Vertical1: 深度-横滚-俯仰
            {-1, -1, 1},   // Vertical2: 深度-横滚+俯仰
            {-1, 1, -1},   // Vertical3: 深度+横滚-俯仰
            {-1, 1, 1}     // Vertical4: 深度+横滚+俯仰
        };

        // 垂直推进器主要控制深度，同时受横滚/俯仰影响，油门 = 深度分量×权重 + 横滚分量×权重 + 俯仰分量×权重
        for (int i = 0; i < 4; i++)
        {
            if (i >= engines.Count) break; // 防止索引越界
            // 计算油门 = 深度分量×权重 + 横滚分量×权重 + 俯仰分量×权重
            float throttle =
                throttleComponent.Depth * verticalWeights[i, 0] +
                throttleComponent.Roll * verticalWeights[i, 1] +
                throttleComponent.Pitch * verticalWeights[i, 2];
            // 正反桨修正 + 限幅到[-1,1]
            throttle *= Sign[VerticalEnginesIdx[i]];
            engines[VerticalEnginesIdx[i]].externalThrottleInput = Mathf.Clamp(throttle, -1f, 1f);
        }

        // 2. 水平推进器（Horizontal1-4）油门分配
        float[,] horizontalWeights = {
            {1, 1},   // Horizontal1: 偏航+
            {1, 1},   // Horizontal2: 偏航+
            {-1, -1}, // Horizontal3: 偏航-
            {-1, -1}  // Horizontal4: 偏航-
        };

        // 水平推进器主要控制偏航，油门 = 偏航分量×权重
        for (int i = 0; i < 4; i++)
        {
            int engineIdx = HorizontalEnginesIdx[i];
            if (engineIdx >= engines.Count) break;
            // 主要控制偏航，输出[-1,1]
            float throttle = throttleComponent.Yaw * horizontalWeights[i, 0];
            throttle *= Sign[engineIdx];
            engines[engineIdx].externalThrottleInput = Mathf.Clamp(throttle, -1f, 1f);
        }
    }

    // 奖励计算（适配仿真训练）
    private void CalculateReward()
    {
        // 实时更新当前状态（确保最新数据）
        currentDepth = -transform.position.z;  // Y轴向上，深度取-Y
        currentEulerAngles = transform.eulerAngles;  // 欧拉角
        currentAngularVelocity = rigidBody.angularVelocity * Mathf.Rad2Deg;  // 角速度转°/s
        
        float reward = 0;
        float distanceToTarget = Vector3.Distance(transform.position, Target.position);

        // 1. 深度控制奖励（误差越小奖励越高）
        float depthError = Mathf.Abs(targetDepth - currentDepth);
        reward += Mathf.Max(0, 1 - depthError / 10); // 误差<10时奖励为正

        // 2. 姿态平稳奖励（横滚/俯仰越接近0奖励越高）
        float rollError = Mathf.Abs(NormalizeAngle(currentEulerAngles.x));
        float pitchError = Mathf.Abs(NormalizeAngle(currentEulerAngles.y));
        reward += Mathf.Max(0, 1 - (rollError + pitchError) / 90); // 误差<90°时奖励为正

        // 3. 位置接近奖励（离目标越近奖励越高）
        reward += Mathf.Max(0, 2 - distanceToTarget / 2); // 距离<4时奖励为正

        // 4. 动作平滑奖励（惩罚角速度，避免震荡）
        reward += Mathf.Max(0, 0.5f - rigidBody.angularVelocity.magnitude / 10);

        // 5. 碰撞惩罚
        if (hasCollisionThisFrame)
        {
            reward += collisionPenalty;
            hasCollisionThisFrame = false;
        }

        // 6. 时间惩罚（鼓励快速完成）
        reward -= 0.001f;

        // 7. 完成任务奖励（深度误差<0.5且距离<0.02）
        if (distanceToTarget < 0.02f && depthError < 0.5f)
        {
            reward += 10f; // 大幅奖励
            EndEpisode();
        }

        // 设置奖励
        SetReward(reward);
        
        // 仅在调试时输出，避免性能问题
        if (Time.frameCount % 100 == 0) // 每100帧输出一次
        {
            Debug.Log($"=== Frame {Time.frameCount} ===");
            Debug.Log($"Reward: {reward:F3} ");
            
            // 输出PID参数
            //Debug.Log($"DepthPID - Kp: {DepthPID.Kp:F4}, Ki: {DepthPID.Ki:F4}, Kd: {DepthPID.Kd:F4}");
            //Debug.Log($"RollOutPID - Kp: {RollOutPID.Kp:F4}, Ki: {RollOutPID.Ki:F4}, Kd: {RollOutPID.Kd:F4}");
            //Debug.Log($"RollInPID - Kp: {RollInPID.Kp:F4}, Ki: {RollInPID.Ki:F4}, Kd: {RollInPID.Kd:F4}");
            //Debug.Log($"PitchOutPID - Kp: {PitchOutPID.Kp:F4}, Ki: {PitchOutPID.Ki:F4}, Kd: {PitchOutPID.Kd:F4}");
            //Debug.Log($"PitchInPID - Kp: {PitchInPID.Kp:F4}, Ki: {PitchInPID.Ki:F4}, Kd: {PitchInPID.Kd:F4}");
            //Debug.Log($"YawOutPID - Kp: {YawOutPID.Kp:F4}, Ki: {YawOutPID.Ki:F4}, Kd: {YawOutPID.Kd:F4}");
            //Debug.Log($"YawInPID - Kp: {YawInPID.Kp:F4}, Ki: {YawInPID.Ki:F4}, Kd: {YawInPID.Kd:F4}");
        }
    }

    // 角度归一化到[-180, 180]（适配Unity欧拉角）
    private float NormalizeAngle(float angle)
    {
        angle = Mathf.Repeat(angle + 180f, 360f);
        if (angle < 0) angle += 360f;
        return angle - 180f;
    }

    void Update()
    {
        if (useHeuristicMode)
        {
            // 每帧调用键盘控制
            HandleKeyboardControl();
        }
    }
    
    // 键盘直接控制引擎
    private void HandleKeyboardControl()
    {
        FinsROVManualInputState inputState = FinsROVAgentRuntime.ReadKeyboardManualInput();
        float throttle = inputState.Surge;
        float throttle2 = inputState.Heave;
        float steering = inputState.Yaw;

        // ===== Vertical推进器 (Vertical1-4) =====
        // 这些推进器主要用于：上下潜（深度控制）
        Engine vertical1 = GetEngineByName("Vertical1");
        Engine vertical2 = GetEngineByName("Vertical2");
        Engine vertical3 = GetEngineByName("Vertical3");
        Engine vertical4 = GetEngineByName("Vertical4");
             
        vertical1.externalThrottleInput = Mathf.Clamp(throttle2, -1f, 1f);
        vertical2.externalThrottleInput = Mathf.Clamp(throttle2, -1f, 1f);
        vertical3.externalThrottleInput = Mathf.Clamp(throttle2, -1f, 1f);
        vertical4.externalThrottleInput = Mathf.Clamp(throttle2, -1f, 1f);



        // ===== Horizontal推进器 (Horizontal1-4) =====
        //水平推进器主要用于：前进/后退（油门）和转向（偏航控制）
        Engine horizontal1 = GetEngineByName("Horizontal1");
        Engine horizontal2 = GetEngineByName("Horizontal2");
        Engine horizontal3 = GetEngineByName("Horizontal3");
        Engine horizontal4 = GetEngineByName("Horizontal4");
        

        horizontal1.externalThrottleInput = Mathf.Clamp(throttle, -1f, 1f) + Mathf.Clamp(steering, -1f, 1f);
        horizontal2.externalThrottleInput = Mathf.Clamp(throttle, -1f, 1f) + Mathf.Clamp(steering, -1f, 1f);
        horizontal3.externalThrottleInput = Mathf.Clamp(-throttle, -1f, 1f) + Mathf.Clamp(steering, -1f, 1f);
        horizontal4.externalThrottleInput = Mathf.Clamp(-throttle, -1f, 1f) + Mathf.Clamp(steering, -1f, 1f);

        

        // 计算奖励用于调试
        CalculateReward();
    }

    // 根据引擎名字获取引擎对象
    private Engine GetEngineByName(string engineName)
    {
        return engines.Find(engine => engine.name == engineName);
    }

    // 手动控制（调试用，直接输出[-1,1]油门）
    //public override void Heuristic(in ActionBuffers actionsOut)
    //{
    //var continuousActions = actionsOut.ContinuousActions;
    // 手动控制时使用默认PID参数，动作置0
    //for (int i = 0; i < continuousActions.Length; i++)
    //{
    //continuousActions[i] = 0;
    //}

    // 可选：手动调试推进器油门（示例）
    // continuousActions[0] = Input.GetAxis("Vertical"); // 深度
    // continuousActions[3] = Input.GetAxis("Horizontal"); // 横滚
    //}

    // 在训练期间或调试时，通过键盘输入手动控制代理的行为
    //public override void Heuristic(in ActionBuffers actionsOut)
    //{
        //var continuousActionsOut = actionsOut.ContinuousActions;
        
        // 读取键盘输入
        //float throttle = inputState.Surge;
        //float throttle2 = inputState.Heave;
        //float throttle3 = inputState.Aux3;
        //float throttle4 = inputState.Aux4;
        //float steering = inputState.Yaw;

        // 输出30维PID参数（全为0表示使用默认参数）
        //for (int i = 0; i < continuousActionsOut.Length; i++)
        //{
            //continuousActionsOut[i] = 0f;
        //}
    //}

    // 碰撞检测
    private void OnCollisionEnter(Collision collision)
    {
        hasCollisionThisFrame = true;
    }

    private void OnCollisionStay(Collision collision)
    {
        hasCollisionThisFrame = true;
    }
}
