// 引入必要的程序包（类似工具箱）
using UnityEngine;               // Unity引擎核心功能
using Unity.MLAgents;           // ML-Agents机器学习框架核心
using Unity.MLAgents.Sensors;   // 用于Agent的观察功能
using Unity.MLAgents.Actuators; // 新增的命名空间
using FinsSim.Actuators;

// 定义RollerAgent类，继承自Agent基类（冒号表示继承）
public class ControlForPosition : Agent // 类名必须与文件名一致
{
    // 【字段声明】
    // Rigidbody类型变量：用于物理模拟的刚体组件
    // private表示仅本类可以访问（未写修饰符时默认为private）
    Rigidbody rigidBody;

    ThrusterController thrusterController;
    readonly Thruster[] orderedThrusters = new Thruster[FinsROVAgentRuntime.DefaultThrusterOrder.Length];
    readonly float[] currentActions = new float[FinsROVAgentRuntime.DefaultThrusterOrder.Length];

    // public表示可以在Unity编辑器中设置，Transform类型存储物体的位置/旋转/缩放信息

    public Transform selfTransform;       // 代理自身的位置信息，由于父物体本身transform有一个偏移，这边采用一个人为外部指定的transform标记潜器中心位置（实际上是父物体的一个空子物体）
    public Transform targetTransform;     // 目标物体的位置信息。使用一个没有collider的立方体作为目标位置的指示
    [Header("Thruster Control")]
    [SerializeField] ThrusterController thrusterControllerOverride;
    [SerializeField] ThrusterCommandMode thrusterCommandMode = ThrusterCommandMode.ScaledForceRequest;
    [ShowWhenThrusterCommandMode(ThrusterCommandMode.ScaledForceRequest)]
    [SerializeField] float actionForceScaleN = 7f;
    [SerializeField] bool autoResolveThrustersFromChildren = true;
    [SerializeField] bool logThrusterResolution = true;

    [SerializeField] bool autoRequestDecisionWhenNoDecisionRequester = true;
    bool hasDecisionRequester;

    // float表示浮点数（带小数），public变量会显示在Unity Inspector面板
    // public float forceMultiplier = 10; // 控制移动力度的放大系数

    // Start方法：Unity的初始化函数，在对象创建后第一帧更新前调用
    void Start()
    {
        // GetComponent<类型>() 获取当前物体上的指定类型组件
        // 这里获取Rigidbody组件并赋值给rigidBody变量
        rigidBody = GetComponent<Rigidbody>();
        hasDecisionRequester = GetComponent("DecisionRequester") != null;
        ResolveRuntimeReferences();
    }

    void FixedUpdate()
    {
        if (autoRequestDecisionWhenNoDecisionRequester && !hasDecisionRequester)
        {
            RequestDecision();
        }
    }

    // OnEpisodeBegin方法：ML-Agents的重写方法，每个训练回合开始时调用
    public override void OnEpisodeBegin()
    {
        FinsROVAgentRuntime.RandomizeEpisodeIfPresent(this);

        // 重置物理状态
        ResolveRuntimeReferences();
        rigidBody.angularVelocity = Vector3.zero; // 角速度归零
        rigidBody.linearVelocity = Vector3.zero;        // 线性速度归零
        FinsROVAgentRuntime.ZeroThrusters(orderedThrusters, thrusterCommandMode, actionForceScaleN);

        // 重置潜器位置到起始点附近 不必完全(0,0)，提升模型的泛化能力
        this.transform.position = new Vector3( //父物体修改重置位置，注意直接修改selfTransform.position可能会有偏移，因为selfTransform可能只移动子物体，导致行为异常
            Random.value * 2,
            Random.value * 2 - 2, // 保持在水面以下
            Random.value * 2
        );

        // 随机初始化目标位置：在半径1m的球面内，且不超过水面
        Vector3 randomTargetPos = GenerateRandomTargetPosition(radius: 3f, centerX: this.transform.position.x, centerY: this.transform.position.y, centerZ: this.transform.position.z);
        targetTransform.position = randomTargetPos;

        // 随机初始化目标朝向：仅头转向（yaw旋转）
        targetTransform.rotation = GenerateRandomHeading();

        // Debug.Log($"Target position: {targetTransform.position}, Target heading: {targetTransform.rotation.eulerAngles.y}°");
    }

    // CollectObservations方法：收集环境观察数据供AI学习
    public override void CollectObservations(VectorSensor sensor)
    {
        Transform reference = selfTransform != null
            ? selfTransform
            : FinsROVAgentRuntime.ResolveReferenceTransform(transform, selfTransform);
        Vector3 localTargetOffset = targetTransform != null
            ? reference.InverseTransformPoint(targetTransform.position)
            : Vector3.zero;
        Quaternion relativeTargetRotation = targetTransform != null
            ? Quaternion.Inverse(reference.rotation) * targetTransform.rotation
            : Quaternion.identity;

        sensor.AddObservation(localTargetOffset / 3f);
        sensor.AddObservation(relativeTargetRotation);
        sensor.AddObservation(reference.InverseTransformDirection(rigidBody.linearVelocity));
        sensor.AddObservation(reference.InverseTransformDirection(rigidBody.angularVelocity));
        sensor.AddObservation(Mathf.Clamp01(localTargetOffset.magnitude / 3f));
    }

    // OnActionReceived方法：处理AI决策的动作输入
    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        ResolveRuntimeReferences();

        /*
        1.考虑对输出再进行离散化，区间取值
        2.改用八个推进器单独控制
        */

        int actionCount = Mathf.Min(currentActions.Length, actionBuffers.ContinuousActions.Length);
        for (int i = 0; i < currentActions.Length; i++)
        {
            currentActions[i] = i < actionCount
                ? Mathf.Clamp(actionBuffers.ContinuousActions[i], -1f, 1f)
                : 0f;
        }
        FinsROVAgentRuntime.ApplyThrusterActions(orderedThrusters, currentActions, thrusterCommandMode, actionForceScaleN);

        // Debug.Log("Actions received: " + string.Join(", ", actionBuffers.ContinuousActions));

        // 计算与目标的距离
        float distanceToTarget = Vector3.Distance(
            selfTransform.position,
            targetTransform.position
        );

        // 计算朝向对齐度（完整的三维朝向对齐，自身朝向与目标朝向的一致程度）
        Vector3 selfForward = selfTransform.forward;
        Vector3 targetForward = targetTransform.forward;
        float headingAlignment = Vector3.Dot(selfForward, targetForward); // 范围：[-1, 1]

        // 计算运动方向对齐度（投影到水平面，自身水平朝向与到达目标方向的一致程度）
        Vector3 selfForwardFlat = new Vector3(selfTransform.forward.x, 0, selfTransform.forward.z).normalized;
        Vector3 directionToTarget = (targetTransform.position - selfTransform.position).normalized;
        float positionAlignment = Vector3.Dot(selfForwardFlat, directionToTarget); // 范围：[-1, 1]

        float reward = 0.0f;

        // ===== 时间惩罚 =====
        // 基础步长惩罚：每步给予微小负奖励，鼓励快速完成
        reward += -0.001f;

        // ===== 距离奖励 =====
        // 距离奖励：离目标越近奖励越高，采用指数衰减曲线使其更平滑
        float distanceReward = Mathf.Exp(-distanceToTarget / 1f) - 0.368f; // e^(-1) ≈ 0.368，作为基准
        reward += 1.5f * distanceReward - 0.4f; //不妨自行绘制这条曲线，看看效果
                                                // reward += 0.85f - distanceToTarget / 2.0f; // 根据距离给予奖励，距离越近奖励越高

        // ===== 朝向奖励 =====
        // 目标朝向对齐奖励：让潜器头朝向与目标朝向一致
        float targetHeadingReward = (headingAlignment + 1f) * 0.5f; // 归一化到 [0, 1]
        reward += 0.08f * targetHeadingReward;

        // 运动方向对齐奖励：让潜器水平朝向与到达目标的方向一致
        float motionAlignmentReward = (positionAlignment + 1f) * 0.5f; // 归一化到 [0, 1]
        reward += 0.06f * motionAlignmentReward;

        // ===== 平稳性惩罚 =====
        // 角速度惩罚：转向过快会被惩罚，鼓励平稳运动
        reward += -0.05f * rigidBody.angularVelocity.magnitude;

        // 线速度惩罚：过快移动会被轻微惩罚，但不如角速度惩罚严厉
        reward += -0.002f * rigidBody.linearVelocity.magnitude;

        SetReward(reward);
        // Debug.Log($"Reward: {reward:F3}");

        // 检查是否到达目标（同时检查距离和朝向）
        if (distanceToTarget < 0.2f && headingAlignment > 0.7f) // 需要头朝向与目标朝向较接近
        {
            reward += 2f;
            SetReward(reward);
            // Debug.Log("Target reached! Ending episode.");
            EndEpisode();      // 结束回合，准备下一个回合
        }
    }

    // 在训练期间或调试时，通过键盘输入手动控制代理的行为
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActionsOut = actionsOut.ContinuousActions;
        var heuristicScratch = new float[8];
        FinsROVAgentRuntime.WriteHeuristicActionsFromManualInput(continuousActionsOut, heuristicScratch);

        // Debug.Log("Heuristic continuous actions: " + string.Join(", ", continuousActionsOut ));


        // continuousActionsOut[0] = Input.GetKey(KeyCode.Keypad1) ? 1f : 0f;
        // continuousActionsOut[1] = Input.GetKey(KeyCode.Keypad2) ? 1f : 0f;
        // continuousActionsOut[2] = Input.GetKey(KeyCode.Keypad3) ? 1f : 0f;
        // continuousActionsOut[0] = Input.GetAxis("Horizontal");
        // continuousActionsOut[1] = Input.GetAxis("Vertical");
    }

    void ResolveRuntimeReferences()
    {
        if (rigidBody == null)
        {
            rigidBody = GetComponent<Rigidbody>();
        }

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
                Debug.Log($"[{nameof(ControlForPosition)}] {statusMessage}", this);
                logThrusterResolution = false;
            }
        }
        else
        {
            Debug.LogError($"[{nameof(ControlForPosition)}] {statusMessage}", this);
        }
    }

    // 生成球面内的随机点，限制在给定半径内且不超过水面
    private Vector3 GenerateRandomTargetPosition(float radius, float centerX = 0f, float centerY = 0f, float centerZ = 0f)
    {
        // 生成球面内的随机点
        Vector3 randomDirection = Random.insideUnitSphere;
        Vector3 randomPoint = randomDirection.normalized * Random.value * radius;

        // 设置目标位置：中心 + 随机偏移，且y ≤ 0
        Vector3 targetPos = new Vector3(
            centerX + randomPoint.x,
            centerY + Mathf.Min(randomPoint.y, 0f), // 确保不超过水面（y=0）
            centerZ + randomPoint.z
        );

        return targetPos;
    }

    // 生成仅有头转向（yaw旋转）的随机四元数
    private Quaternion GenerateRandomHeading()
    {
        // 随机生成 0-360 度的yaw角
        float randomYaw = Random.Range(0f, 360f);
        // 仅绕y轴旋转，表示头的转向
        return Quaternion.Euler(0f, randomYaw, 0f);
    }

}
