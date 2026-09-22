// 引入必要的程序包（类似工具箱）
using System.Collections;        // 基础系统工具（虽然本例未直接使用，但保留无害）
using System.Collections.Generic; // 列表等数据结构工具（本例未直接使用）
using UnityEngine;               // Unity引擎核心功能
using Unity.MLAgents;           // ML-Agents机器学习框架核心
using Unity.MLAgents.Sensors;   // 用于Agent的观察功能
using Unity.MLAgents.Actuators; // 新增的命名空间
using NWH.DWP2.ShipController; // 引入动态水物理的船舶控制器命名空间

// 定义Pursuer类，继承自Agent基类（冒号表示继承）
public class Pursuer : Agent // 类名必须与文件名一致
{
    // 【字段声明】
    // Rigidbody类型变量：用于物理模拟的刚体组件
    // private表示仅本类可以访问（未写修饰符时默认为private）
    Rigidbody rigidBody;

    AdvancedShipController advancedShipController;

    List<Engine> engines;

    public ShipInputActions shipInputActions;

    // public表示可以在Unity编辑器中设置，Transform类型存储物体的位置/旋转/缩放信息
    public Transform Target;     // 目标物体的位置信息。使用一个没有collider的立方体作为目标位置的指示

    // float表示浮点数（带小数），public变量会显示在Unity Inspector面板
    // public float forceMultiplier = 10; // 控制移动力度的放大系数


    // 【碰撞检测相关字段】
    private bool isColliding = false; // 当前是否处于碰撞状态
    private float collisionPenalty = -0.5f; // 每次碰撞的惩罚值
    private int collisionCount = 0; // 本回合内的碰撞次数
    private bool hasCollisionThisFrame = false; // 本帧是否发生过碰撞
    // Start方法：Unity的初始化函数，在对象创建后第一帧更新前调用
    void Start()
    {
        // GetComponent<类型>() 获取当前物体上的指定类型组件
        // 这里获取Rigidbody组件并赋值给rigidBody变量
        rigidBody = GetComponent<Rigidbody>();
        advancedShipController = GetComponent<AdvancedShipController>();
        shipInputActions = new ShipInputActions();
        shipInputActions.Enable();
        engines = advancedShipController.engines;
        foreach (var engine in engines)
        {
            engine.useExternalThrottleInput = true;
        }
    }

    // OnEpisodeBegin方法：ML-Agents的重写方法，每个训练回合开始时调用
    public override void OnEpisodeBegin()
    {
        // 判断代理的Y轴位置是否小于0（掉下平台）
        // transform是当前物体的Transform组件，localPosition是局部坐标位置
        // if (this.transform.localPosition.y < 0) // this可省略，表示当前实例
        // {
        //     // 重置物理状态
        //     rigidBody.angularVelocity = Vector3.zero; // 角速度归零
        //     rigidBody.linearVelocity = Vector3.zero;        // 线性速度归零
        //     // 重置位置到平台中心（0,0.5,0），Y=0.5因为球体半径是0.5
        //     this.transform.localPosition = new Vector3(0, 0.5f, 0);
        // }

        // 重置物理状态
        rigidBody.angularVelocity = Vector3.zero; // 角速度归零
        rigidBody.linearVelocity = Vector3.zero;        // 线性速度归零
        // 重置位置到平台中心（0,0.5,0），Y=0.5因为球体半径是0.5
        // this.transform.localPosition = new Vector3(
        //     98 + Random.value * 2,
        //     0,
        //     100 + Random.value * 2
        // );

        // 重置碰撞相关状态
        isColliding = false;
        collisionCount = 0;
        hasCollisionThisFrame = false;
        // 设置目标物体的新位置：
        // Random.value返回0-1的随机数，8-4的操作使范围变为-4到+4
        // 保持Y轴0.5（立方体高度为1，放在平台表面）
        // Target.localPosition = new Vector3(
        //     Random.value * 8 - 4, // X轴：-4到4
        //     0.5f,                 // Y轴固定
        //     Random.value * 8 - 4  // Z轴：-4到4
        // );
    }

    // CollectObservations方法：收集环境观察数据供AI学习
    public override void CollectObservations(VectorSensor sensor)
    {
        // 添加目标的全局位置观察（7个浮点数：x,y,z + 四元数）
        sensor.AddObservation(Target.position);
        sensor.AddObservation(Target.rotation);

        // 添加代理自身位置观察（7个浮点数，全局位置+方向）
        sensor.AddObservation(this.transform.position);
        sensor.AddObservation(this.transform.rotation);

        // 添加速度观察（只需要x和z轴，y轴速度不影响平面移动）
        sensor.AddObservation(rigidBody.linearVelocity); // 三个轴的线速度
        sensor.AddObservation(rigidBody.angularVelocity); // 三个轴的角速度
        // 总观察值数量 = 7 + 7 + 6 = 20
    }

    // OnActionReceived方法：处理AI决策的动作输入
    public override void OnActionReceived(ActionBuffers actionBuffers)
    {
        // 假设第一个动作控制总油门
        // advancedShipController.input.Throttle = Mathf.Clamp(actionBuffers.ContinuousActions[0], -1f, 1f);
        // 假设第二个动作控制转向
        // advancedShipController.input.Throttle2 = Mathf.Clamp(actionBuffers.ContinuousActions[1], -1f, 1f);
        // 计算与目标的距离（使用三维空间距离公式）

        foreach (Engine engine in engines)
        {
            engine.externalThrottleInput = 0f; //重新清零
        }

        /*
        1.考虑对输出再进行离散化，区间取值
        2.改用八个推进器单独控制
        */

        // // 前半部分抬头/低头
        // GetEngineByName("Vertical1").externalThrottleInput = Mathf.Clamp(actionBuffers.ContinuousActions[0], -1f, 1f);
        // GetEngineByName("Vertical4").externalThrottleInput = Mathf.Clamp(actionBuffers.ContinuousActions[0], -1f, 1f);

        // // 后半部分翘起/低下
        // GetEngineByName("Vertical2").externalThrottleInput = Mathf.Clamp(actionBuffers.ContinuousActions[1], -1f, 1f);
        // GetEngineByName("Vertical3").externalThrottleInput = Mathf.Clamp(actionBuffers.ContinuousActions[1], -1f, 1f);

        // GetEngineByName("Horizontal1").externalThrottleInput = Mathf.Clamp(actionBuffers.ContinuousActions[2], -1f, 1f);
        // GetEngineByName("Horizontal3").externalThrottleInput = Mathf.Clamp(actionBuffers.ContinuousActions[2], -1f, 1f);

        // GetEngineByName("Horizontal2").externalThrottleInput = Mathf.Clamp(actionBuffers.ContinuousActions[3], -1f, 1f);
        // GetEngineByName("Horizontal4").externalThrottleInput = Mathf.Clamp(actionBuffers.ContinuousActions[3], -1f, 1f);

        GetEngineByName("Vertical1").externalThrottleInput = Mathf.Clamp(actionBuffers.ContinuousActions[0], -1f, 1f);
        GetEngineByName("Vertical2").externalThrottleInput = Mathf.Clamp(actionBuffers.ContinuousActions[1], -1f, 1f);
        GetEngineByName("Vertical3").externalThrottleInput = Mathf.Clamp(actionBuffers.ContinuousActions[2], -1f, 1f);
        GetEngineByName("Vertical4").externalThrottleInput = Mathf.Clamp(actionBuffers.ContinuousActions[3], -1f, 1f);
        GetEngineByName("Horizontal1").externalThrottleInput = Mathf.Clamp(actionBuffers.ContinuousActions[4], -1f, 1f);
        GetEngineByName("Horizontal2").externalThrottleInput = Mathf.Clamp(actionBuffers.ContinuousActions[5], -1f, 1f);
        GetEngineByName("Horizontal3").externalThrottleInput = Mathf.Clamp(actionBuffers.ContinuousActions[6], -1f, 1f);
        GetEngineByName("Horizontal4").externalThrottleInput = Mathf.Clamp(actionBuffers.ContinuousActions[7], -1f, 1f);

        // Debug.Log("Actions received: " + string.Join(", ", actionBuffers.ContinuousActions));

        float distanceToTarget = Vector3.Distance(
            this.transform.localPosition,
            Target.localPosition
        );
        // Draw a red sphere at current position with given radius
        // Gizmos.DrawWireSphere(this.transform.position, 0.05f);
        float reward = 0.0f;
        //距离奖励设计
        reward += -0.001f;// 每步给予微小的负奖励，鼓励更快到达目标
        reward += 0.85f - distanceToTarget / 2.0f; // 根据距离给予奖励，距离越近奖励越高，0.5意义是

        //平稳性惩罚
        reward += -0.1f * rigidBody.angularVelocity.magnitude; // 角速度越大，惩罚越多
        reward += -0.001f * rigidBody.linearVelocity.magnitude; // 线速度越大，惩罚越多
        //姿态惩罚
        Vector3 flatForward = new Vector3(this.transform.forward.x, 0, this.transform.forward.z).normalized;
        float alignment = Vector3.Dot(flatForward, (Target.localPosition - this.transform.localPosition).normalized);
        reward += -0.01f * (1 - alignment); // 与目标方向越不一致，惩罚越多



        //碰撞惩罚
        if (hasCollisionThisFrame)
        {
            reward += collisionPenalty; // 碰撞时给予较大的负奖励
            collisionCount++; // 记录碰撞次数
            Debug.Log("Collision detected! Total collisions this episode: " + collisionCount);
        }
        hasCollisionThisFrame = false; // 每帧重置碰撞标志



        // 判断是否到达目标（1.42是经验值，因为立方体对角线≈1.414）
        // if (distanceToTarget < 0.5f)
        // {
        //     reward += 0.25f; //其实可以调用AddReward()函数
        // }
        SetReward(reward);
        Debug.Log("Step reward: " + reward);
        if (distanceToTarget < 0.02f)
        {
            reward += 0.5f;
            EndEpisode();      // 结束回合，准备下一个回合
        }
        // // 如果掉下平台（Y坐标小于0）
        // else if (this.transform.localPosition.y < 0)
        // {
        //     EndEpisode(); // 结束回合（不给奖励）
        // }
    }

    // 在训练期间或调试时，通过键盘输入手动控制代理的行为
    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActionsOut = actionsOut.ContinuousActions;
        var inputTestActions = new float[8] { 0f, 0f, 0f, 0f, 0f, 0f, 0f, 0f };
        //下面的键盘输入仍然使用老的shipoInputActions的键位映射获取，仅仅为了方便调试。实际上continuousActions的输出并非如此控制
        inputTestActions[0] = shipInputActions.ShipControls.Throttle.ReadValue<float>(); // SW
        inputTestActions[1] = shipInputActions.ShipControls.Throttle2.ReadValue<float>(); // 56
        inputTestActions[2] = shipInputActions.ShipControls.Throttle3.ReadValue<float>(); // 78
        inputTestActions[3] = shipInputActions.ShipControls.Throttle4.ReadValue<float>(); // 90
        inputTestActions[4] = shipInputActions.ShipControls.Steering.ReadValue<float>(); // AD

        // Debug.Log("Heuristic input actions: " + string.Join(", ", inputTestActions));

        // 下潜/上浮
        continuousActionsOut[0] += Mathf.Clamp(inputTestActions[1], -1f, 1f);
        continuousActionsOut[1] += Mathf.Clamp(inputTestActions[1], -1f, 1f);
        continuousActionsOut[2] += Mathf.Clamp(inputTestActions[1], -1f, 1f);
        continuousActionsOut[3] += Mathf.Clamp(inputTestActions[1], -1f, 1f);

        // 前进/后退
        continuousActionsOut[4] += Mathf.Clamp(inputTestActions[0], -1f, 1f);
        continuousActionsOut[7] += -Mathf.Clamp(inputTestActions[0], -1f, 1f);
        continuousActionsOut[5] += Mathf.Clamp(inputTestActions[0], -1f, 1f);
        continuousActionsOut[6] += -Mathf.Clamp(inputTestActions[0], -1f, 1f);

        //转向
        continuousActionsOut[4] += Mathf.Clamp(inputTestActions[4], -0.1f, 0.1f);
        continuousActionsOut[5] += Mathf.Clamp(inputTestActions[4], -0.1f, 0.1f);
        continuousActionsOut[6] += Mathf.Clamp(inputTestActions[4], -0.1f, 0.1f);
        continuousActionsOut[7] += Mathf.Clamp(inputTestActions[4], -0.1f, 0.1f);

        // Debug.Log("Heuristic continuous actions: " + string.Join(", ", continuousActionsOut ));


        // continuousActionsOut[0] = Input.GetKey(KeyCode.Keypad1) ? 1f : 0f;
        // continuousActionsOut[1] = Input.GetKey(KeyCode.Keypad2) ? 1f : 0f;
        // continuousActionsOut[2] = Input.GetKey(KeyCode.Keypad3) ? 1f : 0f;
        // continuousActionsOut[0] = Input.GetAxis("Horizontal");
        // continuousActionsOut[1] = Input.GetAxis("Vertical");
        // 假设第一个动作控制总油门
        // advancedShipController.input.Throttle = Mathf.Clamp(continuousActionsOut[0], -1f, 1f);

        // // 假设第二个动作控制转向1
        // advancedShipController.input.Throttle2 = Mathf.Clamp(continuousActionsOut[1], -1f, 1f);
        // advancedShipController.input.Throttle3 = Mathf.Clamp(continuousActionsOut[2], -1f, 1f);

        // foreach (Engine engine in engines)
        // {
        //     engine.input.Throttle = 0f;
        // }
        // for (int i = 0; i < engines.Count; i++)
        // {
        //     // 将 ML-Agents 输出的 [-1, 1] 映射到引擎的输入
        //     // 注意：DWP2 Engine.input.Throttle 通常接收 [0, 1]，
        //     // 如果需要反向推力，需确认引擎设置支持负转速或单独处理
        //     float force = actions.ContinuousActions[i];
        //     engines[i].input.Throttle = Mathf.Clamp(force, -1f, 1f);
        // }
    }

    // 根据引擎名字获取引擎对象
    private Engine GetEngineByName(string engineName)
    {
        return engines.Find(engine => engine.name == engineName);
    }

    /// <summary>
    /// 碰撞进入回调函数：当物体与其他物体碰撞时调用
    /// </summary>
    private void OnCollisionEnter(Collision collision)
    {
        // 标记本帧发生了碰撞
        hasCollisionThisFrame = true;
        isColliding = true;

        // 获取碰撞对象的标签，可用于区分不同类型的碰撞
        string otherTag = collision.gameObject.tag;
        Debug.Log("Collision with: " + collision.gameObject.name + " (Tag: " + otherTag + ")");
    }

    /// <summary>
    /// 碰撞保持回调函数：当物体持续碰撞时每帧调用
    /// </summary>
    private void OnCollisionStay(Collision collision)
    {
        // 在碰撞持续过程中，每帧都标记为碰撞状态
        hasCollisionThisFrame = true;
    }

    /// <summary>
    /// 碰撞退出回调函数：当物体停止碰撞时调用
    /// </summary>
    private void OnCollisionExit(Collision collision)
    {
        // 碰撞结束，更新状态
        isColliding = false;
        Debug.Log("Collision ended with: " + collision.gameObject.name);
    }
}