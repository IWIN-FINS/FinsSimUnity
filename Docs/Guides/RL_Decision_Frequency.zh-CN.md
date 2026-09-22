## 核心关系

`Decision Requester` 的 `Decision Period` **不是直接以秒或 Hz 设置频率**，而是表示：

> 每隔多少个 `Academy Step` 请求一次新的策略决策。

在 ML-Agents 默认的自动步进模式下，`Academy` 每次 `FixedUpdate()` 执行一次 `EnvironmentStep()`；而 `FixedUpdate` 的仿真时间间隔由 `Time.fixedDeltaTime` 决定。([GitHub][1])

因此：

$$
\Delta t_{\text{decision}}
==========================

N_{\text{DecisionPeriod}}
\cdot
\Delta t_{\text{fixed}}
$$

$$
f_{\text{decision}}
===================

\frac{1}
{N_{\text{DecisionPeriod}}\cdot \text{Time.fixedDeltaTime}}
$$

其中：

* `Time.fixedDeltaTime`：物理仿真步长；
* `Decision Period`：每多少个物理步产生一次新动作；
* (f_{\text{decision}})：神经网络实际产生新动作的频率。

---

## 默认设置下的例子

Unity 默认：

```csharp
Time.fixedDeltaTime = 0.02f;
```

因此物理仿真频率为：

$$
f_{\text{physics}}=\frac{1}{0.02}=50\text{ Hz}
$$

不同 `Decision Period` 对应：

| Decision Period |   决策间隔 | 策略决策频率 |
| --------------: | -----: | -----: |
|               1 | 0.02 s |  50 Hz |
|               2 | 0.04 s |  25 Hz |
|               5 | 0.10 s |  10 Hz |
|              10 | 0.20 s |   5 Hz |
|              20 | 0.40 s | 2.5 Hz |

例如：

```text
Fixed Timestep = 0.02 s
Decision Period = 5
```

那么：

* Unity 物理引擎仍以 **50 Hz** 更新；
* Agent 每 **0.1 s 仿真时间**观察并推理一次；
* 神经网络决策频率是 **10 Hz**。

---

## 必须区分三种“频率”

在 ML-Agents 里，“实际决策频率”和“控制执行频率”可能不是同一个东西。

| 含义                        | 频率                                                    |
| ------------------------- | ----------------------------------------------------- |
| 物理积分频率                    | (1/\text{fixedDeltaTime})                             |
| 神经网络推理频率                  | (1/(\text{DecisionPeriod}\cdot\text{fixedDeltaTime})) |
| `OnActionReceived()` 调用频率 | 取决于 `Take Actions Between Decisions`                  |

### `Take Actions Between Decisions = true`

这是连续控制中最常用的设置。

ML-Agents 在非决策步调用 `RequestAction()`，重复使用最近一次决策产生的动作；`RequestAction()` 不进行新的策略推理，但会使用已有动作再次调用 `OnActionReceived()`。([GitHub][1])

假设 `Decision Period = 5`：

```text
Physics step:       0    1    2    3    4    5    6 ...
New decision:       A0                       A1
OnActionReceived:   A0   A0   A0   A0   A0   A1   A1 ...
```

因此：

* 神经网络推理：10 Hz；
* `OnActionReceived()`：50 Hz；
* 动作向量每隔 0.1 s 更新一次；
* 同一个动作在中间的物理步中重复执行。

这本质上相当于控制系统中的**零阶保持**：

$$
u(t)=u_k,\qquad
t\in[k\Delta t_d,(k+1)\Delta t_d)
$$

所以对于类似螺旋桨推力、关节力矩这样的控制：

```csharp
public override void OnActionReceived(ActionBuffers actions)
{
    rb.AddForce(thrust * actions.ContinuousActions[0]);
}
```

即使策略只以 10 Hz 产生新动作，`AddForce()` 仍然会以 50 Hz 使用同一个动作值施加。

### `Take Actions Between Decisions = false`

此时非决策步不会调用 `RequestAction()`，所以通常只有决策步才调用 `OnActionReceived()`。([GitHub][1])

还是 `Decision Period = 5`：

```text
Physics step:       0    1    2    3    4    5
New decision:       A0                       A1
OnActionReceived:   A0   -    -    -    -    A1
```

这并不一定等价于“保持上一次力”。

具体取决于你的控制代码：

* `rb.AddForce()`、`AddTorque()`：通常只在调用的那个物理步施加，关掉重复动作后可能变成间歇性施力；
* 设置电机目标、舵角目标或自己保存 `currentCommand`：目标值可能自然保持；
* 直接设置 `velocity`：该设置产生的状态效果也可能继续存在。

因此，对水下机器人连续推力控制，一般建议勾选 `Take Actions Between Decisions`，否则可能无意中把持续推力变成每隔 (N) 个物理步施加一次的脉冲。

---

## `Decision Step` 不改变频率

当前实现中的判断条件是：

$$
\text{AcademyStepCount}\bmod
\text{DecisionPeriod}
=====================

\text{DecisionStep}
$$

所以 `Decision Step` 只负责改变决策的**相位偏移**，不改变决策频率。([GitHub][1])

例如两个 Agent 都设置：

```text
Decision Period = 5
```

但：

```text
Agent A: Decision Step = 0
Agent B: Decision Step = 2
```

则：

```text
Academy Step: 0  1  2  3  4  5  6  7 ...
Agent A:      D              D
Agent B:            D              D
```

两者都是每 5 步决策一次，但推理时刻错开，可以避免大量 Agent 在同一个物理步同时推理。

---

## 和渲染帧率没有直接对应关系

`Decision Period` 基于 `Academy Step`，默认 Academy 又基于 `FixedUpdate`，因此它不直接依赖：

* 游戏画面是 30 FPS；
* 60 FPS；
* 144 FPS；
* 是否关闭图形渲染。

一个渲染帧中可能执行零次、一次或多次 `FixedUpdate`。Unity 默认固定步长 0.02 s，即每秒仿真时间有 50 个物理更新。([Unity 文档][2])

因此不要使用下面的计算：

$$
f_{\text{decision}}=
\frac{\text{render FPS}}{\text{DecisionPeriod}}
$$

正常情况下应该使用：

$$
f_{\text{decision}}=
\frac{1}
{\text{fixedDeltaTime}\cdot\text{DecisionPeriod}}
$$

---

## `time_scale` 对频率的影响

需要区分**仿真时间频率**和**现实墙钟时间频率**。

假设：

```text
fixedDeltaTime = 0.02
Decision Period = 5
time_scale = 20
```

在仿真时间尺度下仍然是：

* 物理频率：50 Hz；
* 决策频率：10 Hz；
* 每次决策之间模拟了 0.1 s。

但是理想情况下，每秒墙钟时间会推进约 20 秒仿真时间，所以墙钟时间内可能执行约：

$$
f_{\text{decision,wall}}
\approx
\frac{\text{timeScale}}
{\text{DecisionPeriod}\cdot\text{fixedDeltaTime}}
$$

即：

$$
\frac{20}{5\times0.02}
======================

200\text{ 次决策/现实秒}
$$

但这只是理想值。实际速度还受 CPU、物理计算、神经网络推理和 Python 通信速度限制；ML-Agents 官方也提醒，较高的 `time_scale` 可能影响物理仿真的稳定性。([Unity 文档][3])

所以：

* `Decision Period` 决定控制器在**仿真时间中的采样周期**；
* `time_scale` 主要决定训练在**墙钟时间中跑多快**；
* 不应为了改变控制频率而单纯修改 `time_scale`。

---

## 对水下机器人分层控制的对应

假设物理仿真：

```text
fixedDeltaTime = 0.02 s
```

### RL 直接输出推进器推力

建议：

```text
Decision Period = 1 或 2
Take Actions Between Decisions = true
```

对应：

* 50 Hz 或 25 Hz 策略控制；
* 适合需要快速姿态稳定、力矩控制的任务；
* 但推理负担和训练步数较高。

### RL 输出期望速度，底层 PID 控制推进器

可以设置：

```text
高层 RL Decision Period = 5~10
底层 PID 在 FixedUpdate 中运行
```

对应：

* 高层 RL：10～5 Hz；
* 底层 PID：50 Hz；
* RL 的速度目标在多个物理步中保持；
* PID 每个物理步根据当前误差重新计算推进器输出。

这通常比让低频 RL 动作直接作为瞬时推力更合理。

### RL 输出航点或航向目标

可以设置：

```text
Decision Period = 10~20
```

对应 5～2.5 Hz。底层姿态/速度控制器仍然可以运行在 50 Hz 或更高频率。

---

## 训练上还要注意

改变 `Decision Period` 不只是减少推理次数，它实际上改变了强化学习 MDP 的时间步长：

$$
\Delta t_{\text{RL}}
====================

\text{DecisionPeriod}\cdot\text{fixedDeltaTime}
$$

因此会同时影响：

* 每个 action 持续的仿真时间；
* 两个 observation 之间的状态变化量；
* 两次决策之间累计的 reward；
* 折扣因子 (\gamma) 对真实物理时间的含义；
* `time_horizon` 对应的实际仿真时长。

例如从 `Decision Period=1` 改为 5，单个 RL step 从 0.02 s 变成 0.1 s。若其他参数不变，策略看到的系统会表现得更“快”，并且每个 action 对状态的影响更大。ML-Agents 官方也将一次 observation–decision–action–reward 循环与请求决策关联起来，并指出决策间累计奖励需要合理缩放。([Unity Technologies][4])

最准确的一句话是：

> **Decision Period 决定新动作的更新频率；Fixed Timestep 决定物理仿真频率；Take Actions Between Decisions 决定旧动作是否在中间物理步继续被执行。**

[1]: https://github.com/Unity-Technologies/ml-agents/blob/develop/com.unity.ml-agents/Runtime/DecisionRequester.cs "ml-agents/com.unity.ml-agents/Runtime/DecisionRequester.cs at develop · Unity-Technologies/ml-agents · GitHub"
[2]: https://docs.unity3d.com/ru/current/ScriptReference/MonoBehaviour.FixedUpdate.html?utm_source=chatgpt.com "MonoBehaviour-FixedUpdate() - Unity Scripting API"
[3]: https://docs.unity3d.com/ja/current/ScriptReference/Time-fixedDeltaTime.html?utm_source=chatgpt.com "Unity - Scripting API: Time.fixedDeltaTime"
[4]: https://unity-technologies.github.io/ml-agents/Learning-Environment-Design-Agents/?utm_source=chatgpt.com "Agents - Unity ML-Agents Toolkit"
