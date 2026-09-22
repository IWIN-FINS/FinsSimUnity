# FinsROV 键盘直接推进器控制说明

本文档说明当前 FinsROV 在 Unity 中直接使用键盘控制推进器的实现方式、按键映射和排查方法。

## 控制入口

当前直接键盘控制不是写在 `Thruster` 里，也不是写在 `ThrusterController` 里，而是通过：

```text
Assets/Scripts/RL/FinsROVManualThrusterController.cs
```

这个组件完成。

推荐挂载位置：

```text
FinsROV 根节点
```

它会自动查找同一节点或子节点上的 `ThrusterController`，解析 8 个推进器，并最终调用：

```csharp
thrusterController.ApplyInput(manualThrusterInput);
```

因此完整链路是：

```text
键盘
  -> FinsROVAgentRuntime.ReadKeyboardManualInput()
  -> FinsROVAgentRuntime.FillThrusterInputFromManualState()
  -> FinsROVManualThrusterController
  -> ThrusterController.ApplyInput(float[])
  -> Thruster.ApplyInput(float)
  -> Thruster.ApplyForceRequest(float)
  -> Rigidbody.AddForceAtPosition(...)
```

## 当前按键

| 按键 | 语义轴 | 数值变化 | 作用 |
| --- | --- | --- | --- |
| `W` | `Surge` | `+1` | 前进 |
| `S` | `Surge` | `-1` | 后退 |
| `D` | `Yaw` | `+1` | 右转，具体方向取决于推进器布置和力方向 |
| `A` | `Yaw` | `-1` | 左转，具体方向取决于推进器布置和力方向 |
| `6` 或小键盘 `6` | `Heave` | `+1` | 垂向正方向 |
| `5` 或小键盘 `5` | `Heave` | `-1` | 垂向负方向 |
| `8` 或小键盘 `8` | `Aux3` | `+1` | 扩展轴，当前用于 pitch trim |
| `7` 或小键盘 `7` | `Aux3` | `-1` | 扩展轴，当前用于 pitch trim |
| `0` 或小键盘 `0` | `Aux4` | `+1` | 扩展轴，当前用于 roll trim |
| `9` 或小键盘 `9` | `Aux4` | `-1` | 扩展轴，当前用于 roll trim |

按键读取代码在：

```text
Assets/Scripts/RL/FinsROVAgentRuntime.cs
```

对应函数：

```csharp
ReadKeyboardManualInput()
```

## 推进器顺序

当前默认 8 个推进器顺序为：

```text
0: Vertical1
1: Vertical2
2: Vertical3
3: Vertical4
4: Horizontal1
5: Horizontal2
6: Horizontal3
7: Horizontal4
```

`FinsROVManualThrusterController` 会调用：

```csharp
FinsROVAgentRuntime.TryResolveOrderedThrusters(...)
FinsROVAgentRuntime.EnsureThrusterControllerOrder(...)
```

来保证 `ThrusterController.thrusters` 尽量按照上述顺序排列。

## 当前混控规则

键盘语义轴会被映射成 8 路推进器归一化输入，范围为 `[-1, 1]`。

默认参数：

```text
yawMixScale = 0.1
auxMixScale = 1.0
```

垂向推进器：

```text
Vertical1 = heave + pitchTrim + rollTrim
Vertical2 = heave + pitchTrim - rollTrim
Vertical3 = heave - pitchTrim - rollTrim
Vertical4 = heave - pitchTrim + rollTrim
```

水平推进器：

```text
Horizontal1 = surge + yaw
Horizontal2 = surge + yaw
Horizontal3 = -surge + yaw
Horizontal4 = -surge + yaw
```

所有输出都会被 clamp 到 `[-1, 1]`。

## Inspector 调试信息

`FinsROVManualThrusterController` 上可以看：

```text
Runtime Debug / currentInputState
Runtime Debug / currentThrusterInput
```

其中：

```text
currentInputState.Surge
currentInputState.Heave
currentInputState.Yaw
currentInputState.Aux3
currentInputState.Aux4
```

用于确认 Unity 是否读到了键盘输入。

```text
currentThrusterInput[0..7]
```

用于确认语义输入是否正确混控成 8 路推进器命令。

`ThrusterController` 上可以看：

```text
Runtime Debug / runtimeThrusterStates
```

每个推进器会显示：

```text
ThrusterName
TargetForceRequestN
AppliedForceRequestN
TimeSinceForceRequestSec
```

`Thruster` 上可以看：

```text
AppliedBodyName
LastAppliedWorldForce
LastAppliedForcePosition
```

用于确认推力最终是否真的施加到了正确的 `Rigidbody` 上。

## 场景配置注意事项

1. FinsROV 根节点推荐设置 tag 为 `Vehicle`。
2. FinsROV 根节点需要有 `Rigidbody`。
3. `FinsROVManualThrusterController`、`ThrusterController`、`Thruster` 应属于同一 FinsROV 层级。
4. 同一时间只启用一个主动控制源，避免手动控制、RL agent、ROS bridge 同时写推进器。
5. 如果 `Thruster.AppliedBodyName` 不是 `FinsROV`，可以在每个 `Thruster.TargetRigidbody` 中显式拖入 FinsROV 根节点的 `Rigidbody`。

## 快速排查

按键后没有运动时，按这个顺序检查：

1. `FinsROVManualThrusterController.currentInputState` 是否变化。
2. `FinsROVManualThrusterController.currentThrusterInput` 是否变化。
3. `ThrusterController.runtimeThrusterStates` 中 `TargetForceRequestN` 是否变化。
4. `ThrusterController.runtimeThrusterStates` 中 `AppliedForceRequestN` 是否变化。
5. `Thruster.AppliedBodyName` 是否为 `FinsROV`。
6. `Thruster.LastAppliedWorldForce` 是否非零。
7. FinsROV 根节点 `Rigidbody` 是否 `Is Kinematic = false`，约束是否没有锁死。

如果第 1 步没有变化，问题在键盘输入或组件没有启用。

如果第 1 步变化但第 2 步没有变化，问题在混控逻辑。

如果第 2 步变化但第 3 步没有变化，问题在 `ThrusterController` 或推进器顺序解析。

如果第 3 步和第 4 步变化但 FinsROV 不动，优先检查 `AppliedBodyName` 和刚体配置。
