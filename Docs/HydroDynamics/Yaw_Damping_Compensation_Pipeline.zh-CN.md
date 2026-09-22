# Yaw Damping Compensation Pipeline

本文描述方案 B：保留当前 DWP2 水动力，不做完整 6DOF Fossen 建模，只额外加入 yaw-only compensation，用小范围反阻尼抵消仿真里过大的自转阻力。

## 目标

当前判断是：

```text
实机更容易原地转起来
Unity/DWP2 中 yaw 自转阻力偏大
不希望重新建模 6DOF 水动力
不希望动 DWP2 源码
不希望破坏 DWP2 浮力/配平
```

因此新增脚本：

```text
YawDampingCompensationTarget
```

只作用于 Fossen yaw rate `r`，不计算 surge/sway/heave/roll/pitch，不施加浮力，不改 DWP2。

## 模型

脚本每个 `FixedUpdate` 计算机体系 yaw rate：

```text
r = body yaw rate
```

然后施加补偿力矩：

```text
tau_r = +Cr * r + Crr * |r| * r
```

注意这里是正号。普通阻尼是：

```text
tau_damping = -Dr * r - Drr * |r| * r
```

本方案要抵消一部分 DWP2 的过强 yaw 阻尼，所以补偿力矩与 yaw rate 同号。它会注入能量，必须使用小系数和 torque limit。

## 挂载方式

在 FinsROV root，也就是带 `Rigidbody` 的对象上挂：

```text
YawDampingCompensationTarget
```

推荐字段：

```text
targetRigidbody = FinsROV root Rigidbody
bodyFrame = FinsROV root 或 MarkPosition
bodyAxisConvention = FinsRovXForwardYUpZLeft
waterProviderBehaviour = PhysicalWaveWaterDataProvider
compensateRelativeToWaterAngularFlow = true
```

如果不确定 `bodyFrame`，先用 FinsROV root。当前 FinsROV 坐标定义下 yaw rate 对应 local `y`，脚本内部会通过 `HydroMath` 映射到 Fossen `r`。

## 起步参数

先从 0 开始：

```text
linearCompensationNmPerRadSec = 0
quadraticCompensationNmPerRadSec2 = 0
maxCompensationTorqueNm = 0.5
yawRateDeadbandRadPerSec = 0.02
```

然后做 yaw impulse / yaw 单轴测试，逐步加：

```text
linearCompensationNmPerRadSec: 0.02 -> 0.05 -> 0.10 -> 0.20
quadraticCompensationNmPerRadSec2: 0.00 -> 0.02 -> 0.05 -> 0.10
maxCompensationTorqueNm: 0.3 -> 0.5 -> 1.0
```

如果不知道单位量级，优先加 `linearCompensationNmPerRadSec`，不要一开始加很大的二次项。

## Domain Randomization

脚本实现了 `IEpisodeRandomizable`，挂在 `DomainRandomizationCoordinator` 子树下即可被自动发现。

推荐训练范围：

```text
randomizeInTrainMode = true
randomizeInEvaluateMode = false

linearCompensationScale = 0.0 ~ 1.0
quadraticCompensationScale = 0.0 ~ 1.0
torqueLimitScale = 0.8 ~ 1.2
```

含义是：有些 episode 不做补偿，有些 episode 做完整补偿。这样 policy 会同时见过“DWP2 yaw 阻力偏大”和“yaw 更接近实机、更容易转”的情况。

如果已经确定 Unity 比实机阻力大很多，可以扩大到：

```text
linearCompensationScale = 0.3 ~ 1.3
quadraticCompensationScale = 0.2 ~ 1.5
```

但必须先保证不会在空动作下自发越转越快。

## 安全检查

每次改参数后做三项检查：

1. 空动作静置 10 秒，yaw rate 不应自发增长。
2. 给一个短 yaw pulse 后停止，yaw rate 应该衰减，只是比原 DWP2 衰减慢。
3. 训练中统计 `lastBodyYawTorqueNm`，不要长期顶到 `maxCompensationTorqueNm`。

如果出现：

```text
空动作也开始自转
yaw rate 越转越大
lastBodyYawTorqueNm 长期等于 torque limit
```

说明补偿过强。先降低：

```text
linearCompensationNmPerRadSec
quadraticCompensationNmPerRadSec2
maxCompensationTorqueNm
```

## 与 Reward 的关系

这个脚本不是为了让 policy 更爱转，而是让仿真里 yaw 动态覆盖实机的低阻尼情况。训练时仍然应该保留：

```text
yaw rate penalty
action smoothness penalty
yaw action / yaw proxy penalty
```

正确组合是：

```text
环境：yaw 更容易转
奖励：不鼓励无意义自转
策略：学会在低 yaw 阻尼下仍然稳定到点
```

## 不做的事情

本方案不做：

```text
完整 6DOF Fossen
added mass 标定
浮心/重心配平
DWP2 源码修改
每个自由度单独建模
```

它只是一个 yaw-only residual correction，用来弥合 DWP2 yaw 自转阻力偏大的 sim2real 差异。
