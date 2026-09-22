# DWP2 浮力 + EmpiricalDrag6Dof 域随机化 Pipeline

本文面向当前 FinsROV 训练需求：不追求仿真和实机完全一致，但训练时需要覆盖足够宽的动力学扰动；同时尽量沿用 DWP2 的 `WaterObject` 挂载方式，不重新对潜器做浮力配平。

## 结论

推荐训练 backend：

```text
DWP2 WaterObject 负责浮力/体积/现有配平
EmpiricalDrag6Dof 负责 6DOF 阻尼
PhysicalWaveWaterDataProvider 负责水流和波浪
DomainRandomizationCoordinator 负责 episode 级随机化
Dwp2MassBuoyancyRandomizer 只在启用质量随机化时负责质量变化和 DWP2 浮力联动
```

暂时不推荐把 `Fossen6Dof` 作为训练主 backend。它对 added mass、静力学拆分、坐标和 profile 参数更敏感；在当前“潜器容易下沉、姿态难稳定”的阶段，`EmpiricalDrag6Dof` 更适合先把策略训练稳。

也不推荐只随机化 DWP2 的 `hydrodynamicForceCoefficient`。这个系数是 DWP2 `WaterObject` 的全局动态水动力增益，不区分 surge/sway/heave/yaw。只随机它容易让平移和旋转耦在一起，策略可能学会“能到点但一直摆头或原地打转”的行为。

## 力的归属

推荐配置下，每个模块负责的物理项如下：

| 模块                                 | 是否启用       | 负责内容                                     |
| ------------------------------------ | -------------- | -------------------------------------------- |
| DWP2`WaterObject`                  | 启用           | 浮力、体积、原有 hull 配平                   |
| DWP2`hydrodynamicForceCoefficient` | 默认保留原值   | 先保证 DWP2 浮力链路完整；确认过阻尼后再降低 |
| `HydrodynamicsController`          | 启用           | 采样水体，调用 backend，施加参数化阻尼       |
| `EmpiricalDrag6Dof`                | 启用           | 6DOF 线性/二次阻尼                           |
| `PhysicalWaveWaterDataProvider`    | 启用           | 水面高度、水流、解析波浪和 orbital flow      |
| `Dwp2MassBuoyancyRandomizer`       | 推荐启用       | 质量随机化，同时按质量比例修正 DWP2 浮力系数 |
| `RigidbodyRandomizationTarget`     | 不推荐同时启用 | 避免质量被随机两次                           |

`HydrodynamicsProfile.displacedVolume` 推荐设为 `0`，这样浮力不由 `HydrodynamicsController` 计算，继续交给 DWP2。

注意：当前项目里 DWP2 `WaterObject` 的 `hydrodynamicForceCoefficient` 不再由 `HydrodynamicsController` 默认置 0。因为在实际 scene 里，直接把这个值清零可能导致 DWP2 的水动力计算路径不完整，表现就是 `submergedVolume` 有值但静止仍然下沉。稳定优先级更高，所以先保留 DWP2 原始系数；如果后面速度响应明显过慢，再逐步把 DWP2 `hydrodynamicForceCoefficient` 从 `1` 降到 `0.5 / 0.2 / 0.1` 做 A/B。

## Scene 挂载步骤

### FinsROV root

在 FinsROV root，也就是带 `Rigidbody` 的对象上挂：

```text
HydrodynamicsController
FinsROVHydrodynamicsSetup
DomainRandomizationCoordinator
ThrusterRandomizationTarget
Dwp2MassBuoyancyRandomizer
InitialStateRandomizationTarget
```

`InitialStateRandomizationTarget` 可选，但训练时建议启用小范围扰动。

不要同时挂：

```text
RigidbodyRandomizationTarget
Dwp2HydrodynamicForceCoefficientRandomizer
```

如果已经挂了 `Dwp2MassBuoyancyRandomizer`，`FinsROVHydrodynamicsSetup` 现在会默认跳过自动添加 `RigidbodyRandomizationTarget`，避免质量/惯量随机化和 DWP2 浮力联动逻辑打架。

### HydrodynamicsController

推荐字段：

```text
mode = EmpiricalDrag6Dof
bodyAxisConvention = FinsRovXForwardYUpZLeft
profile = FinsROV_HydrodynamicsProfile
waterProviderBehaviour = PhysicalWaveWaterDataProvider
disableDwp2WaterObjectsForParametricBackends = true
zeroDwp2HydrodynamicForceCoefficientForParametricBackends = false
```

`zeroDwp2HydrodynamicForceCoefficientForParametricBackends=false` 是当前推荐值。它表示不让 `HydrodynamicsController` 自动清零 DWP2 的 `WaterObject.hydrodynamicForceCoefficient`，先完整保留 DWP2 的浮力/水动力运行路径。

只有在你已经确认 DWP2 浮力不受影响，并且明确想避免 DWP2 动态阻力与 `EmpiricalDrag6Dof` 重复时，才把 `zeroDwp2HydrodynamicForceCoefficientForParametricBackends=true`。

### HydrodynamicsProfile

推荐配置：

```text
displacedVolume = 0
assumeFullySubmerged = true
linearDamping = 按 [u, v, w, p, q, r] 填
quadraticDamping = 按 [u, v, w, p, q, r] 填
```

`r` 是 yaw 阻尼。要单独减小或增大旋转阻力，优先改：

```text
linearDamping.r
quadraticDamping.r
```

一般判断：

```text
小角速度附近摇摆收不住 -> 增大 linearDamping.r
大角速度原地打转/甩头 -> 增大 quadraticDamping.r
为了覆盖真实世界更容易打转的情况 -> 随机化时允许 r 取较低值
```

### Water provider

在场景水体对象上挂 `PhysicalWaveWaterDataProvider`。

训练推荐：

```text
mode = AnalyticPhysicalWave
randomizeWaterCurrentFromProfile = true
randomizeWavesFromProfile = true
forceAnalyticModeWhenRandomizingWaves = true
```

它会随机化：

```text
steadyCurrent
wave count
wave amplitude
wave wavelength
wave period
wave direction
wave phase
wave flowScale
wave maxFlowSpeed
```

## DomainRandomizationProfile 推荐范围

先从保守范围开始，确认训练不会出现大量坠落、原地打转或控制发散，再逐步放宽。

### Body

推荐：

```text
randomizeBody = false
```

原因是 DWP2 保浮力方案里，质量和浮力要一起动。直接用 `HydrodynamicsController` 随机化 `displacedVolume / centerOfBuoyancy / centerOfMass` 容易破坏配平。

质量随机化交给 `Dwp2MassBuoyancyRandomizer`：

```text
minMassScale = 0.9
maxMassScale = 1.1
positiveBuoyancyMargin = 0.01 ~ 0.02
scaleInertiaTensorWithMass = true
```

### Hydrodynamics

推荐启用：

```text
randomizeHydrodynamics = true
randomizeHydrodynamicAxisScales = true
```

全局范围：

```text
linearDampingScale = 0.8 ~ 1.2
quadraticDampingScale = 0.8 ~ 1.3
addedMassScale = 1.0 ~ 1.0
```

按轴范围 `[u, v, w, p, q, r]`：

```text
linearDampingAxisScaleMin = [1, 1, 1, 1, 1, 0.7]
linearDampingAxisScaleMax = [1, 1, 1, 1, 1, 1.6]
quadraticDampingAxisScaleMin = [1, 1, 1, 1, 1, 0.5]
quadraticDampingAxisScaleMax = [1, 1, 1, 1, 1, 2.0]
```

这样只对 `yaw=r` 做更宽的随机化，其余自由度保持当前 profile 的相对关系。

如果实机 yaw 摆动非常明显，可以把 `quadraticDampingAxisScaleMin.r` 进一步降到 `0.3`，让策略见过“转动阻尼很低”的情况；但同时训练奖励里必须惩罚 yaw rate，否则策略可能继续利用原地旋转。

### Thrusters

推进器不对称是 yaw 打转的重要来源，建议启用：

```text
randomizeThrusters = true
maxForceScale = 0.9 ~ 1.1
forceConstantScale = 0.9 ~ 1.1
timeConstantScale = 0.8 ~ 1.3
delayScale = 0.8 ~ 1.2
slewRateScale = 0.8 ~ 1.2
```

如果实机存在明显单侧推力偏差，后续应把 `ThrusterRandomizationTarget` 扩展为“每个推进器独立采样”，而不是所有推进器共享一个 scale。

### Water Current

起步范围：

```text
randomizeWater = true
currentSpeed = 0 ~ 0.15 m/s
allowVerticalCurrent = false
meanCurrent = (0, 0, 0)
maxAngleFromMeanDegrees = 180
```

训练稳定后：

```text
currentSpeed = 0 ~ 0.25 m/s
```

### Waves

起步范围：

```text
randomizeWaves = true
waveComponentCount = 1 ~ 2
waveAmplitude = 0 ~ 0.03 m
waveWavelength = 3 ~ 8 m
wavePeriod = 2.5 ~ 5 s
waveDirectionDeg = 0 ~ 360
randomizeWavePhase = true
waveFlowScale = 0.6 ~ 1.0
waveMaxFlowSpeed = 0.05 ~ 0.2 m/s
```

小水池或深水下训练时，波浪范围应更小；如果训练目标主要是水下定点，不建议一开始用过强 orbital flow。

### Initial State

推荐起步范围：

```text
randomizeInitialState = true
initialPositionOffset = x/z: -0.5 ~ 0.5, y: -0.1 ~ 0.1
initialEulerOffsetDeg = roll/pitch: -3 ~ 3 deg, yaw: -180 ~ 180 deg
initialLinearVelocity = -0.05 ~ 0.05 m/s
initialAngularVelocity = -0.03 ~ 0.03 rad/s
```

## Yaw 摆动的处理顺序

实机 yaw 摆动很大，甚至原地打转，但仍能到目标点时，优先按下面顺序处理：

1. 奖励函数加入 yaw rate 惩罚，例如惩罚 `r^2`。
2. 如果任务需要朝向，加入 yaw error 惩罚；如果只需要到点，也至少限制高速旋转。
3. 在仿真里降低一部分 episode 的 `quadraticDamping.r`，让策略见过低 yaw 阻尼。
4. 随机化推进器延迟、时间常数和推力系数，覆盖左右/斜向推力不一致。
5. 真实部署前，用相同控制命令做单轴 yaw benchmark，对比角速度峰值、稳态 yaw rate 和停止后的衰减时间。

注意：如果训练中已经频繁原地打转，单纯继续降低 yaw 阻尼会让策略更放飞。此时应该先加 yaw rate 惩罚，再放宽 yaw 阻尼随机化范围。

## 角速度稳定奖励

如果希望最大程度沿用 DWP2 的 `WaterObject`，不再拆分六自由度阻尼，可以把 Agent 脚本从原来的位置控制 reward 换成：

```text
ControlForPosition_AngularStabilityReward
```

这个脚本不修改原有 `ControlForPosition`、`ControlForPosition_IncrementalReward` 或 `ControlForPosition_SmoothNearTargetReward`。它的差别是：

```text
全程惩罚 max(0, angularSpeed - 25deg/s)^2
全程单独惩罚 max(0, yawRate - 25deg/s)^2
全程惩罚超出 deadband 的高速 yaw 自转
接近目标时进一步放大超阈值 angularSpeed/yawRate/roll-pitch rate 和 roll-pitch angle 惩罚
接近目标时继续惩罚线速度，鼓励真正停住
```

推荐初始参数：

```text
successDistance = 0.2
successHeadingAngleDeg = 25
stableSuccessLinearVelocity = 0.1
stableSuccessAngularVelocity = 0.3
stableSuccessStepsRequired = 10
angularVelocityPenaltyScale = 0.025
yawRatePenaltyScale = 0.04
rollPitchAnglePenaltyScale = 0.02
angularPenaltyDeadbandDegPerSec = 25
nearTargetDistance = 0.6
nearTargetAngularVelocityPenaltyScale = 0.18
nearTargetYawRatePenaltyScale = 0.35
nearTargetRollPitchRatePenaltyScale = 0.12
nearTargetRollPitchAnglePenaltyScale = 0.08
spinDeadbandRadPerSec = 0.4363
spinPenaltyScale = 0.08
nearTargetSpinPenaltyScale = 0.3
speedNearTargetPenaltyScale = 0.06
actionEnergyPenaltyScale = 0.002
actionChangePenaltyScale = 0.0025
```

挂载方式：

1. 保留 DWP2 `WaterObject`。
2. 不启用 `HydrodynamicsController` 的 Fossen/Empirical backend，或者将其设为 `Off`。
3. 在 FinsROV root 上禁用旧的 position reward Agent。
4. 添加并启用 `ControlForPosition_AngularStabilityReward`。
5. 复用原来的 `selfTransform`、`targetTransform`、`Behavior Parameters`、`Decision Requester` 和 thruster 配置。

如果训练后潜器变得过于保守、靠近目标变慢，先降低：

```text
nearTargetYawRatePenaltyScale
nearTargetSpinPenaltyScale
actionChangePenaltyScale
```

如果仍然会到点附近大幅摆头，优先提高：

```text
nearTargetYawRatePenaltyScale
nearTargetSpinPenaltyScale
yawRatePenaltyScale
```

为了防止潜器在到达目标点之后或过程中原地打转，建议把约束分成三层：

```text
Reward:
  保留 yawRateExcess^2 和 spinExcess^2 惩罚
  保留 nearTargetYawRatePenaltyScale 和 nearTargetSpinPenaltyScale
  保留 actionChangePenaltyScale，减少推进器反复打满导致的摆头

Success gate:
  success 必须满足 stableSuccessLinearVelocity < 0.1 m/s
  success 必须满足 stableSuccessAngularVelocity < 0.3 rad/s
  success 必须连续稳定 stableSuccessStepsRequired 步

Failure gate:
  如果 abs(yawRate) 长时间超过 0.8~1.2 rad/s，可以直接给 penalty 并 EndEpisode
  如果 nearTargetRatio 很高但 yawRate 长时间超过 0.5 rad/s，也可以直接判失败
```

当前脚本已经实现前两层。第三层“持续高速自转直接失败”还没有默认打开，因为它会改变任务难度，建议等 reward 版训练仍然出现原地打转时再加。

## 新增随机化组件

### 水平推进器幅度随机化

新增组件：

```text
HorizontalThrusterAmplitudeRandomizer
```

用途：只随机化水平推进器的推力幅度，用来模拟实机中左右/前后水平推进器推力不一致、安装角误差和电机效率差异。它默认按 `abs(Thruster.LocalForceDirection.y) <= 0.35` 自动筛选水平推进器，也可以手动指定 `horizontalThrusters`。

推荐初始参数：

```text
minAmplitudeScale = 0.85
maxAmplitudeScale = 1.15
useCommonScale = false
scaleForceConstants = false
randomizePerEpisode = true
```

说明：

```text
useCommonScale = false
```

表示每个水平推进器独立采样，更适合覆盖 yaw 偏转和侧向漂移。如果只是想模拟整组水平推进器整体变强/变弱，才设为 `true`。

注意：这个组件会修改 `Thruster.MaxForwardForceN / MaxReverseForceN`。它和 `ThrusterRandomizationTarget.maxForceScale` 作用在同一类参数上。训练时建议二选一：

```text
方案 A:
  保留 ThrusterRandomizationTarget，但把 DomainRandomizationProfile.maxForceScale 设为 1~1
  使用 HorizontalThrusterAmplitudeRandomizer 专门随机水平推进器幅度

方案 B:
  不挂 HorizontalThrusterAmplitudeRandomizer
  继续用 ThrusterRandomizationTarget 随机所有推进器
```

如果两个都开，结果取决于 episode 随机化调用顺序，不建议作为稳定训练配置。

### 角速度惩罚系数随机化

新增组件：

```text
AngularStabilityRewardRandomizer
```

用途：随机化 `ControlForPosition_AngularStabilityReward` 里的角速度惩罚权重，让策略不要只适应一个固定的转动代价。它不会改原始 reward 脚本，只作用于新的 `ControlForPosition_AngularStabilityReward`。

推荐初始参数：

```text
minGlobalAngularPenaltyScale = 0.8
maxGlobalAngularPenaltyScale = 1.4
minYawPenaltyScale = 0.9
maxYawPenaltyScale = 1.8
minNearTargetPenaltyScale = 1.0
maxNearTargetPenaltyScale = 2.0
randomizePerEpisode = true
```

含义：

```text
globalAngularPenaltyScale:
  缩放整体角速度、roll/pitch 姿态惩罚

yawPenaltyScale:
  额外缩放 yawRate 和 spin 惩罚

nearTargetPenaltyScale:
  额外缩放接近目标点时的角速度/自转惩罚
```

这两个随机化是合理的，但不要一开始范围太大。推荐先让策略收敛到“能稳定到点且不明显自转”，再逐步扩大：

```text
水平推进器幅度: 0.9~1.1 -> 0.85~1.15 -> 0.8~1.2
near-target yaw 惩罚: 1.0~1.5 -> 1.0~2.0
```

## 代码改动说明

本 pipeline 不修改 DWP2 包。

新增/修改的能力：

```text
DomainRandomizationProfile:
  增加 randomizeHydrodynamicAxisScales
  增加 addedMass/linearDamping/quadraticDamping 的 [u,v,w,p,q,r] 按轴 scale 范围

RandomizationContext:
  增加 SixDofVector range 采样

HydrodynamicsProfile:
  增加 ScaleHydrodynamicAxisCoefficients

HydrodynamicsController:
  在 randomizeHydrodynamics 后应用按轴 scale

FinsROVHydrodynamicsSetup:
  默认不再自动添加 RigidbodyRandomizationTarget
  检测到 Dwp2MassBuoyancyRandomizer 时，也会跳过 RigidbodyRandomizationTarget
```

## 最小可运行检查

训练前建议先做三步检查：

1. Play 后静置 10 秒，确认潜器不明显下沉。
2. 只给 yaw 小命令，确认角速度能被阻尼收住。
3. 开启 DR 后打印每个 episode 的 `linearDamping.r / quadraticDamping.r / currentSpeed / thruster scale`，确认随机化真的生效。

如果静置仍下沉，优先查 DWP2 `WaterObject` 的 `buoyantForceCoefficient`、`submergedVolume` 和 `Rigidbody.mass`，不要先调 Fossen/Empirical 阻尼。

当前 FinsROV 如果出现：

```text
Rigidbody.mass ~= 25 kg
totalSubmergedVolume ~= 20
buoyantForceCoefficient ~= 1
```

说明 DWP2 计算出的等效排水量小于刚体质量。此时潜器下沉是正常结果，不是阻尼 backend 的问题。

处理顺序：

1. 如果 25 kg 是凸包/导入误差导致的虚高质量，把 `Rigidbody.mass` 改成真实潜器质量。
2. 如果 25 kg 就是你希望仿真的真实惯性质量，则把参与浮力的 DWP2 `WaterObject.buoyantForceCoefficient` 提高到约 `mass / submergedVolume`，也就是 `25 / 20 = 1.25`。
3. 如果有多个 DWP2 `WaterObject` 一起提供浮力，应统一乘同一个比例，而不是只改 mainbody。
4. Play 时查看 `FinsROVDwp2ForceReadout.estimatedNeutralUniformBuoyantForceCoefficient`，这个字段会按当前总 DWP2 浮力直接估计中性浮力需要的统一系数。

`FinsROVHydrodynamicsSetup.ensureRigidbodyRandomizationTarget` 应保持 `false`。只要浮力由 DWP2 配平，单独随机 `Rigidbody.mass` 都会破坏中性浮力。
