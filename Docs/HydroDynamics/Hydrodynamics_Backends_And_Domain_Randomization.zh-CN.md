# Hydrodynamics Backends 与 Domain Randomization

本文档记录本项目新增的水动力学模块、water provider 模块、domain randomization 模块，以及它们在 FinsROV / ML-Agents 场景中的接入方式。

相关代码位置：

- `Packages/com.iwin-fins.fins-sim/Runtime/Platform/Hydrodynamics/Core`
- `Packages/com.iwin-fins.fins-sim/Runtime/Platform/Hydrodynamics/Backends`
- `Packages/com.iwin-fins.fins-sim/Runtime/Platform/Hydrodynamics/Water`
- `Packages/com.iwin-fins.fins-sim/Runtime/Platform/Hydrodynamics/DomainRandomization`
- `Assets/Scripts/FinsROV/Runtime/FinsROVHydrodynamicsSetup.cs`
- `Assets/Scripts/RL/*Water*Provider.cs`
- `Assets/Scripts/RL/FinsROVAgentRuntime.cs`

## 设计目标

新增模块把水下动力学拆成三个可替换层：

1. `HydrodynamicsController`：Unity `Rigidbody` 上的统一入口，负责采样水体、构造 6DOF 状态、调用 backend、施加力和力矩。
2. `IWaterKinematicsProvider`：只负责给定世界坐标的水面高度、法向、水流速度和角速度。
3. `IEpisodeRandomizable`：负责 episode 级参数扰动，由 `DomainRandomizationCoordinator` 统一触发。

这样做的核心思路是避免把“水动力模型”“海况扰动”“训练 domain randomization”“旧 DWP2 水体系统”混在同一个脚本里。backend 只关心相对水速度和 profile 参数；water provider 只关心水体运动；DR 只负责在 episode 开始时改参数。

## 坐标与 6DOF 约定

`SixDofVector` 使用 Fossen / SNAME 顺序：

```text
[u, v, w, p, q, r]
u: surge, 前向
v: sway, 右向
w: heave, 向下
p: roll, 绕 surge
q: pitch, 绕 sway
r: yaw, 绕 down
```

Unity 本地坐标通常是 `x=right, y=up, z=forward`。`HydroMath` 中统一做映射：

- Unity local linear `(x, y, z)` -> Fossen `(u=z, v=x, w=-y)`
- Unity local angular `(x, y, z)` -> Fossen `(p=z, q=x, r=-y)`

所有 profile 系数都按 `[u, v, w, p, q, r]` 填写。

## HydrodynamicsController

主入口脚本是 `HydrodynamicsController`。它要求挂在带 `Rigidbody` 的对象上。

运行流程：

1. 从 `waterProviderBehaviour` 或场景自动查找 `IWaterKinematicsProvider`。
2. 在 `FixedUpdate` 中采样质心位置处的 water kinematics。
3. 构造 `HydrodynamicsContext`，包含刚体速度、水流速度、相对速度、profile、surface mesh。
4. 根据 `mode` 选择 backend 计算 `HydrodynamicsWrench`。
5. 对 force / torque 做安全 clamp。
6. 调用 `Rigidbody.AddForce` 和 `Rigidbody.AddTorque`。
7. 记录 `LastWrench`、`LastRelativeVelocity6Dof`、`LastWaterSample`，供诊断脚本读取。

关键字段：

- `mode`：选择 backend。
- `profile`：水动力参数资产。运行中会 clone 成 runtime profile，DR 不会直接污染 asset。
- `waterProviderBehaviour`：推荐显式指定 provider。
- `autoFindWaterProvider`：未指定 provider 时自动查找场景内第一个 provider。
- `fallbackWaterHeight` / `fallbackCurrentVelocity`：没有 provider 时的兜底水体。
- `surfaceHydroMeshOverride` / `surfaceHydroMeshFilter`：surface backend 使用的简化水动力 mesh。
- `disableDwp2WaterObjectsForParametricBackends`：避免 DWP2 和新 backend 重复施加水动力。
- `keepMainBodyWaterObjectForSurfaceBackend`：已废弃；surface mode 由自定义 mesh backend 独立施力，DWP2 仅可作为 `SimulationMesh` 来源。

DWP2 interop 会缓存 `WaterObject.hydrodynamicForceCoefficient` 原值。切换到 `Off`、禁用 controller，或关闭 interop 时，会恢复原系数，避免临时置零变成永久状态。

## Backend 模型

### HydrostaticOnly

只计算浮力和可选重力补偿：

```text
F_b = rho * g * displacedVolume * submergence * up
tau_b = (centerOfBuoyancy - centerOfMassWorld) x F_b
```

当 `Rigidbody.useGravity == false` 且 `applyGravityWhenRigidbodyGravityDisabled == true` 时，controller 会额外施加重力项，便于在禁用 Unity gravity 的训练场景中仍保持重量/浮力平衡。

适合：

- 快速验证浮力中心和质量中心。
- 检查姿态稳定性。
- 排查推进器和水动力耦合问题。

### EmpiricalDrag6Dof

在 hydrostatic 基础上增加 6DOF 对角阻尼：

```text
tau_d[i] = -( |D_lin[i]| + |D_quad[i]| * |nu_r[i]| + |D_u[i]| * |u_r| ) * nu_r[i]
```

其中 `nu_r` 是刚体相对水体的 6DOF 速度。该模型本质上是“可调、稳定、训练友好”的经验阻尼。

适合：

- 参数不完整时的默认稳定模型。
- RL 训练初期。
- 不需要 added mass / Coriolis 精度的场景。

### Fossen6Dof

在 empirical drag 基础上增加 Fossen 风格 added mass：

```text
tau = tau_d - M_A * nu_dot - C_A(nu_r) * nu_r
```

实现细节：

- `M_A` 来自 `addedMassDiagonal` 或 `addedMassFull`。
- `nu_dot` 由相邻 fixed step 的 `RelativeVelocity` 差分估计。
- `accelerationFilterAlpha` 对差分加速度做一阶滤波，减轻离散噪声。
- `enableAddedMassForce` 控制 `M_A * nu_dot` 项。
- `enableAddedMassCoriolis` 控制 added-mass Coriolis 项。

适合：

- 默认推荐 backend。
- 需要比纯阻尼更接近 DAVE / Gazebo / Fossen 系列模型的 6DOF 运动。
- AUV / ROV 参数较完整时。

### SurfaceGeometryHydro

`SurfaceGeometryHydro` 采用 Stonefish 风格的分层计算：封闭 physics mesh 的体积和体心用于静浮力；三角面仅用于形阻力和切向摩擦阻力。它不再把静浮力建立在单个面法线压力的净和上。

- 完全浸没：从完整 mesh 的有向四面体积分得到体积与浮心。
- 穿越水面：先按水面裁切每个三角形，再对浸没部分做四面体积分。
- `maxSurfaceTriangles` 只限制动态阻力的均匀采样，不再截断静浮力体积积分。
- 默认优先使用同一 `MeshFilter` 上 DWP2 `WaterObject` 的 `SimulationMesh`；它通常已经简化、凸包化并焊接顶点，更适合作为 physics hull。
- `FossenPlusSurfaceResidual` 中，Fossen 负责静浮力和 added mass，mesh backend 只提供残余阻力，因此不会重复浮力。
- 形阻力和切向摩擦分别按 Stonefish 方式累计，并可通过 `formDragAxisScale` / `skinDragAxisScale`（surge, sway, heave）以及对应的 `*TorqueAxisScale`（roll, pitch, yaw）单独校正。仅降低 mesh yaw 阻尼时，优先调低两个 torque scale 的 `z` 分量。

单独使用该 backend 时，应关闭 DWP2 的浮力和动态力，避免两套系统重复施力；并使用封闭、法线一致的低面数 physics mesh。新的 `HydrodynamicsController` 不再为 surface backend 保留 `mainbody` 的 DWP2 动态力；DWP2 只会提供 `SimulationMesh`。若 mesh 是凸包 simplify 结果且体积与实机排水量不一致，保留 `displacedVolume = mass / waterDensity`，通过 `meshBuoyancyVolumeScale` 校正 mesh 体积，而不是修改 `Rigidbody.mass`。

按 mesh 三角形近似水动力：

1. 取 `surfaceHydroMeshOverride`、`surfaceHydroMeshFilter.sharedMesh` 或 `profile.surfaceHydroMesh`。
2. 将三角形转到世界坐标。
3. 可选用当前 water height 裁剪三角形。
4. 对浸没三角形累加压力、法向 form drag、切向 skin drag。
5. 将每个三角形力矩绕 `worldCenterOfMass` 累加。

关键参数：

- `maxSurfaceTriangles`：限制每步处理三角形数量。
- `clipTrianglesAtWaterSurface`：靠近水面时做裁剪。
- `pressureBuoyancyScale`：压力/浮力近似增益。
- `formDragCoefficient`：法向阻力系数。
- `skinDragCoefficient`：切向阻力系数。
- `minTriangleArea`：忽略过小三角形。

适合：

- 近水面、部分浸没、复杂几何 residual。
- 检查 hull geometry 对力矩的影响。

不建议直接使用高面数渲染 mesh。应使用专门简化过的 physics mesh。

### FossenPlusSurfaceResidual

组合 backend：

```text
total = Fossen6Dof + surfaceResidualScale * SurfaceGeometryHydro
```

默认把 Fossen 作为主模型，把 surface geometry 作为近水面或复杂几何 residual。这是目前最接近“集成各家优点”的模式：参数化 6DOF 模型保证稳定性，mesh residual 补充几何/水面效应。

## HydrodynamicsProfile

`HydrodynamicsProfile` 是 `ScriptableObject`，可通过 Unity 菜单创建：

```text
Create -> Marus -> Hydrodynamics -> Profile
```

主要参数分组：

- Fluid：`waterDensity`、`displacedVolume`、`gravityMagnitude`、`assumeFullySubmerged`。
- Centers：`centerOfMass`、`centerOfBuoyancy`。
- 6DOF Damping：`linearDamping`、`quadraticDamping`、`forwardSpeedDamping`。
- Added Mass：`addedMassDiagonal`、`addedMassFull`、`enableAddedMassForce`、`enableAddedMassCoriolis`。
- Surface Geometry Hydro：mesh、水面裁剪、三角形力参数。
- Safety Clamps：`maxForceMagnitude`、`maxTorqueMagnitude`。

`ApplyGazeboSnameDiagonal` 可将 Gazebo/DAVE 风格 SNAME 对角参数转为 profile 的 added mass / damping 字段。

## Water Provider

所有 water provider 实现：

```csharp
public interface IWaterKinematicsProvider
{
    WaterKinematicsSample Sample(Vector3 worldPoint);
}
```

`WaterKinematicsSample` 包含：

- `Height`：水面高度。
- `Normal`：水面法向。
- `FlowVelocity`：世界坐标水流速度。
- `AngularFlowVelocity`：水体角速度，通常为零。

### FlatWaterKinematicsProvider

最小 provider：

- 固定 `waterHeight`
- 固定 `currentVelocity`
- 固定 `angularCurrentVelocity`

适合 headless RL、控制器单元测试和 baseline。

### StochasticCurrentProvider

episode 开始随机采样一个水流方向和速度，并可叠加每步小噪声。

字段：

- `speedRange`
- `allowVerticalCurrent`
- `meanDirection`
- `maxAngleFromMeanDegrees`
- `noiseAmplitude`

实现 `IEpisodeRandomizable`，可以由 `DomainRandomizationCoordinator` 驱动。

### GaussMarkovCurrentProvider

一阶 Gauss-Markov 漂移水流：

```text
state <- lerp(state, meanCurrent + randomDeviation, alpha)
alpha = 1 - exp(-dt / timeConstantSeconds)
```

适合模拟缓慢变化洋流。DR 可随机化：

- `meanCurrent`
- `maxDeviation`
- `timeConstantSeconds`
- `seed`

### AnalyticWaveKinematicsProvider

解析波浪 provider，由多个 sinusoidal wave component 叠加：

- 高度：多个 cosine 波叠加。
- 法向：由坡度近似。
- 水流：按波浪 orbital velocity 近似，并随深度指数衰减。

适合不依赖 HDRP water 的可复现训练和测试。

### CompositeWaterKinematicsProvider

组合多个 provider：

- 高度取第一个有效 provider。
- 法向累加后归一化。
- `FlowVelocity` 和 `AngularFlowVelocity` 累加。

常用组合：

```text
Flat height + GaussMarkov current
Analytic waves + stochastic current
DWP2/HDRP height + custom current
```

### Dwp2WaterKinematicsAdapter

把现有 `NWH.DWP2.WaterData.WaterDataProvider` 包装为 `IWaterKinematicsProvider`。它会调用 DWP2 的：

- `GetWaterHeights`
- `GetWaterNormals`
- `GetWaterFlows`

不支持的查询使用 fallback。

### 现有 RL provider 适配

以下既有脚本已接入 `IWaterKinematicsProvider`：

- `UnityHDRPWaterDataProvider`
- `PhysicalWaveWaterDataProvider`
- `RandomizedWaterCurrentProvider`
- `WaterCurrentDomainRandomizer`

`PhysicalWaveWaterDataProvider` 还实现了 `IEpisodeRandomizable`，可根据 `DomainRandomizationProfile` 随机化 steady current 和 waves。

`WaterCurrentDomainRandomizer` 保留 legacy 直接对 Rigidbody 施加水流扰动力的模式。与 `HydrodynamicsController` 一起使用时，应关闭：

```text
applyForcesToRigidbodies = false
```

`FinsROVHydrodynamicsSetup.disableLegacyCurrentDirectForces` 会自动做这件事。

## Domain Randomization

### DomainRandomizationProfile

可通过 Unity 菜单创建：

```text
Create -> Marus -> Hydrodynamics -> Domain Randomization Profile
```

随机化分组：

- Body：质量、体积、惯量、质心、浮心。
- Hydrodynamics：added mass、线性阻尼、二次阻尼缩放。
- Thrusters：最大推力、推力常数、延迟、时间常数、slew rate。
- Water Current：流速、流向、噪声幅值、漂移时间常数。
- Water Waves：波数量、振幅、波长、周期、方向、相位、orbital flow scale。
- Initial State：初始位置、姿态、线速度、角速度。

所有采样都通过 `RandomizationContext`，由 `seed` 和 `episodeIndex` 决定，便于复现实验。

### DomainRandomizationCoordinator

`DomainRandomizationCoordinator` 负责在 episode 开始时调用所有 `IEpisodeRandomizable`：

```text
RandomizeForEpisode()
  -> ResolveTargets()
  -> seed = baseSeed + episodeIndex
  -> target.RandomizeForEpisode(context)
```

目标查找方式：

- 显式填 `targetBehaviours`
- 或 `autoFindTargets=true` 时查找自身子物体里的所有 `IEpisodeRandomizable`

命令行 override：

```bash
-fins-dr-seed 12345
-fins-dr-mode Train
```

也支持等号形式：

```bash
-fins-dr-seed=12345 -fins-dr-mode=Evaluate
```

`mode=Disabled` 时不会随机化。

### Randomization Targets

已有 target：

- `HydrodynamicsController`：随机化 runtime profile 的体积、浮心、质心、水动力系数。
- `RigidbodyRandomizationTarget`：随机化 mass 和 inertia tensor。
- `ThrusterRandomizationTarget`：随机化 `Marus.Actuators.Thruster` 参数。
- `InitialStateRandomizationTarget`：随机化初始位置、姿态和速度。
- `StochasticCurrentProvider` / `GaussMarkovCurrentProvider` / `CompositeWaterKinematicsProvider`：随机化水流。
- `PhysicalWaveWaterDataProvider`：随机化 waves 和 steady current。
- `WaterCurrentDomainRandomizer` / `RandomizedWaterCurrentProvider`：legacy provider 的 episode current randomization。

## FinsROV 接入

`FinsROVHydrodynamicsSetup` 是面向 FinsROV 的 glue component。推荐挂在 FinsROV root 上。

它负责：

- 查找并绑定 `HydrodynamicsController`。
- 将 `hydrodynamicsProfile` 填入 controller。
- 查找并绑定 water provider。
- 将 `domainRandomizationProfile` 填入 coordinator。
- 可自动添加 `RigidbodyRandomizationTarget`、`ThrusterRandomizationTarget`、`InitialStateRandomizationTarget`。
- 当 water provider 是 `WaterCurrentDomainRandomizer` 时，自动关闭 legacy 直接扰动力。

最小接入步骤：

1. 在 FinsROV root 上添加 `HydrodynamicsController`。
2. 创建并指定 `HydrodynamicsProfile`。
3. 场景中添加一个 water provider，例如 `FlatWaterKinematicsProvider` 或 `CompositeWaterKinematicsProvider`。
4. 将 provider 拖到 `HydrodynamicsController.waterProviderBehaviour`。
5. 将 `mode` 设置为 `Fossen6Dof` 或 `FossenPlusSurfaceResidual`。
6. 如需 DR，添加 `DomainRandomizationCoordinator` 和 `DomainRandomizationProfile`。
7. 添加 `FinsROVHydrodynamicsSetup`，调用 `ApplyConfiguration` 或等待 `Awake` 自动配置。

## RL Episode 接入

`FinsROVAgentRuntime.RandomizeEpisodeIfPresent(Component owner)` 会：

1. 从 agent 父级查找 `DomainRandomizationCoordinator`。
2. 找不到时查找场景中第一个 coordinator。
3. 调用 `RandomizeForEpisode()`。

多个 RL agent 的 `OnEpisodeBegin()` 已接入该调用，包括 position、velocity、acceleration、moving target、smooth near target、`PropellerPID` 和 `RollerAgent` 等脚本。

注意：如果一个 scene 里有多个独立 agent，最好把 `DomainRandomizationCoordinator` 放在对应 agent 的父级，避免全局查找命中错误 coordinator。

## Backend 选择建议

默认建议：

```text
Fossen6Dof
```

原因是它在稳定性、参数可解释性、与 DAVE/Gazebo/Fossen 模型一致性之间最平衡。

训练早期或参数不确定：

```text
EmpiricalDrag6Dof
```

它更稳定，参数更少，适合先把策略训练流程跑通。

近水面或需要几何 residual：

```text
FossenPlusSurfaceResidual
```

需要准备简化 surface mesh，并调低 `surfaceResidualScale`，避免 mesh 近似力盖过参数化主模型。

只做浮力/重力检查：

```text
HydrostaticOnly
```

排查姿态漂移、浮心/质心设置时最有用。

不建议长期使用：

```text
SurfaceGeometryHydro
```

它适合分析和 residual，不适合作为高频训练的唯一主模型，除非 mesh 很简单且参数已验证。

## 与 DWP2 共存

旧 DWP2 `WaterObject` 仍可保留，用于体积、采样和已有水体系统兼容。但要避免两套系统同时施加完整水动力。

推荐：

- 参数化 backend 时：`disableDwp2WaterObjectsForParametricBackends=true`。
- surface residual 时：可用 `keepMainBodyWaterObjectForSurfaceBackend=true` 保留主 hull。
- legacy current 作为 provider 时：关闭 `applyForcesToRigidbodies`。

如果发现车辆阻尼异常大、几乎动不了，优先检查是否重复启用了 DWP2 hydrodynamic force 和新 backend。

## DWP2 机体系 6DOF 缩放

当前 DWP2 `WaterObject` 已在源码层支持只缩放 hydrodynamic dynamic force/torque，不缩放浮力。默认配置：

- `hydrodynamicAxisScalingEnabled=true`
- `hydrodynamicAxisConvention=FinsRovXForwardYUpZLeft`
- `hydrodynamicForceAxisScale=(1,1,1)`，对应 surge/sway/heave
- `hydrodynamicTorqueAxisScale=(1,1,0.25)`，对应 roll/pitch/yaw
- `hydrodynamicYawDampingMode=MatchRealYawDampingCurve`
- `hydrodynamicYawLinearDamping=0.303`
- `hydrodynamicYawQuadraticDamping=0.476`

在 `MatchRealYawDampingCurve` 模式下，DWP2 不再用固定倍率决定 yaw 动态阻尼，而是按实机辨识曲线计算：

```text
N_yaw = -(0.303 * r + 0.476 * |r| * r)
```

其中 `r` 是 FinsROV 机体系 yaw rate。源码会按 Rigidbody 聚合同一潜器上的多个 `WaterObject`，避免每个 mesh 都重复施加整机 yaw 阻尼。由于 yaw 绕垂直轴理论上不应有静水恢复力，`MatchRealYawDampingCurve` 还会把最终机体系 yaw torque 覆盖为这条实机阻尼曲线对应的 torque；浮力本身仍保留给 heave/roll/pitch 配平使用。

如果需要退回固定倍率调试，把 `hydrodynamicYawDampingMode` 改为 `AxisScaleOnly`。训练时如仍希望随机化固定倍率，可挂载 `Dwp2AxisHydrodynamicScaleRandomizer`，默认只随机化 yaw torque scale：

```text
baselineTorqueAxisScale = (1, 1, 0.25)
yawScaleRange = 0.18 ~ 0.40
```

不要再同时启用 `Dwp2BodyYawTorqueScaler`。该脚本已废弃，若和 DWP2 源码缩放同时开启，会对 yaw torque 重复缩放，导致实机迁移判断混乱。

## 验证命令

命令行编译：

```bash
PROJECT_ROOT="$(git rev-parse --show-toplevel)"
cd "$PROJECT_ROOT"
dotnet restore FinsSimUnity.slnx
dotnet build FinsSimUnity.slnx --no-restore
```

Hydrodynamics filtered EditMode 测试：

```bash
PROJECT_ROOT="$(git rev-parse --show-toplevel)"
UNITY_EDITOR="${UNITY_EDITOR:-$HOME/Unity/Hub/Editor/6000.3.20f1/Editor/Unity}"
xvfb-run -a "$UNITY_EDITOR" \
  -batchmode \
  -projectPath "$PROJECT_ROOT" \
  -runTests \
  -testPlatform EditMode \
  -testFilter HydrodynamicsCoreTest \
  -testResults "$PROJECT_ROOT/Logs/hydrodynamics_core_results.xml" \
  -logFile "$PROJECT_ROOT/Logs/hydrodynamics_core.log"
```

当前验证状态：

- `dotnet build`：通过，0 error，保留项目已有 warning。
- `HydrodynamicsCoreTest`：此前 4/4 通过；新增闭合 mesh 的全浸没与半浸没体积积分测试，需在关闭当前 Editor 后运行上述命令验证。
- 全量 EditMode：新增 Hydrodynamics 测试通过；当前环境下仍有既有 `DvlTest` 和 `ImuTest` 失败，和本模块无关。

## 已知限制

- `SurfaceGeometryHydro` 是近似三角面模型，不是完整 CFD 或高保真 panel method。
- `Fossen6Dof` 的相对加速度由 fixed step 差分估计，强噪声场景需要调 `accelerationFilterAlpha`。
- 当前 added-mass Coriolis 实现按 6x6 added mass 近似构造，适合仿真稳定性和工程使用，但仍需要用具体载体验证参数。
- DR 当前是 episode-level randomization，不处理 episode 内连续随机化。连续洋流扰动应放在 water provider，例如 `GaussMarkovCurrentProvider`。
- 真实车辆参数需要从 DAVE/Gazebo/实验数据/辨识结果导入到 `HydrodynamicsProfile`，默认值只适合 smoke test。

## 后续扩展点

- 增加 profile importer，将 DAVE/Gazebo SDF/URDF hydrodynamic tags 自动转为 `HydrodynamicsProfile`。
- 为不同车辆生成 profile asset preset，例如 ROV、torpedo AUV、glider。
- 增加 scene editor 工具，一键给 FinsROV 添加 controller、provider、DR targets。
- 对 `SurfaceGeometryHydro` 增加 mesh decimation / triangle sampling asset pipeline。
- 把 `HydrodynamicsController.LastWrench` 接入统一 telemetry / ROS topic。
