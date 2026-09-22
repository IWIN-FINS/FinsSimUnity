# CFD 水动力系数测定与 marus-example 参数迁移方案

本文说明如果使用 CFD 软件为 `marus-example` 的水动力 backend 提供参数，应该测什么、怎么测、哪些参数不能靠 CFD 得到，以及推荐的软件和 pipeline。

目标不是用 CFD 完全替代实测，而是形成一个可复现的参数来源：

1. CAD/实物测量给刚体质量、重心、惯量、排水体积、浮心。
2. CFD 给主要水动力阻尼、added mass、部分交叉导数和推进器/艇体干扰初值。
3. 水池实验只做少量关键轴向校准和 benchmark。
4. Unity RL 训练用 domain randomization 覆盖 CFD、制造和环境不确定性。

## marus-example 需要哪些参数

当前水动力参数入口是 `HydrodynamicsProfile`，主要字段如下：

| Unity 字段                                    | 物理含义                                   | 推荐来源                              |
| --------------------------------------------- | ------------------------------------------ | ------------------------------------- |
| `waterDensity`                              | 水密度                                     | 实验水体/仿真场景设定                 |
| `displacedVolume`                           | 排水体积                                   | watertight CAD/实物排水测试           |
| `centerOfMass`                              | 刚体重心，Unity local frame                | CAD 质量属性/吊挂/配平实验            |
| `centerOfBuoyancy`                          | 浮心，Unity local frame                    | watertight CAD 体积积分               |
| `linearDamping [u,v,w,p,q,r]`               | 6DOF 线性阻尼                              | CFD captive tests + 少量实测修正      |
| `quadraticDamping [u,v,w,p,q,r]`            | 6DOF 二次阻尼                              | CFD 速度/角速度扫幅拟合               |
| `forwardSpeedDamping`                       | 前进速度相关附加阻尼                       | 可选，斜航/PMM CFD；没有数据时先置零  |
| `addedMassDiagonal [u,v,w,p,q,r]`           | 对角 added mass / added inertia            | 非定常 CFD、potential/BEM、半经验初值 |
| `addedMassFull`                             | 6x6 added mass 矩阵                        | 高成本 CFD/PMM；先不建议作为第一版    |
| `formDragCoefficient`                       | surface geometry backend 法向/形状阻力尺度 | CFD 反推或调参                        |
| `skinDragCoefficient`                       | surface geometry backend 切向摩擦尺度      | CFD 壁面剪切/经验公式                 |
| `surfaceResidualScale`                      | Fossen + mesh residual 的混合比例          | Unity benchmark 调参                  |
| `maxForceMagnitude`, `maxTorqueMagnitude` | 数值安全限幅                               | 仿真稳定性设置，不是物理系数          |

Unity 当前 Fossen 速度顺序是：

```text
[u, v, w, p, q, r]
```

但 Unity local 轴到 Fossen 轴的映射是：

```text
u = localLinear.z
v = localLinear.x
w = -localLinear.y
p = localAngular.z
q = localAngular.x
r = -localAngular.y
```

因此 CFD、ROS、CAD 坐标系必须先统一到这个顺序，再填 profile。不要直接把 CFD 软件里的全局 X/Y/Z 结果填进 Unity 字段。

## CFD 能测到什么

### 1. 平动阻尼

对应 `linearDamping.u/v/w` 和 `quadraticDamping.u/v/w`。

测法是 captive steady CFD：艇体固定，给定相对来流速度，分别做 pure surge、pure sway、pure heave。

推荐工况：

```text
surge: U = +/- [0.05, 0.10, 0.20, 0.35, 0.50] m/s
sway:  V = +/- [0.03, 0.06, 0.12, 0.20, 0.30] m/s
heave: W = +/- [0.03, 0.06, 0.12, 0.20, 0.30] m/s
```

具体速度按你的水池和任务速度缩放。ROV 低速控制时，低速点比高速点更重要。

每个工况记录艇体总水动力：

```text
X_h, Y_h, Z_h
```

然后对每个自由度拟合：

```text
tau_i = -d1_i * nu_i - d2_i * abs(nu_i) * nu_i
```

在 Unity 中填正数：

```text
linearDamping[i] = abs(d1_i)
quadraticDamping[i] = abs(d2_i)
```

如果正负方向不对称，建议分别拟合正向/负向，再取平均作为 nominal，把差异放进 domain randomization。

### 2. 转动阻尼

对应 `linearDamping.p/q/r` 和 `quadraticDamping.p/q/r`。

测法是 prescribed rotation CFD：让艇体以固定角速度绕自身某轴旋转，或者用 rotating reference / overset mesh / dynamic mesh 施加运动，记录力矩。

工况：

```text
roll:  p = +/- [0.05, 0.10, 0.20, 0.35] rad/s
pitch: q = +/- [0.05, 0.10, 0.20, 0.35] rad/s
yaw:   r = +/- [0.05, 0.10, 0.20, 0.35] rad/s
```

拟合：

```text
K_h = -Kp * p - Kp_abs_p * abs(p) * p
M_h = -Mq * q - Mq_abs_q * abs(q) * q
N_h = -Nr * r - Nr_abs_r * abs(r) * r
```

Unity 填：

```text
linearDamping.p/q/r
quadraticDamping.p/q/r
```

注意 roll/pitch 在实物上很难大角度翻转，但 CFD 不需要完整翻转。可以直接做小角速度 forced rotation，也可以做小幅正弦 forced oscillation。

### 3. Added mass / added inertia

对应 `addedMassDiagonal` 或 `addedMassFull`。

Added mass 是速度变化时流体带来的惯性反力。Fossen 形式中 added mass 是 6DOF 模型的重要部分；Fossen/Fjellstad 的 6DOF 表达允许完整 added inertia 矩阵，并强调矩阵结构、对称性和正定性。

可选测法有三类。

#### 方法 A：恒加速度非定常 CFD

对每个自由度施加分段速度：

```text
constant velocity -> constant acceleration -> constant velocity -> deceleration
```

记录总水动力。恒速段主要是阻尼，变速段包含阻尼和 added mass。用差分分离：

```text
tau_h(t) = -D(nu) * nu - M_A * nudot
```

已知或已拟合 `D(nu)` 后：

```text
M_A ~= -(tau_h + D(nu) * nu) / nudot
```

Javanmard 等用 unsteady RANS 模拟线性加速，从速度相关力和加速度相关力中提取水下航行器平动 added mass，并用椭球解析/实验数据验证。这个方法适合先得到 `addedMassDiagonal.u/v/w`。

#### 方法 B：forced oscillation / virtual PMM

对某个自由度施加正弦运动：

```text
x(t) = A sin(omega t)
nu(t) = A omega cos(omega t)
nudot(t) = -A omega^2 sin(omega t)
```

拟合：

```text
tau_h(t) = -D * nu(t) - M_A * nudot(t)
```

力/力矩中与速度同相的部分对应 damping，与加速度同相的部分对应 added mass。多频率结果可能不同；Unity 控制仿真建议使用低频等效值。

这个方法也可以测：

```text
X_udot, Y_vdot, Z_wdot, K_pdot, M_qdot, N_rdot
```

如果同时记录其他自由度的力/力矩，也可以得到交叉项，例如 `Y_rdot`、`N_vdot`。

#### 方法 C：potential flow / BEM

可以用 WAMIT、Nemoh、Capytaine 等边界元/势流工具快速得到 added mass 和 radiation damping 的初值。

优点：

- 比 RANS 便宜。
- 对封闭光顺艇体的 added mass 初值有用。
- 可以快速得到 6x6 惯性类矩阵。

局限：

- 忽略粘性，不能可靠给 open-frame ROV 的阻尼。
- 对推进器、开架结构、网格支架、涡脱落影响不足。
- 结果应该作为初值，不应直接当最终参数。

### 4. 交叉耦合导数

如果只使用 `addedMassDiagonal` 和对角阻尼，第一版不需要完整交叉项。

如果后续要更高保真，可以通过这些 CFD 工况获得：

| 工况                              | 可得到的典型耦合                                  |
| --------------------------------- | ------------------------------------------------- |
| oblique towing，固定偏航角/侧滑角 | `X_v`, `Y_u`, `N_u`, `N_v` 等方向耦合阻尼 |
| pure sway PMM                     | `Y_v`, `N_v`, `Y_vdot`, `N_vdot`          |
| pure yaw PMM / rotating arm       | `Y_r`, `N_r`, `Y_rdot`, `N_rdot`          |
| heave-pitch oscillation           | `Z_q`, `M_w`, `Z_qdot`, `M_wdot`          |
| full 6DOF prescribed acceleration | `addedMassFull`                                 |

当前 Unity 的 `Fossen6DofBackend` 已支持 `addedMassFull`，但阻尼仍是对角形式。因此第一阶段只建议启用 full added mass，不建议为了交叉阻尼大改 Unity，除非 benchmark 证明对任务有明显收益。

### 5. 推进器和艇体干扰

CFD 可以测：

- 单推进器 open-water 曲线：`T = f(rpm, advance speed)`。
- 推进器安装到艇体后的 thrust deduction / wake fraction。
- 多推进器互相干扰。

但这部分 CFD 对网格、旋转域、MRF/sliding mesh、推进器几何很敏感。实际工程中建议：

1. 推进器静水推力曲线优先实测。
2. CFD 只用于估计安装位置导致的推力损失和流场遮挡。
3. Unity 中推进器最大推力、死区、响应延迟应由实测或硬件日志给出。

## CFD 测不到或不应该由 CFD 决定的参数

### `Rigidbody.mass`

不能由 Unity 凸包、渲染 mesh 或 CFD 得到。

`rb.mass` 应该填潜器真实空气中质量：

```text
rb.mass = 真实整机质量，包含电池、壳体、推进器、配重、密封件、线缆固定件等
```

如果 CAD 质量不准，优先称重。Unity 里的 convex hull 只用于碰撞，不能决定质量。

### `Rigidbody.centerOfMass` / `HydrodynamicsProfile.centerOfMass`

来源优先级：

1. CAD 装配质量属性。
2. 实物吊挂法/多点支撑称重。
3. 水中配平结果反推。

如果潜器稳态 roll/pitch 都为 0，说明重心和浮心的相对位置产生了稳定恢复力，但这不能单独确定重心绝对位置。

### `Rigidbody.inertiaTensor`

来源优先级：

1. CAD 装配质量属性。
2. bifilar/trifilar pendulum 摆振实验。
3. 简化盒体/圆柱体估计，再用 roll/pitch/yaw 动态响应校正。

CFD 的 rotational added inertia 是 `addedMassDiagonal.p/q/r`，不是刚体惯量。二者不能混填。

### `displacedVolume` 和 `centerOfBuoyancy`

这两个更适合由 watertight CAD 体积积分得到，不需要 CFD。

如果模型不闭合：

1. 先清理出单独的 watertight outer hull。
2. 去掉内部零件。
3. 保留会排水的封闭外形。
4. 对开架、推进器、支架可单独估算 displaced volume 后相加。

### Bias、线缆、水池壁面效应

这些不应填进 hydrodynamic damping 或 added mass。

处理方式：

- bias：作为实测数据质量指标，必要时建成外部扰动力。
- 线缆拖拽：单独建 tether force model，或在 domain randomization 中扩大阻尼范围。
- 小水池壁面：不要把贴壁效应当成固有艇体系数；只用于实验误差评估。
- 水流扰动：放到 `IWaterKinematicsProvider`，不是艇体 profile。

## 软件选择

### 推荐结论

如果没有商业授权，首选：

```text
OpenFOAM + ParaView + Python post-processing
```

理由：

- 开源，可放进项目 pipeline。
- Linux/HPC 友好。
- 支持 RANS、动态网格、overset mesh、prescribed motion。
- 适合批量扫速度、扫角速度、输出 forces/moments CSV。

如果有商业授权并希望少踩坑，首选：

```text
STAR-CCM+
```

理由：

- 自动网格、overset、DFBI、报告系统和批处理体验更好。
- 做 forced motion、rotating body、推进器 MRF/sliding mesh 通常更省时间。
- 适合工程团队快速建立稳定模板。

如果团队已有 Ansys 生态：

```text
Ansys Fluent / CFX
```

理由：

- 动态网格、6DOF、MRF/sliding mesh、UDF 能力完整。
- 商业支持强。
- added mass 的恒加速度 RANS 方法已有文献使用 CFX。

如果只想快速云端试算：

```text
SimScale
```

适合稳态阻力和简单工况，不建议作为长期可复现的批量系数识别 pipeline。

### 软件对比

| 软件                  | 适合测什么                                              | 优点                   | 缺点                       | 对本项目建议           |
| --------------------- | ------------------------------------------------------- | ---------------------- | -------------------------- | ---------------------- |
| OpenFOAM              | 阻尼、added mass、overset/prescribed motion、批量 sweep | 免费、可脚本化、可复现 | 学习成本高，模板要自己维护 | 默认推荐               |
| STAR-CCM+             | 全流程工程 CFD、DFBI、overset、推进器                   | 稳定、省时间、后处理强 | 商业授权贵                 | 有 license 时最省力    |
| Ansys Fluent/CFX      | RANS、dynamic mesh、6DOF、推进器                        | 工业成熟、资料多       | UDF/批处理和授权成本       | 团队已有 Ansys 时使用  |
| Capytaine/Nemoh/WAMIT | added mass/radiation 初值                               | 快、适合 6x6 惯性初值  | 不能给粘性阻尼             | 可作为 added mass 初筛 |
| SimScale              | 简单阻力云端计算                                        | 不用本地环境           | 自动化和复杂运动受限       | 只建议试算             |

## 推荐 CFD pipeline

### Step 0：准备几何

输入：

- 原始 CAD：STEP/Parasolid 优先。
- 外表面 watertight mesh：STL/OBJ。
- 坐标系定义文档。
- 真实质量、重心、惯量初值。

处理要求：

1. 不使用 Unity convex hull 作为 CFD 几何。
2. 删除内部不可见零件。
3. 保留会显著影响外流的支架、推进器导管、外露框架。
4. 小螺丝、小倒角、文字、传感器小孔可简化。
5. 保证尺度单位是米。
6. 标出 body origin，最好与 Unity Rigidbody / Fossen body frame 一致。

输出：

```text
vehicle_outer_hull.stl
vehicle_cfd_frame.json
vehicle_mass_properties.json
```

### Step 1：建立基准 CFD 模板

建议第一版：

```text
flow: incompressible, single phase
turbulence: RANS k-omega SST
free surface: off, fully submerged
fluid density: 997 kg/m^3 freshwater or 1025 kg/m^3 seawater
```

如果测试水深很浅或接近水面，再单独做 free-surface VOF；不要一开始就把自由液面加进去，否则计算成本和不确定性都会上升。

计算域经验值：

```text
upstream: 3-5 L
downstream: 8-15 L
side/top/bottom: 4-8 L
```

网格要求：

- 艇体附近局部加密。
- 尾流区加密。
- 至少做 coarse/medium/fine 三套网格收敛。
- 如果解析边界层，目标 `y+ ~= 1`。
- 如果使用壁函数，目标 `y+` 落在壁函数适用区间，通常约 30-100。

### Step 2：平动阻尼 sweep

每个自由度单独跑，输出：

```text
case_id, axis, velocity, force_x, force_y, force_z, moment_x, moment_y, moment_z
```

建议目录：

```text
cfd_cases/
  damping_surge/
    U_pos_0p05/
    U_pos_0p10/
    ...
    U_neg_0p05/
  damping_sway/
  damping_heave/
```

拟合脚本统一输出：

```json
{
  "axis": "u",
  "linear_damping": 12.3,
  "quadratic_damping": 45.6,
  "rmse": 0.8,
  "r2": 0.97,
  "source_cases": [...]
}
```

### Step 3：转动阻尼 sweep

使用 prescribed rotation 或 overset dynamic mesh，记录力矩。

目录：

```text
cfd_cases/
  damping_roll/
  damping_pitch/
  damping_yaw/
```

输出：

```json
{
  "axis": "r",
  "linear_damping": 0.42,
  "quadratic_damping": 1.10,
  "rmse": 0.03,
  "r2": 0.95
}
```

### Step 4：added mass sweep

第一版只做 diagonal：

```text
u_dot, v_dot, w_dot, p_dot, q_dot, r_dot
```

每个 case 使用 prescribed acceleration 或 forced oscillation，输出时间序列：

```text
t, nu_i, nudot_i, X, Y, Z, K, M, N
```

拟合：

```text
tau_i = -d1_i * nu_i - d2_i * abs(nu_i) * nu_i - ma_i * nudot_i
```

其中 `d1_i`、`d2_i` 可以固定为 Step 2/3 的阻尼结果，也可以联合拟合。推荐先固定阻尼再拟合 added mass，避免参数互相吞噬。

输出：

```json
{
  "axis": "u",
  "added_mass": 4.8,
  "fit_method": "prescribed_acceleration",
  "acceleration_range": [0.05, 0.20],
  "rmse": 0.5
}
```

### Step 5：可选耦合项

如果主任务是 RL 控制、避障、低速巡航，通常先跳过。

如果 benchmark 发现斜航、急转、yaw-sway 耦合误差明显，再增加：

```text
oblique towing
pure sway PMM
pure yaw PMM
heave-pitch forced oscillation
```

这些结果可以先作为报告保存。Unity 当前主要可直接消费的是 `addedMassFull`；交叉阻尼需要后续扩展 backend。

### Step 6：生成 Unity 参数文件

建议 CFD 后处理输出一个中间 JSON：

```json
{
  "frame": "fossen_u_v_w_p_q_r",
  "water_density": 997.0,
  "linear_damping": [12.3, 18.1, 22.0, 0.20, 0.35, 0.42],
  "quadratic_damping": [45.6, 70.2, 80.0, 0.75, 0.95, 1.10],
  "added_mass_diagonal": [4.8, 9.2, 10.5, 0.05, 0.08, 0.12],
  "fit_quality": {
    "u": {"r2": 0.97, "rmse": 0.8},
    "v": {"r2": 0.94, "rmse": 1.1}
  }
}
```

迁移到 Unity：

```text
HydrodynamicsProfile.linearDamping = abs(linear_damping)
HydrodynamicsProfile.quadraticDamping = abs(quadratic_damping)
HydrodynamicsProfile.addedMassDiagonal = abs(added_mass_diagonal)
HydrodynamicsProfile.useFullAddedMassMatrix = false
HydrodynamicsProfile.enableAddedMassForce = true
HydrodynamicsProfile.enableAddedMassCoriolis = true after validation
```

如果 Unity 单轴响应出现高频抖动，先把：

```text
enableAddedMassCoriolis = false
```

或者降低 `accelerationFilterAlpha`，再逐步打开。

### Step 7：Unity benchmark

每个主自由度至少做：

1. 阶跃推力/力矩响应。
2. 停推后的自由衰减。
3. 稳态速度/角速度对比。
4. 与实测水池数据对比。

指标：

```text
velocity RMSE
acceleration RMSE
steady-state velocity error
rise time error
decay time constant error
yaw/roll/pitch angle RMSE
```

如果误差大：

1. 先检查坐标轴和符号。
2. 再检查 `rb.mass`、惯量、重心、浮心。
3. 再检查推进器推力曲线。
4. 最后才调 damping / added mass。

## 不确定参数如何处理

### 用 domain randomization 表示不确定性

对 CFD 得到但没有充分实测验证的参数，建议设置随机范围：

```text
addedMassScale:       0.7 - 1.3
linearDampingScale:   0.7 - 1.4
quadraticDampingScale:0.6 - 1.6
volumeScale:          0.98 - 1.02
centerOfBuoyancyOffset: +/- 0.5 - 2 cm
```

如果 CFD 网格收敛很好、实测校准也吻合，可以收窄范围。如果没有实测，范围应该更宽。

### CFD 没算的 roll/pitch 参数

如果暂时没有 roll/pitch CFD：

1. Added inertia：用 CAD 惯量量级的 5%-30% 作为初值。
2. 阻尼：用 yaw 阻尼按投影面积和特征长度缩放。
3. 恢复力：由 `centerOfMass` 与 `centerOfBuoyancy` 的相对位置决定，不填到 damping。
4. 用小角度自由衰减实测校正 roll/pitch damping 和恢复刚度。

### CFD 没算 full added mass

先用 diagonal。

只有当这些现象明显时再做 full matrix：

- 斜航时 yaw/sway 误差大。
- pitch-heave 强耦合。
- 推进器布局导致 roll/yaw 耦合明显。
- 控制器在仿真中稳定但实物中明显串轴。

## 最小可执行版本

如果只想尽快给 `marus-example` 一个可用 profile：

1. CAD/称重得到 `rb.mass`、`centerOfMass`、`inertiaTensor`、`displacedVolume`、`centerOfBuoyancy`。
2. OpenFOAM 或 STAR-CCM+ 做 6 个方向的 steady damping sweep。
3. 用 BEM/等效椭球/简化非定常 CFD 得到 `addedMassDiagonal` 初值。
4. 在 Unity 用 `Fossen6Dof` 跑单轴 benchmark。
5. 用少量水池实测修正 surge、heave、yaw、roll/pitch 衰减。
6. RL 训练打开 domain randomization。

第一版不建议一开始追求：

- full 6x6 damping。
- 高阶三次阻尼。
- 完整推进器滑网格。
- 自由液面 + 线缆 + 池壁全耦合。

这些会显著增加成本，但对当前低速水下 RL 和控制验证不一定带来同比收益。

## 推荐项目产物结构

建议在 CFD 项目或 `reference/cfd` 下保存：

```text
cfd/
  geometry/
    vehicle_outer_hull.step
    vehicle_outer_hull.stl
    vehicle_cfd_frame.md
    mass_properties.json
  cases/
    openfoam/
      damping_surge/
      damping_sway/
      damping_heave/
      damping_roll/
      damping_pitch/
      damping_yaw/
      added_mass_u/
      added_mass_v/
  postprocess/
    fit_hydrodynamic_coefficients.py
    fitted_coefficients.json
    plots/
  unity_export/
    FinsROV_HydrodynamicsProfile.json
    fitting_report.md
```

所有 case 都应记录：

```text
software version
mesh size
turbulence model
water density
boundary conditions
velocity/acceleration profile
force/moment integration surface
coordinate transform to Fossen frame
```

否则后续参数很难复现。

## 调研来源

- Fossen marine craft model：6DOF marine craft 方程、`[u,v,w,p,q,r]` 和 `[X,Y,Z,K,M,N]` 表达、added mass/damping/restoring 结构。https://fossen.biz/html/marineCraftModel.html
- Fossen and Fjellstad, 1995：6DOF nonlinear marine vehicle vector parameterization、added inertia matrix、Coriolis/centripetal 结构。https://www.fossen.biz/publications/1995%20Fossen%20and%20Fjellstad%20JMMS.pdf
- Javanmard, Mansoorzadeh, Mehr, 2020：用 unsteady RANS / 线性加速方法提取水下航行器 translational added mass。https://doi.org/10.1016/j.oceaneng.2020.107857
- Mishra, Vengadesan, Bhattacharyya, 2011：用 CFD 计算轴对称水下航行器带前进速度的 translational added mass。https://doi.org/10.5957/jsr.2011.55.3.185
- Frontiers in Marine Science, 2025：通过 CFD 对 surge/sway/heave 平移和 roll/pitch/yaw 转动分别计算阻尼系数，并拟合推进器/尾鳍 thrust coefficients。https://www.frontiersin.org/journals/marine-science/articles/10.3389/fmars.2025.1648335/full
- OpenFOAM overset documentation：OpenFOAM overset framework 支持静态和动态 overset meshes。https://doc.openfoam.com/2312/tools/processing/numerics/overset/
- OpenFOAM standard solvers：OpenFOAM incompressible solver family，包括 steady SIMPLE 和 transient PIMPLE 类求解器。https://www.openfoam.com/documentation/user-guide/a-reference/a.1-standard-solvers
- Ansys Fluent 6DOF dynamic mesh documentation：Fluent dynamic mesh 中可使用 6DOF solver 或显式 prescribed motion。https://www.afs.enea.it/project/neptunius/docs/fluent/html/ug/node401.htm
- STAR-CCM+ overset/DFBI 说明：overset mesh 可与 DFBI 结合处理运动物体和流体力耦合。
  https://www.cosmositalia.it/wp-content/softwaredoc/siemens/starccm/Siemens-PLM-CD-adapco-STAR-CCM-overset-mesh-fs-59886-A2.pdf
