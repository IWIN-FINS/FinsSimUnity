# MARUS Core 代码结构与工作机制梳理

本文基于当前 Unity 项目中的 `Packages/com.iwin-fins.fins-sim/` 代码整理，目标是回答这几个问题：

1. `marus-core` 主要在做什么
2. 它由哪几部分构成
3. 各部分大致写了什么代码
4. 它是怎么支持多个传感器、多个设备、多个机器人对象的

## 1. 先说结论：`marus-core` 不只是“水模拟”

`marus-core` 更准确地说，是一个面向海洋/水下机器人仿真的 Unity 基础库，而不只是单纯的水体求解器。

它主要做了 6 类事情：

1. 提供海洋机器人常见传感器的仿真
   - IMU、GNSS、Depth、Pose、DVL
   - RGB Camera
   - Lidar
   - 2D/3D Sonar
   - AIS
   - LoRa ranging
2. 提供执行器和载具控制
   - 推进器 `Thruster`
   - 多推进器控制器 `ThrusterController`
   - AUV/ASV 键盘控制器
   - ROS 远程控制器
3. 提供 Unity 与 ROS/gRPC 的桥接
   - 共享 gRPC 连接
   - 传感器数据流式上报
   - 远程控制命令接收
   - TF 发布、时间同步、参数服务器
4. 提供通信介质仿真
   - 声学通信 `Nanomodem`
   - RF/LoRa 通信
   - 介质中的传播延时与距离判断
5. 提供海洋环境相关能力
   - 水面高度采样
   - 浮力/船体水动力
   - 水下后处理效果
6. 提供可视化、标注、日志与测试支持

所以从工程角色看，`marus-core` 是整个 MARUS 仿真项目的“仿真核心库”。

## 2. 目录构成

`Packages/com.iwin-fins.fins-sim/Runtime/Platform/` 下最核心的模块大致如下：

- `Sensors/`：传感器本体、采样框架、ROS 输出包装
- `Actuators/`：推进器和推进分配
- `Controllers/`：AUV/ASV/相机/鱼类等控制器
- `Ros/`：gRPC/ROS 连接、TF、时间、服务流
- `Communications/`：声学与 RF 通信设备和协议
- `BoatPhysics/`：船体浮力、水阻、拍击力等
- `Ocean/`：水面高度采样
- `Visualization/`：点云、轨迹、Marker 可视化
- `ObjectAnnotation/`：目标检测/分割保存
- `DataLogger/`：数据记录
- `protobuf/`：gRPC/Protobuf 自动生成代码

按 C# 文件数量粗看，`Sensors/` 是最大头，其次是 `protobuf/`、`Visualization/`、`Communications/`、`Controllers/`。这也说明 `marus-core` 的重心首先是“传感器仿真 + 对外通信”。

## 3. 核心运行链路

MARUS Core 最关键的主链路可以概括成：

`Unity 物体/物理状态 -> 传感器采样 -> 本地数据缓存 -> ROS/gRPC 消息封装 -> 按地址流式发送`

### 3.1 传感器基类：`SensorBase`

文件：`Packages/com.iwin-fins.fins-sim/Runtime/Platform/Sensors/Core/SensorBase.cs`

这个类定义了所有传感器的共同骨架：

- 每个传感器都有：
  - `frameId`
  - `SampleFrequency`
  - `vehicle`
  - `hasData`
- 在 `Awake()` 中自动向全局采样器 `SensorSampler` 注册回调
- 子类只需要实现 `SampleSensor()`
- `OnEnable/OnDisable` 会启停自己的采样回调
- `UpdateVehicle()` 会自动生成类似
  - `vehicleName/sensorName_frame`
  的 `frameId`

这意味着：每个具体传感器只关心“如何采样”，不需要自己管理统一调度。

### 3.2 全局采样调度：`SensorSampler`

文件：`Packages/com.iwin-fins.fins-sim/Runtime/Platform/Sensors/Core/SensorSampler.cs`

这是多传感器支持的第一层关键机制。

它本质上是一个单例调度器：

- 用 `Dictionary<int, SensorCallback>` 保存所有传感器实例
- key 是 `sensor.GetInstanceID()`
- 在 `FixedUpdate()` 中遍历所有注册传感器
- 根据每个传感器自己的 `SampleFrequency` 决定是否采样

也就是说，**多个传感器并不是通过写死列表管理的，而是每个传感器实例在启动时自动注册，采样器统一按实例 ID 调度**。

这带来几个直接效果：

1. 同一种传感器可以挂多个实例
2. 不同类型传感器可以混挂在同一个载具上
3. 每个实例都能有自己独立的采样频率
4. 启用/禁用某个组件不会影响其它传感器

### 3.3 ROS/gRPC 流式发送基类：`SensorStreamer<TClient, TMsg>`

文件：`Packages/com.iwin-fins.fins-sim/Runtime/Platform/Sensors/Core/SensorStreamer.cs`

这是多传感器支持的第二层关键机制。

它负责把本地传感器数据发送到 ROS/gRPC：

- 每个 ROS 包装器继承 `SensorStreamer<TClient, TMsg>`
- `StreamSensor(...)` 绑定具体传感器和 gRPC client stream
- `ComposeMessage()` 由子类实现，把当前传感器状态转成 protobuf 消息
- `Start()` 中自动：
  - 对齐发送频率与采样频率
  - 自动生成 `address`
  - 建立消息队列和发送线程
- `FixedUpdate()` 周期性调用 `SendMessage()`
- 发送不是直接同步写流，而是先入 `ConcurrentQueue<TMsg>`
- 后台线程 `SendMessagesThread()` 异步写入 gRPC stream

这里的核心价值是：

1. 采样频率和发送频率解耦
2. 每个传感器实例有自己的消息队列
3. 每个传感器实例有自己的 gRPC stream 发送逻辑
4. 队列满了会丢最早消息，避免阻塞 Unity 主线程

所以从机制上讲，**MARUS 支持多个传感器，本质上就是“多个 `SensorBase` 实例 + 多个 `SensorStreamer` 实例并行工作”**。

### 3.4 共享连接层：`RosConnection`

文件：`Packages/com.iwin-fins.fins-sim/Runtime/Platform/Ros/RosConnection.cs`

`RosConnection` 是所有 ROS/gRPC 通信的总入口：

- 管理 gRPC `Channel`
- 维护客户端注册表 `Dictionary<Type, ClientBase>`
- 初始化常用 client：
  - `SensorStreamingClient`
  - `RemoteControlClient`
  - `PingClient`
  - `TfClient`
  - `AcousticTransmissionClient`
  - `VisualizationClient`
  - `LoraTransmissionClient`
- 启动连接检测
- 创建时间、TF、可视化等单例

也就是说，所有传感器 streamer 并不各自建立网络连接，而是**共享同一个 `RosConnection` 和同一套 gRPC client 池**。

## 4. 传感器模块具体由什么组成

`Sensors/` 内部又分成几层：

### 4.1 `Sensors/Core/`

这是框架层，不是某一个具体传感器：

- `SensorBase.cs`：传感器基类
- `SensorSampler.cs`：统一调度器
- `SensorStreamer.cs`：统一 ROS 流发送基类
- `RaycastJobHelper.cs`：并行 raycast 辅助类
- `DepthCameras.cs`、`ComputeBufferDataExtractor.cs`、`SphericalProjectionFilter.cs`：GPU/Compute 相关处理
- `CameraFrustum.cs`：相机视锥相关

其中 `RaycastJobHelper.cs` 很重要，它封装了：

- 射线方向生成
- `RaycastCommand` 批量发射
- `JobHandle` 异步回读
- 按 `SampleFrequency` 控制轮询

Lidar 和 Sonar 都大量复用了它。

### 4.2 `Sensors/Primitive/`

这是“基础物理量/状态型传感器”：

- `DepthSensor.cs`
  - 直接取 `-transform.position.y` 作为深度
- `ImuSensor.cs`
  - 从 `Rigidbody` 计算局部速度、角速度、线加速度、姿态
  - 支持噪声模型
- `GnssSensor.cs`
  - 从 Unity 世界坐标转换到地理坐标
- `PoseSensor.cs`
  - 输出位置、姿态、线速度、角速度
- `DvlSensor.cs`
  - 通过多个子 `RangeSensor` 求海底高度，同时从位移差分得到速度
- `SonarPrimitive.cs`
  - 比较简化的目标测距/测向声呐

这类传感器的特点是：多数直接基于 Unity `Transform`、`Rigidbody`、`Physics.Raycast` 计算，不依赖复杂渲染。

### 4.3 `Sensors/RGB/`

- `CameraSensor.cs`

实现方式：

- 挂在 `Camera` 上
- 关闭相机常驻渲染
- 采样时临时分配 `RenderTexture`
- 调用 `_camera.Render()`
- 用 `AsyncGPUReadback` 把图像数据读回 CPU
- 保存到 `byte[] Data`

对应的 ROS 包装器是：

- `Sensors/ROS/CameraSensorROS.cs`

### 4.4 `Sensors/Lidar/`

- `RaycastLidar.cs`

实现方式：

- 用 `RaycastJobHelper<LidarReading>` 做批量并行 raycast
- 根据分辨率和视场角生成射线
- 命中后生成 `LidarReading`
- 把点云写进 `NativeArray<Vector3> Points`
- 同时可驱动 `PointCloudManager` 做 Unity 内部可视化

它支持三类射线定义：

1. `Uniform`
2. `Angles`
3. `Intervals`

并且有预设配置文件：

- `Packages/com.iwin-fins.fins-sim/Resources/Configs/lidars.json`

里面已经提供了多种雷达模型参数，比如：

- `RS Ruby`
- `Pandar64`
- `Ouster OS1-128`

对应 ROS 输出有两种：

- `RaycastLidarROS.cs`：发自定义 `PointCloud`
- `RaycastLidarPointCloud2ROS.cs`：发标准 `PointCloud2`

### 4.5 `Sensors/Sonar/`

代表文件：

- `Sonar2D.cs`
- `Sonar3D.cs`

其中 `Sonar3D.cs` 比较有代表性，它：

- 同样基于批量 raycast
- 生成 `NativeArray<SonarReading>`
- 把回波转换成极坐标图、笛卡尔图
- 可生成 `Texture2D`
- 支持保存图片、叠加网格、加入噪声
- 支持目标类别/实例标注叠加

ROS 输出：

- `Sonar2DROS.cs`
- `Sonar3DROS.cs`

其中 `Sonar3DROS` 把笛卡尔图编码为 PNG 后通过 `CompressedImageStreamingRequest` 发送。

### 4.6 `Sensors/AIS/`

代表文件：

- `AisDevice.cs`
- `AisSensor.cs`
- `AisManager.cs`

`AisSensor.cs` 会组合：

- `GnssSensor`
- `Rigidbody`
- `AisDevice`

计算：

- 真航向 `TrueHeading`
- 地面航向 `COG`
- 地速 `SOG`

再由 `AISSensorROS.cs` 打包为 AIS 位置报告消息。

### 4.7 `Sensors/ROS/`

这里不是“传感器算法”，而是“每种传感器对应的 ROS 输出适配器”。

基本模式都一样：

1. `GetComponent<某传感器>()`
2. `StreamSensor(sensor, streamingClient.StreamXXX)`
3. `ComposeMessage()` 中把当前采样值转成 protobuf
4. 发往 `address`

这层让算法和通信解耦得比较清楚。

## 5. 多传感器支持是怎么做的

这是这套 core 最值得看的部分之一。

### 5.1 每个传感器实例自动注册

`SensorBase.Awake()` 中：

- 自动调用 `SensorSampler.Instance.AddSensorCallback(this, SampleSensor)`

所以只要你往任意 GameObject 上再挂一个继承自 `SensorBase` 的组件，它就会自动进入系统，不需要中央配置表。

### 5.2 采样按实例独立调度

`SensorSampler` 里每个传感器都有：

- 自己的实例 ID
- 自己的 active 状态
- 自己的 `SampleFrequency`
- 自己上次采样时间

因此多个传感器之间不会互相抢一个全局采样频率。

### 5.3 发送按实例独立建流

每个 `SensorStreamer` 组件：

- 绑定一个具体 sensor
- 绑定一个具体的 gRPC streaming 方法
- 有自己的消息队列
- 有自己的发送线程
- 有自己的 `address`

这就意味着：

- 同一机器人上挂 10 个传感器没问题
- 两个机器人各自挂一套传感器也没问题
- 同类型传感器挂多个实例也可以

### 5.4 地址和 frame 自动按载具命名

`SensorBase.UpdateVehicle()` 会生成：

- `frameId = vehicleName/sensorName_frame`

`SensorStreamer.UpdateVehicle()` / `SetAddresSufix()` 会生成类似：

- `address = vehicleName/sensorName`

这对多机器人、多传感器场景非常关键，因为可以天然避免 topic/address 冲突。

### 5.5 TF 自动补齐

`SensorStreamer.Reset()` 中，如果对象上没有 `TfStreamerROS`，会自动加一个。

`TfStreamerROS` 会根据：

- 当前对象是不是载具本体
- 是否有 `ParentTransform`

来决定发布：

- 载具本体到 `map`
- 传感器到 `base_link`

这样 ROS 侧能拿到每个传感器自己的坐标系。

### 5.6 具体例子

假设一个 AUV 下面挂了：

- `ImuSensor + ImuROS`
- `DepthSensor + DepthSensorROS`
- `RaycastLidar + RaycastLidarROS`
- `CameraSensor + CameraSensorROS`

系统会自动形成四条独立链路：

1. 四个 `SensorBase` 子类都注册进 `SensorSampler`
2. `SensorSampler` 按各自频率采样
3. 四个 `SensorStreamer` 子类各自封装 protobuf
4. 四条 gRPC stream 独立写出
5. 四个 `frameId/address` 自动区分

这就是它支持多传感器的核心方式。

## 6. 执行器与控制部分做了什么

### 6.1 推进器

文件：

- `Actuators/Thruster.cs`
- `Actuators/ThrusterController.cs`
- `Scripts/ThrusterAsset.cs`
- `Datasheets/*.asset`

工作方式：

- `ThrusterAsset` 保存推力曲线
- `Thruster.ApplyInput()` 把归一化输入映射成推力
- 在 `FixedUpdate()` 中用 `Rigidbody.AddForceAtPosition()` 施加力
- `ThrusterController` 负责把多路输入分发到多个推进器

这说明推进器不是直接改速度，而是按力学方式加力。

### 6.2 ROS 控制

文件：

- `Controllers/AUVRosController.cs`

它通过 `RemoteControlClient.ApplyForce(...)` 订阅远程控制流，然后把收到的 PWM 数组交给 `ThrusterController.ApplyInput()`。

因此 MARUS 的控制闭环大致是：

`ROS 控制命令 -> gRPC server stream -> AUVRosController -> ThrusterController -> Thruster -> Rigidbody`

### 6.3 本地控制与多载具切换

文件：

- `Controllers/AUVPrimitiveController.cs`
- `Controllers/ASVPrimitiveController.cs`
- `Controllers/CameraController.cs`
- `Controllers/FPVController.cs`
- `Controllers/AgentManager.cs`

`AgentManager` 是一个简单但很实用的多载具管理器：

- 所有控制对象注册进去
- 按 `C` 键切换当前激活 agent
- 只有当前 `activeAgent` 响应键盘输入

这说明它对“场景里同时存在多个机器人/相机”是有明确支持的。

## 7. 通信介质部分做了什么

### 7.1 声学通信

文件：

- `Communications/Acoustic/AcousticMedium.cs`
- `Communications/Acoustic/AcousticDevice.cs`
- `Communications/Acoustic/Nanomodem.cs`
- `Sensors/ROS/NanomodemROS.cs`

设计思路：

- `AcousticMedium` 负责传播
  - 广播
  - 点对点传输
  - 根据距离和声速 `C` 加传播延迟
- `AcousticDevice` 定义设备抽象
- `Nanomodem` 实现具体声学设备
- `NanomodemROS` 负责把 ROS 请求翻译为 modem 指令并回传响应

这里的传播模型不是复杂水声信道，而是工程上可用的简化模型：

- 距离判断
- 固定声速传播延迟
- 协议与消息打包

### 7.2 RF / LoRa

文件：

- `Communications/RF/RfDevice.cs`
- `Communications/RF/LoraDevice.cs`
- `Communications/RF/LoraRanging.cs`

`LoraRanging` 本身也是一个 `SensorBase`，会：

- 以 `Master` 和 `Targets` 为基础生成测距结果
- 可选引入多个 advanced node
- 支持 `PacketDropRate`

这说明在 MARUS 里，“通信设备”和“传感器”是可以交叉组合的。

## 8. 海洋/水环境相关部分做了什么

### 8.1 水面采样

文件：

- `Ocean/WaterHeightSampler.cs`

这个类会在定义了 `CREST_OCEAN` 时对接 Crest 海洋系统，提供：

- 单点水面高度查询
- 批量点水面高度查询

如果没有 Crest，则默认返回 0。

也就是说，MARUS 的水面高度并不是自己完整求解，而是**优先复用 Crest 这类外部海洋系统**。

### 8.2 浮力与船体水动力

文件：

- `BoatPhysics/BoatPhysics.cs`
- `BoatPhysics/Buoyancy.cs`
- 以及 `BoatPhysicsMath.cs`、`ModifyBoatMesh.cs` 等

这部分主要面向船体/漂浮体：

- 根据网格三角面切分水上/水下区域
- 计算浮力
- 计算黏性阻力、压差阻力、拍击力
- 通过 `AddForceAtPosition()` 施加到刚体

所以它更偏“载具在水中的受力”，不是 Navier-Stokes 级别的流体模拟。

### 8.3 水下视觉效果

文件：

- `PostProcess/UnderwaterPP.cs`
- `PostProcess/FogEffect.cs`

`UnderwaterPP` 会：

- 查询当前相机位置的水面高度
- 判断是否“在水下”
- 使用 HDRP Custom Post Process 做噪声、像素扰动等视觉效果

这部分主要是视觉表现层。

## 9. 坐标系与时间

### 9.1 坐标系转换

文件：

- `TfExtensions.cs`

这里定义了几种关键转换：

- `Unity2Map()`：Unity 世界坐标 -> ROS ENU/map
- `Unity2Body()`：Unity 局部坐标 -> ROS FLU/body

很多 ROS 发送器都依赖这些转换，所以这部分是 Unity/ROS 对接正确性的基础。

### 9.2 时间

文件：

- `Ros/TimeHandler.cs`

`TimeHandler` 维护：

- 仿真起始时间
- 当前累计仿真时间
- 实时/非实时仿真速度

ROS 消息时间戳大量来自这里。

## 10. 可视化、标注、日志、测试

### 10.1 可视化

`Visualization/` 下主要有：

- 点云管理与渲染
- 路径记录
- ROS marker 可视化

尤其 `PointCloudManager` 被 Lidar/Sonar 复用得比较多。

### 10.2 标注

`ObjectAnnotation/` 提供：

- 相机目标检测保存
- 点云分割保存
- 声呐目标检测保存

这说明 `marus-core` 也兼顾了仿真数据集生成用途。

### 10.3 日志

`DataLogger/` 和许多 `Log(...)` 调用一起构成了内部记录能力。

传感器、通信设备、推进器都会记录关键数据。

### 10.4 测试

`Packages/com.iwin-fins.fins-sim/Tests/` 下已有：

- EditMode：
  - `ImuTest`
  - `DepthTest`
  - `GnssTest`
  - `DvlTest`
  - `PoseTest`
  - `RangeTest`
  - `AISTest`
  - `NoiseTest`
  - `ControllerTest`
- PlayMode：
  - `LidarTest`
  - `AcousticTest`
  - `RfTest`
  - `RosConnectionTest`

说明作者对核心功能至少做了基础自动化覆盖。

## 11. 如果把它当成一个系统，核心分层可以这样理解

可以把 `marus-core` 理解成下面这几层：

1. 环境与物理层
   - 水面
   - 浮力
   - 船体水动力
   - 刚体/碰撞
2. 传感器层
   - 从 Unity 物理世界采样
   - 产生本地数据结构
3. 通信适配层
   - 把采样值打包成 protobuf
   - 通过 gRPC 发往 ROS
4. 控制执行层
   - 从 ROS 接收指令
   - 转成推进器/刚体受力
5. 配套层
   - TF
   - 时间
   - 可视化
   - 日志
   - 标注

## 12. 一个比较准确的总结

如果只用一句话概括：

**`Packages/com.iwin-fins.fins-sim` 是 MARUS 的海洋机器人仿真核心库，它把“载具物理 + 传感器仿真 + ROS/gRPC 桥接 + 通信介质 + 可视化/标注”整合到了一套可复用的 Unity 组件体系里。**

而“支持多个传感器”的关键，不靠某个单独的 manager 配表，而是靠下面这套组合：

- 所有传感器继承 `SensorBase`
- `SensorSampler` 按实例统一调度
- 每个 ROS 适配器继承 `SensorStreamer`
- 每个传感器实例有独立 `frameId/address/queue/stream`
- `RosConnection` 统一提供共享 gRPC 连接
- `TfStreamerROS` 自动补足传感器坐标系

所以这套架构非常典型地体现了 Unity 组件化思路：**“一个传感器 = 一个采样组件 + 一个 ROS 输出组件 + 可选 TF 组件 + 可选可视化/标注组件”**。

## 13. 额外说明

当前目录名实际是 `Packages/com.iwin-fins.fins-sim/`，不是 `Assets/manrus-core/`。

另外，从代码看，MARUS 的“水模拟”更偏：

- 水面高度/水下视觉表现
- 浮力和船体受力
- 水下传感器与通信行为

而不是统一的高保真流体求解器。这一点在理解项目定位时很重要。
