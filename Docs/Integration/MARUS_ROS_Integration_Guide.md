# MARUS ROS Integration Guide

本文基于当前仓库里的 `marus-core` 代码整理，目标是回答 3 个问题：

1. MARUS 里的 ROS 系统到底是怎么接进去的
2. 在 Unity 里应该怎么挂载这些组件
3. 对应的 Python 侧应该怎么写

## 1. 先说结论

MARUS 这里的“ROS 系统”本质上不是 Unity 直接发 ROS topic，而是：

- Unity 作为 gRPC client
- 外部 Python/ROS 进程作为 gRPC server
- 双方通过 protobuf message 交换传感器、TF、控制命令、参数和可视化数据

也就是说，外部“调用方”如果想接 MARUS，通常不是单纯写一个 ROS subscriber，而是要先提供一组 gRPC service。

当前项目的入口在：

- `Packages/com.iwin-fins.fins-sim/Runtime/Platform/Ros/RosConnection.cs`
- `Packages/com.iwin-fins.fins-sim/Runtime/Platform/Sensors/Core/SensorStreamer.cs`
- `Packages/com.iwin-fins.fins-sim/Runtime/Platform/Ros/ServerStreamer.cs`

另外，仓库 README 已经明确写了适配器方向：

- `README.md`
- `Docs/ThirdParty/MARUS-README.md`

它们都提到 MARUS 的 ROS 适配器是单独的 `grpc_ros_adapter` 包。

## 2. Unity 侧通信方向

把代码串起来后，MARUS 的 gRPC 方向可以总结成下面这张表。

| 功能 | Unity 侧角色 | gRPC 方式 | Python 侧要做什么 |
| --- | --- | --- | --- |
| 连通性检测 | client | `Ping.Ping` unary | 必须实现，返回 `value=1` |
| 传感器上报 | client | `SensorStreaming.*` client streaming | 实现接收流并转发到 ROS/topic/日志 |
| 推力控制 | client | `RemoteControl.ApplyForce` server streaming | 持续向 Unity 推送 PWM |
| TF 发布到外部 | client | `Tf.PublishFrame` client streaming | 接收 Unity 发出的 TF |
| TF 从外部回流到 Unity | client | `Tf.StreamAllFrames` server streaming | 向 Unity 持续回推 TF 树 |
| 参数服务器 | client | `ParameterServer.Get/Set` unary | 提供经纬度等参数 |
| Marker 可视化 | client | `Visualization.SetMarker/SetMarkerArray` server streaming | 向 Unity 推送 marker |
| 点云可视化回流 | client | `SensorStreaming.RequestPointCloud2` server streaming | 向 Unity 推送 `PointCloud2` |
| 非实时仿真步进 | client | `SimulationControl.SetStartTime/Step` unary | 仅在 `RealtimeSimulation=false` 时需要 |

这里最容易误解的一点是：

- 传感器数据不是 Python 主动来“拉”
- 而是 Unity 主动开 client-stream，把数据“推”到 Python server

控制命令则相反：

- Unity 会订阅 `ApplyForce`
- Python server 要持续把 PWM 流“推”回来

## 3. Unity 里要怎么挂

### 3.1 场景里必须先有一个 `RosConnection`

`RosConnection` 是全局单例入口，负责：

- 建立 gRPC `Channel`
- 初始化 `SensorStreamingClient`、`RemoteControlClient`、`TfClient`、`VisualizationClient` 等 client
- 自动创建 `TfHandler`、`ParamServerHandler`、`TimeHandler`、`VisualizationROS`、`PointCloudRosVisualizer`

默认配置在代码里是：

- `serverIP = "localhost"`
- `serverPort = 30052`

示例场景里已经有这个对象：

- `Assets/FinsSimUnity/Tasks/PlatformExamples/Scenes/ExampleScene.unity`
- `Assets/FinsSimUnity/Tasks/PlatformExamples/Scenes/ExampleSceneCrest.unity`
- `Assets/FinsSimUnity/Tasks/PlatformExamples/Scenes/TestUUV.unity`

### 3.2 Vehicle 根节点一定要有 `Vehicle` tag

很多自动命名都依赖 `Helpers.GetVehicle()`，它会沿父节点往上找第一个 `tag == "Vehicle"` 的对象。

这直接影响：

- 传感器 `frameId`
- 传感器默认 `address`
- 控制订阅地址
- TF 默认父子关系

如果这个 tag 没有设对，地址和 frame 前缀就会乱掉。

相关代码：

- `Packages/com.iwin-fins.fins-sim/Runtime/Platform/Utils/Utils.cs`

### 3.3 传感器挂载规则

每类 ROS 传感器基本都遵循同一个模式：

1. 在某个物体上挂“传感器本体”
2. 在同一个物体上再挂一个对应的 `*ROS` 包装器

常见例子：

| 传感器本体 | ROS 包装器 | gRPC 方法 | 默认地址 |
| --- | --- | --- | --- |
| `ImuSensor` | `ImuROS` | `StreamImuSensor` | `VehicleName/SensorObjectName` |
| `DepthSensor` | `DepthSensorROS` | `StreamDepthSensor` | `VehicleName/SensorObjectName` |
| `GnssSensor` | `GnssROS` | `StreamGnssSensor` | `VehicleName/SensorObjectName` |
| `PoseSensor` | `PoseSensorROS` | `StreamPoseSensor` | `VehicleName/SensorObjectName` |
| `CameraSensor` | `CameraSensorROS` | `StreamCameraSensor` | `VehicleName/SensorObjectName` |
| `RaycastLidar` | `RaycastLidarPointCloud2ROS` | `StreamPointCloud2` | `VehicleName/SensorObjectName` |

代码层的关键机制是：

- `SensorBase` 负责 `frameId` 和采样注册
- `SensorStreamer<TClient, TMsg>` 负责把采样结果包装成 protobuf 并发出去

另外一个很实用的细节是：

- `SensorStreamer.Reset()` 会自动给同物体补一个 `TfStreamerROS`

所以在编辑器里新加 ROS wrapper 时，通常会自动把 TF 发布组件也补上。

### 3.4 控制挂载规则

如果要让外部 Python 控制推进器，Unity 侧要准备这条链路：

1. Vehicle 根节点上有 `ThrusterController`
2. `ThrusterController.thrusters` 列表里排好推进器顺序
3. 同一个 Vehicle 上挂 `AUVRosController`
4. 把 `AUVRosController.thrusterController` 指到上面的 `ThrusterController`

`AUVRosController` 会自动订阅：

- `VehicleName/pwm_out`

然后把收到的 `ForceResponse.pwm.data` 直接按顺序喂给：

- `ThrusterController.ApplyInput(float[] array)`

这里有两个实际使用上的坑：

1. PWM 数组顺序就是 `thrusters` 列表顺序
2. 最好每次发完整长度；长度不够时，后面的推进器不会被本次消息更新

相关代码：

- `Packages/com.iwin-fins.fins-sim/Runtime/Platform/Controllers/AUVRosController.cs`
- `Packages/com.iwin-fins.fins-sim/Runtime/Platform/Actuators/ThrusterController.cs`

### 3.5 TF 的默认命名规则

`TfStreamerROS` 的默认规则非常固定：

- 如果挂在 Vehicle 根节点：
  - `FrameId = VehicleName/base_link`
  - `ParentFrameId = map`
- 如果挂在 Vehicle 的子传感器节点：
  - `FrameId = VehicleName/SensorName_frame`
  - `ParentFrameId = VehicleName/base_link`

因此一个典型层级：

```text
BlueROV2 (tag=Vehicle)
|- imu_link
|- depth_link
|- front_camera
```

通常会得到：

- TF:
  - `map -> BlueROV2/base_link`
  - `BlueROV2/base_link -> BlueROV2/imu_link_frame`
  - `BlueROV2/base_link -> BlueROV2/depth_link_frame`
  - `BlueROV2/base_link -> BlueROV2/front_camera_frame`
- 传感器地址:
  - `BlueROV2/imu_link`
  - `BlueROV2/depth_link`
  - `BlueROV2/front_camera`
- 控制地址:
  - `BlueROV2/pwm_out`

## 4. 一个最小可用的挂载示例

假设你有一台叫 `BlueROV2` 的载具，建议这样挂：

1. `BlueROV2`
   - tag 设为 `Vehicle`
   - 挂 `Rigidbody`
   - 挂 `ThrusterController`
   - 挂 `AUVRosController`
2. `BlueROV2/imu_link`
   - 挂 `ImuSensor`
   - 挂 `ImuROS`
3. `BlueROV2/depth_link`
   - 挂 `DepthSensor`
   - 挂 `DepthSensorROS`
4. `BlueROV2/front_camera`
   - 挂 `CameraSensor`
   - 挂 `CameraSensorROS`
5. 场景中单独放一个 `RosConnection`
   - `serverIP` 指向 Python 进程所在机器
   - `serverPort` 和 Python server 保持一致

如果只是测试最小链路，这样就已经够了。

## 5. Python 端真正要准备什么

### 5.1 先准备 protobuf / gRPC Python 代码

当前仓库里没有 `.proto` 原文件，只保留了 C# 生成结果：

- `Packages/com.iwin-fins.fins-sim/Runtime/Platform/protobuf/*.cs`

从这些生成文件头部可以确认 proto 名称至少包括：

- `std.proto`
- `geometry.proto`
- `sensor.proto`
- `sensor_streaming.proto`
- `remote_control.proto`
- `tf.proto`
- `parameter_server.proto`
- `visualization.proto`
- `simulation_control.proto`
- `commander_service.proto`

所以 Python 侧要么：

- 直接使用上游 `grpc_ros_adapter` 已生成好的 Python 模块

要么：

- 从同一套 `.proto` 重新生成 `*_pb2.py` 和 `*_pb2_grpc.py`

生成命令通常类似：

```bash
python -m grpc_tools.protoc \
  -I <proto_root> \
  --python_out=<out_dir> \
  --grpc_python_out=<out_dir> \
  <proto_root>/std.proto \
  <proto_root>/geometry.proto \
  <proto_root>/sensor.proto \
  <proto_root>/sensor_streaming.proto \
  <proto_root>/remote_control.proto \
  <proto_root>/tf.proto \
  <proto_root>/parameter_server.proto \
  <proto_root>/visualization.proto \
  <proto_root>/simulation_control.proto
```

### 5.2 一个最小可跑的 Python server 骨架

下面这个例子重点演示 4 件事：

- 让 `RosConnection` 能连上
- 接收 Unity 推来的传感器流
- 给 Unity 推 PWM 控制流
- 提供 TF / 参数 / 可视化这些默认会被订阅的服务

```python
import time
import threading
from queue import Queue, Empty
from concurrent import futures

import grpc

import std_pb2
import ping_pb2
import ping_pb2_grpc
import sensor_streaming_pb2
import sensor_streaming_pb2_grpc
import remote_control_pb2
import remote_control_pb2_grpc
import tf_pb2
import tf_pb2_grpc
import parameter_server_pb2
import parameter_server_pb2_grpc
import visualization_pb2
import visualization_pb2_grpc


class PingServicer(ping_pb2_grpc.PingServicer):
    def Ping(self, request, context):
        return ping_pb2.PingMsg(value=1)


class ParameterServerServicer(parameter_server_pb2_grpc.ParameterServerServicer):
    def __init__(self):
        self.params = {
            "/d2/LocalOriginLat": 45.0,
            "/d2/LocalOriginLon": 15.0,
        }

    def GetParameter(self, request, context):
        value = self.params.get(request.name)
        if value is None:
            return parameter_server_pb2.ParamValue()
        if isinstance(value, bool):
            return parameter_server_pb2.ParamValue(valueBool=value)
        if isinstance(value, int):
            return parameter_server_pb2.ParamValue(valueInt=value)
        if isinstance(value, float):
            return parameter_server_pb2.ParamValue(valueDouble=value)
        return parameter_server_pb2.ParamValue(valueStr=str(value))

    def SetParameter(self, request, context):
        v = request.value
        which = v.WhichOneof("parameterValue")
        if which == "valueBool":
            self.params[request.name] = v.valueBool
        elif which == "valueInt":
            self.params[request.name] = v.valueInt
        elif which == "valueDouble":
            self.params[request.name] = v.valueDouble
        elif which == "valueStr":
            self.params[request.name] = v.valueStr
        return std_pb2.Empty()


class TfServicer(tf_pb2_grpc.TfServicer):
    def __init__(self):
        self.frames = {}

    def GetAllFrames(self, request, context):
        return tf_pb2.TfFrameList(frames=list(self.frames.values()))

    def StreamAllFrames(self, request, context):
        while context.is_active():
            yield tf_pb2.TfFrameList(frames=list(self.frames.values()))
            time.sleep(0.1)

    def PublishFrame(self, request_iterator, context):
        for req in request_iterator:
            self.frames[req.childFrameId] = req
            print(f"[tf] {req.frameId} -> {req.childFrameId}")
        return std_pb2.Empty()


class VisualizationServicer(visualization_pb2_grpc.VisualizationServicer):
    def __init__(self):
        self.marker_queues = {}
        self.marker_array_queues = {}

    def SetMarker(self, request, context):
        q = self.marker_queues.setdefault(request.address, Queue())
        while context.is_active():
            try:
                yield q.get(timeout=1.0)
            except Empty:
                continue

    def SetMarkerArray(self, request, context):
        q = self.marker_array_queues.setdefault(request.address, Queue())
        while context.is_active():
            try:
                yield q.get(timeout=1.0)
            except Empty:
                continue


class SensorStreamingServicer(sensor_streaming_pb2_grpc.SensorStreamingServicer):
    def __init__(self):
        self.pointcloud2_queues = {}

    def StreamImuSensor(self, request_iterator, context):
        for req in request_iterator:
            print(f"[imu] address={req.address} frame={req.data.header.frameId}")
        return std_pb2.Empty()

    def StreamDepthSensor(self, request_iterator, context):
        for req in request_iterator:
            z = req.data.pose.pose.position.z
            print(f"[depth] address={req.address} depth={z:.3f}")
        return std_pb2.Empty()

    def StreamGnssSensor(self, request_iterator, context):
        for req in request_iterator:
            print(
                f"[gnss] address={req.address} "
                f"lat={req.data.latitude:.7f} lon={req.data.longitude:.7f}"
            )
        return std_pb2.Empty()

    def StreamPoseSensor(self, request_iterator, context):
        for req in request_iterator:
            p = req.data.pose.pose.position
            print(f"[pose] address={req.address} pos=({p.x:.2f}, {p.y:.2f}, {p.z:.2f})")
        return std_pb2.Empty()

    def StreamCameraSensor(self, request_iterator, context):
        for req in request_iterator:
            if req.HasField("image"):
                print(
                    f"[camera] address={req.address} "
                    f"size={req.image.width}x{req.image.height} bytes={len(req.image.data)}"
                )
        return std_pb2.Empty()

    def StreamPointCloud2(self, request_iterator, context):
        for req in request_iterator:
            print(f"[pc2] address={req.address} points={req.data.width * req.data.height}")
        return std_pb2.Empty()

    def RequestPointCloud2(self, request, context):
        q = self.pointcloud2_queues.setdefault(request.address, Queue())
        while context.is_active():
            try:
                yield q.get(timeout=1.0)
            except Empty:
                continue


class RemoteControlServicer(remote_control_pb2_grpc.RemoteControlServicer):
    def __init__(self):
        self.targets = {
            "BlueROV2/pwm_out": [0.0, 0.0, 0.0, 0.0],
        }
        self.lock = threading.Lock()

    def set_pwm(self, address, values):
        with self.lock:
            self.targets[address] = list(values)

    def ApplyForce(self, request, context):
        print(f"[control] subscriber={request.address}")
        while context.is_active():
            with self.lock:
                pwm = self.targets.get(request.address, [0.0, 0.0, 0.0, 0.0])
            yield remote_control_pb2.ForceResponse(
                success=True,
                pwm=std_pb2.Float32Array(data=pwm),
            )
            time.sleep(0.05)


def serve():
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=16))

    sensor_servicer = SensorStreamingServicer()
    remote_servicer = RemoteControlServicer()

    ping_pb2_grpc.add_PingServicer_to_server(PingServicer(), server)
    parameter_server_pb2_grpc.add_ParameterServerServicer_to_server(
        ParameterServerServicer(), server
    )
    tf_pb2_grpc.add_TfServicer_to_server(TfServicer(), server)
    visualization_pb2_grpc.add_VisualizationServicer_to_server(
        VisualizationServicer(), server
    )
    sensor_streaming_pb2_grpc.add_SensorStreamingServicer_to_server(
        sensor_servicer, server
    )
    remote_control_pb2_grpc.add_RemoteControlServicer_to_server(
        remote_servicer, server
    )

    server.add_insecure_port("[::]:30052")
    server.start()
    print("gRPC server listening on 0.0.0.0:30052")
    server.wait_for_termination()


if __name__ == "__main__":
    serve()
```

## 6. 这个 Python 例子怎么和 Unity 对上

如果 Unity 里对象名就是：

- Vehicle: `BlueROV2`
- IMU 物体: `imu_link`
- Camera 物体: `front_camera`

那么上面的 Python server 会收到类似这些地址：

- `BlueROV2/imu_link`
- `BlueROV2/front_camera`
- `BlueROV2/pwm_out`

你真正做 ROS 集成时，通常就是在这些回调里：

1. 把传感器请求里的 `req.data` 转成 ROS message
2. 按 `req.address` 决定发到哪个 ROS topic
3. 把控制器输出写回 `RemoteControlServicer.set_pwm()`

例如：

- `BlueROV2/imu_link -> /bluerov2/imu/data`
- `BlueROV2/front_camera -> /bluerov2/camera/image_raw`
- `BlueROV2/pwm_out <- 控制器输出`

## 7. 为什么最小 server 也最好把 TF / Visualization / PointCloud2 stub 补上

这是这个项目里一个很容易踩坑的地方。

`RosConnection` 连接成功后会自动创建并启用：

- `TfHandler`
- `VisualizationROS`
- `PointCloudRosVisualizer`

它们默认会主动去订阅这些 server-streaming 接口：

- `Tf.StreamAllFrames`
- `Visualization.SetMarker`
- `Visualization.SetMarkerArray`
- `SensorStreaming.RequestPointCloud2`

所以如果 Python server 完全不实现这些接口，Unity 侧虽然可能先连上 `Ping`，但后续很容易在后台流上报错。

如果你暂时不用这些能力，建议至少提供一个“保持连接但暂时不发消息”的 stub 实现，就像上面的 `Queue + timeout` 写法。

## 8. 常见注意事项

1. `RosConnection` 只负责连到一个 gRPC 地址，不会自动发现多个 ROS 节点。
2. `SensorStreamer.UpdateFrequency` 不能高于底层 `SampleFrequency`，代码里会自动压回较低值。
3. 改了 Vehicle 名、传感器物体名、层级关系后，`address` 和 `frameId` 最好重新检查一遍。
4. `frameId` 和姿态坐标不是原始 Unity 坐标，wrapper 内部会按 `map/body` 约定做转换。
5. 如果 `RealtimeSimulation=false`，还要补 `SimulationControl.SetStartTime` 和 `SimulationControl.Step`。
6. 当前仓库里只有 C# 生成代码，没有 `.proto` 原文件；Python 侧不要直接照抄 C#，而是要用同一套 proto 生成 Python 模块。

## 9. 最短落地步骤

1. 在 Python 侧先跑起 gRPC server，监听 `30052`
2. 至少实现 `Ping`
3. 强烈建议同时补好这些默认会被 Unity 侧主动订阅的 stub：
   - `Tf.StreamAllFrames`
   - `Visualization.SetMarker`
   - `Visualization.SetMarkerArray`
   - `SensorStreaming.RequestPointCloud2`
4. 如果要接收传感器，再实现对应的 `SensorStreaming.Stream*`
5. 如果要远程控制推进器，再实现 `RemoteControl.ApplyForce`
6. 如果要让 Unity 读地理原点参数，再实现 `ParameterServer.Get/Set`
7. 在 Unity 场景里确认有 `RosConnection`
8. 给载具根节点加 `Vehicle` tag
9. 给每个传感器挂“本体 + 对应 ROS wrapper”
10. 给载具根节点挂 `AUVRosController` 并绑定 `ThrusterController`
11. 进入 Play，先确认 Python 端打印出 `Ping`/传感器地址，再开始接 ROS 控制器

按当前代码结构，MARUS 的 ROS 集成核心思路可以浓缩成一句话：

- Unity 侧负责仿真和出流，Python 侧负责实现 gRPC server，并把这些流转成 ROS 世界里的 topic、tf、参数和控制。
