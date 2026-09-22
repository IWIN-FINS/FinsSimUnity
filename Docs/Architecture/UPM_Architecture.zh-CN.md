# FinsSimUnity：工程 + 仓内 UPM 包

`Packages/com.iwin-fins.fins-sim` 是唯一的 FinsSim 平台包。它承载可跨任务复用的运行时代码：

- `Runtime/Core`：不依赖具体场景的空间/数学基础类型；
- `Runtime/Platform`：传感器、推进器、ROS/gRPC、通信、环境、可视化，以及 `Platform/Hydrodynamics`；
- `Runtime/Diagnostics`：运行时采样与诊断；
- `Editor`、`Tests`、`Resources`、`Datasheets`：平台包的编辑器功能、测试和平台数据。

宿主工程只保留应用选择与任务内容：

- `Assets/FinsSimUnity/Tasks/<task-family>`：每一类 RL/IRL、追逐、网捕、布料或平台示例的场景、运行时逻辑与编辑器工具；
- `Assets/FinsSimUnity/Editor/Automation`：外部 Play-mode 控制和场景准备；
- `Assets/FinsSimUnity/Editor/Build`：唯一的 task catalog、构建 daemon 与兼容的任务构建实现；
- `Assets/FinsSimUnity/Editor/Content`：项目 HDRP/内容维护工具；
- `Assets/FinsSimUnity/Content`：项目级 HDRP、环境和纹理资产；
- `Assets/ThirdParty/UniMeshCombiner/Editor`：供应商网格合并工具；
- `Assets/Obi`：保留其供应商要求的物理路径；
- `Packages/com.nwh.*`：DWP2 的第三方嵌入包，不并入 FinsSim 平台源码。

`Grpc.Core`、`Grpc.Core.Api`、`Google.Protobuf` 与 `MathNet.Numerics` 的受控运行时副本由
`com.iwin-fins.fins-sim/Runtime/Plugins` 提供。ML-Agents 保留其 `Google.Protobuf_Packed`
实现；其历史 `Grpc.Core` 编辑器 importer 已关闭，因此 ML-Agents communicator 与 FinsSim
ROS bridge 在 Editor 和 Linux Player 中共同解析 FinsSim 的 gRPC v2，而不是由 Unity 按扫描顺序选择。
旧 `System.Runtime.CompilerServices.Unsafe v4` 保留在包内仅作来源记录，但 importer 已关闭；Unity
Collections 提供的 v6 是工程唯一可加载版本。

任务代码目前保持历史 MonoBehaviour 的脚本 GUID 和序列化类型身份；以后如需把任务类进一步改为 `FinsSim.Tasks.*`，必须采用 Unity 的类型迁移属性并逐场景 resave，不能仅做文本命名空间替换。

## Editor 工具边界

`Editor` 并非只能位于 `Assets/Editor`。在带有 Runtime asmdef 的 `Tasks` 树中，目录名本身不足以建立程序集边界：每个任务的 `Editor/` 目录均有独立的 `FinsSim.Tasks.<Task>.Editor` asmdef，且只包含 `Editor` 平台。这样场景构造、审计与 PropertyDrawer 不会被编进 `FinsSim.Tasks` 的 Linux Player。

项目级工具不属于某一个 task，因此放在 `Assets/FinsSimUnity/Editor`；任务专属工具与它操作的 scene/runtime 相邻。所有现有类名保持不变，故既有 `-executeMethod FinsSimTaskBuild.*` 调用和序列化脚本引用不因目录调整而改变。

## 构建

构建只使用显式 task id，不接受 `Type.Method` 反射调用：

```bash
./tools/unity/build_in_worktree.sh --task pose-control/new-server
./tools/unity/build_in_worktree.sh --task chase/three-headless
```

目录与场景的映射在 `Assets/FinsSimUnity/Editor/Build/FinsSimTaskBuild.cs`。生成物位于本仓库的 `artifacts/unity_builds/<task-id>/`；Python 保持传入该 executable 的直接 `env_path`。
