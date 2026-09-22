# MARUS 自动数据采集与标注系统说明

本文基于当前仓库 `Packages/com.iwin-fins.fins-sim/Runtime/Platform/` 的实现整理，重点回答：

1. MARUS 里有哪些“自动数据采集/标注”能力
2. 在 Unity 里分别应该怎么挂载和使用
3. 最终会输出什么文件
4. 当前实现有哪些限制和注意事项

## 1. 系统总览

MARUS 里和“自动采集/标注”直接相关的能力，实际上分成两类：

### 1.1 数据集导出类

位于 `Packages/com.iwin-fins.fins-sim/Runtime/Platform/ObjectAnnotation/`：

- `CameraObjectDetectionSaver`
  - 相机目标检测数据集
  - 输出图片和 YOLO 风格 bbox 标签
- `SonarObjectDetectionSaver`
  - 3D 声呐图像目标检测数据集
  - 输出声呐图像和 YOLO 风格 bbox 标签
- `PointCloudSegmentationSaver`
  - 点云语义/实例分割数据集
  - 输出 `.pcd` 点云、`.label` 标签和位姿

### 1.2 运行时日志类

位于 `Packages/com.iwin-fins.fins-sim/Runtime/Platform/DataLogger/`：

- `DataLogger`
  - 在运行时缓存各类传感器/推进器/通信数据
- `DataLoggerUtilities`
  - 把缓存导出成 JSON

这两类不要混淆：

- `ObjectAnnotation/*Saver` 面向“训练数据集”
- `DataLogger` 面向“仿真运行记录/调试记录”

## 2. 当前代码里的入口

这次梳理里最核心的文件是：

- `Packages/com.iwin-fins.fins-sim/Runtime/Platform/ObjectAnnotation/CameraObjectDetectionSaver.cs`
- `Packages/com.iwin-fins.fins-sim/Runtime/Platform/ObjectAnnotation/SonarObjectDetectionSaver.cs`
- `Packages/com.iwin-fins.fins-sim/Runtime/Platform/ObjectAnnotation/PointCloudSegmentationSaver.cs`
- `Packages/com.iwin-fins.fins-sim/Runtime/Platform/Sensors/Sonar/Sonar3D.cs`
- `Packages/com.iwin-fins.fins-sim/Runtime/Platform/Sensors/Lidar/RaycastLidar.cs`
- `Packages/com.iwin-fins.fins-sim/Runtime/Platform/DataLogger/DataLogger.cs`
- `Packages/com.iwin-fins.fins-sim/Runtime/Platform/DataLogger/DataLoggerUtilities.cs`

另外我查了当前仓库的场景和 prefab 引用，没有找到这 3 个 saver 的现成挂载示例，所以这套系统目前更像“代码已提供、需要手动组装”，而不是“示例场景已经配好”。

## 3. 相机目标检测数据集

### 3.1 它做什么

`CameraObjectDetectionSaver` 会定期：

1. 遍历你配置好的目标物体
2. 计算这些物体在某个相机视角中的 2D bounding box
3. 抓取该相机当前画面
4. 保存成图片
5. 生成一份 YOLO 风格标签文件

输出格式是：

- 图片：`.jpg`
- 标签：`.txt`
- 标签内容：`class x_center y_center width height`
- 坐标全部是归一化坐标

### 3.2 怎么挂

这个组件和声呐/激光雷达不一样，它不要求挂在某个传感器对象上。

推荐做法是：

1. 新建一个空物体，例如 `DatasetManager`
2. 挂 `CameraObjectDetectionSaver`
3. 在 `ObjectClasses` 中配置类别和目标对象
4. 在 `CameraViews` 中配置需要采集的 `Camera`

它依赖的是：

- 目标对象有 `MeshFilter`
- 目标对象可被相机看到
- 如果使用默认的 `RaycastCheck=true`，目标对象还要有对应 collider

### 3.3 Inspector 里关键参数

最重要的是这些：

- `Enable`
  - 默认 `true`
  - 运行时总开关
- `ResumeFromLastTime`
  - `true` 时继续往上一次 `runX` 目录里追加
  - `false` 时新建下一个 `run`
- `ObjectClasses`
  - 每个元素是一类目标
  - `ClassName` 是类别名
  - `ObjectsInClass` 是该类所有实例
- `CameraViews`
  - 要采集的相机列表
- `SaveFrequencyHz`
  - 保存频率
- `MinimumObjectArea`
  - bbox 像素面积阈值，小于这个值不保存
- `MinVerticalPosition`
  - 用于忽略低于某一高度的顶点，常见用途是忽略水下部分
- `VertexStep`
  - bbox 计算时的顶点采样步长
  - 越大越快，越不精确
- `DatasetFolder`
  - 数据集保存根目录
  - 为空时默认写到 `Assets/camera_detection`
- `GenerateTestSubset`
  - 是否生成 `test`
- `BackgroundImageSaveProbability`
  - 即使没有目标，也以一定概率保存背景图
- `TrainSize` / `ValSize` / `TestSize`
  - train/val/test 划分比例

### 3.4 输出目录结构

目录结构是：

```text
<DatasetFolder>/
  run0/
    images/
      train/
      val/
      test/
    labels/
      train/
      val/
      test/
    simulation.yaml
```

`simulation.yaml` 里会写：

- train/val/test 图片目录
- `nc`
- `names`

因此它本质上是在生成一套接近 YOLO 训练目录的结构。

### 3.5 类别编号规则

相机 saver 的类别编号从 `0` 开始。

另外还有一个细节：

- 如果你在 `ObjectClasses` 里重复填写了相同的 `ClassName`
- 代码会只保留第一次出现的类别名

所以更稳妥的做法是：

- 一个类别名只建一次
- 该类的所有实例都放进同一个 `ObjectsInClass`

### 3.6 使用步骤

最短流程：

1. 给所有待标注目标准备 `MeshFilter`
2. 给这些目标加 collider
3. 新建空物体并挂 `CameraObjectDetectionSaver`
4. 填好 `ObjectClasses`
5. 把采集相机填进 `CameraViews`
6. 设定 `DatasetFolder` 和 `SaveFrequencyHz`
7. 进入 Play，组件会自动开始保存

### 3.7 当前实现的注意事项

这个组件有几个很重要的实现特点：

1. 图片分辨率取决于 Game 视图分辨率，而不是单独的相机输出分辨率。
2. bbox 计算基于 `MeshFilter.sharedMesh.vertices`，不是基于 `SkinnedMeshRenderer`。
3. 默认的可见性检查使用 `Physics.Linecast`，因此目标最好在对应 mesh 物体上就有 collider。
4. 如果保存背景图，代码会生成空标签文件。
5. 多相机配置要谨慎。
   当前实现里 `_objectsInScene` 和异步回调 `OnCompleteReadback()` 是共享状态的，多个相机同时工作时有标签串扰风险。更稳妥的用法是：
   - 一个 saver 先只配置一个相机
   - 或者每个相机各用一个独立 saver

## 4. 声呐目标检测数据集

### 4.1 它做什么

`SonarObjectDetectionSaver` 是给 `Sonar3D` 用的。

它会定期：

1. 读取 `Sonar3D` 生成的笛卡尔声呐图 `sonarCartesianImage`
2. 读取与之对应的类别/实例编码图 `ClassInstanceImage`
3. 根据像素颜色里编码的类别和实例，恢复每个目标的 bbox
4. 保存图像和 YOLO 风格标签

输出格式是：

- 图片：`.png`
- 标签：`.txt`
- 标签格式同样是 `class x_center y_center width height`

### 4.2 怎么挂

它必须和 `Sonar3D` 挂在同一个对象上，因为代码里通过 `GetComponent<Sonar3D>()` 直接取声呐结果。

推荐挂法：

1. 找到你的声呐对象
2. 确保它已经挂了 `Sonar3D`
3. 在同一个对象上再挂 `SonarObjectDetectionSaver`
4. 在 saver 的 `ObjectClasses` 中填写目标类别和实例

### 4.3 它依赖什么

它依赖 `Sonar3D` 在射线命中时，把目标 collider 的类别和实例编码写进 `SonarReading`，之后又写入 `ClassInstanceImage`：

- `R` 通道编码 `ClassId`
- `G` 通道编码 `InstanceId`
- `B` 通道保存强度

因此要想被标注到，目标物体至少要满足：

1. 在 `ObjectClasses` 里被配置到
2. 自身或子物体上有 collider
3. 能被声呐射线命中

### 4.4 Inspector 里关键参数

重点参数：

- `Enable`
  - 默认 `false`
  - 不手动打开就不会保存
- `ResumeFromLastTime`
  - 是否继续往上次 `run` 里追加
- `ObjectClasses`
  - 目标类别配置
- `SaveFrequencyHz`
  - 保存频率
- `IntensityThreshold`
  - 像素强度阈值
  - 低于阈值的不算目标
- `DatasetFolder`
  - 根目录
  - 为空时默认写到 `Assets/sonar_detection`
- `GenerateTestSubset`
- `BackgroundImageSaveProbability`
- `TrainSize` / `ValSize` / `TestSize`

### 4.5 输出目录结构

目录结构和相机 saver 基本一致：

```text
<DatasetFolder>/
  run0/
    images/
      train/
      val/
      test/
    labels/
      train/
      val/
      test/
    simulation.yaml
```

### 4.6 类别和实例编号规则

声呐 saver 的类别编号从 `1` 开始，不是从 `0` 开始。

而且它的实例编号也是按“每个类别内部”重新从 `1` 开始编号的，也就是：

- `ClassA` 的第 1 个物体实例 id 是 1
- `ClassB` 的第 1 个物体实例 id 也是 1

所以如果你后处理时要把 `(class_id, instance_id)` 组合成全局唯一实例，记得自己再做一层映射。

### 4.7 使用步骤

最短流程：

1. 在声呐对象上挂 `Sonar3D`
2. 在同一个对象上挂 `SonarObjectDetectionSaver`
3. 给所有待检测物体加 collider
4. 在 `ObjectClasses` 里配置类别和对象
5. 打开 `Enable`
6. 进入 Play，按 `SaveFrequencyHz` 自动导出

### 4.8 当前实现的注意事项

1. 这个组件默认 `Enable=false`，要手动打开。
2. 它是基于 `ClassInstanceImage` 做像素扫描，不是直接对 3D bbox 做投影。
3. 同一 range-bearing 位置上，如果有多个目标，后写入的目标会覆盖前面的目标。
4. 如果保存的是背景图，当前实现可能只写图片，不一定生成对应的空标签文件。
   如果你后续训练脚本要求“每张图都必须有一个同名 `.txt`”，需要自己补齐。

## 5. 点云分割数据集

### 5.1 它做什么

`PointCloudSegmentationSaver` 是给 `RaycastLidar` 用的。

它会定期：

1. 读取 `RaycastLidar.Points`
2. 读取 `RaycastLidar.Readings`
3. 只保留有效点
4. 保存带 intensity 的 `.pcd`
5. 保存每个点的 `(class_id, instance_id)` 二进制标签
6. 保存相邻帧之间的位姿增量到 `poses.txt`

### 5.2 怎么挂

它必须和 `RaycastLidar` 挂在同一个对象上。

推荐挂法：

1. 在激光雷达对象上挂 `RaycastLidar`
2. 在同一个对象上挂 `PointCloudSegmentationSaver`
3. 在 `ObjectClasses` 中配置目标类别和实例

### 5.3 Inspector 里关键参数

关键参数比较少：

- `Enable`
  - 默认 `false`
- `ObjectClasses`
  - 标注对象类别
- `SaveFrequencyHz`
- `SavePath`
  - 根目录
  - 为空时默认写到 `Assets/pointcloud_segmentation`
- `Namespace`
  - 会作为文件名前缀
  - 例如 `seq01_000123.pcd`

### 5.4 输出目录结构

目录结构如下：

```text
<SavePath>/
  run1/
    lidar/
      000000.pcd
      000001.pcd
    labels/
      000000.label
      000001.label
    poses.txt
```

注意这里的 `run` 起始是 `run1`，不是相机/声呐那样常见的 `run0`。

### 5.5 输出文件格式

#### `.pcd`

由 `PCDSaver.WriteToPcdFileWithIntensity()` 生成，字段是：

- `x`
- `y`
- `z`
- `intensity`

但写入顺序实际是：

- `x`
- `z`
- `y`
- `intensity`

也就是说它对 Unity 坐标做了轴顺序调整。

#### `.label`

这是二进制文件，不是文本。

每个点写 4 个字节：

1. `ushort class_id`
2. `ushort instance_id`

并且和 `.pcd` 里的有效点顺序一一对应。

#### `poses.txt`

每行 12 个数字，表示一帧到下一帧之间的旋转增量矩阵和位移增量：

```text
r00 r01 r02 tx r10 r11 r12 ty r20 r21 r22 tz
```

### 5.6 类别和实例编号规则

这里和声呐又不一样：

- `class_id` 从 `1` 开始
- `instance_id` 也是从 `1` 开始
- 但 `instance_id` 是全局递增，不会在每个类别内部重置

也就是说：

- `ClassA` 第一个实例可能是 `(1,1)`
- `ClassB` 第一个实例可能是 `(2,4)`

### 5.7 使用步骤

最短流程：

1. 在雷达对象上挂 `RaycastLidar`
2. 在同一个对象上挂 `PointCloudSegmentationSaver`
3. 给待分割对象加 collider
4. 配置 `ObjectClasses`
5. 打开 `Enable`
6. 进入 Play 自动保存

### 5.8 当前实现的注意事项

1. 它只保存 `IsValid == true` 的点。
2. `.label` 是二进制格式，后处理脚本要按 `uint16 + uint16` 去读。
3. `poses.txt` 保存的是帧间增量，不是全局位姿。
4. 这个 saver 依赖命中 collider 的 `InstanceID` 做类别映射，所以目标物体必须有 collider。

## 6. DataLogger 运行期记录系统

### 6.1 它做什么

`DataLogger` 不是数据集标注器，而是统一日志缓存器。

项目里很多组件会直接调用：

- `DataLogger.Instance.GetLogger<T>(topic)`
- `logger.Log(value)`

例如：

- `ImuSensor`
- `DepthSensor`
- `GnssSensor`
- `PoseSensor`
- `RaycastLidar`
- `Thruster`
- `DifferentialThruster`
- 声学/RF 通信设备
- `PathRecorder`

### 6.2 存的是什么

每条记录都包含：

- `TimeStamp`
- `SimulationTime`
- `Value`

其中 `Value` 是泛型对象，可以是：

- 向量
- 匿名对象
- 自定义结构

### 6.3 怎么触发保存

常见保存方式有两种：

1. 手动调用
   - `DataLoggerUtilities.SaveAllLogs(...)`
   - `DataLoggerUtilities.SaveLogsForTopic(...)`
2. 通过现有组件间接触发
   - `PauseMenu` 的 `Save()` / `SaveOnExit`
   - `PathRecorder` 在禁用时自动保存对应 topic

### 6.4 输出位置和格式

默认保存到：

- `Assets/Saves`

格式是 JSON。

`SaveAllLogs()` 会输出一个整体 JSON，包含：

- 场景名/描述
- 仿真时长
- 所有 topic 的日志

### 6.5 什么时候用它

如果你的目标是：

- 记录传感器数值用于调试
- 保存推进器输入、通信事件、路径轨迹
- 做离线分析

用 `DataLogger`。

如果你的目标是：

- 训练检测器
- 训练分割网络
- 生成带标签图像/点云

用 `ObjectAnnotation/*Saver`。

## 7. 推荐挂载方式

如果你要在同一个仿真项目里同时做多模态数据集采集，比较稳妥的挂法是：

1. 建一个单独的 `DatasetManager`
   - 挂 `CameraObjectDetectionSaver`
   - 每个 saver 最好先只配一个 camera
2. 在每个 `Sonar3D` 对象上直接挂 `SonarObjectDetectionSaver`
3. 在每个 `RaycastLidar` 对象上直接挂 `PointCloudSegmentationSaver`
4. 所有需要被标注的物体统一加好 collider
5. 类别表尽量在相机/声呐/点云三套 saver 中保持一致命名

## 8. 实用建议

1. 相机保存前，先固定好 Unity Game 视图分辨率，否则图片分辨率会变。
2. 相机 saver 默认做 raycast 可见性检查，所以目标最好加 mesh 级别 collider。
3. 声呐和点云 saver 都依赖 collider 命中结果，没有 collider 就不会被标注。
4. 如果你后处理时要求严格的 YOLO 目录完整性，声呐数据建议自己补“空标签文件”。
5. 如果你要对齐多模态采样时间，尽量把几个 saver 的 `SaveFrequencyHz` 配成同一频率，并避免太高频导致磁盘瓶颈。
6. 点云 `.label` 需要你自己写 reader；它不是现成可读文本。

## 9. 一句话总结

MARUS 当前的自动数据采集体系，本质上是：

- 相机和声呐走“图像 + YOLO bbox 标签”
- 激光雷达走“PCD + 二进制逐点标签 + 位姿增量”
- 运行期数值记录走“DataLogger JSON”

使用上最关键的前提，是把目标对象、相机/声呐/雷达组件和 collider 关系正确挂好。
