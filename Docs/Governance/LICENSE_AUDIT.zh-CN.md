# FinsSimUnity 许可证与授权链审计

审计日期：2026-09-16
审计范围：当前 Unity 工程的 Assets、Packages、Git LFS 文件、许可证/NOTICE、上游资源与已检出项目历史。
状态：初审；在排除或获得授权前，不应将整个 Unity 工程作为 Apache-2.0 + Commercial License 项目公开。

## 结论摘要

FinsSim 自有脚本、场景、配置及模型可在权属证明齐全后按 Apache-2.0 发布；商业协议只覆盖权属明确的 FinsSim 自有部分。工程目前包含 Unity Asset Store 原始资源、多个独立开源组件、上游 MARUS/UUV 模型、来源不明的模型与二进制。根 LICENSE 不能覆盖这些第三方内容；当前根 [THIRD_PARTY_NOTICES.md](../THIRD_PARTY_NOTICES.md) 需要扩充。

Unity Asset Store 标准 EULA 的授权重点是把资产作为非实质组成部分嵌入有实质原创内容的产品，并限制未获授权的资产分发；因此本审计把直接公开 Asset Store 原始工程资源列为阻断项，除非对应 Provider 条款或书面授权明确允许。[Unity Asset Store Terms / EULA](https://unity.com/legal/as-terms)

## 建议的授权链条

```text
模型/脚本/场景/文档的实际作者、雇员、承包商、合作机构、上游贡献者
  │ 逐项确认职务作品、委托开发、共同开发、机构/资助约束与许可/转让
  ▼
经核实的 FinsSim 权利主体（签约主体必须使用完整法定名称）
  ├─ 对 FinsSim 自有贡献：可授予 Apache-2.0
  ├─ 对同一自有贡献：可按单独书面商业协议授予额外商业权利
  └─ 对第三方资产：只能按权利人原许可证使用，或取得明确扩展许可
       ├─ MARUS / UUV / MIT / BSD / Apache / CC / OFL / Unity Companion：保留原声明
       ├─ Unity Asset Store / 私有二进制：排除，或取得公开源文件再分发授权
       └─ 来源不明模型/素材：在权属证据补齐前不发布
  ▼
源码仓库 / LFS / Unity Package / 构建产物分别生成许可证清单和第三方通知
```

落地原则：

1. 将“FinsSim 作者身份”和“有权授权的主体”分开证明。仓库 NOTICE 当前写 IWIN-FINS；发布或签商业合同前，确认其完整法定名称、权利归属、代表人权限以及职务作品/委托成果约定。Git 作者名、Unity .meta 的 Store 标记或单独支付 Asset Store 费用均不等于取得原始资产公开再分发权。
2. 建立逐项权利台账：路径、作者/权利人、上游 URL 和 commit/版本、取得方式、原始许可证、修改/派生说明、允许的分发形式、授权凭证、负责核验人。
3. FinsSim 自有部分可以双授权，但第三方组件保持其自身许可证。Apache-2.0 本身允许商业使用；商业合同应准确说明它提供的额外内容（如专有再分发、支持、服务），不能把上游代码或素材写成 FinsSim 所有，也不能撤回已经授出的 Apache 权利。
4. 源码、Git LFS、Unity Package、预制件/场景序列化数据、DLL/原生库、最终玩家构建分别检查。移除第三方源文件时，同时审查场景/Prefab 是否带有受限素材副本或不可公开的序列化内容，并提供合法的重新安装步骤。

## 逐项资源盘点

| 资源 | 仓库内证据 / 许可证 | 结论 / 建议动作 |
| --- | --- | --- |
| FinsSim 自有 Assets/Scripts、场景、ProjectSettings、配置和文档 | 根 LICENSE 为 Apache-2.0；NOTICE 声明 FinsSim 自有部分适用 | 权属证明完成后可 Apache 发布。历史中包含 MARUS/LABUST 作者贡献，必须保留上游归属；确认自有新增代码的职务作品、合作和外部贡献授权后再双授权。 |
| Obi Rope | Assets/Obi 约 2,002 个跟踪文件；AssetOrigin 元数据记录 Asset Store 产品及版本 7.1.1；未发现其目录的公开源许可证 | **阻断项。** 从公开仓库/LFS/发行包中排除，或者取得 Provider 对公开源文件分发的书面授权。仅在 Unity 产品中使用的许可不能自动证明可在公开代码仓库分发原始资产。 |
| NWH Common / Dynamic Water Physics 2 / Samples | Packages/com.nwh.common 约 348 个文件、com.nwh.dynamicwaterphysics 约 342 个文件、Assets/Samples/BaseSample 约 480 个文件；AssetOrigin 显示产品版本 14.1.0 | **阻断项。** 这些包和样例须按 Asset Store/Provider 条款处理，不能由 Apache 或 FinsSim 商业许可覆盖。若从公共版移除，另给合法获取和导入说明；检查引用它们的场景、Prefab 和构建产物。包内 MIConvexHull 有 MIT LICENSE，Inconsolata 字体有 OFL 文件，这些独立子项保留各自条款。 |
| Flexible Color Picker | Assets/Plugins/FlexibleColorPicker 有 93 个跟踪文件；README 称其为 Unity Asset Store 免费资源 | “免费”不代表可公开镜像原始源码/素材。**阻断项：** 排除或获得明确再分发许可。 |
| Store 标记脚本 | 扫描到 316 个含 licenseType: Store 的 .meta；在已识别 Obi/NWH 外仍有 Packages/com.iwin-fins.fins-sim/Runtime/Platform/Diver 和 TutorialInfo/ReadmeEditor 等 | 核对每个文件来源、Asset Store Provider 及具体条款；不能只依赖名称猜测其为 Unity Standard Assets。确认后在 NOTICE 中按实际许可记录，无法证明则移除/替换。 |
| Crest 核心 | Assets/Crest/Crest 有 MIT LICENSE | 可按 MIT 分发并保留 Wave Harmonic/贡献者版权。此许可证不自动覆盖 Crest-Examples 中的独立素材。 |
| Crest 示例、地形、灯光和图标 | LakesAndRivers/ThirdPartyNotices 说明场景艺术资产受 Unity Asset Store EULA 管辖，并记载 OpenStreetMap/USGS 来源；MaterialIcons 有 Apache LICENSE；LightFlares 有 Unity Companion License | LakesAndRivers 的 Store 艺术资产是发布阻断项，除非得到授权；保留 OSM/USGS 署名、MaterialIcons Apache 条款和 LightFlares Companion License。后者限定与有效 Unity Engine License 的使用关系。[Unity Companion License](https://unity.com/cn/legal/licenses/unity-companion-license) |
| Crest 示例音频 | 本地 License.txt 将 WaterPier 标为 CC BY 3.0，UnderwaterWhiteNoise 标为 CC0；当前 Freesound 页面显示 WaterPier 作者 aesqe、CC BY 4.0，另一条音频由 psiboy123 上传且为 CC0 | WaterPier 的本地与当前来源许可证版本不一致，应查当时下载记录/文件许可；补足 aesqe 署名、标题、链接、适用许可版本。UnderwaterWhiteNoise 按 CC0 记录来源即可。[WaterPier](https://freesound.org/people/aesqe/sounds/39901/)、[UnderwaterWhiteNoise](https://freesound.org/people/psiboy123/sounds/448460/) |
| MARUS core 与嵌套水面资源 | Packages/com.iwin-fins.fins-sim 有 Apache LICENSE；Environment/UnityHDRPSimpleWater 有独立 MIT LICENSE | 保留两者原许可证和 LABUST/MARUS 署名。MARUS 上游说明核心为 Apache-2.0；此目录内的 Store 插件、字体和 DLL 仍要逐项独立分类。[MARUS core](https://github.com/MARUSimulator/marus-core) |
| OpenSourceCloth | ClothBehaviourSimulation 为 Apache-2.0；TenMinutePhysicsUnity 和 UnitySimplePhysics 各有 MIT LICENSE | 许可证文件已随组件保留；更新第三方清单，确保 NOTICE 能从仓库根目录发现这三项。 |
| MathNet.Numerics DLL | 已统一为 `Packages/com.iwin-fins.fins-sim/Runtime/Plugins/MathNet.Numerics.dll` 的单一副本；OpenSourceCloth 不再携带重复 importer | 上游声明 Math.NET Numerics 为 MIT；版本线为 5.x，已在包内第三方通知登记。发布前仍应记录发行时 hash。[Math.NET Numerics](https://github.com/mathnet/mathnet-numerics) |
| Unity ML-Agents package | Packages/com.unity.ml-agents 为 4.0.3，目录有 Unity Apache LICENSE | Unity ML-Agents 自身 Apache 声明保留；其目录内另带 Google.Protobuf、Grpc.Core、System.Interactive.Async、System.IO.Abstractions/TestHelpers 和原生 gRPC 文件，须补齐每项版本、许可证和 NOTICE。 |
| MARUS 插件 DLL 与原生库 | Google.Protobuf、Grpc.Core/API、MathNet、DiagnosticSource、Unsafe 及 gRPC 平台二进制；部分由 Git LFS 跟踪 | **需核验后才能作为完整工程公开。** 记录确切上游版本、许可证、版权和二进制来源；不要只把 MARUS 根 Apache LICENSE 当作所有 DLL 的许可证。 |
| ClothWaterInteraction.dll | 编译 DLL 引用了 NWH/Obi 类型；仓库存在相关源码，但 DLL 的生成来源/版本关系未记录 | **需核验或不发布。** 确认是纯自有构建产物、与源码对应且未包含第三方受限实现；更稳妥是只发源码及构建步骤，不发不必要的二进制。 |
| ThirdPartyModels | 包含 BlueROV2、Desistek SAGA、ECA A9、LAUV、RexROV2、Orca4 等 URDF/网格/转换资源；部分 raw URDF 带 UUV Apache 版权头 | 按来源逐项保留许可证/NOTICE。UUV Simulator 和 ECA A9 上游为 Apache-2.0；Orca4 上游为 MIT。Unity 根 THIRD_PARTY_NOTICES 尚未逐一列出这些模型、上游提交及转换情况。[UUV Simulator](https://github.com/uuvsimulator/uuv_simulator)、[ECA A9](https://github.com/uuvsimulator/eca_a9)、[Orca4 LICENSE](https://github.com/clydemcqueen/orca4/blob/main/LICENSE) |
| BlueROV 独立 OBJ、Yamaha 船模、FinsROV FBX、浮标纹理 | Assets/Models 下来源说明不完整；多个模型由 Git LFS 跟踪；FinsROV 模型可疑似自有，但文件本身不能证明权属 | **阻断项。** 找到原始来源/作者及授权文件；对 BlueROV/Yamaha 外观与商标也确认权利边界。FinsROV 由员工/外包设计的模型需有职务作品/委托成果链。无法证明就从公开版移除。 |
| MARUS 示例船与岛屿地形 | 仓库历史对应 MARUSimulator/marus-example 的 Apache-2.0 项目来源 | 按 Apache 保留原项目/作者通知；不能把这些既有 MARUS 内容列成 FinsSim 独占创作。[MARUS 示例 LICENSE](https://github.com/MARUSimulator/marus-example/blob/main/LICENSE) |
| 其他项目输出 | Unity 仓库还跟踪 Mono crash dump、log 等文件 | 非许可证授权项，但属于发布卫生/潜在隐私风险；应确认是否必要并在公开发布前清理。 |

## 发布门禁

- [ ] 确认 FinsSim 的完整法定版权主体、人员贡献的职务/委托/合作权利链；商业授权范围准确列出 FinsSim 自有部分。
- [ ] 公开版移除 Obi、NWH、Flexible Color Picker 和受 Asset Store 条款约束的 Crest 示例资源，或取得明确的公开源文件再分发许可。
- [ ] 解决 316 个 Store 元数据标记中未分类资源的来源；逐个关联到具体许可证/Provider。
- [ ] 为 Google.Protobuf、Grpc.Core、ML-Agents 插件、MathNet、原生 gRPC 等二进制完成组件清单和通知。
- [ ] 确认 BlueROV、Yamaha、FinsROV、标记/浮标及 LFS 素材的权利来源；核对 ClothWaterInteraction.dll。
- [ ] 修正 WaterPier 音频许可证版本/署名，并在第三方声明中列出 Crest、OpenSourceCloth、MaterialIcons、字体、模型、音频和 OSM/USGS 资源。
- [ ] 对源码仓库和最终 Unity Player 分别做发行审查；Player 中只按第三方许可证允许的形式分发资源，不因源仓库 Apache 声明而扩大授权。
- [ ] 清理不必要的 crash dump/log，确保公开 LFS 对象的访问和许可范围与 Git 追踪文件一致。

## 参考来源

- [Apache License 2.0 正文](https://www.apache.org/licenses/LICENSE-2.0.txt)
- [Unity Asset Store Terms / EULA](https://unity.com/legal/as-terms)
- [Unity Companion License](https://unity.com/cn/legal/licenses/unity-companion-license)
- [MARUS core](https://github.com/MARUSimulator/marus-core) 与 [MARUS 示例 LICENSE](https://github.com/MARUSimulator/marus-example/blob/main/LICENSE)
- [UUV Simulator](https://github.com/uuvsimulator/uuv_simulator)、[ECA A9](https://github.com/uuvsimulator/eca_a9)、[Orca4 LICENSE](https://github.com/clydemcqueen/orca4/blob/main/LICENSE)
- Freesound: [WaterPier by aesqe](https://freesound.org/people/aesqe/sounds/39901/)、[Underwater White Noise by psiboy123](https://freesound.org/people/psiboy123/sounds/448460/)
- [Math.NET Numerics](https://github.com/mathnet/mathnet-numerics)

本清单描述审计时仓库中的资源与可见许可证证据，不替代权利人书面许可、供应商合同或正式法律意见。遇到上游页面许可与仓库随附许可证冲突时，应以取得该副本时的授权记录和适用协议为准；在证据补齐前按“未获公开再分发许可”处理。
