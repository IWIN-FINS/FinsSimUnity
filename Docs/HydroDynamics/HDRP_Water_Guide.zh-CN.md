
有讲究，而且 HDRP Water 的颜色不是单纯“材质颜色”。它更像一组光学参数：水面反射、折射、吸收、散射、泡沫、焦散、雾和水深一起决定最后看到的颜色。

**核心调色项**
在 `Water Surface > Appearance` 里主要看这些：

- `Refraction Color`：透过水看物体时的染色。偏蓝/绿会让水体更海水感。
- `Absorption Distance`：越小，水越快吸光，越暗、越浑、越不透明；越大，越清澈、看得越深。
- `Max Refraction Distance`：折射影响距离。太大容易画面扭曲，仿真里别过头。
- `Scattering Color`：水体散射出来的颜色，决定“水本身发蓝/发绿”的感觉。
- `Ambient / Height / Displacement Scattering`：越高越亮、越雾、越奶；浑浊水可加，清水要低。
- `Smoothness`：影响反射锐利度。清水/泳池高一点，浑浊河水低一点。
- `Foam / Caustics / Underwater`：泡沫、焦散、水下后处理，对真实感很重要。

一个直觉：**清澈水 = 高 Absorption Distance + 低 Scattering；浑浊水 = 低 Absorption Distance + 高 Scattering + 低 Smoothness。**

**三种 Water Surface 类型**
HDRP 17 里真正的水面类型是三种，文档也这么写：

`Ocean, Sea, or Lake`
适合大面积水体。它有最多的波浪模拟：两个 swell 大浪 band，加一个 ripples 小波 band。适合海洋、湖泊、水库、开放水域。这个最适合你们水下仿真平台的默认大水体。

`River`
适合有明确流向的水。它有 agitation + ripples，重点是 current/flow 方向、河道流动和局部水流控制。适合河流、运河、水槽里有明显流速的场景。

`Pool`
适合小而清澈、边界明确的水。只有 ripples，没有大尺度 swell，泡沫/浪尖散射能力也少。适合泳池、实验池、静态水箱、室内测试池。

**Water Decal 是什么**
`Water Decal` 不是一片新水面，它是局部影响器。可以局部加：

- foam 泡沫
- deformation 水面变形
- current 水流方向/强度
- mask 抑制某处 wave/foam/swell/ripple

比如岸边泡沫、船边扰动、局部水流、障碍物附近涡流，都应该用 Water Decal，而不是新建一片水。

**Excluder 是什么**
`Water Excluder` 也不是水。它是“把水面从某些屏幕像素里挖掉”。典型用途：船舱、甲板内部、浮体内部不应该出现水面，就给船内部放一个 excluder mesh。

注意它主要影响从上方看到的水面，不等于真实切掉水体，也不一定影响 underwater view。

**我建议你这样选**
水下仿真平台默认：`Ocean, Sea, or Lake`。
如果是海试/湖试/大水域，调低浪高也比用 Pool 更合适，因为它的水下、焦散、波浪 band 更完整。

训练稳定性优先：视觉用 `Ocean, Sea, or Lake`，物理仍可用 DWP2 flat/current provider，避免波浪扰动物理。
视觉真实优先：HDRP Water + DWP2 `UnityHDRPWaterDataProvider`，让水面高度参与物理。

大概参数方向：

- 清澈海水：蓝绿 `Scattering Color`，`Absorption Distance` 50-150m，smoothness 高。
- 近岸浑水：偏绿/青灰，`Absorption Distance` 5-30m，scattering 增加，smoothness 降低。
- 湖水：偏绿蓝，浪小，swell 低，ripples 中等。
- 河水：用 `River`，颜色偏绿/棕，absorption 小，current 明确。
- 实验池：用 `Pool`，absorption 大，scattering 小，ripples 很弱。

最重要的一点：**不要只调水面颜色，要同时调 underwater volume/fog。** 水下仿真的真实感，大头其实来自吸收、散射、雾、能见度和光照，而不是水面那一层颜色。
