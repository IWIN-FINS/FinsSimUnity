using UnityEngine;
using NWH.DWP2.WaterObjects;
using NWH.DWP2.ShipController;
using NWH.DWP2.WaterData;
using NWH.DWP2.SailController;
/// <summary>
/// 统一控制 DWP2 所有调试 Gizmos 的显示/隐藏
/// 挂载到任意场景对象即可生效
/// </summary>
public class DWP2DebugController : MonoBehaviour
{
    [Header("调试显示设置")]
    [Tooltip("是否显示浮力球和网格调试")]
    public bool showBuoyancyDebug = false;

    [Tooltip("是否显示推进器调试")]
    public bool showThrusterDebug = false;

    [Tooltip("是否显示水数据提供商标记")]
    public bool showWaterProviderDebug = false;

    [Tooltip("是否显示锚点调试")]
    public bool showAnchorDebug = false;

    [Tooltip("是否显示帆控制器调试")]
    public bool showSailControllerDebug = false;

    [Tooltip("是否显示沉没调试点")]
    public bool showSinkDebug = false;

    [Tooltip("是否显示质心调试")]
    public bool showCoMDebug = false;

    [Tooltip("是否显示水数据提供商的触发器碰撞体（巨大黄色球体）。\n注意：这是编辑器只显示，取消勾选Scene视图顶部的Gizmos>Colliders来隐藏")]
    public bool showWaterProviderCollider = true;

    void Start()
    {
        ApplySettings();
    }

    void OnValidate()
    {
        ApplySettings();
    }

    public void ApplySettings()
    {
        // 浮力球和网格调试
        WaterObject.ShowWaterObjectGizmos = showBuoyancyDebug;

        // 推进器调试
        AdvancedShipController.ShowShipControllerGizmos = showThrusterDebug;

        // 水数据提供商标记
        WaterDataProvider.ShowWaterDataProviderGizmos = showWaterProviderDebug;

        // 锚点调试
        Anchor.ShowAnchorGizmos = showAnchorDebug;

        // 帆控制器调试
        SailController.ShowSailControllerGizmos = showSailControllerDebug;

        // 沉没调试点
        Sink.ShowSinkGizmos = showSinkDebug;

        // 质心调试 - 需要通过其他方式控制
        // CoM调试在 VariableCenterOfMass 中
        
        // 注意：WaterDataProvider 的巨大 SphereCollider（黄色球体）不能通过代码禁用
        // 因为会破坏触发器检测。请在 Scene 视图顶部的 Gizmos > Colliders 中取消勾选来隐藏
    }

    void OnDrawGizmos()
    {
        // 此脚本本身不绘制任何 Gizmos
    }

    /// <summary>
    /// 快速关闭所有调试显示
    /// </summary>
    public void HideAllDebug()
    {
        showBuoyancyDebug = false;
        showThrusterDebug = false;
        showWaterProviderDebug = false;
        showAnchorDebug = false;
        showSailControllerDebug = false;
        showSinkDebug = false;
        showCoMDebug = false;
        ApplySettings();
    }

    /// <summary>
    /// 快速显示所有调试显示
    /// </summary>
    public void ShowAllDebug()
    {
        showBuoyancyDebug = true;
        showThrusterDebug = true;
        showWaterProviderDebug = true;
        showAnchorDebug = true;
        showSailControllerDebug = true;
        showSinkDebug = true;
        showCoMDebug = true;
        ApplySettings();
    }

    /// <summary>
    /// 在运行时通过控制台命令切换
    /// 在 Unity Console 中输入: DWP2DebugController.ToggleAll()
    /// </summary>
    public static void ToggleAll()
    {
        bool newState = !WaterObject.ShowWaterObjectGizmos;
        WaterObject.ShowWaterObjectGizmos = newState;
        AdvancedShipController.ShowShipControllerGizmos = newState;
        WaterDataProvider.ShowWaterDataProviderGizmos = newState;
        Anchor.ShowAnchorGizmos = newState;
        SailController.ShowSailControllerGizmos = newState;
        Sink.ShowSinkGizmos = newState;

        Debug.Log($"DWP2 Debug Gizmos set to: {newState}");
    }
}