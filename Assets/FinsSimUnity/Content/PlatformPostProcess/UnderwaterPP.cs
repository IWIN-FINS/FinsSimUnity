using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.HighDefinition;
using System;
using FinsSim.Ocean;

[Serializable, VolumeComponentMenu("Post-processing/Custom/UnderwaterPP")]
public sealed class UnderwaterPP : CustomPostProcessVolumeComponent, IPostProcessComponent
{
    static readonly int MainTexId = Shader.PropertyToID("_MainTex");

    [Tooltip("Controls the intensity of the effect.")]
    public ClampedFloatParameter noiseScale = new ClampedFloatParameter(0f, 0f, 1f);
    public ClampedFloatParameter noiseFrequency = new ClampedFloatParameter(0f, 0f, 10f);
    public ClampedFloatParameter noiseSpeed = new ClampedFloatParameter(0f, 0f, 10f);
    public ClampedFloatParameter pixelOffset = new ClampedFloatParameter(0f, 0f, 1f);
    public BoolParameter applyInSceneView = new BoolParameter(false);

    Material m_Material;

    public bool IsActive()
    {
        return m_Material != null &&
            (noiseScale.value > 0f || noiseFrequency.value > 0f || noiseSpeed.value > 0f || pixelOffset.value > 0f);
    }

    // Do not forget to add this post process in the Custom Post Process Orders list (Project Settings > Graphics > HDRP Settings).
    public override CustomPostProcessInjectionPoint injectionPoint => CustomPostProcessInjectionPoint.AfterPostProcess;

    const string kShaderName = "Hidden/Shader/UnderwaterPP";

    public override void Setup()
    {
        if (Shader.Find(kShaderName) != null)
            m_Material = new Material(Shader.Find(kShaderName));
        else
            Debug.LogError($"Unable to find shader '{kShaderName}'. Post Process Volume UnderwaterPP is unable to load.");
    }

    public override void Render(CommandBuffer cmd, HDCamera camera, RTHandle source, RTHandle destination)
    {
        if (m_Material == null)
            return;

        if (IsSceneViewCamera(camera) && !applyInSceneView.value)
        {
            HDUtils.BlitCameraTexture(cmd, source, destination);
            return;
        }

        if (!IsCameraUnderwater(camera))
        {
            HDUtils.BlitCameraTexture(cmd, source, destination);
            return;
        }

        m_Material.SetFloat("_NoiseScale", noiseScale.value);
        m_Material.SetFloat("_NoiseFrequency", noiseFrequency.value);
        m_Material.SetFloat("_NoiseSpeed", noiseSpeed.value);
        m_Material.SetFloat("_PixelOffset", pixelOffset.value);
        m_Material.SetTexture(MainTexId, source);
        HDUtils.DrawFullScreen(cmd, m_Material, destination, shaderPassId: 0);
    }

    static bool IsSceneViewCamera(HDCamera camera)
    {
        Camera unityCamera = camera?.camera;
        return unityCamera != null && unityCamera.cameraType == CameraType.SceneView;
    }

    static bool IsCameraUnderwater(HDCamera camera)
    {
        Camera unityCamera = camera?.camera;
        if (unityCamera == null)
            return false;

        float waterLevel = 0f;
        if (WaterHeightSampler.TryGetInstance(out WaterHeightSampler sampler))
        {
            waterLevel = sampler.GetWaterLevel(unityCamera.transform.position);
        }

        return unityCamera.transform.position.y < waterLevel;
    }

    public override void Cleanup()
    {
        CoreUtils.Destroy(m_Material);
    }
}
