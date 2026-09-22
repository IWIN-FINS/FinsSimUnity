using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.Rendering.HighDefinition;
using System;
// #if CREST_OCEAN
//     using Crest;
// #endif
using FinsSim.Ocean;

[Serializable, VolumeComponentMenu("Post-processing/Custom/FogEffect")]
public sealed class FogEffect : CustomPostProcessVolumeComponent, IPostProcessComponent
{
    static readonly int MainTexId = Shader.PropertyToID("_MainTex");

    [Tooltip("Controls the intensity of the effect.")]
    public ClampedFloatParameter intensity = new ClampedFloatParameter(0f, 0f, 1f);
    public ColorParameter fogColor = new ColorParameter(Color.blue);
    public BoolParameter applyInSceneView = new BoolParameter(false);

    Material m_Material;

    public bool IsActive()
    {
        return m_Material != null && intensity.value > 0f;
    }

    // Do not forget to add this post process in the Custom Post Process Orders list (Project Settings > Graphics > HDRP Settings).
    public override CustomPostProcessInjectionPoint injectionPoint => CustomPostProcessInjectionPoint.AfterPostProcess;

    const string kShaderName = "Hidden/Shader/FogEffect";

    public override void Setup()
    {
        if (Shader.Find(kShaderName) != null)
            m_Material = new Material(Shader.Find(kShaderName));
        else
            Debug.LogError($"Unable to find shader '{kShaderName}'. Post Process Volume FogEffect is unable to load.");
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

        m_Material.SetFloat("_Intensity", intensity.value);
        m_Material.SetColor("_FogColorr", fogColor.value);
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
