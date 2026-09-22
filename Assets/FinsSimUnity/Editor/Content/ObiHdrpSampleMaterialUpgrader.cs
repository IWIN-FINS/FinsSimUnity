using UnityEditor;
using UnityEditor.Rendering.HighDefinition;
using UnityEngine;

/// <summary>
/// Converts the visual-only materials shipped with Obi sample scenes from the
/// Built-in pipeline shaders to HDRP/Lit. Obi solver and renderer scripts are
/// deliberately untouched.
/// </summary>
public static class ObiHdrpSampleMaterialUpgrader
{
    private static readonly string[] SampleRoots =
    {
        "Assets/Obi/Samples/Common/SampleResources/Materials",
        "Assets/Obi/Samples/RopeAndRod/SampleResources/Materials",
    };

    [MenuItem("FinsSim/Obi/Upgrade Sample Materials to HDRP")]
    public static void UpgradeSampleMaterials()
    {
        Shader hdrpLit = Shader.Find("HDRP/Lit");
        if (hdrpLit == null)
        {
            throw new System.InvalidOperationException("HDRP/Lit shader was not found. Enable HDRP before upgrading Obi sample materials.");
        }

        int upgraded = 0;
        int instancingEnabled = 0;
        foreach (string guid in AssetDatabase.FindAssets("t:Material", SampleRoots))
        {
            string path = AssetDatabase.GUIDToAssetPath(guid);
            Material material = AssetDatabase.LoadAssetAtPath<Material>(path);
            if (material == null)
            {
                continue;
            }

            if (material.shader != hdrpLit)
            {
                // Preserve the visible part of the sample materials before the
                // shader swap removes their Built-in Standard properties.
                Color baseColor = material.HasProperty("_Color") ? material.color : Color.white;
                Texture baseMap = material.HasProperty("_MainTex") ? material.mainTexture : null;
                float metallic = material.HasProperty("_Metallic") ? material.GetFloat("_Metallic") : 0f;
                float smoothness = material.HasProperty("_Glossiness") ? material.GetFloat("_Glossiness") : 0.5f;
                Color emission = material.HasProperty("_EmissionColor") ? material.GetColor("_EmissionColor") : Color.black;

                material.shader = hdrpLit;
                material.SetColor("_BaseColor", baseColor);
                material.SetTexture("_BaseColorMap", baseMap);
                material.SetFloat("_Metallic", metallic);
                material.SetFloat("_Smoothness", smoothness);
                material.SetColor("_EmissiveColor", emission);
                HDShaderUtils.ResetMaterialKeywords(material);
                upgraded++;
            }

            // BurstChainRopeRenderSystem uses Graphics.RenderMeshInstanced.
            // HDRP materials must explicitly opt in or every rope-chain scene
            // logs an exception and renders no links.
            if (!material.enableInstancing)
            {
                material.enableInstancing = true;
                instancingEnabled++;
            }

            EditorUtility.SetDirty(material);
        }

        AssetDatabase.SaveAssets();
        AssetDatabase.Refresh();
        Debug.Log($"Upgraded {upgraded} Obi sample materials to HDRP/Lit; enabled instancing on {instancingEnabled} materials.");
    }
}
