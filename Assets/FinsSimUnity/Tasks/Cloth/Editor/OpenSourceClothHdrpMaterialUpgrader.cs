using System;
using UnityEditor;
using UnityEditor.Rendering.HighDefinition;
using UnityEngine;

/// <summary>
/// Converts the materials imported for the open-source cloth comparisons to
/// HDRP/Lit.  The source projects target the Built-in Render Pipeline, so
/// retaining their original shader references makes HDRP report unsupported
/// materials (and usually renders them pink).
/// </summary>
public static class OpenSourceClothHdrpMaterialUpgrader
{
    private static readonly string[] MaterialRoots =
    {
        "Assets/FinsSimUnity/Generated/OpenSourceCloth",
        "Assets/ThirdParty/OpenSourceCloth",
    };

    [MenuItem("FinsSim/Open Source Cloth/Upgrade Materials to HDRP")]
    public static void UpgradeMaterials()
    {
        Shader hdrpLit = Shader.Find("HDRP/Lit");
        if (hdrpLit == null)
        {
            throw new InvalidOperationException(
                "HDRP/Lit shader was not found. Assign the HDRP render pipeline asset before upgrading the cloth materials.");
        }

        int converted = 0;
        int alreadyHdrp = 0;
        foreach (string guid in AssetDatabase.FindAssets("t:Material", MaterialRoots))
        {
            string path = AssetDatabase.GUIDToAssetPath(guid);
            Material material = AssetDatabase.LoadAssetAtPath<Material>(path);
            if (material == null)
            {
                continue;
            }

            if (material.shader == hdrpLit)
            {
                alreadyHdrp++;
                continue;
            }

            // Read Built-in properties before assigning HDRP/Lit; Unity drops
            // the unsupported property block as part of that assignment.
            Color baseColor = ReadColor(material, "_BaseColor", "_Color", Color.white);
            Texture baseColorMap = ReadTexture(material, "_BaseColorMap", "_BaseMap", "_MainTex");
            float metallic = ReadFloat(material, "_Metallic", 0f);
            float smoothness = ReadFloat(material, "_Smoothness", "_Glossiness", 0.5f);
            Color emission = ReadColor(material, "_EmissiveColor", "_EmissionColor", Color.black);

            material.shader = hdrpLit;
            material.SetColor("_BaseColor", baseColor);
            material.SetTexture("_BaseColorMap", baseColorMap);
            material.SetFloat("_Metallic", metallic);
            material.SetFloat("_Smoothness", smoothness);
            material.SetColor("_EmissiveColor", emission);
            material.SetFloat("_SurfaceType", baseColor.a < 0.999f ? 1f : 0f);
            material.enableInstancing = true;
            HDShaderUtils.ResetMaterialKeywords(material);
            EditorUtility.SetDirty(material);
            converted++;
        }

        AssetDatabase.SaveAssets();
        AssetDatabase.Refresh();
        Debug.Log($"[OpenSourceCloth] Converted {converted} materials to HDRP/Lit; {alreadyHdrp} were already HDRP/Lit.");
    }

    private static Color ReadColor(Material material, string primary, string fallback, Color defaultValue)
    {
        if (material.HasProperty(primary))
        {
            return material.GetColor(primary);
        }

        return material.HasProperty(fallback) ? material.GetColor(fallback) : defaultValue;
    }

    private static Texture ReadTexture(Material material, params string[] propertyNames)
    {
        foreach (string propertyName in propertyNames)
        {
            if (material.HasProperty(propertyName))
            {
                Texture texture = material.GetTexture(propertyName);
                if (texture != null)
                {
                    return texture;
                }
            }
        }

        return null;
    }

    private static float ReadFloat(Material material, string primary, string fallback, float defaultValue)
    {
        if (material.HasProperty(primary))
        {
            return material.GetFloat(primary);
        }

        return material.HasProperty(fallback) ? material.GetFloat(fallback) : defaultValue;
    }

    private static float ReadFloat(Material material, string propertyName, float defaultValue)
    {
        return material.HasProperty(propertyName) ? material.GetFloat(propertyName) : defaultValue;
    }
}
