using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

[CustomEditor(typeof(FishNetGenerator))]
public class FishNetGeneratorEditor : Editor
{
    public override void OnInspectorGUI()
    {
        serializedObject.Update();
        DrawDefaultInspector();
        serializedObject.ApplyModifiedProperties();

        FishNetGenerator generator = (FishNetGenerator)target;

        EditorGUILayout.Space();
        EditorGUILayout.LabelField("Static Net Tools", EditorStyles.boldLabel);
        EditorGUILayout.HelpBox(
            "Generate Static Net 会在当前场景中直接创建 GeneratedFishNet 子层级。若关闭 generateOnStart，运行时会复用这张静态网。",
            MessageType.Info);

        using (new EditorGUI.DisabledScope(Application.isPlaying))
        {
            if (GUILayout.Button("Generate / Regenerate Static Net"))
            {
                GenerateStaticNet(generator, disableRuntimeRebuild: false);
            }

            if (GUILayout.Button("Generate Static Net And Disable Runtime Rebuild"))
            {
                GenerateStaticNet(generator, disableRuntimeRebuild: true);
            }

            using (new EditorGUI.DisabledScope(generator.NetRoot == null))
            {
                if (GUILayout.Button("Clear Static Net"))
                {
                    ClearStaticNet(generator);
                }
            }
        }

        if (Application.isPlaying)
        {
            EditorGUILayout.HelpBox("Static generation buttons are disabled in Play Mode.", MessageType.None);
        }
    }

    private static void GenerateStaticNet(FishNetGenerator generator, bool disableRuntimeRebuild)
    {
        Undo.RecordObject(generator, "Generate Static Fish Net");

        if (generator.NetRoot != null)
        {
            Undo.DestroyObjectImmediate(generator.NetRoot.gameObject);
            generator.SetGeneratedRootForEditor(null);
        }

        if (disableRuntimeRebuild)
        {
            generator.generateOnStart = false;
        }

        generator.GenerateStaticNetForEditor(GetBlueprintAssetPath(generator));

        if (generator.NetRoot != null)
        {
            Undo.RegisterCreatedObjectUndo(generator.NetRoot.gameObject, "Generate Static Fish Net");
            Selection.activeObject = generator.NetRoot.gameObject;
        }

        MarkDirty(generator);
    }

    private static void ClearStaticNet(FishNetGenerator generator)
    {
        Undo.RecordObject(generator, "Clear Static Fish Net");

        if (generator.NetRoot != null)
        {
            Undo.DestroyObjectImmediate(generator.NetRoot.gameObject);
            generator.SetGeneratedRootForEditor(null);
        }

        generator.DeleteGeneratedBlueprintAsset();
        MarkDirty(generator);
    }

    private static void MarkDirty(FishNetGenerator generator)
    {
        EditorUtility.SetDirty(generator);
        if (generator.areaManager != null)
        {
            EditorUtility.SetDirty(generator.areaManager);
        }

        if (generator.NetRoot != null)
        {
            EditorUtility.SetDirty(generator.NetRoot.gameObject);
        }

        EditorSceneManager.MarkSceneDirty(generator.gameObject.scene);
    }

    private static string GetBlueprintAssetPath(FishNetGenerator generator)
    {
        string sceneName = generator.gameObject.scene.IsValid()
            ? generator.gameObject.scene.name
            : "UnsavedScene";
        string objectName = generator.gameObject.name;
        return $"Assets/FinsSimUnity/Generated/FishNet/{SanitizeFileName(sceneName)}_{SanitizeFileName(objectName)}_RopeBlueprints.asset";
    }

    private static string SanitizeFileName(string value)
    {
        if (string.IsNullOrEmpty(value))
        {
            return "Unnamed";
        }

        char[] chars = value.ToCharArray();
        for (int i = 0; i < chars.Length; i++)
        {
            char c = chars[i];
            bool valid = char.IsLetterOrDigit(c) || c == '_' || c == '-';
            if (!valid)
            {
                chars[i] = '_';
            }
        }

        return new string(chars);
    }
}
