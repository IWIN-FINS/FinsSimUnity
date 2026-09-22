using System;
using System.Collections.Generic;
using System.IO;
using System.Reflection;
using Unity.MLAgents.Policies;
using UnityEditor;
using UnityEditor.Build;
using UnityEditor.Build.Reporting;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.SceneManagement;

public static class RLChaseLinuxBuild
{
    public const string ThreeChaseOneSourceScenePath = "Assets/FinsSimUnity/Tasks/Chase/ThreeChaseOne/Scenes/3Chase1.unity";
    public const string ThreeChaseOneHeadlessScenePath = "Assets/FinsSimUnity/Generated/Headless/3Chase1_headless.unity";
    public const string TriNetCaptureSourceScenePath = "Assets/FinsSimUnity/Tasks/NetCapture/Scenes/TriNetCapture.unity";
    public const string TriNetCaptureHeadlessScenePath = "Assets/FinsSimUnity/Generated/Headless/TriNetCapture_headless.unity";

    public static void Build3Chase1HeadlessServer(string outputPath)
    {
        ValidateOutputPath(outputPath);
        // The authoritative source scene owns the replicated TrainingArea setup.
        // Prepare it in the build worktree before copying it to the generated
        // headless scene so builds cannot fall back to the archived single-area scene.
        ThreeChaseOneParallelSceneBuilder.EnsurePrepared();
        Prepare3Chase1HeadlessScene();
        BuildLinux(ThreeChaseOneHeadlessScenePath, outputPath, StandaloneBuildSubtarget.Server);
    }

    public static void BuildTriNetCaptureHeadlessServer(string outputPath)
    {
        ValidateOutputPath(outputPath);
        TriNetCaptureSceneBuilder.PrepareTriNetCaptureScene();
        PrepareTriNetCaptureHeadlessScene();
        BuildLinux(TriNetCaptureHeadlessScenePath, outputPath, StandaloneBuildSubtarget.Server);
    }

    private static void ValidateOutputPath(string outputPath)
    {
        if (string.IsNullOrWhiteSpace(outputPath))
        {
            throw new ArgumentException("A catalog-owned output path is required.", nameof(outputPath));
        }
    }

    public static void PrepareTriNetCaptureHeadlessScene()
    {
        CopySceneAsset(TriNetCaptureSourceScenePath, TriNetCaptureHeadlessScenePath);
        Scene scene = EditorSceneManager.OpenScene(TriNetCaptureHeadlessScenePath, OpenSceneMode.Single);
        int missingScriptsRemoved = RemoveMissingScripts(scene);
        int displayComponentsDisabled = DisableDisplayOnlyComponents(scene);
        int behaviorParametersConfigured = ConfigureBehaviorParameters(scene);
        // Do not call ConfigureTrainingControllers here: TriNetCapture has a
        // passive Target and deliberately disables the old chase baseline.
        EditorSceneManager.MarkSceneDirty(scene);
        if (!EditorSceneManager.SaveScene(scene))
        {
            throw new Exception($"Failed to save prepared scene: {TriNetCaptureHeadlessScenePath}");
        }
        AssetDatabase.SaveAssets();
        AssetDatabase.Refresh();
        Debug.Log(
            "Prepared TriNetCapture headless scene. " +
            $"missingScriptsRemoved={missingScriptsRemoved}, " +
            $"displayComponentsDisabled={displayComponentsDisabled}, " +
            $"behaviorParametersConfigured={behaviorParametersConfigured}");
    }

    public static void Prepare3Chase1HeadlessScene()
    {
        CopySceneAsset(ThreeChaseOneSourceScenePath, ThreeChaseOneHeadlessScenePath);

        Scene scene = EditorSceneManager.OpenScene(ThreeChaseOneHeadlessScenePath, OpenSceneMode.Single);
        int missingScriptsRemoved = RemoveMissingScripts(scene);
        int displayComponentsDisabled = DisableDisplayOnlyComponents(scene);
        int behaviorParametersConfigured = ConfigureBehaviorParameters(scene);
        int debugControllersConfigured = ConfigureTrainingControllers(scene);

        EditorSceneManager.MarkSceneDirty(scene);
        if (!EditorSceneManager.SaveScene(scene))
        {
            throw new Exception($"Failed to save prepared scene: {ThreeChaseOneHeadlessScenePath}");
        }

        AssetDatabase.SaveAssets();
        AssetDatabase.Refresh();

        Debug.Log(
            "Prepared 3Chase1 headless scene. " +
            $"missingScriptsRemoved={missingScriptsRemoved}, " +
            $"displayComponentsDisabled={displayComponentsDisabled}, " +
            $"behaviorParametersConfigured={behaviorParametersConfigured}, " +
            $"debugControllersConfigured={debugControllersConfigured}");
    }

    private static void BuildLinux(string scenePath, string outputPath, StandaloneBuildSubtarget subtarget)
    {
        string outputDirectory = Path.GetDirectoryName(outputPath);
        if (string.IsNullOrEmpty(outputDirectory))
        {
            throw new ArgumentException($"Invalid output path: {outputPath}");
        }

        if (Directory.Exists(outputDirectory))
        {
            Directory.Delete(outputDirectory, true);
        }
        Directory.CreateDirectory(outputDirectory);

        EditorUserBuildSettings.SwitchActiveBuildTarget(BuildTargetGroup.Standalone, BuildTarget.StandaloneLinux64);

        PlayerSettings.defaultScreenWidth = 1280;
        PlayerSettings.defaultScreenHeight = 720;
        PlayerSettings.defaultIsNativeResolution = false;
        PlayerSettings.fullScreenMode = FullScreenMode.Windowed;
        PlayerSettings.resizableWindow = true;

        var options = new BuildPlayerOptions
        {
            scenes = new[] { scenePath },
            locationPathName = outputPath,
            target = BuildTarget.StandaloneLinux64,
            subtarget = (int)subtarget,
            options = BuildOptions.None,
        };

        BuildReport report = BuildPipeline.BuildPlayer(options);
        BuildSummary summary = report.summary;

        Debug.Log(
            $"RLChase Linux build finished. result={summary.result}, " +
            $"totalSize={summary.totalSize}, totalTime={summary.totalTime}, output={outputPath}");

        if (summary.result != BuildResult.Succeeded)
        {
            throw new Exception($"RLChase Linux build failed: {summary.result}");
        }
    }

    private static void CopySceneAsset(string sourceScenePath, string targetScenePath)
    {
        EnsureAssetFolder(Path.GetDirectoryName(targetScenePath)?.Replace('\\', '/'));
        SceneAsset sourceScene = AssetDatabase.LoadAssetAtPath<SceneAsset>(sourceScenePath);
        if (sourceScene == null)
        {
            throw new FileNotFoundException($"Source scene not found: {sourceScenePath}");
        }

        if (AssetDatabase.LoadAssetAtPath<SceneAsset>(targetScenePath) != null)
        {
            if (!AssetDatabase.DeleteAsset(targetScenePath))
            {
                throw new IOException($"Failed to delete existing generated scene: {targetScenePath}");
            }
        }

        if (!AssetDatabase.CopyAsset(sourceScenePath, targetScenePath))
        {
            throw new IOException($"Failed to copy scene from {sourceScenePath} to {targetScenePath}");
        }

        AssetDatabase.ImportAsset(targetScenePath);
    }

    private static void EnsureAssetFolder(string assetFolder)
    {
        if (string.IsNullOrWhiteSpace(assetFolder) || AssetDatabase.IsValidFolder(assetFolder))
        {
            return;
        }

        string parent = Path.GetDirectoryName(assetFolder)?.Replace('\\', '/');
        EnsureAssetFolder(parent);
        AssetDatabase.CreateFolder(parent, Path.GetFileName(assetFolder));
    }

    private static void EnsureSceneAssetExists(string scenePath)
    {
        SceneAsset scene = AssetDatabase.LoadAssetAtPath<SceneAsset>(scenePath);
        if (scene == null)
        {
            throw new FileNotFoundException($"Scene not found: {scenePath}");
        }
    }

    private static int RemoveMissingScripts(Scene scene)
    {
        int removed = 0;
        foreach (GameObject gameObject in EnumerateSceneGameObjects(scene))
        {
            removed += GameObjectUtility.RemoveMonoBehavioursWithMissingScript(gameObject);
        }
        return removed;
    }

    private static int DisableDisplayOnlyComponents(Scene scene)
    {
        int changed = 0;

        changed += DisableSceneBehaviours<Camera>(scene);
        changed += DisableSceneBehaviours<AudioListener>(scene);
        changed += DisableSceneBehaviours<AudioSource>(scene);
        changed += DisableSceneBehaviours<Canvas>(scene);
        changed += DisableSceneBehaviours<EventSystem>(scene);
        changed += DisableSceneBehaviours<BaseInputModule>(scene);

        foreach (MonoBehaviour behaviour in FindSceneObjects<MonoBehaviour>(scene))
        {
            if (behaviour == null)
            {
                continue;
            }

            string typeName = behaviour.GetType().Name;
            if (typeName == nameof(FreeCameraDragController)
                || typeName == nameof(UuvDirectionalSpeedTester)
                || typeName == nameof(NetSeparationStressTest))
            {
                if (behaviour.enabled)
                {
                    behaviour.enabled = false;
                    EditorUtility.SetDirty(behaviour);
                    changed++;
                }
            }
        }

        foreach (CatchAreaManager manager in FindSceneObjects<CatchAreaManager>(scene))
        {
            if (manager.showRewardDebug)
            {
                manager.showRewardDebug = false;
                EditorUtility.SetDirty(manager);
                changed++;
            }
        }

        return changed;
    }

    private static int ConfigureBehaviorParameters(Scene scene)
    {
        int changed = 0;
        foreach (BehaviorParameters behavior in FindSceneObjects<BehaviorParameters>(scene))
        {
            if (behavior.BehaviorType != BehaviorType.Default)
            {
                behavior.BehaviorType = BehaviorType.Default;
                EditorUtility.SetDirty(behavior);
                changed++;
            }
        }
        return changed;
    }

    private static int ConfigureTrainingControllers(Scene scene)
    {
        int changed = 0;
        FieldInfo defaultsInitializedField = typeof(ThreeChaseOneBaselineController).GetField(
            "baselineRoleControlDefaultsInitialized",
            BindingFlags.Instance | BindingFlags.NonPublic);

        foreach (ThreeChaseOneBaselineController controller in FindSceneObjects<ThreeChaseOneBaselineController>(scene))
        {
            controller.baselineEnabled = true;
            controller.baselineControlsChasers = false;
            controller.baselineControlsPrey = true;
            if (defaultsInitializedField != null)
            {
                defaultsInitializedField.SetValue(controller, true);
            }
            EditorUtility.SetDirty(controller);
            changed++;
        }

        return changed;
    }

    private static int DisableSceneBehaviours<T>(Scene scene) where T : Behaviour
    {
        int changed = 0;
        foreach (T behaviour in FindSceneObjects<T>(scene))
        {
            if (behaviour == null || !behaviour.enabled)
            {
                continue;
            }

            behaviour.enabled = false;
            EditorUtility.SetDirty(behaviour);
            changed++;
        }
        return changed;
    }

    private static IEnumerable<T> FindSceneObjects<T>(Scene scene) where T : UnityEngine.Object
    {
        foreach (T obj in Resources.FindObjectsOfTypeAll<T>())
        {
            if (obj == null)
            {
                continue;
            }

            if (obj is Component component)
            {
                if (component.gameObject.scene == scene)
                {
                    yield return obj;
                }
                continue;
            }

            if (obj is GameObject gameObject && gameObject.scene == scene)
            {
                yield return obj;
            }
        }
    }

    private static IEnumerable<GameObject> EnumerateSceneGameObjects(Scene scene)
    {
        foreach (GameObject root in scene.GetRootGameObjects())
        {
            foreach (Transform transform in root.GetComponentsInChildren<Transform>(true))
            {
                yield return transform.gameObject;
            }
        }
    }
}
