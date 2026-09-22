using System;
using System.Collections.Generic;
using System.IO;
using Obi;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Policies;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.Rendering;
using UnityEngine.SceneManagement;

public static class ThreeChaseOneFinsRovSceneBuilder
{
    public const string SourceScenePath = "Assets/FinsSimUnity/Tasks/Chase/ThreeChaseOne/Scenes/3Chase1.unity";
    public const string TargetScenePath = "Assets/FinsSimUnity/Tasks/Chase/ThreeChaseOne/Scenes/3Chase1_new.unity";
    public const string HeadlessScenePath = "Assets/FinsSimUnity/Tasks/Chase/ThreeChaseOne/Scenes/3Chase1_new_headless.unity";
    public const string FinsRovPrefabPath = "Assets/Models/FinsROV/FinsROV.prefab";
    public const string NetBlueprintPath = "Assets/FinsSimUnity/Generated/FishNet/3Chase1_new_MLSceneManager_RopeBlueprints.asset";

    private const int ChaserObservationSize = ChaserAgent.VectorObservationSize;
    private const int ChaserActionSize = 8;
    // FinsROV_Fossen thrusters expire force requests after 0.2 s.  At the
    // 0.02 s physics cadence, refresh actions within 0.1 s so command delay
    // and scheduling cannot cause intermittent zero thrust.
    private const int DecisionPeriod = 5;
    private const float FinsRovActionForceScaleN = 50f;
    private const string LegacyWaterPrefabPath = "Assets/Samples/BaseSample/Prefabs/Water.prefab";
    private const string HdrpOceanPrefabPath = "Assets/Environments/Ocean.prefab";
    private const string SkyAndFogVolumeName = "Sky and Fog Global Volume";
    private static readonly string[] DefaultHdrpVolumeProfilePaths =
    {
        "Assets/FinsSimUnity/Content/PlatformHDRPDefaultResources/DefaultSettingsVolumeProfile.asset",
        "Assets/HDRPDefaultResources/DefaultSettingsVolumeProfile.asset",
    };

    private static readonly Dictionary<string, string> ReplacementNames = new Dictionary<string, string>
    {
        { "UUV7", "FinsROV7" },
        { "UUV8", "FinsROV8" },
        { "UUV9", "FinsROV9" },
    };

    public static void Create3Chase1NewScene()
    {
        if (Application.isBatchMode && AssetDatabase.LoadAssetAtPath<SceneAsset>(TargetScenePath) != null)
        {
            Debug.Log(
                "[ThreeChaseOneFinsRovSceneBuilder] Using existing 3Chase1_new scene. " +
                "Batch/null-gfx prefab instantiation is intentionally skipped for FinsROV; repairing existing scene instead.");
            Repair3Chase1NewScene();
            return;
        }

        Debug.Log("[ThreeChaseOneFinsRovSceneBuilder] Copying source scene.");
        CopySceneAsset(SourceScenePath, TargetScenePath);

        Debug.Log("[ThreeChaseOneFinsRovSceneBuilder] Opening target scene copy.");
        Scene scene = EditorSceneManager.OpenScene(TargetScenePath, OpenSceneMode.Single);
        Debug.Log("[ThreeChaseOneFinsRovSceneBuilder] Loading FinsROV prefab.");
        GameObject prefab = AssetDatabase.LoadAssetAtPath<GameObject>(FinsRovPrefabPath);
        if (prefab == null)
        {
            throw new FileNotFoundException($"FinsROV prefab not found: {FinsRovPrefabPath}");
        }

        Debug.Log("[ThreeChaseOneFinsRovSceneBuilder] Resolving original scene references.");
        CatchAreaManager areaManager = FindSceneObject<CatchAreaManager>(scene);
        FishNetGenerator fishNetGenerator = FindSceneObject<FishNetGenerator>(scene);
        PreyAgent prey = FindSceneObject<PreyAgent>(scene);
        if (areaManager == null || fishNetGenerator == null || prey == null)
        {
            throw new InvalidOperationException(
                $"3Chase1 scene is missing core references. areaManager={areaManager != null}, " +
                $"fishNetGenerator={fishNetGenerator != null}, prey={prey != null}");
        }

        GameObject oldUuv7 = FindSceneGameObject(scene, "UUV7");
        GameObject oldUuv8 = FindSceneGameObject(scene, "UUV8");
        GameObject oldUuv9 = FindSceneGameObject(scene, "UUV9");
        if (oldUuv7 == null || oldUuv8 == null || oldUuv9 == null)
        {
            throw new InvalidOperationException("Could not find UUV7/UUV8/UUV9 in source scene copy.");
        }

        Debug.Log("[ThreeChaseOneFinsRovSceneBuilder] Replacing UUV7 with FinsROV.");
        ChaserAgent newNetter1 = ReplaceChaserWithFinsRov(scene, prefab, oldUuv7, AgentRole.Netter, "Netter", 1);
        Debug.Log("[ThreeChaseOneFinsRovSceneBuilder] Replacing UUV8 with FinsROV.");
        ChaserAgent newNetter2 = ReplaceChaserWithFinsRov(scene, prefab, oldUuv8, AgentRole.Netter, "Netter", 2);
        Debug.Log("[ThreeChaseOneFinsRovSceneBuilder] Replacing UUV9 with FinsROV.");
        ChaserAgent newHerder = ReplaceChaserWithFinsRov(scene, prefab, oldUuv9, AgentRole.Herder, "Herder", 0);

        Debug.Log("[ThreeChaseOneFinsRovSceneBuilder] Rewiring agent and manager references.");
        newNetter1.partnerNetter = newNetter2;
        newNetter2.partnerNetter = newNetter1;
        newHerder.partnerNetter = null;

        ChaserAgent[] chasers = { newNetter1, newNetter2, newHerder };
        for (int i = 0; i < chasers.Length; i++)
        {
            chasers[i].areaManager = areaManager;
            chasers[i].fish = prey.transform;
            chasers[i].selfTransform = chasers[i].transform;
            EditorUtility.SetDirty(chasers[i]);
        }

        areaManager.netter1 = newNetter1;
        areaManager.netter2 = newNetter2;
        areaManager.herder = newHerder;
        areaManager.fish = prey;
        EnsureCurriculumController(areaManager);
        EditorUtility.SetDirty(areaManager);

        prey.chasers = new[] { newNetter1.transform, newNetter2.transform, newHerder.transform };
        prey.areaManager = areaManager;
        EditorUtility.SetDirty(prey);

        fishNetGenerator.areaManager = areaManager;
        fishNetGenerator.netter1 = newNetter1.transform;
        fishNetGenerator.netter2 = newNetter2.transform;
        fishNetGenerator.netter1MarkPosition = null;
        fishNetGenerator.netter2MarkPosition = null;
        fishNetGenerator.netter1LocalAttachmentOffset = Vector3.zero;
        fishNetGenerator.netter2LocalAttachmentOffset = Vector3.zero;
        fishNetGenerator.useDynamicUuvAttachments = true;
        fishNetGenerator.addMissingObiCollidersToNetters = true;
        fishNetGenerator.generateOnStart = false;
        SetHiddenString(fishNetGenerator, "generatedBlueprintAssetPath", NetBlueprintPath);
        EditorUtility.SetDirty(fishNetGenerator);

        Debug.Log("[ThreeChaseOneFinsRovSceneBuilder] Removing old UUV scene objects.");
        DestroySceneObject(oldUuv7);
        DestroySceneObject(oldUuv8);
        DestroySceneObject(oldUuv9);

        Debug.Log("[ThreeChaseOneFinsRovSceneBuilder] Keeping the static GeneratedFishNet and disabling runtime regeneration.");
        ConfigureStaticGeneratedNet(scene, areaManager, fishNetGenerator);
        EditorUtility.SetDirty(areaManager);

        Debug.Log("[ThreeChaseOneFinsRovSceneBuilder] Restoring default HDRP environment and cleanup.");
        RestoreDefaultHdrpEnvironment(scene);
        RemoveMissingScripts(scene);

        Debug.Log("[ThreeChaseOneFinsRovSceneBuilder] Saving scene.");
        EditorSceneManager.MarkSceneDirty(scene);
        if (!EditorSceneManager.SaveScene(scene))
        {
            throw new IOException($"Failed to save generated scene: {TargetScenePath}");
        }

        AssetDatabase.SaveAssets();
        AssetDatabase.Refresh();

        Debug.Log("[ThreeChaseOneFinsRovSceneBuilder] Created 3Chase1_new with FinsROV chasers.");
    }

    public static void Validate3Chase1NewScene()
    {
        Scene scene = EditorSceneManager.OpenScene(TargetScenePath, OpenSceneMode.Single);
        CatchAreaManager areaManager = FindSceneObject<CatchAreaManager>(scene);
        FishNetGenerator fishNetGenerator = FindSceneObject<FishNetGenerator>(scene);
        GameObject skyAndFog = FindSceneGameObject(scene, SkyAndFogVolumeName);
        VolumeProfile volumeProfile = LoadDefaultHdrpVolumeProfile();

        string status =
            $"areaManager={areaManager != null}, " +
            $"netter1={NameOrNull(areaManager != null ? areaManager.netter1 : null)}, " +
            $"netter2={NameOrNull(areaManager != null ? areaManager.netter2 : null)}, " +
            $"herder={NameOrNull(areaManager != null ? areaManager.herder : null)}, " +
            $"fishNetGenerator={fishNetGenerator != null}, " +
            $"netter1MarkPosition={(fishNetGenerator != null && fishNetGenerator.netter1MarkPosition != null ? fishNetGenerator.netter1MarkPosition.name : "null")}, " +
            $"netter2MarkPosition={(fishNetGenerator != null && fishNetGenerator.netter2MarkPosition != null ? fishNetGenerator.netter2MarkPosition.name : "null")}, " +
            $"skyAndFog={skyAndFog != null}, " +
            $"volumeProfile={(skyAndFog != null ? skyAndFog.GetComponent<Volume>()?.sharedProfile?.name ?? "null" : "null")}, " +
            $"ambientIntensity={RenderSettings.ambientIntensity:F2}, " +
            $"skybox={(RenderSettings.skybox != null ? RenderSettings.skybox.name : "null")}";

        Debug.Log($"[ThreeChaseOneFinsRovSceneBuilder] Validation: {status}");

        if (areaManager == null || areaManager.netter1 == null || areaManager.netter2 == null || areaManager.herder == null)
        {
            throw new InvalidOperationException("3Chase1_new validation failed: manager chaser references are incomplete.");
        }

        if (fishNetGenerator == null || fishNetGenerator.netter1MarkPosition != null || fishNetGenerator.netter2MarkPosition != null)
        {
            throw new InvalidOperationException("3Chase1_new validation failed: net generator still depends on MarkPosition.");
        }

        if (skyAndFog == null)
        {
            throw new InvalidOperationException("3Chase1_new validation failed: Sky and Fog Global Volume is missing.");
        }

        Volume volume = skyAndFog.GetComponent<Volume>();
        if (volume == null || volume.sharedProfile != volumeProfile)
        {
            throw new InvalidOperationException("3Chase1_new validation failed: Sky and Fog Global Volume is not using the default HDRP profile.");
        }

        if (!Mathf.Approximately(RenderSettings.ambientIntensity, 1f))
        {
            throw new InvalidOperationException("3Chase1_new validation failed: ambient intensity should match the default HDRP scene setup.");
        }
    }

    [MenuItem("FinsSim/3Chase1/Repair 3Chase1 New Scene")]
    public static void Repair3Chase1NewScene()
    {
        Scene scene = EditorSceneManager.OpenScene(TargetScenePath, OpenSceneMode.Single);
        int changed = Repair3Chase1NewScene(scene, keepVisuals: true);
        SaveSceneAndAssets(scene, TargetScenePath);
        Debug.Log($"[ThreeChaseOneFinsRovSceneBuilder] Repaired 3Chase1_new scene. changed={changed}");
    }

    public static void Prepare3Chase1NewHeadlessScene()
    {
        Prepare3Chase1NewHeadlessScene(TargetScenePath, HeadlessScenePath);
    }

    public static void Prepare3Chase1NewHeadlessScene(string sourceScenePath, string targetScenePath)
    {
        CopySceneAsset(sourceScenePath, targetScenePath);
        Scene scene = EditorSceneManager.OpenScene(targetScenePath, OpenSceneMode.Single);
        int changed = Repair3Chase1NewScene(scene, keepVisuals: false);
        changed += DisableSceneDisplayOnlyComponents(scene);
        RemoveMissingScripts(scene);
        SaveSceneAndAssets(scene, targetScenePath);
        Debug.Log(
            "[ThreeChaseOneFinsRovSceneBuilder] Prepared 3Chase1_new headless scene. " +
            $"scene={targetScenePath}, changed={changed}");
    }

    public static void Validate3Chase1NewSceneStatic()
    {
        string fullPath = Path.Combine(Application.dataPath, "../", TargetScenePath);
        if (!File.Exists(fullPath))
        {
            throw new FileNotFoundException($"3Chase1_new scene not found: {TargetScenePath}");
        }

        string sceneText = File.ReadAllText(fullPath);
        Require(sceneText.Contains("m_SourcePrefab: {fileID: 100100000, guid: f111b1f0dec340287aacce49033acf7f, type: 3}"),
            "missing FinsROV prefab instance");
        Require(CountOccurrences(sceneText, "m_SourcePrefab: {fileID: 100100000, guid: f111b1f0dec340287aacce49033acf7f, type: 3}") == 3,
            "expected exactly three FinsROV prefab instances");
        Require(CountOccurrences(sceneText, "useFinsRovThrusters: 1") == 3,
            "expected three ChaserAgent FinsROV thruster controls");
        Require(!sceneText.Contains("m_Name: UUV7") && !sceneText.Contains("m_Name: UUV8") && !sceneText.Contains("m_Name: UUV9"),
            "old UUV7/UUV8/UUV9 objects are still present");
        Require(sceneText.Contains("m_Name: GeneratedFishNet"),
            "static GeneratedFishNet is missing");
        Require(sceneText.Contains("netter1MarkPosition: {fileID: 0}") && sceneText.Contains("netter2MarkPosition: {fileID: 0}"),
            "FishNetGenerator still depends on MarkPosition");
        Require(sceneText.Contains("generateOnStart: 0"),
            "FishNetGenerator should reuse the static GeneratedFishNet instead of rebuilding at runtime");
        Require(sceneText.Contains($"m_Name: {SkyAndFogVolumeName}"),
            "HDRP Sky and Fog Global Volume is missing");
        Require(sceneText.Contains("sharedProfile: {fileID: 11400000, guid: 02450fb6644cf8a49a23a806ae437932, type: 2}"),
            "HDRP Sky and Fog Global Volume is not using the default volume profile");
        Require(!sceneText.Contains("guid: 780611a67e8e941a2b3aa96e5915a793"),
            "legacy Water.prefab instance is still present");
        Require(sceneText.Contains("guid: b98ef1c91752e98caa8bf3c05954c621"),
            "HDRP Ocean prefab instance is missing");
        Require(!sceneText.Contains("guid: b98ef1c91752e98caa8bf3c05954c621, type: 3}\n      propertyPath: m_IsActive\n      value: 0"),
            "HDRP Ocean prefab instance is disabled");

        Debug.Log("[ThreeChaseOneFinsRovSceneBuilder] Static validation passed for 3Chase1_new.");
    }

    private static ChaserAgent ReplaceChaserWithFinsRov(
        Scene scene,
        GameObject prefab,
        GameObject oldObject,
        AgentRole role,
        string behaviorName,
        int teamId)
    {
        Debug.Log($"[ThreeChaseOneFinsRovSceneBuilder] {oldObject.name}: capturing old chaser settings.");
        ChaserAgent oldChaser = oldObject.GetComponent<ChaserAgent>();
        string serializedChaser = oldChaser != null ? EditorJsonUtility.ToJson(oldChaser) : string.Empty;

        Transform oldTransform = oldObject.transform;
        Transform oldParent = oldTransform.parent;
        int siblingIndex = oldTransform.GetSiblingIndex();
        Vector3 localPosition = oldTransform.localPosition;
        Quaternion localRotation = oldTransform.localRotation;
        Vector3 localScale = oldTransform.localScale;

        Debug.Log($"[ThreeChaseOneFinsRovSceneBuilder] {oldObject.name}: instantiating FinsROV prefab.");
        GameObject instance = UnityEngine.Object.Instantiate(prefab);
        if (instance == null)
        {
            throw new InvalidOperationException($"Failed to instantiate {FinsRovPrefabPath}");
        }

        SceneManager.MoveGameObjectToScene(instance, scene);

        Debug.Log($"[ThreeChaseOneFinsRovSceneBuilder] {oldObject.name}: applying transform and tag.");
        instance.name = ReplacementNames.TryGetValue(oldObject.name, out string replacementName)
            ? replacementName
            : $"FinsROV_{oldObject.name}";

        instance.transform.SetParent(oldParent, false);
        instance.transform.SetSiblingIndex(siblingIndex);
        instance.transform.localPosition = localPosition;
        instance.transform.localRotation = localRotation;
        instance.transform.localScale = localScale;
        TryAssignTag(instance, oldObject.tag);

        Debug.Log($"[ThreeChaseOneFinsRovSceneBuilder] {oldObject.name}: removing single-agent components.");
        RemoveSingleAgentAndNetworkComponents(instance);
        Debug.Log($"[ThreeChaseOneFinsRovSceneBuilder] {oldObject.name}: ensuring net attachment collider.");
        EnsureFinsRovNetAttachmentCollider(instance);

        Debug.Log($"[ThreeChaseOneFinsRovSceneBuilder] {oldObject.name}: configuring ChaserAgent.");
        ChaserAgent chaser = instance.GetComponent<ChaserAgent>();
        if (chaser == null)
        {
            chaser = instance.AddComponent<ChaserAgent>();
        }

        if (!string.IsNullOrEmpty(serializedChaser))
        {
            EditorJsonUtility.FromJsonOverwrite(serializedChaser, chaser);
        }

        chaser.role = role;
        chaser.partnerNetter = null;
        chaser.selfTransform = instance.transform;
        ConfigureFinsRovChaserSerializedFields(chaser);
        Debug.Log($"[ThreeChaseOneFinsRovSceneBuilder] {oldObject.name}: configuring ML-Agents components.");
        ConfigureBehaviorParameters(instance, behaviorName, teamId);
        ConfigureDecisionRequester(instance);

        EditorUtility.SetDirty(instance);
        EditorUtility.SetDirty(chaser);
        return chaser;
    }

    private static void ConfigureFinsRovChaserSerializedFields(ChaserAgent chaser)
    {
        SerializedObject serializedObject = new SerializedObject(chaser);
        SetBool(serializedObject, "useFinsRovThrusters", true);
        SetFloat(serializedObject, "actionForceScaleN", FinsRovActionForceScaleN);
        SetBool(serializedObject, "autoResolveThrustersFromChildren", true);
        SetBool(serializedObject, "logThrusterResolution", true);
        SetBool(serializedObject, "allowUnifiedMaxThrustToOverrideFinsRovForceScale", false);
        serializedObject.ApplyModifiedPropertiesWithoutUndo();
    }

    private static void ConfigureBehaviorParameters(GameObject instance, string behaviorName, int teamId)
    {
        BehaviorParameters behavior = instance.GetComponent<BehaviorParameters>();
        if (behavior == null)
        {
            behavior = instance.AddComponent<BehaviorParameters>();
        }

        behavior.BehaviorName = behaviorName;
        behavior.TeamId = teamId;
        behavior.BehaviorType = BehaviorType.Default;
        behavior.BrainParameters.VectorObservationSize = ChaserObservationSize;
        behavior.BrainParameters.ActionSpec = ActionSpec.MakeContinuous(ChaserActionSize);
        behavior.UseChildSensors = false;
        behavior.UseChildActuators = true;
        EditorUtility.SetDirty(behavior);
    }

    private static void ConfigureDecisionRequester(GameObject instance)
    {
        DecisionRequester requester = instance.GetComponent<DecisionRequester>();
        if (requester == null)
        {
            requester = instance.AddComponent<DecisionRequester>();
        }

        requester.DecisionPeriod = DecisionPeriod;
        requester.TakeActionsBetweenDecisions = true;
        EditorUtility.SetDirty(requester);
    }

    private static void RemoveSingleAgentAndNetworkComponents(GameObject root)
    {
        DestroyComponentsInChildren<DecisionRequester>(root);
        RemoveTrainingOnlyFinsRovComponents(root);
        RemoveFinsRovChaserVisualOverhead(root);
    }

    private static int Repair3Chase1NewScene(Scene scene, bool keepVisuals)
    {
        int changed = 0;
        changed += RestoreDefaultHdrpEnvironment(scene);

        CatchAreaManager areaManager = FindSceneObject<CatchAreaManager>(scene);
        FishNetGenerator fishNetGenerator = FindSceneObject<FishNetGenerator>(scene);
        changed += EnsureCurriculumController(areaManager);
        if (fishNetGenerator != null)
        {
            changed += ConfigureStaticGeneratedNet(scene, areaManager, fishNetGenerator, keepVisuals);
        }

        foreach (string chaserName in ReplacementNames.Values)
        {
            GameObject chaser = FindSceneGameObject(scene, chaserName);
            if (chaser == null)
            {
                continue;
            }

            changed += RemoveTrainingOnlyFinsRovComponents(chaser);
            if (!keepVisuals)
            {
                changed += RemoveFinsRovChaserVisualOverhead(chaser);
            }
            ConfigureBehaviorParameters(chaser, chaserName == "FinsROV9" ? "Herder" : "Netter", chaserName == "FinsROV9" ? 0 : chaserName == "FinsROV7" ? 1 : 2);
            ConfigureDecisionRequester(chaser);
        }

        return changed;
    }

    private static int EnsureCurriculumController(CatchAreaManager areaManager)
    {
        if (areaManager == null)
        {
            return 0;
        }

        ThreeChaseOneCurriculumController controller =
            areaManager.GetComponent<ThreeChaseOneCurriculumController>();
        if (controller != null)
        {
            return 0;
        }

        controller = areaManager.gameObject.AddComponent<ThreeChaseOneCurriculumController>();
        EditorUtility.SetDirty(controller);
        EditorUtility.SetDirty(areaManager.gameObject);
        return 1;
    }

    private static int RestoreDefaultHdrpEnvironment(Scene scene)
    {
        int changed = 0;
        changed += RemoveLegacyWaterPrefabs(scene);
        changed += EnsureHdrpOceanActive(scene);
        changed += EnsureSkyAndFogGlobalVolume(scene);

        GameObject sun = FindSceneGameObject(scene, "Sun");
        if (sun != null)
        {
            Transform sunTransform = sun.transform;
            if (sunTransform.parent != null)
            {
                Vector3 localPosition = sunTransform.localPosition;
                if (localPosition != new Vector3(0f, 10f, 0f))
                {
                    sunTransform.localPosition = new Vector3(0f, 10f, 0f);
                    EditorUtility.SetDirty(sunTransform);
                    changed++;
                }
            }
        }

        if (!Mathf.Approximately(RenderSettings.ambientIntensity, 1f))
        {
            RenderSettings.ambientIntensity = 1f;
            changed++;
        }

        if (RenderSettings.skybox != null)
        {
            RenderSettings.skybox = null;
            changed++;
        }

        if (RenderSettings.fog)
        {
            RenderSettings.fog = false;
            changed++;
        }

        if (RenderSettings.ambientMode != AmbientMode.Skybox)
        {
            RenderSettings.ambientMode = AmbientMode.Skybox;
            changed++;
        }

        DynamicGI.UpdateEnvironment();
        return changed;
    }

    private static int EnsureSkyAndFogGlobalVolume(Scene scene)
    {
        int changed = 0;
        VolumeProfile volumeProfile = LoadDefaultHdrpVolumeProfile();
        GameObject environment = FindSceneGameObject(scene, "Environment");

        GameObject skyAndFog = FindSceneGameObject(scene, SkyAndFogVolumeName);
        if (skyAndFog == null)
        {
            skyAndFog = new GameObject(SkyAndFogVolumeName);
            changed++;
        }

        if (environment != null && skyAndFog.transform.parent != environment.transform)
        {
            skyAndFog.transform.SetParent(environment.transform, false);
            changed++;
        }

        if (skyAndFog.transform.localPosition != Vector3.zero)
        {
            skyAndFog.transform.localPosition = Vector3.zero;
            changed++;
        }

        if (skyAndFog.transform.localRotation != Quaternion.identity)
        {
            skyAndFog.transform.localRotation = Quaternion.identity;
            changed++;
        }

        if (skyAndFog.transform.localScale != Vector3.one)
        {
            skyAndFog.transform.localScale = Vector3.one;
            changed++;
        }

        Volume volume = skyAndFog.GetComponent<Volume>();
        if (volume == null)
        {
            volume = skyAndFog.AddComponent<Volume>();
            changed++;
        }

        if (!volume.isGlobal)
        {
            volume.isGlobal = true;
            changed++;
        }

        if (!Mathf.Approximately(volume.priority, 0f))
        {
            volume.priority = 0f;
            changed++;
        }

        if (!Mathf.Approximately(volume.blendDistance, 0f))
        {
            volume.blendDistance = 0f;
            changed++;
        }

        if (!Mathf.Approximately(volume.weight, 1f))
        {
            volume.weight = 1f;
            changed++;
        }

        if (volume.sharedProfile != volumeProfile)
        {
            volume.sharedProfile = volumeProfile;
            changed++;
        }

        if (!skyAndFog.activeSelf)
        {
            skyAndFog.SetActive(true);
            changed++;
        }

        if (changed > 0)
        {
            EditorUtility.SetDirty(skyAndFog);
            EditorUtility.SetDirty(volume);
        }

        return changed;
    }

    private static int RemoveLegacyWaterPrefabs(Scene scene)
    {
        int changed = 0;
        var removedRoots = new HashSet<GameObject>();
        foreach (GameObject rootObject in scene.GetRootGameObjects())
        {
            foreach (Transform transform in rootObject.GetComponentsInChildren<Transform>(true))
            {
                GameObject gameObject = transform.gameObject;
                string prefabPath = PrefabUtility.GetPrefabAssetPathOfNearestInstanceRoot(gameObject);
                if (!string.Equals(prefabPath, LegacyWaterPrefabPath, StringComparison.Ordinal))
                {
                    continue;
                }

                GameObject instanceRoot = PrefabUtility.GetNearestPrefabInstanceRoot(gameObject);
                if (instanceRoot == null || !removedRoots.Add(instanceRoot))
                {
                    continue;
                }

                UnityEngine.Object.DestroyImmediate(instanceRoot, true);
                changed++;
            }
        }

        return changed;
    }

    private static int EnsureHdrpOceanActive(Scene scene)
    {
        int changed = 0;
        foreach (GameObject rootObject in scene.GetRootGameObjects())
        {
            foreach (Transform transform in rootObject.GetComponentsInChildren<Transform>(true))
            {
                GameObject gameObject = transform.gameObject;
                string prefabPath = PrefabUtility.GetPrefabAssetPathOfNearestInstanceRoot(gameObject);
                if (!string.Equals(prefabPath, HdrpOceanPrefabPath, StringComparison.Ordinal))
                {
                    continue;
                }

                GameObject instanceRoot = PrefabUtility.GetNearestPrefabInstanceRoot(gameObject);
                if (instanceRoot == null)
                {
                    continue;
                }

                if (!instanceRoot.activeSelf)
                {
                    instanceRoot.SetActive(true);
                    EditorUtility.SetDirty(instanceRoot);
                    changed++;
                }

                Transform oceanTransform = instanceRoot.transform;
                if (oceanTransform.localPosition != Vector3.zero)
                {
                    oceanTransform.localPosition = Vector3.zero;
                    EditorUtility.SetDirty(oceanTransform);
                    changed++;
                }

                return changed;
            }
        }

        GameObject oceanPrefab = AssetDatabase.LoadAssetAtPath<GameObject>(HdrpOceanPrefabPath);
        if (oceanPrefab == null)
        {
            Debug.LogWarning($"[ThreeChaseOneFinsRovSceneBuilder] HDRP Ocean prefab not found: {HdrpOceanPrefabPath}");
            return changed;
        }

        GameObject environment = FindSceneGameObject(scene, "Environment");
        GameObject ocean = (GameObject)PrefabUtility.InstantiatePrefab(oceanPrefab, scene);
        ocean.name = "Ocean";
        ocean.transform.SetParent(environment != null ? environment.transform : null, false);
        ocean.transform.localPosition = Vector3.zero;
        ocean.transform.localRotation = Quaternion.identity;
        ocean.transform.localScale = Vector3.one;
        EditorUtility.SetDirty(ocean);
        changed++;
        return changed;
    }


    private static VolumeProfile LoadDefaultHdrpVolumeProfile()
    {
        foreach (string assetPath in DefaultHdrpVolumeProfilePaths)
        {
            VolumeProfile profile = AssetDatabase.LoadAssetAtPath<VolumeProfile>(assetPath);
            if (profile != null)
            {
                return profile;
            }
        }

        throw new FileNotFoundException(
            "Default HDRP volume profile not found. Checked: " +
            string.Join(", ", DefaultHdrpVolumeProfilePaths));
    }

    private static int ConfigureStaticGeneratedNet(
        Scene scene,
        CatchAreaManager areaManager,
        FishNetGenerator fishNetGenerator,
        bool keepVisuals = true)
    {
        int changed = 0;
        Transform generatedRoot = null;
        GameObject generatedRootObject = FindSceneGameObject(scene, "GeneratedFishNet");
        if (generatedRootObject != null)
        {
            generatedRoot = generatedRootObject.transform;
        }

        if (fishNetGenerator.generateOnStart)
        {
            fishNetGenerator.generateOnStart = false;
            changed++;
        }

        if (generatedRoot != null && fishNetGenerator.NetRoot != generatedRoot)
        {
            fishNetGenerator.SetGeneratedRootForEditor(generatedRoot);
            changed++;
        }

        if (generatedRootObject != null && generatedRootObject.activeSelf != keepVisuals)
        {
            generatedRootObject.SetActive(keepVisuals);
            EditorUtility.SetDirty(generatedRootObject);
            changed++;
        }

        if (areaManager != null && generatedRoot != null && areaManager.net != generatedRoot)
        {
            areaManager.net = generatedRoot;
            EditorUtility.SetDirty(areaManager);
            changed++;
        }

        EditorUtility.SetDirty(fishNetGenerator);
        return changed;
    }

    private static int RemoveTrainingOnlyFinsRovComponents(GameObject root)
    {
        int removed = 0;
        foreach (MonoBehaviour behaviour in root.GetComponentsInChildren<MonoBehaviour>(true))
        {
            if (behaviour == null || !ShouldRemoveFromFinsRovChaser(behaviour))
            {
                continue;
            }

            UnityEngine.Object.DestroyImmediate(behaviour, true);
            removed++;
        }

        return removed;
    }

    private static bool ShouldRemoveFromFinsRovChaser(MonoBehaviour behaviour)
    {
        string typeName = behaviour.GetType().Name;
        string fullName = behaviour.GetType().FullName ?? string.Empty;
        return typeName == nameof(ControlForPosition)
            || typeName == nameof(ControlForPosition_IncrementalReward)
            || typeName == "ControlForMovingTargetReward"
            || typeName == "ControlForVelocity_IncrementalReward"
            || typeName == "ControlForAcceleration_IncrementalReward"
            || typeName == "RLControlForVelocity"
            || typeName == "FinsROVResetRosBridge"
            || typeName == "FinsROVManualThrusterController"
            || typeName == "FinsROVPhysicsDiagnosticRuntimeProbe"
            || typeName == "DWP2DebugController"
            || typeName == "VelocityDebugDisplay"
            || typeName == "UuvDirectionalSpeedTester"
            || typeName == "NetSeparationStressTest"
            || fullName == "FinsSim.Networking.VehicleRosBridge"
            || fullName.StartsWith("FinsSim.Sensors.ROS.")
            || fullName.StartsWith("FinsSim.ROS.");
    }

    private static int RemoveFinsRovChaserVisualOverhead(GameObject root)
    {
        int changed = 0;
        changed += DisableComponentsInChildren<Camera>(root);
        changed += DisableComponentsInChildren<AudioListener>(root);
        changed += DisableComponentsInChildren<AudioSource>(root);
        changed += DisableComponentsInChildren<Light>(root);
        changed += DisableComponentsInChildren<ReflectionProbe>(root);
        changed += DisableComponentsInChildren<ParticleSystem>(root);
        changed += DestroyComponentsInChildren<ParticleSystemRenderer>(root);
        changed += DestroyComponentsInChildren<Volume>(root);

        foreach (Transform transform in root.GetComponentsInChildren<Transform>(true))
        {
            string objectName = transform.gameObject.name;
            if (objectName.Contains("DefaultWaterParticleSystem")
                || objectName == "Cameras"
                || objectName == "CameraDrag")
            {
                if (transform.gameObject.activeSelf)
                {
                    transform.gameObject.SetActive(false);
                    EditorUtility.SetDirty(transform.gameObject);
                    changed++;
                }
            }
        }

        return changed;
    }

    private static int DisableSceneDisplayOnlyComponents(Scene scene)
    {
        int changed = 0;
        foreach (Component component in FindSceneComponents<Component>(scene))
        {
            if (component == null)
            {
                continue;
            }

            if (component is Camera
                || component is AudioListener
                || component is AudioSource
                || component is Canvas
                || component is Light
                || component is ReflectionProbe
                || component is ParticleSystem
                || component is Renderer
                || IsDisplayOnlyBehaviour(component))
            {
                changed += DisableComponent(component);
            }
        }

        return changed;
    }

    private static bool IsDisplayOnlyBehaviour(Component component)
    {
        string fullName = component.GetType().FullName ?? string.Empty;
        return fullName == "NWH.Common.Cameras.CameraMouseDrag"
            || fullName == "NWH.Common.Cameras.CameraChanger"
            || fullName == "NWH.DWP2.UnderwaterFog"
            || fullName == "UnityEngine.Rendering.Volume"
            || fullName == "UnityEngine.Rendering.HighDefinition.HDAdditionalCameraData"
            || fullName == "UnityEngine.Rendering.HighDefinition.HDAdditionalLightData";
    }

    private static int DisableComponent(Component component)
    {
        if (component is ParticleSystem)
        {
            if (!component.gameObject.activeSelf)
            {
                return 0;
            }

            component.gameObject.SetActive(false);
            EditorUtility.SetDirty(component.gameObject);
            return 1;
        }

        if (component is Behaviour behaviour)
        {
            if (!behaviour.enabled)
            {
                return 0;
            }

            behaviour.enabled = false;
            EditorUtility.SetDirty(behaviour);
            return 1;
        }

        if (component is Renderer renderer)
        {
            if (!renderer.enabled)
            {
                return 0;
            }

            renderer.enabled = false;
            EditorUtility.SetDirty(renderer);
            return 1;
        }

        if (component is ReflectionProbe reflectionProbe)
        {
            if (!reflectionProbe.enabled)
            {
                return 0;
            }

            reflectionProbe.enabled = false;
            EditorUtility.SetDirty(reflectionProbe);
            return 1;
        }

        return 0;
    }

    private static int DestroyComponentsInChildren<T>(GameObject root) where T : Component
    {
        int removed = 0;
        foreach (T component in root.GetComponentsInChildren<T>(true))
        {
            if (component == null)
            {
                continue;
            }

            UnityEngine.Object.DestroyImmediate(component, true);
            removed++;
        }

        return removed;
    }

    private static int DisableComponentsInChildren<T>(GameObject root) where T : Component
    {
        int changed = 0;
        foreach (T component in root.GetComponentsInChildren<T>(true))
        {
            if (component == null)
            {
                continue;
            }

            changed += DisableComponent(component);
        }

        return changed;
    }

    private static void EnsureFinsRovNetAttachmentCollider(GameObject instance)
    {
        BoxCollider boxCollider = instance.GetComponent<BoxCollider>();
        if (boxCollider == null)
        {
            boxCollider = instance.AddComponent<BoxCollider>();
        }

        boxCollider.center = Vector3.zero;
        boxCollider.size = new Vector3(0.75f, 0.65f, 0.55f);
        boxCollider.isTrigger = true;

        ObiCollider obiCollider = instance.GetComponent<ObiCollider>();
        if (obiCollider == null)
        {
            obiCollider = instance.AddComponent<ObiCollider>();
        }

        EditorUtility.SetDirty(boxCollider);
        EditorUtility.SetDirty(obiCollider);
    }

    private static void CopySceneAsset(string sourceScenePath, string targetScenePath)
    {
        SceneAsset sourceScene = AssetDatabase.LoadAssetAtPath<SceneAsset>(sourceScenePath);
        if (sourceScene == null)
        {
            throw new FileNotFoundException($"Source scene not found: {sourceScenePath}");
        }

        if (AssetDatabase.LoadAssetAtPath<SceneAsset>(targetScenePath) != null && !AssetDatabase.DeleteAsset(targetScenePath))
        {
            throw new IOException($"Failed to delete existing generated scene: {targetScenePath}");
        }

        if (!AssetDatabase.CopyAsset(sourceScenePath, targetScenePath))
        {
            throw new IOException($"Failed to copy scene from {sourceScenePath} to {targetScenePath}");
        }

        AssetDatabase.ImportAsset(targetScenePath);
    }

    private static void RemoveMissingScripts(Scene scene)
    {
        foreach (GameObject gameObject in EnumerateSceneGameObjects(scene))
        {
            GameObjectUtility.RemoveMonoBehavioursWithMissingScript(gameObject);
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

    private static IEnumerable<T> FindSceneComponents<T>(Scene scene) where T : Component
    {
        foreach (T component in Resources.FindObjectsOfTypeAll<T>())
        {
            if (component != null && component.gameObject.scene == scene)
            {
                yield return component;
            }
        }
    }

    private static GameObject FindSceneGameObject(Scene scene, string objectName)
    {
        foreach (GameObject gameObject in EnumerateSceneGameObjects(scene))
        {
            if (gameObject.name == objectName)
            {
                return gameObject;
            }
        }

        return null;
    }

    private static T FindSceneObject<T>(Scene scene) where T : Component
    {
        foreach (T component in Resources.FindObjectsOfTypeAll<T>())
        {
            if (component != null && component.gameObject.scene == scene)
            {
                return component;
            }
        }

        return null;
    }

    private static void DestroySceneObject(GameObject gameObject)
    {
        if (gameObject != null)
        {
            UnityEngine.Object.DestroyImmediate(gameObject, true);
        }
    }

    private static void SaveSceneAndAssets(Scene scene, string scenePath)
    {
        EditorSceneManager.MarkSceneDirty(scene);
        if (!EditorSceneManager.SaveScene(scene))
        {
            throw new IOException($"Failed to save scene: {scenePath}");
        }

        AssetDatabase.SaveAssets();
        AssetDatabase.Refresh();
    }

    private static void SetHiddenString(UnityEngine.Object target, string fieldName, string value)
    {
        SerializedObject serializedObject = new SerializedObject(target);
        SerializedProperty property = serializedObject.FindProperty(fieldName);
        if (property != null)
        {
            property.stringValue = value;
            serializedObject.ApplyModifiedPropertiesWithoutUndo();
        }
    }

    private static void SetBool(SerializedObject serializedObject, string propertyName, bool value)
    {
        SerializedProperty property = serializedObject.FindProperty(propertyName);
        if (property != null)
        {
            property.boolValue = value;
        }
    }

    private static void SetFloat(SerializedObject serializedObject, string propertyName, float value)
    {
        SerializedProperty property = serializedObject.FindProperty(propertyName);
        if (property != null)
        {
            property.floatValue = value;
        }
    }

    private static void TryAssignTag(GameObject instance, string tagName)
    {
        try
        {
            instance.tag = tagName;
        }
        catch (UnityException)
        {
            Debug.LogWarning($"[ThreeChaseOneFinsRovSceneBuilder] Could not assign tag '{tagName}' to {instance.name}; keeping '{instance.tag}'.");
        }
    }

    private static string NameOrNull(Component component)
    {
        return component != null ? component.name : "null";
    }

    private static void Require(bool condition, string message)
    {
        if (!condition)
        {
            throw new InvalidOperationException($"3Chase1_new static validation failed: {message}");
        }
    }

    private static int CountOccurrences(string text, string pattern)
    {
        int count = 0;
        int index = 0;
        while ((index = text.IndexOf(pattern, index, StringComparison.Ordinal)) >= 0)
        {
            count++;
            index += pattern.Length;
        }

        return count;
    }
}
