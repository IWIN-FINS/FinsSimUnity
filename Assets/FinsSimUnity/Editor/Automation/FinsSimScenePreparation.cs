using FinsSim.Hydrodynamics;
using NWH.DWP2.WaterData;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

public static class FinsSimScenePreparation
{
    public const string ControlForPositionScenePath = "Assets/FinsSimUnity/Tasks/PoseControl/Scenes/ControlForPosition_new.unity";
    public const string ControlForPositionSmoothNearTargetScenePath = "Assets/FinsSimUnity/Tasks/PoseControl/Scenes/ControlForPosition_smooth_near_target.unity";
    public const string ControlForPositionSmoothNearTargetManualScenePath = "Assets/FinsSimUnity/Tasks/PoseControl/Scenes/ControlForPosition_smooth_near_target_manual.unity";
    public const string ControlForPositionDynamicScenePath = "Assets/FinsSimUnity/Tasks/PoseControl/Scenes/ControlForPosition_Dynamic.unity";
    public const string ControlForPositionAngularStabilityScenePath = "Assets/FinsSimUnity/Tasks/PoseControl/Scenes/ControlForPosition_AngularStability.unity";
    public const string WaterDomainRandomizationProfilePath =
        "Assets/FinsSimUnity/Generated/DomainRandomization/FinsROVWaterDomainRandomization.asset";
    static string activePreparationScenePath = ControlForPositionScenePath;

    public static void PrepareControlForPositionFlatWater()
    {
        PrepareControlForPositionFlatWater(ControlForPositionScenePath, false);
    }

    public static void PrepareControlForPositionSmoothNearTargetFlatWater()
    {
        if (AssetDatabase.LoadAssetAtPath<SceneAsset>(ControlForPositionSmoothNearTargetScenePath))
        {
            AssetDatabase.DeleteAsset(ControlForPositionSmoothNearTargetScenePath);
        }

        AssetDatabase.CopyAsset(ControlForPositionScenePath, ControlForPositionSmoothNearTargetScenePath);
        AssetDatabase.Refresh();
        PrepareControlForPositionFlatWater(ControlForPositionSmoothNearTargetScenePath, true);
    }

    public static void PrepareControlForPositionSmoothNearTargetManualFlatWater()
    {
        PrepareControlForPositionFlatWater(ControlForPositionSmoothNearTargetManualScenePath, true);
    }

    public static void PrepareControlForPositionDynamicFlatWater()
    {
        PrepareControlForPositionFlatWater(ControlForPositionDynamicScenePath, false);
    }

    public static void PrepareControlForPositionCustomFlatWater(string scenePath)
    {
        PrepareControlForPositionFlatWater(scenePath, false, true, true);
    }

    public static void PrepareControlForPositionCustomFlatWaterVisual(string scenePath)
    {
        PrepareControlForPositionFlatWater(scenePath, false, false, true);
    }

    public static void PrepareControlForPositionDomainRandomizedWater()
    {
        PrepareControlForPositionWater(ControlForPositionScenePath, false, true, true, true);
    }

    public static void PrepareControlForPositionSmoothNearTargetDomainRandomizedWater()
    {
        if (AssetDatabase.LoadAssetAtPath<SceneAsset>(ControlForPositionSmoothNearTargetScenePath))
        {
            AssetDatabase.DeleteAsset(ControlForPositionSmoothNearTargetScenePath);
        }

        AssetDatabase.CopyAsset(ControlForPositionScenePath, ControlForPositionSmoothNearTargetScenePath);
        AssetDatabase.Refresh();
        PrepareControlForPositionWater(ControlForPositionSmoothNearTargetScenePath, true, true, true, true);
    }

    public static void PrepareControlForPositionSmoothNearTargetManualDomainRandomizedWater()
    {
        PrepareControlForPositionWater(ControlForPositionSmoothNearTargetManualScenePath, true, true, true, true);
    }

    static void PrepareControlForPositionFlatWater(string scenePath, bool useSmoothNearTargetReward)
    {
        PrepareControlForPositionFlatWater(scenePath, useSmoothNearTargetReward, true, true);
    }

    static void PrepareControlForPositionFlatWater(
        string scenePath,
        bool useSmoothNearTargetReward,
        bool disableTrainingOnlyGraphics,
        bool disableMlAgentsChildSensors)
    {
        PrepareControlForPositionWater(
            scenePath,
            useSmoothNearTargetReward,
            false,
            disableTrainingOnlyGraphics,
            disableMlAgentsChildSensors);
    }

    static void PrepareControlForPositionWater(
        string scenePath,
        bool useSmoothNearTargetReward,
        bool enableWaterDomainRandomization,
        bool disableTrainingOnlyGraphics = true,
        bool disableMlAgentsChildSensors = false)
    {
        activePreparationScenePath = scenePath;
        var scene = EditorSceneManager.OpenScene(scenePath);
        GameObject environment = FindSceneObject("Environment");
        GameObject ocean = FindSceneObject("Ocean");

        if (ocean != null)
        {
            ocean.SetActive(false);
            EditorUtility.SetDirty(ocean);
        }

        GameObject flatWaterProvider = FindSceneObject("FlatWaterProvider");
        if (flatWaterProvider == null)
        {
            flatWaterProvider = new GameObject("FlatWaterProvider");
            Undo.RegisterCreatedObjectUndo(flatWaterProvider, "Create flat water provider");
        }

        if (environment != null && flatWaterProvider.transform.parent != environment.transform)
        {
            flatWaterProvider.transform.SetParent(environment.transform, false);
        }

        flatWaterProvider.transform.localPosition = Vector3.zero;
        flatWaterProvider.transform.localRotation = Quaternion.identity;
        flatWaterProvider.transform.localScale = Vector3.one;
        flatWaterProvider.SetActive(true);

        PhysicalWaveWaterDataProvider provider =
            flatWaterProvider.GetComponent<PhysicalWaveWaterDataProvider>();
        if (provider == null)
        {
            provider = flatWaterProvider.AddComponent<PhysicalWaveWaterDataProvider>();
        }

        if (enableWaterDomainRandomization)
        {
            ConfigureDomainRandomizedWater(environment, flatWaterProvider, provider);
        }
        else
        {
            ConfigureFlatWaterProvider(environment, provider);
        }

        EditorUtility.SetDirty(flatWaterProvider);
        EditorUtility.SetDirty(provider);

        RemoveMissingScriptsFromOpenScene();
        if (useSmoothNearTargetReward)
        {
            ReplacePositionRewardAgentWithSmoothNearTargetReward();
        }

        RemoveTrainingOnlyNetworkComponents();
        if (disableTrainingOnlyGraphics)
        {
            DisableTrainingOnlyGraphics();
        }
        if (disableMlAgentsChildSensors)
        {
            DisableMlAgentsChildSensors();
        }

        EditorSceneManager.MarkSceneDirty(scene);
        EditorSceneManager.SaveScene(scene);
    }

    static void ConfigureFlatWaterProvider(GameObject environment, PhysicalWaveWaterDataProvider provider)
    {
        provider.mode = PhysicalWaveWaterDataProvider.WavePhysicsMode.FlatFallback;
        provider.fallbackWaterHeight = 0f;
        provider.stillWaterHeight = 0f;
        provider.targetSurfaceOverride = null;
        provider.steadyCurrent = Vector3.zero;
        provider.waves.Clear();
        DisableWaterDomainRandomization(environment, provider);
    }

    static void ConfigureDomainRandomizedWater(
        GameObject environment,
        GameObject flatWaterProvider,
        PhysicalWaveWaterDataProvider provider)
    {
        provider.mode = PhysicalWaveWaterDataProvider.WavePhysicsMode.AnalyticPhysicalWave;
        provider.fallbackWaterHeight = 0f;
        provider.stillWaterHeight = 0f;
        provider.targetSurfaceOverride = null;
        provider.includeWaveNormals = true;
        provider.includeWaveFlow = true;
        provider.includeVerticalOrbitalFlow = true;
        provider.randomizeWaterCurrentFromProfile = true;
        provider.randomizeWavesFromProfile = true;
        provider.forceAnalyticModeWhenRandomizingWaves = true;

        if (provider.waves.Count == 0)
        {
            provider.waves.Add(new PhysicalWaveWaterDataProvider.WaveComponent
            {
                amplitude = 0.03f,
                wavelength = 8f,
                period = 5f,
                directionDeg = 0f,
                phaseRad = 0f,
            });
            provider.waves.Add(new PhysicalWaveWaterDataProvider.WaveComponent
            {
                amplitude = 0.015f,
                wavelength = 3.5f,
                period = 2.8f,
                directionDeg = 55f,
                phaseRad = 1.7f,
            });
        }

        GameObject coordinatorHost = environment != null ? environment : flatWaterProvider;
        DomainRandomizationCoordinator coordinator =
            coordinatorHost.GetComponent<DomainRandomizationCoordinator>();
        if (coordinator == null)
        {
            coordinator = coordinatorHost.AddComponent<DomainRandomizationCoordinator>();
        }

        coordinator.enabled = true;
        coordinator.profile = LoadOrCreateWaterDomainRandomizationProfile();
        coordinator.mode = DomainRandomizationMode.Train;
        coordinator.baseSeed = 12345;
        coordinator.incrementSeedPerEpisode = true;
        coordinator.randomizeOnStart = false;
        coordinator.autoFindTargets = false;
        coordinator.targetBehaviours.Clear();
        coordinator.targetBehaviours.Add(provider);
        EditorUtility.SetDirty(coordinatorHost);
        EditorUtility.SetDirty(coordinator);
    }

    static void DisableWaterDomainRandomization(GameObject environment, PhysicalWaveWaterDataProvider provider)
    {
        GameObject coordinatorHost = environment != null ? environment : provider.gameObject;
        DomainRandomizationCoordinator coordinator =
            coordinatorHost.GetComponent<DomainRandomizationCoordinator>();
        if (coordinator == null)
        {
            return;
        }

        coordinator.targetBehaviours.Remove(provider);
        DomainRandomizationProfile waterProfile =
            AssetDatabase.LoadAssetAtPath<DomainRandomizationProfile>(WaterDomainRandomizationProfilePath);
        if (coordinator.profile == waterProfile && coordinator.targetBehaviours.Count == 0)
        {
            coordinator.profile = null;
            coordinator.enabled = false;
        }

        EditorUtility.SetDirty(coordinator);
    }

    static DomainRandomizationProfile LoadOrCreateWaterDomainRandomizationProfile()
    {
        EnsureGeneratedDomainRandomizationFolder();
        DomainRandomizationProfile profile =
            AssetDatabase.LoadAssetAtPath<DomainRandomizationProfile>(WaterDomainRandomizationProfilePath);
        if (profile == null)
        {
            profile = ScriptableObject.CreateInstance<DomainRandomizationProfile>();
            AssetDatabase.CreateAsset(profile, WaterDomainRandomizationProfilePath);
        }

        ConfigureDefaultWaterDomainRandomizationProfile(profile);
        EditorUtility.SetDirty(profile);
        AssetDatabase.SaveAssets();
        return profile;
    }

    static void EnsureGeneratedDomainRandomizationFolder()
    {
        if (!AssetDatabase.IsValidFolder("Assets/Generated"))
        {
            AssetDatabase.CreateFolder("Assets", "Generated");
        }

        if (!AssetDatabase.IsValidFolder("Assets/FinsSimUnity/Generated/DomainRandomization"))
        {
            AssetDatabase.CreateFolder("Assets/Generated", "DomainRandomization");
        }
    }

    static void ConfigureDefaultWaterDomainRandomizationProfile(DomainRandomizationProfile profile)
    {
        profile.randomizeBody = false;
        profile.randomizeHydrodynamics = false;
        profile.randomizeThrusters = false;
        profile.randomizeWater = true;
        profile.currentSpeed = new FloatRange(0f, 0.35f);
        profile.allowVerticalCurrent = false;
        profile.meanCurrent = Vector3.zero;
        profile.maxAngleFromMeanDegrees = 180f;
        profile.randomizeWaves = true;
        profile.waveComponentCount = new IntRange(1, 3);
        profile.waveAmplitude = new FloatRange(0f, 0.05f);
        profile.waveWavelength = new FloatRange(2.5f, 12f);
        profile.wavePeriod = new FloatRange(2f, 7f);
        profile.waveDirectionDeg = new FloatRange(0f, 360f);
        profile.randomizeWavePhase = true;
        profile.waveFlowScale = new FloatRange(0.5f, 1.2f);
        profile.waveMaxFlowSpeed = new FloatRange(0.1f, 0.35f);
    }

    public static void ConfigureDwp2RandomizationCoordinatorForScene(string scenePath)
    {
        activePreparationScenePath = scenePath;
        var scene = EditorSceneManager.OpenScene(scenePath);
        var randomizationTargets = new System.Collections.Generic.List<MonoBehaviour>();

        foreach (MonoBehaviour behaviour in Resources.FindObjectsOfTypeAll<MonoBehaviour>())
        {
            if (behaviour == null || !IsSceneObject(behaviour.gameObject))
            {
                continue;
            }

            string typeName = behaviour.GetType().Name;
            if (typeName == nameof(Dwp2HydrodynamicForceCoefficientRandomizer)
                || typeName == nameof(Dwp2MassBuoyancyRandomizer))
            {
                randomizationTargets.Add(behaviour);
            }
        }

        if (randomizationTargets.Count == 0)
        {
            return;
        }

        GameObject environment = FindSceneObject("Environment");
        GameObject coordinatorHost = environment != null
            ? environment
            : randomizationTargets[0].gameObject;
        DomainRandomizationCoordinator coordinator =
            coordinatorHost.GetComponent<DomainRandomizationCoordinator>();
        if (coordinator == null)
        {
            coordinator = coordinatorHost.AddComponent<DomainRandomizationCoordinator>();
        }

        coordinator.enabled = true;
        coordinator.profile = LoadOrCreateWaterDomainRandomizationProfile();
        coordinator.mode = DomainRandomizationMode.Train;
        coordinator.baseSeed = 12345;
        coordinator.incrementSeedPerEpisode = true;
        coordinator.randomizeOnStart = false;
        coordinator.autoFindTargets = false;
        coordinator.targetBehaviours.Clear();
        coordinator.targetBehaviours.AddRange(randomizationTargets);

        EditorUtility.SetDirty(coordinatorHost);
        EditorUtility.SetDirty(coordinator);
        EditorSceneManager.MarkSceneDirty(scene);
        EditorSceneManager.SaveScene(scene);
    }

    static void ReplacePositionRewardAgentWithSmoothNearTargetReward()
    {
        foreach (ControlForPosition_IncrementalReward oldAgent in Resources.FindObjectsOfTypeAll<ControlForPosition_IncrementalReward>())
        {
            if (oldAgent == null || !IsSceneObject(oldAgent.gameObject))
            {
                continue;
            }

            GameObject agentObject = oldAgent.gameObject;
            Transform selfTransform = oldAgent.selfTransform;
            Transform targetTransform = oldAgent.targetTransform;
            string serializedAgent = EditorJsonUtility.ToJson(oldAgent);
            Object.DestroyImmediate(oldAgent, true);

            ControlForPosition_SmoothNearTargetReward smoothAgent =
                agentObject.GetComponent<ControlForPosition_SmoothNearTargetReward>();
            if (smoothAgent == null)
            {
                smoothAgent = agentObject.AddComponent<ControlForPosition_SmoothNearTargetReward>();
            }

            EditorJsonUtility.FromJsonOverwrite(serializedAgent, smoothAgent);
            smoothAgent.selfTransform = selfTransform;
            smoothAgent.targetTransform = targetTransform;
            EditorUtility.SetDirty(agentObject);
            Debug.Log($"[FinsSimScenePreparation] Replaced ControlForPosition_IncrementalReward with ControlForPosition_SmoothNearTargetReward on {agentObject.name}");
        }
    }

    static void RemoveMissingScriptsFromOpenScene()
    {
        foreach (GameObject gameObject in Resources.FindObjectsOfTypeAll<GameObject>())
        {
            if (!IsSceneObject(gameObject))
            {
                continue;
            }

            int removed = GameObjectUtility.RemoveMonoBehavioursWithMissingScript(gameObject);
            if (removed > 0)
            {
                Debug.Log($"[FinsSimScenePreparation] Removed {removed} missing script(s) from {gameObject.name}");
                EditorUtility.SetDirty(gameObject);
            }
        }
    }

    static void RemoveTrainingOnlyNetworkComponents()
    {
        foreach (Component component in Resources.FindObjectsOfTypeAll<Component>())
        {
            if (component == null || !IsSceneObject(component.gameObject))
            {
                continue;
            }

            if (!IsTrainingOnlyNetworkComponent(component))
            {
                continue;
            }

            string componentName = component.GetType().Name;
            string objectName = component.gameObject.name;
            Object.DestroyImmediate(component, true);
            Debug.Log($"[FinsSimScenePreparation] Removed {componentName} from {objectName}");
        }
    }

    static bool IsTrainingOnlyNetworkComponent(Component component)
    {
        string typeName = component.GetType().FullName;
        if (string.IsNullOrEmpty(typeName))
        {
            return false;
        }

        return typeName == "FinsSim.Networking.VehicleRosBridge"
            || typeName == "FinsROVResetRosBridge"
            || typeName == "FinsROVManualThrusterController"
            || typeName == "DWP2DebugController"
            || typeName == "VelocityDebugDisplay"
            || typeName == "NWH.Common.Demo.DragObject"
            || typeName == "NWH.DWP2.WaterObjects.WaterParticleSystem"
            || typeName.StartsWith("FinsSim.Sensors.ROS.")
            || typeName.StartsWith("FinsSim.ROS.");
    }

    static void DisableTrainingOnlyGraphics()
    {
        foreach (Component component in Resources.FindObjectsOfTypeAll<Component>())
        {
            if (component == null || !IsSceneObject(component.gameObject))
            {
                continue;
            }

            if (component is Camera
                || component is AudioListener
                || component is Light
                || component is ReflectionProbe
                || component is Renderer
                || IsVisualOnlyBehaviour(component))
            {
                DisableComponent(component);
            }

            if (IsVisualOnlyGameObjectComponent(component))
            {
                SetInactive(component.gameObject);
            }
        }

        GameObject skyAndFog = FindSceneObject("Sky and Fog Global Volume");
        if (skyAndFog != null)
        {
            SetInactive(skyAndFog);
        }
    }

    static bool IsVisualOnlyBehaviour(Component component)
    {
        string typeName = component.GetType().FullName;
        return typeName == "NWH.Common.Cameras.CameraMouseDrag"
            || typeName == "NWH.Common.Cameras.CameraChanger"
            || typeName == "NWH.DWP2.UnderwaterFog"
            || typeName == "UnityEngine.Rendering.Volume"
            || typeName == "UnityEngine.Rendering.HighDefinition.HDAdditionalCameraData"
            || typeName == "UnityEngine.Rendering.HighDefinition.HDAdditionalLightData";
    }

    static bool IsVisualOnlyGameObjectComponent(Component component)
    {
        string typeName = component.GetType().FullName;
        return component is Camera
            || component is AudioListener
            || typeName == "NWH.Common.Cameras.CameraMouseDrag"
            || typeName == "NWH.Common.Cameras.CameraChanger"
            || typeName == "NWH.DWP2.UnderwaterFog"
            || typeName == "UnityEngine.Rendering.Volume"
            || typeName == "UnityEngine.Rendering.HighDefinition.HDAdditionalCameraData";
    }

    static void DisableMlAgentsChildSensors()
    {
        foreach (Component component in Resources.FindObjectsOfTypeAll<Component>())
        {
            if (component == null || !IsSceneObject(component.gameObject))
            {
                continue;
            }

            if (component.GetType().FullName != "Unity.MLAgents.Policies.BehaviorParameters")
            {
                continue;
            }

            SerializedObject serializedObject = new SerializedObject(component);
            SerializedProperty useChildSensors = serializedObject.FindProperty("m_UseChildSensors");
            if (useChildSensors == null || !useChildSensors.boolValue)
            {
                continue;
            }

            useChildSensors.boolValue = false;
            serializedObject.ApplyModifiedPropertiesWithoutUndo();
            EditorUtility.SetDirty(component);
            Debug.Log($"[FinsSimScenePreparation] Disabled child sensors on {component.gameObject.name}");
        }
    }

    static void DisableComponent(Component component)
    {
        SerializedObject serializedObject = new SerializedObject(component);
        SerializedProperty enabledProperty = serializedObject.FindProperty("m_Enabled");
        if (enabledProperty == null || !enabledProperty.boolValue)
        {
            return;
        }

        enabledProperty.boolValue = false;
        serializedObject.ApplyModifiedPropertiesWithoutUndo();
        EditorUtility.SetDirty(component);
        Debug.Log($"[FinsSimScenePreparation] Disabled {component.GetType().Name} on {component.gameObject.name}");
    }

    static void SetInactive(GameObject gameObject)
    {
        if (!gameObject.activeSelf)
        {
            return;
        }

        gameObject.SetActive(false);
        EditorUtility.SetDirty(gameObject);
        Debug.Log($"[FinsSimScenePreparation] Set inactive {gameObject.name}");
    }

    static GameObject FindSceneObject(string objectName)
    {
        foreach (GameObject gameObject in Resources.FindObjectsOfTypeAll<GameObject>())
        {
            if (gameObject.name != objectName)
            {
                continue;
            }

            if (!gameObject.scene.IsValid())
            {
                continue;
            }

            return gameObject;
        }

        return null;
    }

    static bool IsSceneObject(GameObject gameObject)
    {
        Scene scene = gameObject.scene;
        return scene.IsValid() && scene.path == activePreparationScenePath;
    }
}
