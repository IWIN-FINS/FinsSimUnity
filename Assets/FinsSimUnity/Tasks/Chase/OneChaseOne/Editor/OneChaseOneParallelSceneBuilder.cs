using System;
using FinsSim.Hydrodynamics;
using FinsSim.Actuators;
using NWH.DWP2.WaterData;
using Unity.MLAgents.Areas;
using Unity.MLAgents.Policies;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

/// <summary>Converts the 1Chase1 scene into an isolated replicated training area.</summary>
public static class OneChaseOneParallelSceneBuilder
{
    public const string ScenePath = "Assets/FinsSimUnity/Tasks/Chase/OneChaseOne/Scenes/1Chase1.unity";
    public const string DomainRandomizationProfilePath =
        "Assets/Models/FinsROV/Hydrodynamics/DomainRandomizationProfile_HoldForPosition.asset";
    public const string AreaName = "OneChaseOneTrainingArea";
    public const string ReplicatorName = "__OneChaseOneTrainingAreaReplicator";
    public const float AreaSeparation = 32f;
    public const int InspectorDefaultAreaCount = 2048;

    [MenuItem("FinsSim/RL/Prepare 1Chase1 Parallel Scene")]
    public static void PrepareOneChaseOneParallelScene()
    {
        Scene scene = EditorSceneManager.OpenScene(ScenePath, OpenSceneMode.Single);
        TrainingAreaReplicator existing = UnityEngine.Object.FindFirstObjectByType<TrainingAreaReplicator>();
        if (existing == null)
        {
            CreateParallelArea(scene);
        }
        else
        {
            NormalizeExistingParallelArea(existing);
        }

        ValidateParallelScene(scene);
        EditorSceneManager.MarkSceneDirty(scene);
        if (!EditorSceneManager.SaveScene(scene))
        {
            throw new InvalidOperationException($"Unable to save {ScenePath}.");
        }
        Debug.Log("Prepared 1Chase1 with replicated training areas and scene-local domain randomization.");
    }

    public static void EnsurePrepared()
    {
        PrepareOneChaseOneParallelScene();
    }

    [MenuItem("FinsSim/RL/Validate 1Chase1 Parallel Scene")]
    public static void ValidateCurrentParallelScene()
    {
        Scene scene = SceneManager.GetActiveScene();
        ValidateParallelScene(scene);
        Debug.Log("1Chase1 parallel scene validation passed.");
    }

    static void CreateParallelArea(Scene scene)
    {
        OneChaseOnePoseAgent agent = UnityEngine.Object.FindFirstObjectByType<OneChaseOnePoseAgent>();
        if (agent == null)
        {
            throw new InvalidOperationException("1Chase1 has no OneChaseOnePoseAgent.");
        }
        OneChaseOnePreyController prey = agent.preyController != null
            ? agent.preyController
            : UnityEngine.Object.FindFirstObjectByType<OneChaseOnePreyController>();
        if (prey == null)
        {
            throw new InvalidOperationException("1Chase1 has no OneChaseOnePreyController.");
        }

        var area = new GameObject(AreaName);
        area.transform.SetPositionAndRotation(Vector3.zero, Quaternion.identity);
        agent.transform.SetParent(area.transform, true);
        prey.transform.SetParent(area.transform, true);

        PhysicalWaveWaterDataProvider provider = CreateProvider(area.transform);
        EnsureDomainRandomization(agent, provider);

        var runtime = area.AddComponent<OneChaseOneParallelAreaRuntime>();
        runtime.areaSeparation = AreaSeparation;
        runtime.localWaterSurfaceY = 0f;
        runtime.localMinWaterY = -6f;
        runtime.seedStride = 1_000_003;
        runtime.ConfigureArea();

        var replicatorRoot = new GameObject(ReplicatorName);
        TrainingAreaReplicator replicator = replicatorRoot.AddComponent<TrainingAreaReplicator>();
        replicator.baseArea = area;
        replicator.numAreas = InspectorDefaultAreaCount;
        replicator.separation = AreaSeparation;
        replicator.buildOnly = true;
    }

    static void NormalizeExistingParallelArea(TrainingAreaReplicator replicator)
    {
        if (replicator.baseArea == null)
        {
            throw new InvalidOperationException("1Chase1 TrainingAreaReplicator has no baseArea.");
        }
        replicator.name = ReplicatorName;
        replicator.numAreas = InspectorDefaultAreaCount;
        replicator.separation = AreaSeparation;
        replicator.buildOnly = true;

        OneChaseOneParallelAreaRuntime runtime = replicator.baseArea.GetComponent<OneChaseOneParallelAreaRuntime>();
        if (runtime == null)
        {
            throw new InvalidOperationException("1Chase1 base area has no OneChaseOneParallelAreaRuntime.");
        }
        runtime.areaSeparation = AreaSeparation;
        runtime.ConfigureArea();
    }

    static PhysicalWaveWaterDataProvider CreateProvider(Transform parent)
    {
        var providerObject = new GameObject("PhysicalWaveWaterProvider");
        providerObject.transform.SetParent(parent, false);
        PhysicalWaveWaterDataProvider provider = providerObject.AddComponent<PhysicalWaveWaterDataProvider>();
        provider.mode = PhysicalWaveWaterDataProvider.WavePhysicsMode.FlatFallback;
        provider.stillWaterHeight = 0f;
        provider.fallbackWaterHeight = 0f;
        provider.includeWaveNormals = true;
        provider.includeWaveFlow = true;
        provider.includeVerticalOrbitalFlow = true;
        provider.randomizeWaterCurrentFromProfile = true;
        provider.randomizeWavesFromProfile = true;
        provider.forceAnalyticModeWhenRandomizingWaves = true;
        return provider;
    }

    static void EnsureDomainRandomization(OneChaseOnePoseAgent agent, PhysicalWaveWaterDataProvider provider)
    {
        DomainRandomizationProfile profile = AssetDatabase.LoadAssetAtPath<DomainRandomizationProfile>(
            DomainRandomizationProfilePath);
        if (profile == null)
        {
            throw new InvalidOperationException($"Missing DR profile at {DomainRandomizationProfilePath}.");
        }

        GameObject vehicle = agent.gameObject;
        if (vehicle.GetComponent<ThrusterRandomizationTarget>() == null)
        {
            vehicle.AddComponent<ThrusterRandomizationTarget>();
        }

        DomainRandomizationCoordinator coordinator = vehicle.GetComponent<DomainRandomizationCoordinator>();
        if (coordinator == null)
        {
            coordinator = vehicle.AddComponent<DomainRandomizationCoordinator>();
        }
        coordinator.profile = profile;
        coordinator.mode = DomainRandomizationMode.Train;
        coordinator.baseSeed = 12345;
        coordinator.incrementSeedPerEpisode = true;
        coordinator.randomizeOnStart = false;
        coordinator.autoFindTargets = true;
        coordinator.targetBehaviours.Clear();
        coordinator.targetBehaviours.Add(provider);

        foreach (HydrodynamicsController controller in vehicle.GetComponentsInChildren<HydrodynamicsController>(true))
        {
            controller.waterProviderBehaviour = provider;
        }
    }

    static void ValidateParallelScene(Scene scene)
    {
        if (scene.path != ScenePath)
        {
            throw new InvalidOperationException($"Expected active scene {ScenePath}, got {scene.path}.");
        }
        TrainingAreaReplicator replicator = UnityEngine.Object.FindFirstObjectByType<TrainingAreaReplicator>();
        if (replicator == null || replicator.baseArea == null || !replicator.buildOnly ||
            !Mathf.Approximately(replicator.separation, AreaSeparation))
        {
            throw new InvalidOperationException("1Chase1 TrainingAreaReplicator is missing or has an invalid configuration.");
        }

        OneChaseOnePoseAgent[] agents = replicator.baseArea.GetComponentsInChildren<OneChaseOnePoseAgent>(true);
        OneChaseOnePreyController[] prey = replicator.baseArea.GetComponentsInChildren<OneChaseOnePreyController>(true);
        if (agents.Length != 1 || prey.Length != 1 || agents[0].preyTransform != prey[0].transform ||
            agents[0].preyController != prey[0])
        {
            throw new InvalidOperationException("1Chase1 base area does not have isolated agent/prey references.");
        }

        BehaviorParameters behavior = agents[0].GetComponent<BehaviorParameters>();
        if (behavior == null || behavior.BehaviorName != "OneChaseOnePose" ||
            behavior.BrainParameters.VectorObservationSize != 14 ||
            behavior.BrainParameters.ActionSpec.NumContinuousActions != 8)
        {
            throw new InvalidOperationException("1Chase1 parallel behavior must remain 14D observations and 8D actions.");
        }

        if (replicator.baseArea.GetComponentInChildren<PhysicalWaveWaterDataProvider>(true) == null ||
            replicator.baseArea.GetComponentInChildren<DomainRandomizationCoordinator>(true) == null ||
            replicator.baseArea.GetComponent<OneChaseOneParallelAreaRuntime>() == null)
        {
            throw new InvalidOperationException("1Chase1 base area is missing isolated water, DR, or runtime wiring.");
        }
    }
}
