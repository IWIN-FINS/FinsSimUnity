using System;
using FinsSim.Actuators;
using FinsSim.Hydrodynamics;
using Unity.MLAgents.Areas;
using Unity.MLAgents.Policies;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

/// <summary>Creates and validates the replicated Fossen hold-position scenes.</summary>
public static class HoldForPositionFossenParallelSceneBuilder
{
    public const string SourceScenePath = "Assets/FinsSimUnity/Tasks/HoldPosition/Scenes/HoldForPosition_Fossen.unity";
    public const string ParallelScenePath = "Assets/FinsSimUnity/Tasks/HoldPosition/Scenes/HoldForPosition_Fossen_Parallel.unity";
    public const string Parallel15sScenePath = "Assets/FinsSimUnity/Tasks/HoldPosition/Scenes/HoldForPosition_Fossen_Parallel_15s.unity";
    public const string Parallel30sScenePath = "Assets/FinsSimUnity/Tasks/HoldPosition/Scenes/HoldForPosition_Fossen_Parallel_30s.unity";
    public const string Parallel30sNoDrScenePath = "Assets/FinsSimUnity/Tasks/HoldPosition/Scenes/HoldForPosition_Fossen_Parallel_30s_NoDR.unity";
    public const string Parallel30sNormalizedMaxForceScenePath =
        "Assets/FinsSimUnity/Tasks/HoldPosition/Scenes/HoldForPosition_Fossen_Parallel_30s_NormalizedMaxForce.unity";
    public const string Parallel30sNormalizedMaxForceNoDrScenePath =
        "Assets/FinsSimUnity/Tasks/HoldPosition/Scenes/HoldForPosition_Fossen_Parallel_30s_NormalizedMaxForce_NoDR.unity";
    public const float AreaSeparation = 32f;
    public const int InspectorDefaultAreaCount = 2048;
    public const int DefaultEpisodeMaxStep = 3000;
    // Fixed timestep is 0.02 s; ML-Agents MaxStep is measured in physics steps.
    public const int ShortEpisodeMaxStep = 750;
    public const int ThirtySecondEpisodeMaxStep = 1500;

    [MenuItem("FinsSim/RL/Prepare HoldForPosition Fossen Parallel Scene")]
    public static void PrepareHoldForPositionFossenParallelScene()
    {
        PrepareParallelScene(ParallelScenePath, DefaultEpisodeMaxStep);
    }

    [MenuItem("FinsSim/RL/Prepare HoldForPosition Fossen Parallel 15s Scene")]
    public static void PrepareHoldForPositionFossenParallel15sScene()
    {
        PrepareParallelScene(Parallel15sScenePath, ShortEpisodeMaxStep);
    }

    [MenuItem("FinsSim/RL/Prepare HoldForPosition Fossen Parallel 30s Scene")]
    public static void PrepareHoldForPositionFossenParallel30sScene()
    {
        // Use the existing 15s parallel scene as the template so its explicit
        // Fossen, DR, reward, and thruster wiring is preserved. The source is
        // opened read-only and saved to a new scene before any changes occur.
        PrepareExistingParallelScene(
            Parallel15sScenePath,
            Parallel30sScenePath,
            ThirtySecondEpisodeMaxStep,
            thrusterCommandMode: ThrusterCommandMode.NormalizedMaxForceRequest);
    }

    [MenuItem("FinsSim/RL/Prepare HoldForPosition Fossen Parallel 30s No-DR Scene")]
    public static void PrepareHoldForPositionFossenParallel30sNoDrScene()
    {
        // Start from the built 30 s Fossen scene so water, hydrodynamics,
        // reward, action, and replication settings remain identical. Only
        // the episode DR coordinator is disabled in the copied scene.
        PrepareExistingParallelScene(
            Parallel30sScenePath,
            Parallel30sNoDrScenePath,
            ThirtySecondEpisodeMaxStep,
            disableDomainRandomization: true,
            thrusterCommandMode: ThrusterCommandMode.NormalizedMaxForceRequest);
    }

    [MenuItem("FinsSim/RL/Prepare HoldForPosition Fossen Parallel 30s Normalized-Max-Force Scene")]
    public static void PrepareHoldForPositionFossenParallel30sNormalizedMaxForceScene()
    {
        // Kept as a named compatibility scene. All HoldForPosition Fossen
        // scenes now use this contract: [-1, 1] maps to each thruster's
        // current positive/negative calibrated force limit.
        PrepareExistingParallelScene(
            Parallel30sScenePath,
            Parallel30sNormalizedMaxForceScenePath,
            ThirtySecondEpisodeMaxStep,
            thrusterCommandMode: ThrusterCommandMode.NormalizedMaxForceRequest);
    }

    [MenuItem("FinsSim/RL/Prepare HoldForPosition Fossen Parallel 30s Normalized-Max-Force No-DR Scene")]
    public static void PrepareHoldForPositionFossenParallel30sNormalizedMaxForceNoDrScene()
    {
        // Copy the explicit normalized-max-force variant so this build has an
        // unambiguous actuator ABI. Keep every task setting intact and only
        // disable the physical/environmental DR coordinator in the copy.
        PrepareExistingParallelScene(
            Parallel30sNormalizedMaxForceScenePath,
            Parallel30sNormalizedMaxForceNoDrScenePath,
            ThirtySecondEpisodeMaxStep,
            disableDomainRandomization: true,
            thrusterCommandMode: ThrusterCommandMode.NormalizedMaxForceRequest);
    }

    static void PrepareExistingParallelScene(
        string sourcePath,
        string outputPath,
        int maxPhysicsSteps,
        bool disableDomainRandomization = false,
        ThrusterCommandMode? thrusterCommandMode = null)
    {
        Scene scene = EditorSceneManager.OpenScene(sourcePath, OpenSceneMode.Single);
        if (!EditorSceneManager.SaveScene(scene, outputPath, true))
        {
            throw new InvalidOperationException($"Unable to copy {sourcePath} to {outputPath}.");
        }

        scene = EditorSceneManager.OpenScene(outputPath, OpenSceneMode.Single);
        HoldForPosition agent = UnityEngine.Object.FindFirstObjectByType<HoldForPosition>();
        if (agent == null)
        {
            throw new InvalidOperationException($"Source parallel scene has no HoldForPosition agent: {sourcePath}.");
        }

        agent.ConfigureEpisodeMaxStep(maxPhysicsSteps);
        agent.ConfigureThrusterCommandMode(
            thrusterCommandMode ?? ThrusterCommandMode.NormalizedMaxForceRequest);
        if (disableDomainRandomization)
        {
            DisableDomainRandomization(scene);
        }

        ValidateParallelScene(
            scene,
            outputPath,
            maxPhysicsSteps,
            domainRandomizationEnabled: !disableDomainRandomization,
            expectedThrusterCommandMode:
                thrusterCommandMode ?? ThrusterCommandMode.NormalizedMaxForceRequest);
        EditorSceneManager.MarkSceneDirty(scene);
        if (!EditorSceneManager.SaveScene(scene))
        {
            throw new InvalidOperationException($"Unable to save {outputPath}.");
        }
        Debug.Log($"Prepared {outputPath} from {sourcePath}.");
    }

    static void PrepareParallelScene(string outputPath, int maxPhysicsSteps)
    {
        PrepareParallelSceneFromSource(SourceScenePath, outputPath, maxPhysicsSteps);
    }

    static void PrepareParallelSceneFromSource(string sourcePath, string outputPath, int maxPhysicsSteps)
    {
        Scene scene = EditorSceneManager.OpenScene(sourcePath, OpenSceneMode.Single);
        if (!EditorSceneManager.SaveScene(scene, outputPath, true))
        {
            throw new InvalidOperationException($"Unable to copy {sourcePath} to {outputPath}.");
        }

        // SaveScene with a new path writes a copy but retains the source as
        // the active scene. Re-open the copy before modifying it so the
        // original HoldForPosition_Fossen scene remains untouched.
        scene = EditorSceneManager.OpenScene(outputPath, OpenSceneMode.Single);
        RemoveExistingParallelRoot(scene);
        GameObject ships = FindRoot(scene, "Ships");
        HoldForPosition agent = ships.GetComponentInChildren<HoldForPosition>(true);
        if (agent == null)
        {
            throw new InvalidOperationException("Source scene has no HoldForPosition agent below Ships.");
        }
        agent.ConfigureEpisodeMaxStep(maxPhysicsSteps);
        agent.ConfigureThrusterCommandMode(ThrusterCommandMode.NormalizedMaxForceRequest);
        Transform target = FindDirectChild(ships.transform, "Cube");
        if (target == null)
        {
            throw new InvalidOperationException("Source scene has no Cube target below Ships.");
        }
        MonoBehaviour sourceProvider = FindAreaWaterProvider();
        if (sourceProvider == null)
        {
            throw new InvalidOperationException($"Source scene has no {nameof(IAreaWaterKinematicsProvider)}.");
        }

        var area = new GameObject("HoldForPositionTrainingArea");
        area.transform.SetPositionAndRotation(Vector3.zero, Quaternion.identity);
        agent.transform.SetParent(area.transform, true);
        agent.name = "FinsROV_Fossen";
        target.SetParent(area.transform, true);
        target.name = "Target";

        // The Fossen source keeps its provider at scene scope. Move an exact
        // copy into the base area so TrainingAreaReplicator duplicates it with
        // every agent, then disable the original scene-wide instance.
        GameObject providerObject = UnityEngine.Object.Instantiate(sourceProvider.gameObject, area.transform, true);
        providerObject.name = "PhysicalWaveWaterProvider";
        sourceProvider.enabled = false;

        var runtime = area.AddComponent<HoldForPositionParallelAreaRuntime>();
        runtime.areaSeparation = AreaSeparation;
        runtime.localWaterHeight = 0f;
        runtime.seedStride = 1_000_003;
        runtime.ConfigureArea();

        var replicatorRoot = new GameObject("__HoldForPositionTrainingAreaReplicator");
        TrainingAreaReplicator replicator = replicatorRoot.AddComponent<TrainingAreaReplicator>();
        replicator.baseArea = area;
        replicator.numAreas = InspectorDefaultAreaCount;
        replicator.separation = AreaSeparation;
        replicator.buildOnly = true;

        ValidateParallelScene(
            scene,
            outputPath,
            maxPhysicsSteps,
            expectedThrusterCommandMode: ThrusterCommandMode.NormalizedMaxForceRequest);
        EditorSceneManager.MarkSceneDirty(scene);
        if (!EditorSceneManager.SaveScene(scene))
        {
            throw new InvalidOperationException($"Unable to save {outputPath}.");
        }
        Debug.Log($"Prepared {outputPath}. ML-Agents controls the runtime area count through numAreas.");
    }

    [MenuItem("FinsSim/RL/Validate HoldForPosition Fossen Parallel Scene")]
    public static void ValidateCurrentParallelScene()
    {
        Scene scene = SceneManager.GetActiveScene();
        int expectedMaxStep = scene.path == Parallel15sScenePath
            ? ShortEpisodeMaxStep
            : scene.path == Parallel30sScenePath ||
              scene.path == Parallel30sNoDrScenePath ||
              scene.path == Parallel30sNormalizedMaxForceScenePath ||
              scene.path == Parallel30sNormalizedMaxForceNoDrScenePath
                ? ThirtySecondEpisodeMaxStep
                : DefaultEpisodeMaxStep;
        ValidateParallelScene(
            scene,
            scene.path,
            expectedMaxStep,
            domainRandomizationEnabled:
                scene.path != Parallel30sNoDrScenePath &&
                scene.path != Parallel30sNormalizedMaxForceNoDrScenePath,
            expectedThrusterCommandMode: ThrusterCommandMode.NormalizedMaxForceRequest);
        Debug.Log("HoldForPosition Fossen parallel scene validation passed.");
    }

    public static void EnsurePrepared()
    {
        ValidateExistingParallelScene(
            ParallelScenePath,
            DefaultEpisodeMaxStep,
            expectedThrusterCommandMode: ThrusterCommandMode.NormalizedMaxForceRequest);
    }

    public static void EnsurePrepared15s()
    {
        ValidateExistingParallelScene(
            Parallel15sScenePath,
            ShortEpisodeMaxStep,
            expectedThrusterCommandMode: ThrusterCommandMode.NormalizedMaxForceRequest);
    }

    public static void EnsurePrepared30s()
    {
        ValidateExistingParallelScene(
            Parallel30sScenePath,
            ThirtySecondEpisodeMaxStep,
            expectedThrusterCommandMode: ThrusterCommandMode.NormalizedMaxForceRequest);
    }

    public static void EnsurePrepared30sNoDr()
    {
        ValidateExistingParallelScene(
            Parallel30sNoDrScenePath,
            ThirtySecondEpisodeMaxStep,
            domainRandomizationEnabled: false,
            expectedThrusterCommandMode: ThrusterCommandMode.NormalizedMaxForceRequest);
    }

    public static void EnsurePrepared30sNormalizedMaxForce()
    {
        ValidateExistingParallelScene(
            Parallel30sNormalizedMaxForceScenePath,
            ThirtySecondEpisodeMaxStep,
            expectedThrusterCommandMode: ThrusterCommandMode.NormalizedMaxForceRequest);
    }

    public static void EnsurePrepared30sNormalizedMaxForceNoDr()
    {
        ValidateExistingParallelScene(
            Parallel30sNormalizedMaxForceNoDrScenePath,
            ThirtySecondEpisodeMaxStep,
            domainRandomizationEnabled: false,
            expectedThrusterCommandMode: ThrusterCommandMode.NormalizedMaxForceRequest);
    }

    static void ValidateExistingParallelScene(
        string scenePath,
        int expectedMaxPhysicsSteps,
        bool domainRandomizationEnabled = true,
        ThrusterCommandMode? expectedThrusterCommandMode = null)
    {
        Scene scene = EditorSceneManager.OpenScene(scenePath, OpenSceneMode.Single);
        ValidateParallelScene(
            scene,
            scenePath,
            expectedMaxPhysicsSteps,
            domainRandomizationEnabled,
            expectedThrusterCommandMode);
        Debug.Log($"Validated {scenePath}; build preparation did not copy or modify another scene.");
    }

    static void ValidateParallelScene(
        Scene scene,
        string expectedPath,
        int expectedMaxPhysicsSteps,
        bool domainRandomizationEnabled = true,
        ThrusterCommandMode? expectedThrusterCommandMode = null)
    {
        if (scene.path != expectedPath)
        {
            throw new InvalidOperationException($"Expected active scene {expectedPath}, got {scene.path}.");
        }
        TrainingAreaReplicator replicator = UnityEngine.Object.FindFirstObjectByType<TrainingAreaReplicator>();
        if (replicator == null || replicator.baseArea == null || !replicator.buildOnly || !Mathf.Approximately(replicator.separation, AreaSeparation))
        {
            throw new InvalidOperationException("TrainingAreaReplicator is missing or has an invalid configuration.");
        }
        HoldForPosition[] agents = replicator.baseArea.GetComponentsInChildren<HoldForPosition>(true);
        if (agents.Length != 1)
        {
            throw new InvalidOperationException($"Base area must contain exactly one HoldForPosition agent, found {agents.Length}.");
        }
        if (agents[0].MaxStep != expectedMaxPhysicsSteps || agents[0].RecommendedMaxStep != expectedMaxPhysicsSteps)
        {
            throw new InvalidOperationException(
                $"HoldForPosition episode limit must be {expectedMaxPhysicsSteps} physics steps.");
        }
        if (expectedThrusterCommandMode.HasValue &&
            agents[0].CurrentThrusterCommandMode != expectedThrusterCommandMode.Value)
        {
            throw new InvalidOperationException(
                $"HoldForPosition thruster command mode must be {expectedThrusterCommandMode.Value}, " +
                $"got {agents[0].CurrentThrusterCommandMode}.");
        }
        BehaviorParameters behavior = agents[0].GetComponent<BehaviorParameters>();
        if (behavior == null || behavior.BehaviorName != "HoldForPosition")
        {
            throw new InvalidOperationException("The base agent must use behavior name HoldForPosition.");
        }
        if (behavior.BrainParameters.ActionSpec.NumContinuousActions != HoldForPosition.ContinuousActionSize)
        {
            throw new InvalidOperationException("HoldForPosition parallel action size must be eight continuous values.");
        }
        DomainRandomizationCoordinator[] coordinators =
            replicator.baseArea.GetComponentsInChildren<DomainRandomizationCoordinator>(true);
        if (FindAreaWaterProvider(replicator.baseArea) == null ||
            coordinators.Length == 0 ||
            replicator.baseArea.transform.Find("Target") == null)
        {
            throw new InvalidOperationException("Base area is missing isolated water, randomization, or target objects.");
        }

        if (!domainRandomizationEnabled)
        {
            foreach (DomainRandomizationCoordinator coordinator in coordinators)
            {
                if (coordinator.enabled ||
                    coordinator.mode != DomainRandomizationMode.Disabled ||
                    coordinator.randomizeOnStart ||
                    coordinator.applyCommandLineOverrides)
                {
                    throw new InvalidOperationException(
                        "No-DR scene must disable the DomainRandomizationCoordinator and all DR entry points.");
                }
            }
        }
    }

    static MonoBehaviour FindAreaWaterProvider(GameObject root = null)
    {
        MonoBehaviour[] behaviours = root != null
            ? root.GetComponentsInChildren<MonoBehaviour>(true)
            : UnityEngine.Object.FindObjectsByType<MonoBehaviour>(FindObjectsInactive.Include, FindObjectsSortMode.None);
        foreach (MonoBehaviour behaviour in behaviours)
        {
            if (behaviour is IAreaWaterKinematicsProvider)
            {
                return behaviour;
            }
        }

        return null;
    }

    static void DisableDomainRandomization(Scene scene)
    {
        DomainRandomizationCoordinator[] coordinators =
            UnityEngine.Object.FindObjectsByType<DomainRandomizationCoordinator>(
                FindObjectsInactive.Include,
                FindObjectsSortMode.None);
        if (coordinators.Length == 0)
        {
            throw new InvalidOperationException($"No DomainRandomizationCoordinator found in {scene.path}.");
        }

        foreach (DomainRandomizationCoordinator coordinator in coordinators)
        {
            coordinator.enabled = false;
            coordinator.mode = DomainRandomizationMode.Disabled;
            coordinator.randomizeOnStart = false;
            coordinator.applyCommandLineOverrides = false;
        }
    }

    static void RemoveExistingParallelRoot(Scene scene)
    {
        foreach (GameObject root in scene.GetRootGameObjects())
        {
            if (root.name == "__HoldForPositionTrainingAreaReplicator" || root.name == "HoldForPositionTrainingArea")
            {
                UnityEngine.Object.DestroyImmediate(root);
            }
        }
    }

    static GameObject FindRoot(Scene scene, string name)
    {
        foreach (GameObject root in scene.GetRootGameObjects())
        {
            if (root.name == name)
            {
                return root;
            }
        }
        throw new InvalidOperationException($"Could not find root GameObject {name}.");
    }

    static Transform FindDirectChild(Transform parent, string name)
    {
        foreach (Transform child in parent)
        {
            if (child.name == name)
            {
                return child;
            }
        }
        return null;
    }
}
