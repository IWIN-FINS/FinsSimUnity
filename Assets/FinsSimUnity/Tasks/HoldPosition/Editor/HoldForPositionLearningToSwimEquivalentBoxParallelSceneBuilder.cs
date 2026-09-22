using System;
using System.Collections.Generic;
using FinsSim.Hydrodynamics;
using Unity.MLAgents.Areas;
using Unity.MLAgents.Policies;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

/// <summary>
/// Builds the 30 s HoldForPosition Player that uses the IsaacLab
/// Learning-to-Swim equivalent-box water model.  Both the direct-thruster and
/// Python-side wrench allocator training paths use this same 8-thruster scene.
/// </summary>
public static class HoldForPositionLearningToSwimEquivalentBoxParallelSceneBuilder
{
    public const string SourceScenePath = "Assets/FinsSimUnity/Tasks/HoldPosition/Scenes/HoldForPosition_Fossen_Parallel_30s.unity";
    public const string DirectScenePath = "Assets/FinsSimUnity/Tasks/HoldPosition/Scenes/HoldForPosition_LearningToSwimEquivalentBox_Parallel_1024_30s_8Thruster.unity";
    public const string HydrodynamicsProfilePath = "Assets/Models/FinsROV/Hydrodynamics/FinsROV_HydrodynamicsProfile_LearningToSwimEquivalentBox.asset";
    public const string DomainRandomizationProfilePath = "Assets/Models/FinsROV/Hydrodynamics/DomainRandomizationProfile_LearningToSwimEquivalentBox.asset";
    public const int AreaCount = 1024;
    public const int EpisodeMaxStep = 1500;

    const string DirectBehaviorName = "HoldForPositionEquivalentBox8Thruster";

    [MenuItem("FinsSim/RL/Prepare HoldForPosition EquivalentBox 1024x30s 8-Thruster")]
    public static void PrepareDirectScene()
    {
        PrepareScene(DirectScenePath, DirectBehaviorName);
    }

    public static void EnsureDirectPrepared()
    {
        ValidateExistingScene(DirectScenePath, DirectBehaviorName);
    }

    static void PrepareScene(string outputPath, string behaviorName)
    {
        Scene source = EditorSceneManager.OpenScene(SourceScenePath, OpenSceneMode.Single);
        if (!EditorSceneManager.SaveScene(source, outputPath, true))
        {
            throw new InvalidOperationException($"Unable to create {outputPath} from {SourceScenePath}.");
        }

        Scene scene = EditorSceneManager.OpenScene(outputPath, OpenSceneMode.Single);
        // Opening a scene may unload assets that were loaded beforehand.  Load
        // both profiles only after the output scene is active so the references
        // assigned below cannot become Unity "fake null" objects.
        HydrodynamicsProfile hydrodynamicsProfile = AssetDatabase.LoadAssetAtPath<HydrodynamicsProfile>(HydrodynamicsProfilePath);
        DomainRandomizationProfile randomizationProfile = AssetDatabase.LoadAssetAtPath<DomainRandomizationProfile>(DomainRandomizationProfilePath);
        if (hydrodynamicsProfile == null || randomizationProfile == null)
        {
            throw new InvalidOperationException("Equivalent-box hydrodynamics or DR profile asset is missing.");
        }
        TrainingAreaReplicator replicator = UnityEngine.Object.FindFirstObjectByType<TrainingAreaReplicator>();
        if (replicator == null || replicator.baseArea == null)
        {
            throw new InvalidOperationException("Source scene has no TrainingAreaReplicator base area.");
        }

        GameObject area = replicator.baseArea;
        HoldForPosition[] agents = area.GetComponentsInChildren<HoldForPosition>(true);
        if (agents.Length != 1)
        {
            throw new InvalidOperationException($"Expected one HoldForPosition base agent, found {agents.Length}.");
        }
        agents[0].ConfigureEpisodeMaxStep(EpisodeMaxStep);
        RecordSceneOverride(agents[0]);

        BehaviorParameters behavior = agents[0].GetComponent<BehaviorParameters>();
        if (behavior == null)
        {
            throw new InvalidOperationException("HoldForPosition base agent has no BehaviorParameters component.");
        }
        behavior.BehaviorName = behaviorName;
        RecordSceneOverride(behavior);

        HydrodynamicsController[] controllers = area.GetComponentsInChildren<HydrodynamicsController>(true);
        if (controllers.Length == 0)
        {
            throw new InvalidOperationException("Base area has no HydrodynamicsController.");
        }
        foreach (HydrodynamicsController controller in controllers)
        {
            controller.mode = HydrodynamicsMode.LearningToSwimEquivalentBox;
            controller.profile = hydrodynamicsProfile;
            controller.ResetRuntimeProfileFromSource();
            RecordSceneOverride(controller);
        }

        // This component normally reapplies its profile in Awake.  Store the
        // same asset in both places so no old Fossen profile can overwrite the
        // equivalent-box setup when a replicated area starts.
        foreach (FinsROVHydrodynamicsSetup setup in area.GetComponentsInChildren<FinsROVHydrodynamicsSetup>(true))
        {
            setup.hydrodynamicsProfile = hydrodynamicsProfile;
            RecordSceneOverride(setup);
        }

        DomainRandomizationCoordinator[] coordinators = area.GetComponentsInChildren<DomainRandomizationCoordinator>(true);
        if (coordinators.Length == 0)
        {
            throw new InvalidOperationException("Base area has no DomainRandomizationCoordinator.");
        }
        foreach (DomainRandomizationCoordinator coordinator in coordinators)
        {
            coordinator.profile = randomizationProfile;
            RecordSceneOverride(coordinator);
        }

        // Academy.NumAreas overwrites this at train time; setting it here also
        // makes a standalone Player reproduce the requested 1024-area layout.
        replicator.numAreas = AreaCount;
        replicator.buildOnly = true;
        RecordSceneOverride(replicator);

        Validate(scene, outputPath, behaviorName, hydrodynamicsProfile, randomizationProfile);
        EditorSceneManager.MarkSceneDirty(scene);
        if (!EditorSceneManager.SaveScene(scene))
        {
            throw new InvalidOperationException($"Unable to save {outputPath}.");
        }
        Debug.Log($"Prepared {outputPath}: {AreaCount} areas, {EpisodeMaxStep} physics steps, behavior={behaviorName}.");
    }

    static void ValidateExistingScene(string scenePath, string behaviorName)
    {
        Scene scene = EditorSceneManager.OpenScene(scenePath, OpenSceneMode.Single);
        HydrodynamicsProfile hydrodynamicsProfile = AssetDatabase.LoadAssetAtPath<HydrodynamicsProfile>(HydrodynamicsProfilePath);
        DomainRandomizationProfile randomizationProfile = AssetDatabase.LoadAssetAtPath<DomainRandomizationProfile>(DomainRandomizationProfilePath);
        Validate(scene, scenePath, behaviorName, hydrodynamicsProfile, randomizationProfile);
        Debug.Log($"Validated {scenePath}.");
    }

    static void Validate(
        Scene scene,
        string expectedPath,
        string behaviorName,
        HydrodynamicsProfile hydrodynamicsProfile,
        DomainRandomizationProfile randomizationProfile)
    {
        var errors = new List<string>();
        if (scene.path != expectedPath || hydrodynamicsProfile == null || randomizationProfile == null)
        {
            errors.Add($"scene/profile: path='{scene.path}', hydrodynamics={hydrodynamicsProfile}, dr={randomizationProfile}");
        }

        TrainingAreaReplicator replicator = UnityEngine.Object.FindFirstObjectByType<TrainingAreaReplicator>();
        if (replicator == null || replicator.baseArea == null || !replicator.buildOnly || replicator.numAreas != AreaCount)
        {
            errors.Add($"replicator: found={replicator != null}, baseArea={(replicator != null ? replicator.baseArea : null)}, buildOnly={(replicator != null && replicator.buildOnly)}, numAreas={(replicator != null ? replicator.numAreas : -1)}");
        }

        if (replicator == null || replicator.baseArea == null)
        {
            throw new InvalidOperationException("Equivalent-box scene validation failed: " + string.Join(" | ", errors));
        }

        HoldForPosition[] agents = replicator.baseArea.GetComponentsInChildren<HoldForPosition>(true);
        if (agents.Length != 1 || agents[0].MaxStep != EpisodeMaxStep || agents[0].RecommendedMaxStep != EpisodeMaxStep)
        {
            errors.Add($"agent: count={agents.Length}, maxStep={(agents.Length == 1 ? agents[0].MaxStep : -1)}, recommended={(agents.Length == 1 ? agents[0].RecommendedMaxStep : -1)}");
        }
        BehaviorParameters behavior = agents.Length == 1 ? agents[0].GetComponent<BehaviorParameters>() : null;
        if (behavior == null || behavior.BehaviorName != behaviorName ||
            behavior.BrainParameters.VectorObservationSize != HoldForPosition.VectorObservationSize ||
            behavior.BrainParameters.ActionSpec.NumContinuousActions != HoldForPosition.ContinuousActionSize)
        {
            errors.Add($"behavior: name='{(behavior != null ? behavior.BehaviorName : "<null>")}', observations={(behavior != null ? behavior.BrainParameters.VectorObservationSize : -1)}, actions={(behavior != null ? behavior.BrainParameters.ActionSpec.NumContinuousActions : -1)}");
        }

        foreach (HydrodynamicsController controller in replicator.baseArea.GetComponentsInChildren<HydrodynamicsController>(true))
        {
            if (controller.mode != HydrodynamicsMode.LearningToSwimEquivalentBox || controller.profile != hydrodynamicsProfile)
            {
                errors.Add($"hydrodynamics '{controller.name}': mode={controller.mode}, profile={controller.profile}");
            }
        }
        foreach (DomainRandomizationCoordinator coordinator in replicator.baseArea.GetComponentsInChildren<DomainRandomizationCoordinator>(true))
        {
            if (coordinator.profile != randomizationProfile)
            {
                errors.Add($"DR '{coordinator.name}': profile={coordinator.profile}");
            }
        }
        if (errors.Count > 0)
        {
            throw new InvalidOperationException("Equivalent-box scene validation failed: " + string.Join(" | ", errors));
        }
    }

    static void RecordSceneOverride(UnityEngine.Object target)
    {
        PrefabUtility.RecordPrefabInstancePropertyModifications(target);
        EditorUtility.SetDirty(target);
    }
}
