using System;
using FinsSim.Hydrodynamics;
using Unity.MLAgents.Areas;
using Unity.MLAgents.Policies;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

/// <summary>
/// Validates the hand-authored T2 trajectory training scene before a build.
///
/// This intentionally does not derive the T2 scene from a HoldForPosition
/// template. The scene is the source of truth for all task and Inspector
/// configuration, so a build must never rewrite it.
/// </summary>
public static class TrajectoryTrackingFossenParallelSceneBuilder
{
    public const string ScenePath = "Assets/FinsSimUnity/Tasks/TrajectoryTracking/Scenes/TrajectoryTracking_Fossen_Parallel_30s.unity";
    public const int EpisodeMaxStep = 1500;

    [MenuItem("FinsSim/RL/Validate T2 TrajectoryTracking Fossen Scene")]
    public static void ValidateSceneForBuild()
    {
        Scene scene = EditorSceneManager.OpenScene(ScenePath, OpenSceneMode.Single);
        Validate(scene);
        Debug.Log($"Validated {ScenePath}; no scene content was changed.");
    }

    /// <summary>
    /// Called by the Linux build method. Keep this validation-only so edits to
    /// the T2 scene, including user-configured DR and visualizer settings,
    /// remain intact across builds.
    /// </summary>
    public static void EnsurePrepared()
    {
        Scene scene = EditorSceneManager.OpenScene(ScenePath, OpenSceneMode.Single);
        Validate(scene);
    }

    static void Validate(Scene scene)
    {
        if (scene.path != ScenePath)
        {
            throw new InvalidOperationException($"Expected active scene {ScenePath}, got {scene.path}.");
        }

        TrainingAreaReplicator replicator = UnityEngine.Object.FindFirstObjectByType<TrainingAreaReplicator>();
        if (replicator == null || replicator.baseArea == null || !replicator.buildOnly)
        {
            throw new InvalidOperationException("T2 scene requires a build-only TrainingAreaReplicator with a base area.");
        }

        GameObject area = replicator.baseArea;
        TrajectoryTrackingAgent[] agents = area.GetComponentsInChildren<TrajectoryTrackingAgent>(true);
        if (agents.Length != 1)
        {
            throw new InvalidOperationException($"T2 base area must contain exactly one {nameof(TrajectoryTrackingAgent)}, found {agents.Length}.");
        }
        if (agents[0].MaxStep != EpisodeMaxStep)
        {
            throw new InvalidOperationException($"T2 agent MaxStep must be {EpisodeMaxStep}, got {agents[0].MaxStep}.");
        }

        HoldForPosition[] legacyAgents = area.GetComponentsInChildren<HoldForPosition>(true);
        if (legacyAgents.Length != 0)
        {
            throw new InvalidOperationException("T2 base area still contains a HoldForPosition agent.");
        }

        BehaviorParameters behavior = agents[0].GetComponent<BehaviorParameters>();
        if (behavior == null || behavior.BehaviorName != "TrajectoryTracking" ||
            behavior.BrainParameters.VectorObservationSize != TrajectoryTrackingAgent.VectorObservationSize ||
            behavior.BrainParameters.ActionSpec.NumContinuousActions != TrajectoryTrackingAgent.ThrusterActionSize)
        {
            throw new InvalidOperationException("T2 behavior must be TrajectoryTracking with 30 observations and 8 continuous actions.");
        }

        if (area.GetComponent<TrajectoryTrackingParallelAreaRuntime>() == null)
        {
            throw new InvalidOperationException("T2 base area is missing TrajectoryTrackingParallelAreaRuntime.");
        }
        if (area.transform.Find("TrajectoryReference") == null)
        {
            throw new InvalidOperationException("T2 base area is missing TrajectoryReference.");
        }
        if (area.GetComponentInChildren<TrajectoryTrackingReferenceVisualizer>(true) == null)
        {
            throw new InvalidOperationException("T2 base area is missing TrajectoryTrackingReferenceVisualizer.");
        }
        if (area.GetComponentInChildren<DomainRandomizationCoordinator>(true) == null)
        {
            throw new InvalidOperationException("T2 base area is missing DomainRandomizationCoordinator.");
        }
    }
}
