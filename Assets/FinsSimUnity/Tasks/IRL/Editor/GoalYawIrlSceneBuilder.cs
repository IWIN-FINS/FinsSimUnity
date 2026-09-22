using System;
using System.Collections.Generic;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Policies;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

/// <summary>Creates the generated, single-area GoalYaw AIRL/GAIL scene.</summary>
public static class GoalYawIrlSceneBuilder
{
    public const string SourceScenePath = "Assets/FinsSimUnity/Tasks/PoseControl/Scenes/ControlForPosition_Fossen.unity";
    public const string GeneratedScenePath = "Assets/FinsSimUnity/Tasks/IRL/Scenes/Generated/GoalYawIRL_Fossen_10Hz.unity";
    public const string BehaviorName = "GoalYawIRL";
    public const ThrusterCommandMode IrlThrusterCommandMode =
        global::ThrusterCommandMode.NormalizedMaxForceRequest;
    // T1 NoDR scene's explicit Rigidbody inertia tensor in the FinsROV body
    // frame.  Keeping it here prevents the generated IRL scene from inheriting
    // a different auto-computed tensor from its ControlForPosition source.
    static readonly Vector3 T1InertiaTensor = new Vector3(0.05f, 0.24651344f, 0.18f);
    public const int DecisionPeriod = 5;
    // Agents execute an action on every 0.02 s Academy step while policy
    // decisions arrive every five steps (10 Hz).  MaxStep is counted in the
    // former unit, so 90 simulated seconds require 90 / 0.02 = 4500 steps.
    public const int EpisodeMaxStep = 4500;

    [MenuItem("FinsSim/IRL/Prepare GoalYaw AIRL-GAIL Fossen Scene")]
    public static void PrepareScene()
    {
        EnsureFolder("Assets/FinsSimUnity/Tasks/IRL/Scenes");
        EnsureFolder("Assets/FinsSimUnity/Tasks/IRL/Scenes/Generated");
        if (AssetDatabase.LoadAssetAtPath<SceneAsset>(GeneratedScenePath) != null &&
            !AssetDatabase.DeleteAsset(GeneratedScenePath))
        {
            throw new InvalidOperationException($"Could not replace generated scene {GeneratedScenePath}.");
        }
        if (!AssetDatabase.CopyAsset(SourceScenePath, GeneratedScenePath))
        {
            throw new InvalidOperationException($"Could not copy {SourceScenePath} to {GeneratedScenePath}.");
        }

        Scene scene = EditorSceneManager.OpenScene(GeneratedScenePath, OpenSceneMode.Single);
        Agent[] existingAgents = FindSceneObjects<Agent>(scene);
        if (existingAgents.Length != 1)
        {
            throw new InvalidOperationException($"Expected one source Agent in {SourceScenePath}, found {existingAgents.Length}.");
        }
        GameObject agentObject = existingAgents[0].gameObject;
        Transform target = FindSceneTransform(scene, "Cube") ?? FindSceneTransform(scene, "Target");
        Transform reference = FindChild(agentObject.transform, "MarkPosition") ?? agentObject.transform;
        if (target == null)
        {
            throw new InvalidOperationException($"Source scene {SourceScenePath} has no Cube/Target transform.");
        }

        // DecisionRequester serializes an Agent reference.  Unity therefore
        // refuses to remove the old task component until this dependent is
        // removed first; recreate it below for the new task component.
        foreach (DecisionRequester oldRequester in FindSceneObjects<DecisionRequester>(scene))
        {
            UnityEngine.Object.DestroyImmediate(oldRequester);
        }
        UnityEngine.Object.DestroyImmediate(existingAgents[0]);
        GoalYawIrlAgent agent = agentObject.AddComponent<GoalYawIrlAgent>();
        agent.ConfigureScene(reference, target, EpisodeMaxStep);
        agent.ConfigureThrusterCommandMode(IrlThrusterCommandMode);
        ConfigureT1Rigidbody(agentObject);

        BehaviorParameters behavior = agentObject.GetComponent<BehaviorParameters>();
        if (behavior == null)
        {
            throw new InvalidOperationException("Source Agent has no BehaviorParameters component.");
        }
        behavior.BehaviorName = BehaviorName;
        behavior.BrainParameters.VectorObservationSize = GoalYawIrlAgent.VectorObservationSize;
        behavior.BrainParameters.ActionSpec = ActionSpec.MakeContinuous(GoalYawIrlAgent.ContinuousActionSize);

        DecisionRequester requester = agentObject.GetComponent<DecisionRequester>();
        if (requester == null)
        {
            requester = agentObject.AddComponent<DecisionRequester>();
        }
        requester.DecisionPeriod = DecisionPeriod;
        requester.TakeActionsBetweenDecisions = true;

        Validate(scene);
        EditorSceneManager.MarkSceneDirty(scene);
        if (!EditorSceneManager.SaveScene(scene))
        {
            throw new InvalidOperationException($"Could not save {GeneratedScenePath}.");
        }
        AssetDatabase.SaveAssets();
        AssetDatabase.Refresh();
        Debug.Log($"Prepared {GeneratedScenePath}: behavior={BehaviorName}, obs=16, actions=8, maxStep={EpisodeMaxStep}.");
    }

    public static void EnsurePrepared()
    {
        if (AssetDatabase.LoadAssetAtPath<SceneAsset>(GeneratedScenePath) == null)
        {
            PrepareScene();
            return;
        }
        Scene scene = EditorSceneManager.OpenScene(GeneratedScenePath, OpenSceneMode.Single);
        GoalYawIrlAgent[] agents = FindSceneObjects<GoalYawIrlAgent>(scene);
        bool changed = false;
        if (agents.Length == 1 && agents[0].CurrentThrusterCommandMode != IrlThrusterCommandMode)
        {
            agents[0].ConfigureThrusterCommandMode(IrlThrusterCommandMode);
            changed = true;
        }
        if (agents.Length == 1 && ConfigureT1Rigidbody(agents[0].gameObject))
        {
            changed = true;
        }
        if (changed)
        {
            EditorSceneManager.MarkSceneDirty(scene);
            if (!EditorSceneManager.SaveScene(scene))
            {
                throw new InvalidOperationException($"Could not update {GeneratedScenePath}.");
            }
        }
        Validate(scene);
    }

    static void Validate(Scene scene)
    {
        if (scene.path != GeneratedScenePath)
        {
            throw new InvalidOperationException($"Expected {GeneratedScenePath}, got {scene.path}.");
        }
        GoalYawIrlAgent[] agents = FindSceneObjects<GoalYawIrlAgent>(scene);
        Agent[] allAgents = FindSceneObjects<Agent>(scene);
        Rigidbody body = agents.Length == 1 ? agents[0].GetComponent<Rigidbody>() : null;
        if (agents.Length != 1 || allAgents.Length != 1 || agents[0].MaxStep != EpisodeMaxStep ||
            agents[0].TargetTransform == null || agents[0].CurrentThrusterCommandMode != IrlThrusterCommandMode ||
            body == null || (body.inertiaTensor - T1InertiaTensor).sqrMagnitude > 1e-10f)
        {
            throw new InvalidOperationException("Generated GoalYaw scene violates the T1 thrust/inertia contract.");
        }
        BehaviorParameters behavior = agents[0].GetComponent<BehaviorParameters>();
        DecisionRequester requester = agents[0].GetComponent<DecisionRequester>();
        if (behavior == null || behavior.BehaviorName != BehaviorName ||
            behavior.BrainParameters.VectorObservationSize != GoalYawIrlAgent.VectorObservationSize ||
            behavior.BrainParameters.ActionSpec.NumContinuousActions != GoalYawIrlAgent.ContinuousActionSize ||
            requester == null || requester.DecisionPeriod != DecisionPeriod)
        {
            throw new InvalidOperationException("Generated GoalYaw scene violates its obs16/act8/10Hz behavior contract.");
        }
    }

    static T[] FindSceneObjects<T>(Scene scene) where T : Component
    {
        var result = new List<T>();
        foreach (GameObject root in scene.GetRootGameObjects())
        {
            result.AddRange(root.GetComponentsInChildren<T>(true));
        }
        return result.ToArray();
    }

    static Transform FindSceneTransform(Scene scene, string name)
    {
        foreach (GameObject root in scene.GetRootGameObjects())
        {
            Transform found = FindChild(root.transform, name);
            if (found != null)
            {
                return found;
            }
        }
        return null;
    }

    static Transform FindChild(Transform root, string name)
    {
        if (root.name == name)
        {
            return root;
        }
        foreach (Transform child in root)
        {
            Transform found = FindChild(child, name);
            if (found != null)
            {
                return found;
            }
        }
        return null;
    }

    static bool ConfigureT1Rigidbody(GameObject agentObject)
    {
        Rigidbody body = agentObject.GetComponent<Rigidbody>();
        if (body == null)
        {
            throw new InvalidOperationException("GoalYawIRL agent requires a Rigidbody.");
        }
        if ((body.inertiaTensor - T1InertiaTensor).sqrMagnitude <= 1e-10f)
        {
            return false;
        }
        body.inertiaTensor = T1InertiaTensor;
        return true;
    }

    static void EnsureFolder(string assetFolder)
    {
        if (AssetDatabase.IsValidFolder(assetFolder))
        {
            return;
        }
        string parent = System.IO.Path.GetDirectoryName(assetFolder)?.Replace('\\', '/');
        string name = System.IO.Path.GetFileName(assetFolder);
        if (string.IsNullOrEmpty(parent) || string.IsNullOrEmpty(name))
        {
            throw new InvalidOperationException($"Invalid asset folder {assetFolder}.");
        }
        EnsureFolder(parent);
        if (AssetDatabase.CreateFolder(parent, name) == string.Empty)
        {
            throw new InvalidOperationException($"Could not create asset folder {assetFolder}.");
        }
    }
}
