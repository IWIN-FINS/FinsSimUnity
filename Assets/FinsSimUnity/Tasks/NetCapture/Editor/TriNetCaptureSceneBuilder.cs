using System;
using System.IO;
using NWH.DWP2.WaterObjects;
using Unity.MLAgents.Areas;
using Unity.MLAgents.Policies;
using Unity.MLAgents.Sensors;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

/// <summary>Configures the scene-local physical triangular net for TriNetCapture.</summary>
public static class TriNetCaptureSceneBuilder
{
    public const string ScenePath = "Assets/FinsSimUnity/Tasks/NetCapture/Scenes/TriNetCapture.unity";

    // Fits the triangular ROV formation within the one-metre-deep pool while
    // providing approximately twice the original capture area.
    private const float TriangleSideLength = 1.0f;
    private const float InitialRovFormationX = -1.20f;
    private const float InitialTopRovY = -0.20f;
    private const float InitialLowerRovsY = -0.80f;
    private const int TriangleSubdivisions = 14;
    private const int TargetBuoyancyTriangleCount = 32;
    // Single source of truth for both the serialized recovery-volume geometry
    // and the scene validation below. Edit these values, then run Prepare
    // Scene to apply them to TriNetCapture.unity.
    private static readonly Vector3 RecoveryGoalAnchorLocal = new(-1.25f, 0f, 0f);
    private static readonly Vector3 RecoveryGoalVolumeCenter = new(0f, -0.125f, 0f);
    private static readonly Vector3 RecoveryGoalVolumeSize = new(0.6f, 0.6f, 0.5f);
    // Unity's built-in capsule is one unit wide and two units tall.
    // This is a 6.5 cm x 20 cm 500 mL-style bottle proxy.
    private static readonly Vector3 TargetBottleScale = new(0.065f, 0.10f, 0.065f);
    private const string XpbdNetName = "TriNetCaptureXpbdCloth";
    private const string XpbdNetMaterialPath = "Assets/FinsSimUnity/Generated/OpenSourceCloth/HabradorTriangleTowNet.mat";
    // Extra material belongs inside the net face, never beyond its three
    // FinsROV attachment points. This is the maximum -X rest-shape pocket at
    // the triangle centre; it falls smoothly to zero on every boundary edge.
    private const float XpbdNetRearwardPocketDepthM = 0.00f;

    [MenuItem("FinsSim/TriNetCapture/Prepare Scene")]
    public static void PrepareTriNetCaptureScene()
    {
        Scene scene = EditorSceneManager.OpenScene(ScenePath, OpenSceneMode.Single);
        TrainingAreaReplicator replicator = UnityEngine.Object.FindFirstObjectByType<TrainingAreaReplicator>();
        if (replicator == null || replicator.baseArea == null)
        {
            throw new InvalidOperationException("TriNetCapture must contain a TrainingAreaReplicator with a base area.");
        }

        GameObject area = replicator.baseArea;
        EnsureTimeScaleController(area);
        CatchAreaManager manager = UnityEngine.Object.FindFirstObjectByType<CatchAreaManager>(FindObjectsInactive.Include);
        TriNetCaptureTaskManager task = area.GetComponent<TriNetCaptureTaskManager>();
        TriNetTarget target = task != null ? task.Target : null;
        if (task == null || target == null)
        {
            throw new InvalidOperationException("TriNetCapture is missing its task manager or Target.");
        }

        TriNetCaptureAgent[] existingAgents = area.GetComponentsInChildren<TriNetCaptureAgent>(true);
        if (existingAgents.Length != 3)
        {
            throw new InvalidOperationException("TriNetCapture requires exactly three TriNetCaptureAgent components in the scene.");
        }

        TriNetCaptureAgent left = FindAgent(existingAgents, "FinsROV_Fossen_Left");
        TriNetCaptureAgent top = FindAgent(existingAgents, "FinsROV_Fossen_Top");
        TriNetCaptureAgent right = FindAgent(existingAgents, "FinsROV_Fossen_Right");
        TriNetCaptureAgent[] agents = { left, top, right };

        // Base-area origin is world origin. TrainingAreaReplicator offsets only its clones.
        SetWorldPosition(top.transform, new Vector3(InitialRovFormationX, InitialTopRovY, 0f));
        SetWorldPosition(left.transform, new Vector3(InitialRovFormationX, InitialLowerRovsY, -TriangleSideLength * 0.5f));
        SetWorldPosition(right.transform, new Vector3(InitialRovFormationX, InitialLowerRovsY, TriangleSideLength * 0.5f));
        // The copied 3Chase1 agents inherited heterogeneous headings (the
        // former Herder was 180 degrees from both Netters). A triangular-net
        // transport task needs one common controller basis: otherwise the
        // same centroid translation becomes three opposing thrust commands.
        SetWorldRotation(top.transform, Quaternion.identity);
        SetWorldRotation(left.transform, Quaternion.identity);
        SetWorldRotation(right.transform, Quaternion.identity);
        SetWorldPosition(target.transform, new Vector3(1.35f, -0.40f, 0f));
        target.transform.localScale = TargetBottleScale;

        if (manager != null)
        {
            manager.captureCriterion = CatchAreaManager.CaptureCriterion.NetSurfaceDistance;
            manager.netSurfaceCaptureDistance = 0.10f;
            manager.netSurfaceCaptureHoldTime = 0.20f;
        }

        // TriNetCapture owns the episode/reward lifecycle. Keep these legacy
        // components serialized for the parallel-area rebinder, but inactive.
        if (manager != null)
        {
            manager.enabled = false;
        }
        ThreeChaseOneBaselineController legacyBaseline = area.GetComponentInChildren<ThreeChaseOneBaselineController>(true);
        if (legacyBaseline != null)
        {
            legacyBaseline.enabled = false;
        }
        foreach (BehaviorParameters targetBehavior in target.GetComponentsInChildren<BehaviorParameters>(true))
        {
            targetBehavior.enabled = false;
            EditorUtility.SetDirty(targetBehavior);
        }

        target.Configure(target.GetComponent<Rigidbody>(), target.GetComponent<Collider>());
        RefreshTargetBuoyancyMesh(target);

        TriNetRecoveryGoal recoveryGoal = EnsureRecoveryGoal(area.transform);

        HabradorTriangleTowDriver xpbdNet = ReplaceWithXpbdNet(area.transform, target, left, top, right);
        if (manager != null)
        {
            manager.net = xpbdNet.transform;
        }
        task.Configure(area.transform, left, top, right, target, recoveryGoal, xpbdNet);
        if (manager != null)
        {
            EditorUtility.SetDirty(manager);
        }
        EditorUtility.SetDirty(target);
        EditorUtility.SetDirty(recoveryGoal);
        EditorUtility.SetDirty(task);
        foreach (TriNetCaptureAgent agent in agents)
        {
            BehaviorParameters behavior = agent.GetComponent<BehaviorParameters>();
            if (behavior == null)
            {
                throw new InvalidOperationException($"{agent.name} has no BehaviorParameters.");
            }
            behavior.BehaviorName = agent == left
                ? "FinsROV_Fossen_Left"
                : agent == top
                    ? "FinsROV_Fossen_Top"
                    : "FinsROV_Fossen_Right";
            behavior.BrainParameters.VectorObservationSize = TriNetCaptureAgent.VectorObservationSize;
            EditorUtility.SetDirty(agent);
            EditorUtility.SetDirty(behavior);
        }

        ValidateTriNetCaptureScene(scene, task, manager, xpbdNet);
        EditorSceneManager.MarkSceneDirty(scene);
        if (!EditorSceneManager.SaveScene(scene))
        {
            throw new IOException($"Failed to save {ScenePath}.");
        }

        AssetDatabase.SaveAssets();
        AssetDatabase.Refresh();
        Debug.Log(
            $"Prepared TriNetCapture with static XPBD Reuleaux triangular cloth, " +
            $"rearward rest-pocket={XpbdNetRearwardPocketDepthM:F2}m.");
    }

    [MenuItem("FinsSim/TriNetCapture/Rebuild Static XPBD Cloth")]
    public static void RebuildStaticXpbdCloth()
    {
        if (EditorApplication.isPlayingOrWillChangePlaymode)
        {
            throw new InvalidOperationException("Stop Play Mode before rebuilding the TriNetCapture XPBD cloth.");
        }

        Scene scene = SceneManager.GetActiveScene();
        TriNetCaptureTaskManager task = UnityEngine.Object.FindFirstObjectByType<TriNetCaptureTaskManager>();
        if (task == null || task.Target == null || task.Left == null || task.Top == null || task.Right == null)
        {
            throw new InvalidOperationException("TriNetCapture is missing task references required for XPBD cloth.");
        }

        HabradorTriangleTowDriver xpbdNet = ReplaceWithXpbdNet(
            task.transform, task.Target, task.Left, task.Top, task.Right);
        task.Configure(task.transform, task.Left, task.Top, task.Right, task.Target, task.RecoveryGoal, xpbdNet);
        EditorSceneManager.MarkSceneDirty(scene);
        EditorSceneManager.SaveScene(scene);
        Debug.Log("Rebuilt TriNetCapture static XPBD cloth.");
    }

    [MenuItem("FinsSim/TriNetCapture/Validate Scene")]
    public static void ValidateTriNetCaptureScene()
    {
        Scene scene = EditorSceneManager.OpenScene(ScenePath, OpenSceneMode.Single);
        TrainingAreaReplicator replicator = UnityEngine.Object.FindFirstObjectByType<TrainingAreaReplicator>();
        GameObject area = replicator != null ? replicator.baseArea : null;
        CatchAreaManager manager = area != null ? area.GetComponentInChildren<CatchAreaManager>(true) : null;
        HabradorTriangleTowDriver xpbdNet = area != null ? area.GetComponentInChildren<HabradorTriangleTowDriver>(true) : null;
        TriNetCaptureTaskManager task = area != null ? area.GetComponent<TriNetCaptureTaskManager>() : null;
        ValidateTriNetCaptureScene(scene, task, manager, xpbdNet);
        Debug.Log("TriNetCapture validation passed.");
    }

    [MenuItem("FinsSim/TriNetCapture/Regenerate Target Buoyancy Mesh")]
    public static void RegenerateTargetBuoyancyMesh()
    {
        if (EditorApplication.isPlayingOrWillChangePlaymode)
        {
            throw new InvalidOperationException("Stop Play Mode before regenerating the TriNetCapture Target buoyancy mesh.");
        }

        Scene scene = SceneManager.GetActiveScene();
        if (scene.path != ScenePath)
        {
            throw new InvalidOperationException(
                $"Expected active scene {ScenePath}, found {scene.path}. Open TriNetCapture before regenerating its Target mesh.");
        }

        TriNetTarget target = UnityEngine.Object.FindFirstObjectByType<TriNetTarget>(FindObjectsInactive.Include);
        WaterObject waterObject = target != null ? target.GetComponent<WaterObject>() : null;
        if (waterObject == null)
        {
            throw new InvalidOperationException("TriNetCapture Target needs a DWP2 WaterObject to generate buoyancy.");
        }

        target.transform.localScale = TargetBottleScale;
        RefreshTargetBuoyancyMesh(target);

        EditorSceneManager.MarkSceneDirty(scene);
        if (!EditorSceneManager.SaveScene(scene))
        {
            throw new IOException($"Failed to save {ScenePath} after regenerating the Target buoyancy mesh.");
        }

        AssetDatabase.SaveAssets();
    }

    /// <summary>
    /// Rebuild the exact DWP2 mesh used for buoyancy and then re-neutralize
    /// the Rigidbody mass from that mesh's volume. Any scene/tool that edits
    /// Target's MeshFilter or its mesh-processing options must call this.
    /// </summary>
    public static void RefreshTargetBuoyancyMesh(TriNetTarget target)
    {
        WaterObject waterObject = target.GetComponent<WaterObject>();
        if (waterObject == null)
        {
            throw new InvalidOperationException("TriNetCapture Target needs a DWP2 WaterObject to generate buoyancy.");
        }

        // Small, high-density render meshes fall below DWP2's fixed per-triangle
        // area threshold. Keep a coarse, closed simulation mesh instead of using
        // the visual capsule tessellation.
        waterObject.simplifyMesh = true;
        waterObject.targetTriangleCount = TargetBuoyancyTriangleCount;
        waterObject.convexifyMesh = false;
        waterObject.weldColocatedVertices = true;
        waterObject.GenerateSimMesh();

        int generatedTriangleCount = waterObject.serializedSimulationMesh?.triangles?.Length / 3 ?? 0;
        if (generatedTriangleCount <= 0)
        {
            throw new InvalidOperationException("DWP2 generated an empty Target buoyancy mesh.");
        }

        MassFromVolume massFromVolume = target.GetComponent<MassFromVolume>();
        if (massFromVolume != null)
        {
            // Neutral density must be calculated from the mesh DWP2 actually
            // integrates, not from the much denser visual capsule tessellation.
            massFromVolume.CalculateAndApplyFromDensity(waterObject.fluidDensity);
            EditorUtility.SetDirty(massFromVolume);
        }

        EditorUtility.SetDirty(waterObject);
        EditorUtility.SetDirty(waterObject.targetRigidbody);
        Debug.Log(
            $"Regenerated TriNetCapture Target buoyancy mesh: {generatedTriangleCount} triangles, " +
            $"neutral mass={waterObject.targetRigidbody.mass:F6} kg.");
    }

    private static HabradorTriangleTowDriver ReplaceWithXpbdNet(
        Transform area,
        TriNetTarget target,
        TriNetCaptureAgent left,
        TriNetCaptureAgent top,
        TriNetCaptureAgent right)
    {
        HabradorTriangleTowDriver existingXpbd = area.GetComponentInChildren<HabradorTriangleTowDriver>(true);
        if (existingXpbd != null)
        {
            UnityEngine.Object.DestroyImmediate(existingXpbd.gameObject);
        }

        FishNetGenerator legacyGenerator = area.GetComponentInChildren<FishNetGenerator>(true);
        if (legacyGenerator != null)
        {
            UnityEngine.Object.DestroyImmediate(legacyGenerator.gameObject);
        }

        GameObject netObject = new(XpbdNetName);
        netObject.transform.SetParent(area, false);
        MeshFilter meshFilter = netObject.AddComponent<MeshFilter>();
        MeshRenderer renderer = netObject.AddComponent<MeshRenderer>();
        renderer.sharedMaterial = AssetDatabase.LoadAssetAtPath<Material>(XpbdNetMaterialPath);
        if (renderer.sharedMaterial == null)
        {
            throw new InvalidOperationException($"Missing XPBD cloth material: {XpbdNetMaterialPath}");
        }

        TriangleTowLattice lattice = TriangleTowLattice.CreateReuleauxRearwardPocket(
            top.transform.position,
            left.transform.position,
            right.transform.position,
            TriangleSubdivisions,
            XpbdNetRearwardPocketDepthM);
        meshFilter.sharedMesh = TriangleTowLattice.CreateMesh("TriNetCaptureXpbdMesh", lattice);

        HabradorTriangleTowDriver xpbdNet = netObject.AddComponent<HabradorTriangleTowDriver>();
        xpbdNet.topAnchor = top.transform;
        xpbdNet.leftAnchor = left.transform;
        xpbdNet.rightAnchor = right.transform;
        xpbdNet.subdivisions = TriangleSubdivisions;
        xpbdNet.solverSubsteps = 8;
        xpbdNet.stretchingCompliance = 0f;
        xpbdNet.bendingCompliance = 0.00005f;
        xpbdNet.velocityDamping = 2f;
        xpbdNet.dynamicParticleInverseMassScale = 0.0002f;
        xpbdNet.anchorSpringRestLengthM = 0.10f;
        xpbdNet.anchorSpringCompliance = 0.000001f;
        xpbdNet.enableTutorialFloorCollision = false;
        xpbdNet.targetBody = target.Rigidbody;
        xpbdNet.targetCapsuleCollider = target.GetComponent<CapsuleCollider>();
        xpbdNet.targetFriction = 0.8f;
        xpbdNet.topParticleIndex = lattice.TopIndex;
        xpbdNet.leftParticleIndex = lattice.LeftIndex;
        xpbdNet.rightParticleIndex = lattice.RightIndex;

        EditorUtility.SetDirty(netObject);
        EditorUtility.SetDirty(xpbdNet);
        return xpbdNet;
    }

    private static void ValidateTriNetCaptureScene(
        Scene scene,
        TriNetCaptureTaskManager task,
        CatchAreaManager manager,
        HabradorTriangleTowDriver xpbdNet)
    {
        TriNetCaptureAgent left = task != null ? task.Left : null;
        TriNetCaptureAgent top = task != null ? task.Top : null;
        TriNetCaptureAgent right = task != null ? task.Right : null;
        TriNetTarget target = task != null ? task.Target : null;
        TriNetRecoveryGoal goal = task != null ? task.RecoveryGoal : null;
        BehaviorParameters[] targetBehaviors = target != null
            ? target.GetComponentsInChildren<BehaviorParameters>(true) : Array.Empty<BehaviorParameters>();
        if (!scene.IsValid() || xpbdNet == null || task == null ||
            left == null || top == null || right == null || target == null)
        {
            throw new InvalidOperationException("TriNetCapture validation failed: incomplete scene references.");
        }

        if (task.GetComponent<TriNetCaptureTimeScaleController>() == null)
        {
            throw new InvalidOperationException(
                "TriNetCapture validation failed: runtime time-scale controller is missing.");
        }

        if ((manager != null && manager.enabled) || !task.enabled || goal == null ||
            Array.Exists(targetBehaviors, behavior => behavior.enabled))
        {
            throw new InvalidOperationException("TriNetCapture validation failed: passive Target or task lifecycle is not configured.");
        }

        Vector3 anchor = goal.Anchor;
        if ((anchor - RecoveryGoalAnchorLocal).sqrMagnitude > 1e-6f || goal.RecoveryVolume == null ||
            goal.RecoveryVolume.size != RecoveryGoalVolumeSize ||
            goal.RecoveryVolume.center != RecoveryGoalVolumeCenter)
        {
            throw new InvalidOperationException("TriNetCapture validation failed: recovery volume does not match the submerged left-side goal.");
        }

        if (xpbdNet.topAnchor != top.transform || xpbdNet.leftAnchor != left.transform ||
            xpbdNet.rightAnchor != right.transform || !xpbdNet.TryGetTriangleVertices(out _, out _, out _))
        {
            throw new InvalidOperationException("TriNetCapture validation failed: XPBD cloth anchors are invalid.");
        }

        float expectedSide = (Vector3.Distance(top.transform.position, left.transform.position) +
            Vector3.Distance(left.transform.position, right.transform.position) +
            Vector3.Distance(right.transform.position, top.transform.position)) / 3f;
        if (Mathf.Abs(task.DesiredSideLength - expectedSide) > 0.002f)
        {
            throw new InvalidOperationException(
                "TriNetCapture validation failed: task shape reference must match the XPBD cloth side.");
        }

        AssertInsidePool(left.transform.position, left.name);
        AssertInsidePool(top.transform.position, top.name);
        AssertInsidePool(right.transform.position, right.name);
        AssertInsidePool(target.transform.position, target.name);

        if (!Mathf.Approximately(top.transform.position.y, InitialTopRovY) ||
            !Mathf.Approximately(left.transform.position.y, InitialLowerRovsY) ||
            !Mathf.Approximately(right.transform.position.y, InitialLowerRovsY))
        {
            throw new InvalidOperationException("TriNetCapture validation failed: initial ROV anchor heights do not match the configured formation.");
        }

        Quaternion referenceRotation = top.transform.rotation;
        foreach (TriNetCaptureAgent agent in new[] { left, right })
        {
            if (Quaternion.Angle(referenceRotation, agent.transform.rotation) > 0.01f)
            {
                throw new InvalidOperationException(
                    "TriNetCapture validation failed: all ROVs must start with one shared controller heading.");
            }
        }

        foreach (TriNetCaptureAgent agent in new[] { left, top, right })
        {
            BehaviorParameters behavior = agent.GetComponent<BehaviorParameters>();
            if (behavior == null || behavior.BrainParameters.VectorObservationSize != TriNetCaptureAgent.VectorObservationSize)
            {
                throw new InvalidOperationException(
                    $"TriNetCapture validation failed: agent observation size must be " +
                    $"{TriNetCaptureAgent.VectorObservationSize}D " +
                    $"({TriNetCaptureAgent.ActorObservationSize}D actor + " +
                    $"{TriNetCaptureAgent.PrivilegedStateSize}D privileged state).");
            }
        }
    }

    private static TriNetRecoveryGoal EnsureRecoveryGoal(Transform area)
    {
        Transform existing = area.Find("TriNetRecoveryGoal");
        GameObject goalObject;
        if (existing == null)
        {
            goalObject = new GameObject("TriNetRecoveryGoal");
            goalObject.transform.SetParent(area, false);
        }
        else
        {
            goalObject = existing.gameObject;
        }
        goalObject.transform.localPosition = RecoveryGoalAnchorLocal;
        goalObject.transform.localRotation = Quaternion.identity;
        BoxCollider volume = goalObject.GetComponent<BoxCollider>();
        if (volume == null)
        {
            volume = goalObject.AddComponent<BoxCollider>();
        }
        volume.isTrigger = true;
        volume.center = RecoveryGoalVolumeCenter;
        volume.size = RecoveryGoalVolumeSize;
        TriNetRecoveryGoal goal = goalObject.GetComponent<TriNetRecoveryGoal>();
        if (goal == null)
        {
            goal = goalObject.AddComponent<TriNetRecoveryGoal>();
        }
        goal.Configure(volume);
        EditorUtility.SetDirty(goalObject);
        return goal;
    }

    private static void EnsureTimeScaleController(GameObject area)
    {
        TriNetCaptureTimeScaleController controller = area.GetComponent<TriNetCaptureTimeScaleController>();
        if (controller == null)
        {
            controller = area.AddComponent<TriNetCaptureTimeScaleController>();
        }
        EditorUtility.SetDirty(controller);
    }

    private static void SetWorldPosition(Transform target, Vector3 position)
    {
        target.position = position;
        EditorUtility.SetDirty(target);
    }

    private static void SetWorldRotation(Transform target, Quaternion rotation)
    {
        target.rotation = rotation;
        EditorUtility.SetDirty(target);
    }

    private static TriNetCaptureAgent FindAgent(TriNetCaptureAgent[] agents, string gameObjectName)
    {
        TriNetCaptureAgent agent = Array.Find(
            agents,
            candidate => candidate != null && candidate.gameObject.name == gameObjectName);
        if (agent == null)
        {
            throw new InvalidOperationException($"TriNetCapture requires a ROV named {gameObjectName}.");
        }
        return agent;
    }

    private static void AssertInsidePool(Vector3 position, string objectName)
    {
        if (position.x < -2f || position.x > 2f || position.y < -1f || position.y > 0f ||
            position.z < -1f || position.z > 1f)
        {
            throw new InvalidOperationException($"TriNetCapture validation failed: {objectName} is outside the pool bounds.");
        }
    }
}
