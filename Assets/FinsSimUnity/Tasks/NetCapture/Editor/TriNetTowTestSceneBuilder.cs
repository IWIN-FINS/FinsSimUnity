using System;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

/// <summary>Creates an isolated physical tow test without changing TriNetCapture.</summary>
public static class TriNetTowTestSceneBuilder
{
    public const string SourceScenePath = "Assets/FinsSimUnity/Tasks/NetCapture/Scenes/TriNetCapture.unity";
    public const string TestScenePath = "Assets/FinsSimUnity/Tasks/NetCapture/Scenes/TriNetTowTest.unity";
    private const string XpbdNetName = "TriNetTowXpbdCloth";
    private const int XpbdSubdivisions = 16;
    private const float TargetContactRadiusM = 0.105f;
    private const float TargetContactFriction = 0.8f;

    // All three anchors share X, making the net a vertical Y-Z plane. It can
    // therefore push/drag the Target horizontally along +X as a real trawl.
    private static readonly Vector3 LeftStart = new(-0.50f, -0.80f, -0.50f);
    private static readonly Vector3 TopStart = new(-0.50f, -0.25f, 0f);
    private static readonly Vector3 RightStart = new(-0.50f, -0.80f, 0.50f);
    // The bottle begins immediately ahead of the vertical net. The small
    // X offset avoids initial deep penetration while ensuring first contact.
    private static readonly Vector3 TargetStart = new(-0.45f, -0.52f, 0f);

    [MenuItem("FinsSim/TriNetTowTest/Create Test Scene")]
    public static void CreateTestScene()
    {
        if (AssetDatabase.LoadAssetAtPath<SceneAsset>(TestScenePath) != null)
        {
            throw new InvalidOperationException(
                $"{TestScenePath} already exists. It is intentionally not overwritten; delete it manually to create a fresh copy.");
        }

        if (!AssetDatabase.CopyAsset(SourceScenePath, TestScenePath))
        {
            throw new InvalidOperationException($"Could not copy {SourceScenePath} to {TestScenePath}.");
        }

        AssetDatabase.SaveAssets();
        Scene scene = EditorSceneManager.OpenScene(TestScenePath, OpenSceneMode.Single);
        TriNetCaptureTaskManager task = UnityEngine.Object.FindFirstObjectByType<TriNetCaptureTaskManager>();
        FishNetGenerator net = UnityEngine.Object.FindFirstObjectByType<FishNetGenerator>();
        if (task == null || net == null || task.Left == null || task.Top == null || task.Right == null || task.Target == null)
        {
            throw new InvalidOperationException("TriNetTowTest copy is missing the TriNetCapture agents, Target, or FishNetGenerator.");
        }

        SetPose(task.Left.transform, LeftStart);
        SetPose(task.Top.transform, TopStart);
        SetPose(task.Right.transform, RightStart);
        SetPose(task.Target.transform, TargetStart);
        TriNetCaptureSceneBuilder.RefreshTargetBuoyancyMesh(task.Target);
        Rigidbody targetBody = task.Target.GetComponent<Rigidbody>();
        if (targetBody != null)
        {
            targetBody.isKinematic = false;
            targetBody.linearVelocity = Vector3.zero;
            targetBody.angularVelocity = Vector3.zero;
        }

        task.enabled = false;
        task.Left.enabled = false;
        task.Top.enabled = false;
        task.Right.enabled = false;
        TriNetTowTestController controller = task.GetComponent<TriNetTowTestController>();
        if (controller == null)
        {
            controller = task.gameObject.AddComponent<TriNetTowTestController>();
        }
        controller.Configure(task, LeftStart, TopStart, RightStart);

        ReplaceLegacyNetWithXpbd(task, net);

        EditorUtility.SetDirty(task);
        EditorUtility.SetDirty(net);
        EditorUtility.SetDirty(task.Target);
        EditorUtility.SetDirty(controller);
        EditorSceneManager.MarkSceneDirty(scene);
        if (!EditorSceneManager.SaveScene(scene))
        {
            throw new InvalidOperationException($"Failed to save {TestScenePath}.");
        }

        Debug.Log("Created TriNetTowTest: three kinematic FinsROVs tow a dynamic Target through the triangular net along +X.");
    }

    [MenuItem("FinsSim/TriNetTowTest/Reset Vertical YZ Tow Formation")]
    public static void ResetVerticalYzTowFormation()
    {
        Scene scene = SceneManager.GetActiveScene();
        if (scene.path != TestScenePath)
        {
            throw new InvalidOperationException($"Open {TestScenePath} before resetting the tow formation.");
        }

        TriNetCaptureTaskManager task = UnityEngine.Object.FindFirstObjectByType<TriNetCaptureTaskManager>();
        FishNetGenerator net = UnityEngine.Object.FindFirstObjectByType<FishNetGenerator>();
        TriNetTowTestController controller = UnityEngine.Object.FindFirstObjectByType<TriNetTowTestController>();
        if (task == null || net == null || controller == null || task.Left == null || task.Top == null || task.Right == null || task.Target == null)
        {
            throw new InvalidOperationException("TriNetTowTest is missing a required towing reference.");
        }

        SetPose(task.Left.transform, LeftStart);
        SetPose(task.Top.transform, TopStart);
        SetPose(task.Right.transform, RightStart);
        SetPose(task.Target.transform, TargetStart);
        TriNetCaptureSceneBuilder.RefreshTargetBuoyancyMesh(task.Target);
        Rigidbody targetBody = task.Target.GetComponent<Rigidbody>();
        if (targetBody != null)
        {
            targetBody.linearVelocity = Vector3.zero;
            targetBody.angularVelocity = Vector3.zero;
        }

        controller.Configure(task, LeftStart, TopStart, RightStart);
        ReplaceLegacyNetWithXpbd(task, net);
        EditorUtility.SetDirty(controller);
        EditorUtility.SetDirty(net);
        EditorUtility.SetDirty(task.Target);
        EditorSceneManager.MarkSceneDirty(scene);
        EditorSceneManager.SaveScene(scene);
        Debug.Log("Reset TriNetTowTest to vertical Y-Z net formation; towing direction is +X.");
    }

    [MenuItem("FinsSim/TriNetTowTest/Replace Net With XPBD Cloth")]
    public static void ReplaceActiveSceneNetWithXpbd()
    {
        Scene scene = SceneManager.GetActiveScene();
        if (scene.path != TestScenePath)
        {
            throw new InvalidOperationException($"Open {TestScenePath} before replacing its net.");
        }

        TriNetCaptureTaskManager task = UnityEngine.Object.FindFirstObjectByType<TriNetCaptureTaskManager>();
        FishNetGenerator legacyNet = UnityEngine.Object.FindFirstObjectByType<FishNetGenerator>();
        if (task == null || legacyNet == null || task.Left == null || task.Top == null || task.Right == null || task.Target == null)
        {
            throw new InvalidOperationException("TriNetTowTest is missing its agents, Target, or legacy net generator.");
        }

        ReplaceLegacyNetWithXpbd(task, legacyNet);
        int removedMissingComponents = RemoveMissingMonoBehaviours(scene);
        EditorSceneManager.MarkSceneDirty(scene);
        if (!EditorSceneManager.SaveScene(scene))
        {
            throw new InvalidOperationException($"Failed to save {TestScenePath} after XPBD conversion.");
        }

        Debug.Log($"TriNetTowTest now uses the XPBD triangular cloth net; removedMissingComponents={removedMissingComponents}.");
    }

    private static void ReplaceLegacyNetWithXpbd(TriNetCaptureTaskManager task, FishNetGenerator legacyNet)
    {
        HabradorTriangleTowDriver existingXpbd = UnityEngine.Object.FindFirstObjectByType<HabradorTriangleTowDriver>();
        if (existingXpbd != null)
        {
            UnityEngine.Object.DestroyImmediate(existingXpbd.gameObject);
        }

        Transform legacyRoot = legacyNet.NetRoot;
        if (legacyRoot != null)
        {
            UnityEngine.Object.DestroyImmediate(legacyRoot.gameObject);
        }
        legacyNet.enabled = false;

        Rigidbody targetBody = task.Target.GetComponent<Rigidbody>();
        if (targetBody == null)
        {
            throw new InvalidOperationException("TriNetTowTest Target needs a Rigidbody for XPBD contact.");
        }

        GameObject netObject = new(XpbdNetName);
        MeshFilter meshFilter = netObject.AddComponent<MeshFilter>();
        MeshRenderer renderer = netObject.AddComponent<MeshRenderer>();
        renderer.sharedMaterial = AssetDatabase.LoadAssetAtPath<Material>(
            "Assets/FinsSimUnity/Generated/OpenSourceCloth/HabradorTriangleTowNet.mat");
        if (renderer.sharedMaterial == null)
        {
            throw new InvalidOperationException("Missing XPBD net material: Assets/FinsSimUnity/Generated/OpenSourceCloth/HabradorTriangleTowNet.mat.");
        }

        TriangleTowLattice lattice = TriangleTowLattice.Create(
            task.Top.transform.position,
            task.Left.transform.position,
            task.Right.transform.position,
            XpbdSubdivisions);
        meshFilter.sharedMesh = TriangleTowLattice.CreateMesh("TriNetTowXpbdMesh", lattice);

        HabradorTriangleTowDriver xpbd = netObject.AddComponent<HabradorTriangleTowDriver>();
        xpbd.topAnchor = task.Top.transform;
        xpbd.leftAnchor = task.Left.transform;
        xpbd.rightAnchor = task.Right.transform;
        xpbd.subdivisions = XpbdSubdivisions;
        xpbd.solverSubsteps = 8;
        xpbd.stretchingCompliance = 0f;
        xpbd.bendingCompliance = 0.00005f;
        xpbd.velocityDamping = 2f;
        xpbd.dynamicParticleInverseMassScale = 0.0002f;
        xpbd.enableTutorialFloorCollision = false;
        xpbd.targetBody = targetBody;
        xpbd.targetCapsuleCollider = task.Target.GetComponent<CapsuleCollider>();
        xpbd.targetRadiusM = TargetContactRadiusM;
        xpbd.targetFriction = TargetContactFriction;
        xpbd.topParticleIndex = lattice.TopIndex;
        xpbd.leftParticleIndex = lattice.LeftIndex;
        xpbd.rightParticleIndex = lattice.RightIndex;

        EditorUtility.SetDirty(legacyNet);
        EditorUtility.SetDirty(xpbd);
    }

    private static int RemoveMissingMonoBehaviours(Scene scene)
    {
        int removed = 0;
        foreach (GameObject root in scene.GetRootGameObjects())
        {
            foreach (Transform transform in root.GetComponentsInChildren<Transform>(true))
            {
                removed += GameObjectUtility.RemoveMonoBehavioursWithMissingScript(transform.gameObject);
            }
        }

        return removed;
    }

    private static void SetPose(Transform transform, Vector3 position)
    {
        transform.SetPositionAndRotation(position, Quaternion.identity);
        Rigidbody body = transform.GetComponent<Rigidbody>();
        if (body != null)
        {
            body.position = position;
            body.rotation = Quaternion.identity;
            body.linearVelocity = Vector3.zero;
            body.angularVelocity = Vector3.zero;
        }
    }
}
