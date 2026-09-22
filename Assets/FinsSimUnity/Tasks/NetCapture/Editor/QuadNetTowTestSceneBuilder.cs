using System;
using Unity.MLAgents.Policies;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

/// <summary>Builds a four-ROV rectangular-net tow test without touching the triangular test scene.</summary>
public static class QuadNetTowTestSceneBuilder
{
    private const string SourceScenePath = "Assets/FinsSimUnity/Tasks/NetCapture/Scenes/TriNetTowTest.unity";
    private const string TestScenePath = "Assets/FinsSimUnity/Tasks/NetCapture/Scenes/QuadNetTowTest.unity";
    private const string BlueprintPath = "Assets/FinsSimUnity/Generated/FishNet/QuadNetTowTest_RopeBlueprints.asset";

    private static readonly Vector3 TopLeft = new(-0.50f, -0.25f, -0.55f);
    private static readonly Vector3 TopRight = new(-0.50f, -0.25f, 0.55f);
    private static readonly Vector3 BottomLeft = new(-0.50f, -0.85f, -0.55f);
    private static readonly Vector3 BottomRight = new(-0.50f, -0.85f, 0.55f);
    private static readonly Vector3 TargetStart = new(-0.45f, -0.55f, 0f);

    [MenuItem("FinsSim/QuadNetTowTest/Create Test Scene")]
    public static void CreateTestScene()
    {
        if (AssetDatabase.LoadAssetAtPath<SceneAsset>(TestScenePath) != null)
        {
            throw new InvalidOperationException($"{TestScenePath} already exists; it will not be overwritten.");
        }

        if (!AssetDatabase.CopyAsset(SourceScenePath, TestScenePath))
        {
            throw new InvalidOperationException($"Could not copy {SourceScenePath} to {TestScenePath}.");
        }

        AssetDatabase.SaveAssets();
        Scene scene = EditorSceneManager.OpenScene(TestScenePath, OpenSceneMode.Single);
        TriNetCaptureTaskManager task = UnityEngine.Object.FindFirstObjectByType<TriNetCaptureTaskManager>();
        FishNetGenerator net = UnityEngine.Object.FindFirstObjectByType<FishNetGenerator>();
        TriNetTowTestController triangleController = UnityEngine.Object.FindFirstObjectByType<TriNetTowTestController>();
        if (task == null || net == null || task.Left == null || task.Top == null || task.Right == null || task.Target == null)
        {
            throw new InvalidOperationException("QuadNetTowTest copy is missing required TriNet references.");
        }

        Transform bottomRight = UnityEngine.Object.Instantiate(task.Right.gameObject, task.Right.transform.parent).transform;
        bottomRight.name = "FinsROV_Fossen_BottomRight";
        TriNetCaptureAgent clonedAgent = bottomRight.GetComponent<TriNetCaptureAgent>();
        if (clonedAgent != null)
        {
            clonedAgent.enabled = false;
        }
        foreach (BehaviorParameters behavior in bottomRight.GetComponentsInChildren<BehaviorParameters>(true))
        {
            behavior.enabled = false;
        }
        if (triangleController != null)
        {
            UnityEngine.Object.DestroyImmediate(triangleController);
        }

        Transform[] rovs = { task.Left.transform, task.Top.transform, task.Right.transform, bottomRight };
        Vector3[] starts = { TopLeft, TopRight, BottomLeft, BottomRight };
        for (int index = 0; index < rovs.Length; index++)
        {
            SetPose(rovs[index], starts[index]);
        }
        SetPose(task.Target.transform, TargetStart);
        Rigidbody targetBody = task.Target.GetComponent<Rigidbody>();
        if (targetBody != null)
        {
            targetBody.isKinematic = false;
        }

        task.enabled = false;
        task.Left.enabled = false;
        task.Top.enabled = false;
        task.Right.enabled = false;
        net.netTopology = FishNetGenerator.NetTopology.QuadrilateralFourAnchor;
        net.netter1 = rovs[0];
        net.netter2 = rovs[1];
        net.netter3 = rovs[2];
        net.netter4 = rovs[3];
        net.netter1MarkPosition = rovs[0];
        net.netter2MarkPosition = rovs[1];
        net.netter3MarkPosition = rovs[2];
        net.netter4MarkPosition = rovs[3];
        net.resolution = new Vector2Int(16, 10);
        // This scene isolates distance constraints. Do not let dense-rope
        // self contacts or a soft UUV pin settle the initially planar net.
        net.useSelfCollisions = false;
        net.useParticleCollisions = false;
        net.uuvAttachmentCompliance = 0f;
        net.gridRopeRestLengthScale = 0.97f;
        net.GenerateStaticNetForEditor(BlueprintPath);

        QuadNetTowTestController controller = task.GetComponent<QuadNetTowTestController>();
        if (controller == null)
        {
            controller = task.gameObject.AddComponent<QuadNetTowTestController>();
        }
        controller.Configure(rovs, starts);

        EditorUtility.SetDirty(task);
        EditorUtility.SetDirty(net);
        EditorUtility.SetDirty(task.Target);
        EditorUtility.SetDirty(controller);
        EditorSceneManager.MarkSceneDirty(scene);
        if (!EditorSceneManager.SaveScene(scene))
        {
            throw new InvalidOperationException($"Failed to save {TestScenePath}.");
        }

        Debug.Log("Created QuadNetTowTest: four FinsROVs tow a dense vertical quadrilateral net along +X.");
    }

    [MenuItem("FinsSim/QuadNetTowTest/Rebuild Net Without Self Collision")]
    public static void RebuildNetWithoutSelfCollision()
    {
        Scene scene = EditorSceneManager.OpenScene(TestScenePath, OpenSceneMode.Single);
        FishNetGenerator net = UnityEngine.Object.FindFirstObjectByType<FishNetGenerator>();
        if (net == null)
        {
            throw new InvalidOperationException("QuadNetTowTest is missing FishNetGenerator.");
        }

        net.useSelfCollisions = false;
        net.useParticleCollisions = false;
        net.uuvAttachmentCompliance = 0f;
        net.gridRopeRestLengthScale = 0.97f;
        net.GenerateStaticNetForEditor(BlueprintPath);

        EditorUtility.SetDirty(net);
        EditorSceneManager.MarkSceneDirty(scene);
        if (!EditorSceneManager.SaveScene(scene))
        {
            throw new InvalidOperationException($"Failed to save {TestScenePath}.");
        }

        Debug.Log("Rebuilt QuadNetTowTest net with self collision disabled and rigid UUV attachments.");
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
