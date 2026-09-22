using System;
using System.Collections.Generic;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;
using JointCloth = OpenSourceCloth.UnitySimplePhysics.UnitySimplePhysicsCloth;
using JointClothGenerator = OpenSourceCloth.UnitySimplePhysics.ClothGenerator;

/// <summary>Builds comparable soccer-goal collision tests for three open-source cloth implementations.</summary>
public static class OpenSourceClothGoalSceneBuilder
{
    private const string SceneFolder = "Assets/FinsSimUnity/Tasks/Cloth/Scenes/OpenSourceCloth";
    private const string GeneratedFolder = "Assets/FinsSimUnity/Generated/OpenSourceCloth";
    private const float GoalWidth = 5f;
    private const float GoalHeight = 3f;
    private const float GoalBottomY = 0.25f;

    [MenuItem("FinsSim/Open Source Cloth/Create All Soccer Goal Tests")]
    public static void CreateAllSoccerGoalTests()
    {
        EnsureFolder("Assets/Scenes", "OpenSourceCloth");
        EnsureFolder("Assets/Generated", "OpenSourceCloth");
        BuildUnitySimplePhysicsScene();
        BuildHabradorScene();
        BuildClothBehaviourScene();
        AssetDatabase.SaveAssets();
        Debug.Log("Created three open-source soccer-goal tests in Assets/FinsSimUnity/Tasks/Cloth/Scenes/OpenSourceCloth.");
    }

    private static void BuildUnitySimplePhysicsScene()
    {
        Scene scene = CreateBaseScene("UnitySimplePhysics: Rigidbody joints + dynamic MeshCollider");
        Transform goal = CreateGoalFrame();

        GameObject generatorObject = new("UnitySimplePhysics_JointCloth");
        generatorObject.transform.SetParent(goal, false);
        generatorObject.transform.position = new Vector3(-GoalWidth * 0.5f, GoalBottomY + GoalHeight, 0f);
        JointClothGenerator generator = generatorObject.AddComponent<JointClothGenerator>();
        generator.prefab = GetOrCreateNetPointPrefab();
        generator.rows = 13;
        generator.cols = 21;
        generator.spacing = GoalWidth / (generator.cols - 1);
        generator.lineMaterial = GetOrCreateMaterial("UnitySimpleLine", new Color(0.05f, 0.75f, 0.95f));
        generator.clothMaterial = GetOrCreateMaterial("UnitySimpleNet", new Color(0.04f, 0.42f, 0.65f, 0.55f));
        generator.angularStrength = 350f;
        generator.angularDamping = 25f;
        generator.projectionDistance = 0.005f;
        generator.GenerateCloth();

        JointCloth cloth = generatorObject.GetComponentInChildren<JointCloth>();
        cloth.showLineRenderers = true;
        cloth.showMesh = true;
        cloth.addMeshCollider = true;
        cloth.relayCollisionImpulse = true;
        cloth.impulseNearestN = 6;
        cloth.impulseScale = 0.8f;
        PinClothBoundary(cloth);

        CreateBall("UnitySimplePhysics_Ball");
        SaveScene(scene, "UnitySimplePhysicsSoccerGoal.unity");
    }

    private static void BuildHabradorScene()
    {
        Scene scene = CreateBaseScene("Habrador: XPBD triangular cloth + MeshCollider comparison proxy");
        Transform goal = CreateGoalFrame();

        GameObject clothObject = new("Habrador_XPBD_Cloth");
        clothObject.transform.SetParent(goal, false);
        MeshRenderer renderer = clothObject.AddComponent<MeshRenderer>();
        renderer.sharedMaterial = GetOrCreateMaterial("HabradorNet", new Color(0.1f, 0.85f, 0.35f, 0.58f));
        clothObject.AddComponent<MeshFilter>();
        clothObject.AddComponent<MeshCollider>();
        clothObject.AddComponent<DynamicMeshColliderSynchronizer>();
        HabradorGoalClothDriver driver = clothObject.AddComponent<HabradorGoalClothDriver>();
        driver.columns = 25;
        driver.rows = 15;
        driver.width = GoalWidth;
        driver.height = GoalHeight;
        driver.bottomY = GoalBottomY;

        CreateBall("Habrador_Ball");
        SaveScene(scene, "HabradorXPBDSoccerGoal.unity");
    }

    private static void BuildClothBehaviourScene()
    {
        Scene scene = CreateBaseScene("ClothBehaviour: spring cloth + MeshCollider comparison proxy");
        Transform goal = CreateGoalFrame();
        GameObject ball = CreateBall("ClothBehaviour_Ball");

        GameObject clothObject = new("ClothBehaviour_SpringCloth");
        clothObject.transform.SetParent(goal, false);
        MeshFilter filter = clothObject.AddComponent<MeshFilter>();
        filter.sharedMesh = CreateGoalMesh(25, 15);
        MeshRenderer renderer = clothObject.AddComponent<MeshRenderer>();
        renderer.sharedMaterial = GetOrCreateMaterial("ClothBehaviourNet", new Color(0.95f, 0.48f, 0.06f, 0.58f));
        clothObject.AddComponent<MeshCollider>();
        clothObject.AddComponent<DynamicMeshColliderSynchronizer>();

        ClothBehaviour cloth = clothObject.AddComponent<ClothBehaviour>();
        cloth.m_Paused = false;
        cloth.m_Gravity = Vector3.zero;
        cloth.m_MeshMass = 2f;
        cloth.m_NodeDamping = 0.45f;
        cloth.m_SpringDamping = 0.6f;
        cloth.m_TractionStiffness = 240f;
        cloth.m_FlexionStiffness = 170f;
        cloth.m_SolvingMethod = ClothBehaviour.Solver.Simplectic;
        cloth.m_FixingByTexture = false;
        cloth.m_Fixers = CreateCornerFixers(goal);
        cloth.m_CanCollide = true;
        cloth.m_CollidingMeshes = new List<GameObject> { ball };
        cloth.m_PenaltyStiffness = 800f;
        cloth.m_CollisionOffsetDistance = 0.08f;

        SaveScene(scene, "ClothBehaviourSoccerGoal.unity");
    }

    private static Scene CreateBaseScene(string title)
    {
        Scene scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
        RenderSettings.ambientMode = UnityEngine.Rendering.AmbientMode.Flat;
        RenderSettings.ambientLight = new Color(0.34f, 0.34f, 0.38f);

        GameObject lightObject = new("Directional Light");
        Light light = lightObject.AddComponent<Light>();
        light.type = LightType.Directional;
        light.intensity = 2.5f;
        lightObject.transform.rotation = Quaternion.Euler(48f, -30f, 0f);

        GameObject cameraObject = new("Main Camera");
        Camera camera = cameraObject.AddComponent<Camera>();
        camera.tag = "MainCamera";
        cameraObject.transform.SetPositionAndRotation(new Vector3(8f, 5f, -12f), Quaternion.Euler(14f, -32f, 0f));
        camera.fieldOfView = 52f;

        GameObject floor = GameObject.CreatePrimitive(PrimitiveType.Cube);
        floor.name = "Pitch";
        floor.transform.SetPositionAndRotation(new Vector3(0f, -0.1f, 1f), Quaternion.identity);
        floor.transform.localScale = new Vector3(16f, 0.2f, 20f);
        floor.GetComponent<Renderer>().sharedMaterial = GetOrCreateMaterial("Pitch", new Color(0.08f, 0.26f, 0.12f));

        GameObject label = new($"TEST: {title}");
        label.transform.position = new Vector3(0f, GoalBottomY + GoalHeight + 0.8f, 0.3f);
        return scene;
    }

    private static Transform CreateGoalFrame()
    {
        GameObject goal = new("SoccerGoalFrame");
        CreateFrameBar(goal.transform, "LeftPost", new Vector3(-GoalWidth * 0.5f, GoalBottomY + GoalHeight * 0.5f, 0f), new Vector3(0.12f, GoalHeight, 0.12f));
        CreateFrameBar(goal.transform, "RightPost", new Vector3(GoalWidth * 0.5f, GoalBottomY + GoalHeight * 0.5f, 0f), new Vector3(0.12f, GoalHeight, 0.12f));
        CreateFrameBar(goal.transform, "Crossbar", new Vector3(0f, GoalBottomY + GoalHeight, 0f), new Vector3(GoalWidth, 0.12f, 0.12f));
        return goal.transform;
    }

    private static void CreateFrameBar(Transform parent, string name, Vector3 position, Vector3 scale)
    {
        GameObject bar = GameObject.CreatePrimitive(PrimitiveType.Cube);
        bar.name = name;
        bar.transform.SetParent(parent, false);
        bar.transform.position = position;
        bar.transform.localScale = scale;
        bar.GetComponent<Renderer>().sharedMaterial = GetOrCreateMaterial("GoalFrame", Color.white);
    }

    private static GameObject CreateBall(string name)
    {
        GameObject ball = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        ball.name = name;
        ball.transform.localScale = Vector3.one * 0.38f;
        ball.GetComponent<Renderer>().sharedMaterial = GetOrCreateMaterial("Ball", Color.white);
        Rigidbody body = ball.AddComponent<Rigidbody>();
        body.mass = 0.43f;
        body.useGravity = false;
        body.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
        body.interpolation = RigidbodyInterpolation.Interpolate;
        ball.AddComponent<GoalBallLauncher>();
        return ball;
    }

    private static void PinClothBoundary(JointCloth cloth)
    {
        for (int row = 0; row < cloth.rows; row++)
        {
            for (int col = 0; col < cloth.cols; col++)
            {
                if (row != 0 && row != cloth.rows - 1 && col != 0 && col != cloth.cols - 1)
                    continue;
                Rigidbody body = cloth.points[row * cloth.cols + col].GetComponent<Rigidbody>();
                body.isKinematic = true;
            }
        }
    }

    private static List<GameObject> CreateCornerFixers(Transform parent)
    {
        List<GameObject> fixers = new();
        foreach (Vector3 position in new[]
        {
            new Vector3(-GoalWidth * 0.5f, GoalBottomY + GoalHeight, 0f),
            new Vector3(GoalWidth * 0.5f, GoalBottomY + GoalHeight, 0f),
            new Vector3(-GoalWidth * 0.5f, GoalBottomY, 0f),
            new Vector3(GoalWidth * 0.5f, GoalBottomY, 0f),
        })
        {
            GameObject fixer = new("NetCornerFixer");
            fixer.transform.SetParent(parent, false);
            fixer.transform.position = position;
            BoxCollider collider = fixer.AddComponent<BoxCollider>();
            collider.size = Vector3.one * 0.1f;
            fixers.Add(fixer);
        }
        return fixers;
    }

    private static Mesh CreateGoalMesh(int columns, int rows)
    {
        Mesh mesh = new() { name = "ClothBehaviourGoalMesh" };
        Vector3[] vertices = new Vector3[columns * rows];
        Vector2[] uv = new Vector2[vertices.Length];
        for (int row = 0; row < rows; row++)
        {
            float v = row / (float)(rows - 1);
            for (int col = 0; col < columns; col++)
            {
                float u = col / (float)(columns - 1);
                int i = row * columns + col;
                vertices[i] = new Vector3(Mathf.Lerp(-GoalWidth * 0.5f, GoalWidth * 0.5f, u), Mathf.Lerp(GoalBottomY + GoalHeight, GoalBottomY, v), 0f);
                uv[i] = new Vector2(u, v);
            }
        }

        int[] triangles = new int[(columns - 1) * (rows - 1) * 6];
        int t = 0;
        for (int row = 0; row < rows - 1; row++)
        {
            for (int col = 0; col < columns - 1; col++)
            {
                int a = row * columns + col;
                int b = a + 1;
                int c = a + columns;
                int d = c + 1;
                triangles[t++] = a;
                triangles[t++] = b;
                triangles[t++] = c;
                triangles[t++] = b;
                triangles[t++] = d;
                triangles[t++] = c;
            }
        }

        mesh.vertices = vertices;
        mesh.uv = uv;
        mesh.triangles = triangles;
        mesh.RecalculateNormals();
        mesh.RecalculateBounds();
        return mesh;
    }

    private static GameObject GetOrCreateNetPointPrefab()
    {
        const string path = GeneratedFolder + "/UnitySimplePhysicsGoalPoint.prefab";
        GameObject existing = AssetDatabase.LoadAssetAtPath<GameObject>(path);
        if (existing != null)
            return existing;

        GameObject point = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        point.name = "UnitySimplePhysicsGoalPoint";
        point.transform.localScale = Vector3.one * 0.08f;
        point.GetComponent<Renderer>().enabled = false;
        Rigidbody body = point.AddComponent<Rigidbody>();
        body.mass = 0.015f;
        body.useGravity = false;
        body.linearDamping = 1.5f;
        body.angularDamping = 2f;
        body.constraints = RigidbodyConstraints.FreezeRotation;
        GameObject prefab = PrefabUtility.SaveAsPrefabAsset(point, path);
        UnityEngine.Object.DestroyImmediate(point);
        return prefab;
    }

    private static Material GetOrCreateMaterial(string name, Color color)
    {
        string path = $"{GeneratedFolder}/{name}.mat";
        Material material = AssetDatabase.LoadAssetAtPath<Material>(path);
        if (material == null)
        {
            Shader shader = Shader.Find("HDRP/Lit") ?? Shader.Find("Standard");
            material = new Material(shader);
            AssetDatabase.CreateAsset(material, path);
        }

        if (material.HasProperty("_BaseColor")) material.SetColor("_BaseColor", color);
        if (material.HasProperty("_Color")) material.color = color;
        if (material.HasProperty("_SurfaceType")) material.SetFloat("_SurfaceType", color.a < 0.99f ? 1f : 0f);
        return material;
    }

    private static void SaveScene(Scene scene, string fileName)
    {
        string path = $"{SceneFolder}/{fileName}";
        if (!EditorSceneManager.SaveScene(scene, path, false))
            throw new InvalidOperationException($"Failed to save {path}.");
    }

    private static void EnsureFolder(string parent, string child)
    {
        if (!AssetDatabase.IsValidFolder($"{parent}/{child}"))
            AssetDatabase.CreateFolder(parent, child);
    }
}
