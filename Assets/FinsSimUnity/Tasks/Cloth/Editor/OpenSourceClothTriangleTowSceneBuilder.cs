using System.Collections.Generic;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEditor.Rendering.HighDefinition;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnitySimpleCloth = OpenSourceCloth.UnitySimplePhysics.UnitySimplePhysicsCloth;

/// <summary>Creates like-for-like triangular-net towing tests for the three imported cloth implementations.</summary>
public static class OpenSourceClothTriangleTowSceneBuilder
{
    private const string SceneFolder = "Assets/FinsSimUnity/Tasks/Cloth/Scenes/OpenSourceCloth";
    private const string MaterialFolder = "Assets/FinsSimUnity/Generated/OpenSourceCloth";
    private const float StartX = -1.5f;

    [MenuItem("FinsSim/Open Source Cloth/Create All Triangle Tow Tests")]
    public static void CreateAllTriangleTowTests()
    {
        EnsureFolder("Assets/Scenes", "OpenSourceCloth");
        EnsureFolder("Assets/Generated", "OpenSourceCloth");
        CreateUnitySimplePhysicsScene();
        CreateHabradorScene();
        CreateClothBehaviourScene();
        AssetDatabase.SaveAssets();
        AssetDatabase.Refresh();
        Debug.Log("[OpenSourceCloth] Created the three triangular-net tow tests.");
    }

    private static void CreateUnitySimplePhysicsScene()
    {
        Scene scene = CreateBaseScene("UnitySimplePhysics triangular joint net");
        Transform anchors = CreateAnchors();
        GameObject target = CreateTargetBall();

        GameObject net = new("UnitySimplePhysicsTriangleCloth");
        MeshRenderer renderer = net.AddComponent<MeshRenderer>();
        renderer.sharedMaterial = GetMaterial("UnitySimpleTriangleTowNet", new Color(0.03f, 0.73f, 0.96f, 1f));
        net.AddComponent<MeshFilter>();
        net.AddComponent<UnitySimpleCloth>();
        UnitySimplePhysicsTriangleClothGenerator component = net.AddComponent<UnitySimplePhysicsTriangleClothGenerator>();
        component.clothMaterial = renderer.sharedMaterial;
        component.lineMaterial = renderer.sharedMaterial;
        ConfigureNet(component, anchors);
        CreateUnitySimplePhysicsStaticTopology(component);

        Save(scene, "UnitySimplePhysicsTriangleTow.unity");
    }

    private static void CreateHabradorScene()
    {
        Scene scene = CreateBaseScene("TenMinutePhysics XPBD triangular net");
        Transform anchors = CreateAnchors();
        GameObject target = CreateTargetBall();

        GameObject net = new("HabradorXPBDTriangleNet");
        MeshRenderer renderer = net.AddComponent<MeshRenderer>();
        renderer.sharedMaterial = GetMaterial("HabradorTriangleTowNet", new Color(0.08f, 0.9f, 0.34f, 0.62f));
        MeshFilter filter = net.AddComponent<MeshFilter>();
        TriangleTowLattice lattice = TriangleTowLattice.Create(
            anchors.GetChild(0).position, anchors.GetChild(1).position, anchors.GetChild(2).position, 8);
        // The initial triangle mesh is serialized in this scene. XPBD only
        // deforms its runtime copy while playing; it does not invent topology.
        filter.sharedMesh = TriangleTowLattice.CreateMesh("HabradorTriangleTowMesh", lattice);
        HabradorTriangleTowDriver component = net.AddComponent<HabradorTriangleTowDriver>();
        ConfigureNet(component, anchors);
        component.topParticleIndex = lattice.TopIndex;
        component.leftParticleIndex = lattice.LeftIndex;
        component.rightParticleIndex = lattice.RightIndex;
        component.targetBody = target.GetComponent<Rigidbody>();
        component.targetRadiusM = 0.3f;
        component.targetFriction = 0.8f;

        Save(scene, "HabradorXPBDTriangleTow.unity");
    }

    private static void CreateClothBehaviourScene()
    {
        Scene scene = CreateBaseScene("ClothBehaviour triangular spring net");
        Transform anchors = CreateAnchors();
        GameObject target = CreateTargetBall();

        GameObject net = new("ClothBehaviourTriangleNet");
        MeshFilter filter = net.AddComponent<MeshFilter>();
        filter.sharedMesh = TriangleTowLattice.CreateMesh(
            "ClothBehaviourTriangleTowMesh",
            TriangleTowLattice.Create(anchors.GetChild(0).position, anchors.GetChild(1).position, anchors.GetChild(2).position, 8));
        MeshRenderer renderer = net.AddComponent<MeshRenderer>();
        renderer.sharedMaterial = GetMaterial("ClothBehaviourTriangleTowNet", new Color(1f, 0.46f, 0.05f, 0.62f));
        net.AddComponent<MeshCollider>();
        net.AddComponent<DynamicMeshColliderSynchronizer>();

        ClothBehaviour cloth = net.AddComponent<ClothBehaviour>();
        cloth.m_Paused = false;
        cloth.m_Gravity = Vector3.zero;
        cloth.m_MeshMass = 0.8f;
        cloth.m_NodeDamping = 0.5f;
        cloth.m_SpringDamping = 0.55f;
        cloth.m_TractionStiffness = 280f;
        cloth.m_FlexionStiffness = 180f;
        cloth.m_SolvingMethod = ClothBehaviour.Solver.Simplectic;
        cloth.m_FixingByTexture = false;
        cloth.m_Fixers = new List<GameObject>
        {
            anchors.GetChild(0).gameObject,
            anchors.GetChild(1).gameObject,
            anchors.GetChild(2).gameObject,
        };
        cloth.m_CanCollide = true;
        cloth.m_CollidingMeshes = new List<GameObject> { target };
        cloth.m_PenaltyStiffness = 1200f;
        cloth.m_CollisionOffsetDistance = 0.05f;

        Save(scene, "ClothBehaviourTriangleTow.unity");
    }

    private static Scene CreateBaseScene(string title)
    {
        Scene scene = EditorSceneManager.NewScene(NewSceneSetup.EmptyScene, NewSceneMode.Single);
        RenderSettings.ambientMode = UnityEngine.Rendering.AmbientMode.Flat;
        RenderSettings.ambientLight = new Color(0.45f, 0.45f, 0.48f);

        GameObject lightObject = new("Directional Light");
        Light light = lightObject.AddComponent<Light>();
        light.type = LightType.Directional;
        light.intensity = 2.4f;
        lightObject.transform.rotation = Quaternion.Euler(45f, -35f, 0f);

        GameObject cameraObject = new("Main Camera");
        Camera camera = cameraObject.AddComponent<Camera>();
        camera.tag = "MainCamera";
        camera.fieldOfView = 54f;
        camera.transform.position = new Vector3(-6.8f, 3.4f, -6.4f);
        camera.transform.rotation = Quaternion.LookRotation(new Vector3(0.5f, 0f, 0f) - camera.transform.position);

        GameObject floor = GameObject.CreatePrimitive(PrimitiveType.Cube);
        floor.name = $"ReferenceFloor_{title}";
        floor.transform.position = new Vector3(0.5f, -1.25f, 0f);
        floor.transform.localScale = new Vector3(9f, 0.1f, 6f);
        floor.GetComponent<Renderer>().sharedMaterial = GetMaterial("TriangleTowFloor", new Color(0.06f, 0.18f, 0.12f));
        return scene;
    }

    private static Transform CreateAnchors()
    {
        GameObject group = new("ThreeTowParticles");
        Transform[] points =
        {
            CreateAnchor(group.transform, "TowParticle_Top", new Vector3(StartX, 1.0f, 0f)),
            CreateAnchor(group.transform, "TowParticle_Left", new Vector3(StartX, -0.75f, -1.15f)),
            CreateAnchor(group.transform, "TowParticle_Right", new Vector3(StartX, -0.75f, 1.15f)),
        };

        TriangleTowAnchorDriver driver = group.AddComponent<TriangleTowAnchorDriver>();
        driver.top = points[0];
        driver.left = points[1];
        driver.right = points[2];
        driver.speedMps = 0.55f;
        driver.travelDistanceM = 4f;
        return group.transform;
    }

    private static Transform CreateAnchor(Transform parent, string name, Vector3 position)
    {
        GameObject anchor = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        anchor.name = name;
        anchor.transform.SetParent(parent, false);
        anchor.transform.position = position;
        anchor.transform.localScale = Vector3.one * 0.18f;
        anchor.GetComponent<Renderer>().sharedMaterial = GetMaterial("TowParticle", new Color(1f, 0.18f, 0.1f));
        Rigidbody body = anchor.AddComponent<Rigidbody>();
        body.isKinematic = true;
        body.useGravity = false;
        return anchor.transform;
    }

    private static GameObject CreateTargetBall()
    {
        GameObject target = GameObject.CreatePrimitive(PrimitiveType.Sphere);
        target.name = "TargetBall_NoGravity";
        target.transform.position = new Vector3(-0.25f, 0f, 0f);
        target.transform.localScale = Vector3.one * 0.6f;
        target.GetComponent<Renderer>().sharedMaterial = GetMaterial("TowTargetBall", new Color(0.98f, 0.88f, 0.16f));
        Rigidbody body = target.AddComponent<Rigidbody>();
        body.mass = 0.43f;
        body.useGravity = false;
        body.linearDamping = 0.1f;
        body.angularDamping = 0.1f;
        body.interpolation = RigidbodyInterpolation.Interpolate;
        body.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
        return target;
    }

    private static void ConfigureNet(UnitySimplePhysicsTriangleClothGenerator net, Transform anchors)
    {
        net.topAnchor = anchors.GetChild(0);
        net.leftAnchor = anchors.GetChild(1);
        net.rightAnchor = anchors.GetChild(2);
        net.subdivisions = 8;
        net.nodeMassKg = 0.025f;
        net.nodeLinearDamping = 1.8f;
        net.nodeColliderRadiusM = 0.035f;
        net.jointProjectionDistanceM = 0.003f;
    }

    private static void CreateUnitySimplePhysicsStaticTopology(UnitySimplePhysicsTriangleClothGenerator net)
    {
        TriangleTowLattice lattice = TriangleTowLattice.Create(
            net.topAnchor.position, net.leftAnchor.position, net.rightAnchor.position, net.subdivisions);
        List<Transform> points = new(lattice.Vertices.Length);
        for (int i = 0; i < lattice.Vertices.Length; i++)
        {
            Transform anchor = i == lattice.TopIndex ? net.topAnchor
                : i == lattice.LeftIndex ? net.leftAnchor
                : i == lattice.RightIndex ? net.rightAnchor
                : null;
            if (anchor != null)
            {
                points.Add(anchor);
                continue;
            }

            GameObject node = new($"ClothPoint_{i:00}");
            node.transform.SetParent(net.transform, true);
            node.transform.position = lattice.Vertices[i];
            Rigidbody body = node.AddComponent<Rigidbody>();
            body.mass = net.nodeMassKg;
            body.useGravity = false;
            body.linearDamping = net.nodeLinearDamping;
            body.angularDamping = 2f;
            body.constraints = RigidbodyConstraints.FreezeRotation;
            body.interpolation = RigidbodyInterpolation.Interpolate;
            body.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
            node.AddComponent<SphereCollider>().radius = net.nodeColliderRadiusM;
            points.Add(node.transform);
        }

        List<Vector2Int> edges = UnitySimplePhysicsTriangleClothGenerator.BuildEdges(lattice.Triangles);
        foreach (Vector2Int edge in edges)
            UnitySimplePhysicsTriangleClothGenerator.CreateDistanceJoint(
                points[edge.x].GetComponent<Rigidbody>(), points[edge.y].GetComponent<Rigidbody>(), net.jointProjectionDistanceM);
        net.SetStaticTopology(points, lattice.Triangles, edges);
    }

    private static void ConfigureNet(HabradorTriangleTowDriver net, Transform anchors)
    {
        net.topAnchor = anchors.GetChild(0);
        net.leftAnchor = anchors.GetChild(1);
        net.rightAnchor = anchors.GetChild(2);
        net.subdivisions = 8;
        net.solverSubsteps = 8;
        net.stretchingCompliance = 0.00002f;
        net.bendingCompliance = 0.001f;
        net.velocityDamping = 2f;
        net.enableTutorialFloorCollision = false;
    }

    private static Material GetMaterial(string name, Color color)
    {
        string path = $"{MaterialFolder}/{name}.mat";
        Material material = AssetDatabase.LoadAssetAtPath<Material>(path);
        if (material == null)
        {
            Shader shader = Shader.Find("HDRP/Lit");
            if (shader == null)
                throw new System.InvalidOperationException("HDRP/Lit is required for the triangle tow scenes.");
            material = new Material(shader);
            AssetDatabase.CreateAsset(material, path);
        }

        material.SetColor("_BaseColor", color);
        material.SetFloat("_SurfaceType", color.a < 0.999f ? 1f : 0f);
        // In HDRP, _CullMode is derived from _DoubleSidedEnable when material
        // keywords are reset.  Setting cull mode alone gets overwritten back
        // to Back culling, which made the cloth disappear from its reverse.
        material.SetFloat("_DoubleSidedEnable", 1f);
        material.SetFloat("_DoubleSidedNormalMode", 1f);
        material.SetFloat("_DoubleSidedGIMode", 0f);
        material.doubleSidedGI = true;
        material.enableInstancing = true;
        HDShaderUtils.ResetMaterialKeywords(material);
        EditorUtility.SetDirty(material);
        return material;
    }

    private static void Save(Scene scene, string name)
    {
        if (!EditorSceneManager.SaveScene(scene, $"{SceneFolder}/{name}", false))
            throw new System.InvalidOperationException($"Failed to save {name}.");
    }

    private static void EnsureFolder(string parent, string child)
    {
        if (!AssetDatabase.IsValidFolder($"{parent}/{child}"))
            AssetDatabase.CreateFolder(parent, child);
    }
}
