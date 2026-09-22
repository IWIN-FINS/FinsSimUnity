using System.Collections.Generic;
using UnityEngine;
using Obi;
#if UNITY_EDITOR
using UnityEditor;
#endif

/**
 * Procedurally generates an Obi rope net between the two netter UUVs.
 *
 * The net keeps the original sample's "rigidbody nodes + independent rope
 * segments" structure, but adds two short lead ropes from the UUVs to the
 * upper corners of the net.
 */
public class FishNetGenerator : MonoBehaviour
{
    private const string GeneratedRootName = "GeneratedFishNet";

    public enum NetTopology
    {
        RectangularTwoAnchor = 0,
        TriangularThreeAnchor = 1,
        QuadrilateralFourAnchor = 2,
    }

    [Header("References")]
    public CatchAreaManager areaManager;
    public Transform netter1;
    public Transform netter2;
    [Tooltip("Third anchor used only by TriangularThreeAnchor. Assign the Herder transform.")]
    public Transform herder;
    [Tooltip("Drag UUV7/MarkPosition here. If left empty, the generator looks for a child named MarkPosition under netter1.")]
    public Transform netter1MarkPosition;
    [Tooltip("Drag UUV8/MarkPosition here. If left empty, the generator looks for a child named MarkPosition under netter2.")]
    public Transform netter2MarkPosition;
    [Tooltip("Optional third-anchor marker. If empty, MarkPosition under herder is used.")]
    public Transform herderMarkPosition;
    [Tooltip("Third corner used only by QuadrilateralFourAnchor.")]
    public Transform netter3;
    [Tooltip("Fourth corner used only by QuadrilateralFourAnchor.")]
    public Transform netter4;
    public Transform netter3MarkPosition;
    public Transform netter4MarkPosition;
    public Material material;

    [Header("Net Geometry")]
    public NetTopology netTopology = NetTopology.RectangularTwoAnchor;
    public Vector2Int resolution = new Vector2Int(5, 5);
    public Vector2 size = new Vector2(0.5f, 0.5f);
    public float nodeSize = 0.2f;
    [Tooltip("If enabled, the generated net width is the netter distance minus the two lead ropes at build/reset time.")]
    public bool fitWidthBetweenNettersOnBuild = true;
    [Tooltip("Used when fitWidthBetweenNettersOnBuild is disabled. <= 0 falls back to resolution.x * size.x.")]
    public float fixedNetWidth = 0f;
    public float leadRopeLength = 0.5f;
    [Tooltip("Extra perpendicular arc at the midpoint of each UUV lead rope. This gives the lead rope enough particles and slack to bend when the three UUVs change formation.")]
    [Min(0f)] public float leadRopeSlackM = 0.10f;
    public Vector3 netter1LocalAttachmentOffset = Vector3.zero;
    public Vector3 netter2LocalAttachmentOffset = Vector3.zero;
    public Vector3 herderLocalAttachmentOffset = Vector3.zero;
    public Vector3 netter3LocalAttachmentOffset = Vector3.zero;
    public Vector3 netter4LocalAttachmentOffset = Vector3.zero;
    public Vector3 netWorldOffset = Vector3.zero;
    [Tooltip("Rows along each triangular edge. Four creates 30 internal rope segments.")]
    [Min(1)] public int triangularResolution = 4;

    [Header("Obi Solver")]
    [Tooltip("Compute/GPU can fail on OpenGL/OpenGLES due to storage-buffer limits. Burst is safer for this rope net.")]
    public ObiSolver.BackendType solverBackend = ObiSolver.BackendType.Burst;
    [Tooltip("Automatically use Burst when Compute is selected but Unity is running on OpenGL/OpenGLES.")]
    public bool fallbackToBurstOnOpenGL = false;
    [Tooltip("Disable constraint groups this rope net does not use. Turn this off to keep ObiSolver's default constraint groups.")]
    public bool optimizeSolverConstraintGroups = false;

    [Header("Obi Rope")]
    [Tooltip("Lower values create fewer rope particles. Surface collisions usually allow 0.05-0.08 for this net.")]
    public float ropeBlueprintResolution = 0.06f;
    public float ropeThickness = 0.02f;
    [Tooltip("Mass used by each generated rope control point/particle. 0.0001 is the supported near-massless minimum, so the net retains Obi constraints and collisions without appreciable inertia.")]
    [Min(0.0001f)] public float controlPointMass = 0.0001f;
    [Tooltip("Rest-length multiplier applied only to internal grid ropes. Values below 1 pre-tension the net; keep lead ropes at their true length.")]
    [Range(0.8f, 1f)] public float gridRopeRestLengthScale = 1f;
    [Tooltip("Use separate Obi filter categories for rope, prey, UUVs, and net pin nodes.")]
    public bool useStructuredObiCollisionFilters = true;
    [Range(0, 15)]
    public int ropeCollisionCategory = 0;
    [Range(0, 15)]
    public int preyCollisionCategory = 1;
    [Range(0, 15)]
    public int uuvCollisionCategory = 2;
    [Range(0, 15)]
    public int netPinCollisionCategory = 3;
    [Tooltip("Allow generated ropes to collide with UUV ObiColliders in addition to being pinned to them.")]
    public bool ropesCollideWithUuvs = true;
    [Tooltip("Allow generated node helper colliders to collide with ropes. Usually keep disabled; they are primarily pin targets.")]
    public bool netPinCollidersCollideWithRopes = false;
    [Tooltip("Obi collision filter for generated rope particles.")]
    public int collisionFilter = ObiUtils.MakeFilter(ObiUtils.CollideWithEverything, 0);
    [Tooltip("Obi collision filter for generated node pin colliders.")]
    public int pinColliderFilter = ObiUtils.MakeFilter(ObiUtils.CollideWithNothing, 0);
    [Tooltip("Obi collision filter for UUV ObiColliders used by dynamic lead-rope attachments.")]
    public int uuvColliderFilter = ObiUtils.MakeFilter(1 << 0, 2);
    [Tooltip("Collision material applied to every generated rope. Assign the same material to the Target so rope-vs-Target contacts use its high-friction values.")]
    public ObiCollisionMaterial ropeCollisionMaterial;

    [Header("Net Weight / Buoyancy")]
    [Tooltip("Gravity used by the generated ObiSolver. Use Vector3.zero for a neutrally buoyant underwater net.")]
    public Vector3 solverGravity = Vector3.zero;
    [Tooltip("Mass of each rigidbody node at rope intersections. 0.0001 is the supported near-massless minimum, so the nodes retain collision and attachment constraints without appreciable inertia.")]
    [Min(0.0001f)] public float nodeMass = 0.0001f;
    [Tooltip("Whether the rigidbody nodes should use Unity gravity. Usually keep this disabled underwater.")]
    public bool nodeUseGravity = false;
    [Tooltip("Linear damping applied to rigidbody nodes.")]
    public float nodeLinearDamping = 0.25f;
    [Tooltip("Angular damping applied to rigidbody nodes.")]
    public float nodeAngularDamping = 0.25f;

    [Header("Collision")]
    [Tooltip("Use rope surface/edge collisions instead of sparse particle-only contacts. Recommended for this net.")]
    public bool useSurfaceCollisions = true;
    [Tooltip("Allow rope particles/simplices to collide with each other. Keep enabled for net self-intersection physics and future collision rewards.")]
    public bool useSelfCollisions = true;
    [Tooltip("Enable Obi particle-particle collision constraints. Required for rope self collision contacts to be solved/reported.")]
    public bool useParticleCollisions = true;
    [Tooltip("Enable rope-vs-ObiCollider contact solving. Leave disabled for distance-constraint tests and GPU/OpenGL compatibility.")]
    public bool useColliderCollisions = false;
    [Tooltip("Iterations used by Obi's surface collision closest-point search.")]
    [Range(1, 32)]
    public int surfaceCollisionIterations = 8;
    [Tooltip("Tolerance used by Obi's surface collision closest-point search.")]
    public float surfaceCollisionTolerance = 0.005f;

    [Header("UUV Attachment")]
    [Tooltip("Use two-way Obi pin constraints at the UUV ends. MarkPosition is still used as the rope start position, but the pin target is the UUV Rigidbody/ObiCollider.")]
    public bool useDynamicUuvAttachments = true;
    [Tooltip("Automatically add ObiCollider to UUV Rigidbody objects when using dynamic UUV attachments.")]
    public bool addMissingObiCollidersToNetters = true;
    [Tooltip("Dynamic attachment compliance. 0 is stiff; larger values make the UUV-to-rope attachment softer.")]
    public float uuvAttachmentCompliance = 0f;

    [Header("Runtime")]
    [Tooltip("If enabled, play mode and CatchAreaManager reset rebuild the net from the current MarkPosition anchors. Disable this to use an editor-generated static net.")]
    public bool generateOnStart = false;
    public bool hideNodeRenderers = false;

    [SerializeField, HideInInspector] private Transform generatedRoot;
    private ObiSolver solver;
    private int lastRegenerateFrame = -1;
    private Material fallbackMaterial;
    private bool hasSavedNetState;
    private Vector3 savedRootLocalPosition;
    private Quaternion savedRootLocalRotation;
    private RigidbodySnapshot[] savedRigidbodies = new RigidbodySnapshot[0];

    private struct RigidbodySnapshot
    {
        public Rigidbody body;
        public Vector3 localPosition;
        public Quaternion localRotation;
    }

#if UNITY_EDITOR
    [SerializeField, HideInInspector] private string generatedBlueprintAssetPath;
    private string activeEditorBlueprintAssetPath;
    private bool editorBlueprintMainAssetCreated;
    private Object editorBlueprintMainAsset;
#endif

    public Transform NetRoot
    {
        get
        {
            ResolveGeneratedRoot();
            return generatedRoot;
        }
    }

    private void Awake()
    {
        if (!generateOnStart)
        {
            ResolveGeneratedRoot();
            EnsureStaticGeneratedNetReadyForRuntime();
            ApplyRuntimeCollisionSettingsToCurrentNet();
            CaptureCurrentNetState();
        }
    }

    private void Start()
    {
        if (generateOnStart && generatedRoot == null)
        {
            ResolveGeneratedRoot();
            GenerateNet();
        }
        else if (!generateOnStart)
        {
            ResolveGeneratedRoot();
            EnsureStaticGeneratedNetReadyForRuntime();
            ApplyRuntimeCollisionSettingsToCurrentNet();
            CaptureCurrentNetState();
        }
    }

    private void OnValidate()
    {
        RefreshStructuredCollisionFilters();
    }

    [ContextMenu("Generate Net")]
    public void GenerateNet()
    {
        ResolveReferences();
        ClearGeneratedNet();

        Transform anchor1 = GetNetterAnchor(netter1, netter1MarkPosition);
        Transform anchor2 = GetNetterAnchor(netter2, netter2MarkPosition);
        Transform anchor3 = GetNetterAnchor(herder, herderMarkPosition);
        bool missingRequiredAnchor = anchor1 == null || anchor2 == null ||
            (netTopology == NetTopology.TriangularThreeAnchor && anchor3 == null);
        if (missingRequiredAnchor)
        {
            Debug.LogWarning(
                netTopology == NetTopology.TriangularThreeAnchor
                    ? "FishNetGenerator could not find all three triangular-net anchors."
                    : "FishNetGenerator could not find both netter MarkPosition anchors.",
                this);
            return;
        }

        resolution.x = Mathf.Max(1, resolution.x);
        resolution.y = Mathf.Max(1, resolution.y);
        size.x = Mathf.Max(0.01f, size.x);
        size.y = Mathf.Max(0.01f, size.y);
        nodeSize = Mathf.Max(0.01f, nodeSize);
        leadRopeLength = Mathf.Max(0f, leadRopeLength);
        leadRopeSlackM = Mathf.Max(0f, leadRopeSlackM);
        ropeThickness = Mathf.Max(0.001f, ropeThickness);
        ropeBlueprintResolution = Mathf.Clamp(ropeBlueprintResolution, 0.001f, 1f);
        triangularResolution = Mathf.Max(1, triangularResolution);
        controlPointMass = Mathf.Max(0.0001f, controlPointMass);
        RefreshStructuredCollisionFilters();
        if (collisionFilter == 1)
        {
            collisionFilter = ObiUtils.MakeFilter(ObiUtils.CollideWithEverything, 0);
        }

        nodeMass = Mathf.Max(0.0001f, nodeMass);
        nodeLinearDamping = Mathf.Max(0f, nodeLinearDamping);
        nodeAngularDamping = Mathf.Max(0f, nodeAngularDamping);
        surfaceCollisionIterations = Mathf.Clamp(surfaceCollisionIterations, 1, 32);
        surfaceCollisionTolerance = Mathf.Max(0.000001f, surfaceCollisionTolerance);

        CreateRootAndSolver();
        CreateNet();
        ApplyRuntimeCollisionSettingsToCurrentNet();
        CaptureCurrentNetState();

        if (areaManager != null)
        {
            areaManager.net = generatedRoot;
        }
    }

#if UNITY_EDITOR
    public void GenerateStaticNetForEditor(string blueprintAssetPath)
    {
        if (Application.isPlaying)
        {
            GenerateNet();
            return;
        }

        if (string.IsNullOrEmpty(blueprintAssetPath))
        {
            Debug.LogWarning("FishNetGenerator needs a blueprint asset path for static editor generation.", this);
            return;
        }

        // A grid creates one Obi blueprint per rope. Defer imports until all
        // blueprints have been attached to their single asset, otherwise the
        // editor repeatedly refreshes this growing asset during generation.
        AssetDatabase.StartAssetEditing();
        try
        {
            // A copied scene may still point at another scene's Blueprint asset.
            // Never delete that source asset while creating this scene's static net.
            if (generatedBlueprintAssetPath == blueprintAssetPath)
            {
                DeleteGeneratedBlueprintAsset();
            }
            DeleteAssetIfExists(blueprintAssetPath);
            DeleteAssetIfExists(GetSiblingAssetPath(blueprintAssetPath, "GeneratedFishNet_FallbackRopeMaterial.mat"));
            EnsureAssetFolder(blueprintAssetPath);

            activeEditorBlueprintAssetPath = blueprintAssetPath;
            generatedBlueprintAssetPath = blueprintAssetPath;
            editorBlueprintMainAssetCreated = false;
            editorBlueprintMainAsset = null;

            GenerateNet();
            AssetDatabase.SaveAssets();
        }
        finally
        {
            activeEditorBlueprintAssetPath = null;
            editorBlueprintMainAssetCreated = false;
            editorBlueprintMainAsset = null;
            AssetDatabase.StopAssetEditing();
        }
    }
#endif

    public void RegenerateNet()
    {
        if (Application.isPlaying && lastRegenerateFrame == Time.frameCount)
        {
            return;
        }

        lastRegenerateFrame = Time.frameCount;
        GenerateNet();
    }

    public bool ResetNetForEpisode()
    {
        ResolveGeneratedRoot();
        EnsureStaticGeneratedNetReadyForRuntime();

        if (generateOnStart)
        {
            RegenerateNet();
            ResolveGeneratedRoot();
            return generatedRoot != null;
        }

        ApplyUuvColliderFilters();
        if (generatedRoot == null)
        {
            return false;
        }

        if (!hasSavedNetState)
        {
            CaptureCurrentNetState();
        }

        RestoreSavedNetState();
        ApplyRuntimeCollisionSettingsToCurrentNet();
        return true;
    }

    public bool ApplyRuntimeCollisionSettingsToCurrentNet()
    {
        RefreshStructuredCollisionFilters();
        ResolveGeneratedRoot();
        EnsureStaticGeneratedNetReadyForRuntime();
        if (generatedRoot == null)
        {
            return false;
        }

        EnsureNetSurfaceModel();

        bool allReady = true;
        ObiSolver currentSolver = generatedRoot.GetComponentInChildren<ObiSolver>(true);
        if (currentSolver != null)
        {
            currentSolver.particleCollisionConstraintParameters.enabled = true;
            currentSolver.particleFrictionConstraintParameters.enabled = true;
            currentSolver.PushSolverParameters();
        }
        else
        {
            allReady = false;
        }

        ObiRope[] ropes = generatedRoot.GetComponentsInChildren<ObiRope>(true);
        int ropeMask = ObiUtils.GetMaskFromFilter(GetRopeParticleFilter());
        int ropeCategory = ObiUtils.GetCategoryFromFilter(GetRopeParticleFilter());
        for (int i = 0; i < ropes.Length; ++i)
        {
            ObiRope rope = ropes[i];
            if (rope == null)
            {
                continue;
            }

            rope.surfaceCollisions = useSurfaceCollisions;
            rope.selfCollisions = true;
            if (rope.isLoaded)
            {
                rope.SetFilterCategory(ropeCategory);
                rope.SetFilterMask(ropeMask);
            }

            allReady &= rope.isLoaded;
        }

        ApplyUuvColliderFilters();

        return allReady && ropes.Length > 0;
    }

    public bool EnsureStaticGeneratedNetReadyForRuntime()
    {
        if (generateOnStart)
        {
            return true;
        }

        ResolveGeneratedRoot();
        if (generatedRoot == null)
        {
            return false;
        }

        if (!Application.isPlaying)
        {
            return true;
        }

        if (!generatedRoot.gameObject.activeSelf)
        {
            generatedRoot.gameObject.SetActive(true);
        }

        return generatedRoot.gameObject.activeInHierarchy;
    }

    public int GetRopeParticleFilter()
    {
        if (!useStructuredObiCollisionFilters)
        {
            return collisionFilter;
        }

        int mask = CategoryMask(ropeCollisionCategory) |
                   CategoryMask(preyCollisionCategory);
        if (ropesCollideWithUuvs)
        {
            mask |= CategoryMask(uuvCollisionCategory);
        }

        if (netPinCollidersCollideWithRopes)
        {
            mask |= CategoryMask(netPinCollisionCategory);
        }

        return ObiUtils.MakeFilter(mask, ClampCategory(ropeCollisionCategory));
    }

    public int GetPreyColliderFilter()
    {
        return ObiUtils.MakeFilter(CategoryMask(ropeCollisionCategory), ClampCategory(preyCollisionCategory));
    }

    public int GetUuvColliderFilter()
    {
        if (!useStructuredObiCollisionFilters)
        {
            return uuvColliderFilter;
        }

        int mask = ropesCollideWithUuvs ? CategoryMask(ropeCollisionCategory) : ObiUtils.CollideWithNothing;
        return ObiUtils.MakeFilter(mask, ClampCategory(uuvCollisionCategory));
    }

    public int GetNetPinColliderFilter()
    {
        if (!useStructuredObiCollisionFilters)
        {
            return pinColliderFilter;
        }

        int mask = netPinCollidersCollideWithRopes ? CategoryMask(ropeCollisionCategory) : ObiUtils.CollideWithNothing;
        return ObiUtils.MakeFilter(mask, ClampCategory(netPinCollisionCategory));
    }

    private void RefreshStructuredCollisionFilters()
    {
        if (!useStructuredObiCollisionFilters)
        {
            return;
        }

        collisionFilter = GetRopeParticleFilter();
        pinColliderFilter = GetNetPinColliderFilter();
        uuvColliderFilter = GetUuvColliderFilter();
    }

    private void ApplyUuvColliderFilters()
    {
        ApplyUuvColliderFilter(netter1);
        ApplyUuvColliderFilter(netter2);
        if (netTopology == NetTopology.TriangularThreeAnchor)
        {
            ApplyUuvColliderFilter(herder);
        }
        else if (netTopology == NetTopology.QuadrilateralFourAnchor)
        {
            ApplyUuvColliderFilter(netter3);
            ApplyUuvColliderFilter(netter4);
        }
    }

    private void ApplyUuvColliderFilter(Transform netter)
    {
        ObiCollider obiCollider = GetObiCollider(netter);
        if (obiCollider != null)
        {
            obiCollider.Filter = GetUuvColliderFilter();
        }
    }

    private static int ClampCategory(int category)
    {
        return Mathf.Clamp(category, ObiUtils.MinCategory, ObiUtils.MaxCategory);
    }

    private static int CategoryMask(int category)
    {
        return 1 << ClampCategory(category);
    }

    private void CaptureCurrentNetState()
    {
        ResolveGeneratedRoot();
        if (generatedRoot == null)
        {
            hasSavedNetState = false;
            savedRigidbodies = new RigidbodySnapshot[0];
            return;
        }

        savedRootLocalPosition = generatedRoot.localPosition;
        savedRootLocalRotation = generatedRoot.localRotation;

        Rigidbody[] rigidbodies = generatedRoot.GetComponentsInChildren<Rigidbody>(true);
        savedRigidbodies = new RigidbodySnapshot[rigidbodies.Length];
        for (int i = 0; i < rigidbodies.Length; ++i)
        {
            savedRigidbodies[i] = new RigidbodySnapshot
            {
                body = rigidbodies[i],
                localPosition = rigidbodies[i].transform.localPosition,
                localRotation = rigidbodies[i].transform.localRotation
            };
        }

        hasSavedNetState = true;
    }

    private void RestoreSavedNetState()
    {
        if (generatedRoot == null || !hasSavedNetState)
        {
            return;
        }

        generatedRoot.localPosition = savedRootLocalPosition;
        generatedRoot.localRotation = savedRootLocalRotation;

        for (int i = 0; i < savedRigidbodies.Length; ++i)
        {
            Rigidbody body = savedRigidbodies[i].body;
            if (body == null)
            {
                continue;
            }

            body.transform.localPosition = savedRigidbodies[i].localPosition;
            body.transform.localRotation = savedRigidbodies[i].localRotation;
            body.linearVelocity = Vector3.zero;
            body.angularVelocity = Vector3.zero;
            body.Sleep();
        }

        ResetObiActorsToBlueprints();

        NetSurfaceModel model = generatedRoot.GetComponent<NetSurfaceModel>();
        if (model != null)
        {
            model.ResolveHierarchy();
        }
    }

    private void ResetObiActorsToBlueprints()
    {
        ObiActor[] actors = generatedRoot.GetComponentsInChildren<ObiActor>(true);
        for (int i = 0; i < actors.Length; ++i)
        {
            actors[i].ResetParticles();
        }
    }

    private void ResolveReferences()
    {
        if (areaManager == null)
        {
            areaManager = GetComponent<CatchAreaManager>();
        }

        if (areaManager == null)
        {
            areaManager = FindFirstObjectByType<CatchAreaManager>();
        }

        if (netter1 == null && areaManager != null && areaManager.netter1 != null)
        {
            netter1 = areaManager.netter1.transform;
        }

        if (netter2 == null && areaManager != null && areaManager.netter2 != null)
        {
            netter2 = areaManager.netter2.transform;
        }

        if (herder == null && areaManager != null && areaManager.herder != null)
        {
            herder = areaManager.herder.transform;
        }

        if (netter1 == null)
        {
            GameObject uuv7 = GameObject.Find("UUV7");
            if (uuv7 != null)
            {
                netter1 = uuv7.transform;
            }
        }

        if (netter2 == null)
        {
            GameObject uuv8 = GameObject.Find("UUV8");
            if (uuv8 != null)
            {
                netter2 = uuv8.transform;
            }
        }

        if (netter1MarkPosition == null && netter1 != null)
        {
            Transform marker = netter1.Find("MarkPosition");
            if (marker != null)
            {
                netter1MarkPosition = marker;
            }
        }

        if (netter2MarkPosition == null && netter2 != null)
        {
            Transform marker = netter2.Find("MarkPosition");
            if (marker != null)
            {
                netter2MarkPosition = marker;
            }
        }

        if (herderMarkPosition == null && herder != null)
        {
            Transform marker = herder.Find("MarkPosition");
            if (marker != null)
            {
                herderMarkPosition = marker;
            }
        }
    }

    [ContextMenu("Clear Generated Net")]
    public void ClearGeneratedNet()
    {
        ResolveGeneratedRoot();
        if (generatedRoot == null)
        {
            return;
        }

        GameObject oldRoot = generatedRoot.gameObject;
        generatedRoot = null;
        solver = null;

        if (Application.isPlaying)
        {
            oldRoot.SetActive(false);
            Destroy(oldRoot);
        }
        else
        {
            DestroyImmediate(oldRoot);
        }
    }

#if UNITY_EDITOR
    public void DeleteGeneratedBlueprintAsset()
    {
        if (Application.isPlaying)
        {
            return;
        }

        fallbackMaterial = null;

        string path = generatedBlueprintAssetPath;
        if (!string.IsNullOrEmpty(activeEditorBlueprintAssetPath))
        {
            path = activeEditorBlueprintAssetPath;
        }

        if (!string.IsNullOrEmpty(path) && AssetDatabase.LoadAssetAtPath<Object>(path) != null)
        {
            DeleteAssetIfExists(path);
        }

        if (!string.IsNullOrEmpty(path))
        {
            string materialPath = GetSiblingAssetPath(path, "GeneratedFishNet_FallbackRopeMaterial.mat");
            DeleteAssetIfExists(materialPath);
        }
    }
#endif

    public void SetGeneratedRootForEditor(Transform root)
    {
        generatedRoot = root;
    }

    private void ResolveGeneratedRoot()
    {
        if (generatedRoot != null)
        {
            return;
        }

        Transform existingRoot = transform.Find(GeneratedRootName);
        if (existingRoot != null)
        {
            generatedRoot = existingRoot;
        }
    }

    private void CreateRootAndSolver()
    {
        GameObject rootObject = new GameObject(GeneratedRootName);
        generatedRoot = rootObject.transform;
        generatedRoot.SetParent(transform, false);

        GameObject solverObject = new GameObject("ObiSolver", typeof(ObiSolver));
        solverObject.transform.SetParent(generatedRoot, false);
        solver = solverObject.GetComponent<ObiSolver>();
        solver.backendType = GetCompatibleBackend();
        solver.substeps = 4;
        solver.gravity = solverGravity;
        solver.gravitySpace = Space.World;
        solver.parameters.gravity = solver.transform.InverseTransformVector(solverGravity);
        solver.parameters.surfaceCollisionIterations = surfaceCollisionIterations;
        solver.parameters.surfaceCollisionTolerance = surfaceCollisionTolerance;
        ConfigureSolverConstraintGroups();
        solver.distanceConstraintParameters.iterations = 8;
        solver.pinConstraintParameters.iterations = 4;
        solver.parameters.sleepThreshold = 0.001f;
        solver.PushSolverParameters();
    }

    private ObiSolver.BackendType GetCompatibleBackend()
    {
        if (solverBackend == ObiSolver.BackendType.Compute && !SystemInfo.supportsComputeShaders)
        {
            Debug.LogWarning(
                "FishNetGenerator switched Obi backend from Compute to Burst because this runtime does not support compute shaders.",
                this);
            return ObiSolver.BackendType.Burst;
        }

        if (solverBackend != ObiSolver.BackendType.Compute || !fallbackToBurstOnOpenGL)
        {
            return solverBackend;
        }

        UnityEngine.Rendering.GraphicsDeviceType graphicsDevice = SystemInfo.graphicsDeviceType;
        bool isOpenGL =
            graphicsDevice == UnityEngine.Rendering.GraphicsDeviceType.OpenGLCore ||
            graphicsDevice == UnityEngine.Rendering.GraphicsDeviceType.OpenGLES2 ||
            graphicsDevice == UnityEngine.Rendering.GraphicsDeviceType.OpenGLES3;

        if (!isOpenGL)
        {
            return solverBackend;
        }

        Debug.LogWarning(
            $"FishNetGenerator switched Obi backend from Compute to Burst because {graphicsDevice} can hit compute storage-buffer binding limits.",
            this);
        return ObiSolver.BackendType.Burst;
    }

    private void ConfigureSolverConstraintGroups()
    {
        if (!optimizeSolverConstraintGroups)
        {
            return;
        }

        solver.distanceConstraintParameters.enabled = true;
        solver.pinConstraintParameters.enabled = true;
        solver.collisionConstraintParameters.enabled = useColliderCollisions;
        solver.frictionConstraintParameters.enabled = useColliderCollisions;
        solver.particleCollisionConstraintParameters.enabled = useParticleCollisions;
        solver.particleFrictionConstraintParameters.enabled = useParticleCollisions;

        solver.bendingConstraintParameters.enabled = false;
        solver.skinConstraintParameters.enabled = false;
        solver.volumeConstraintParameters.enabled = false;
        solver.shapeMatchingConstraintParameters.enabled = false;
        solver.tetherConstraintParameters.enabled = false;
        solver.pinholeConstraintParameters.enabled = false;
        solver.stitchConstraintParameters.enabled = false;
        solver.densityConstraintParameters.enabled = false;
        solver.stretchShearConstraintParameters.enabled = false;
        solver.bendTwistConstraintParameters.enabled = false;
        solver.chainConstraintParameters.enabled = false;
    }

    private void CreateNet()
    {
        if (netTopology == NetTopology.TriangularThreeAnchor)
        {
            CreateTriangularNet();
            return;
        }

        if (netTopology == NetTopology.QuadrilateralFourAnchor)
        {
            CreateQuadrilateralNet();
            return;
        }

        CreateRectangularNet();
    }

    private void CreateQuadrilateralNet()
    {
        Transform topLeftAnchor = GetNetterAnchor(netter1, netter1MarkPosition);
        Transform topRightAnchor = GetNetterAnchor(netter2, netter2MarkPosition);
        Transform bottomLeftAnchor = GetNetterAnchor(netter3, netter3MarkPosition);
        Transform bottomRightAnchor = GetNetterAnchor(netter4, netter4MarkPosition);
        if (topLeftAnchor == null || topRightAnchor == null || bottomLeftAnchor == null || bottomRightAnchor == null)
        {
            Debug.LogError("FishNetGenerator QuadrilateralFourAnchor requires four corner transforms.", this);
            return;
        }

        Vector3 topLeft = GetAnchorPosition(topLeftAnchor, netter1LocalAttachmentOffset);
        Vector3 topRight = GetAnchorPosition(topRightAnchor, netter2LocalAttachmentOffset);
        Vector3 bottomLeft = GetAnchorPosition(bottomLeftAnchor, netter3LocalAttachmentOffset);
        Vector3 bottomRight = GetAnchorPosition(bottomRightAnchor, netter4LocalAttachmentOffset);

        ObiCollider[,] nodes = new ObiCollider[resolution.x + 1, resolution.y + 1];
        Transform[] layers = CreateLayers();
        for (int y = 0; y <= resolution.y; y++)
        {
            float vertical = y / (float)resolution.y;
            Vector3 rowLeft = Vector3.Lerp(topLeft, bottomLeft, vertical);
            Vector3 rowRight = Vector3.Lerp(topRight, bottomRight, vertical);
            for (int x = 0; x <= resolution.x; x++)
            {
                float horizontal = x / (float)resolution.x;
                Vector3 position = Vector3.Lerp(rowLeft, rowRight, horizontal);
                nodes[x, y] = CreateNode(layers[Mathf.Min(y, layers.Length - 1)], $"Q_{y}_{x}", position);
            }
        }

        CreateGridRopes(nodes);
        CreateLeadRope(topLeft, topLeftAnchor, nodes[0, 0], "Lead_TopLeft");
        CreateLeadRope(topRight, topRightAnchor, nodes[resolution.x, 0], "Lead_TopRight");
        CreateLeadRope(bottomLeft, bottomLeftAnchor, nodes[0, resolution.y], "Lead_BottomLeft");
        CreateLeadRope(bottomRight, bottomRightAnchor, nodes[resolution.x, resolution.y], "Lead_BottomRight");
    }

    private void CreateRectangularNet()
    {
        Transform anchorTransformA = GetNetterAnchor(netter1, netter1MarkPosition);
        Transform anchorTransformB = GetNetterAnchor(netter2, netter2MarkPosition);
        Vector3 anchorA = GetAnchorPosition(anchorTransformA, netter1LocalAttachmentOffset);
        Vector3 anchorB = GetAnchorPosition(anchorTransformB, netter2LocalAttachmentOffset);
        Vector3 right = Vector3.ProjectOnPlane(anchorB - anchorA, Vector3.up);
        if (right.sqrMagnitude < 1e-6f)
        {
            right = transform.right.sqrMagnitude > 1e-6f ? transform.right : Vector3.right;
        }
        right.Normalize();

        float netWidth = CalculateNetWidth(anchorA, anchorB);
        float cellWidth = netWidth / resolution.x;
        float netHeight = size.y * resolution.y;
        Vector3 topCenter = (anchorA + anchorB) * 0.5f + netWorldOffset;
        Vector3 topLeft = topCenter - right * (netWidth * 0.5f);
        Vector3 up = Vector3.up;

        ObiCollider[,] nodes = new ObiCollider[resolution.x + 1, resolution.y + 1];
        Transform[] layers = CreateLayers();

        for (int y = 0; y <= resolution.y; ++y)
        {
            for (int x = 0; x <= resolution.x; ++x)
            {
                Vector3 position = topLeft + right * (x * cellWidth) - up * (y * size.y);
                nodes[x, y] = CreateNode(layers[y], $"L_{x}", position);
            }
        }

        CreateHorizontalMarkers(topLeft, right, up, netWidth, netHeight);
        CreateGridRopes(nodes);
        CreateLeadRope(anchorA, GetUuvAttachmentTarget(netter1, anchorTransformA), nodes[0, 0], "LeadRope_Left");
        CreateLeadRope(anchorB, GetUuvAttachmentTarget(netter2, anchorTransformB), nodes[resolution.x, 0], "LeadRope_Right");
        ConfigureNetModel(netWidth, netHeight);
    }

    private void CreateTriangularNet()
    {
        Transform herderAnchor = GetNetterAnchor(herder, herderMarkPosition);
        Transform netter1Anchor = GetNetterAnchor(netter1, netter1MarkPosition);
        Transform netter2Anchor = GetNetterAnchor(netter2, netter2MarkPosition);
        Vector3 anchorA = GetAnchorPosition(herderAnchor, herderLocalAttachmentOffset) + netWorldOffset;
        Vector3 anchorB = GetAnchorPosition(netter1Anchor, netter1LocalAttachmentOffset) + netWorldOffset;
        Vector3 anchorC = GetAnchorPosition(netter2Anchor, netter2LocalAttachmentOffset) + netWorldOffset;

        Vector3 centroid = (anchorA + anchorB + anchorC) / 3f;
        float inset = Mathf.Max(0f, leadRopeLength);
        Vector3 vertexA = InsetTowardsCentroid(anchorA, centroid, inset);
        Vector3 vertexB = InsetTowardsCentroid(anchorB, centroid, inset);
        Vector3 vertexC = InsetTowardsCentroid(anchorC, centroid, inset);
        int subdivisions = triangularResolution;

        ObiCollider[][] nodes = new ObiCollider[subdivisions + 1][];
        List<Transform> flattenedNodes = new List<Transform>((subdivisions + 1) * (subdivisions + 2) / 2);
        for (int row = 0; row <= subdivisions; row++)
        {
            GameObject layer = new GameObject($"TriangleLayer{row}");
            layer.transform.SetParent(generatedRoot, false);
            nodes[row] = new ObiCollider[row + 1];

            float t = row / (float)subdivisions;
            for (int column = 0; column <= row; column++)
            {
                float u = row == 0 ? 0f : column / (float)row;
                Vector3 basePoint = Vector3.Lerp(vertexB, vertexC, u);
                Vector3 position = Vector3.Lerp(vertexA, basePoint, t);
                ObiCollider node = CreateNode(layer.transform, $"T_{row}_{column}", position);
                nodes[row][column] = node;
                flattenedNodes.Add(node.transform);
            }
        }

        CreateTriangularGridRopes(nodes, subdivisions);
        CreateLeadRope(anchorA, GetUuvAttachmentTarget(herder, herderAnchor), nodes[0][0], "LeadRope_Herder");
        CreateLeadRope(anchorB, GetUuvAttachmentTarget(netter1, netter1Anchor), nodes[subdivisions][0], "LeadRope_Netter1");
        CreateLeadRope(anchorC, GetUuvAttachmentTarget(netter2, netter2Anchor), nodes[subdivisions][subdivisions], "LeadRope_Netter2");

        NetSurfaceModel model = EnsureNetSurfaceModel();
        model.ConfigureTriangularSurface(
            nodes[0][0].transform,
            nodes[subdivisions][0].transform,
            nodes[subdivisions][subdivisions].transform,
            flattenedNodes,
            subdivisions);
        model.SetTargetGeometry(Vector3.Distance(vertexB, vertexC), GetTriangleHeight(vertexA, vertexB, vertexC));
    }

    private static Vector3 InsetTowardsCentroid(Vector3 vertex, Vector3 centroid, float inset)
    {
        return Vector3.MoveTowards(vertex, centroid, Mathf.Min(inset, Vector3.Distance(vertex, centroid) * 0.8f));
    }

    private static float GetTriangleHeight(Vector3 apex, Vector3 baseA, Vector3 baseB)
    {
        Vector3 baseDirection = baseB - baseA;
        if (baseDirection.sqrMagnitude < 1e-8f)
        {
            return 0f;
        }

        Vector3 offset = apex - baseA;
        return (offset - Vector3.Project(offset, baseDirection)).magnitude;
    }

    private void CreateTriangularGridRopes(ObiCollider[][] nodes, int subdivisions)
    {
        for (int row = 0; row < subdivisions; row++)
        {
            for (int column = 0; column <= row; column++)
            {
                CreatePinnedRope(nodes[row][column], nodes[row + 1][column], $"Rope_TL_{row}_{column}");
                CreatePinnedRope(nodes[row][column], nodes[row + 1][column + 1], $"Rope_TR_{row}_{column}");
            }
        }

        for (int row = 1; row <= subdivisions; row++)
        {
            for (int column = 0; column < row; column++)
            {
                CreatePinnedRope(nodes[row][column], nodes[row][column + 1], $"Rope_TH_{row}_{column}");
            }
        }
    }

    private void CreatePinnedRope(ObiCollider nodeA, ObiCollider nodeB, string ropeName)
    {
        Vector3 pointA = nodeA.transform.position;
        Vector3 pointB = nodeB.transform.position;
        Vector3 direction = pointB - pointA;
        float length = direction.magnitude;
        if (length < 1e-6f)
        {
            return;
        }

        // The particle attachments below pin the rope endpoints to the node
        // transforms themselves. Build the blueprint between those same points:
        // shortening it to the node surfaces here would pre-stretch every edge
        // as soon as Obi resolves the attachments.
        ObiRope rope = CreateRope(pointA, pointB, ropeName);
        PinRope(rope, nodeA, nodeB);
    }

    private Transform GetNetterAnchor(Transform netter, Transform marker)
    {
        if (marker != null)
        {
            return marker;
        }

        if (netter == null)
        {
            return null;
        }

        Transform childMarker = netter.Find("MarkPosition");
        return childMarker != null ? childMarker : netter;
    }

    private Vector3 GetAnchorPosition(Transform anchor, Vector3 localOffset)
    {
        return anchor.TransformPoint(localOffset);
    }

    private float CalculateNetWidth(Vector3 anchorA, Vector3 anchorB)
    {
        if (fitWidthBetweenNettersOnBuild)
        {
            return Mathf.Max(nodeSize, Vector3.Distance(anchorA, anchorB) - leadRopeLength * 2f);
        }

        if (fixedNetWidth > 0f)
        {
            return fixedNetWidth;
        }

        return Mathf.Max(nodeSize, resolution.x * size.x);
    }

    private Transform[] CreateLayers()
    {
        Transform[] layers = new Transform[resolution.y + 1];
        for (int y = 0; y <= resolution.y; ++y)
        {
            GameObject layer = new GameObject($"Layer{y + 1}");
            layer.transform.SetParent(generatedRoot, false);
            layers[y] = layer.transform;
        }

        return layers;
    }

    private ObiCollider CreateNode(Transform parent, string objectName, Vector3 position)
    {
        GameObject node = GameObject.CreatePrimitive(PrimitiveType.Cube);
        node.name = objectName;
        node.transform.SetParent(parent, true);
        node.transform.position = position;
        node.transform.localScale = Vector3.one * nodeSize;

        if (hideNodeRenderers && node.TryGetComponent(out Renderer renderer))
        {
            renderer.enabled = false;
        }

        Rigidbody rb = node.AddComponent<Rigidbody>();
        rb.mass = nodeMass;
        rb.useGravity = nodeUseGravity;
        rb.linearDamping = nodeLinearDamping;
        rb.angularDamping = nodeAngularDamping;

        ObiCollider obiCollider = node.AddComponent<ObiCollider>();
        obiCollider.Filter = GetNetPinColliderFilter();
        return obiCollider;
    }

    private void CreateHorizontalMarkers(Vector3 topLeft, Vector3 right, Vector3 up, float netWidth, float netHeight)
    {
        GameObject horizontal = new GameObject("Horizontal");
        horizontal.transform.SetParent(generatedRoot, false);

        for (int i = 0; i < 3; i++)
        {
            float u = i * 0.5f;
            GameObject marker = new GameObject($"H_{i}");
            marker.transform.SetParent(horizontal.transform, false);
            marker.transform.position = topLeft + right * (netWidth * u) - up * (netHeight * 0.5f);
        }
    }

    private void CreateGridRopes(ObiCollider[,] nodes)
    {
        for (int x = 0; x <= resolution.x; ++x)
        {
            for (int y = 0; y <= resolution.y; ++y)
            {
                Vector3 pos = nodes[x, y].transform.position;

                if (x < resolution.x)
                {
                    ObiRope rope = CreateRope(pos, nodes[x + 1, y].transform.position, $"Rope_H_{y}_{x}");
                    rope.stretchingScale = gridRopeRestLengthScale;
                    PinRope(rope, nodes[x, y], nodes[x + 1, y]);
                }

                if (y < resolution.y)
                {
                    ObiRope rope = CreateRope(pos, nodes[x, y + 1].transform.position, $"Rope_V_{x}_{y}");
                    rope.stretchingScale = gridRopeRestLengthScale;
                    PinRope(rope, nodes[x, y], nodes[x, y + 1]);
                }
            }
        }
    }

    private void CreateLeadRope(Vector3 anchorPosition, Transform uuvAnchor, ObiCollider netCorner, string ropeName)
    {
        Vector3 cornerPosition = netCorner.transform.position;
        // As with the internal grid, use the real attachment positions as the
        // blueprint endpoints. The UUV and corner attachments both operate at
        // transform centers, so an inset endpoint would create artificial
        // lead-rope tension before the ROVs have moved.
        ObiRope rope = CreateRope(anchorPosition, cornerPosition, ropeName, leadRopeSlackM);

        ObiParticleAttachment.AttachmentType uuvAttachmentType =
            useDynamicUuvAttachments && GetObiCollider(uuvAnchor) != null
                ? ObiParticleAttachment.AttachmentType.Dynamic
                : ObiParticleAttachment.AttachmentType.Static;

        AttachRopeEnd(rope, 0, uuvAnchor, anchorPosition, uuvAttachmentType, uuvAttachmentCompliance);
        AttachRopeEnd(rope, 1, netCorner.transform, cornerPosition, ObiParticleAttachment.AttachmentType.Dynamic);
    }

    private void PinRope(ObiRope rope, ObiCollider bodyA, ObiCollider bodyB)
    {
        AttachRopeEnd(rope, 0, bodyA.transform, bodyA.transform.position, ObiParticleAttachment.AttachmentType.Dynamic);
        AttachRopeEnd(rope, 1, bodyB.transform, bodyB.transform.position, ObiParticleAttachment.AttachmentType.Dynamic);
    }

    private void AttachRopeEnd(
        ObiRope rope,
        int groupIndex,
        Transform target,
        Vector3 fallbackTargetPosition,
        ObiParticleAttachment.AttachmentType attachmentType,
        float compliance = 0f)
    {
        ObiParticleAttachment attachment = rope.gameObject.AddComponent<ObiParticleAttachment>();
        attachment.attachmentType = attachmentType;
        attachment.target = target != null ? target : CreateStaticAnchor(fallbackTargetPosition, rope.name);
        attachment.particleGroup = rope.ropeBlueprint.groups[groupIndex];
        attachment.compliance = Mathf.Max(0f, compliance);
    }

    private Transform CreateStaticAnchor(Vector3 position, string ropeName)
    {
        GameObject anchor = new GameObject($"{ropeName}_UUVAnchor");
        anchor.transform.SetParent(generatedRoot, true);
        anchor.transform.position = position;
        return anchor.transform;
    }

    private Transform GetUuvAttachmentTarget(Transform netter, Transform marker)
    {
        if (!useDynamicUuvAttachments)
        {
            return marker != null ? marker : netter;
        }

        Transform target = netter;
        if (target == null && marker != null)
        {
            Rigidbody parentRigidbody = marker.GetComponentInParent<Rigidbody>();
            if (parentRigidbody != null)
            {
                target = parentRigidbody.transform;
            }
        }

        if (target == null)
        {
            return marker;
        }

        return EnsureObiCollider(target) != null ? target : marker;
    }

    private ObiCollider GetObiCollider(Transform target)
    {
        return target != null ? target.GetComponent<ObiCollider>() : null;
    }

    private ObiCollider EnsureObiCollider(Transform target)
    {
        if (target == null)
        {
            return null;
        }

        ObiCollider obiCollider = target.GetComponent<ObiCollider>();
        if (obiCollider != null)
        {
            return obiCollider;
        }

        if (!addMissingObiCollidersToNetters)
        {
            Debug.LogWarning(
                $"FishNetGenerator needs an ObiCollider on {target.name} for dynamic UUV attachment.",
                this);
            return null;
        }

        Collider unityCollider = target.GetComponent<Collider>();
        Rigidbody unityRigidbody = target.GetComponent<Rigidbody>();
        if (unityCollider == null || unityRigidbody == null)
        {
            Debug.LogWarning(
                $"FishNetGenerator cannot dynamically attach to {target.name}: it needs both Collider and Rigidbody on the same object.",
                this);
            return null;
        }

#if UNITY_EDITOR
        obiCollider = !Application.isPlaying
            ? Undo.AddComponent<ObiCollider>(target.gameObject)
            : target.gameObject.AddComponent<ObiCollider>();
#else
        obiCollider = target.gameObject.AddComponent<ObiCollider>();
#endif
        obiCollider.Filter = GetUuvColliderFilter();
        return obiCollider;
    }

    private ObiRope CreateRope(Vector3 pointA, Vector3 pointB, string ropeName, float midpointSlackM = 0f)
    {
        GameObject ropeObject = new GameObject(ropeName, typeof(ObiRope), typeof(ObiRopeLineRenderer));
        ropeObject.transform.SetParent(solver.transform, true);

        ObiRope rope = ropeObject.GetComponent<ObiRope>();
        ObiRopeLineRenderer ropeRenderer = ropeObject.GetComponent<ObiRopeLineRenderer>();
        ropeRenderer.material = GetRopeMaterial();
        ropeRenderer.uvScale = new Vector2(1, 5);

        ObiPathSmoother smoother = ropeObject.GetComponent<ObiPathSmoother>();
        if (smoother != null)
        {
            smoother.decimation = 0.1f;
        }

        ObiRopeBlueprint blueprint = ScriptableObject.CreateInstance<ObiRopeBlueprint>();
        blueprint.name = $"{ropeName}_Blueprint";
        blueprint.resolution = ropeBlueprintResolution;
        blueprint.thickness = ropeThickness;
        blueprint.pooledParticles = 0;

#if UNITY_EDITOR
        PersistBlueprintForEditorBeforePathEvents(blueprint);
#endif

        Vector3 localA = rope.transform.InverseTransformPoint(pointA);
        Vector3 localB = rope.transform.InverseTransformPoint(pointB);
        Vector3 span = localB - localA;
        Vector3 direction = span * 0.25f;
        int ropeFilter = GetRopeParticleFilter();

        blueprint.path.Clear();
        if (midpointSlackM > 0f && span.sqrMagnitude > 1e-8f)
        {
            // A two-particle rope is a rigid straight segment: it has no point
            // that can bend when its endpoints move closer together. Insert an
            // arced midpoint so the generated lead has several particles and a
            // finite slack length while keeping its two physical endpoints.
            Vector3 spanDirection = span.normalized;
            Vector3 reference = Mathf.Abs(Vector3.Dot(spanDirection, Vector3.up)) < 0.95f
                ? Vector3.up
                : Vector3.forward;
            Vector3 perpendicular = Vector3.Cross(spanDirection, reference).normalized;
            Vector3 localMid = (localA + localB) * 0.5f + perpendicular * midpointSlackM;
            Vector3 tangentA = (localMid - localA) * 0.25f;
            Vector3 tangentMid = span * 0.20f;
            Vector3 tangentB = (localB - localMid) * 0.25f;

            blueprint.path.AddControlPoint(localA, -tangentA, tangentA, Vector3.up, controlPointMass, controlPointMass, 1, ropeFilter, Color.white, "A");
            blueprint.path.AddControlPoint(localMid, -tangentMid, tangentMid, Vector3.up, controlPointMass, controlPointMass, 1, ropeFilter, Color.white, "Slack");
            blueprint.path.AddControlPoint(localB, -tangentB, tangentB, Vector3.up, controlPointMass, controlPointMass, 1, ropeFilter, Color.white, "B");
        }
        else
        {
            blueprint.path.AddControlPoint(localA, -direction, direction, Vector3.up, controlPointMass, controlPointMass, 1, ropeFilter, Color.white, "A");
            blueprint.path.AddControlPoint(localB, -direction, direction, Vector3.up, controlPointMass, controlPointMass, 1, ropeFilter, Color.white, "B");
        }
        blueprint.path.FlushEvents();
        blueprint.GenerateImmediate();

        rope.ropeBlueprint = blueprint;
        rope.collisionMaterial = ropeCollisionMaterial;
        rope.surfaceCollisions = useSurfaceCollisions;
        rope.selfCollisions = useSelfCollisions;
        return rope;
    }

    private Material GetRopeMaterial()
    {
        if (material != null)
        {
            return material;
        }

        if (fallbackMaterial != null)
        {
            return fallbackMaterial;
        }

#if UNITY_EDITOR
        if (!Application.isPlaying && !string.IsNullOrEmpty(activeEditorBlueprintAssetPath))
        {
            string materialPath = GetSiblingAssetPath(activeEditorBlueprintAssetPath, "GeneratedFishNet_FallbackRopeMaterial.mat");
            fallbackMaterial = AssetDatabase.LoadAssetAtPath<Material>(materialPath);
            if (fallbackMaterial == null)
            {
                fallbackMaterial = CreateFallbackMaterial();
                AssetDatabase.CreateAsset(fallbackMaterial, materialPath);
                EditorUtility.SetDirty(fallbackMaterial);
            }

            return fallbackMaterial;
        }
#endif

        fallbackMaterial = CreateFallbackMaterial();
        return fallbackMaterial;
    }

    private static Material CreateFallbackMaterial()
    {
        Shader shader = Shader.Find("Universal Render Pipeline/Unlit");
        if (shader == null)
        {
            shader = Shader.Find("Unlit/Color");
        }

        if (shader == null)
        {
            shader = Shader.Find("Sprites/Default");
        }

        if (shader == null)
        {
            shader = Shader.Find("Standard");
        }

        Material mat = new Material(shader);
        mat.name = "GeneratedFishNet_FallbackRopeMaterial";
        Color color = new Color(0.05f, 0.85f, 0.95f, 1f);
        if (mat.HasProperty("_BaseColor"))
        {
            mat.SetColor("_BaseColor", color);
        }

        if (mat.HasProperty("_Color"))
        {
            mat.SetColor("_Color", color);
        }

        return mat;
    }

#if UNITY_EDITOR
    private void PersistBlueprintForEditorBeforePathEvents(ObiRopeBlueprint blueprint)
    {
        if (Application.isPlaying || string.IsNullOrEmpty(activeEditorBlueprintAssetPath))
        {
            return;
        }

        if (!editorBlueprintMainAssetCreated)
        {
            AssetDatabase.CreateAsset(blueprint, activeEditorBlueprintAssetPath);
            editorBlueprintMainAssetCreated = true;
            editorBlueprintMainAsset = blueprint;
        }
        else
        {
            Object mainAsset = editorBlueprintMainAsset != null
                ? editorBlueprintMainAsset
                : AssetDatabase.LoadMainAssetAtPath(activeEditorBlueprintAssetPath);
            if (mainAsset == null)
            {
                throw new System.InvalidOperationException(
                    $"FishNetGenerator could not resolve Blueprint asset {activeEditorBlueprintAssetPath}.");
            }
            AssetDatabase.AddObjectToAsset(blueprint, mainAsset);
        }

        EditorUtility.SetDirty(blueprint);
    }

    private static void EnsureAssetFolder(string assetPath)
    {
        int lastSlash = assetPath.LastIndexOf('/');
        if (lastSlash <= 0)
        {
            return;
        }

        string folderPath = assetPath.Substring(0, lastSlash);
        if (AssetDatabase.IsValidFolder(folderPath))
        {
            return;
        }

        string[] parts = folderPath.Split('/');
        string current = parts[0];
        for (int i = 1; i < parts.Length; i++)
        {
            string next = $"{current}/{parts[i]}";
            if (!AssetDatabase.IsValidFolder(next))
            {
                AssetDatabase.CreateFolder(current, parts[i]);
            }

            current = next;
        }
    }

    private static string GetSiblingAssetPath(string assetPath, string siblingFileName)
    {
        int lastSlash = assetPath.LastIndexOf('/');
        if (lastSlash < 0)
        {
            return siblingFileName;
        }

        return $"{assetPath.Substring(0, lastSlash + 1)}{siblingFileName}";
    }

    private static void DeleteAssetIfExists(string assetPath)
    {
        if (!string.IsNullOrEmpty(assetPath) && AssetDatabase.LoadAssetAtPath<Object>(assetPath) != null)
        {
            AssetDatabase.DeleteAsset(assetPath);
        }
    }
#endif

    private void ConfigureNetModel(float netWidth, float netHeight)
    {
        NetSurfaceModel model = EnsureNetSurfaceModel();
        model.SetTargetGeometry(netWidth, netHeight);
        model.ResolveHierarchy();
    }

    private NetSurfaceModel EnsureNetSurfaceModel()
    {
        if (generatedRoot == null)
        {
            return null;
        }

        NetSurfaceModel model = generatedRoot.GetComponent<NetSurfaceModel>();
        if (model == null)
        {
            model = generatedRoot.gameObject.AddComponent<NetSurfaceModel>();
        }

        model.ResolveHierarchy();
        return model;
    }
}
