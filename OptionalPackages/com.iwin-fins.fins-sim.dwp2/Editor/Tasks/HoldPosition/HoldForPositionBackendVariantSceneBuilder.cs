using System;
using System.Collections.Generic;
using System.IO;
using FinsSim.Actuators;
using FinsSim.Hydrodynamics;
using NWH.DWP2.WaterData;
using NWH.DWP2.WaterObjects;
using Unity.MLAgents;
using Unity.MLAgents.Areas;
using Unity.MLAgents.Policies;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

/// <summary>
/// Builds HoldForPosition backend/actuator ablation scenes from the
/// normalized-max-force Fossen No-DR baseline.  This class deliberately keeps
/// task, action, water-provider, and parallel-area settings identical; each
/// variant changes only its declared hydrodynamic backend, DR mode, or
/// thruster response model.
/// </summary>
public static class HoldForPositionBackendVariantSceneBuilder
{
    public const string BaselineScenePath =
        "Assets/FinsSimUnity/Tasks/HoldPosition/Scenes/HoldForPosition_Fossen_Parallel_30s_NormalizedMaxForce_NoDR.unity";

    public const string Dwp2NoDrScenePath =
        "Assets/FinsSimUnity/Tasks/HoldPosition/Scenes/HoldForPosition_DWP2Mesh_Parallel_30s_NormalizedMaxForce_NoDR.unity";
    public const string Dwp2DrScenePath =
        "Assets/FinsSimUnity/Tasks/HoldPosition/Scenes/HoldForPosition_DWP2Mesh_Parallel_30s_NormalizedMaxForce_DR.unity";
    public const string EquivalentBoxNoDrScenePath =
        "Assets/FinsSimUnity/Tasks/HoldPosition/Scenes/HoldForPosition_LearningToSwimEquivalentBox_Parallel_30s_NormalizedMaxForce_NoDR.unity";
    public const string EquivalentBoxDrScenePath =
        "Assets/FinsSimUnity/Tasks/HoldPosition/Scenes/HoldForPosition_LearningToSwimEquivalentBox_Parallel_30s_NormalizedMaxForce_DR.unity";
    public const string FossenInstantThrustersScenePath =
        "Assets/FinsSimUnity/Tasks/HoldPosition/Scenes/HoldForPosition_Fossen_Parallel_30s_NormalizedMaxForce_NoDR_InstantThrusters.unity";
    public const string FossenThrusterCap25ScenePath =
        "Assets/FinsSimUnity/Tasks/HoldPosition/Scenes/HoldForPosition_Fossen_Parallel_30s_NormalizedMaxForce_NoDR_ThrusterCap25.unity";
    public const string FossenWrenchScaledScenePath =
        "Assets/FinsSimUnity/Tasks/HoldPosition/Scenes/HoldForPosition_Fossen_Parallel_30s_NormalizedMaxForce_NoDR_WrenchScaled.unity";

    const string Dwp2VehiclePrefabPath =
        "Assets/Models/FinsROV/Calibrated/FinsROV_DWP2Calibrated.prefab";
    const string EquivalentBoxProfilePath =
        "Assets/Models/FinsROV/Hydrodynamics/FinsROV_HydrodynamicsProfile_LearningToSwimEquivalentBox.asset";
    const string EquivalentBoxDrProfilePath =
        "Assets/Models/FinsROV/Hydrodynamics/DomainRandomizationProfile_LearningToSwimEquivalentBox.asset";
    const string Dwp2YawDrProfilePath =
        "Assets/Models/FinsROV/Hydrodynamics/DomainRandomizationProfile_DWP2YawOnly.asset";

    const int ThirtySecondEpisodeMaxStep = 1500;
    const float Dwp2FluidDensity = 997f;
    // DWP2 mesh fixture buoyancy setting.  This remains separate from the
    // hydrodynamic force/torque model and is shared by regenerated variants.
    const float Dwp2BuoyancyCoefficient = 0.48f;
    const float Dwp2HydrodynamicCoefficient = 0.20f;
    const int Dwp2PhysicsMeshTargetTriangleCount = 256;
    const float Dwp2HydrodynamicCoefficientMin = 0.10f;
    const float Dwp2HydrodynamicCoefficientMax = 0.52f;
    const float ThrusterActionCap25 = 0.25f;

    enum VariantKind
    {
        Dwp2NoDr,
        Dwp2Dr,
        EquivalentBoxNoDr,
        EquivalentBoxDr,
        FossenInstantThrusters,
        FossenThrusterCap25,
        FossenWrenchScaled,
    }

    struct MeshEdge
    {
        public int a;
        public int b;
        public float sqrLength;
    }

    [MenuItem("FinsSim/RL/Prepare HoldForPosition Backend Variants/Prepare All")]
    public static void PrepareAll()
    {
        PrepareVariant(Dwp2NoDrScenePath, VariantKind.Dwp2NoDr);
        PrepareVariant(Dwp2DrScenePath, VariantKind.Dwp2Dr);
        PrepareVariant(EquivalentBoxNoDrScenePath, VariantKind.EquivalentBoxNoDr);
        PrepareVariant(EquivalentBoxDrScenePath, VariantKind.EquivalentBoxDr);
        PrepareVariant(FossenInstantThrustersScenePath, VariantKind.FossenInstantThrusters);
        PrepareVariant(FossenThrusterCap25ScenePath, VariantKind.FossenThrusterCap25);
        PrepareVariant(FossenWrenchScaledScenePath, VariantKind.FossenWrenchScaled);
        Debug.Log("Prepared all HoldForPosition backend-variant scenes.");
    }

    [MenuItem("FinsSim/RL/Prepare HoldForPosition Backend Variants/DWP2 Mesh No-DR")]
    public static void PrepareDwp2NoDr() => PrepareVariant(Dwp2NoDrScenePath, VariantKind.Dwp2NoDr);

    [MenuItem("FinsSim/RL/Prepare HoldForPosition Backend Variants/DWP2 Mesh DR")]
    public static void PrepareDwp2Dr() => PrepareVariant(Dwp2DrScenePath, VariantKind.Dwp2Dr);

    [MenuItem("FinsSim/RL/Prepare HoldForPosition Backend Variants/Equivalent Box No-DR")]
    public static void PrepareEquivalentBoxNoDr() => PrepareVariant(EquivalentBoxNoDrScenePath, VariantKind.EquivalentBoxNoDr);

    [MenuItem("FinsSim/RL/Prepare HoldForPosition Backend Variants/Equivalent Box DR")]
    public static void PrepareEquivalentBoxDr() => PrepareVariant(EquivalentBoxDrScenePath, VariantKind.EquivalentBoxDr);

    [MenuItem("FinsSim/RL/Prepare HoldForPosition Backend Variants/Fossen Instant Thrusters")]
    public static void PrepareFossenInstantThrusters() => PrepareVariant(FossenInstantThrustersScenePath, VariantKind.FossenInstantThrusters);

    [MenuItem("FinsSim/RL/Prepare HoldForPosition Backend Variants/Fossen Thruster Cap 25 Percent")]
    public static void PrepareFossenThrusterCap25() => PrepareVariant(FossenThrusterCap25ScenePath, VariantKind.FossenThrusterCap25);

    [MenuItem("FinsSim/RL/Prepare HoldForPosition Backend Variants/Fossen Wrench Scaled")]
    public static void PrepareFossenWrenchScaled() => PrepareVariant(FossenWrenchScaledScenePath, VariantKind.FossenWrenchScaled);

    [MenuItem("FinsSim/RL/Prepare HoldForPosition Backend Variants/Validate Current Scene")]
    public static void ValidateCurrentScene()
    {
        Scene scene = SceneManager.GetActiveScene();
        ValidateScene(scene, VariantForPath(scene.path));
        Debug.Log($"Validated HoldForPosition backend variant: {scene.path}");
    }

    public static void EnsurePrepared(string scenePath)
    {
        VariantKind kind = VariantForPath(scenePath);
        // These files are generated backend fixtures rather than hand-authored
        // scenes. Recreate them from the common baseline at build time so a
        // stale fixture cannot silently retain an earlier backend setting.
        PrepareVariant(scenePath, kind);
    }

    static void PrepareVariant(string outputPath, VariantKind kind)
    {
        Scene source = EditorSceneManager.OpenScene(BaselineScenePath, OpenSceneMode.Single);
        if (!EditorSceneManager.SaveScene(source, outputPath, true))
        {
            throw new InvalidOperationException($"Unable to copy {BaselineScenePath} to {outputPath}.");
        }

        Scene scene = EditorSceneManager.OpenScene(outputPath, OpenSceneMode.Single);
        switch (kind)
        {
            case VariantKind.Dwp2NoDr:
                ReplaceFossenVehicleWithDwp2(scene, enableDr: false, areaCount: 1);
                break;
            case VariantKind.Dwp2Dr:
                ReplaceFossenVehicleWithDwp2(scene, enableDr: true, areaCount: 1);
                break;
            case VariantKind.EquivalentBoxNoDr:
                ConfigureEquivalentBox(scene, enableDr: false);
                break;
            case VariantKind.EquivalentBoxDr:
                ConfigureEquivalentBox(scene, enableDr: true);
                break;
            case VariantKind.FossenInstantThrusters:
                ConfigureFossenInstantThrusters(scene);
                break;
            case VariantKind.FossenThrusterCap25:
                ConfigureFossenThrusterCap25(scene);
                break;
            case VariantKind.FossenWrenchScaled:
                ConfigureFossenWrenchScaled(scene);
                break;
            default:
                throw new ArgumentOutOfRangeException(nameof(kind), kind, null);
        }

        ValidateScene(scene, kind);
        EditorSceneManager.MarkSceneDirty(scene);
        if (!EditorSceneManager.SaveScene(scene))
        {
            throw new InvalidOperationException($"Unable to save {outputPath}.");
        }

        // Verify the serialized fixture, rather than only its in-memory state.
        Scene persisted = EditorSceneManager.OpenScene(outputPath, OpenSceneMode.Single);
        ValidateScene(persisted, kind);
    }

    static void ReplaceFossenVehicleWithDwp2(Scene scene, bool enableDr, int areaCount)
    {
        // DWP2 chooses the last enabled provider whose trigger contains the
        // vehicle.  Both DWP2 fixtures intentionally use only a flat provider:
        // the DR condition varies the mesh-force coefficient, not waves or
        // currents.  This keeps the backend comparison interpretable and
        // avoids sampling a second, inactive-in-spirit wave provider.
        ConfigureDwp2FlatWaterProvider();

        TrainingAreaReplicator replicator = FindReplicator();
        // DWP2 evaluates a physics mesh synchronously for every active area.
        // Serialize the topology explicitly: multi-binary has one Player per
        // area, whereas the separate multi-area fixture has one Player with
        // 1024 replicated areas.  Do not inherit the baseline's 2048-area
        // inspector value because it can be instantiated before the Python
        // communicator negotiates the requested area count.
        replicator.numAreas = areaCount;
        GameObject area = replicator.baseArea;
        HoldForPosition oldAgent = RequireSingleAgent(area, "baseline");
        Transform target = RequireTarget(area.transform);

        GameObject dwp2Prefab = AssetDatabase.LoadAssetAtPath<GameObject>(Dwp2VehiclePrefabPath);
        if (dwp2Prefab == null)
        {
            throw new InvalidOperationException($"Missing DWP2 training vehicle prefab: {Dwp2VehiclePrefabPath}.");
        }

        GameObject oldRoot = oldAgent.gameObject;
        GameObject vehicle = (GameObject)PrefabUtility.InstantiatePrefab(dwp2Prefab, area.transform);
        vehicle.name = "FinsROV_DWP2Mesh";
        vehicle.transform.SetLocalPositionAndRotation(oldRoot.transform.localPosition, oldRoot.transform.localRotation);
        vehicle.transform.localScale = oldRoot.transform.localScale;

        // The DWP2 calibration prefab is also used by legacy ControlForPosition
        // scenes, so it intentionally does not own a HoldForPosition component.
        // Add this task-specific agent only to the scene instance, never to the
        // reusable calibration prefab.
        HoldForPosition agent = vehicle.GetComponent<HoldForPosition>();
        if (agent == null)
        {
            agent = vehicle.AddComponent<HoldForPosition>();
        }
        CopyAgentConfiguration(oldRoot, oldAgent, vehicle, agent, target);
        CopyRigidbodyAndThrusterConfiguration(oldRoot, vehicle);
        DisableNonTrainingControlComponents(vehicle);
        ConfigureDwp2Hydrodynamics(vehicle, enableDr);

        UnityEngine.Object.DestroyImmediate(oldRoot);
        ConfigureParallelAgent(agent, target);
    }

    static void ConfigureEquivalentBox(Scene scene, bool enableDr)
    {
        TrainingAreaReplicator replicator = FindReplicator();
        GameObject area = replicator.baseArea;
        HoldForPosition agent = RequireSingleAgent(area, "Fossen baseline");
        HydrodynamicsController controller = agent.GetComponentInChildren<HydrodynamicsController>(true);
        HydrodynamicsProfile profile = AssetDatabase.LoadAssetAtPath<HydrodynamicsProfile>(EquivalentBoxProfilePath);
        if (controller == null || profile == null)
        {
            throw new InvalidOperationException("Equivalent-box scene requires a HydrodynamicsController and its profile asset.");
        }

        controller.enabled = true;
        controller.mode = HydrodynamicsMode.LearningToSwimEquivalentBox;
        controller.profile = profile;
        // `mode` is a scalar override on an instantiated prefab. Explicitly
        // record it so Unity persists the enum together with the profile.
        EditorUtility.SetDirty(controller);
        PrefabUtility.RecordPrefabInstancePropertyModifications(controller);
        ConfigureThrusterDynamics(agent.gameObject, firstOrder: true, null);

        DomainRandomizationCoordinator coordinator = RequireOrAddCoordinator(agent.gameObject);
        if (enableDr)
        {
            DomainRandomizationProfile drProfile = AssetDatabase.LoadAssetAtPath<DomainRandomizationProfile>(EquivalentBoxDrProfilePath);
            if (drProfile == null)
            {
                throw new InvalidOperationException($"Missing equivalent-box DR profile: {EquivalentBoxDrProfilePath}.");
            }
            ConfigureCoordinator(coordinator, drProfile, enabled: true, allowCommandLineOverrides: true);
        }
        else
        {
            ConfigureCoordinator(coordinator, null, enabled: false, allowCommandLineOverrides: false);
        }

        ConfigureParallelAgent(agent, RequireTarget(area.transform));
    }

    static void ConfigureFossenInstantThrusters(Scene scene)
    {
        TrainingAreaReplicator replicator = FindReplicator();
        GameObject area = replicator.baseArea;
        HoldForPosition agent = RequireSingleAgent(area, "Fossen baseline");
        HydrodynamicsController controller = agent.GetComponentInChildren<HydrodynamicsController>(true);
        if (controller == null || controller.mode != HydrodynamicsMode.Fossen6Dof)
        {
            throw new InvalidOperationException("Instant-thruster ablation must retain the baseline Fossen6Dof backend.");
        }

        ConfigureThrusterDynamics(agent.gameObject, firstOrder: false, null);
        ConfigureCoordinator(RequireOrAddCoordinator(agent.gameObject), null, enabled: false, allowCommandLineOverrides: false);
        ConfigureParallelAgent(agent, RequireTarget(area.transform));
    }

    static void ConfigureFossenThrusterCap25(Scene scene)
    {
        TrainingAreaReplicator replicator = FindReplicator();
        GameObject area = replicator.baseArea;
        HoldForPosition agent = RequireSingleAgent(area, "Fossen baseline");
        RequireFossenNoDr(agent, "Thruster-cap ablation");
        ConfigureParallelAgent(agent, RequireTarget(area.transform));
        agent.ConfigureNormalizedThrusterActionLimit(ThrusterActionCap25);
        EditorUtility.SetDirty(agent);
        PrefabUtility.RecordPrefabInstancePropertyModifications(agent);
    }

    static void ConfigureFossenWrenchScaled(Scene scene)
    {
        TrainingAreaReplicator replicator = FindReplicator();
        GameObject area = replicator.baseArea;
        HoldForPosition agent = RequireSingleAgent(area, "Fossen baseline");
        RequireFossenNoDr(agent, "Wrench-scaled ablation");
        ConfigureParallelAgent(agent, RequireTarget(area.transform));
        // Unity receives only the canonical eight-thruster command. The 6D
        // [surge, sway, heave, roll, pitch, yaw] scaling belongs to the Python
        // physical-wrench allocator, whose resolved training YAML is paired
        // with this scene. Keep Unity's direct-command envelope nominal.
        agent.ConfigureNormalizedThrusterActionLimit(1f);
        EditorUtility.SetDirty(agent);
        PrefabUtility.RecordPrefabInstancePropertyModifications(agent);
    }

    static void RequireFossenNoDr(HoldForPosition agent, string context)
    {
        HydrodynamicsController controller = agent.GetComponentInChildren<HydrodynamicsController>(true);
        if (controller == null || controller.mode != HydrodynamicsMode.Fossen6Dof)
        {
            throw new InvalidOperationException($"{context} must retain the baseline Fossen6Dof backend.");
        }
        ConfigureCoordinator(RequireOrAddCoordinator(agent.gameObject), null, enabled: false, allowCommandLineOverrides: false);
    }

    static void CopyAgentConfiguration(
        GameObject oldRoot,
        HoldForPosition oldAgent,
        GameObject newRoot,
        HoldForPosition newAgent,
        Transform target)
    {
        EditorUtility.CopySerialized(oldAgent, newAgent);
        newAgent.selfTransform = null;
        newAgent.targetTransform = target;
        newAgent.enabled = true;

        BehaviorParameters oldBehavior = oldRoot.GetComponent<BehaviorParameters>();
        BehaviorParameters newBehavior = newRoot.GetComponent<BehaviorParameters>();
        if (newBehavior == null)
        {
            newBehavior = newRoot.AddComponent<BehaviorParameters>();
        }
        if (oldBehavior == null)
        {
            throw new InvalidOperationException("Baseline training vehicle has no BehaviorParameters.");
        }
        EditorUtility.CopySerialized(oldBehavior, newBehavior);
        newBehavior.enabled = true;

        Component oldRequester = oldRoot.GetComponent("DecisionRequester");
        Component newRequester = newRoot.GetComponent("DecisionRequester");
        if (oldRequester != null && newRequester == null)
        {
            newRequester = newRoot.AddComponent(oldRequester.GetType());
        }
        if (oldRequester != null && newRequester != null && oldRequester.GetType() == newRequester.GetType())
        {
            EditorUtility.CopySerialized(oldRequester, newRequester);
            ((Behaviour)newRequester).enabled = true;
        }

        // A legacy ControlForPosition component can coexist on the calibration
        // prefab.  It must not request or apply actions in a HoldForPosition
        // training scene.
        foreach (Agent candidate in newRoot.GetComponents<Agent>())
        {
            if (candidate != newAgent)
            {
                candidate.enabled = false;
            }
        }
    }

    static void CopyRigidbodyAndThrusterConfiguration(GameObject sourceRoot, GameObject destinationRoot)
    {
        Rigidbody sourceBody = sourceRoot.GetComponent<Rigidbody>();
        Rigidbody destinationBody = destinationRoot.GetComponent<Rigidbody>();
        if (sourceBody == null || destinationBody == null)
        {
            throw new InvalidOperationException("Both training vehicles must have a root Rigidbody.");
        }
        EditorUtility.CopySerialized(sourceBody, destinationBody);

        Thruster[] source = ResolveOrderedThrusters(sourceRoot);
        Thruster[] destination = ResolveOrderedThrusters(destinationRoot);
        for (int i = 0; i < source.Length; i++)
        {
            EditorUtility.CopySerialized(source[i], destination[i]);
            destination[i].TargetRigidbody = destinationBody;
            destination[i].DynamicsModel = Thruster.ActuatorDynamicsModel.FirstOrderDelaySlew;
        }
        FinsROVAgentRuntime.EnsureThrusterControllerOrder(destinationRoot.GetComponent<ThrusterController>(), destination);
    }

    static void ConfigureDwp2Hydrodynamics(GameObject vehicle, bool enableDr)
    {
        HydrodynamicsController controller = vehicle.GetComponentInChildren<HydrodynamicsController>(true);
        if (controller != null)
        {
            controller.enabled = false;
        }

        WaterObject[] allWaterObjects = vehicle.GetComponentsInChildren<WaterObject>(true);
        var activeWaterObjects = new List<WaterObject>();
        foreach (WaterObject candidate in allWaterObjects)
        {
            if (candidate != null && candidate.isActiveAndEnabled)
            {
                activeWaterObjects.Add(candidate);
            }
        }
        if (activeWaterObjects.Count != 1)
        {
            throw new InvalidOperationException(
                $"DWP2 mesh variant requires exactly one active WaterObject; found {activeWaterObjects.Count}.");
        }

        WaterObject activeWaterObject = activeWaterObjects[0];
        activeWaterObject.fluidDensity = Dwp2FluidDensity;
        activeWaterObject.buoyantForceCoefficient = Dwp2BuoyancyCoefficient;
        activeWaterObject.hydrodynamicForceCoefficient = Dwp2HydrodynamicCoefficient;
        // The calibrated prefab already provides a closed, 680-triangle DWP2
        // simulation mesh.  Decimate that audited physics mesh to the training
        // budget instead of running the package decimator on the full CAD mesh;
        // the latter is unnecessarily expensive and can stall a batch build.
        RegenerateDwp2PhysicsMesh(activeWaterObject);
        // Both fixtures use a pure DWP2 mesh wrench: retain DWP2's raw body
        // force/torque rather than applying the FinsROV axis-scale calibration
        // or replacing its yaw torque with the measured yaw-damping curve.
        activeWaterObject.hydrodynamicAxisScalingEnabled = false;
        activeWaterObject.hydrodynamicForceAxisScale = Vector3.one;
        activeWaterObject.hydrodynamicTorqueAxisScale = Vector3.one;
        activeWaterObject.hydrodynamicYawDampingMode = HydrodynamicYawDampingMode.AxisScaleOnly;

        // WaterObject performs a one-time migration for old prefabs in Awake.
        // Mark this generated fixture as explicitly configured so that migration
        // cannot silently re-enable axis scaling or MatchRealYawDampingCurve.
        SerializedObject serializedWaterObject = new SerializedObject(activeWaterObject);
        SerializedProperty axisConfigVersion = serializedWaterObject.FindProperty("hydrodynamicAxisScalingConfigVersion");
        if (axisConfigVersion != null)
        {
            axisConfigVersion.intValue = 1;
            serializedWaterObject.ApplyModifiedPropertiesWithoutUndo();
        }

        // This probe is useful while calibrating a single vehicle, but each
        // replicated training area would otherwise open a CSV and emit a
        // diagnostic line every second.  It is strictly observational and
        // must not be active in a 1024-area headless training fixture.
        foreach (FinsROVDwp2YawRuntimeProbe probe in
                 vehicle.GetComponentsInChildren<FinsROVDwp2YawRuntimeProbe>(true))
        {
            probe.enabled = false;
            PrefabUtility.RecordPrefabInstancePropertyModifications(probe);
        }

        Dwp2AxisHydrodynamicScaleRandomizer randomizer =
            vehicle.GetComponent<Dwp2AxisHydrodynamicScaleRandomizer>();
        if (randomizer == null)
        {
            randomizer = vehicle.AddComponent<Dwp2AxisHydrodynamicScaleRandomizer>();
        }
        randomizer.targetRoot = vehicle.transform;
        randomizer.includeInactiveWaterObjects = true;
        randomizer.autoRefreshTargets = true;
        randomizer.enableAxisScalingOnTargets = false;
        randomizer.axisConvention = HydrodynamicAxisConvention.FinsRovXForwardYUpZLeft;
        randomizer.yawDampingMode = HydrodynamicYawDampingMode.AxisScaleOnly;
        randomizer.baselineForceAxisScale = Vector3.one;
        randomizer.baselineTorqueAxisScale = Vector3.one;
        randomizer.randomizeSurge = false;
        randomizer.randomizeSway = false;
        randomizer.randomizeHeave = false;
        randomizer.randomizeRoll = false;
        randomizer.randomizePitch = false;
        randomizer.randomizeYaw = false;
        randomizer.yawScaleRange = new FloatRange(1f, 1f);
        randomizer.randomizeOnStart = false;
        randomizer.randomizePerEpisode = false;
        randomizer.applyBaselineOnAwake = false;
        randomizer.logRandomizedValue = false;
        randomizer.enabled = false;

        Dwp2HydrodynamicForceCoefficientRandomizer coefficientRandomizer =
            vehicle.GetComponent<Dwp2HydrodynamicForceCoefficientRandomizer>();
        if (coefficientRandomizer == null)
        {
            coefficientRandomizer = vehicle.AddComponent<Dwp2HydrodynamicForceCoefficientRandomizer>();
        }
        coefficientRandomizer.targetRoot = vehicle.transform;
        // Only the active main-body mesh participates in this fixture.  The
        // disabled thruster WaterObjects must not silently become a DR target.
        coefficientRandomizer.includeInactiveWaterObjects = false;
        coefficientRandomizer.autoRefreshTargets = true;
        coefficientRandomizer.minHydrodynamicForceCoefficient = Dwp2HydrodynamicCoefficientMin;
        coefficientRandomizer.maxHydrodynamicForceCoefficient = Dwp2HydrodynamicCoefficientMax;
        coefficientRandomizer.randomizeOnStart = false;
        coefficientRandomizer.randomizePerEpisode = enableDr;
        coefficientRandomizer.logRandomizedValue = false;
        coefficientRandomizer.enabled = enableDr;

        DomainRandomizationCoordinator coordinator = RequireOrAddCoordinator(vehicle);
        if (enableDr)
        {
            ConfigureCoordinator(coordinator, GetOrCreateDwp2YawOnlyProfile(), enabled: true, allowCommandLineOverrides: true);
        }
        else
        {
            ConfigureCoordinator(coordinator, null, enabled: false, allowCommandLineOverrides: false);
        }
    }

    /// <summary>
    /// Rebuilds a compact, closed DWP2 physics mesh from an audited simulation
    /// mesh. Backend-variant builders share this rather than invoking the DWP2
    /// CAD-mesh decimator during a batch build.
    /// </summary>
    public static void RegenerateDwp2PhysicsMesh(WaterObject waterObject)
    {
        Mesh physicsMesh = waterObject.SimulationMesh;
        if (physicsMesh == null || physicsMesh.triangles.Length == 0)
        {
            throw new InvalidOperationException(
                "DWP2 fixture requires a pre-generated closed simulation mesh before simplification.");
        }

        // The package MeshDecimate implementation throws on this particular
        // watertight FinsROV mesh.  Use a small build-time edge-collapse path
        // instead.  It accepts a collapse only when the result is still a
        // closed 2-manifold, so it cannot serialize an open or self-duplicated
        // mesh merely to meet the speed budget.
        Mesh reducedMesh = DecimateClosedPhysicsMesh(physicsMesh, Dwp2PhysicsMeshTargetTriangleCount);
        physicsMesh.Clear();
        physicsMesh.vertices = reducedMesh.vertices;
        physicsMesh.triangles = reducedMesh.triangles;
        physicsMesh.RecalculateBounds();
        physicsMesh.RecalculateNormals();
        physicsMesh.name = "DWP_SIM_MESH";

        waterObject.simplifyMesh = true;
        waterObject.targetTriangleCount = Dwp2PhysicsMeshTargetTriangleCount;
        waterObject.serializedSimulationMesh ??= new SerializedMesh();
        waterObject.serializedSimulationMesh.Serialize(physicsMesh);
        waterObject.triangleCount = physicsMesh.triangles.Length / 3;
        EditorUtility.SetDirty(waterObject);
    }

    static Mesh DecimateClosedPhysicsMesh(Mesh source, int targetTriangleCount)
    {
        if (targetTriangleCount <= 0 || targetTriangleCount % 2 != 0)
        {
            throw new ArgumentOutOfRangeException(nameof(targetTriangleCount));
        }

        var vertices = new List<Vector3>(source.vertices);
        int[] sourceTriangles = source.triangles;
        var triangles = new List<int[]>(sourceTriangles.Length / 3);
        for (int i = 0; i < sourceTriangles.Length; i += 3)
        {
            triangles.Add(new[] { sourceTriangles[i], sourceTriangles[i + 1], sourceTriangles[i + 2] });
        }

        if (!IsClosedTwoManifold(vertices.Count, triangles))
        {
            throw new InvalidOperationException("The source DWP2 physics mesh is not a closed 2-manifold.");
        }

        while (triangles.Count > targetTriangleCount)
        {
            if (!TryCollapseShortestValidEdge(ref vertices, ref triangles))
            {
                throw new InvalidOperationException(
                    $"Unable to safely reduce the closed DWP2 physics mesh from {triangles.Count} to {targetTriangleCount} triangles.");
            }
        }

        if (triangles.Count != targetTriangleCount || !IsClosedTwoManifold(vertices.Count, triangles))
        {
            throw new InvalidOperationException("DWP2 reduced physics mesh failed its closed-manifold validation.");
        }

        var outputTriangles = new int[triangles.Count * 3];
        for (int i = 0; i < triangles.Count; i++)
        {
            outputTriangles[3 * i] = triangles[i][0];
            outputTriangles[3 * i + 1] = triangles[i][1];
            outputTriangles[3 * i + 2] = triangles[i][2];
        }

        return NWH.DWP2.WaterObjects.MeshUtility.GenerateMesh(vertices.ToArray(), outputTriangles);
    }

    static bool TryCollapseShortestValidEdge(ref List<Vector3> vertices, ref List<int[]> triangles)
    {
        var candidateEdges = new List<MeshEdge>();
        var seenEdges = new HashSet<long>();
        for (int i = 0; i < triangles.Count; i++)
        {
            int[] triangle = triangles[i];
            AddCandidateEdge(triangle[0], triangle[1], vertices, seenEdges, candidateEdges);
            AddCandidateEdge(triangle[1], triangle[2], vertices, seenEdges, candidateEdges);
            AddCandidateEdge(triangle[2], triangle[0], vertices, seenEdges, candidateEdges);
        }
        candidateEdges.Sort((left, right) => left.sqrLength.CompareTo(right.sqrLength));

        for (int i = 0; i < candidateEdges.Count; i++)
        {
            MeshEdge edge = candidateEdges[i];
            if (TryCollapseEdge(vertices, triangles, edge.a, edge.b,
                    out List<Vector3> collapsedVertices, out List<int[]> collapsedTriangles))
            {
                vertices = collapsedVertices;
                triangles = collapsedTriangles;
                return true;
            }
        }
        return false;
    }

    static void AddCandidateEdge(
        int first, int second, List<Vector3> vertices, HashSet<long> seenEdges, List<MeshEdge> candidateEdges)
    {
        int a = Mathf.Min(first, second);
        int b = Mathf.Max(first, second);
        long key = EdgeKey(a, b);
        if (!seenEdges.Add(key))
        {
            return;
        }
        candidateEdges.Add(new MeshEdge { a = a, b = b, sqrLength = (vertices[a] - vertices[b]).sqrMagnitude });
    }

    static bool TryCollapseEdge(
        List<Vector3> vertices, List<int[]> triangles, int keep, int remove,
        out List<Vector3> collapsedVertices, out List<int[]> collapsedTriangles)
    {
        collapsedVertices = null;
        collapsedTriangles = null;
        var replacedTriangles = new List<int[]>(triangles.Count - 2);
        var seenFaces = new HashSet<long>();
        int removedFaceCount = 0;

        for (int i = 0; i < triangles.Count; i++)
        {
            int[] source = triangles[i];
            int a = source[0] == remove ? keep : source[0];
            int b = source[1] == remove ? keep : source[1];
            int c = source[2] == remove ? keep : source[2];
            if (a == b || b == c || c == a)
            {
                removedFaceCount++;
                continue;
            }
            if (!seenFaces.Add(FaceKey(a, b, c)))
            {
                return false;
            }
            replacedTriangles.Add(new[] { a, b, c });
        }

        // On a closed triangular manifold, a valid interior-edge collapse
        // removes exactly its two incident faces.
        if (removedFaceCount != 2 || replacedTriangles.Count != triangles.Count - 2)
        {
            return false;
        }

        var remap = new int[vertices.Count];
        Array.Fill(remap, -1);
        var compactVertices = new List<Vector3>(vertices.Count - 1);
        for (int i = 0; i < vertices.Count; i++)
        {
            if (i == remove)
            {
                continue;
            }
            remap[i] = compactVertices.Count;
            compactVertices.Add(i == keep ? 0.5f * (vertices[keep] + vertices[remove]) : vertices[i]);
        }

        for (int i = 0; i < replacedTriangles.Count; i++)
        {
            int[] face = replacedTriangles[i];
            face[0] = remap[face[0]];
            face[1] = remap[face[1]];
            face[2] = remap[face[2]];
        }

        if (!IsClosedTwoManifold(compactVertices.Count, replacedTriangles))
        {
            return false;
        }

        collapsedVertices = compactVertices;
        collapsedTriangles = replacedTriangles;
        return true;
    }

    static bool IsClosedTwoManifold(int vertexCount, List<int[]> triangles)
    {
        var edgeUses = new Dictionary<long, int>();
        var faces = new HashSet<long>();
        for (int i = 0; i < triangles.Count; i++)
        {
            int[] face = triangles[i];
            if (face.Length != 3 || face[0] < 0 || face[1] < 0 || face[2] < 0 ||
                face[0] >= vertexCount || face[1] >= vertexCount || face[2] >= vertexCount ||
                face[0] == face[1] || face[1] == face[2] || face[2] == face[0] ||
                !faces.Add(FaceKey(face[0], face[1], face[2])))
            {
                return false;
            }
            CountEdge(face[0], face[1], edgeUses);
            CountEdge(face[1], face[2], edgeUses);
            CountEdge(face[2], face[0], edgeUses);
        }

        foreach (KeyValuePair<long, int> edge in edgeUses)
        {
            if (edge.Value != 2)
            {
                return false;
            }
        }
        // FinsROV's physics mesh is genus zero; retain that topology too.
        return vertexCount - edgeUses.Count + triangles.Count == 2;
    }

    static void CountEdge(int first, int second, Dictionary<long, int> edgeUses)
    {
        long key = EdgeKey(first, second);
        edgeUses.TryGetValue(key, out int count);
        edgeUses[key] = count + 1;
    }

    static long EdgeKey(int first, int second)
    {
        int a = Mathf.Min(first, second);
        int b = Mathf.Max(first, second);
        return ((long)a << 32) | (uint)b;
    }

    static long FaceKey(int first, int second, int third)
    {
        int a = first;
        int b = second;
        int c = third;
        if (a > b) { (a, b) = (b, a); }
        if (b > c) { (b, c) = (c, b); }
        if (a > b) { (a, b) = (b, a); }
        return ((long)a << 42) | ((long)b << 21) | (uint)c;
    }

    static void ConfigureDwp2FlatWaterProvider()
    {
        PhysicalWaveWaterDataProvider flatProvider = null;
        PhysicalWaveWaterDataProvider[] providers =
            UnityEngine.Object.FindObjectsByType<PhysicalWaveWaterDataProvider>(
                FindObjectsInactive.Include,
                FindObjectsSortMode.None);

        for (int i = 0; i < providers.Length; i++)
        {
            PhysicalWaveWaterDataProvider provider = providers[i];
            if (provider == null)
            {
                continue;
            }

            if (provider.gameObject.name == "FlatWaterProvider")
            {
                flatProvider = provider;
                continue;
            }

            if (provider.gameObject.name == "PhysicalWaveWaterProvider")
            {
                provider.enabled = false;
                provider.gameObject.SetActive(false);
            }
        }

        if (flatProvider == null)
        {
            throw new InvalidOperationException("DWP2 fixtures require the FlatWaterProvider scene object.");
        }

        flatProvider.gameObject.SetActive(true);
        flatProvider.enabled = true;
        flatProvider.mode = PhysicalWaveWaterDataProvider.WavePhysicsMode.FlatFallback;
        flatProvider.fallbackWaterHeight = 0f;
        flatProvider.stillWaterHeight = 0f;
        flatProvider.includeWaveNormals = false;
        flatProvider.includeWaveFlow = false;
        flatProvider.includeVerticalOrbitalFlow = false;
        flatProvider.flowScale = 0f;
        flatProvider.maxFlowSpeed = 0f;
        flatProvider.steadyCurrent = Vector3.zero;
        flatProvider.waves.Clear();
        flatProvider.randomizeWaterCurrentFromProfile = false;
        flatProvider.randomizeWavesFromProfile = false;
        flatProvider.forceAnalyticModeWhenRandomizingWaves = false;
        flatProvider.logEpisodeRandomization = false;
        EditorUtility.SetDirty(flatProvider);
    }

    static void ConfigureCoordinator(
        DomainRandomizationCoordinator coordinator,
        DomainRandomizationProfile profile,
        bool enabled,
        bool allowCommandLineOverrides)
    {
        coordinator.profile = profile;
        coordinator.enabled = enabled;
        coordinator.mode = enabled ? DomainRandomizationMode.Train : DomainRandomizationMode.Disabled;
        coordinator.baseSeed = 12345;
        coordinator.incrementSeedPerEpisode = true;
        coordinator.randomizeOnStart = false;
        coordinator.autoFindTargets = true;
        coordinator.targetBehaviours.Clear();
        coordinator.applyCommandLineOverrides = allowCommandLineOverrides;
        coordinator.seedCommandLineArg = "-fins-dr-seed";
        coordinator.modeCommandLineArg = "-fins-dr-mode";
    }

    static DomainRandomizationProfile GetOrCreateDwp2YawOnlyProfile()
    {
        DomainRandomizationProfile profile = AssetDatabase.LoadAssetAtPath<DomainRandomizationProfile>(Dwp2YawDrProfilePath);
        if (profile != null)
        {
            return profile;
        }

        profile = ScriptableObject.CreateInstance<DomainRandomizationProfile>();
        profile.name = "DomainRandomizationProfile_DWP2YawOnly";
        profile.randomizeBody = false;
        profile.randomizeHydrodynamics = false;
        profile.randomizeHydrodynamicAxisScales = false;
        profile.randomizeThrusters = false;
        profile.randomizeWater = false;
        profile.randomizeWaves = false;
        profile.randomizeInitialState = false;
        AssetDatabase.CreateAsset(profile, Dwp2YawDrProfilePath);
        AssetDatabase.SaveAssets();
        return profile;
    }

    static void ConfigureParallelAgent(HoldForPosition agent, Transform target)
    {
        agent.targetTransform = target;
        agent.selfTransform = null;
        agent.ConfigureEpisodeMaxStep(ThirtySecondEpisodeMaxStep);
        agent.ConfigureThrusterCommandMode(ThrusterCommandMode.NormalizedMaxForceRequest);
        BehaviorParameters behavior = agent.GetComponent<BehaviorParameters>();
        if (behavior == null || behavior.BrainParameters.ActionSpec.NumContinuousActions != HoldForPosition.ContinuousActionSize)
        {
            throw new InvalidOperationException("HoldForPosition variant must expose the canonical eight-thruster action ABI.");
        }
        behavior.BehaviorName = "HoldForPosition";
    }

    static void ConfigureThrusterDynamics(GameObject vehicle, bool firstOrder, Thruster[] sourceDynamics)
    {
        Thruster[] thrusters = ResolveOrderedThrusters(vehicle);
        for (int i = 0; i < thrusters.Length; i++)
        {
            if (sourceDynamics != null)
            {
                thrusters[i].CommandDelaySec = sourceDynamics[i].CommandDelaySec;
                thrusters[i].ForceTimeConstantSec = sourceDynamics[i].ForceTimeConstantSec;
                thrusters[i].MaxForceSlewRateNPerSec = sourceDynamics[i].MaxForceSlewRateNPerSec;
            }
            thrusters[i].DynamicsModel = firstOrder
                ? Thruster.ActuatorDynamicsModel.FirstOrderDelaySlew
                : Thruster.ActuatorDynamicsModel.None;
            // Thrusters belong to the Fossen prefab instance. Record scalar
            // dynamics edits explicitly so the instantaneous-actuator fixture
            // survives a scene close/reopen cycle.
            EditorUtility.SetDirty(thrusters[i]);
            PrefabUtility.RecordPrefabInstancePropertyModifications(thrusters[i]);
        }
    }

    static void DisableNonTrainingControlComponents(GameObject vehicle)
    {
        foreach (MonoBehaviour behaviour in vehicle.GetComponentsInChildren<MonoBehaviour>(true))
        {
            if (behaviour == null)
            {
                continue;
            }
            string typeName = behaviour.GetType().Name;
            if (typeName == "FinsROVManualThrusterController" || typeName == "VehicleRosBridge")
            {
                behaviour.enabled = false;
            }
        }
    }

    static void ValidateScene(Scene scene, VariantKind kind)
    {
        TrainingAreaReplicator replicator = FindReplicator();
        if (replicator.baseArea == null || !replicator.buildOnly)
        {
            throw new InvalidOperationException($"Variant {scene.path} has no build-only TrainingAreaReplicator base area.");
        }

        int expectedDwp2AreaCount = Dwp2AreaCount(kind);
        if (expectedDwp2AreaCount > 0 && replicator.numAreas != expectedDwp2AreaCount)
        {
            throw new InvalidOperationException(
                $"DWP2 mesh variant must serialize {expectedDwp2AreaCount} training area(s), found {replicator.numAreas}.");
        }

        HoldForPosition agent = RequireSingleAgent(replicator.baseArea, scene.path);
        if (agent.MaxStep != ThirtySecondEpisodeMaxStep ||
            agent.RecommendedMaxStep != ThirtySecondEpisodeMaxStep ||
            agent.CurrentThrusterCommandMode != ThrusterCommandMode.NormalizedMaxForceRequest)
        {
            throw new InvalidOperationException("Variant changed the shared 30 s normalized-max-force task contract.");
        }

        float expectedActionLimit = kind == VariantKind.FossenThrusterCap25
            ? ThrusterActionCap25
            : 1f;
        if (!Mathf.Approximately(agent.NormalizedThrusterActionLimit, expectedActionLimit))
        {
            throw new InvalidOperationException(
                $"Variant action limit is {agent.NormalizedThrusterActionLimit}, expected {expectedActionLimit}.");
        }

        BehaviorParameters behavior = agent.GetComponent<BehaviorParameters>();
        if (behavior == null || behavior.BehaviorName != "HoldForPosition" ||
            behavior.BrainParameters.ActionSpec.NumContinuousActions != HoldForPosition.ContinuousActionSize)
        {
            throw new InvalidOperationException("Variant does not expose the expected HoldForPosition eight-action behavior.");
        }

        bool expectDr = kind == VariantKind.Dwp2Dr ||
                        kind == VariantKind.EquivalentBoxDr;
        DomainRandomizationCoordinator coordinator = agent.GetComponent<DomainRandomizationCoordinator>();
        if (coordinator == null || coordinator.enabled != expectDr ||
            coordinator.mode != (expectDr ? DomainRandomizationMode.Train : DomainRandomizationMode.Disabled))
        {
            throw new InvalidOperationException("Variant domain-randomization state does not match its scene name.");
        }

        bool expectFirstOrder = kind != VariantKind.FossenInstantThrusters;
        foreach (Thruster thruster in ResolveOrderedThrusters(agent.gameObject))
        {
            Thruster.ActuatorDynamicsModel expected = expectFirstOrder
                ? Thruster.ActuatorDynamicsModel.FirstOrderDelaySlew
                : Thruster.ActuatorDynamicsModel.None;
            if (thruster.DynamicsModel != expected)
            {
                throw new InvalidOperationException($"Thruster {thruster.name} has {thruster.DynamicsModel}, expected {expected}.");
            }
        }

        if (Dwp2AreaCount(kind) > 0)
        {
            HydrodynamicsController controller = agent.GetComponentInChildren<HydrodynamicsController>(true);
            if (controller != null && controller.enabled)
            {
                throw new InvalidOperationException("DWP2 mesh variants must not retain an active Fossen controller.");
            }
            WaterObject[] waterObjects = agent.GetComponentsInChildren<WaterObject>(true);
            int activeCount = 0;
            foreach (WaterObject waterObject in waterObjects)
            {
                if (waterObject != null && waterObject.isActiveAndEnabled)
                {
                    activeCount++;
                    if (!Mathf.Approximately(waterObject.fluidDensity, Dwp2FluidDensity) ||
                        !Mathf.Approximately(waterObject.buoyantForceCoefficient, Dwp2BuoyancyCoefficient) ||
                        !Mathf.Approximately(waterObject.hydrodynamicForceCoefficient, Dwp2HydrodynamicCoefficient))
                    {
                        throw new InvalidOperationException("DWP2 mesh coefficients differ from the FinsROV DWP2 default fixture.");
                    }
                    if (waterObject.hydrodynamicAxisScalingEnabled ||
                        waterObject.hydrodynamicYawDampingMode != HydrodynamicYawDampingMode.AxisScaleOnly)
                    {
                        throw new InvalidOperationException(
                            "DWP2 training variants must use unscaled raw mesh hydrodynamics without a yaw-damping override.");
                    }
                    if (!waterObject.simplifyMesh ||
                        waterObject.targetTriangleCount != Dwp2PhysicsMeshTargetTriangleCount)
                    {
                        throw new InvalidOperationException(
                            "DWP2 training variants must regenerate the main physics mesh at the configured triangle budget.");
                    }
                }
            }
            if (activeCount != 1)
            {
                throw new InvalidOperationException($"DWP2 mesh variant must have exactly one active WaterObject, found {activeCount}.");
            }
            foreach (FinsROVDwp2YawRuntimeProbe probe in
                     agent.GetComponentsInChildren<FinsROVDwp2YawRuntimeProbe>(true))
            {
                if (probe.enabled)
                {
                    throw new InvalidOperationException(
                        "DWP2 training variants must disable the runtime yaw diagnostic probe.");
                }
            }

            ValidateDwp2WaterProviderAndRandomization(agent, kind == VariantKind.Dwp2Dr);
        }

        if (kind == VariantKind.EquivalentBoxNoDr || kind == VariantKind.EquivalentBoxDr)
        {
            HydrodynamicsController controller = agent.GetComponentInChildren<HydrodynamicsController>(true);
            if (controller == null || controller.mode != HydrodynamicsMode.LearningToSwimEquivalentBox)
            {
                throw new InvalidOperationException("Equivalent-box variant does not use LearningToSwimEquivalentBox.");
            }
        }

        if (kind == VariantKind.FossenInstantThrusters ||
            kind == VariantKind.FossenThrusterCap25 ||
            kind == VariantKind.FossenWrenchScaled)
        {
            HydrodynamicsController controller = agent.GetComponentInChildren<HydrodynamicsController>(true);
            if (controller == null || controller.mode != HydrodynamicsMode.Fossen6Dof)
            {
                throw new InvalidOperationException("Fossen actuator/wrench variant changed the Fossen backend.");
            }
        }
    }

    static void ValidateDwp2WaterProviderAndRandomization(HoldForPosition agent, bool expectDr)
    {
        PhysicalWaveWaterDataProvider flatProvider = null;
        PhysicalWaveWaterDataProvider[] providers =
            UnityEngine.Object.FindObjectsByType<PhysicalWaveWaterDataProvider>(
                FindObjectsInactive.Include,
                FindObjectsSortMode.None);
        for (int i = 0; i < providers.Length; i++)
        {
            PhysicalWaveWaterDataProvider provider = providers[i];
            if (provider == null)
            {
                continue;
            }

            if (provider.gameObject.name == "FlatWaterProvider")
            {
                flatProvider = provider;
            }
            else if (provider.gameObject.name == "PhysicalWaveWaterProvider" &&
                     (provider.gameObject.activeInHierarchy || provider.enabled))
            {
                throw new InvalidOperationException(
                    "DWP2 fixtures must disable the additional physical-wave provider.");
            }
        }

        if (flatProvider == null || !flatProvider.gameObject.activeInHierarchy || !flatProvider.enabled ||
            flatProvider.mode != PhysicalWaveWaterDataProvider.WavePhysicsMode.FlatFallback ||
            flatProvider.includeWaveNormals || flatProvider.includeWaveFlow ||
            flatProvider.waves.Count != 0)
        {
            throw new InvalidOperationException("DWP2 fixtures must use the configured flat-water provider.");
        }

        Dwp2HydrodynamicForceCoefficientRandomizer coefficientRandomizer =
            agent.GetComponent<Dwp2HydrodynamicForceCoefficientRandomizer>();
        if (coefficientRandomizer == null || coefficientRandomizer.enabled != expectDr ||
            coefficientRandomizer.randomizePerEpisode != expectDr ||
            coefficientRandomizer.includeInactiveWaterObjects ||
            !Mathf.Approximately(coefficientRandomizer.minHydrodynamicForceCoefficient,
                Dwp2HydrodynamicCoefficientMin) ||
            !Mathf.Approximately(coefficientRandomizer.maxHydrodynamicForceCoefficient,
                Dwp2HydrodynamicCoefficientMax))
        {
            throw new InvalidOperationException(
                "DWP2 DR must randomize only the active physics-mesh hydrodynamic coefficient.");
        }
    }

    static TrainingAreaReplicator FindReplicator()
    {
        TrainingAreaReplicator replicator = UnityEngine.Object.FindFirstObjectByType<TrainingAreaReplicator>();
        if (replicator == null)
        {
            throw new InvalidOperationException("No TrainingAreaReplicator was found.");
        }
        return replicator;
    }

    static HoldForPosition RequireSingleAgent(GameObject root, string context)
    {
        HoldForPosition[] agents = root.GetComponentsInChildren<HoldForPosition>(true);
        if (agents.Length != 1)
        {
            throw new InvalidOperationException($"{context} must contain exactly one HoldForPosition agent, found {agents.Length}.");
        }
        return agents[0];
    }

    static Transform RequireTarget(Transform area)
    {
        Transform target = area.Find("Target");
        if (target == null)
        {
            throw new InvalidOperationException("Training area has no Target child.");
        }
        return target;
    }

    static DomainRandomizationCoordinator RequireOrAddCoordinator(GameObject vehicle)
    {
        DomainRandomizationCoordinator coordinator = vehicle.GetComponent<DomainRandomizationCoordinator>();
        return coordinator != null ? coordinator : vehicle.AddComponent<DomainRandomizationCoordinator>();
    }

    static Thruster[] ResolveOrderedThrusters(GameObject vehicle)
    {
        var ordered = new Thruster[HoldForPosition.ContinuousActionSize];
        if (!FinsROVAgentRuntime.TryResolveOrderedThrusters(
                vehicle.GetComponent<HoldForPosition>(),
                vehicle.GetComponent<ThrusterController>(),
                autoResolveFromChildren: true,
                ordered,
                out string status))
        {
            throw new InvalidOperationException($"Unable to resolve canonical thruster order: {status}");
        }
        return ordered;
    }

    static VariantKind VariantForPath(string scenePath)
    {
        if (scenePath == Dwp2NoDrScenePath) return VariantKind.Dwp2NoDr;
        if (scenePath == Dwp2DrScenePath) return VariantKind.Dwp2Dr;
        if (scenePath == EquivalentBoxNoDrScenePath) return VariantKind.EquivalentBoxNoDr;
        if (scenePath == EquivalentBoxDrScenePath) return VariantKind.EquivalentBoxDr;
        if (scenePath == FossenInstantThrustersScenePath) return VariantKind.FossenInstantThrusters;
        if (scenePath == FossenThrusterCap25ScenePath) return VariantKind.FossenThrusterCap25;
        if (scenePath == FossenWrenchScaledScenePath) return VariantKind.FossenWrenchScaled;
        throw new InvalidOperationException($"Not a supported HoldForPosition backend-variant scene: {scenePath}.");
    }

    static int Dwp2AreaCount(VariantKind kind)
    {
        switch (kind)
        {
            case VariantKind.Dwp2NoDr:
            case VariantKind.Dwp2Dr:
                return 1;
            default:
                return 0;
        }
    }

}
