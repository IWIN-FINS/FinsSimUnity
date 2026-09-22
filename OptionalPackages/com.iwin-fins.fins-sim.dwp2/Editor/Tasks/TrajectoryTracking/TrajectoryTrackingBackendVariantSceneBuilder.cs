using System;
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
/// Creates T2 backend/actuator variants. Parameterized variants are derived
/// from the hand-authored Fossen scene. DWP2 variants retain the T2 task
/// contract but use one vehicle per Unity Player, deliberately removing
/// TrainingAreaReplicator; Python supplies process-level parallelism.
/// </summary>
public static class TrajectoryTrackingBackendVariantSceneBuilder
{
    public const string SourceScenePath =
        "Assets/FinsSimUnity/Tasks/TrajectoryTracking/Scenes/TrajectoryTracking_Fossen_Parallel_30s.unity";
    public const string FossenNoDrScenePath =
        "Assets/FinsSimUnity/Tasks/TrajectoryTracking/Scenes/TrajectoryTracking_Fossen_Parallel_30s_NoDR.unity";
    public const string FossenNoDrInstantThrustersScenePath =
        "Assets/FinsSimUnity/Tasks/TrajectoryTracking/Scenes/TrajectoryTracking_Fossen_Parallel_30s_NoDR_InstantThrusters.unity";
    public const string EquivalentBoxNoDrScenePath =
        "Assets/FinsSimUnity/Tasks/TrajectoryTracking/Scenes/TrajectoryTracking_LearningToSwimEquivalentBox_Parallel_30s_NoDR.unity";
    public const string EquivalentBoxDrScenePath =
        "Assets/FinsSimUnity/Tasks/TrajectoryTracking/Scenes/TrajectoryTracking_LearningToSwimEquivalentBox_Parallel_30s_DR.unity";
    public const string Dwp2NoDrScenePath =
        "Assets/FinsSimUnity/Tasks/TrajectoryTracking/Scenes/TrajectoryTracking_DWP2Mesh_Parallel_30s_NoDR.unity";
    public const string Dwp2DrScenePath =
        "Assets/FinsSimUnity/Tasks/TrajectoryTracking/Scenes/TrajectoryTracking_DWP2Mesh_Parallel_30s_DR.unity";

    const string EquivalentBoxProfilePath =
        "Assets/Models/FinsROV/Hydrodynamics/FinsROV_HydrodynamicsProfile_LearningToSwimEquivalentBox.asset";
    const string EquivalentBoxDrProfilePath =
        "Assets/Models/FinsROV/Hydrodynamics/DomainRandomizationProfile_LearningToSwimEquivalentBox.asset";
    // The operational T1 DWP2 fixture is the source of truth for the DWP2
    // vehicle hierarchy and its audited physics mesh.  T2 copies that vehicle
    // into the trajectory scene, then replaces only the task components.  A
    // plain prefab instantiation is insufficient: scene-level overrides in
    // this fixture are part of the intended DWP2 training configuration.
    const string Dwp2FixtureScenePath =
        "Assets/FinsSimUnity/Tasks/PoseControl/Scenes/ControlForPosition_DWP2Mesh_Mode1_NormalizedMaxForce_DR.unity";
    const string Dwp2OperationalNoDrScenePath =
        ControlForPositionDwp2Mode1SceneBuilder.NoDrScenePath;
    const string Dwp2OperationalDrScenePath =
        ControlForPositionDwp2Mode1SceneBuilder.DrScenePath;
    const string Dwp2CoefficientOnlyDrProfilePath =
        "Assets/Models/FinsROV/Hydrodynamics/DomainRandomizationProfile_DWP2CoefficientOnly.asset";

    const float Dwp2FluidDensity = 997f;
    const float Dwp2BuoyancyCoefficient = 1.01f;
    const float Dwp2HydrodynamicCoefficient = 0.20f;
    const float Dwp2CoefficientMin = 0.10f;
    const float Dwp2CoefficientMax = 0.52f;
    enum Variant
    {
        FossenNoDr,
        FossenNoDrInstantThrusters,
        EquivalentBoxNoDr,
        EquivalentBoxDr,
        Dwp2NoDr,
        Dwp2Dr,
    }

    [MenuItem("FinsSim/RL/Prepare T2 Backend Variants/All")]
    public static void PrepareAll()
    {
        Prepare(FossenNoDrScenePath, Variant.FossenNoDr);
        Prepare(FossenNoDrInstantThrustersScenePath, Variant.FossenNoDrInstantThrusters);
        Prepare(EquivalentBoxNoDrScenePath, Variant.EquivalentBoxNoDr);
        Prepare(EquivalentBoxDrScenePath, Variant.EquivalentBoxDr);
        Prepare(Dwp2NoDrScenePath, Variant.Dwp2NoDr);
        Prepare(Dwp2DrScenePath, Variant.Dwp2Dr);
    }

    public static void PrepareFossenNoDr() => Prepare(FossenNoDrScenePath, Variant.FossenNoDr);
    public static void PrepareFossenNoDrInstantThrusters() => Prepare(FossenNoDrInstantThrustersScenePath, Variant.FossenNoDrInstantThrusters);
    public static void PrepareEquivalentBoxNoDr() => Prepare(EquivalentBoxNoDrScenePath, Variant.EquivalentBoxNoDr);
    public static void PrepareEquivalentBoxDr() => Prepare(EquivalentBoxDrScenePath, Variant.EquivalentBoxDr);
    public static void PrepareDwp2NoDr() => Prepare(Dwp2NoDrScenePath, Variant.Dwp2NoDr);
    public static void PrepareDwp2Dr() => Prepare(Dwp2DrScenePath, Variant.Dwp2Dr);

    public static void EnsurePrepared(string scenePath)
    {
        Variant variant = VariantForScene(scenePath);
        // These are generated fixtures, not hand-authored task scenes.  Build
        // them from the common Fossen source every time, as the T1 builder
        // does, so an earlier serialized DWP2 prefab/mesh/component state
        // cannot survive a code or calibration update unnoticed.
        Prepare(scenePath, variant);
    }

    static Variant VariantForScene(string scenePath)
    {
        if (scenePath == FossenNoDrScenePath) return Variant.FossenNoDr;
        if (scenePath == FossenNoDrInstantThrustersScenePath) return Variant.FossenNoDrInstantThrusters;
        if (scenePath == EquivalentBoxNoDrScenePath) return Variant.EquivalentBoxNoDr;
        if (scenePath == EquivalentBoxDrScenePath) return Variant.EquivalentBoxDr;
        if (scenePath == Dwp2NoDrScenePath) return Variant.Dwp2NoDr;
        if (scenePath == Dwp2DrScenePath) return Variant.Dwp2Dr;
        throw new ArgumentException($"Unknown T2 variant scene: {scenePath}", nameof(scenePath));
    }

    static void Prepare(string outputPath, Variant variant)
    {
        bool dwp2 = variant == Variant.Dwp2NoDr || variant == Variant.Dwp2Dr;
        string sourcePath = dwp2
            ? (variant == Variant.Dwp2Dr ? Dwp2OperationalDrScenePath : Dwp2OperationalNoDrScenePath)
            : SourceScenePath;
        if (dwp2)
        {
            ControlForPositionDwp2Mode1SceneBuilder.EnsurePrepared(sourcePath);
        }

        Scene source = EditorSceneManager.OpenScene(sourcePath, OpenSceneMode.Single);
        if (!EditorSceneManager.SaveScene(source, outputPath, true))
        {
            throw new InvalidOperationException($"Unable to create {outputPath} from {sourcePath}.");
        }

        Scene scene = EditorSceneManager.OpenScene(outputPath, OpenSceneMode.Single);
        switch (variant)
        {
            case Variant.FossenNoDr:
                ConfigureFossenNoDr();
                break;
            case Variant.FossenNoDrInstantThrusters:
                ConfigureFossenNoDrInstantThrusters();
                break;
            case Variant.EquivalentBoxNoDr:
            case Variant.EquivalentBoxDr:
                ConfigureEquivalentBox(variant == Variant.EquivalentBoxDr);
                break;
            case Variant.Dwp2NoDr:
            case Variant.Dwp2Dr:
                ConfigureDwp2FromOperationalFixture(variant == Variant.Dwp2Dr);
                break;
            default:
                throw new ArgumentOutOfRangeException(nameof(variant), variant, null);
        }

        Validate(scene, variant);
        EditorSceneManager.MarkSceneDirty(scene);
        if (!EditorSceneManager.SaveScene(scene))
        {
            throw new InvalidOperationException($"Unable to save {outputPath}.");
        }
        Debug.Log($"Prepared T2 backend variant {outputPath} ({variant}).");
    }

    static void ConfigureFossenNoDr()
    {
        TrajectoryTrackingAgent agent = RequireAgent();
        RequireFossen(agent);
        ConfigureCoordinator(agent.gameObject, null, false, false, false);
    }

    static void ConfigureFossenNoDrInstantThrusters()
    {
        TrajectoryTrackingAgent agent = RequireAgent();
        RequireFossen(agent);
        ConfigureThrusterDynamics(agent.gameObject, false);
        ConfigureCoordinator(agent.gameObject, null, false, false, false);
    }

    static void ConfigureEquivalentBox(bool enableDr)
    {
        TrajectoryTrackingAgent agent = RequireAgent();
        HydrodynamicsProfile profile = AssetDatabase.LoadAssetAtPath<HydrodynamicsProfile>(EquivalentBoxProfilePath);
        HydrodynamicsController controller = agent.GetComponent<HydrodynamicsController>();
        if (controller == null || profile == null)
        {
            throw new InvalidOperationException("The T2 equivalent-box variant requires a Fossen controller and profile asset.");
        }

        controller.enabled = true;
        controller.mode = HydrodynamicsMode.LearningToSwimEquivalentBox;
        controller.profile = profile;
        controller.ResetRuntimeProfileFromSource();
        Record(controller);

        FinsROVHydrodynamicsSetup setup = agent.GetComponent<FinsROVHydrodynamicsSetup>();
        if (setup != null)
        {
            setup.hydrodynamicsProfile = profile;
            Record(setup);
        }

        DomainRandomizationProfile drProfile = enableDr
            ? AssetDatabase.LoadAssetAtPath<DomainRandomizationProfile>(EquivalentBoxDrProfilePath)
            : null;
        if (enableDr && drProfile == null)
        {
            throw new InvalidOperationException("Missing Learning-to-Swim domain-randomization profile.");
        }
        ConfigureCoordinator(agent.gameObject, drProfile, enableDr, enableDr, enableDr);
        ConfigureThrusterDynamics(agent.gameObject, true);
    }

    static void ConfigureDwp2FromOperationalFixture(bool enableDr)
    {
        HoldForPosition oldAgent = UnityEngine.Object.FindFirstObjectByType<HoldForPosition>();
        if (oldAgent == null)
        {
            throw new InvalidOperationException(
                "The operational DWP2 source fixture has no HoldForPosition agent.");
        }

        GameObject vehicle = oldAgent.gameObject;
        Component oldRequester = vehicle.GetComponent("DecisionRequester");
        if (oldRequester != null)
        {
            // DecisionRequester requires the outgoing T1 agent. Remove it
            // before installing the T2 agent and copying its requester.
            UnityEngine.Object.DestroyImmediate(oldRequester, true);
        }
        UnityEngine.Object.DestroyImmediate(oldAgent, true);

        Scene taskSourceScene = EditorSceneManager.OpenScene(SourceScenePath, OpenSceneMode.Additive);
        try
        {
            TrajectoryTrackingAgent sourceAgent = FindSingleTrajectoryAgent(taskSourceScene);
            TrajectoryTrackingAgent agent = vehicle.AddComponent<TrajectoryTrackingAgent>();
            Transform trajectoryReference = new GameObject("TrajectoryReference").transform;
            trajectoryReference.SetPositionAndRotation(Vector3.zero, Quaternion.identity);
            CopyT2AgentConfiguration(sourceAgent.gameObject, sourceAgent, vehicle, agent, trajectoryReference);
            ConfigureThrusterDynamics(vehicle, true);
            DisableNonTrainingControlComponents(vehicle);

            DomainRandomizationCoordinator coordinator = vehicle.GetComponent<DomainRandomizationCoordinator>();
            if (coordinator == null || coordinator.enabled != enableDr)
            {
                throw new InvalidOperationException(
                    "Operational DWP2 source does not match the requested DR state.");
            }
        }
        finally
        {
            EditorSceneManager.CloseScene(taskSourceScene, true);
        }
    }

    static TrajectoryTrackingAgent FindSingleTrajectoryAgent(Scene scene)
    {
        TrajectoryTrackingAgent found = null;
        foreach (GameObject root in scene.GetRootGameObjects())
        {
            foreach (TrajectoryTrackingAgent candidate in root.GetComponentsInChildren<TrajectoryTrackingAgent>(true))
            {
                if (found != null)
                {
                    throw new InvalidOperationException($"Scene {scene.path} has more than one T2 agent.");
                }
                found = candidate;
            }
        }
        return found ?? throw new InvalidOperationException($"Scene {scene.path} has no T2 agent.");
    }

    static void ReplaceFossenVehicleWithDwp2(bool enableDr)
    {
        TrainingAreaReplicator replicator = RequireReplicator();
        GameObject area = replicator.baseArea;
        TrajectoryTrackingAgent oldAgent = RequireAgent();
        Transform trajectoryReference = area.transform.Find("TrajectoryReference");
        if (trajectoryReference == null)
        {
            throw new InvalidOperationException("T2 base area has no TrajectoryReference.");
        }

        GameObject oldRoot = oldAgent.gameObject;
        GameObject vehicle = InstantiateDwp2FixtureVehicle(oldRoot.scene, area.transform);
        vehicle.name = "FinsROV_DWP2Mesh";
        vehicle.transform.SetLocalPositionAndRotation(oldRoot.transform.localPosition, oldRoot.transform.localRotation);
        vehicle.transform.localScale = oldRoot.transform.localScale;

        // The fixture carries the T1 Agent.  Remove it rather than merely
        // disabling it, so this single-player T2 scene has exactly one
        // ML-Agents endpoint before the T2 task component is installed.
        foreach (Agent candidate in vehicle.GetComponents<Agent>())
        {
            UnityEngine.Object.DestroyImmediate(candidate, true);
        }
        TrajectoryTrackingAgent agent = vehicle.AddComponent<TrajectoryTrackingAgent>();
        CopyT2AgentConfiguration(oldRoot, oldAgent, vehicle, agent, trajectoryReference);
        CopyRigidbodyAndThrusterConfiguration(oldRoot, vehicle);
        DisableNonTrainingControlComponents(vehicle);
        ConfigureDwp2Hydrodynamics(vehicle, enableDr);
        agent.ConfigureProcessSeedOffset(true);

        TrajectoryTrackingReferenceVisualizer visualizer =
            area.GetComponentInChildren<TrajectoryTrackingReferenceVisualizer>(true);
        visualizer?.Bind(agent);

        UnityEngine.Object.DestroyImmediate(oldRoot);
        ConfigureDwp2FlatWaterProvider();
        RemoveAreaReplication(replicator, area);
    }

    static GameObject InstantiateDwp2FixtureVehicle(Scene destinationScene, Transform destinationParent)
    {
        Scene fixtureScene = EditorSceneManager.OpenScene(Dwp2FixtureScenePath, OpenSceneMode.Additive);
        try
        {
            HoldForPosition fixtureAgent = null;
            foreach (HoldForPosition candidate in UnityEngine.Object.FindObjectsByType<HoldForPosition>(
                         FindObjectsInactive.Include, FindObjectsSortMode.None))
            {
                if (candidate != null && candidate.gameObject.scene == fixtureScene)
                {
                    fixtureAgent = candidate;
                    break;
                }
            }
            if (fixtureAgent == null)
            {
                throw new InvalidOperationException(
                    $"DWP2 fixture {Dwp2FixtureScenePath} has no HoldForPosition vehicle.");
            }

            GameObject vehicle = UnityEngine.Object.Instantiate(fixtureAgent.gameObject);
            SceneManager.MoveGameObjectToScene(vehicle, destinationScene);
            vehicle.transform.SetParent(destinationParent, false);
            return vehicle;
        }
        finally
        {
            EditorSceneManager.CloseScene(fixtureScene, true);
        }
    }

    static void RemoveAreaReplication(TrainingAreaReplicator replicator, GameObject area)
    {
        // Keep the base area as the sole world, including its trajectory
        // reference and water provider. Only clone-specific runtime state is
        // removed; each outer Unity Player receives its own launcher seed.
        TrajectoryTrackingParallelAreaRuntime runtime =
            area.GetComponent<TrajectoryTrackingParallelAreaRuntime>();
        if (runtime != null)
        {
            UnityEngine.Object.DestroyImmediate(runtime);
        }
        UnityEngine.Object.DestroyImmediate(replicator.gameObject);
    }

    static void CopyT2AgentConfiguration(
        GameObject sourceRoot,
        TrajectoryTrackingAgent sourceAgent,
        GameObject destinationRoot,
        TrajectoryTrackingAgent destinationAgent,
        Transform trajectoryReference)
    {
        EditorUtility.CopySerialized(sourceAgent, destinationAgent);
        destinationAgent.selfTransform = null;
        destinationAgent.trajectoryReference = trajectoryReference;
        destinationAgent.enabled = true;

        BehaviorParameters sourceBehavior = sourceRoot.GetComponent<BehaviorParameters>();
        if (sourceBehavior == null)
        {
            throw new InvalidOperationException("T2 source vehicle has no BehaviorParameters.");
        }
        BehaviorParameters destinationBehavior = destinationRoot.GetComponent<BehaviorParameters>();
        if (destinationBehavior == null)
        {
            destinationBehavior = destinationRoot.AddComponent<BehaviorParameters>();
        }
        EditorUtility.CopySerialized(sourceBehavior, destinationBehavior);
        destinationBehavior.enabled = true;

        Component sourceRequester = sourceRoot.GetComponent("DecisionRequester");
        Component destinationRequester = destinationRoot.GetComponent("DecisionRequester");
        if (sourceRequester != null && destinationRequester == null)
        {
            destinationRequester = destinationRoot.AddComponent(sourceRequester.GetType());
        }
        if (sourceRequester != null && destinationRequester != null && sourceRequester.GetType() == destinationRequester.GetType())
        {
            EditorUtility.CopySerialized(sourceRequester, destinationRequester);
            ((Behaviour)destinationRequester).enabled = true;
        }

        foreach (Agent candidate in destinationRoot.GetComponents<Agent>())
        {
            if (candidate != destinationAgent)
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
            throw new InvalidOperationException("T2 source and DWP2 vehicle must both have a root Rigidbody.");
        }
        EditorUtility.CopySerialized(sourceBody, destinationBody);

        Thruster[] sourceThrusters = ResolveOrderedThrusters(sourceRoot);
        Thruster[] destinationThrusters = ResolveOrderedThrusters(destinationRoot);
        if (sourceThrusters.Length != destinationThrusters.Length)
        {
            throw new InvalidOperationException("T2 source and DWP2 vehicle must expose the same eight-thruster layout.");
        }
        for (int i = 0; i < sourceThrusters.Length; i++)
        {
            EditorUtility.CopySerialized(sourceThrusters[i], destinationThrusters[i]);
            destinationThrusters[i].TargetRigidbody = destinationBody;
            destinationThrusters[i].DynamicsModel = Thruster.ActuatorDynamicsModel.FirstOrderDelaySlew;
            Record(destinationThrusters[i]);
        }
        FinsROVAgentRuntime.EnsureThrusterControllerOrder(
            destinationRoot.GetComponent<ThrusterController>(), destinationThrusters);
    }

    static void ConfigureDwp2Hydrodynamics(GameObject vehicle, bool enableDr)
    {
        foreach (HydrodynamicsController controller in vehicle.GetComponentsInChildren<HydrodynamicsController>(true))
        {
            controller.enabled = false;
            Record(controller);
        }

        WaterObject mainBody = null;
        foreach (WaterObject waterObject in vehicle.GetComponentsInChildren<WaterObject>(true))
        {
            if (waterObject == null)
            {
                continue;
            }
            bool isMainBody = waterObject.name.Equals("mainbody", StringComparison.OrdinalIgnoreCase);
            waterObject.enabled = isMainBody;
            if (isMainBody)
            {
                if (mainBody != null)
                {
                    throw new InvalidOperationException("DWP2 vehicle has more than one mainbody WaterObject.");
                }
                mainBody = waterObject;
            }
            Record(waterObject);
        }
        if (mainBody == null)
        {
            throw new InvalidOperationException("DWP2 vehicle has no WaterObject named mainbody.");
        }

        mainBody.fluidDensity = Dwp2FluidDensity;
        mainBody.buoyantForceCoefficient = Dwp2BuoyancyCoefficient;
        mainBody.hydrodynamicForceCoefficient = Dwp2HydrodynamicCoefficient;
        mainBody.hydrodynamicAxisScalingEnabled = false;
        mainBody.hydrodynamicForceAxisScale = Vector3.one;
        mainBody.hydrodynamicTorqueAxisScale = Vector3.one;
        mainBody.hydrodynamicYawDampingMode = HydrodynamicYawDampingMode.AxisScaleOnly;
        Record(mainBody);

        Dwp2AxisHydrodynamicScaleRandomizer axisRandomizer = vehicle.GetComponent<Dwp2AxisHydrodynamicScaleRandomizer>();
        if (axisRandomizer != null)
        {
            axisRandomizer.enabled = false;
            Record(axisRandomizer);
        }

        Dwp2HydrodynamicForceCoefficientRandomizer coefficientRandomizer =
            vehicle.GetComponent<Dwp2HydrodynamicForceCoefficientRandomizer>();
        if (coefficientRandomizer == null)
        {
            coefficientRandomizer = vehicle.AddComponent<Dwp2HydrodynamicForceCoefficientRandomizer>();
        }
        coefficientRandomizer.targetRoot = vehicle.transform;
        coefficientRandomizer.includeInactiveWaterObjects = false;
        coefficientRandomizer.autoRefreshTargets = false;
        coefficientRandomizer.waterObjects.Clear();
        coefficientRandomizer.waterObjects.Add(mainBody);
        coefficientRandomizer.minHydrodynamicForceCoefficient = Dwp2CoefficientMin;
        coefficientRandomizer.maxHydrodynamicForceCoefficient = Dwp2CoefficientMax;
        coefficientRandomizer.randomizeOnStart = false;
        coefficientRandomizer.randomizePerEpisode = enableDr;
        coefficientRandomizer.logRandomizedValue = false;
        coefficientRandomizer.enabled = enableDr;
        coefficientRandomizer.ResolveTargets();
        Record(coefficientRandomizer);

        ConfigureCoordinator(
            vehicle,
            enableDr ? GetDwp2CoefficientOnlyDrProfile() : null,
            enableDr,
            enableDr,
            false,
            enableDr ? coefficientRandomizer : null);
        ConfigureThrusterDynamics(vehicle, true);
    }

    static void ConfigureDwp2FlatWaterProvider()
    {
        PhysicalWaveWaterDataProvider flatProvider = null;
        foreach (PhysicalWaveWaterDataProvider provider in
                 UnityEngine.Object.FindObjectsByType<PhysicalWaveWaterDataProvider>(
                     FindObjectsInactive.Include, FindObjectsSortMode.None))
        {
            if (provider.gameObject.name == "FlatWaterProvider")
            {
                flatProvider = provider;
            }
            else if (provider.gameObject.name == "PhysicalWaveWaterProvider")
            {
                provider.enabled = false;
                provider.gameObject.SetActive(false);
            }
        }
        if (flatProvider == null)
        {
            throw new InvalidOperationException("T2 DWP2 variant has no FlatWaterProvider.");
        }
        flatProvider.gameObject.SetActive(true);
        flatProvider.enabled = true;
        flatProvider.mode = PhysicalWaveWaterDataProvider.WavePhysicsMode.FlatFallback;
        flatProvider.fallbackWaterHeight = 0f;
        flatProvider.stillWaterHeight = 0f;
        flatProvider.steadyCurrent = Vector3.zero;
        flatProvider.includeWaveNormals = false;
        flatProvider.includeWaveFlow = false;
        flatProvider.includeVerticalOrbitalFlow = false;
        flatProvider.randomizeWaterCurrentFromProfile = false;
        flatProvider.randomizeWavesFromProfile = false;
        flatProvider.forceAnalyticModeWhenRandomizingWaves = false;
        flatProvider.waves.Clear();
        Record(flatProvider);
    }

    static void ConfigureCoordinator(
        GameObject vehicle,
        DomainRandomizationProfile profile,
        bool enabled,
        bool allowCommandLineOverrides,
        bool autoFindTargets,
        MonoBehaviour explicitTarget = null)
    {
        DomainRandomizationCoordinator coordinator = vehicle.GetComponent<DomainRandomizationCoordinator>();
        if (coordinator == null)
        {
            coordinator = vehicle.AddComponent<DomainRandomizationCoordinator>();
        }
        coordinator.profile = profile;
        coordinator.enabled = enabled;
        coordinator.mode = enabled ? DomainRandomizationMode.Train : DomainRandomizationMode.Disabled;
        coordinator.baseSeed = 12345;
        coordinator.incrementSeedPerEpisode = true;
        coordinator.randomizeOnStart = false;
        coordinator.autoFindTargets = autoFindTargets;
        coordinator.targetBehaviours.Clear();
        if (explicitTarget != null)
        {
            coordinator.targetBehaviours.Add(explicitTarget);
        }
        coordinator.applyCommandLineOverrides = allowCommandLineOverrides;
        coordinator.seedCommandLineArg = "-fins-dr-seed";
        coordinator.modeCommandLineArg = "-fins-dr-mode";
        Record(coordinator);
    }

    static DomainRandomizationProfile GetDwp2CoefficientOnlyDrProfile()
    {
        DomainRandomizationProfile profile = AssetDatabase.LoadAssetAtPath<DomainRandomizationProfile>(Dwp2CoefficientOnlyDrProfilePath);
        if (profile == null)
        {
            profile = ScriptableObject.CreateInstance<DomainRandomizationProfile>();
            profile.name = "DomainRandomizationProfile_DWP2CoefficientOnly";
            AssetDatabase.CreateAsset(profile, Dwp2CoefficientOnlyDrProfilePath);
        }
        profile.randomizeBody = false;
        profile.randomizeHydrodynamics = false;
        profile.randomizeHydrodynamicAxisScales = false;
        profile.randomizeThrusters = false;
        profile.randomizeWater = false;
        profile.randomizeWaves = false;
        profile.randomizeInitialState = false;
        Record(profile);
        AssetDatabase.SaveAssets();
        return profile;
    }

    static void ConfigureThrusterDynamics(GameObject vehicle, bool firstOrder)
    {
        foreach (Thruster thruster in ResolveOrderedThrusters(vehicle))
        {
            thruster.DynamicsModel = firstOrder
                ? Thruster.ActuatorDynamicsModel.FirstOrderDelaySlew
                : Thruster.ActuatorDynamicsModel.None;
            Record(thruster);
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
            // Runtime probes are intended for one-off, interactive hydrodynamic
            // diagnosis.  They open a CSV under a developer-machine path and
            // enable detailed per-thruster statistics, neither of which belongs
            // in a headless RL Player (especially a multi-binary DWP2 run).
            if (typeName == "FinsROVManualThrusterController" ||
                typeName == "VehicleRosBridge" ||
                typeName == "FinsROVDwp2YawRuntimeProbe" ||
                typeName == "FinsROVDwp2WaterObjectMotionBenchmark" ||
                typeName == "Dwp2BodyYawTorqueScaler")
            {
                behaviour.enabled = false;
                Record(behaviour);
            }
        }
    }

    static void RequireFossen(TrajectoryTrackingAgent agent)
    {
        HydrodynamicsController controller = agent.GetComponent<HydrodynamicsController>();
        if (controller == null || !controller.enabled || controller.mode != HydrodynamicsMode.Fossen6Dof)
        {
            throw new InvalidOperationException("This T2 variant must retain the Fossen6Dof backend.");
        }
    }

    static TrainingAreaReplicator RequireReplicator()
    {
        TrainingAreaReplicator replicator = UnityEngine.Object.FindFirstObjectByType<TrainingAreaReplicator>();
        if (replicator == null || replicator.baseArea == null || !replicator.buildOnly)
        {
            throw new InvalidOperationException("T2 variant requires the build-only TrainingAreaReplicator base area.");
        }
        return replicator;
    }

    static TrajectoryTrackingAgent RequireAgent()
    {
        TrainingAreaReplicator replicator = UnityEngine.Object.FindFirstObjectByType<TrainingAreaReplicator>();
        if (replicator != null && replicator.baseArea != null)
        {
            TrajectoryTrackingAgent[] areaAgents = replicator.baseArea.GetComponentsInChildren<TrajectoryTrackingAgent>(true);
            if (areaAgents.Length == 1)
            {
                return areaAgents[0];
            }
        }

        TrajectoryTrackingAgent[] sceneAgents =
            UnityEngine.Object.FindObjectsByType<TrajectoryTrackingAgent>(FindObjectsInactive.Include, FindObjectsSortMode.None);
        if (sceneAgents.Length != 1)
        {
            throw new InvalidOperationException($"Expected exactly one T2 agent, found {sceneAgents.Length}.");
        }
        return sceneAgents[0];
    }

    static Thruster[] ResolveOrderedThrusters(GameObject vehicle)
    {
        ThrusterController controller = vehicle.GetComponent<ThrusterController>();
        if (controller == null)
        {
            throw new InvalidOperationException($"{vehicle.name} has no ThrusterController.");
        }
        Thruster[] discovered = vehicle.GetComponentsInChildren<Thruster>(true);
        if (discovered.Length != TrajectoryTrackingAgent.ThrusterActionSize)
        {
            throw new InvalidOperationException($"{vehicle.name} must expose eight thrusters, found {discovered.Length}.");
        }
        var ordered = new Thruster[TrajectoryTrackingAgent.ThrusterActionSize];
        if (!FinsROVAgentRuntime.TryResolveOrderedThrusters(vehicle.transform, controller, true, ordered, out string status))
        {
            throw new InvalidOperationException($"Unable to resolve T2 canonical thrusters: {status}");
        }
        FinsROVAgentRuntime.EnsureThrusterControllerOrder(controller, ordered);
        return ordered;
    }

    static void Validate(Scene scene, Variant variant)
    {
        if (scene.path != PathForVariant(variant))
        {
            throw new InvalidOperationException($"T2 variant saved to unexpected scene path: {scene.path}.");
        }
        TrajectoryTrackingAgent agent = RequireAgent();
        BehaviorParameters behavior = agent.GetComponent<BehaviorParameters>();
        if (agent.MaxStep != 1500 || behavior == null || behavior.BehaviorName != "TrajectoryTracking" ||
            behavior.BrainParameters.VectorObservationSize != TrajectoryTrackingAgent.VectorObservationSize ||
            behavior.BrainParameters.ActionSpec.NumContinuousActions != TrajectoryTrackingAgent.ThrusterActionSize)
        {
            throw new InvalidOperationException("T2 variant changed the trajectory task contract.");
        }

        bool expectDr = variant == Variant.EquivalentBoxDr || variant == Variant.Dwp2Dr;
        DomainRandomizationCoordinator coordinator = agent.GetComponent<DomainRandomizationCoordinator>();
        if (coordinator == null || coordinator.enabled != expectDr ||
            coordinator.mode != (expectDr ? DomainRandomizationMode.Train : DomainRandomizationMode.Disabled))
        {
            throw new InvalidOperationException("T2 variant domain-randomization state is inconsistent with its name.");
        }

        bool expectFirstOrder = variant != Variant.FossenNoDrInstantThrusters;
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

        if (variant == Variant.Dwp2NoDr || variant == Variant.Dwp2Dr)
        {
            if (UnityEngine.Object.FindFirstObjectByType<TrainingAreaReplicator>() != null ||
                UnityEngine.Object.FindFirstObjectByType<TrajectoryTrackingParallelAreaRuntime>() != null)
            {
                throw new InvalidOperationException(
                    "T2 DWP2 variants must use the single-vehicle multi-binary topology without area replication.");
            }
            if (agent.GetComponent<HydrodynamicsController>() is { enabled: true })
            {
                throw new InvalidOperationException("T2 DWP2 variant retains an active parameterized hydrodynamics controller.");
            }
            int activeWaterObjects = 0;
            foreach (WaterObject waterObject in agent.GetComponentsInChildren<WaterObject>(true))
            {
                if (waterObject != null && waterObject.isActiveAndEnabled)
                {
                    activeWaterObjects++;
                    if (!Mathf.Approximately(waterObject.fluidDensity, Dwp2FluidDensity) ||
                        !Mathf.Approximately(waterObject.buoyantForceCoefficient, Dwp2BuoyancyCoefficient) ||
                        !Mathf.Approximately(waterObject.hydrodynamicForceCoefficient, Dwp2HydrodynamicCoefficient) ||
                        waterObject.hydrodynamicAxisScalingEnabled)
                    {
                        throw new InvalidOperationException("T2 DWP2 mesh settings differ from the ControlForPosition DWP2 fixture.");
                    }
                }
            }
            if (activeWaterObjects != 1)
            {
                throw new InvalidOperationException($"T2 DWP2 fixture must have one active main-body WaterObject, found {activeWaterObjects}.");
            }
            WaterObject mainBody = null;
            foreach (WaterObject waterObject in agent.GetComponentsInChildren<WaterObject>(true))
            {
                if (waterObject != null &&
                    waterObject.name.Equals("mainbody", StringComparison.OrdinalIgnoreCase))
                {
                    mainBody = waterObject;
                    break;
                }
            }
            if (mainBody == null || mainBody.SimulationMesh == null ||
                mainBody.SimulationMesh.triangles.Length == 0)
            {
                throw new InvalidOperationException(
                    "T2 DWP2 fixture has no usable main-body physics mesh.");
            }
            foreach (FinsROVDwp2YawRuntimeProbe probe in
                     agent.GetComponentsInChildren<FinsROVDwp2YawRuntimeProbe>(true))
            {
                if (probe.enabled)
                {
                    throw new InvalidOperationException(
                        "T2 DWP2 fixture must disable FinsROVDwp2YawRuntimeProbe during training.");
                }
            }
        }
        else
        {
            HydrodynamicsController controller = agent.GetComponent<HydrodynamicsController>();
            HydrodynamicsMode expected = (variant == Variant.EquivalentBoxNoDr || variant == Variant.EquivalentBoxDr)
                ? HydrodynamicsMode.LearningToSwimEquivalentBox
                : HydrodynamicsMode.Fossen6Dof;
            if (controller == null || !controller.enabled || controller.mode != expected)
            {
                throw new InvalidOperationException($"T2 variant hydrodynamics is not {expected}.");
            }
        }
    }

    static string PathForVariant(Variant variant)
    {
        return variant switch
        {
            Variant.FossenNoDr => FossenNoDrScenePath,
            Variant.FossenNoDrInstantThrusters => FossenNoDrInstantThrustersScenePath,
            Variant.EquivalentBoxNoDr => EquivalentBoxNoDrScenePath,
            Variant.EquivalentBoxDr => EquivalentBoxDrScenePath,
            Variant.Dwp2NoDr => Dwp2NoDrScenePath,
            Variant.Dwp2Dr => Dwp2DrScenePath,
            _ => throw new ArgumentOutOfRangeException(nameof(variant), variant, null),
        };
    }

    static void Record(UnityEngine.Object target)
    {
        if (target == null)
        {
            return;
        }
        EditorUtility.SetDirty(target);
        if (target is Component component)
        {
            PrefabUtility.RecordPrefabInstancePropertyModifications(component);
        }
    }
}
