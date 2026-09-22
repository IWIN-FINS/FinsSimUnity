using System;
using FinsSim.Hydrodynamics;
using NWH.DWP2.WaterData;
using NWH.DWP2.WaterObjects;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Policies;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

/// <summary>
/// Builds lightweight, single-ROV DWP2 fixtures from the operational
/// ControlForPosition small-force scene. They retain its DWP2 vehicle and
/// one-Player-per-environment structure while using the T1 mode-1 task.
/// </summary>
public static class ControlForPositionDwp2Mode1SceneBuilder
{
    public const string SourceScenePath =
        "Assets/FinsSimUnity/Tasks/PoseControl/Scenes/ControlForPosition_SmallForceCoefficient.unity";
    public const string NoDrScenePath =
        "Assets/FinsSimUnity/Tasks/PoseControl/Scenes/ControlForPosition_DWP2Mesh_Mode1_NormalizedMaxForce_NoDR.unity";
    public const string DrScenePath =
        "Assets/FinsSimUnity/Tasks/PoseControl/Scenes/ControlForPosition_DWP2Mesh_Mode1_NormalizedMaxForce_DR.unity";

    const float CoefficientMin = 0.10f;
    const float CoefficientMax = 0.52f;
    const float Dwp2FluidDensity = 997f;
    // Keep the two fixtures independently configurable while using the same
    // calibrated buoyancy setting for the current DR/NoDR comparison.
    const float Dwp2NoDrBuoyancyCoefficient = 1.01f;
    const float Dwp2DrBuoyancyCoefficient = 1.01f;
    const float Dwp2HydrodynamicCoefficient = 0.20f;

    [MenuItem("FinsSim/RL/Prepare ControlForPosition DWP2 Mode 1/No DR")]
    public static void PrepareNoDr() => Prepare(NoDrScenePath, enableDr: false);

    [MenuItem("FinsSim/RL/Prepare ControlForPosition DWP2 Mode 1/DR")]
    public static void PrepareDr() => Prepare(DrScenePath, enableDr: true);

    public static void EnsurePrepared(string scenePath)
    {
        if (scenePath == NoDrScenePath)
        {
            Prepare(scenePath, enableDr: false);
        }
        else if (scenePath == DrScenePath)
        {
            Prepare(scenePath, enableDr: true);
        }
        else
        {
            throw new ArgumentOutOfRangeException(nameof(scenePath), scenePath, "Unknown DWP2 mode-1 fixture.");
        }
    }

    static void Prepare(string outputPath, bool enableDr)
    {
        Scene source = EditorSceneManager.OpenScene(SourceScenePath, OpenSceneMode.Single);
        if (!EditorSceneManager.SaveScene(source, outputPath, true))
        {
            throw new InvalidOperationException($"Unable to copy {SourceScenePath} to {outputPath}.");
        }

        Scene scene = EditorSceneManager.OpenScene(outputPath, OpenSceneMode.Single);
        ControlForPosition_IncrementalReward oldAgent =
            UnityEngine.Object.FindFirstObjectByType<ControlForPosition_IncrementalReward>();
        if (oldAgent == null)
        {
            throw new InvalidOperationException("The small-force DWP2 source scene has no incremental-reward agent.");
        }

        GameObject vehicle = oldAgent.gameObject;
        Transform selfTransform = oldAgent.selfTransform;
        Transform targetTransform = oldAgent.targetTransform;
        int maxStep = oldAgent.MaxStep;
        DecisionRequester oldRequester = vehicle.GetComponent<DecisionRequester>();
        if (oldRequester != null)
        {
            // The source requester declares the incremental-reward agent as
            // its required component. Remove it before replacing that agent.
            UnityEngine.Object.DestroyImmediate(oldRequester, true);
        }
        UnityEngine.Object.DestroyImmediate(oldAgent, true);

        HoldForPosition agent = vehicle.GetComponent<HoldForPosition>();
        if (agent == null)
        {
            agent = vehicle.AddComponent<HoldForPosition>();
        }
        agent.enabled = true;
        agent.selfTransform = selfTransform;
        agent.targetTransform = targetTransform;
        agent.ConfigureEpisodeMaxStep(maxStep);
        agent.ConfigureDenseRewardMode(HoldForPosition.DenseRewardMode.LearnToSwimGoalHold);
        agent.ConfigureThrusterCommandMode(ThrusterCommandMode.NormalizedMaxForceRequest);
        agent.ConfigureNormalizedThrusterActionLimit(1f);

        ConfigureBehavior(vehicle);
        ConfigureDecisionRate(vehicle);
        DisableNonTrainingComponents(vehicle);
        ConfigureFlatWaterOnly();
        WaterObject mainBodyWaterObject = ConfigurePureDwp2MainBody(
            vehicle,
            enableDr ? Dwp2DrBuoyancyCoefficient : Dwp2NoDrBuoyancyCoefficient);
        ConfigureDwp2Randomization(vehicle, mainBodyWaterObject, enableDr);
        Validate(agent, enableDr);

        EditorSceneManager.MarkSceneDirty(scene);
        if (!EditorSceneManager.SaveScene(scene))
        {
            throw new InvalidOperationException($"Unable to save {outputPath}.");
        }

        EditorSceneManager.OpenScene(outputPath, OpenSceneMode.Single);
        Validate(UnityEngine.Object.FindFirstObjectByType<HoldForPosition>(), enableDr);
    }

    static void ConfigureBehavior(GameObject vehicle)
    {
        BehaviorParameters behavior = vehicle.GetComponent<BehaviorParameters>();
        if (behavior == null)
        {
            behavior = vehicle.AddComponent<BehaviorParameters>();
        }
        behavior.BehaviorName = "HoldForPosition";
        behavior.BrainParameters.ActionSpec = ActionSpec.MakeContinuous(HoldForPosition.ContinuousActionSize);
        behavior.enabled = true;
        EditorUtility.SetDirty(behavior);
    }

    static void ConfigureDecisionRate(GameObject vehicle)
    {
        DecisionRequester requester = vehicle.GetComponent<DecisionRequester>();
        if (requester == null)
        {
            requester = vehicle.AddComponent<DecisionRequester>();
        }
        requester.DecisionPeriod = 5; // fixed step 0.02 s -> 10 Hz
        requester.TakeActionsBetweenDecisions = true;
        requester.enabled = true;
        EditorUtility.SetDirty(requester);
    }

    static void DisableNonTrainingComponents(GameObject vehicle)
    {
        foreach (MonoBehaviour behaviour in vehicle.GetComponentsInChildren<MonoBehaviour>(true))
        {
            if (behaviour == null)
            {
                continue;
            }
            string typeName = behaviour.GetType().Name;
            if (typeName == "FinsROVManualThrusterController" ||
                typeName == "VehicleRosBridge" ||
                typeName == "FinsROVDwp2YawRuntimeProbe" ||
                typeName == "FinsROVDwp2WaterObjectMotionBenchmark" ||
                typeName == "Dwp2BodyYawTorqueScaler")
            {
                behaviour.enabled = false;
            }
        }

        foreach (Agent candidate in vehicle.GetComponents<Agent>())
        {
            if (!(candidate is HoldForPosition))
            {
                candidate.enabled = false;
            }
        }
    }

    static void ConfigureFlatWaterOnly()
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
            throw new InvalidOperationException("The DWP2 source fixture has no FlatWaterProvider.");
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
        EditorUtility.SetDirty(flatProvider);
    }

    static WaterObject ConfigurePureDwp2MainBody(GameObject vehicle, float buoyancyCoefficient)
    {
        HydrodynamicsController parameterizedHydrodynamics =
            vehicle.GetComponentInChildren<HydrodynamicsController>(true);
        if (parameterizedHydrodynamics != null)
        {
            parameterizedHydrodynamics.enabled = false;
        }

        WaterObject[] allWaterObjects = vehicle.GetComponentsInChildren<WaterObject>(true);
        WaterObject mainBodyWaterObject = FindMainBodyWaterObject(vehicle);

        // DWP2's per-thruster WaterObjects are useful for visual/component
        // diagnostics, but are not part of this pure hull-mesh training
        // fixture.  Their propulsion force remains supplied by Thruster.
        foreach (WaterObject candidate in allWaterObjects)
        {
            if (candidate != null)
            {
                candidate.enabled = candidate == mainBodyWaterObject;
                EditorUtility.SetDirty(candidate);
            }
        }

        mainBodyWaterObject.fluidDensity = Dwp2FluidDensity;
        mainBodyWaterObject.buoyantForceCoefficient = buoyancyCoefficient;
        mainBodyWaterObject.hydrodynamicForceCoefficient = Dwp2HydrodynamicCoefficient;
        mainBodyWaterObject.hydrodynamicAxisScalingEnabled = false;
        mainBodyWaterObject.hydrodynamicForceAxisScale = Vector3.one;
        mainBodyWaterObject.hydrodynamicTorqueAxisScale = Vector3.one;
        mainBodyWaterObject.hydrodynamicYawDampingMode = HydrodynamicYawDampingMode.AxisScaleOnly;
        EditorUtility.SetDirty(mainBodyWaterObject);
        return mainBodyWaterObject;
    }

    static WaterObject FindMainBodyWaterObject(GameObject vehicle)
    {
        WaterObject mainBodyWaterObject = null;
        foreach (WaterObject candidate in vehicle.GetComponentsInChildren<WaterObject>(true))
        {
            if (candidate == null ||
                !candidate.name.Equals("mainbody", StringComparison.OrdinalIgnoreCase))
            {
                continue;
            }
            if (mainBodyWaterObject != null)
            {
                throw new InvalidOperationException("Expected one DWP2 mainbody WaterObject, found more than one.");
            }
            mainBodyWaterObject = candidate;
        }
        if (mainBodyWaterObject == null)
        {
            throw new InvalidOperationException("The source DWP2 vehicle has no WaterObject named mainbody.");
        }
        return mainBodyWaterObject;
    }

    static void ConfigureDwp2Randomization(
        GameObject vehicle,
        WaterObject mainBodyWaterObject,
        bool enableDr)
    {
        int activeWaterObjectCount = 0;
        foreach (WaterObject waterObject in vehicle.GetComponentsInChildren<WaterObject>(true))
        {
            if (waterObject != null && waterObject.isActiveAndEnabled)
            {
                activeWaterObjectCount++;
            }
        }
        if (activeWaterObjectCount != 1)
        {
            throw new InvalidOperationException(
                $"The small-force DWP2 fixture must retain exactly one active WaterObject, found {activeWaterObjectCount}.");
        }

        Dwp2AxisHydrodynamicScaleRandomizer axisRandomizer =
            vehicle.GetComponent<Dwp2AxisHydrodynamicScaleRandomizer>();
        if (axisRandomizer != null)
        {
            // The DR counterpart differs only in mesh-force coefficient, not
            // in post-mesh axis scaling or a real-yaw diagnostic correction.
            axisRandomizer.enabled = false;
        }

        Dwp2HydrodynamicForceCoefficientRandomizer coefficientRandomizer =
            vehicle.GetComponent<Dwp2HydrodynamicForceCoefficientRandomizer>();
        if (coefficientRandomizer == null)
        {
            coefficientRandomizer = vehicle.AddComponent<Dwp2HydrodynamicForceCoefficientRandomizer>();
        }
        coefficientRandomizer.targetRoot = vehicle.transform;
        coefficientRandomizer.includeInactiveWaterObjects = false;
        // Persist an explicit single-object target list.  Auto-discovery would
        // rediscover disabled diagnostic/thruster water objects in editors
        // that inspect inactive hierarchy nodes.
        coefficientRandomizer.autoRefreshTargets = false;
        coefficientRandomizer.waterObjects.Clear();
        coefficientRandomizer.waterObjects.Add(mainBodyWaterObject);
        coefficientRandomizer.minHydrodynamicForceCoefficient = CoefficientMin;
        coefficientRandomizer.maxHydrodynamicForceCoefficient = CoefficientMax;
        coefficientRandomizer.randomizeOnStart = false;
        coefficientRandomizer.randomizePerEpisode = enableDr;
        coefficientRandomizer.logRandomizedValue = false;
        coefficientRandomizer.enabled = enableDr;
        coefficientRandomizer.ResolveTargets();
        if (coefficientRandomizer.waterObjects.Count != 1 ||
            coefficientRandomizer.waterObjects[0] != mainBodyWaterObject)
        {
            throw new InvalidOperationException(
                $"Coefficient-only DWP2 DR must target the active main-body mesh exactly once, found {coefficientRandomizer.waterObjects.Count} targets.");
        }

        DomainRandomizationCoordinator coordinator = vehicle.GetComponent<DomainRandomizationCoordinator>();
        if (coordinator == null)
        {
            coordinator = vehicle.AddComponent<DomainRandomizationCoordinator>();
        }
        coordinator.profile = GetOrCreateCoefficientOnlyProfile();
        coordinator.enabled = enableDr;
        coordinator.mode = enableDr ? DomainRandomizationMode.Train : DomainRandomizationMode.Disabled;
        coordinator.baseSeed = 12345;
        coordinator.incrementSeedPerEpisode = true;
        coordinator.randomizeOnStart = false;
        coordinator.autoFindTargets = false;
        coordinator.targetBehaviours.Clear();
        if (enableDr)
        {
            coordinator.targetBehaviours.Add(coefficientRandomizer);
        }
        coordinator.applyCommandLineOverrides = enableDr;
        coordinator.seedCommandLineArg = "-fins-dr-seed";
        coordinator.modeCommandLineArg = "-fins-dr-mode";
        EditorUtility.SetDirty(coefficientRandomizer);
        EditorUtility.SetDirty(coordinator);
    }

    static DomainRandomizationProfile GetOrCreateCoefficientOnlyProfile()
    {
        const string profilePath =
            "Assets/Models/FinsROV/Hydrodynamics/DomainRandomizationProfile_DWP2CoefficientOnly.asset";
        DomainRandomizationProfile profile = AssetDatabase.LoadAssetAtPath<DomainRandomizationProfile>(profilePath);
        if (profile == null)
        {
            profile = ScriptableObject.CreateInstance<DomainRandomizationProfile>();
            AssetDatabase.CreateAsset(profile, profilePath);
        }
        profile.randomizeBody = false;
        profile.randomizeHydrodynamics = false;
        profile.randomizeHydrodynamicAxisScales = false;
        profile.randomizeThrusters = false;
        profile.randomizeWater = false;
        profile.randomizeWaves = false;
        profile.randomizeInitialState = false;
        EditorUtility.SetDirty(profile);
        AssetDatabase.SaveAssets();
        return profile;
    }

    static void Validate(HoldForPosition agent, bool expectDr)
    {
        if (agent == null || !agent.enabled ||
            agent.CurrentThrusterCommandMode != ThrusterCommandMode.NormalizedMaxForceRequest ||
            !Mathf.Approximately(agent.NormalizedThrusterActionLimit, 1f) ||
            agent.CurrentDenseRewardMode != HoldForPosition.DenseRewardMode.LearnToSwimGoalHold)
        {
            throw new InvalidOperationException("DWP2 mode-1 task contract was not serialized correctly.");
        }

        BehaviorParameters behavior = agent.GetComponent<BehaviorParameters>();
        if (behavior == null || behavior.BehaviorName != "HoldForPosition" ||
            behavior.BrainParameters.ActionSpec.NumContinuousActions != HoldForPosition.ContinuousActionSize)
        {
            throw new InvalidOperationException("DWP2 mode-1 fixture must expose the canonical eight-thruster Unity action ABI.");
        }
        DecisionRequester requester = agent.GetComponent<DecisionRequester>();
        if (requester == null || !requester.enabled || requester.DecisionPeriod != 5 ||
            !requester.TakeActionsBetweenDecisions)
        {
            throw new InvalidOperationException("DWP2 mode-1 fixture must retain the 10 Hz decision contract.");
        }

        DomainRandomizationCoordinator coordinator = agent.GetComponent<DomainRandomizationCoordinator>();
        Dwp2HydrodynamicForceCoefficientRandomizer coefficientRandomizer =
            agent.GetComponent<Dwp2HydrodynamicForceCoefficientRandomizer>();
        WaterObject mainBodyWaterObject = FindMainBodyWaterObject(agent.gameObject);
        if (coordinator == null || coordinator.enabled != expectDr ||
            coordinator.mode != (expectDr ? DomainRandomizationMode.Train : DomainRandomizationMode.Disabled) ||
            coefficientRandomizer == null || coefficientRandomizer.enabled != expectDr ||
            coefficientRandomizer.randomizePerEpisode != expectDr ||
            coefficientRandomizer.includeInactiveWaterObjects ||
            coefficientRandomizer.autoRefreshTargets ||
            coefficientRandomizer.waterObjects.Count != 1 ||
            coefficientRandomizer.waterObjects[0] != mainBodyWaterObject ||
            !Mathf.Approximately(coefficientRandomizer.minHydrodynamicForceCoefficient, CoefficientMin) ||
            !Mathf.Approximately(coefficientRandomizer.maxHydrodynamicForceCoefficient, CoefficientMax))
        {
            throw new InvalidOperationException("DWP2 DR must be coefficient-only and match the scene name.");
        }

        int activeWaterObjectCount = 0;
        foreach (WaterObject waterObject in agent.GetComponentsInChildren<WaterObject>(true))
        {
            if (waterObject != null && waterObject.isActiveAndEnabled)
            {
                activeWaterObjectCount++;
            }
        }
        if (!mainBodyWaterObject.isActiveAndEnabled || activeWaterObjectCount != 1)
        {
            throw new InvalidOperationException("The DWP2 training fixture must enable only the main-body physics mesh.");
        }
    }
}
