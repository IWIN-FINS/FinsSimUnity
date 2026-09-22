using System;
using System.Collections.Generic;
using System.Linq;
using Obi;
using FinsSim.Actuators;
using FinsSim.Hydrodynamics;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Areas;
using Unity.MLAgents.Policies;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

/// <summary>Converts 3Chase1 into independently replicated training areas.</summary>
public static class ThreeChaseOneParallelSceneBuilder
{
    public const string ScenePath = "Assets/FinsSimUnity/Tasks/Chase/ThreeChaseOne/Scenes/3Chase1.unity";
    public const string DomainRandomizationProfilePath =
        "Assets/Models/FinsROV/Hydrodynamics/DomainRandomizationProfile_HoldForPosition.asset";
    public const string FinsRovFossenPrefabPath =
        "Assets/Models/FinsROV/Calibrated/FinsROV_Fossen.prefab";
    public const string AreaName = "ThreeChaseOneTrainingArea";
    public const string ReplicatorName = "__ThreeChaseOneTrainingAreaReplicator";
    public const float AreaSeparation = 96f;
    public const int InspectorDefaultAreaCount = 1024;

    private const float CompactNetterSeparation = 8f;
    private const float CompactLeadRopeLength = 0.5f;
    private const float CompactNetWidth = CompactNetterSeparation - 2f * CompactLeadRopeLength;
    private const float CompactNetHeight = 2f;
    private const int CompactNetColumns = 7;
    private const int CompactNetRows = 4;
    private const int CompactEpisodeMaxSteps = 2400;

    [MenuItem("FinsSim/MARL/Prepare 3Chase1 Parallel Scene")]
    public static void PrepareThreeChaseOneParallelScene()
    {
        Scene scene = EditorSceneManager.OpenScene(ScenePath, OpenSceneMode.Single);
        TrainingAreaReplicator existing = UnityEngine.Object.FindFirstObjectByType<TrainingAreaReplicator>();
        if (existing == null)
        {
            CreateParallelArea(scene);
            existing = UnityEngine.Object.FindFirstObjectByType<TrainingAreaReplicator>();
        }

        if (existing == null || existing.baseArea == null)
        {
            throw new InvalidOperationException("3Chase1 TrainingAreaReplicator was not created correctly.");
        }

        EnsureFinsRovFossenChasers(existing.baseArea);
        NormalizeExistingParallelArea(existing);

        ValidateParallelScene(scene);
        EditorSceneManager.MarkSceneDirty(scene);
        if (!EditorSceneManager.SaveScene(scene))
        {
            throw new InvalidOperationException($"Unable to save {ScenePath}.");
        }
        Debug.Log("Prepared 3Chase1 with replicated training areas and scene-local domain randomization.");
    }

    public static void EnsurePrepared()
    {
        PrepareThreeChaseOneParallelScene();
    }

    [MenuItem("FinsSim/MARL/Configure Compact 3Chase1 Task")]
    public static void ConfigureCompactThreeChaseOneTask()
    {
        Scene scene = EditorSceneManager.OpenScene(ScenePath, OpenSceneMode.Single);
        TrainingAreaReplicator replicator = UnityEngine.Object.FindFirstObjectByType<TrainingAreaReplicator>();
        if (replicator == null || replicator.baseArea == null)
        {
            throw new InvalidOperationException("3Chase1 requires a prepared TrainingAreaReplicator before compact task setup.");
        }

        GameObject area = replicator.baseArea;
        CatchAreaManager manager = area.GetComponentInChildren<CatchAreaManager>(true);
        FishNetGenerator generator = area.GetComponentInChildren<FishNetGenerator>(true);
        if (manager == null || generator == null || manager.netter1 == null || manager.netter2 == null
            || manager.herder == null || manager.fish == null)
        {
            throw new InvalidOperationException("3Chase1 compact task setup is missing a manager, net generator, or actors.");
        }

        // Keep the original task orientation but shorten the net-to-prey-to-herder corridor.
        SetAreaLocalPosition(manager.netter1.transform, area.transform, new Vector3(2f, -5f, -4f));
        SetAreaLocalPosition(manager.netter2.transform, area.transform, new Vector3(2f, -5f, 4f));
        SetAreaLocalPosition(manager.fish.transform, area.transform, new Vector3(7f, -5f, 0f));
        SetAreaLocalPosition(manager.herder.transform, area.transform, new Vector3(12f, -5f, 0f));

        ConfigureCompactRewards(manager);
        RemoveStaleGeneratedNetBackups();
        ConfigureCompactStaticNet(generator, manager);

        ValidateCompactTask(manager, generator);
        EditorSceneManager.MarkSceneDirty(scene);
        if (!EditorSceneManager.SaveScene(scene))
        {
            throw new InvalidOperationException($"Unable to save compact 3Chase1 task scene: {ScenePath}.");
        }

        AssetDatabase.SaveAssets();
        Debug.Log(
            $"Configured compact 3Chase1 task: net={CompactNetWidth:F1}m x {CompactNetHeight:F1}m, " +
            $"cells={CompactNetColumns}x{CompactNetRows}, maxSteps={CompactEpisodeMaxSteps}.");
    }

    [MenuItem("FinsSim/MARL/Validate 3Chase1 Parallel Scene")]
    public static void ValidateCurrentParallelScene()
    {
        ValidateParallelScene(SceneManager.GetActiveScene());
        Debug.Log("3Chase1 parallel scene validation passed.");
    }

    private static void SetAreaLocalPosition(Transform actor, Transform area, Vector3 areaLocalPosition)
    {
        actor.position = area.TransformPoint(areaLocalPosition);
        EditorUtility.SetDirty(actor);
    }

    private static void ConfigureCompactRewards(CatchAreaManager manager)
    {
        manager.captureCriterion = CatchAreaManager.CaptureCriterion.NetSurfaceDistance;
        manager.netSurfaceCaptureDistance = 0.75f;
        manager.netSurfaceCaptureHoldTime = 0.20f;
        manager.uuvCaptureDistance = 1.5f;
        manager.chaserCaptureReward = 30f;
        manager.fishCapturePenalty = -30f;

        ChaserAgent[] chasers = { manager.netter1, manager.netter2, manager.herder };
        foreach (ChaserAgent chaser in chasers)
        {
            chaser.MaxStep = CompactEpisodeMaxSteps;
            chaser.rewardMode = ChaserRewardMode.HerdingNet;
            chaser.stepPenalty = -0.0005f;
            chaser.simpleChaseDistanceRewardScale = 0.04f;
            chaser.simpleChaseProgressRewardScale = 0.20f;
            chaser.simpleChaseDistanceRewardRange = 8f;
            chaser.fishClosingRewardScale = 0.08f;
            chaser.netterSpacingRewardScale = 0.015f;
            chaser.netterSpacingProgressRewardScale = 0.04f;
            chaser.desiredNetterSpacing = CompactNetterSeparation;
            chaser.netterSpacingTolerance = 1f;
            chaser.herderFishClosingRewardScale = 0.06f;
            chaser.herdingProgressRewardScale = 0.12f;
            chaser.angularVelocityPenaltyScale = 0.002f;
            chaser.linearVelocityPenaltyScale = 0.0005f;
            chaser.actionMagnitudePenaltyScale = 0.001f;
            chaser.actionDeltaPenaltyScale = 0.002f;
            EditorUtility.SetDirty(chaser);
        }

        PreyAgent prey = manager.fish;
        prey.MaxStep = CompactEpisodeMaxSteps;
        prey.moveSpeed = 0.35f;
        prey.verticalSpeedScale = 0.15f;
        prey.survivalReward = 0.0002f;
        prey.averageSeparationProgressRewardScale = 0.02f;
        prey.nearestThreatProgressRewardScale = 0.05f;
        EditorUtility.SetDirty(prey);
        EditorUtility.SetDirty(manager);
    }

    private static void ConfigureCompactStaticNet(FishNetGenerator generator, CatchAreaManager manager)
    {
        generator.netter1 = manager.netter1.transform;
        generator.netter2 = manager.netter2.transform;
        generator.netter1MarkPosition = manager.netter1.transform;
        generator.netter2MarkPosition = manager.netter2.transform;
        generator.resolution = new Vector2Int(CompactNetColumns, CompactNetRows);
        generator.size = new Vector2(CompactNetWidth / CompactNetColumns, CompactNetHeight / CompactNetRows);
        generator.fitWidthBetweenNettersOnBuild = true;
        generator.fixedNetWidth = 0f;
        generator.leadRopeLength = CompactLeadRopeLength;
        generator.ropeBlueprintResolution = 0.10f;
        generator.generateOnStart = false;
        generator.GenerateStaticNetForEditor("Assets/FinsSimUnity/Generated/FishNet/3Chase1_MLSceneManager_RopeBlueprints.asset");

        Transform root = generator.NetRoot;
        if (root == null)
        {
            throw new InvalidOperationException("Static FishNet generation did not create a net root.");
        }

        NetSurfaceModel model = root.GetComponent<NetSurfaceModel>();
        if (model == null)
        {
            throw new InvalidOperationException("Static FishNet generation did not create a NetSurfaceModel.");
        }

        model.SetTargetGeometry(CompactNetWidth, CompactNetHeight);
        model.SetGeometryTolerances(0.75f, 0.30f);
        model.ResolveHierarchy();
        manager.net = root;
        EditorUtility.SetDirty(model);
        EditorUtility.SetDirty(generator);
        EditorUtility.SetDirty(manager);
    }

    private static void RemoveStaleGeneratedNetBackups()
    {
        foreach (Transform candidate in UnityEngine.Object.FindObjectsByType<Transform>(
                     FindObjectsInactive.Include,
                     FindObjectsSortMode.None))
        {
            if (candidate != null && candidate.name == "GeneratedFishNetBackup")
            {
                UnityEngine.Object.DestroyImmediate(candidate.gameObject);
            }
        }
    }

    private static void ValidateCompactTask(CatchAreaManager manager, FishNetGenerator generator)
    {
        if (generator.generateOnStart || generator.resolution.x != CompactNetColumns || generator.resolution.y != CompactNetRows)
        {
            throw new InvalidOperationException("3Chase1 compact net must remain a 7x4 static FishNet.");
        }

        NetSurfaceModel model = generator.NetRoot != null
            ? generator.NetRoot.GetComponent<NetSurfaceModel>()
            : null;
        if (model == null
            || !Mathf.Approximately(model.TargetNetWidth, CompactNetWidth)
            || !Mathf.Approximately(model.TargetNetHeight, CompactNetHeight))
        {
            throw new InvalidOperationException("3Chase1 compact net surface geometry is invalid.");
        }

        int expectedRopeCount = CompactNetColumns * (CompactNetRows + 1)
                                + (CompactNetColumns + 1) * CompactNetRows
                                + 2;
        if (generator.NetRoot.GetComponentsInChildren<ObiRope>(true).Length != expectedRopeCount
            || UnityEngine.Object.FindObjectsByType<Transform>(FindObjectsInactive.Include, FindObjectsSortMode.None)
                .Any(transform => transform.name == "GeneratedFishNetBackup"))
        {
            throw new InvalidOperationException("3Chase1 compact task must contain exactly one 7x4 static FishNet.");
        }

        float netterDistance = Vector3.Distance(manager.netter1.transform.position, manager.netter2.transform.position);
        if (!Mathf.Approximately(netterDistance, CompactNetterSeparation)
            || manager.netter1.MaxStep != CompactEpisodeMaxSteps
            || manager.fish.MaxStep != CompactEpisodeMaxSteps)
        {
            throw new InvalidOperationException("3Chase1 compact spawn geometry or episode limit is invalid.");
        }
    }

    static void EnsureFinsRovFossenChasers(GameObject area)
    {
        ChaserAgent[] chasers = area.GetComponentsInChildren<ChaserAgent>(true);
        if (chasers.Length != 3)
        {
            throw new InvalidOperationException("3Chase1 base area must contain exactly three chasers before FinsROV conversion.");
        }

        GameObject prefab = AssetDatabase.LoadAssetAtPath<GameObject>(FinsRovFossenPrefabPath);
        if (prefab == null)
        {
            throw new InvalidOperationException($"Missing FinsROV_Fossen prefab: {FinsRovFossenPrefabPath}.");
        }

        int netterIndex = 0;
        foreach (ChaserAgent chaser in chasers)
        {
            if (IsFinsRovFossenInstance(chaser.gameObject))
            {
                ConfigureFinsRovChaser(chaser, chaser.role, netterIndex);
                if (chaser.role == AgentRole.Netter)
                {
                    netterIndex++;
                }
                continue;
            }

            int roleIndex = chaser.role == AgentRole.Netter ? netterIndex++ : 0;
            ReplaceLegacyChaserWithFinsRovFossen(chaser, prefab, roleIndex);
        }

        ChaserAgent[] converted = area.GetComponentsInChildren<ChaserAgent>(true);
        if (converted.Length != 3 || Array.Exists(converted, chaser => !IsFinsRovFossenInstance(chaser.gameObject)))
        {
            throw new InvalidOperationException("3Chase1 FinsROV_Fossen conversion did not produce exactly three prefab instances.");
        }

        CatchAreaManager manager = area.GetComponentInChildren<CatchAreaManager>(true);
        PreyAgent prey = area.GetComponentInChildren<PreyAgent>(true);
        FishNetGenerator generator = manager != null ? manager.GetComponentInChildren<FishNetGenerator>(true) : null;
        if (manager == null || prey == null || generator == null)
        {
            throw new InvalidOperationException("3Chase1 FinsROV_Fossen conversion lost a required manager, prey, or net generator.");
        }

        ChaserAgent[] netters = Array.FindAll(converted, chaser => chaser.role == AgentRole.Netter);
        ChaserAgent[] herders = Array.FindAll(converted, chaser => chaser.role == AgentRole.Herder);
        if (netters.Length != 2 || herders.Length != 1)
        {
            throw new InvalidOperationException("3Chase1 must retain two netters and one herder after FinsROV_Fossen conversion.");
        }

        manager.netter1 = netters[0];
        manager.netter2 = netters[1];
        manager.herder = herders[0];
        manager.fish = prey;
        netters[0].partnerNetter = netters[1];
        netters[1].partnerNetter = netters[0];
        herders[0].partnerNetter = null;

        prey.areaManager = manager;
        prey.chasers = new[] { netters[0].transform, netters[1].transform, herders[0].transform };
        generator.areaManager = manager;
        generator.netter1 = netters[0].transform;
        generator.netter2 = netters[1].transform;
        generator.netter1MarkPosition = netters[0].transform;
        generator.netter2MarkPosition = netters[1].transform;
        generator.useDynamicUuvAttachments = true;
        generator.addMissingObiCollidersToNetters = true;
        RebindStaticLeadRopes(generator, netters[0].transform, netters[1].transform);

        if (generator.NetRoot != null)
        {
            manager.net = generator.NetRoot;
        }

        EditorUtility.SetDirty(manager);
        EditorUtility.SetDirty(prey);
        EditorUtility.SetDirty(generator);
    }

    static bool IsFinsRovFossenInstance(GameObject gameObject)
    {
        return string.Equals(
            PrefabUtility.GetPrefabAssetPathOfNearestInstanceRoot(gameObject),
            FinsRovFossenPrefabPath,
            StringComparison.Ordinal);
    }

    static void ReplaceLegacyChaserWithFinsRovFossen(ChaserAgent legacy, GameObject prefab, int roleIndex)
    {
        string serializedChaser = EditorJsonUtility.ToJson(legacy);
        Transform legacyTransform = legacy.transform;
        Transform parent = legacyTransform.parent;
        int siblingIndex = legacyTransform.GetSiblingIndex();
        Vector3 localPosition = legacyTransform.localPosition;
        Quaternion localRotation = legacyTransform.localRotation;
        Vector3 localScale = legacyTransform.localScale;
        AgentRole role = legacy.role;

        GameObject instance = PrefabUtility.InstantiatePrefab(prefab, legacy.gameObject.scene) as GameObject;
        if (instance == null)
        {
            throw new InvalidOperationException("Failed to instantiate FinsROV_Fossen prefab.");
        }

        instance.name = role == AgentRole.Netter
            ? $"FinsROV_Fossen_Netter{roleIndex + 1}"
            : "FinsROV_Fossen_Herder";
        instance.transform.SetParent(parent, false);
        instance.transform.SetSiblingIndex(siblingIndex);
        instance.transform.localPosition = localPosition;
        instance.transform.localRotation = localRotation;
        instance.transform.localScale = localScale;
        instance.tag = legacy.tag;

        RemoveNonChaserControlComponents(instance);
        ChaserAgent chaser = instance.GetComponent<ChaserAgent>();
        if (chaser == null)
        {
            chaser = instance.AddComponent<ChaserAgent>();
        }
        EditorJsonUtility.FromJsonOverwrite(serializedChaser, chaser);
        ConfigureFinsRovChaser(chaser, role, roleIndex);

        UnityEngine.Object.DestroyImmediate(legacy.gameObject, true);
        EditorUtility.SetDirty(instance);
        EditorUtility.SetDirty(chaser);
    }

    static void ConfigureFinsRovChaser(ChaserAgent chaser, AgentRole role, int roleIndex)
    {
        chaser.role = role;
        chaser.selfTransform = chaser.transform;

        RemoveNonChaserControlComponents(chaser.gameObject, chaser);
        EnsureNetAttachmentObiComponents(chaser.gameObject);

        BehaviorParameters behavior = chaser.GetComponent<BehaviorParameters>();
        if (behavior == null)
        {
            behavior = chaser.gameObject.AddComponent<BehaviorParameters>();
        }
        behavior.BehaviorName = role == AgentRole.Netter ? "Netter" : "Herder";
        behavior.TeamId = role == AgentRole.Netter ? roleIndex + 1 : 0;
        behavior.BehaviorType = BehaviorType.Default;
        behavior.BrainParameters.VectorObservationSize = ChaserAgent.VectorObservationSize;
        behavior.BrainParameters.ActionSpec = ActionSpec.MakeContinuous(8);
        behavior.UseChildSensors = false;
        behavior.UseChildActuators = true;

        DecisionRequester requester = chaser.GetComponent<DecisionRequester>();
        if (requester == null)
        {
            requester = chaser.gameObject.AddComponent<DecisionRequester>();
        }
        // Keep the 0.1 s action refresh comfortably below the Fossen
        // thrusters' 0.2 s force-request timeout.
        requester.DecisionPeriod = 5;
        requester.TakeActionsBetweenDecisions = true;

        SerializedObject serialized = new SerializedObject(chaser);
        SetBool(serialized, "useFinsRovThrusters", true);
        SetEnum(serialized, "thrusterCommandMode", (int)ThrusterCommandMode.NormalizedMaxForceRequest);
        SetFloat(serialized, "actionForceScaleN", 1f);
        SetBool(serialized, "autoResolveThrustersFromChildren", true);
        SetBool(serialized, "logThrusterResolution", true);
        SetBool(serialized, "allowUnifiedMaxThrustToOverrideFinsRovForceScale", false);
        serialized.ApplyModifiedPropertiesWithoutUndo();

        EditorUtility.SetDirty(behavior);
        EditorUtility.SetDirty(requester);
    }

    static void RemoveNonChaserControlComponents(GameObject root, ChaserAgent keepChaser = null)
    {
        foreach (MonoBehaviour behaviour in root.GetComponentsInChildren<MonoBehaviour>(true))
        {
            if (behaviour == null || behaviour == keepChaser)
            {
                continue;
            }

            string typeName = behaviour.GetType().Name;
            string fullName = behaviour.GetType().FullName ?? string.Empty;
            if (behaviour is Agent
                || behaviour is DecisionRequester
                || typeName == "FinsROVResetRosBridge"
                || typeName == "FinsROVManualThrusterController"
                || typeName == "FinsROVPhysicsDiagnosticRuntimeProbe"
                || typeName == "DWP2DebugController"
                || typeName == "VelocityDebugDisplay"
                || typeName == "UuvDirectionalSpeedTester"
                || typeName == "NetSeparationStressTest"
                || fullName == "FinsSim.Networking.VehicleRosBridge"
                || fullName.StartsWith("FinsSim.Sensors.ROS.", StringComparison.Ordinal)
                || fullName.StartsWith("FinsSim.ROS.", StringComparison.Ordinal))
            {
                UnityEngine.Object.DestroyImmediate(behaviour, true);
            }
        }
    }

    static void EnsureNetAttachmentObiComponents(GameObject root)
    {
        if (root.GetComponent<Rigidbody>() == null)
        {
            throw new InvalidOperationException($"{root.name} FinsROV_Fossen instance has no root Rigidbody.");
        }

        BoxCollider collider = root.GetComponent<BoxCollider>();
        if (collider == null)
        {
            collider = root.AddComponent<BoxCollider>();
        }
        collider.center = Vector3.zero;
        collider.size = new Vector3(0.75f, 0.65f, 0.55f);
        collider.isTrigger = true;

        if (root.GetComponent<ObiRigidbody>() == null)
        {
            root.AddComponent<ObiRigidbody>();
        }

        ObiCollider obiCollider = root.GetComponent<ObiCollider>();
        if (obiCollider == null)
        {
            obiCollider = root.AddComponent<ObiCollider>();
        }
        obiCollider.sourceCollider = collider;
    }

    internal static void RebindStaticLeadRopes(
        FishNetGenerator generator,
        Transform netter1,
        Transform netter2)
    {
        Transform net = generator != null ? generator.NetRoot : null;
        if (net == null)
        {
            return;
        }

        foreach (ObiParticleAttachment attachment in net.GetComponentsInChildren<ObiParticleAttachment>(true))
        {
            if (attachment.gameObject.name == "LeadRope_Left")
            {
                ObiParticleAttachment[] attachments = attachment.GetComponents<ObiParticleAttachment>();
                if (attachments.Length > 0 && attachment == attachments[0])
                {
                    attachment.target = netter1;
                    attachment.attachmentType = ObiParticleAttachment.AttachmentType.Dynamic;
                }
            }
            else if (attachment.gameObject.name == "LeadRope_Right")
            {
                ObiParticleAttachment[] attachments = attachment.GetComponents<ObiParticleAttachment>();
                if (attachments.Length > 0 && attachment == attachments[0])
                {
                    attachment.target = netter2;
                    attachment.attachmentType = ObiParticleAttachment.AttachmentType.Dynamic;
                }
            }
        }
    }

    static void SetBool(SerializedObject serialized, string propertyName, bool value)
    {
        SerializedProperty property = serialized.FindProperty(propertyName);
        if (property != null)
        {
            property.boolValue = value;
        }
    }

    static void SetFloat(SerializedObject serialized, string propertyName, float value)
    {
        SerializedProperty property = serialized.FindProperty(propertyName);
        if (property != null)
        {
            property.floatValue = value;
        }
    }

    static void SetEnum(SerializedObject serialized, string propertyName, int value)
    {
        SerializedProperty property = serialized.FindProperty(propertyName);
        if (property != null)
        {
            property.enumValueIndex = value;
        }
    }

    static void CreateParallelArea(Scene scene)
    {
        CatchAreaManager manager = UnityEngine.Object.FindFirstObjectByType<CatchAreaManager>();
        PreyAgent prey = UnityEngine.Object.FindFirstObjectByType<PreyAgent>();
        ChaserAgent[] chasers = UnityEngine.Object.FindObjectsByType<ChaserAgent>(FindObjectsInactive.Include, FindObjectsSortMode.None);
        if (manager == null || prey == null || chasers.Length != 3)
        {
            throw new InvalidOperationException(
                $"3Chase1 must contain one {nameof(CatchAreaManager)}, one {nameof(PreyAgent)}, and three {nameof(ChaserAgent)} instances.");
        }

        var area = new GameObject(AreaName);
        area.transform.SetPositionAndRotation(Vector3.zero, Quaternion.identity);
        manager.transform.SetParent(area.transform, true);
        prey.transform.SetParent(area.transform, true);
        foreach (ChaserAgent chaser in chasers)
        {
            chaser.transform.SetParent(area.transform, true);
        }

        ConfigureTrainingOnlyComponents(manager);
        WaterCurrentDomainRandomizer current = CreateCurrentRandomizer(area.transform, chasers);
        ConfigureDomainRandomization(area, chasers, current);

        var runtime = area.AddComponent<ThreeChaseOneParallelAreaRuntime>();
        runtime.areaSeparation = AreaSeparation;
        runtime.seedStride = 1_000_003;
        runtime.ConfigureArea();

        var replicatorRoot = new GameObject(ReplicatorName);
        TrainingAreaReplicator replicator = replicatorRoot.AddComponent<TrainingAreaReplicator>();
        replicator.baseArea = area;
        replicator.numAreas = InspectorDefaultAreaCount;
        replicator.separation = AreaSeparation;
        replicator.buildOnly = true;
    }

    static void NormalizeExistingParallelArea(TrainingAreaReplicator replicator)
    {
        if (replicator.baseArea == null)
        {
            throw new InvalidOperationException("3Chase1 TrainingAreaReplicator has no baseArea.");
        }
        replicator.name = ReplicatorName;
        replicator.numAreas = InspectorDefaultAreaCount;
        replicator.separation = AreaSeparation;
        replicator.buildOnly = true;

        ThreeChaseOneParallelAreaRuntime runtime = replicator.baseArea.GetComponent<ThreeChaseOneParallelAreaRuntime>();
        if (runtime == null)
        {
            throw new InvalidOperationException("3Chase1 base area has no ThreeChaseOneParallelAreaRuntime.");
        }
        runtime.areaSeparation = AreaSeparation;
        ConfigureTrainingOnlyComponents(replicator.baseArea.GetComponentInChildren<CatchAreaManager>(true));
        ChaserAgent[] chasers = replicator.baseArea.GetComponentsInChildren<ChaserAgent>(true);
        WaterCurrentDomainRandomizer current = replicator.baseArea.GetComponentInChildren<WaterCurrentDomainRandomizer>(true);
        if (current == null)
        {
            current = CreateCurrentRandomizer(replicator.baseArea.transform, chasers);
        }
        else
        {
            current.applyForcesToRigidbodies = false;
            current.randomizeOnStart = false;
            current.randomizePeriodically = false;
        }
        ConfigureDomainRandomization(replicator.baseArea, chasers, current);
        runtime.ConfigureArea();
    }

    static void ConfigureTrainingOnlyComponents(CatchAreaManager manager)
    {
        if (manager == null)
        {
            throw new InvalidOperationException("3Chase1 base area has no CatchAreaManager.");
        }

        manager.showRewardDebug = false;
        ThreeChaseOneBaselineController baseline = manager.GetComponent<ThreeChaseOneBaselineController>();
        if (baseline != null)
        {
            baseline.baselineEnabled = false;
            baseline.enabled = false;
        }
    }

    static WaterCurrentDomainRandomizer CreateCurrentRandomizer(Transform parent, ChaserAgent[] chasers)
    {
        var currentObject = new GameObject("WaterCurrentDomainRandomizer");
        currentObject.transform.SetParent(parent, false);
        WaterCurrentDomainRandomizer current = currentObject.AddComponent<WaterCurrentDomainRandomizer>();
        current.randomizeOnStart = false;
        current.randomizePeriodically = false;
        // HydrodynamicsController samples this component as its water provider.
        // Adding a second direct force here would apply the same current twice.
        current.applyForcesToRigidbodies = false;
        current.logRandomizedCurrent = false;
        current.drawDebug = false;
        foreach (ChaserAgent chaser in chasers)
        {
            Rigidbody rigidbody = chaser.GetComponent<Rigidbody>();
            if (rigidbody != null)
            {
                current.targetRigidbodies.Add(rigidbody);
            }
        }
        return current;
    }

    static void ConfigureDomainRandomization(
        GameObject area,
        IEnumerable<ChaserAgent> chasers,
        WaterCurrentDomainRandomizer current)
    {
        DomainRandomizationProfile profile = AssetDatabase.LoadAssetAtPath<DomainRandomizationProfile>(
            DomainRandomizationProfilePath);
        if (profile == null)
        {
            throw new InvalidOperationException($"Missing DR profile at {DomainRandomizationProfilePath}.");
        }

        DomainRandomizationCoordinator coordinator = area.GetComponent<DomainRandomizationCoordinator>();
        if (coordinator == null)
        {
            coordinator = area.AddComponent<DomainRandomizationCoordinator>();
        }
        coordinator.profile = profile;
        coordinator.mode = DomainRandomizationMode.Train;
        coordinator.baseSeed = 12345;
        coordinator.incrementSeedPerEpisode = true;
        coordinator.randomizeOnStart = false;
        coordinator.autoFindTargets = true;
        coordinator.targetBehaviours.Clear();
        coordinator.targetBehaviours.Add(current);

        foreach (ChaserAgent chaser in chasers)
        {
            if (chaser.GetComponentsInChildren<Thruster>(true).Length > 0 &&
                chaser.GetComponent<ThrusterRandomizationTarget>() == null)
            {
                chaser.gameObject.AddComponent<ThrusterRandomizationTarget>();
            }

            // FinsROV_Fossen randomizes its Fossen hydrodynamics through the
            // coordinator. The DWP2-only targets remain for legacy UUV scenes.
            if (chaser.GetComponent<HydrodynamicsController>() == null)
            {
                EnsureDwp2Targets(chaser);
            }
        }
    }

    static void EnsureDwp2Targets(ChaserAgent chaser)
    {
        Rigidbody rigidbody = chaser.GetComponent<Rigidbody>();
        if (rigidbody == null)
        {
            throw new InvalidOperationException($"{chaser.name} has no Rigidbody for domain randomization.");
        }

        Dwp2MassBuoyancyRandomizer mass = chaser.GetComponent<Dwp2MassBuoyancyRandomizer>();
        if (mass == null)
        {
            mass = chaser.gameObject.AddComponent<Dwp2MassBuoyancyRandomizer>();
        }
        mass.targetRigidbody = rigidbody;
        mass.waterObjectRoot = chaser.transform;
        mass.randomizeOnStart = false;
        mass.randomizePerEpisode = true;
        mass.logRandomizedValue = false;

        Dwp2HydrodynamicForceCoefficientRandomizer drag =
            chaser.GetComponent<Dwp2HydrodynamicForceCoefficientRandomizer>();
        if (drag == null)
        {
            drag = chaser.gameObject.AddComponent<Dwp2HydrodynamicForceCoefficientRandomizer>();
        }
        drag.targetRoot = chaser.transform;
        drag.randomizeOnStart = false;
        drag.randomizePerEpisode = true;
        drag.logRandomizedValue = false;

        Dwp2AxisHydrodynamicScaleRandomizer axis =
            chaser.GetComponent<Dwp2AxisHydrodynamicScaleRandomizer>();
        if (axis == null)
        {
            axis = chaser.gameObject.AddComponent<Dwp2AxisHydrodynamicScaleRandomizer>();
        }
        axis.targetRoot = chaser.transform;
        axis.randomizeOnStart = false;
        axis.randomizePerEpisode = true;
        axis.logRandomizedValue = false;
    }

    static void ValidateParallelScene(Scene scene)
    {
        if (scene.path != ScenePath)
        {
            throw new InvalidOperationException($"Expected active scene {ScenePath}, got {scene.path}.");
        }
        TrainingAreaReplicator replicator = UnityEngine.Object.FindFirstObjectByType<TrainingAreaReplicator>();
        if (replicator == null || replicator.baseArea == null || !replicator.buildOnly ||
            !Mathf.Approximately(replicator.separation, AreaSeparation))
        {
            throw new InvalidOperationException("3Chase1 TrainingAreaReplicator is missing or invalid.");
        }

        ThreeChaseOneParallelAreaRuntime runtime =
            replicator.baseArea.GetComponent<ThreeChaseOneParallelAreaRuntime>();
        CatchAreaManager[] managers = replicator.baseArea.GetComponentsInChildren<CatchAreaManager>(true);
        ChaserAgent[] chasers = replicator.baseArea.GetComponentsInChildren<ChaserAgent>(true);
        PreyAgent[] prey = replicator.baseArea.GetComponentsInChildren<PreyAgent>(true);
        if (runtime == null || managers.Length != 1 || chasers.Length != 3 || prey.Length != 1)
        {
            throw new InvalidOperationException("3Chase1 base area is missing isolated runtime actors.");
        }

        CatchAreaManager manager = managers[0];
        if (manager.netter1 == null || manager.netter2 == null || manager.herder == null || manager.fish != prey[0] ||
            manager.netter1.areaManager != manager || manager.netter2.areaManager != manager || manager.herder.areaManager != manager ||
            prey[0].areaManager != manager || prey[0].chasers == null || prey[0].chasers.Length != 3)
        {
            throw new InvalidOperationException("3Chase1 base area has cross-area manager or actor references.");
        }

        foreach (ChaserAgent chaser in chasers)
        {
            BehaviorParameters behavior = chaser.GetComponent<BehaviorParameters>();
            if (behavior == null || behavior.BrainParameters.VectorObservationSize != ChaserAgent.VectorObservationSize ||
                behavior.BrainParameters.ActionSpec.NumContinuousActions != 8)
            {
                throw new InvalidOperationException(
                    $"3Chase1 chasers must retain {ChaserAgent.VectorObservationSize}D observations and 8D actions."
                );
            }

            if (!IsFinsRovFossenInstance(chaser.gameObject) ||
                chaser.GetComponent<HydrodynamicsController>() == null ||
                chaser.GetComponent<ObiRigidbody>() == null ||
                chaser.GetComponent<ObiCollider>() == null ||
                chaser.GetComponentsInChildren<Thruster>(true).Length != 8)
            {
                throw new InvalidOperationException(
                    "3Chase1 chasers must be FinsROV_Fossen scene instances with Fossen hydrodynamics, Obi, and eight thrusters.");
            }
        }

        if (replicator.baseArea.GetComponentInChildren<WaterCurrentDomainRandomizer>(true) == null ||
            replicator.baseArea.GetComponentInChildren<DomainRandomizationCoordinator>(true) == null ||
            manager.GetComponentInChildren<FishNetGenerator>(true) == null || manager.net == null)
        {
            throw new InvalidOperationException("3Chase1 base area is missing scene-local net or DR components.");
        }
    }
}
