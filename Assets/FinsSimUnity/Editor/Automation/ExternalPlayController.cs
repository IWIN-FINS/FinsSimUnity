using System;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;
using FinsSim.Hydrodynamics;
using FinsSim.Networking;
using Unity.MLAgents;

[InitializeOnLoad]
public static class ExternalPlayController
{
    private const string CommandFile = "/tmp/unity_play_command";
    private const string StatusFile = "/tmp/unity_play_status.json";
    // Keep the historical global files for existing experiment runners, but
    // offer a project-specific channel when more than one Unity Editor is open.
    // This prevents an unrelated MARUS/FinsSim worktree from consuming a command
    // intended for this project.
    private const string ProjectCommandFile = "/tmp/finssimunity_play_command";
    private const string ProjectStatusFile = "/tmp/finssimunity_play_status.json";
    private const double CheckIntervalSeconds = 0.2;
    private const double StatusIntervalSeconds = 1.0;

    private static double nextCommandCheckTime;
    private static double nextStatusWriteTime;
    private static string lastCommand = "";

    static ExternalPlayController()
    {
        EditorApplication.update -= CheckCommand;
        EditorApplication.update += CheckCommand;
        EditorApplication.playModeStateChanged -= OnPlayModeStateChanged;
        EditorApplication.playModeStateChanged += OnPlayModeStateChanged;

        WriteStatus("initialized");
        Debug.Log($"[ExternalPlayController] Listening for commands at {CommandFile} and {ProjectCommandFile}");
    }

    private static void CheckCommand()
    {
        var now = EditorApplication.timeSinceStartup;
        if (now >= nextStatusWriteTime)
        {
            nextStatusWriteTime = now + StatusIntervalSeconds;
            WriteStatus("heartbeat");
        }

        if (now < nextCommandCheckTime)
        {
            return;
        }

        nextCommandCheckTime = now + CheckIntervalSeconds;

        if (EditorApplication.isCompiling || EditorApplication.isUpdating)
        {
            return;
        }

        // Prefer the project-specific mailbox.  The global mailbox remains for
        // compatibility with existing automation that runs a single Editor.
        string commandFile = File.Exists(ProjectCommandFile)
            ? ProjectCommandFile
            : CommandFile;
        if (!File.Exists(commandFile))
        {
            return;
        }

        string command;
        try
        {
            // Keep the payload's original case.  `open_scene:Assets/...` is
            // case-sensitive on Linux, while command dispatch below remains
            // case-insensitive.
            command = File.ReadAllText(commandFile).Trim();
            File.Delete(commandFile);
        }
        catch (IOException)
        {
            return;
        }
        catch (UnauthorizedAccessException e)
        {
            Debug.LogError($"[ExternalPlayController] Cannot read command file: {e.Message}");
            WriteStatus("command_read_failed", e.Message);
            return;
        }

        lastCommand = command;
        HandleCommand(command);
    }

    private static void HandleCommand(string command)
    {
        string normalizedCommand = command.ToLowerInvariant();
        switch (normalizedCommand)
        {
            case "start":
            case "play":
                StartPlayMode();
                break;
            case "stop":
                StopPlayMode();
                break;
            case "toggle":
                TogglePlayMode();
                break;
            case "pause":
                EditorApplication.isPaused = true;
                Debug.Log("[ExternalPlayController] Paused");
                WriteStatus("paused");
                break;
            case "resume":
                EditorApplication.isPaused = false;
                Debug.Log("[ExternalPlayController] Resumed");
                WriteStatus("resumed");
                break;
            case "step":
                EditorApplication.Step();
                Debug.Log("[ExternalPlayController] Stepped one frame");
                WriteStatus("stepped");
                break;
            case "status":
                WriteStatus("status_requested");
                break;
            case "install_fossen_reset_diagnostics":
                FinsROVFossenDiagnosticsSceneSetup.InstallInActiveScene();
                WriteStatus("fossen_reset_diagnostics_installed");
                break;
            case "prepare_hold_for_position_fossen_parallel_30s_normalized_max_force":
                if (SceneManager.GetActiveScene().isDirty)
                {
                    WriteStatus(
                        "normalized_max_force_scene_prepare_failed",
                        "active scene has unsaved changes; save or discard them before preparing a new scene");
                    break;
                }
                HoldForPositionFossenParallelSceneBuilder
                    .PrepareHoldForPositionFossenParallel30sNormalizedMaxForceScene();
                WriteStatus("normalized_max_force_scene_prepared");
                break;
            case var evaluationCommand when evaluationCommand.StartsWith("prepare_ros_evaluation_scene", StringComparison.Ordinal):
                if (FinsROSRosEvaluationSceneSetup.PrepareActiveScene(command, out var evaluationMessage))
                {
                    WriteStatus("ros_evaluation_scene_prepared", evaluationMessage);
                }
                else
                {
                    WriteStatus("ros_evaluation_scene_prepare_failed", evaluationMessage);
                }
                break;
            case "audit_ros_evaluation_thrusters":
                if (FinsROSRosEvaluationSceneSetup.TryGetRuntimeThrusterAudit(out var auditMessage))
                {
                    WriteStatus("ros_evaluation_thruster_audit", auditMessage);
                }
                else
                {
                    WriteStatus("ros_evaluation_thruster_audit_failed", auditMessage);
                }
                break;
            case var sceneCommand when sceneCommand.StartsWith("open_scene:", StringComparison.Ordinal):
                if (FinsROSRosEvaluationSceneSetup.OpenProjectScene(command, out var sceneMessage))
                {
                    WriteStatus("scene_opened", sceneMessage);
                }
                else
                {
                    WriteStatus("scene_open_failed", sceneMessage);
                }
                break;
            case "isolate_3chase1_herder_ros":
                ThreeChaseOneRosIsolationSceneSetup.InstallHerderIsolation();
                WriteStatus("3chase1_herder_ros_isolation_installed");
                break;
            case "clear_3chase1_herder_ros_isolation":
                ThreeChaseOneRosIsolationSceneSetup.RemoveHerderIsolation();
                WriteStatus("3chase1_herder_ros_isolation_cleared");
                break;
            case "enable_3chase1_action_audit":
                ThreeChaseOneActionAuditSceneSetup.SetEnabled(true);
                WriteStatus("3chase1_action_audit_enabled");
                break;
            case "disable_3chase1_action_audit":
                ThreeChaseOneActionAuditSceneSetup.SetEnabled(false);
                WriteStatus("3chase1_action_audit_disabled");
                break;
            case "regenerate_trinetcapture_target_buoyancy_mesh":
                TriNetCaptureSceneBuilder.RegenerateTargetBuoyancyMesh();
                WriteStatus("trinetcapture_target_buoyancy_mesh_regenerated");
                break;
            case "prepare_trinetcapture_scene":
                TriNetCaptureSceneBuilder.PrepareTriNetCaptureScene();
                WriteStatus("trinetcapture_scene_prepared");
                break;
            case "rebuild_trinetcapture_static_net":
                TriNetCaptureSceneBuilder.RebuildStaticXpbdCloth();
                WriteStatus("trinetcapture_static_xpbd_cloth_rebuilt");
                break;
            case "upgrade_obi_sample_materials_hdrp":
                ObiHdrpSampleMaterialUpgrader.UpgradeSampleMaterials();
                WriteStatus("obi_sample_materials_upgraded_hdrp");
                break;
            case "create_trinet_tow_test_scene":
                TriNetTowTestSceneBuilder.CreateTestScene();
                WriteStatus("trinet_tow_test_scene_created");
                break;
            case "reset_trinet_tow_test_vertical_yz":
                TriNetTowTestSceneBuilder.ResetVerticalYzTowFormation();
                WriteStatus("trinet_tow_test_vertical_yz_reset");
                break;
            case "create_quadnet_tow_test_scene":
                QuadNetTowTestSceneBuilder.CreateTestScene();
                WriteStatus("quadnet_tow_test_scene_created");
                break;
            case "reload_trinetcapture_scene":
                ReloadTriNetCaptureScene();
                WriteStatus("trinetcapture_scene_reloaded");
                break;
            default:
                Debug.LogWarning($"[ExternalPlayController] Unknown command: {command}");
                WriteStatus("unknown_command", command);
                break;
        }
    }

    private static void StartPlayMode()
    {
        if (EditorApplication.isPlayingOrWillChangePlaymode)
        {
            Debug.Log("[ExternalPlayController] Already playing or entering Play Mode");
            WriteStatus("already_playing_or_entering");
            return;
        }

        Debug.Log("[ExternalPlayController] Starting Play Mode");
        WriteStatus("starting_play_mode");
        EditorApplication.isPlaying = true;
    }

    private static void StopPlayMode()
    {
        if (!EditorApplication.isPlayingOrWillChangePlaymode)
        {
            Debug.Log("[ExternalPlayController] Already stopped");
            WriteStatus("already_stopped");
            return;
        }

        Debug.Log("[ExternalPlayController] Stopping Play Mode");
        WriteStatus("stopping_play_mode");
        EditorApplication.isPlaying = false;
    }

    private static void TogglePlayMode()
    {
        if (EditorApplication.isPlayingOrWillChangePlaymode)
        {
            StopPlayMode();
        }
        else
        {
            StartPlayMode();
        }
    }

    private static void ReloadTriNetCaptureScene()
    {
        if (EditorApplication.isPlayingOrWillChangePlaymode)
        {
            Debug.LogWarning("[ExternalPlayController] Stop Play Mode before reloading TriNetCapture.");
            return;
        }

        const string scenePath = "Assets/FinsSimUnity/Tasks/NetCapture/Scenes/TriNetCapture.unity";
        EditorSceneManager.OpenScene(scenePath, OpenSceneMode.Single);
        Debug.Log("[ExternalPlayController] Reloaded TriNetCapture scene.");
    }

    private static void OnPlayModeStateChanged(PlayModeStateChange state)
    {
        WriteStatus(state.ToString());
    }

    private static void WriteStatus(string eventName, string message = "")
    {
        try
        {
            Scene activeScene = SceneManager.GetActiveScene();
            string projectPath = Directory.GetParent(Application.dataPath)?.FullName ?? string.Empty;
            var json =
                "{\n" +
                $"  \"event\": \"{EscapeJson(eventName)}\",\n" +
                $"  \"message\": \"{EscapeJson(message)}\",\n" +
                $"  \"lastCommand\": \"{EscapeJson(lastCommand)}\",\n" +
                $"  \"processId\": {System.Diagnostics.Process.GetCurrentProcess().Id},\n" +
                $"  \"unityVersion\": \"{EscapeJson(Application.unityVersion)}\",\n" +
                $"  \"projectPath\": \"{EscapeJson(projectPath)}\",\n" +
                $"  \"activeScenePath\": \"{EscapeJson(activeScene.path)}\",\n" +
                $"  \"activeSceneName\": \"{EscapeJson(activeScene.name)}\",\n" +
                $"  \"isPlaying\": {ToJsonBool(EditorApplication.isPlaying)},\n" +
                $"  \"isPlayingOrWillChangePlaymode\": {ToJsonBool(EditorApplication.isPlayingOrWillChangePlaymode)},\n" +
                $"  \"isPaused\": {ToJsonBool(EditorApplication.isPaused)},\n" +
                $"  \"isCompiling\": {ToJsonBool(EditorApplication.isCompiling)},\n" +
                $"  \"isUpdating\": {ToJsonBool(EditorApplication.isUpdating)},\n" +
                $"  \"timeSinceStartup\": {EditorApplication.timeSinceStartup:F3}\n" +
                "}\n";

            File.WriteAllText(StatusFile, json);
            File.WriteAllText(ProjectStatusFile, json);
        }
        catch (Exception e)
        {
            Debug.LogWarning($"[ExternalPlayController] Failed to write status: {e.Message}");
        }
    }

    private static string ToJsonBool(bool value)
    {
        return value ? "true" : "false";
    }

    private static string EscapeJson(string value)
    {
        return (value ?? string.Empty)
            .Replace("\\", "\\\\")
            .Replace("\"", "\\\"");
    }
}

/// <summary>
/// Configures an already-open Fossen scene for a ROS2 controlled experiment.
///
/// The RL training scenes deliberately enable an ML-Agents policy and disable
/// the VehicleRosBridge.  That is appropriate for Python ML-Agents training,
/// but it would leave two unrelated action sources in a hardware-aligned ROS2
/// evaluation.  The experiment recorder therefore asks the editor to apply
/// this scene-instance-only setup before Play mode: ML-Agents components are
/// disabled, VehicleRosBridge is enabled, and the requested reset pose becomes
/// the vehicle's initial state.  No prefab or scene asset is saved here.
///
/// Command format (written by the ROS2 simulation runner):
/// <c>prepare_ros_evaluation_scene:&lt;x&gt;,&lt;y&gt;,&lt;z&gt;,&lt;yaw_deg&gt;[,force_n|normalized_direct][,time_scale]</c>.
/// Omitting the suffix uses the world origin and zero yaw.
/// </summary>
public static class FinsROSRosEvaluationSceneSetup
{
    private const string CommandPrefix = "prepare_ros_evaluation_scene";
    private const string OpenSceneCommandPrefix = "open_scene:";

    /// <summary>
    /// Open an `Assets/...` scene requested by the experiment runner.  This is
    /// only used immediately after the runner starts a fresh Editor; it never
    /// silently replaces a scene in an editor that the operator already had
    /// open.
    /// </summary>
    public static bool OpenProjectScene(string command, out string message)
    {
        message = string.Empty;
        if (string.IsNullOrWhiteSpace(command) ||
            !command.StartsWith(OpenSceneCommandPrefix, StringComparison.OrdinalIgnoreCase))
        {
            message = $"expected `{OpenSceneCommandPrefix}Assets/.../scene.unity`, got `{command}`";
            return false;
        }

        string assetPath = command.Substring(OpenSceneCommandPrefix.Length).Trim().Replace('\\', '/');
        if (!assetPath.StartsWith("Assets/", StringComparison.Ordinal) || !assetPath.EndsWith(".unity", StringComparison.OrdinalIgnoreCase))
        {
            message = "scene path must be a project-relative Assets/.../*.unity path";
            return false;
        }

        string projectPath = Directory.GetParent(Application.dataPath)?.FullName ?? string.Empty;
        string absolutePath = Path.GetFullPath(Path.Combine(projectPath, assetPath));
        if (!File.Exists(absolutePath))
        {
            message = $"scene does not exist: {assetPath}";
            return false;
        }

        EditorSceneManager.OpenScene(assetPath, OpenSceneMode.Single);
        message = $"activeScene={assetPath}";
        Debug.Log($"[{nameof(FinsROSRosEvaluationSceneSetup)}] Opened experiment scene: {assetPath}");
        return true;
    }

    public static bool PrepareActiveScene(string command, out string message)
    {
        if (!TryParseInitialPose(
                command,
                out var position,
                out var yawDeg,
                out var thrusterCommandMode,
                out var simulationTimeScale,
                out var ros2ControlLockstep,
                out message))
        {
            return false;
        }

        if (!ConfigureSteppedSimulationClock(simulationTimeScale, ros2ControlLockstep, out var clockMessage))
        {
            message = clockMessage;
            Debug.LogError($"[{nameof(FinsROSRosEvaluationSceneSetup)}] {message}");
            return false;
        }

        var vehicle = FindEvaluationVehicle();
        if (vehicle == null)
        {
            message = "no FinsROV vehicle with a VehicleRosBridge exists in the active scene";
            Debug.LogError($"[{nameof(FinsROSRosEvaluationSceneSetup)}] {message}");
            return false;
        }

        var bridge = vehicle.GetComponent<VehicleRosBridge>() ?? vehicle.GetComponentInChildren<VehicleRosBridge>(true);
        if (bridge == null)
        {
            message = $"{vehicle.name} has no VehicleRosBridge component";
            Debug.LogError($"[{nameof(FinsROSRosEvaluationSceneSetup)}] {message}", vehicle);
            return false;
        }

        // The simulation runner selects this per-controller profile.  This
        // scene-instance-only path is intentionally force-based so the ROS
        // replay command is not reinterpreted as a normalized ML-Agents action.
        bridge.bridgeName = "finsrov";
        bridge.thrusterTopic = "/finsrov/thrusters_out";
        bridge.resetTopic = "/finsrov/reset";
        bridge.controllerPoseTopic = "/finsrov/controller/pose";
        bridge.controllerImuTopic = "/finsrov/controller/imu";
        bridge.controllerDvlTopic = "/finsrov/controller/dvl";
        bridge.controllerDepthTopic = "/finsrov/controller/depth";
        bridge.thrusterAppliedWrenchTopic = "/finsrov/debug/thruster_applied_wrench";
        bridge.subscribeThrusters = true;
        bridge.subscribeReset = true;
        bridge.publishPose = true;
        bridge.publishImu = true;
        bridge.publishDvl = true;
        bridge.publishDepth = true;
        bridge.publishThrusterAppliedWrench = true;
        bridge.autoResolveThrustersFromChildren = true;
        bridge.reapplyLastThrusterCommandInFixedUpdate = true;
        bridge.disableManualControllerOnEnable = true;
        bridge.zeroThrustersWhenDisabled = true;
        bridge.thrusterCommandMode = thrusterCommandMode;
        bridge.enabled = true;

        // A training scene may contain an Agent on the target vehicle as well
        // as area/replicator components that instantiate or reactivate agents
        // when Play starts.  Disable them across the active scene rather than
        // only on the selected GameObject: ROS must be the sole action source
        // for an input--output hydrodynamics replay.
        var disabledActionSourceCount = 0;
        foreach (var behaviour in UnityEngine.Object.FindObjectsByType<Behaviour>(
                     FindObjectsInactive.Include,
                     FindObjectsSortMode.None))
        {
            if (behaviour == null || behaviour == bridge || !IsRosEvaluationActionSource(behaviour))
            {
                continue;
            }

            if (behaviour.enabled)
            {
                behaviour.enabled = false;
                disabledActionSourceCount++;
            }
        }

        var thrusterController = vehicle.GetComponent<FinsSim.Actuators.ThrusterController>() ??
                                 vehicle.GetComponentInChildren<FinsSim.Actuators.ThrusterController>(true);
        if (thrusterController != null)
        {
            // The hydrodynamic replay publishes forces reconstructed from the
            // physical motor RPM telemetry.  They are therefore already the
            // force delivered by each real actuator, not a motor-level demand.
            // Applying a scene's training-only response compression or
            // first-order actuator model here would apply the same actuator
            // transfer twice and invalidate a body-wrench comparison.  Keep
            // this an unsaved, evaluation-instance override: normal training
            // scenes still use their configured actuator dynamics.
            if (thrusterCommandMode == VehicleRosThrusterCommandMode.ForceN)
            {
                thrusterController.ForceResponseCompressionEnabled = false;
                EditorUtility.SetDirty(thrusterController);
                foreach (var thruster in thrusterController.thrusters)
                {
                    if (thruster != null)
                    {
                        thruster.DynamicsModel = FinsSim.Actuators.Thruster.ActuatorDynamicsModel.None;
                        EditorUtility.SetDirty(thruster);
                    }
                }
            }

            thrusterController.ApplyInput(new float[8]);
        }

        vehicle.transform.SetPositionAndRotation(position, Quaternion.Euler(0f, yawDeg, 0f));
        var rigidBody = vehicle.GetComponent<Rigidbody>() ?? vehicle.GetComponentInChildren<Rigidbody>(true);
        if (rigidBody != null)
        {
            rigidBody.linearVelocity = Vector3.zero;
            rigidBody.angularVelocity = Vector3.zero;
        }
        Physics.SyncTransforms();

        // Entering Play serializes the active scene instance.  Mark the
        // instance dirty so the transient ROS-only configuration (notably the
        // ForceN mode) survives that transition, but deliberately do not save
        // the scene asset: stopping Play restores the on-disk training scene.
        EditorUtility.SetDirty(vehicle);
        EditorUtility.SetDirty(bridge);
        EditorSceneManager.MarkSceneDirty(SceneManager.GetActiveScene());

        message = $"vehicle={vehicle.name}; initial=[{position.x:F3},{position.y:F3},{position.z:F3},{yawDeg:F1}]; thrusterMode={thrusterCommandMode}; disabledActionSources={disabledActionSourceCount}; {clockMessage}";
        Debug.Log($"[{nameof(FinsROSRosEvaluationSceneSetup)}] ROS evaluation scene prepared: {message}", vehicle);
        return true;
    }

    static bool IsRosEvaluationActionSource(Behaviour behaviour)
    {
        var type = behaviour.GetType();
        var typeName = type.Name;
        return behaviour is Agent ||
            behaviour is DecisionRequester ||
            typeName == "ControlForPosition" ||
            typeName == "HoldForPosition" ||
            typeName == "TrajectoryTrackingAgent" ||
            typeName == "HoldForPositionParallelAreaRuntime" ||
            typeName == "TrainingAreaReplicator" ||
            typeName.Contains("Manual", StringComparison.OrdinalIgnoreCase);
    }

    // Fossen and DWP2 variants deliberately have distinct root names.  Resolve
    // by bridge rather than a backend-specific component so this temporary
    // ROS-only setup can exercise either hydrodynamics implementation.
    static GameObject FindEvaluationVehicle()
    {
        foreach (string name in new[] { "FinsROV_Fossen", "FinsROV_DWP2Mesh", "FinsROV" })
        {
            GameObject vehicle = GameObject.Find(name);
            if (vehicle != null && vehicle.GetComponentInChildren<VehicleRosBridge>(true) != null)
            {
                return vehicle;
            }
        }

        VehicleRosBridge[] bridges = UnityEngine.Object.FindObjectsByType<VehicleRosBridge>(
            FindObjectsInactive.Include,
            FindObjectsSortMode.None);
        return bridges.Length == 1 ? bridges[0].gameObject : null;
    }

    public static bool TryGetRuntimeThrusterAudit(out string message)
    {
        var vehicle = FindEvaluationVehicle();
        if (vehicle == null)
        {
            message = "no unambiguous FinsROV vehicle with a VehicleRosBridge exists in the active scene";
            return false;
        }

        var bridge = vehicle.GetComponent<VehicleRosBridge>() ?? vehicle.GetComponentInChildren<VehicleRosBridge>(true);
        if (bridge == null)
        {
            message = $"{vehicle.name} has no VehicleRosBridge component";
            return false;
        }

        var values = bridge.GetResolvedOrderedThrustersForTests();
        var entries = values.Select((thruster, index) =>
        {
            if (thruster == null)
            {
                return $"{index}:<missing>";
            }

            return $"{index}:{thruster.name}:target={thruster.TargetForceRequest:F3}:applied={thruster.LastForceRequest:F3}:age={thruster.TimeSinceForceRequest:F3}";
        });
        var enabledAgents = UnityEngine.Object.FindObjectsByType<Agent>(
                FindObjectsInactive.Include,
                FindObjectsSortMode.None)
            .Count(agent => agent.enabled);
        var thrusterController = vehicle.GetComponent<FinsSim.Actuators.ThrusterController>() ??
                                 vehicle.GetComponentInChildren<FinsSim.Actuators.ThrusterController>(true);
        var actuatorMode = values
            .Where(thruster => thruster != null)
            .Select(thruster => thruster.DynamicsModel.ToString())
            .Distinct()
            .ToArray();
        message = $"bridge={bridge.enabled}; mode={bridge.thrusterCommandMode}; enabledAgents={enabledAgents}; " +
                  $"responseCompression={(thrusterController != null && thrusterController.ForceResponseCompressionEnabled)}; " +
                  $"actuatorDynamics=[{string.Join(",", actuatorMode)}]; " +
                  string.Join(";", entries);
        Debug.Log($"[{nameof(FinsROSRosEvaluationSceneSetup)}] {message}", vehicle);
        return true;
    }

    private static bool ConfigureSteppedSimulationClock(
        float simulationTimeScale,
        bool ros2ControlLockstep,
        out string message)
    {
        var connections = UnityEngine.Object.FindObjectsByType<RosConnection>(
            FindObjectsInactive.Include,
            FindObjectsSortMode.None);
        if (connections.Length == 0)
        {
            message = "no RosConnection exists in the active scene; cannot configure the Unity /clock source";
            return false;
        }

        var rosConnection = connections[0];
        // The training scene stores its RosConnection disabled.  If it stays
        // inactive, the generic singleton creates a second runtime instance
        // with the default real-time settings, silently bypassing this
        // experiment's clock contract.  Activate the configured scene object
        // before Play so TimeHandler binds to this exact instance.
        if (!rosConnection.gameObject.activeSelf)
        {
            rosConnection.gameObject.SetActive(true);
        }
        var runtimeClock = rosConnection.GetComponent<FinsROSEvaluationRuntimeClock>();
        if (runtimeClock == null)
        {
            runtimeClock = rosConnection.gameObject.AddComponent<FinsROSEvaluationRuntimeClock>();
        }
        runtimeClock.Configure(simulationTimeScale, ros2ControlLockstep);
        var lockstep = rosConnection.GetComponent<FinsROS2ControlLockstep>();
        if (ros2ControlLockstep)
        {
            if (lockstep == null)
            {
                lockstep = rosConnection.gameObject.AddComponent<FinsROS2ControlLockstep>();
            }
            lockstep.Configure(simulationTimeScale);
            lockstep.enabled = true;
        }
        else if (lockstep != null)
        {
            lockstep.enabled = false;
        }
        rosConnection.RealtimeSimulation = false;
        rosConnection.SimulationSpeed = simulationTimeScale;
        rosConnection.Ros2ControlLockstep = ros2ControlLockstep;
        // A preceding manual Play session may have left a non-unit editor time
        // scale.  TimeHandler takes ownership at runtime and applies the
        // requested SimulationSpeed on every physics update.
        Time.timeScale = 1f;
        message = $"clock=stepped;/clock=true; timeScale={simulationTimeScale:F2}; ros2ControlLockstep={ros2ControlLockstep}";
        return true;
    }

    private static bool TryParseInitialPose(
        string command,
        out Vector3 position,
        out float yawDeg,
        out VehicleRosThrusterCommandMode thrusterCommandMode,
        out float simulationTimeScale,
        out bool ros2ControlLockstep,
        out string message)
    {
        position = Vector3.zero;
        yawDeg = 0f;
        thrusterCommandMode = VehicleRosThrusterCommandMode.ForceN;
        simulationTimeScale = 1f;
        ros2ControlLockstep = false;
        message = "";
        if (string.IsNullOrWhiteSpace(command))
        {
            message = "empty preparation command";
            return false;
        }

        var suffixIndex = command.IndexOf(':');
        if (suffixIndex < 0)
        {
            return true;
        }
        var payload = command.Substring(suffixIndex + 1).Trim();
        if (string.IsNullOrEmpty(payload))
        {
            return true;
        }
        var values = payload.Split(',');
        if ((values.Length != 4 && values.Length != 5 && values.Length != 6 && values.Length != 7) ||
            !float.TryParse(values[0], System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out var x) ||
            !float.TryParse(values[1], System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out var y) ||
            !float.TryParse(values[2], System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out var z) ||
            !float.TryParse(values[3], System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out yawDeg))
        {
            message = $"expected `{CommandPrefix}:x,y,z,yaw_deg[,force_n|normalized_direct][,time_scale][,ros2_control_lockstep]`, got `{command}`";
            return false;
        }
        if (!float.IsFinite(x) || !float.IsFinite(y) || !float.IsFinite(z) || !float.IsFinite(yawDeg))
        {
            message = "initial pose contains a non-finite value";
            return false;
        }
        if (values.Length >= 5)
        {
            var requestedMode = values[4].Trim().ToLowerInvariant();
            if (requestedMode == "force_n" || requestedMode == "force")
            {
                thrusterCommandMode = VehicleRosThrusterCommandMode.ForceN;
            }
            else if (requestedMode == "normalized_direct" || requestedMode == "normalized")
            {
                thrusterCommandMode = VehicleRosThrusterCommandMode.NormalizedDirect;
            }
            else
            {
                message = $"unsupported Unity thruster command mode `{values[4]}`";
                return false;
            }
        }
        if (values.Length >= 6 &&
            (!float.TryParse(values[5], System.Globalization.NumberStyles.Float, System.Globalization.CultureInfo.InvariantCulture, out simulationTimeScale) ||
             !float.IsFinite(simulationTimeScale) || simulationTimeScale <= 0f))
        {
            message = $"simulation time scale must be a finite positive float, got `{values[5]}`";
            return false;
        }
        if (values.Length == 7)
        {
            var requestedClockMode = values[6].Trim().ToLowerInvariant();
            if (requestedClockMode == "ros2_control_lockstep" || requestedClockMode == "lockstep")
            {
                ros2ControlLockstep = true;
            }
            else if (requestedClockMode == "free_running" || requestedClockMode == "stepped")
            {
                ros2ControlLockstep = false;
            }
            else
            {
                message = $"unsupported simulation clock mode `{values[6]}`";
                return false;
            }
        }
        position = new Vector3(x, y, z);
        return true;
    }
}

/// <summary>
/// Installs the Fossen reset probe on the active scene instance only. It never
/// writes to the FinsROV prefab asset.
/// </summary>
public static class FinsROVFossenDiagnosticsSceneSetup
{
    // Kept alongside the command listener so an external scene-only install is available.
    public static void InstallInActiveScene()
    {
        Scene scene = SceneManager.GetActiveScene();
        GameObject vehicle = GameObject.Find("FinsROV_Fossen") ?? GameObject.Find("FinsROV");
        if (vehicle == null)
        {
            Debug.LogError("[FinsROVFossenDiagnosticsSceneSetup] No FinsROV/FinsROV_Fossen object exists in the active scene.");
            return;
        }

        if (vehicle.GetComponent<HydrodynamicsController>() == null)
        {
            Debug.LogError("[FinsROVFossenDiagnosticsSceneSetup] Target vehicle has no HydrodynamicsController.", vehicle);
            return;
        }

        Type probeType = AppDomain.CurrentDomain.GetAssemblies()
            .Select(assembly => assembly.GetType("FinsROVFossenResetDiagnosticsProbe"))
            .FirstOrDefault(type => type != null);
        if (probeType == null)
        {
            Debug.LogError("[FinsROVFossenDiagnosticsSceneSetup] FinsROVFossenResetDiagnosticsProbe has not been compiled yet.");
            return;
        }

        if (vehicle.GetComponent(probeType) == null)
        {
            // The runtime probe may be imported in the same editor refresh as this
            // controller, so resolve it by name instead of creating a compile-time
            // Editor-to-runtime dependency.
            Undo.AddComponent(vehicle, probeType);
        }

        EditorSceneManager.MarkSceneDirty(scene);
        EditorSceneManager.SaveScene(scene);
        Debug.Log($"[FinsROVFossenDiagnosticsSceneSetup] Installed probe on `{vehicle.name}` in `{scene.path}`.", vehicle);
    }
}

/// <summary>Editor-only logging switch for comparing received and applied chaser actions.</summary>
public static class ThreeChaseOneActionAuditSceneSetup
{
    public static void SetEnabled(bool enabled)
    {
        int count = 0;
        foreach (ChaserAgent agent in UnityEngine.Object.FindObjectsByType<ChaserAgent>(
                     FindObjectsInactive.Include,
                     FindObjectsSortMode.None))
        {
            agent.SetActionAuditEnabled(enabled);
            count++;
        }

        Debug.Log($"[ThreeChaseOneActionAudit] enabled={enabled}; chasers={count}.");
    }
}

/// <summary>
/// Editor-only fixture for a single-vehicle ROS force audit in 3Chase1.
/// It changes only the active scene instance and intentionally never saves it.
/// </summary>
public static class ThreeChaseOneRosIsolationSceneSetup
{
    private const string HerderName = "FinsROV_Fossen_Herder";

    public static void InstallHerderIsolation()
    {
        if (EditorApplication.isPlayingOrWillChangePlaymode)
        {
            Debug.LogError("[ThreeChaseOneRosIsolation] Stop Play Mode before installing the isolation fixture.");
            return;
        }

        GameObject vehicle = GameObject.Find(HerderName);
        if (vehicle == null)
        {
            Debug.LogError($"[ThreeChaseOneRosIsolation] Cannot find `{HerderName}` in the active scene.");
            return;
        }

        ChaserAgent agent = vehicle.GetComponent<ChaserAgent>();
        if (agent == null)
        {
            Debug.LogError("[ThreeChaseOneRosIsolation] Herder has no ChaserAgent.", vehicle);
            return;
        }

        if (vehicle.GetComponent<Rigidbody>() == null || vehicle.GetComponent<HydrodynamicsController>() == null)
        {
            Debug.LogError("[ThreeChaseOneRosIsolation] Herder is missing Rigidbody or HydrodynamicsController.", vehicle);
            return;
        }

        // Freeze every scene agent. The two netters and prey can otherwise move
        // the shared Obi net and contaminate a single-Herder propulsion test.
        int disabledAgentCount = 0;
        foreach (Agent sceneAgent in UnityEngine.Object.FindObjectsByType<Agent>(
                     FindObjectsInactive.Include,
                     FindObjectsSortMode.None))
        {
            if (sceneAgent.enabled)
            {
                sceneAgent.enabled = false;
                disabledAgentCount++;
            }
        }

        VehicleRosBridge bridge = vehicle.GetComponent<VehicleRosBridge>();
        if (bridge == null)
        {
            bridge = Undo.AddComponent<VehicleRosBridge>(vehicle);
        }

        bridge.bridgeName = "finsrov";
        bridge.thrusterTopic = "/finsrov/thrusters_out";
        bridge.poseTopic = "/finsrov/pose";
        bridge.imuTopic = "/finsrov/imu_link";
        bridge.dvlTopic = "/finsrov/dvl_link";
        bridge.depthTopic = "/finsrov/depth_link";
        bridge.controllerPoseTopic = "/finsrov/controller/pose";
        bridge.controllerImuTopic = "/finsrov/controller/imu";
        bridge.controllerDvlTopic = "/finsrov/controller/dvl";
        bridge.controllerDepthTopic = "/finsrov/controller/depth";
        bridge.subscribeThrusters = true;
        bridge.subscribeReset = false;
        bridge.publishPose = true;
        bridge.publishImu = true;
        bridge.publishDvl = true;
        bridge.publishDepth = true;
        bridge.publishThrusterAppliedWrench = true;
        bridge.thrusterAppliedWrenchTopic = "/finsrov/debug/thruster_applied_wrench";
        bridge.thrusterCommandMode = VehicleRosThrusterCommandMode.ForceN;
        bridge.autoResolveThrustersFromChildren = true;
        bridge.reapplyLastThrusterCommandInFixedUpdate = true;
        bridge.disableManualControllerOnEnable = true;
        bridge.zeroThrustersWhenDisabled = true;
        bridge.logThrusterResolution = true;
        bridge.enabled = true;

        EditorUtility.SetDirty(vehicle);
        Debug.Log(
            $"[ThreeChaseOneRosIsolation] Installed on Herder. Disabled {disabledAgentCount} ML-Agents; " +
            "ROS force input is `/finsrov/thrusters_out` (adapter exposes `/sim/finsrov/thrusters_out`).",
            vehicle
        );
    }

    public static void RemoveHerderIsolation()
    {
        if (EditorApplication.isPlayingOrWillChangePlaymode)
        {
            Debug.LogError("[ThreeChaseOneRosIsolation] Stop Play Mode before clearing the isolation fixture.");
            return;
        }

        GameObject vehicle = GameObject.Find(HerderName);
        if (vehicle == null)
        {
            Debug.LogWarning($"[ThreeChaseOneRosIsolation] `{HerderName}` no longer exists in the active scene.");
            return;
        }

        VehicleRosBridge bridge = vehicle.GetComponent<VehicleRosBridge>();
        if (bridge != null)
        {
            Undo.DestroyObjectImmediate(bridge);
        }

        foreach (Agent sceneAgent in UnityEngine.Object.FindObjectsByType<Agent>(
                     FindObjectsInactive.Include,
                     FindObjectsSortMode.None))
        {
            sceneAgent.enabled = true;
        }

        EditorUtility.SetDirty(vehicle);
        Debug.Log("[ThreeChaseOneRosIsolation] Removed from Herder; ChaserAgent restored.", vehicle);
    }
}
