using System;
using System.Collections.Generic;
using System.IO;
using UnityEditor;
using UnityEditor.Build;
using UnityEditor.Build.Reporting;
using UnityEngine;

/// <summary>
/// The sole command-line build entry point.  A build is addressed by a stable
/// task-family/variant id, rather than an arbitrary C# method name.
/// </summary>
public static class FinsSimTaskBuild
{
    public readonly struct Definition
    {
        public Definition(string id, string scenePath, bool server)
        {
            Id = id;
            ScenePath = scenePath;
            Server = server;
        }

        public string Id { get; }
        public string ScenePath { get; }
        public bool Server { get; }
    }

    static readonly Dictionary<string, Definition> Definitions = new(StringComparer.Ordinal)
    {
        ["pose-control/new-server"] = new("pose-control/new-server", "Assets/FinsSimUnity/Tasks/PoseControl/Scenes/ControlForPosition_new.unity", true),
        ["pose-control/new-player"] = new("pose-control/new-player", "Assets/FinsSimUnity/Tasks/PoseControl/Scenes/ControlForPosition_new.unity", false),
        ["pose-control/smooth-near-target-manual-server"] = new("pose-control/smooth-near-target-manual-server", "Assets/FinsSimUnity/Tasks/PoseControl/Scenes/ControlForPosition_smooth_near_target_manual.unity", true),
        ["pose-control/fossen-server"] = new("pose-control/fossen-server", "Assets/FinsSimUnity/Tasks/PoseControl/Scenes/ControlForPosition_Fossen.unity", true),
        ["pose-control/yaw-torque-dr-server"] = new("pose-control/yaw-torque-dr-server", "Assets/FinsSimUnity/Tasks/PoseControl/Scenes/ControlForPosition_YawTorque_DR.unity", true),
        ["hold-position/fossen-server"] = new("hold-position/fossen-server", "Assets/FinsSimUnity/Tasks/HoldPosition/Scenes/HoldForPosition_Fossen.unity", true),
        ["hold-position/fossen-parallel-server"] = new("hold-position/fossen-parallel-server", "Assets/FinsSimUnity/Tasks/HoldPosition/Scenes/HoldForPosition_Fossen_Parallel_30s.unity", true),
        ["hold-position/fossen-parallel-normalized-max-force-server"] = new("hold-position/fossen-parallel-normalized-max-force-server", "Assets/FinsSimUnity/Tasks/HoldPosition/Scenes/HoldForPosition_Fossen_Parallel_30s_NormalizedMaxForce.unity", true),
        ["trajectory-tracking/fossen-server"] = new("trajectory-tracking/fossen-server", "Assets/FinsSimUnity/Tasks/TrajectoryTracking/Scenes/TrajectoryTracking_Fossen_Parallel_30s.unity", true),
        ["chase/one-headless"] = new("chase/one-headless", "Assets/FinsSimUnity/Tasks/Chase/OneChaseOne/Scenes/1Chase1.unity", true),
        ["chase/one-visual"] = new("chase/one-visual", "Assets/FinsSimUnity/Tasks/Chase/OneChaseOne/Scenes/1Chase1.unity", false),
        ["chase/three-headless"] = new("chase/three-headless", "Assets/FinsSimUnity/Tasks/Chase/ThreeChaseOne/Scenes/3Chase1.unity", true),
        ["net-capture/headless"] = new("net-capture/headless", "Assets/FinsSimUnity/Tasks/NetCapture/Scenes/TriNetCapture.unity", true),
        ["cloth/triangle-tow"] = new("cloth/triangle-tow", "Assets/FinsSimUnity/Tasks/Cloth/Scenes/OpenSourceCloth/HabradorXPBDTriangleTow.unity", false),
        ["platform/example"] = new("platform/example", "Assets/FinsSimUnity/Tasks/PlatformExamples/Scenes/ExampleScene.unity", false),
    };

    public static IEnumerable<Definition> All => Definitions.Values;

    public static void BuildFromCommandLine()
    {
        string taskId = ReadArgument("-finsSimTask");
        if (string.IsNullOrWhiteSpace(taskId))
        {
            throw new ArgumentException("Missing -finsSimTask <task-family/variant>. Use FinsSimTaskBuild.ListTaskIds() to inspect the catalog.");
        }

        Build(taskId);
    }

    public static void ListTaskIds()
    {
        foreach (Definition definition in Definitions.Values)
        {
            Debug.Log($"[FinsSimTaskBuild] {definition.Id} -> {definition.ScenePath} ({(definition.Server ? "server" : "player")})");
        }
    }

    public static void Build(string taskId)
    {
        if (!Definitions.TryGetValue(taskId, out Definition definition))
        {
            throw new ArgumentException($"Unknown FinsSim task '{taskId}'.");
        }

        if (AssetDatabase.LoadAssetAtPath<SceneAsset>(definition.ScenePath) == null)
        {
            throw new FileNotFoundException($"Task scene not found: {definition.ScenePath}");
        }

        string outputPath = GetOutputPath(definition);
        // These two variants have a deterministic build-worktree preparation
        // step (copying and stripping a generated headless scene). Keep that
        // preparation behind the catalog boundary rather than exposing it as a
        // command-line C# method.
        if (taskId == "chase/three-headless")
        {
            RLChaseLinuxBuild.Build3Chase1HeadlessServer(outputPath);
            return;
        }
        if (taskId == "net-capture/headless")
        {
            RLChaseLinuxBuild.BuildTriNetCaptureHeadlessServer(outputPath);
            return;
        }

        EditorUserBuildSettings.SwitchActiveBuildTarget(BuildTargetGroup.Standalone, BuildTarget.StandaloneLinux64);
        BuildReport report = BuildPipeline.BuildPlayer(new BuildPlayerOptions
        {
            scenes = new[] { definition.ScenePath },
            locationPathName = outputPath,
            target = BuildTarget.StandaloneLinux64,
            subtarget = definition.Server ? (int)StandaloneBuildSubtarget.Server : 0,
            options = BuildOptions.None,
        });

        if (report.summary.result != BuildResult.Succeeded)
        {
            throw new InvalidOperationException($"Task build failed: {taskId}; result={report.summary.result}");
        }

        Debug.Log($"[FinsSimTaskBuild] completed task={taskId}, output={outputPath}");
    }

    private static string GetOutputPath(Definition definition)
    {
        string projectRoot = Directory.GetParent(Application.dataPath)?.FullName
            ?? throw new InvalidOperationException("Cannot resolve Unity project root.");
        string outputDirectory = Path.Combine(projectRoot, "artifacts", "unity_builds", definition.Id);
        Directory.CreateDirectory(outputDirectory);
        return Path.Combine(outputDirectory, "FinsSimUnity.x86_64");
    }

    static string ReadArgument(string name)
    {
        string[] arguments = Environment.GetCommandLineArgs();
        for (int index = 0; index + 1 < arguments.Length; index++)
        {
            if (string.Equals(arguments[index], name, StringComparison.Ordinal))
            {
                return arguments[index + 1];
            }
        }

        return null;
    }
}
