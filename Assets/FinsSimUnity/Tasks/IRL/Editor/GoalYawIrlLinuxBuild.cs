using System;
using System.IO;
using UnityEditor;
using UnityEditor.Build;
using UnityEditor.Build.Reporting;

/// <summary>Builds the generated GoalYaw AIRL/GAIL Fossen scene for Linux.</summary>
public static class GoalYawIrlLinuxBuild
{
    public const string ServerOutputPath =
        "/home/fins/UnderwaterSim/Code/FinsSimUnity/artifacts/unity_builds/irl/linux/GoalYawIRL_Fossen_10Hz_Server/GoalYawIRL.x86_64";
    public const string PlayerOutputPath =
        "/home/fins/UnderwaterSim/Code/FinsSimUnity/artifacts/unity_builds/irl/linux/GoalYawIRL_Fossen_10Hz_Player/GoalYawIRL.x86_64";

    public static void BuildGoalYawIrlFossenServer()
    {
        BuildGoalYawIrlFossen(ServerOutputPath, StandaloneBuildSubtarget.Server);
    }

    public static void BuildGoalYawIrlFossenPlayer()
    {
        BuildGoalYawIrlFossen(PlayerOutputPath, StandaloneBuildSubtarget.Player);
    }

    static void BuildGoalYawIrlFossen(string outputPath, StandaloneBuildSubtarget subtarget)
    {
        GoalYawIrlSceneBuilder.EnsurePrepared();
        string outputDirectory = Path.GetDirectoryName(outputPath);
        if (string.IsNullOrEmpty(outputDirectory))
        {
            throw new InvalidOperationException($"Invalid build output {outputPath}.");
        }
        if (Directory.Exists(outputDirectory))
        {
            Directory.Delete(outputDirectory, true);
        }
        Directory.CreateDirectory(outputDirectory);

        EditorUserBuildSettings.SwitchActiveBuildTarget(BuildTargetGroup.Standalone, BuildTarget.StandaloneLinux64);
        BuildReport report = BuildPipeline.BuildPlayer(new BuildPlayerOptions
        {
            scenes = new[] { GoalYawIrlSceneBuilder.GeneratedScenePath },
            locationPathName = outputPath,
            target = BuildTarget.StandaloneLinux64,
            subtarget = (int)subtarget,
            options = BuildOptions.None,
        });
        if (report.summary.result != BuildResult.Succeeded)
        {
            throw new InvalidOperationException($"GoalYawIRL build failed: {report.summary.result}.");
        }
    }
}
