using System;
using System.IO;
using UnityEditor;
using UnityEngine;

public static class FinsSimBuildDaemon
{
    private const double PollIntervalSeconds = 0.5;

    private static string requestPath;
    private static string resultPath;
    private static string readyPath;
    private static double nextPollTime;
    private static bool running;
    private static bool readyWritten;
    private static BuildRequest pendingRequest;

    [Serializable]
    private sealed class BuildRequest
    {
        public string id;
        public string task_id;
    }

    [Serializable]
    private sealed class BuildResult
    {
        public string id;
        public string task_id;
        public bool success;
        public string message;
        public string started_at;
        public string finished_at;
    }

    public static void Start()
    {
        requestPath = GetEnvOrDefault("FINSIM_UNITY_BUILD_REQUEST", "/tmp/finsim_unity_build_request.json");
        resultPath = GetEnvOrDefault("FINSIM_UNITY_BUILD_RESULT", "/tmp/finsim_unity_build_result.json");
        readyPath = GetEnvOrDefault("FINSIM_UNITY_BUILD_READY", "/tmp/finsim_unity_build_ready");

        Directory.CreateDirectory(Path.GetDirectoryName(requestPath));
        Directory.CreateDirectory(Path.GetDirectoryName(resultPath));
        Directory.CreateDirectory(Path.GetDirectoryName(readyPath));

        if (!running)
        {
            EditorApplication.update += Poll;
            running = true;
        }

        Debug.Log(
            $"[FinsSimBuildDaemon] Starting. request={requestPath}, result={resultPath}, ready={readyPath}");
    }

    public static void Stop()
    {
        Debug.Log("[FinsSimBuildDaemon] Stop requested.");
        EditorApplication.Exit(0);
    }

    public static void Ping()
    {
        Debug.Log("[FinsSimBuildDaemon] Ping.");
    }

    private static void Poll()
    {
        if (EditorApplication.timeSinceStartup < nextPollTime)
        {
            return;
        }

        nextPollTime = EditorApplication.timeSinceStartup + PollIntervalSeconds;
        if (!IsEditorIdle())
        {
            return;
        }

        if (!readyWritten)
        {
            File.WriteAllText(readyPath, DateTime.UtcNow.ToString("O"));
            readyWritten = true;
            Debug.Log("[FinsSimBuildDaemon] Editor idle; daemon is ready for build requests.");
        }

        if (pendingRequest != null)
        {
            BuildRequest request = pendingRequest;
            pendingRequest = null;
            EditorApplication.delayCall += () => TryRunPendingRequest(request);
            return;
        }

        if (string.IsNullOrWhiteSpace(requestPath) || !File.Exists(requestPath))
        {
            return;
        }

        string json;
        try
        {
            json = File.ReadAllText(requestPath);
            File.Delete(requestPath);
        }
        catch (Exception exc)
        {
            Debug.LogWarning($"[FinsSimBuildDaemon] Failed to consume request: {exc.Message}");
            return;
        }

        BuildRequest parsedRequest;
        try
        {
            parsedRequest = JsonUtility.FromJson<BuildRequest>(json);
        }
        catch (Exception exc)
        {
            WriteResult(new BuildResult
            {
                id = "",
                task_id = "",
                success = false,
                message = $"Invalid request JSON: {exc.Message}",
                started_at = DateTime.UtcNow.ToString("O"),
                finished_at = DateTime.UtcNow.ToString("O"),
            });
            return;
        }

        pendingRequest = parsedRequest;
    }

    private static void TryRunPendingRequest(BuildRequest request)
    {
        if (!IsEditorIdle())
        {
            EditorApplication.delayCall += () => TryRunPendingRequest(request);
            return;
        }

        RunRequest(request);
    }

    private static void RunRequest(BuildRequest request)
    {
        string startedAt = DateTime.UtcNow.ToString("O");
        if (request == null || string.IsNullOrWhiteSpace(request.task_id))
        {
            WriteResult(new BuildResult
            {
                id = request?.id ?? "",
                task_id = request?.task_id ?? "",
                success = false,
                message = "Request task_id is empty.",
                started_at = startedAt,
                finished_at = DateTime.UtcNow.ToString("O"),
            });
            return;
        }

        Debug.Log($"[FinsSimBuildDaemon] Build requested: id={request.id}, task={request.task_id}");
        try
        {
            RefreshAssetDatabase();
            FinsSimTaskBuild.Build(request.task_id);
            WriteResult(new BuildResult
            {
                id = request.id,
                task_id = request.task_id,
                success = true,
                message = "Build completed.",
                started_at = startedAt,
                finished_at = DateTime.UtcNow.ToString("O"),
            });
        }
        catch (Exception exc)
        {
            WriteResult(new BuildResult
            {
                id = request.id,
                task_id = request.task_id,
                success = false,
                message = exc.ToString(),
                started_at = startedAt,
                finished_at = DateTime.UtcNow.ToString("O"),
            });
        }
    }

    private static void RefreshAssetDatabase()
    {
        Debug.Log("[FinsSimBuildDaemon] Refreshing AssetDatabase before build request.");
        AssetDatabase.Refresh(ImportAssetOptions.ForceSynchronousImport);
    }

    private static void WriteResult(BuildResult result)
    {
        string tempPath = resultPath + ".tmp";
        File.WriteAllText(tempPath, JsonUtility.ToJson(result, true));
        if (File.Exists(resultPath))
        {
            File.Delete(resultPath);
        }

        File.Move(tempPath, resultPath);
        Debug.Log(
            $"[FinsSimBuildDaemon] Build result: id={result.id}, task={result.task_id}, success={result.success}");
    }

    private static string GetEnvOrDefault(string key, string fallback)
    {
        string value = Environment.GetEnvironmentVariable(key);
        return string.IsNullOrWhiteSpace(value) ? fallback : value;
    }

    private static bool IsEditorIdle()
    {
        return !EditorApplication.isCompiling && !EditorApplication.isUpdating;
    }
}
