using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using Unity.Profiling;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

[InitializeOnLoad]
public static class ThreeChaseOneProfilerRunner
{
    const string DefaultScenePath = "Assets/FinsSimUnity/Tasks/Chase/ThreeChaseOne/Scenes/3Chase1_new.unity";
    const string DefaultOutputDir = "Temp/3chase1_new_profile";
    const string SessionActiveKey = "ThreeChaseOneProfilerRunner.Active";
    const string SessionExitingKey = "ThreeChaseOneProfilerRunner.Exiting";
    const string SessionSceneKey = "ThreeChaseOneProfilerRunner.Scene";
    const string SessionOutputDirKey = "ThreeChaseOneProfilerRunner.OutputDir";
    const string SessionDurationKey = "ThreeChaseOneProfilerRunner.Duration";

    static bool waitingForPlayMode;
    static bool exitingAfterPlayMode;
    static string scenePath;
    static string outputDir;
    static float durationSec;

    static ThreeChaseOneProfilerRunner()
    {
        EditorApplication.update -= Update;
        EditorApplication.update += Update;
        EditorApplication.playModeStateChanged -= OnPlayModeStateChanged;
        EditorApplication.playModeStateChanged += OnPlayModeStateChanged;
    }

    public static void Run3Chase1New()
    {
        scenePath = GetArg("-profileScene", DefaultScenePath);
        outputDir = GetArg("-profileOutputDir", DefaultOutputDir);
        durationSec = ParseFloatArg("-profileDuration", 8f);
        StoreSession();

        Debug.Log(
            $"[ThreeChaseOneProfilerRunner] Starting profiler run: scene={scenePath}, " +
            $"duration={durationSec.ToString("F2", CultureInfo.InvariantCulture)}s, outputDir={outputDir}");

        EditorSceneManager.OpenScene(scenePath);
        waitingForPlayMode = true;
        EditorApplication.EnterPlaymode();
    }

    static void OnPlayModeStateChanged(PlayModeStateChange state)
    {
        if (state == PlayModeStateChange.EnteredPlayMode && SessionState.GetBool(SessionActiveKey, false))
        {
            waitingForPlayMode = false;
            LoadSession();
            StartRuntimeProbe();
        }
        else if (state == PlayModeStateChange.EnteredEditMode && SessionState.GetBool(SessionExitingKey, false))
        {
            CleanupAndExit(0);
        }
    }

    static void Update()
    {
        if (waitingForPlayMode && EditorApplication.isPlaying)
        {
            waitingForPlayMode = false;
            StartRuntimeProbe();
            return;
        }

        if (exitingAfterPlayMode && !EditorApplication.isPlaying)
        {
            CleanupAndExit(0);
        }
    }

    static void StartRuntimeProbe()
    {
        LoadSession();
        var host = new GameObject("ThreeChaseOneProfilerProbe");
        UnityEngine.Object.DontDestroyOnLoad(host);
        var probe = host.AddComponent<ThreeChaseOneProfilerProbe>();
        probe.Configure(outputDir, durationSec, () =>
        {
            exitingAfterPlayMode = true;
            SessionState.SetBool(SessionExitingKey, true);
            EditorApplication.ExitPlaymode();
        });
    }

    static void CleanupAndExit(int exitCode)
    {
        ClearSession();
        EditorApplication.update -= Update;
        EditorApplication.playModeStateChanged -= OnPlayModeStateChanged;
        EditorApplication.Exit(exitCode);
    }

    static string GetArg(string name, string defaultValue)
    {
        string[] args = Environment.GetCommandLineArgs();
        for (int i = 0; i < args.Length; i++)
        {
            if (args[i] == name && i + 1 < args.Length)
            {
                return args[i + 1];
            }

            string prefix = name + "=";
            if (args[i].StartsWith(prefix, StringComparison.Ordinal))
            {
                return args[i].Substring(prefix.Length);
            }
        }

        return defaultValue;
    }

    static float ParseFloatArg(string name, float defaultValue)
    {
        string raw = GetArg(name, defaultValue.ToString(CultureInfo.InvariantCulture));
        return float.TryParse(raw, NumberStyles.Float, CultureInfo.InvariantCulture, out float value)
            ? value
            : defaultValue;
    }

    static void StoreSession()
    {
        SessionState.SetBool(SessionActiveKey, true);
        SessionState.SetBool(SessionExitingKey, false);
        SessionState.SetString(SessionSceneKey, scenePath);
        SessionState.SetString(SessionOutputDirKey, outputDir);
        SessionState.SetFloat(SessionDurationKey, durationSec);
    }

    static void LoadSession()
    {
        scenePath = SessionState.GetString(SessionSceneKey, scenePath ?? DefaultScenePath);
        outputDir = SessionState.GetString(SessionOutputDirKey, outputDir ?? DefaultOutputDir);
        durationSec = SessionState.GetFloat(SessionDurationKey, durationSec > 0f ? durationSec : 8f);
        exitingAfterPlayMode = SessionState.GetBool(SessionExitingKey, false);
    }

    static void ClearSession()
    {
        SessionState.EraseBool(SessionActiveKey);
        SessionState.EraseBool(SessionExitingKey);
        SessionState.EraseString(SessionSceneKey);
        SessionState.EraseString(SessionOutputDirKey);
        SessionState.EraseFloat(SessionDurationKey);
    }
}

public sealed class ThreeChaseOneProfilerProbe : MonoBehaviour
{
    struct Sample
    {
        public float Time;
        public int Frame;
        public int FixedFrame;
        public long MainThreadNs;
        public long RenderThreadNs;
        public long BehaviourUpdateNs;
        public long FixedUpdateNs;
        public long PhysicsSimulateNs;
        public long CameraRenderNs;
        public long GcAllocBytes;
        public long GcUsedBytes;
        public long TotalUsedBytes;
        public long SystemUsedBytes;
    }

    readonly List<ProfilerRecorder> recorders = new List<ProfilerRecorder>();
    readonly List<Sample> samples = new List<Sample>(2048);
    readonly Dictionary<string, ProfilerRecorder> recorderByName = new Dictionary<string, ProfilerRecorder>(StringComparer.Ordinal);
    readonly Dictionary<string, List<long>> markerSamples = new Dictionary<string, List<long>>(StringComparer.Ordinal);
    readonly Dictionary<string, List<double>> runtimeMarkerFrameTotalMs = new Dictionary<string, List<double>>(StringComparer.Ordinal);
    readonly Dictionary<string, List<int>> runtimeMarkerFrameCounts = new Dictionary<string, List<int>>(StringComparer.Ordinal);
    readonly Dictionary<string, string> unavailable = new Dictionary<string, string>(StringComparer.Ordinal);
    static readonly string[] InstrumentedMarkers =
    {
        "FinsSim.ChaserAgent.FixedUpdate",
        "FinsSim.CatchAreaManager.FixedUpdate",
        "FinsSim.CatchAreaManager.EnsureNetActorCollisionFilters",
        "FinsSim.CatchAreaManager.UpdateNetSurfaceCaptureState",
        "FinsSim.CatchAreaManager.TryResolveCapture",
        "FinsSim.NetHydrodynamicProxy.FixedUpdate",
        "FinsSim.HydrodynamicsController.FixedUpdate",
        "FinsSim.Thruster.FixedUpdate",
        "FinsSim.DWP2.WaterObject.FixedUpdate",
        "FinsSim.DWP2.WaterObject.GetWaterData",
        "FinsSim.DWP2.WaterObject.TickWaterObject",
        "FinsSim.ObiSolver.FixedUpdate",
        "FinsSim.ObiSolver.LateUpdate",
    };

    string outputDir;
    float durationSec;
    float startRealtime;
    int fixedFrameCount;
    Action onFinished;

    public void Configure(string profilerOutputDir, float profilerDurationSec, Action finished)
    {
        outputDir = profilerOutputDir;
        durationSec = Mathf.Max(1f, profilerDurationSec);
        onFinished = finished;
    }

    void Start()
    {
        FinsSimRuntimeProfiler.Reset();
        startRealtime = Time.realtimeSinceStartup;
        StartRecorder("Main Thread", ProfilerCategory.Internal);
        StartRecorder("Render Thread", ProfilerCategory.Internal);
        StartRecorder("BehaviourUpdate", ProfilerCategory.Scripts);
        StartRecorder("FixedUpdate.ScriptRunBehaviourFixedUpdate", ProfilerCategory.Scripts);
        StartRecorder("Physics.Simulate", ProfilerCategory.Physics);
        StartRecorder("Camera.Render", ProfilerCategory.Render);
        StartRecorder("GC Allocated In Frame", ProfilerCategory.Memory);
        StartRecorder("GC Used Memory", ProfilerCategory.Memory);
        StartRecorder("Total Used Memory", ProfilerCategory.Memory);
        StartRecorder("System Used Memory", ProfilerCategory.Memory);
        foreach (string markerName in InstrumentedMarkers)
        {
            StartRecorder(markerName, ProfilerCategory.Scripts);
            markerSamples[markerName] = new List<long>(2048);
            runtimeMarkerFrameTotalMs[markerName] = new List<double>(2048);
            runtimeMarkerFrameCounts[markerName] = new List<int>(2048);
        }

        Debug.Log(
            $"[ThreeChaseOneProfilerProbe] Recording for {durationSec.ToString("F2", CultureInfo.InvariantCulture)}s. " +
            $"activeRecorders={recorders.Count}, unavailable={unavailable.Count}");
    }

    void FixedUpdate()
    {
        fixedFrameCount++;
    }

    void Update()
    {
        samples.Add(new Sample
        {
            Time = Time.realtimeSinceStartup - startRealtime,
            Frame = Time.frameCount,
            FixedFrame = fixedFrameCount,
            MainThreadNs = LastValue("Main Thread"),
            RenderThreadNs = LastValue("Render Thread"),
            BehaviourUpdateNs = LastValue("BehaviourUpdate"),
            FixedUpdateNs = LastValue("FixedUpdate.ScriptRunBehaviourFixedUpdate"),
            PhysicsSimulateNs = LastValue("Physics.Simulate"),
            CameraRenderNs = LastValue("Camera.Render"),
            GcAllocBytes = LastValue("GC Allocated In Frame"),
            GcUsedBytes = LastValue("GC Used Memory"),
            TotalUsedBytes = LastValue("Total Used Memory"),
            SystemUsedBytes = LastValue("System Used Memory"),
        });

        foreach (string markerName in InstrumentedMarkers)
        {
            markerSamples[markerName].Add(LastValue(markerName));
        }

        foreach (FinsSimRuntimeProfiler.MarkerStats markerStats in FinsSimRuntimeProfiler.SnapshotAndResetFrame())
        {
            if (!runtimeMarkerFrameTotalMs.TryGetValue(markerStats.Name, out List<double> totalMs))
            {
                totalMs = new List<double>(2048);
                runtimeMarkerFrameTotalMs.Add(markerStats.Name, totalMs);
            }

            if (!runtimeMarkerFrameCounts.TryGetValue(markerStats.Name, out List<int> counts))
            {
                counts = new List<int>(2048);
                runtimeMarkerFrameCounts.Add(markerStats.Name, counts);
            }

            totalMs.Add(markerStats.TotalMs);
            counts.Add(markerStats.Count);
        }

        if (Time.realtimeSinceStartup - startRealtime >= durationSec)
        {
            Finish();
        }
    }

    void OnDestroy()
    {
        DisposeRecorders();
    }

    void StartRecorder(string markerName, ProfilerCategory category)
    {
        try
        {
            ProfilerRecorder recorder = ProfilerRecorder.StartNew(category, markerName, 1);
            if (recorder.Valid)
            {
                recorders.Add(recorder);
                recorderByName[markerName] = recorder;
                return;
            }

            recorder.Dispose();
            unavailable[markerName] = "invalid";
        }
        catch (Exception ex)
        {
            unavailable[markerName] = ex.GetType().Name + ": " + ex.Message;
        }
    }

    long LastValue(string markerName)
    {
        return recorderByName.TryGetValue(markerName, out ProfilerRecorder recorder) && recorder.Valid
            ? recorder.LastValue
            : 0L;
    }

    void Finish()
    {
        WriteOutputs();
        DisposeRecorders();
        enabled = false;
        onFinished?.Invoke();
    }

    void DisposeRecorders()
    {
        for (int i = 0; i < recorders.Count; i++)
        {
            if (recorders[i].Valid)
            {
                recorders[i].Dispose();
            }
        }

        recorders.Clear();
        recorderByName.Clear();
    }

    void WriteOutputs()
    {
        string fullOutputDir = Path.GetFullPath(outputDir);
        Directory.CreateDirectory(fullOutputDir);
        string csvPath = Path.Combine(fullOutputDir, "samples.csv");
        string summaryPath = Path.Combine(fullOutputDir, "summary.txt");

        var csv = new StringBuilder(samples.Count * 256);
        csv.AppendLine("time_s,frame,fixed_frame,main_thread_ms,render_thread_ms,behaviour_update_ms,fixed_update_ms,physics_simulate_ms,camera_render_ms,gc_alloc_bytes,gc_used_mib,total_used_mib,system_used_mib");
        foreach (Sample sample in samples)
        {
            csv.AppendFormat(
                CultureInfo.InvariantCulture,
                "{0:F4},{1},{2},{3:F4},{4:F4},{5:F4},{6:F4},{7:F4},{8:F4},{9},{10:F2},{11:F2},{12:F2}\n",
                sample.Time,
                sample.Frame,
                sample.FixedFrame,
                NsToMs(sample.MainThreadNs),
                NsToMs(sample.RenderThreadNs),
                NsToMs(sample.BehaviourUpdateNs),
                NsToMs(sample.FixedUpdateNs),
                NsToMs(sample.PhysicsSimulateNs),
                NsToMs(sample.CameraRenderNs),
                sample.GcAllocBytes,
                BytesToMiB(sample.GcUsedBytes),
                BytesToMiB(sample.TotalUsedBytes),
                BytesToMiB(sample.SystemUsedBytes));
        }

        File.WriteAllText(csvPath, csv.ToString());
        File.WriteAllText(summaryPath, BuildSummary());
        Debug.Log($"[ThreeChaseOneProfilerProbe] Wrote {csvPath} and {summaryPath}");
    }

    string BuildSummary()
    {
        var summary = new StringBuilder();
        summary.AppendLine("ThreeChaseOneProfiler summary");
        summary.AppendLine($"samples={samples.Count}");
        summary.AppendLine($"duration_s={(samples.Count > 0 ? samples[samples.Count - 1].Time : 0f).ToString("F3", CultureInfo.InvariantCulture)}");
        summary.AppendLine($"frames={(samples.Count > 0 ? samples[samples.Count - 1].Frame - samples[0].Frame + 1 : 0)}");
        summary.AppendLine($"fixed_frames={fixedFrameCount}");
        summary.AppendLine($"approx_update_fps={ComputeUpdateFps().ToString("F2", CultureInfo.InvariantCulture)}");
        summary.AppendLine($"approx_fixed_hz={ComputeFixedHz().ToString("F2", CultureInfo.InvariantCulture)}");
        summary.AppendLine();
        AppendNsStats(summary, "main_thread_ms", s => s.MainThreadNs);
        AppendNsStats(summary, "render_thread_ms", s => s.RenderThreadNs);
        AppendNsStats(summary, "behaviour_update_ms", s => s.BehaviourUpdateNs);
        AppendNsStats(summary, "fixed_update_ms", s => s.FixedUpdateNs);
        AppendNsStats(summary, "physics_simulate_ms", s => s.PhysicsSimulateNs);
        AppendNsStats(summary, "camera_render_ms", s => s.CameraRenderNs);
        AppendBytesStats(summary, "gc_alloc_bytes", s => s.GcAllocBytes);
        AppendMiBStats(summary, "gc_used_mib", s => s.GcUsedBytes);
        AppendMiBStats(summary, "total_used_mib", s => s.TotalUsedBytes);
        AppendMiBStats(summary, "system_used_mib", s => s.SystemUsedBytes);

        summary.AppendLine();
        summary.AppendLine("instrumented_marker_ms:");
        foreach (string markerName in InstrumentedMarkers)
        {
            AppendLongNsStats(summary, "  " + markerName, markerSamples.TryGetValue(markerName, out List<long> values)
                ? values
                : null);
        }

        if (unavailable.Count > 0)
        {
            summary.AppendLine();
            summary.AppendLine("unavailable_recorders:");
            foreach (KeyValuePair<string, string> item in unavailable)
            {
                summary.AppendLine($"  {item.Key}: {item.Value}");
            }
        }

        summary.AppendLine();
        summary.AppendLine("runtime_stopwatch_marker_ms:");
        AppendRuntimeStopwatchStats(summary);

        return summary.ToString();
    }

    float ComputeUpdateFps()
    {
        if (samples.Count < 2)
        {
            return 0f;
        }

        float duration = Mathf.Max(1e-6f, samples[samples.Count - 1].Time - samples[0].Time);
        return (samples[samples.Count - 1].Frame - samples[0].Frame) / duration;
    }

    float ComputeFixedHz()
    {
        if (samples.Count == 0)
        {
            return 0f;
        }

        float duration = Mathf.Max(1e-6f, samples[samples.Count - 1].Time);
        return fixedFrameCount / duration;
    }

    void AppendNsStats(StringBuilder summary, string name, Func<Sample, long> selector)
    {
        double[] values = NonZeroValues(selector).Select(NsToMs).ToArray();
        AppendDoubleStats(summary, name, values);
    }

    void AppendBytesStats(StringBuilder summary, string name, Func<Sample, long> selector)
    {
        double[] values = NonZeroValues(selector).Select(v => (double)v).ToArray();
        AppendDoubleStats(summary, name, values);
    }

    void AppendMiBStats(StringBuilder summary, string name, Func<Sample, long> selector)
    {
        double[] values = NonZeroValues(selector).Select(BytesToMiB).ToArray();
        AppendDoubleStats(summary, name, values);
    }

    IEnumerable<long> NonZeroValues(Func<Sample, long> selector)
    {
        return samples.Select(selector).Where(v => v > 0);
    }

    static void AppendDoubleStats(StringBuilder summary, string name, double[] values)
    {
        if (values.Length == 0)
        {
            summary.AppendLine($"{name}: unavailable");
            return;
        }

        Array.Sort(values);
        summary.AppendLine(
            $"{name}: avg={values.Average().ToString("F4", CultureInfo.InvariantCulture)}, " +
            $"p50={Percentile(values, 0.50).ToString("F4", CultureInfo.InvariantCulture)}, " +
            $"p95={Percentile(values, 0.95).ToString("F4", CultureInfo.InvariantCulture)}, " +
            $"max={values[values.Length - 1].ToString("F4", CultureInfo.InvariantCulture)}");
    }

    static void AppendLongNsStats(StringBuilder summary, string name, List<long> nsValues)
    {
        if (nsValues == null)
        {
            summary.AppendLine($"{name}: unavailable");
            return;
        }

        double[] values = nsValues.Where(v => v > 0).Select(NsToMs).ToArray();
        AppendDoubleStats(summary, name, values);
    }

    void AppendRuntimeStopwatchStats(StringBuilder summary)
    {
        Dictionary<string, FinsSimRuntimeProfiler.MarkerStats> totalByName =
            FinsSimRuntimeProfiler.SnapshotTotal().ToDictionary(s => s.Name, StringComparer.Ordinal);
        List<string> markerNames = runtimeMarkerFrameTotalMs.Keys.OrderBy(k => k, StringComparer.Ordinal).ToList();

        foreach (string markerName in markerNames)
        {
            List<double> frameTotals = runtimeMarkerFrameTotalMs[markerName];
            List<int> frameCounts = runtimeMarkerFrameCounts.TryGetValue(markerName, out List<int> counts)
                ? counts
                : new List<int>();
            double[] nonZeroFrameTotals = frameTotals.Where(v => v > 0.0).ToArray();
            if (!totalByName.TryGetValue(markerName, out FinsSimRuntimeProfiler.MarkerStats totalStats) ||
                totalStats.Count <= 0 ||
                nonZeroFrameTotals.Length == 0)
            {
                summary.AppendLine($"  {markerName}: no_calls");
                continue;
            }

            Array.Sort(nonZeroFrameTotals);
            double averageCallsPerSample = frameCounts.Count > 0
                ? frameCounts.Average()
                : 0.0;
            double averageCallMs = totalStats.TotalMs / Math.Max(1, totalStats.Count);
            summary.AppendLine(
                $"  {markerName}: " +
                $"frames={nonZeroFrameTotals.Length}, " +
                $"calls={totalStats.Count}, " +
                $"avg_calls_per_sample={averageCallsPerSample.ToString("F2", CultureInfo.InvariantCulture)}, " +
                $"avg_frame_total={nonZeroFrameTotals.Average().ToString("F4", CultureInfo.InvariantCulture)}, " +
                $"p50_frame_total={Percentile(nonZeroFrameTotals, 0.50).ToString("F4", CultureInfo.InvariantCulture)}, " +
                $"p95_frame_total={Percentile(nonZeroFrameTotals, 0.95).ToString("F4", CultureInfo.InvariantCulture)}, " +
                $"max_frame_total={nonZeroFrameTotals[nonZeroFrameTotals.Length - 1].ToString("F4", CultureInfo.InvariantCulture)}, " +
                $"total_ms={totalStats.TotalMs.ToString("F4", CultureInfo.InvariantCulture)}, " +
                $"avg_call_ms={averageCallMs.ToString("F4", CultureInfo.InvariantCulture)}, " +
                $"max_call_ms={totalStats.MaxMs.ToString("F4", CultureInfo.InvariantCulture)}");
        }
    }

    static double Percentile(double[] sortedValues, double p)
    {
        if (sortedValues.Length == 0)
        {
            return 0.0;
        }

        double index = Mathf.Clamp01((float)p) * (sortedValues.Length - 1);
        int lower = Mathf.FloorToInt((float)index);
        int upper = Mathf.CeilToInt((float)index);
        if (lower == upper)
        {
            return sortedValues[lower];
        }

        return sortedValues[lower] + (sortedValues[upper] - sortedValues[lower]) * (index - lower);
    }

    static double NsToMs(long ns)
    {
        return ns / 1_000_000.0;
    }

    static double BytesToMiB(long bytes)
    {
        return bytes / 1024.0 / 1024.0;
    }
}
