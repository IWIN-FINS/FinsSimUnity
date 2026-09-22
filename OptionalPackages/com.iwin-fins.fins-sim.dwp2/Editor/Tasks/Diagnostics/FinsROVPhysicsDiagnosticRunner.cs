using System;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using FinsSim.Actuators;
using NWH.DWP2.WaterData;
using NWH.DWP2.WaterObjects;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

[InitializeOnLoad]
public static class FinsROVPhysicsDiagnosticRunner
{
    const string DefaultScenePath = "Assets/FinsSimUnity/Tasks/PoseControl/Scenes/ControlForPosition_new.unity";
    const string DefaultOutputPath = "Temp/finsrov_physics_diagnostic.csv";
    const string SessionActiveKey = "FinsROVPhysicsDiagnosticRunner.Active";
    const string SessionExitingKey = "FinsROVPhysicsDiagnosticRunner.Exiting";
    const string SessionModeKey = "FinsROVPhysicsDiagnosticRunner.Mode";
    const string SessionOutputKey = "FinsROVPhysicsDiagnosticRunner.Output";
    const string SessionDurationKey = "FinsROVPhysicsDiagnosticRunner.Duration";
    const string SessionYawKey = "FinsROVPhysicsDiagnosticRunner.Yaw";

    static bool waitingForPlayMode;
    static bool exitingAfterPlayMode;
    static string mode;
    static string outputPath;
    static float durationSec;
    static float yawCommand;

    static FinsROVPhysicsDiagnosticRunner()
    {
        EditorApplication.update -= Update;
        EditorApplication.update += Update;
        EditorApplication.playModeStateChanged -= OnPlayModeStateChanged;
        EditorApplication.playModeStateChanged += OnPlayModeStateChanged;
    }

    public static void Run()
    {
        mode = GetArg("-diagMode", "idle");
        outputPath = GetArg("-diagOutput", DefaultOutputPath);
        durationSec = ParseFloatArg("-diagDuration", 8f);
        yawCommand = ParseFloatArg("-diagYaw", 1f);
        StoreSession();

        Debug.Log($"Starting FinsROV physics diagnostic: mode={mode}, duration={durationSec}, output={outputPath}");
        EditorSceneManager.OpenScene(DefaultScenePath);
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
        GameObject rov = GameObject.Find("FinsROV");
        if (rov == null)
        {
            Debug.LogError("FinsROV not found in scene.");
            EditorApplication.Exit(2);
            return;
        }

        Debug.Log("FinsROV found; attaching runtime diagnostic probe.");
        var probe = rov.AddComponent<FinsROVPhysicsDiagnosticRuntimeProbe>();
        probe.Configure(mode, outputPath, durationSec, yawCommand, () =>
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
        SessionState.SetString(SessionModeKey, mode);
        SessionState.SetString(SessionOutputKey, outputPath);
        SessionState.SetFloat(SessionDurationKey, durationSec);
        SessionState.SetFloat(SessionYawKey, yawCommand);
    }

    static void LoadSession()
    {
        mode = SessionState.GetString(SessionModeKey, mode ?? "idle");
        outputPath = SessionState.GetString(SessionOutputKey, outputPath ?? DefaultOutputPath);
        durationSec = SessionState.GetFloat(SessionDurationKey, durationSec > 0f ? durationSec : 8f);
        yawCommand = SessionState.GetFloat(SessionYawKey, yawCommand);
        exitingAfterPlayMode = SessionState.GetBool(SessionExitingKey, false);
    }

    static void ClearSession()
    {
        SessionState.EraseBool(SessionActiveKey);
        SessionState.EraseBool(SessionExitingKey);
        SessionState.EraseString(SessionModeKey);
        SessionState.EraseString(SessionOutputKey);
        SessionState.EraseFloat(SessionDurationKey);
        SessionState.EraseFloat(SessionYawKey);
    }
}

public class FinsROVPhysicsDiagnosticProbe : MonoBehaviour
{
    string mode;
    string outputPath;
    float durationSec;
    float yawCommand;
    float startTime;
    int fixedFrameCount;
    readonly StringBuilder csv = new StringBuilder(64 * 1024);
    Action onFinished;

    Rigidbody body;
    ThrusterController thrusterController;
    WaterObject[] waterObjects;
    FinsROVManualThrusterController manualController;
    UnityHDRPWaterDataProvider hdrpProvider;

    float maxAngularSpeed;
    float maxLinearSpeed;
    float maxAbsLocalYawRate;
    float maxAbsWaterTorque;
    float maxAbsWaterForce;
    float maxAbsThrusterTorque;
    float maxAbsRollPitchDeg;
    int sampleCount;

    public void Configure(string diagnosticMode, string diagnosticOutputPath, float diagnosticDurationSec, float diagnosticYawCommand, Action finished)
    {
        mode = diagnosticMode;
        outputPath = diagnosticOutputPath;
        durationSec = Mathf.Max(0.5f, diagnosticDurationSec);
        yawCommand = Mathf.Clamp(diagnosticYawCommand, -1f, 1f);
        onFinished = finished;
    }

    void Start()
    {
        body = GetComponent<Rigidbody>();
        thrusterController = GetComponent<ThrusterController>();
        waterObjects = GetComponentsInChildren<WaterObject>(true);
        manualController = GetComponent<FinsROVManualThrusterController>();
        hdrpProvider = FindFirstObjectByType<UnityHDRPWaterDataProvider>();

        if (manualController != null)
        {
            manualController.enabled = false;
        }

        ApplyModeConfiguration();
        startTime = Time.fixedTime;

        csv.AppendLine("time,mode,position_x,position_y,position_z,euler_x,euler_y,euler_z,linear_speed,angular_speed,local_yaw_rate,water_force_mag,water_torque_mag,thruster_force_mag,thruster_torque_mag,submerged_volume");
    }

    void FixedUpdate()
    {
        fixedFrameCount++;

        if (mode.Contains("yaw", StringComparison.OrdinalIgnoreCase) && thrusterController != null)
        {
            float[] inputs = { 0f, 0f, 0f, 0f, yawCommand, yawCommand, yawCommand, yawCommand };
            thrusterController.ApplyInput(inputs);
        }

        RecordSample();

        if (Time.fixedTime - startTime >= durationSec)
        {
            WriteOutput();
            enabled = false;
            onFinished?.Invoke();
        }
    }

    void ApplyModeConfiguration()
    {
        if (mode.Contains("noProvider", StringComparison.OrdinalIgnoreCase) && hdrpProvider != null)
        {
            hdrpProvider.enabled = false;
        }

        bool noHydro = mode.Contains("noHydro", StringComparison.OrdinalIgnoreCase);
        bool mainBodyOnly = mode.Contains("mainBodyOnly", StringComparison.OrdinalIgnoreCase);
        foreach (WaterObject waterObject in waterObjects)
        {
            if (waterObject == null)
            {
                continue;
            }

            if (noHydro)
            {
                waterObject.hydrodynamicForceCoefficient = 0f;
            }

            if (mainBodyOnly && !waterObject.name.Equals("mainbody", StringComparison.OrdinalIgnoreCase))
            {
                waterObject.enabled = false;
            }
        }
    }

    void RecordSample()
    {
        Vector3 localAngularVelocity = transform.InverseTransformDirection(body.angularVelocity);
        Vector3 waterForce = Vector3.zero;
        Vector3 waterTorque = Vector3.zero;
        float submergedVolume = 0f;

        foreach (WaterObject waterObject in waterObjects)
        {
            if (waterObject == null || !waterObject.enabled)
            {
                continue;
            }

            waterForce += waterObject.ResultForce;
            waterTorque += waterObject.ResultTorque;
            submergedVolume += waterObject.submergedVolume;
        }

        GetThrusterNetForceTorque(out Vector3 thrusterForce, out Vector3 thrusterTorque);

        float angularSpeed = body.angularVelocity.magnitude;
        float linearSpeed = body.linearVelocity.magnitude;
        float absLocalYawRate = Mathf.Abs(localAngularVelocity.y);
        float waterForceMag = waterForce.magnitude;
        float waterTorqueMag = waterTorque.magnitude;
        float thrusterTorqueMag = thrusterTorque.magnitude;
        Vector3 euler = transform.eulerAngles;
        float rollPitch = Mathf.Max(Mathf.Abs(Mathf.DeltaAngle(0f, euler.x)), Mathf.Abs(Mathf.DeltaAngle(0f, euler.z)));

        maxAngularSpeed = Mathf.Max(maxAngularSpeed, angularSpeed);
        maxLinearSpeed = Mathf.Max(maxLinearSpeed, linearSpeed);
        maxAbsLocalYawRate = Mathf.Max(maxAbsLocalYawRate, absLocalYawRate);
        maxAbsWaterForce = Mathf.Max(maxAbsWaterForce, waterForceMag);
        maxAbsWaterTorque = Mathf.Max(maxAbsWaterTorque, waterTorqueMag);
        maxAbsThrusterTorque = Mathf.Max(maxAbsThrusterTorque, thrusterTorqueMag);
        maxAbsRollPitchDeg = Mathf.Max(maxAbsRollPitchDeg, rollPitch);
        sampleCount++;

        csv.AppendFormat(
            CultureInfo.InvariantCulture,
            "{0:F4},{1},{2:F6},{3:F6},{4:F6},{5:F4},{6:F4},{7:F4},{8:F6},{9:F6},{10:F6},{11:F6},{12:F6},{13:F6},{14:F6},{15:F6}\n",
            Time.fixedTime - startTime,
            mode,
            transform.position.x,
            transform.position.y,
            transform.position.z,
            euler.x,
            euler.y,
            euler.z,
            linearSpeed,
            angularSpeed,
            localAngularVelocity.y,
            waterForceMag,
            waterTorqueMag,
            thrusterForce.magnitude,
            thrusterTorqueMag,
            submergedVolume);
    }

    void GetThrusterNetForceTorque(out Vector3 force, out Vector3 torque)
    {
        force = Vector3.zero;
        torque = Vector3.zero;

        if (thrusterController == null)
        {
            return;
        }

        Vector3 origin = body != null ? body.worldCenterOfMass : transform.position;
        foreach (Thruster thruster in thrusterController.thrusters.Where(t => t != null))
        {
            Vector3 thrusterForce = thruster.LastAppliedWorldForce;
            Vector3 leverArm = thruster.LastAppliedForcePosition - origin;
            force += thrusterForce;
            torque += Vector3.Cross(leverArm, thrusterForce);
        }
    }

    void WriteOutput()
    {
        string fullOutputPath = Path.GetFullPath(outputPath);
        Directory.CreateDirectory(Path.GetDirectoryName(fullOutputPath));
        File.WriteAllText(fullOutputPath, csv.ToString());

        string summaryPath = Path.ChangeExtension(fullOutputPath, ".summary.txt");
        var summary = new StringBuilder();
        summary.AppendLine($"mode={mode}");
        summary.AppendLine($"samples={sampleCount}");
        summary.AppendLine($"fixed_frames={fixedFrameCount}");
        summary.AppendLine($"water_objects_total={waterObjects.Length}");
        summary.AppendLine($"water_objects_enabled={waterObjects.Count(w => w != null && w.enabled)}");
        summary.AppendLine($"provider_enabled={(hdrpProvider != null && hdrpProvider.enabled)}");
        summary.AppendLine($"max_linear_speed_mps={maxLinearSpeed.ToString("F6", CultureInfo.InvariantCulture)}");
        summary.AppendLine($"max_angular_speed_radps={maxAngularSpeed.ToString("F6", CultureInfo.InvariantCulture)}");
        summary.AppendLine($"max_abs_local_yaw_rate_radps={maxAbsLocalYawRate.ToString("F6", CultureInfo.InvariantCulture)}");
        summary.AppendLine($"max_abs_roll_pitch_deg={maxAbsRollPitchDeg.ToString("F6", CultureInfo.InvariantCulture)}");
        summary.AppendLine($"max_water_force_n={maxAbsWaterForce.ToString("F6", CultureInfo.InvariantCulture)}");
        summary.AppendLine($"max_water_torque_nm={maxAbsWaterTorque.ToString("F6", CultureInfo.InvariantCulture)}");
        summary.AppendLine($"max_thruster_torque_nm={maxAbsThrusterTorque.ToString("F6", CultureInfo.InvariantCulture)}");
        File.WriteAllText(summaryPath, summary.ToString());

        Debug.Log($"FinsROV physics diagnostic wrote {fullOutputPath} and {summaryPath}");
    }
}
