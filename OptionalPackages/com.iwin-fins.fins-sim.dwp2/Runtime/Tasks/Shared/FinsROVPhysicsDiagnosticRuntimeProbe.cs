using System;
using System.Globalization;
using System.IO;
using System.Linq;
using System.Text;
using FinsSim.Hydrodynamics;
using FinsSim.Core.Spatial;
using FinsSim.Actuators;
using NWH.DWP2.WaterData;
using NWH.DWP2.WaterObjects;
using UnityEngine;

public class FinsROVPhysicsDiagnosticRuntimeProbe : MonoBehaviour
{
    string mode;
    string outputPath;
    float durationSec;
    float yawCommand;
    float startTime;
    int fixedFrameCount;
    [SerializeField] bool collectSamples = true;
    [SerializeField] bool enableThrusterDebugStateForSamples = true;
    readonly StringBuilder csv = new StringBuilder(64 * 1024);
    Action onFinished;

    Rigidbody body;
    ThrusterController thrusterController;
    WaterObject[] waterObjects;
    FinsROVManualThrusterController manualController;
    UnityHDRPWaterDataProvider hdrpProvider;
    HydrodynamicsController hydrodynamicsController;

    float maxAngularSpeed;
    float maxLinearSpeed;
    float maxAbsLocalYawRate;
    float maxAbsWaterTorque;
    float maxAbsWaterForce;
    float maxAbsThrusterTorque;
    float maxAbsBackendHydroForce;
    float maxAbsBackendHydroTorque;
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
        hydrodynamicsController = GetComponent<HydrodynamicsController>();

        ConfigureThrusterDebugSampling();

        if (manualController != null)
        {
            manualController.enabled = false;
        }

        ApplyModeConfiguration();
        startTime = Time.fixedTime;

        csv.AppendLine("time,mode,backend,position_x,position_y,position_z,euler_x,euler_y,euler_z,linear_speed,angular_speed,local_yaw_rate,water_force_mag,water_torque_mag,backend_hydro_force_mag,backend_hydro_torque_mag,thruster_force_mag,thruster_torque_mag,submerged_volume,rel_u,rel_v,rel_w,rel_p,rel_q,rel_r,water_flow_x,water_flow_y,water_flow_z");
        Debug.Log($"FinsROV diagnostic probe started: mode={mode}, backend={(hydrodynamicsController != null ? hydrodynamicsController.mode.ToString() : "none")}, waterObjects={waterObjects.Length}, provider={(hdrpProvider != null ? hdrpProvider.enabled : false)}");
    }

    void FixedUpdate()
    {
        fixedFrameCount++;

        if (mode.Contains("yaw", StringComparison.OrdinalIgnoreCase) && thrusterController != null)
        {
            float[] inputs = { 0f, 0f, 0f, 0f, yawCommand, yawCommand, yawCommand, yawCommand };
            thrusterController.ApplyInput(inputs);
        }

        if (collectSamples)
        {
            RecordSample();
        }

        if (Time.fixedTime - startTime >= durationSec)
        {
            WriteOutput();
            enabled = false;
            onFinished?.Invoke();
        }
    }

    void ConfigureThrusterDebugSampling()
    {
        if (thrusterController == null || !enableThrusterDebugStateForSamples)
        {
            return;
        }

        thrusterController.EnableRuntimeNetWrenchStats = collectSamples;
        foreach (Thruster thruster in thrusterController.thrusters.Where(t => t != null))
        {
            thruster.EnableRuntimeDebugState = collectSamples;
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
        Vector3 backendForce = hydrodynamicsController != null ? hydrodynamicsController.LastWrench.WorldForce : Vector3.zero;
        Vector3 backendTorque = hydrodynamicsController != null ? hydrodynamicsController.LastWrench.WorldTorque : Vector3.zero;
        SixDofVector relativeVelocity = hydrodynamicsController != null ? hydrodynamicsController.LastRelativeVelocity6Dof : SixDofVector.Zero;
        Vector3 waterFlow = hydrodynamicsController != null ? hydrodynamicsController.LastWaterSample.FlowVelocity : Vector3.zero;

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
        maxAbsBackendHydroForce = Mathf.Max(maxAbsBackendHydroForce, backendForce.magnitude);
        maxAbsBackendHydroTorque = Mathf.Max(maxAbsBackendHydroTorque, backendTorque.magnitude);
        maxAbsRollPitchDeg = Mathf.Max(maxAbsRollPitchDeg, rollPitch);
        sampleCount++;

        csv.AppendFormat(
            CultureInfo.InvariantCulture,
            "{0:F4},{1},{2},{3:F6},{4:F6},{5:F6},{6:F4},{7:F4},{8:F4},{9:F6},{10:F6},{11:F6},{12:F6},{13:F6},{14:F6},{15:F6},{16:F6},{17:F6},{18:F6},{19:F6},{20:F6},{21:F6},{22:F6},{23:F6},{24:F6},{25:F6},{26:F6},{27:F6}\n",
            Time.fixedTime - startTime,
            mode,
            hydrodynamicsController != null ? hydrodynamicsController.mode.ToString() : "none",
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
            backendForce.magnitude,
            backendTorque.magnitude,
            thrusterForce.magnitude,
            thrusterTorqueMag,
            submergedVolume,
            relativeVelocity.u,
            relativeVelocity.v,
            relativeVelocity.w,
            relativeVelocity.p,
            relativeVelocity.q,
            relativeVelocity.r,
            waterFlow.x,
            waterFlow.y,
            waterFlow.z);
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
        summary.AppendLine($"hydrodynamics_backend={(hydrodynamicsController != null ? hydrodynamicsController.mode.ToString() : "none")}");
        summary.AppendLine($"max_linear_speed_mps={maxLinearSpeed.ToString("F6", CultureInfo.InvariantCulture)}");
        summary.AppendLine($"max_angular_speed_radps={maxAngularSpeed.ToString("F6", CultureInfo.InvariantCulture)}");
        summary.AppendLine($"max_abs_local_yaw_rate_radps={maxAbsLocalYawRate.ToString("F6", CultureInfo.InvariantCulture)}");
        summary.AppendLine($"max_abs_roll_pitch_deg={maxAbsRollPitchDeg.ToString("F6", CultureInfo.InvariantCulture)}");
        summary.AppendLine($"max_water_force_n={maxAbsWaterForce.ToString("F6", CultureInfo.InvariantCulture)}");
        summary.AppendLine($"max_water_torque_nm={maxAbsWaterTorque.ToString("F6", CultureInfo.InvariantCulture)}");
        summary.AppendLine($"max_backend_hydro_force_n={maxAbsBackendHydroForce.ToString("F6", CultureInfo.InvariantCulture)}");
        summary.AppendLine($"max_backend_hydro_torque_nm={maxAbsBackendHydroTorque.ToString("F6", CultureInfo.InvariantCulture)}");
        summary.AppendLine($"max_thruster_torque_nm={maxAbsThrusterTorque.ToString("F6", CultureInfo.InvariantCulture)}");
        File.WriteAllText(summaryPath, summary.ToString());

        Debug.Log($"FinsROV physics diagnostic wrote {fullOutputPath} and {summaryPath}");
    }
}
