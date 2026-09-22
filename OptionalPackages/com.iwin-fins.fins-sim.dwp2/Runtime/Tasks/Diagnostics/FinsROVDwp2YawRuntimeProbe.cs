using System;
using System.Globalization;
using System.IO;
using FinsSim.Actuators;
using NWH.DWP2.WaterObjects;
using UnityEngine;
using UnityEngine.SceneManagement;

[DisallowMultipleComponent]
public sealed class FinsROVDwp2YawRuntimeProbe : MonoBehaviour
{
    [Header("References")]
    [SerializeField] Rigidbody targetRigidbody;
    [SerializeField] Transform waterObjectSearchRoot;
    [SerializeField] ThrusterController thrusterController;

    [Header("Logging")]
    [SerializeField] bool enableLogging = true;
    [SerializeField] bool logOnlyWhenMovingOrCommanded = true;
    [SerializeField, Min(0.05f)] float logIntervalSec = 1.0f;
    [SerializeField, Min(0f)] float minYawRateRadPerSec = 0.02f;
    [SerializeField, Min(0f)] float minThrusterTorqueNm = 0.01f;

    [Header("CSV")]
    [SerializeField] bool enableCsvLogging = true;
    [SerializeField] string csvOutputDirectory = "/home/fins/UnderwaterSim/Code/FinsSim/ros2_ws/data/hydrodynamics/unity/probe";
    [SerializeField] string csvFilePrefix = "dwp2_yaw_runtime_probe";
    [SerializeField, Min(0f)] float csvSampleIntervalSec = 0.02f;
    [SerializeField] string activeCsvPath;

    [Header("Runtime Readout")]
    [SerializeField] int waterObjectCount;
    [SerializeField] bool axisScalingEnabled;
    [SerializeField] HydrodynamicYawDampingMode yawDampingMode;
    [SerializeField] Vector3 hydrodynamicTorqueAxisScale;
    [SerializeField] float yawLinearDamping;
    [SerializeField] float yawQuadraticDamping;
    [SerializeField] float bodyYawRateRadPerSec;
    [SerializeField] float thrusterTargetBodyYawTorqueNm;
    [SerializeField] float thrusterAppliedBodyYawTorqueNm;
    [SerializeField] float unityActualBodyYawTorqueNm;
    [SerializeField] float thrusterBodyYawTorqueNm;
    [SerializeField] float dwp2RawHydroBodyYawTorqueNm;
    [SerializeField] float dwp2ScaledHydroBodyYawTorqueNm;
    [SerializeField] float dwp2TargetYawDampingNm;
    [SerializeField] float dwp2AppliedYawDampingNm;
    [SerializeField] float dwp2FinalBodyYawTorqueNm;
    [SerializeField] float unityAppliedPlusDwp2HydroYawTorqueNm;
    [SerializeField] int yawDampingShareCount = 1;

    WaterObject[] _waterObjects;
    float _nextLogTime;
    float _nextCsvTime;
    StreamWriter _csvWriter;

    void Reset()
    {
        ResolveReferences();
    }

    void Awake()
    {
        ResolveReferences();
        EnableThrusterDebug();
    }

    void OnEnable()
    {
        EnableThrusterDebug();
        OpenCsvIfNeeded();
    }

    void OnDisable()
    {
        CloseCsv();
    }

    void LateUpdate()
    {
        RefreshReadout();

        bool active = Mathf.Abs(bodyYawRateRadPerSec) >= minYawRateRadPerSec ||
                      Mathf.Abs(thrusterTargetBodyYawTorqueNm) >= minThrusterTorqueNm ||
                      Mathf.Abs(thrusterAppliedBodyYawTorqueNm) >= minThrusterTorqueNm ||
                      Mathf.Abs(dwp2AppliedYawDampingNm) >= minThrusterTorqueNm;
        WriteCsvIfNeeded(active);

        if (!enableLogging || Time.time < _nextLogTime)
        {
            return;
        }

        if (logOnlyWhenMovingOrCommanded && !active)
        {
            return;
        }

        _nextLogTime = Time.time + logIntervalSec;
        Debug.Log(
            "[FinsROVDwp2YawRuntimeProbe] " +
            $"wo={waterObjectCount}, scaling={axisScalingEnabled}, mode={yawDampingMode}, " +
            $"torqueScale={hydrodynamicTorqueAxisScale}, d1={yawLinearDamping:F3}, d2={yawQuadraticDamping:F3}, " +
            $"r={bodyYawRateRadPerSec:F4} rad/s, " +
            $"thrusterTargetYaw={thrusterTargetBodyYawTorqueNm:F4} Nm, " +
            $"thrusterAppliedYaw={thrusterAppliedBodyYawTorqueNm:F4} Nm, " +
            $"shareCount={yawDampingShareCount}, " +
            $"dwp2RawYaw={dwp2RawHydroBodyYawTorqueNm:F4} Nm, " +
            $"dwp2ScaledYaw={dwp2ScaledHydroBodyYawTorqueNm:F4} Nm, " +
            $"dwp2TargetYaw={dwp2TargetYawDampingNm:F4} Nm, " +
            $"dwp2AppliedYaw={dwp2AppliedYawDampingNm:F4} Nm, " +
            $"dwp2FinalYaw={dwp2FinalBodyYawTorqueNm:F4} Nm, " +
            $"thrusterPlusHydroYaw={unityAppliedPlusDwp2HydroYawTorqueNm:F4} Nm",
            this);
    }

    [ContextMenu("Resolve References")]
    public void ResolveReferences()
    {
        if (targetRigidbody == null)
        {
            targetRigidbody = GetComponentInParent<Rigidbody>();
        }

        if (targetRigidbody == null)
        {
            targetRigidbody = GetComponentInChildren<Rigidbody>(true);
        }

        if (waterObjectSearchRoot == null)
        {
            waterObjectSearchRoot = targetRigidbody != null ? targetRigidbody.transform : transform;
        }

        if (thrusterController == null)
        {
            thrusterController = targetRigidbody != null
                ? targetRigidbody.GetComponent<ThrusterController>()
                : GetComponentInChildren<ThrusterController>(true);
        }

        RefreshWaterObjects();
    }

    [ContextMenu("Refresh WaterObjects")]
    public void RefreshWaterObjects()
    {
        _waterObjects = waterObjectSearchRoot != null
            ? waterObjectSearchRoot.GetComponentsInChildren<WaterObject>(true)
            : GetComponentsInChildren<WaterObject>(true);
    }

    void EnableThrusterDebug()
    {
        if (thrusterController == null)
        {
            ResolveReferences();
        }

        if (thrusterController == null)
        {
            return;
        }

        thrusterController.EnableRuntimeNetWrenchStats = true;
        for (int i = 0; i < thrusterController.thrusters.Count; i++)
        {
            Thruster thruster = thrusterController.thrusters[i];
            if (thruster != null)
            {
                thruster.EnableRuntimeDebugState = true;
            }
        }
    }

    void RefreshReadout()
    {
        if (_waterObjects == null)
        {
            RefreshWaterObjects();
        }

        bodyYawRateRadPerSec = GetBodyYawRate();
        thrusterTargetBodyYawTorqueNm = ComputeThrusterBodyYawTorque(useAppliedForce: false);
        thrusterAppliedBodyYawTorqueNm = ComputeThrusterBodyYawTorque(useAppliedForce: true);
        unityActualBodyYawTorqueNm = thrusterAppliedBodyYawTorqueNm;
        thrusterBodyYawTorqueNm = thrusterAppliedBodyYawTorqueNm;
        waterObjectCount = 0;
        yawDampingShareCount = 1;
        axisScalingEnabled = false;
        yawDampingMode = HydrodynamicYawDampingMode.AxisScaleOnly;
        hydrodynamicTorqueAxisScale = Vector3.zero;
        yawLinearDamping = 0f;
        yawQuadraticDamping = 0f;
        dwp2RawHydroBodyYawTorqueNm = 0f;
        dwp2ScaledHydroBodyYawTorqueNm = 0f;
        dwp2TargetYawDampingNm = 0f;
        dwp2AppliedYawDampingNm = 0f;
        dwp2FinalBodyYawTorqueNm = 0f;

        if (_waterObjects == null)
        {
            return;
        }

        for (int i = 0; i < _waterObjects.Length; i++)
        {
            WaterObject waterObject = _waterObjects[i];
            if (waterObject == null || !waterObject.isActiveAndEnabled)
            {
                continue;
            }

            waterObjectCount++;
            axisScalingEnabled |= waterObject.hydrodynamicAxisScalingEnabled;
            yawDampingMode = waterObject.hydrodynamicYawDampingMode;
            hydrodynamicTorqueAxisScale = waterObject.hydrodynamicTorqueAxisScale;
            yawLinearDamping = waterObject.hydrodynamicYawLinearDamping;
            yawQuadraticDamping = waterObject.hydrodynamicYawQuadraticDamping;
            dwp2RawHydroBodyYawTorqueNm += waterObject.ResultHydrodynamicBodyTorqueRaw.z;
            dwp2ScaledHydroBodyYawTorqueNm += waterObject.ResultHydrodynamicBodyTorqueScaled.z;
            dwp2TargetYawDampingNm += waterObject.ResultHydrodynamicBodyYawDampingTargetNm;
            dwp2AppliedYawDampingNm += waterObject.ResultHydrodynamicBodyYawDampingAppliedNm;
            dwp2FinalBodyYawTorqueNm += WorldTorqueToFinsRovBody(waterObject.ResultTorque).z;
            yawDampingShareCount = Mathf.Max(yawDampingShareCount, waterObject.ResultHydrodynamicBodyYawDampingShareCount);
        }

        unityAppliedPlusDwp2HydroYawTorqueNm = unityActualBodyYawTorqueNm + dwp2AppliedYawDampingNm;
    }

    float GetBodyYawRate()
    {
        if (targetRigidbody == null)
        {
            return 0f;
        }

        Vector3 localAngularVelocity = targetRigidbody.transform.InverseTransformDirection(targetRigidbody.angularVelocity);
        return localAngularVelocity.y;
    }

    float ComputeThrusterBodyYawTorque(bool useAppliedForce)
    {
        if (targetRigidbody == null || thrusterController == null)
        {
            return 0f;
        }

        Vector3 worldTorque = Vector3.zero;
        Vector3 origin = targetRigidbody.worldCenterOfMass;
        for (int i = 0; i < thrusterController.thrusters.Count; i++)
        {
            Thruster thruster = thrusterController.thrusters[i];
            if (thruster == null)
            {
                continue;
            }

            Vector3 force = GetThrusterWorldForceDirection(thruster) *
                            (useAppliedForce ? thruster.LastForceRequest : thruster.TargetForceRequest);
            Vector3 leverArm = thruster.transform.position - origin;
            worldTorque += Vector3.Cross(leverArm, force);
        }

        return WorldTorqueToFinsRovBody(worldTorque).z;
    }

    static Vector3 GetThrusterWorldForceDirection(Thruster thruster)
    {
        Vector3 localDirection = thruster.LocalForceDirection.sqrMagnitude > 1e-8f
            ? thruster.LocalForceDirection.normalized
            : Vector3.forward;
        return thruster.transform.TransformDirection(localDirection).normalized;
    }

    void OpenCsvIfNeeded()
    {
        if (!enableCsvLogging || _csvWriter != null)
        {
            return;
        }

        try
        {
            Directory.CreateDirectory(csvOutputDirectory);
            string sceneName = SceneManager.GetActiveScene().name;
            string timestamp = DateTime.Now.ToString("yyyyMMdd_HHmmss", CultureInfo.InvariantCulture);
            string fileName = $"{SanitizeFileName(csvFilePrefix)}_{SanitizeFileName(sceneName)}_{timestamp}.csv";
            activeCsvPath = Path.Combine(csvOutputDirectory, fileName);
            _csvWriter = new StreamWriter(activeCsvPath, append: false);
            _csvWriter.WriteLine(
                "time_sec,scene,body_yaw_rate_radps," +
                "thruster_target_yaw_tau_nm,thruster_applied_yaw_tau_nm,unity_actual_yaw_tau_nm," +
                "dwp2_raw_hydro_yaw_tau_nm,dwp2_scaled_hydro_yaw_tau_nm," +
                "dwp2_target_real_curve_yaw_tau_nm,dwp2_applied_hydro_yaw_tau_nm,dwp2_final_yaw_tau_nm," +
                "unity_applied_plus_dwp2_hydro_yaw_tau_nm," +
                "waterobject_count,yaw_damping_share_count,axis_scaling_enabled,yaw_damping_mode,yaw_linear_damping,yaw_quadratic_damping");
            _nextCsvTime = 0f;
        }
        catch (Exception exception)
        {
            Debug.LogWarning($"[FinsROVDwp2YawRuntimeProbe] Failed to open CSV: {exception.Message}", this);
            CloseCsv();
        }
    }

    void WriteCsvIfNeeded(bool active)
    {
        if (!enableCsvLogging)
        {
            return;
        }

        OpenCsvIfNeeded();
        if (_csvWriter == null || Time.time < _nextCsvTime)
        {
            return;
        }

        if (logOnlyWhenMovingOrCommanded && !active)
        {
            return;
        }

        _nextCsvTime = Time.time + csvSampleIntervalSec;
        _csvWriter.Write(Time.time.ToString("F6", CultureInfo.InvariantCulture));
        _csvWriter.Write(',');
        _csvWriter.Write(EscapeCsv(SceneManager.GetActiveScene().name));
        WriteCsvFloat(bodyYawRateRadPerSec);
        WriteCsvFloat(thrusterTargetBodyYawTorqueNm);
        WriteCsvFloat(thrusterAppliedBodyYawTorqueNm);
        WriteCsvFloat(unityActualBodyYawTorqueNm);
        WriteCsvFloat(dwp2RawHydroBodyYawTorqueNm);
        WriteCsvFloat(dwp2ScaledHydroBodyYawTorqueNm);
        WriteCsvFloat(dwp2TargetYawDampingNm);
        WriteCsvFloat(dwp2AppliedYawDampingNm);
        WriteCsvFloat(dwp2FinalBodyYawTorqueNm);
        WriteCsvFloat(unityAppliedPlusDwp2HydroYawTorqueNm);
        _csvWriter.Write(',');
        _csvWriter.Write(waterObjectCount.ToString(CultureInfo.InvariantCulture));
        _csvWriter.Write(',');
        _csvWriter.Write(yawDampingShareCount.ToString(CultureInfo.InvariantCulture));
        _csvWriter.Write(',');
        _csvWriter.Write(axisScalingEnabled ? "1" : "0");
        _csvWriter.Write(',');
        _csvWriter.Write(yawDampingMode.ToString());
        WriteCsvFloat(yawLinearDamping);
        WriteCsvFloat(yawQuadraticDamping);
        _csvWriter.WriteLine();
        _csvWriter.Flush();
    }

    void WriteCsvFloat(float value)
    {
        _csvWriter.Write(',');
        _csvWriter.Write(value.ToString("F9", CultureInfo.InvariantCulture));
    }

    void CloseCsv()
    {
        if (_csvWriter == null)
        {
            return;
        }

        _csvWriter.Flush();
        _csvWriter.Dispose();
        _csvWriter = null;
    }

    static string SanitizeFileName(string value)
    {
        if (string.IsNullOrWhiteSpace(value))
        {
            return "unnamed";
        }

        char[] invalid = Path.GetInvalidFileNameChars();
        for (int i = 0; i < invalid.Length; i++)
        {
            value = value.Replace(invalid[i], '_');
        }

        return value.Replace(' ', '_');
    }

    static string EscapeCsv(string value)
    {
        value ??= string.Empty;
        if (value.IndexOfAny(new[] { ',', '"', '\n', '\r' }) < 0)
        {
            return value;
        }

        return "\"" + value.Replace("\"", "\"\"") + "\"";
    }

    Vector3 WorldTorqueToFinsRovBody(Vector3 worldTorque)
    {
        if (targetRigidbody == null)
        {
            return worldTorque;
        }

        Vector3 localTorque = targetRigidbody.transform.InverseTransformDirection(worldTorque);
        return new Vector3(localTorque.x, localTorque.z, localTorque.y);
    }
}
