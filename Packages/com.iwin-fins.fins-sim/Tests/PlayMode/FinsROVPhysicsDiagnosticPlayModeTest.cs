using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Reflection;
using System.Text;
using NUnit.Framework;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.TestTools;

public class FinsROVPhysicsDiagnosticPlayModeTest
{
    const string ScenePath = "Assets/FinsSimUnity/Tasks/PoseControl/Scenes/ControlForPosition_new.unity";
    const float DurationSec = 8f;
    const float YawCommand = 1f;

    static readonly string[] Modes =
    {
        "idle",
        "idle_noProvider",
        "idle_noHydro",
        "idle_mainBodyOnly",
        "yaw",
        "yaw_noProvider",
        "yaw_noHydro",
        "yaw_mainBodyOnly",
        "idle_noWater",
        "idle_depth2",
        "yaw_depth2",
        "yaw_depth2_noHydro",
    };

    [UnityTest]
    public IEnumerator RunFinsROVPhysicsDiagnostics()
    {
        LogAssert.ignoreFailingMessages = true;

        string outputDir = Environment.GetEnvironmentVariable("FINSROV_DIAG_OUT");
        if (string.IsNullOrWhiteSpace(outputDir))
        {
            outputDir = "/home/fins/UnderwaterSim/Code/UnityDiagnostics";
        }

        Directory.CreateDirectory(outputDir);

        var allSummaries = new StringBuilder();
        allSummaries.AppendLine("mode,max_linear_speed_mps,max_angular_speed_radps,max_abs_local_yaw_rate_radps,max_abs_roll_pitch_deg,max_water_force_n,max_water_torque_nm,max_thruster_torque_nm,water_objects_total,water_objects_enabled,provider_enabled,start_y,end_y,delta_y");

        foreach (string mode in Modes)
        {
            DiagnosticSummary summary = null;
            yield return RunMode(mode, outputDir, s => summary = s);
            Assert.NotNull(summary, $"Diagnostic mode {mode} did not produce a summary.");
            allSummaries.AppendLine(summary.ToCsv());
        }

        File.WriteAllText(Path.Combine(outputDir, "finsrov_diag_all_modes.summary.csv"), allSummaries.ToString());
    }

    static IEnumerator RunMode(string mode, string outputDir, Action<DiagnosticSummary> onSummary)
    {
        bool sceneLoaded = false;
        void OnSceneLoaded(Scene scene, LoadSceneMode loadMode)
        {
            if (scene.path == ScenePath)
            {
                sceneLoaded = true;
            }
        }

        SceneManager.sceneLoaded += OnSceneLoaded;
        EditorSceneManager.LoadSceneInPlayMode(ScenePath, new LoadSceneParameters(LoadSceneMode.Single));

        GameObject rov = null;
        float loadDeadline = Time.realtimeSinceStartup + 10f;
        while (Time.realtimeSinceStartup < loadDeadline)
        {
            if (sceneLoaded && SceneManager.GetActiveScene().path == ScenePath)
            {
                rov = FindGameObjectByName("FinsROV");
                if (rov != null)
                {
                    break;
                }
            }

            yield return null;
        }

        SceneManager.sceneLoaded -= OnSceneLoaded;
        Assert.NotNull(rov, "FinsROV not found in diagnostic scene.");

        Rigidbody body = rov.GetComponent<Rigidbody>();
        Assert.NotNull(body, "FinsROV Rigidbody not found.");

        DisableManualControllers(rov);
        MonoBehaviour thrusterController = FindComponentByName(rov, "ThrusterController");
        MonoBehaviour[] waterObjects = FindComponentsByName(rov, "WaterObject");
        MonoBehaviour[] providers = FindAllComponentsByName("UnityHDRPWaterDataProvider");
        ApplyModeConfiguration(mode, waterObjects, providers);
        ApplyInitialState(mode, rov, body);

        string safeMode = mode.Replace(" ", "_");
        string csvPath = Path.Combine(outputDir, $"finsrov_diag_{safeMode}.csv");
        string summaryPath = Path.Combine(outputDir, $"finsrov_diag_{safeMode}.summary.txt");

        float startTime = Time.fixedTime;
        float startY = rov.transform.position.y;
        int fixedFrames = 0;
        var csv = new StringBuilder(64 * 1024);
        csv.AppendLine("time,mode,position_x,position_y,position_z,euler_x,euler_y,euler_z,linear_speed,angular_speed,local_yaw_rate,water_force_mag,water_torque_mag,thruster_force_mag,thruster_torque_mag,submerged_volume");

        var summary = new DiagnosticSummary
        {
            Mode = mode,
            StartY = startY,
            WaterObjectsTotal = waterObjects.Length,
        };

        while (Time.fixedTime - startTime < DurationSec)
        {
            if (mode.IndexOf("yaw", StringComparison.OrdinalIgnoreCase) >= 0)
            {
                ApplyYaw(thrusterController);
            }

            yield return new WaitForFixedUpdate();
            fixedFrames++;

            Vector3 localAngularVelocity = rov.transform.InverseTransformDirection(body.angularVelocity);
            Vector3 waterForce = Vector3.zero;
            Vector3 waterTorque = Vector3.zero;
            float submergedVolume = 0f;
            int enabledWaterObjects = 0;

            foreach (MonoBehaviour waterObject in waterObjects)
            {
                if (waterObject == null || !waterObject.enabled)
                {
                    continue;
                }

                enabledWaterObjects++;
                waterForce += GetField<Vector3>(waterObject, "ResultForce");
                waterTorque += GetField<Vector3>(waterObject, "ResultTorque");
                submergedVolume += GetField<float>(waterObject, "submergedVolume");
            }

            GetThrusterNetForceTorque(thrusterController, body, out Vector3 thrusterForce, out Vector3 thrusterTorque);

            float linearSpeed = body.linearVelocity.magnitude;
            float angularSpeed = body.angularVelocity.magnitude;
            float absYawRate = Mathf.Abs(localAngularVelocity.y);
            float rollPitch = MaxAbsRollPitchDeg(rov.transform.eulerAngles);

            summary.MaxLinearSpeed = Mathf.Max(summary.MaxLinearSpeed, linearSpeed);
            summary.MaxAngularSpeed = Mathf.Max(summary.MaxAngularSpeed, angularSpeed);
            summary.MaxAbsLocalYawRate = Mathf.Max(summary.MaxAbsLocalYawRate, absYawRate);
            summary.MaxAbsRollPitchDeg = Mathf.Max(summary.MaxAbsRollPitchDeg, rollPitch);
            summary.MaxWaterForce = Mathf.Max(summary.MaxWaterForce, waterForce.magnitude);
            summary.MaxWaterTorque = Mathf.Max(summary.MaxWaterTorque, waterTorque.magnitude);
            summary.MaxThrusterTorque = Mathf.Max(summary.MaxThrusterTorque, thrusterTorque.magnitude);
            summary.WaterObjectsEnabled = enabledWaterObjects;

            Vector3 euler = rov.transform.eulerAngles;
            csv.AppendFormat(
                CultureInfo.InvariantCulture,
                "{0:F4},{1},{2:F6},{3:F6},{4:F6},{5:F4},{6:F4},{7:F4},{8:F6},{9:F6},{10:F6},{11:F6},{12:F6},{13:F6},{14:F6},{15:F6}\n",
                Time.fixedTime - startTime,
                mode,
                rov.transform.position.x,
                rov.transform.position.y,
                rov.transform.position.z,
                euler.x,
                euler.y,
                euler.z,
                linearSpeed,
                angularSpeed,
                localAngularVelocity.y,
                waterForce.magnitude,
                waterTorque.magnitude,
                thrusterForce.magnitude,
                thrusterTorque.magnitude,
                submergedVolume);
        }

        summary.FixedFrames = fixedFrames;
        summary.ProviderEnabled = AnyEnabled(providers);
        summary.EndY = rov.transform.position.y;
        File.WriteAllText(csvPath, csv.ToString());
        File.WriteAllText(summaryPath, summary.ToText());
        Debug.Log($"FinsROV diagnostic wrote {csvPath} and {summaryPath}");
        onSummary(summary);
    }

    static void DisableManualControllers(GameObject rov)
    {
        foreach (MonoBehaviour component in rov.GetComponents<MonoBehaviour>())
        {
            if (component == null)
            {
                continue;
            }

            string typeName = component.GetType().Name;
            if (typeName.Contains("Manual", StringComparison.OrdinalIgnoreCase) ||
                typeName.Contains("Agent", StringComparison.OrdinalIgnoreCase))
            {
                component.enabled = false;
            }
        }
    }

    static void ApplyModeConfiguration(string mode, MonoBehaviour[] waterObjects, MonoBehaviour[] providers)
    {
        if (mode.IndexOf("noProvider", StringComparison.OrdinalIgnoreCase) >= 0)
        {
            foreach (MonoBehaviour provider in providers)
            {
                provider.enabled = false;
            }
        }

        bool noHydro = mode.IndexOf("noHydro", StringComparison.OrdinalIgnoreCase) >= 0;
        bool noWater = mode.IndexOf("noWater", StringComparison.OrdinalIgnoreCase) >= 0;
        bool mainBodyOnly = mode.IndexOf("mainBodyOnly", StringComparison.OrdinalIgnoreCase) >= 0;

        foreach (MonoBehaviour waterObject in waterObjects)
        {
            if (waterObject == null)
            {
                continue;
            }

            if (noWater)
            {
                waterObject.enabled = false;
            }

            if (noHydro)
            {
                SetField(waterObject, "hydrodynamicForceCoefficient", 0f);
            }

            if (mainBodyOnly && !waterObject.name.Equals("mainbody", StringComparison.OrdinalIgnoreCase))
            {
                waterObject.enabled = false;
            }
        }
    }

    static void ApplyInitialState(string mode, GameObject rov, Rigidbody body)
    {
        if (mode.IndexOf("depth2", StringComparison.OrdinalIgnoreCase) >= 0)
        {
            Vector3 position = rov.transform.position;
            position.y = -2f;
            rov.transform.position = position;
        }

        rov.transform.rotation = Quaternion.identity;
        body.linearVelocity = Vector3.zero;
        body.angularVelocity = Vector3.zero;
    }

    static void ApplyYaw(MonoBehaviour thrusterController)
    {
        if (thrusterController == null)
        {
            return;
        }

        MethodInfo method = thrusterController.GetType().GetMethod("ApplyInput", new[] { typeof(float[]) });
        method?.Invoke(thrusterController, new object[] { new[] { 0f, 0f, 0f, 0f, YawCommand, YawCommand, YawCommand, YawCommand } });
    }

    static void GetThrusterNetForceTorque(MonoBehaviour thrusterController, Rigidbody body, out Vector3 force, out Vector3 torque)
    {
        force = Vector3.zero;
        torque = Vector3.zero;
        if (thrusterController == null)
        {
            return;
        }

        FieldInfo thrustersField = thrusterController.GetType().GetField("thrusters");
        if (thrustersField?.GetValue(thrusterController) is not IEnumerable thrusters)
        {
            return;
        }

        Vector3 origin = body.worldCenterOfMass;
        foreach (object thruster in thrusters)
        {
            if (thruster == null)
            {
                continue;
            }

            Vector3 thrusterForce = GetField<Vector3>(thruster, "LastAppliedWorldForce");
            Vector3 position = GetField<Vector3>(thruster, "LastAppliedForcePosition");
            force += thrusterForce;
            torque += Vector3.Cross(position - origin, thrusterForce);
        }
    }

    static MonoBehaviour FindComponentByName(GameObject root, string typeName)
    {
        foreach (MonoBehaviour component in root.GetComponentsInChildren<MonoBehaviour>(true))
        {
            if (component != null && component.GetType().Name == typeName)
            {
                return component;
            }
        }

        return null;
    }

    static MonoBehaviour[] FindComponentsByName(GameObject root, string typeName)
    {
        var results = new List<MonoBehaviour>();
        foreach (MonoBehaviour component in root.GetComponentsInChildren<MonoBehaviour>(true))
        {
            if (component != null && component.GetType().Name == typeName)
            {
                results.Add(component);
            }
        }

        return results.ToArray();
    }

    static MonoBehaviour[] FindAllComponentsByName(string typeName)
    {
        var results = new List<MonoBehaviour>();
        foreach (MonoBehaviour component in UnityEngine.Object.FindObjectsByType<MonoBehaviour>(FindObjectsInactive.Include, FindObjectsSortMode.None))
        {
            if (component != null && component.GetType().Name == typeName)
            {
                results.Add(component);
            }
        }

        return results.ToArray();
    }

    static GameObject FindGameObjectByName(string objectName)
    {
        foreach (Transform transform in UnityEngine.Object.FindObjectsByType<Transform>(FindObjectsInactive.Include, FindObjectsSortMode.None))
        {
            if (transform != null && transform.name == objectName)
            {
                return transform.gameObject;
            }
        }

        return null;
    }

    static T GetField<T>(object target, string fieldName)
    {
        if (target == null)
        {
            return default;
        }

        FieldInfo field = target.GetType().GetField(fieldName, BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic);
        if (field == null)
        {
            return default;
        }

        object value = field.GetValue(target);
        return value is T typed ? typed : default;
    }

    static void SetField<T>(object target, string fieldName, T value)
    {
        FieldInfo field = target.GetType().GetField(fieldName, BindingFlags.Instance | BindingFlags.Public | BindingFlags.NonPublic);
        field?.SetValue(target, value);
    }

    static bool AnyEnabled(MonoBehaviour[] components)
    {
        foreach (MonoBehaviour component in components)
        {
            if (component != null && component.enabled)
            {
                return true;
            }
        }

        return false;
    }

    static float MaxAbsRollPitchDeg(Vector3 euler)
    {
        return Mathf.Max(Mathf.Abs(Mathf.DeltaAngle(0f, euler.x)), Mathf.Abs(Mathf.DeltaAngle(0f, euler.z)));
    }

    class DiagnosticSummary
    {
        public string Mode;
        public int FixedFrames;
        public int WaterObjectsTotal;
        public int WaterObjectsEnabled;
        public bool ProviderEnabled;
        public float MaxLinearSpeed;
        public float MaxAngularSpeed;
        public float MaxAbsLocalYawRate;
        public float MaxAbsRollPitchDeg;
        public float MaxWaterForce;
        public float MaxWaterTorque;
        public float MaxThrusterTorque;
        public float StartY;
        public float EndY;

        public string ToText()
        {
            var sb = new StringBuilder();
            sb.AppendLine($"mode={Mode}");
            sb.AppendLine($"fixed_frames={FixedFrames}");
            sb.AppendLine($"water_objects_total={WaterObjectsTotal}");
            sb.AppendLine($"water_objects_enabled={WaterObjectsEnabled}");
            sb.AppendLine($"provider_enabled={ProviderEnabled}");
            sb.AppendLine($"max_linear_speed_mps={MaxLinearSpeed.ToString("F6", CultureInfo.InvariantCulture)}");
            sb.AppendLine($"max_angular_speed_radps={MaxAngularSpeed.ToString("F6", CultureInfo.InvariantCulture)}");
            sb.AppendLine($"max_abs_local_yaw_rate_radps={MaxAbsLocalYawRate.ToString("F6", CultureInfo.InvariantCulture)}");
            sb.AppendLine($"max_abs_roll_pitch_deg={MaxAbsRollPitchDeg.ToString("F6", CultureInfo.InvariantCulture)}");
            sb.AppendLine($"max_water_force_n={MaxWaterForce.ToString("F6", CultureInfo.InvariantCulture)}");
            sb.AppendLine($"max_water_torque_nm={MaxWaterTorque.ToString("F6", CultureInfo.InvariantCulture)}");
            sb.AppendLine($"max_thruster_torque_nm={MaxThrusterTorque.ToString("F6", CultureInfo.InvariantCulture)}");
            sb.AppendLine($"start_y={StartY.ToString("F6", CultureInfo.InvariantCulture)}");
            sb.AppendLine($"end_y={EndY.ToString("F6", CultureInfo.InvariantCulture)}");
            sb.AppendLine($"delta_y={(EndY - StartY).ToString("F6", CultureInfo.InvariantCulture)}");
            return sb.ToString();
        }

        public string ToCsv()
        {
            return string.Format(
                CultureInfo.InvariantCulture,
                "{0},{1:F6},{2:F6},{3:F6},{4:F6},{5:F6},{6:F6},{7:F6},{8},{9},{10},{11:F6},{12:F6},{13:F6}",
                Mode,
                MaxLinearSpeed,
                MaxAngularSpeed,
                MaxAbsLocalYawRate,
                MaxAbsRollPitchDeg,
                MaxWaterForce,
                MaxWaterTorque,
                MaxThrusterTorque,
                WaterObjectsTotal,
                WaterObjectsEnabled,
                ProviderEnabled,
                StartY,
                EndY,
                EndY - StartY);
        }
    }
}
