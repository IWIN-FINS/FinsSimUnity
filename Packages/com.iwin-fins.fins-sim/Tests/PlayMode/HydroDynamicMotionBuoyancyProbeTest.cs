using System;
using System.Collections;
using System.Globalization;
using System.IO;
using System.Reflection;
using System.Text;
using FinsSim.Hydrodynamics;
using NUnit.Framework;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;
using UnityEngine.TestTools;

public class HydroDynamicMotionBuoyancyProbeTest
{
    const string ScenePath = "Assets/FinsSimUnity/Tasks/PlatformExamples/Scenes/HydroDynamicMotion.unity";
    const float DurationSec = 3f;

    [UnityTest]
    public IEnumerator ProbeFossen6DofDwp2BuoyancyBalance()
    {
        LogAssert.ignoreFailingMessages = true;

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
        float deadline = Time.realtimeSinceStartup + 10f;
        while (Time.realtimeSinceStartup < deadline)
        {
            if (sceneLoaded && SceneManager.GetActiveScene().path == ScenePath)
            {
                rov = FindFinsRov();
                if (rov != null)
                {
                    break;
                }
            }

            yield return null;
        }

        SceneManager.sceneLoaded -= OnSceneLoaded;
        Assert.NotNull(rov, "FinsROV not found in HydroDynamicMotion scene.");

        Rigidbody body = rov.GetComponent<Rigidbody>();
        Assert.NotNull(body, "FinsROV Rigidbody not found.");

        HydrodynamicsController hydro = rov.GetComponent<HydrodynamicsController>();
        Assert.NotNull(hydro, "HydrodynamicsController not found on FinsROV.");

        hydro.mode = HydrodynamicsMode.Fossen6Dof;
        body.linearVelocity = Vector3.zero;
        body.angularVelocity = Vector3.zero;
        rov.transform.position = new Vector3(rov.transform.position.x, -3f, rov.transform.position.z);
        rov.transform.rotation = Quaternion.identity;

        MonoBehaviour[] waterObjects = FindComponentsByName(rov, "WaterObject");
        var csv = new StringBuilder();
        csv.AppendLine("time,y,vy,total_dwp2_force_y,mainbody_dwp2_force_y,hydro_force_y,gravity_force_y,net_y,total_submerged_volume,mainbody_submerged_volume,enabled_water_objects");

        float startTime = Time.fixedTime;
        float startY = rov.transform.position.y;
        float endY = startY;
        Vector3 lastDwp2Force = Vector3.zero;
        Vector3 lastMainBodyForce = Vector3.zero;
        Vector3 lastHydroForce = Vector3.zero;
        float lastTotalSubmerged = 0f;
        float lastMainBodySubmerged = 0f;
        int lastEnabledWaterObjects = 0;

        while (Time.fixedTime - startTime < DurationSec)
        {
            yield return new WaitForFixedUpdate();

            AggregateWaterObjects(
                waterObjects,
                out Vector3 dwp2Force,
                out float totalSubmerged,
                out Vector3 mainBodyForce,
                out float mainBodySubmerged,
                out int enabledWaterObjects);

            lastDwp2Force = dwp2Force;
            lastMainBodyForce = mainBodyForce;
            lastHydroForce = hydro.LastWrench.WorldForce;
            lastTotalSubmerged = totalSubmerged;
            lastMainBodySubmerged = mainBodySubmerged;
            lastEnabledWaterObjects = enabledWaterObjects;
            endY = rov.transform.position.y;

            float gravityY = Physics.gravity.y * body.mass;
            csv.AppendFormat(
                CultureInfo.InvariantCulture,
                "{0:F4},{1:F6},{2:F6},{3:F6},{4:F6},{5:F6},{6:F6},{7:F6},{8:F6},{9:F6},{10}\n",
                Time.fixedTime - startTime,
                rov.transform.position.y,
                body.linearVelocity.y,
                dwp2Force.y,
                mainBodyForce.y,
                hydro.LastWrench.WorldForce.y,
                gravityY,
                dwp2Force.y + hydro.LastWrench.WorldForce.y + gravityY,
                totalSubmerged,
                mainBodySubmerged,
                enabledWaterObjects);
        }

        string outputDir = "/home/fins/UnderwaterSim/Code/FinsSim/ros2_ws/data/unity_hydrodynamic_benchmark/hydro_dynamic_motion_buoyancy_probe";
        Directory.CreateDirectory(outputDir);
        string csvPath = Path.Combine(outputDir, "hydro_dynamic_motion_buoyancy_probe.csv");
        string summaryPath = Path.Combine(outputDir, "hydro_dynamic_motion_buoyancy_probe.summary.txt");
        File.WriteAllText(csvPath, csv.ToString());
        File.WriteAllText(
            summaryPath,
            string.Format(
                CultureInfo.InvariantCulture,
                "start_y={0:F6}\nend_y={1:F6}\ndelta_y={2:F6}\nrigidbody_mass_kg={3:F6}\ngravity_force_y={4:F6}\nlast_total_dwp2_force_y={5:F6}\nlast_mainbody_dwp2_force_y={6:F6}\nlast_hydro_force_y={7:F6}\nlast_net_y={8:F6}\nlast_total_submerged_volume={9:F6}\nlast_mainbody_submerged_volume={10:F6}\nlast_enabled_water_objects={11}\n",
                startY,
                endY,
                endY - startY,
                body.mass,
                Physics.gravity.y * body.mass,
                lastDwp2Force.y,
                lastMainBodyForce.y,
                lastHydroForce.y,
                lastDwp2Force.y + lastHydroForce.y + Physics.gravity.y * body.mass,
                lastTotalSubmerged,
                lastMainBodySubmerged,
                lastEnabledWaterObjects));

        Debug.Log($"HydroDynamicMotion buoyancy probe wrote {csvPath} and {summaryPath}");
    }

    static GameObject FindFinsRov()
    {
        GameObject named = FindActiveGameObjectByName("FinsROV");
        if (named != null)
        {
            return named;
        }

        foreach (HydrodynamicsController controller in UnityEngine.Object.FindObjectsByType<HydrodynamicsController>(FindObjectsInactive.Exclude, FindObjectsSortMode.None))
        {
            if (controller != null && controller.GetComponent<Rigidbody>() != null)
            {
                return controller.gameObject;
            }
        }

        return null;
    }

    static GameObject FindActiveGameObjectByName(string objectName)
    {
        foreach (Transform transform in UnityEngine.Object.FindObjectsByType<Transform>(FindObjectsInactive.Exclude, FindObjectsSortMode.None))
        {
            if (transform != null && transform.name == objectName)
            {
                return transform.gameObject;
            }
        }

        return null;
    }

    static MonoBehaviour[] FindComponentsByName(GameObject root, string typeName)
    {
        var components = root.GetComponentsInChildren<MonoBehaviour>(true);
        int count = 0;
        for (int i = 0; i < components.Length; i++)
        {
            if (components[i] != null && components[i].GetType().Name == typeName)
            {
                count++;
            }
        }

        var results = new MonoBehaviour[count];
        int index = 0;
        for (int i = 0; i < components.Length; i++)
        {
            if (components[i] != null && components[i].GetType().Name == typeName)
            {
                results[index++] = components[i];
            }
        }

        return results;
    }

    static void AggregateWaterObjects(
        MonoBehaviour[] waterObjects,
        out Vector3 totalForce,
        out float totalSubmerged,
        out Vector3 mainBodyForce,
        out float mainBodySubmerged,
        out int enabledWaterObjects)
    {
        totalForce = Vector3.zero;
        totalSubmerged = 0f;
        mainBodyForce = Vector3.zero;
        mainBodySubmerged = 0f;
        enabledWaterObjects = 0;

        for (int i = 0; i < waterObjects.Length; i++)
        {
            MonoBehaviour waterObject = waterObjects[i];
            if (waterObject == null || !waterObject.isActiveAndEnabled)
            {
                continue;
            }

            enabledWaterObjects++;
            Vector3 force = GetField<Vector3>(waterObject, "ResultForce");
            float submerged = GetField<float>(waterObject, "submergedVolume");
            totalForce += force;
            totalSubmerged += submerged;

            if (waterObject.name.Equals("mainbody", StringComparison.OrdinalIgnoreCase))
            {
                mainBodyForce += force;
                mainBodySubmerged += submerged;
            }
        }
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
}
