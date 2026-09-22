#if UNITY_EDITOR
using FinsSim.Networking;
using UnityEditor;
using UnityEngine;

[CustomEditor(typeof(VehicleRosBridge)), CanEditMultipleObjects]
public class VehicleRosBridgeEditor : Editor
{
    public override void OnInspectorGUI()
    {
        serializedObject.Update();

        DrawScriptField();
        DrawSection("Bridge Name", "bridgeName");
        DrawSection("Command Topics", "thrusterTopic", "resetTopic");

        EditorGUILayout.Space();
        EditorGUILayout.LabelField("State Publishing", EditorStyles.boldLabel);
        EditorGUILayout.PropertyField(serializedObject.FindProperty("statePublishMode"));

        var modeProperty = serializedObject.FindProperty("statePublishMode");
        var mode = (VehicleRosStatePublishMode)modeProperty.enumValueIndex;
        if (mode == VehicleRosStatePublishMode.RawFusion || mode == VehicleRosStatePublishMode.RawAndSimTruth)
        {
            DrawSection(
                "Raw Fusion Topics",
                "poseTopic",
                "imuTopic",
                "dvlTopic",
                "depthTopic");
            DrawSection(
                "Raw Fusion Frame IDs",
                "poseFrameId",
                "imuFrameId",
                "dvlFrameId",
                "depthFrameId");
        }

        if (mode == VehicleRosStatePublishMode.SimTruth || mode == VehicleRosStatePublishMode.RawAndSimTruth)
        {
            DrawSection(
                "Sim Truth Topics",
                "controllerPoseTopic",
                "controllerImuTopic",
                "controllerDvlTopic",
                "controllerDepthTopic");
            DrawSection(
                "Sim Truth Frame IDs",
                "controllerWorldFrameId",
                "controllerBodyFrameId");
            DrawSection("Sim Truth Water Origin", "controllerWaterSurfaceY");
        }

        DrawSection(
            "Bridge Switches",
            "subscribeThrusters",
            "subscribeReset",
            "publishPose",
            "publishImu",
            "publishDvl",
            "publishDepth");
        DrawSection(
            "Thruster Command",
            "thrusterCommandMode",
            "autoResolveThrustersFromChildren",
            "zeroThrustersWhenDisabled",
            "logThrusterResolution");
        DrawSection("Control Conflicts", "disableManualControllerOnEnable");
        DrawSection(
            "Publishing Rates",
            "posePublishHz",
            "imuPublishHz",
            "dvlPublishHz",
            "depthPublishHz");
        DrawSection(
            "Queue Sizes",
            "poseQueueSize",
            "imuQueueSize",
            "dvlQueueSize",
            "depthQueueSize");
        DrawSection("IMU Settings", "removeGravityFromAcceleration");
        DrawSection("DWP2 Normalized Fallback", "engineOrder");
        DrawSection("Overrides", "thrusterControllerOverride", "advancedShipControllerOverride");

        serializedObject.ApplyModifiedProperties();
    }

    void DrawScriptField()
    {
        using (new EditorGUI.DisabledScope(true))
        {
            EditorGUILayout.PropertyField(serializedObject.FindProperty("m_Script"));
        }
    }

    void DrawSection(string label, params string[] propertyNames)
    {
        EditorGUILayout.Space();
        EditorGUILayout.LabelField(label, EditorStyles.boldLabel);
        foreach (var propertyName in propertyNames)
        {
            var property = serializedObject.FindProperty(propertyName);
            if (property != null)
            {
                EditorGUILayout.PropertyField(property, true);
            }
        }
    }
}
#endif
