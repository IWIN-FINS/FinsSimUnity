using UnityEditor;
using UnityEngine;

[CustomEditor(typeof(NetConstraintModel))]
public class NetConstraintModelEditor : Editor
{
    public override void OnInspectorGUI()
    {
        serializedObject.Update();
        DrawDefaultInspector();
        serializedObject.ApplyModifiedProperties();

        NetConstraintModel model = (NetConstraintModel)target;

        EditorGUILayout.Space();
        EditorGUILayout.LabelField("Hydrodynamic Frame", EditorStyles.boldLabel);

        if (!model.IsReady())
        {
            EditorGUILayout.HelpBox(
                "NetConstraintModel only resolves the coarse frame used by NetHydrodynamicProxy. Check Layer*/L_* and Horizontal/H_* names.",
                MessageType.Warning);
            return;
        }

        EditorGUILayout.LabelField("Current Geometry", $"W {model.GetNetWidth():F2} / H {model.GetNetHeight():F2}");
        EditorGUILayout.LabelField("Center", FormatVector(model.GetNetCenter()));
    }

    private static string FormatVector(Vector3 value)
    {
        return $"({value.x:F2}, {value.y:F2}, {value.z:F2})";
    }
}
