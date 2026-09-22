#if UNITY_EDITOR
using System;
using UnityEditor;
using UnityEngine;

[CustomEditor(typeof(CallMethodInChildren))]
public class CallMethodInChildrenInspector : Editor
{
    public override void OnInspectorGUI()
    {
        DrawDefaultInspector();

        if (GUILayout.Button("Call method in children"))
        {
            UpdateChildren();
        }
    }

    private void UpdateChildren()
    {
        var component = (CallMethodInChildren)target;
        if (!String.IsNullOrEmpty(component.callbackName))
        {
            component.gameObject.BroadcastMessage(
                component.callbackName,
                null,
                SendMessageOptions.DontRequireReceiver);
        }
    }
}
#endif
