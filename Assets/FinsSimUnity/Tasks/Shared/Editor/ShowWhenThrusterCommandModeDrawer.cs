using UnityEditor;
using UnityEngine;

[CustomPropertyDrawer(typeof(ShowWhenThrusterCommandModeAttribute))]
public sealed class ShowWhenThrusterCommandModeDrawer : PropertyDrawer
{
    bool ShouldShow(SerializedProperty property)
    {
        SerializedProperty modeProperty = property.serializedObject.FindProperty("thrusterCommandMode");
        if (modeProperty == null || modeProperty.propertyType != SerializedPropertyType.Enum)
        {
            return true;
        }

        var condition = (ShowWhenThrusterCommandModeAttribute)attribute;
        return modeProperty.enumValueIndex == (int)condition.RequiredMode;
    }

    public override float GetPropertyHeight(SerializedProperty property, GUIContent label)
    {
        return ShouldShow(property)
            ? EditorGUI.GetPropertyHeight(property, label, true)
            : -EditorGUIUtility.standardVerticalSpacing;
    }

    public override void OnGUI(Rect position, SerializedProperty property, GUIContent label)
    {
        if (ShouldShow(property))
        {
            EditorGUI.PropertyField(position, property, label, true);
        }
    }
}
