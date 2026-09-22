using System;
using UnityEngine;

/// <summary>
/// Shows a serialized field only for the selected FinsROV thruster-command mode.
/// This keeps mode-specific Inspector settings from suggesting that they affect
/// another action-to-force contract.
/// </summary>
[AttributeUsage(AttributeTargets.Field)]
public sealed class ShowWhenThrusterCommandModeAttribute : PropertyAttribute
{
    public readonly ThrusterCommandMode RequiredMode;

    public ShowWhenThrusterCommandModeAttribute(ThrusterCommandMode requiredMode)
    {
        RequiredMode = requiredMode;
    }
}
