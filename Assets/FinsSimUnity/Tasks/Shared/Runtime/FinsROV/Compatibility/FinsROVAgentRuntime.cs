using System;
using System.Collections.Generic;
using FinsSim.Hydrodynamics;
using FinsSim.Actuators;
using UnityEngine;

[Serializable]
public struct FinsROVManualInputState
{
    public float Surge;
    public float Heave;
    public float Yaw;
    public float Aux3;
    public float Aux4;
}

public enum ThrusterCommandMode
{
    NormalizedInput,
    ScaledForceRequest,
    // Maps [-1, 1] directly to this Thruster's configured force limits before
    // its own delay, first-order response, and slew-rate model are applied.
    NormalizedMaxForceRequest
}

public static class FinsROVAgentRuntime
{
    static int standaloneRandomizationEpisodeIndex;

    public static readonly string[] DefaultThrusterOrder =
    {
        "Vertical1",
        "Vertical2",
        "Vertical3",
        "Vertical4",
        "Horizontal1",
        "Horizontal2",
        "Horizontal3",
        "Horizontal4",
    };

    public static Transform ResolveReferenceTransform(Transform rootTransform, Transform explicitReferenceTransform)
    {
        if (explicitReferenceTransform != null)
        {
            return explicitReferenceTransform;
        }

        foreach (Transform childTransform in rootTransform.GetComponentsInChildren<Transform>(true))
        {
            if (childTransform.name == "MarkPosition")
            {
                return childTransform;
            }
        }

        return rootTransform;
    }

    public static bool TryResolveOrderedThrusters(
        Component owner,
        ThrusterController explicitController,
        bool autoResolveFromChildren,
        Thruster[] orderedThrusters,
        out string statusMessage)
    {
        Array.Clear(orderedThrusters, 0, orderedThrusters.Length);

        var uniqueThrusters = new List<Thruster>();
        var thrustersByName = new Dictionary<string, Thruster>(StringComparer.Ordinal);

        void AddThruster(Thruster thruster)
        {
            if (thruster == null || uniqueThrusters.Contains(thruster))
            {
                return;
            }

            uniqueThrusters.Add(thruster);
            if (!thrustersByName.ContainsKey(thruster.name))
            {
                thrustersByName.Add(thruster.name, thruster);
            }
        }

        ThrusterController controller = explicitController;
        if (controller == null)
        {
            controller = owner.GetComponent<ThrusterController>();
        }
        if (controller == null)
        {
            controller = owner.GetComponentInChildren<ThrusterController>(true);
        }

        if (controller != null && controller.thrusters != null)
        {
            foreach (Thruster thruster in controller.thrusters)
            {
                AddThruster(thruster);
            }
        }

        if (uniqueThrusters.Count == 0 && autoResolveFromChildren)
        {
            foreach (Thruster thruster in owner.GetComponentsInChildren<Thruster>(true))
            {
                AddThruster(thruster);
            }
        }

        if (uniqueThrusters.Count == 0)
        {
            statusMessage = "No Thruster components found under the agent root.";
            return false;
        }

        var usedThrusters = new HashSet<Thruster>();
        for (int i = 0; i < DefaultThrusterOrder.Length && i < orderedThrusters.Length; i++)
        {
            if (thrustersByName.TryGetValue(DefaultThrusterOrder[i], out Thruster namedThruster))
            {
                orderedThrusters[i] = namedThruster;
                usedThrusters.Add(namedThruster);
            }
        }

        int fallbackIndex = 0;
        for (int i = 0; i < orderedThrusters.Length; i++)
        {
            if (orderedThrusters[i] != null)
            {
                continue;
            }

            while (fallbackIndex < uniqueThrusters.Count && usedThrusters.Contains(uniqueThrusters[fallbackIndex]))
            {
                fallbackIndex++;
            }

            if (fallbackIndex >= uniqueThrusters.Count)
            {
                break;
            }

            orderedThrusters[i] = uniqueThrusters[fallbackIndex];
            usedThrusters.Add(uniqueThrusters[fallbackIndex]);
            fallbackIndex++;
        }

        int resolvedCount = 0;
        int resolvedByNameCount = 0;
        for (int i = 0; i < orderedThrusters.Length; i++)
        {
            if (orderedThrusters[i] != null)
            {
                resolvedCount++;
            }

            if (i < DefaultThrusterOrder.Length &&
                orderedThrusters[i] != null &&
                orderedThrusters[i].name == DefaultThrusterOrder[i])
            {
                resolvedByNameCount++;
            }
        }

        statusMessage = $"Resolved {resolvedCount}/{orderedThrusters.Length} thrusters ({resolvedByNameCount} by canonical name).";
        return resolvedCount == orderedThrusters.Length;
    }

    public static void ApplyThrusterActions(
        Thruster[] orderedThrusters,
        float[] normalizedActions,
        ThrusterCommandMode commandMode)
    {
        ApplyThrusterActions(orderedThrusters, normalizedActions, commandMode, 1.0f);
    }

    public static void ApplyThrusterActions(
        Thruster[] orderedThrusters,
        float[] normalizedActions,
        ThrusterCommandMode commandMode,
        float forceScaleN)
    {
        int actionCount = normalizedActions != null ? normalizedActions.Length : 0;
        for (int i = 0; i < orderedThrusters.Length; i++)
        {
            float action = i < actionCount ? normalizedActions[i] : 0f;
            ApplyThrusterAction(orderedThrusters[i], action, commandMode, forceScaleN);
        }
    }

    public static void EnsureThrusterControllerOrder(ThrusterController controller, Thruster[] orderedThrusters)
    {
        if (controller == null || orderedThrusters == null)
        {
            return;
        }

        if (controller.thrusters == null)
        {
            controller.thrusters = new List<Thruster>();
        }

        controller.thrusters.Clear();
        for (int i = 0; i < orderedThrusters.Length; i++)
        {
            if (orderedThrusters[i] != null)
            {
                controller.thrusters.Add(orderedThrusters[i]);
            }
        }
    }

    public static FinsROVManualInputState ReadKeyboardManualInput()
    {
        return new FinsROVManualInputState
        {
            Surge = ReadSignedKeys(KeyCode.W, KeyCode.S),
            Heave = ReadSignedKeys(KeyCode.Alpha6, KeyCode.Alpha5, KeyCode.Keypad6, KeyCode.Keypad5),
            Yaw = ReadSignedKeys(KeyCode.D, KeyCode.A),
            Aux3 = ReadSignedKeys(KeyCode.Alpha8, KeyCode.Alpha7, KeyCode.Keypad8, KeyCode.Keypad7),
            Aux4 = ReadSignedKeys(KeyCode.Alpha0, KeyCode.Alpha9, KeyCode.Keypad0, KeyCode.Keypad9),
        };
    }

    public static void FillThrusterInputFromManualState(
        FinsROVManualInputState inputState,
        float[] thrusterInput,
        float yawMixScale = 0.1f,
        float auxMixScale = 1.0f)
    {
        if (thrusterInput == null)
        {
            return;
        }

        Array.Clear(thrusterInput, 0, thrusterInput.Length);

        float surge = Mathf.Clamp(inputState.Surge, -1f, 1f);
        float heave = Mathf.Clamp(inputState.Heave, -1f, 1f);
        float yaw = Mathf.Clamp(inputState.Yaw, -1f, 1f) * yawMixScale;
        float pitchTrim = Mathf.Clamp(inputState.Aux3, -1f, 1f) * auxMixScale;
        float rollTrim = Mathf.Clamp(inputState.Aux4, -1f, 1f) * auxMixScale;

        if (thrusterInput.Length > 0) thrusterInput[0] = Mathf.Clamp(heave + pitchTrim + rollTrim, -1f, 1f); // Vertical1 front-right
        if (thrusterInput.Length > 1) thrusterInput[1] = Mathf.Clamp(heave + pitchTrim - rollTrim, -1f, 1f); // Vertical2 front-left
        if (thrusterInput.Length > 2) thrusterInput[2] = Mathf.Clamp(heave - pitchTrim - rollTrim, -1f, 1f); // Vertical3 rear-left
        if (thrusterInput.Length > 3) thrusterInput[3] = Mathf.Clamp(heave - pitchTrim + rollTrim, -1f, 1f); // Vertical4 rear-right

        if (thrusterInput.Length > 4) thrusterInput[4] = Mathf.Clamp(surge + yaw, -1f, 1f); // Horizontal1
        if (thrusterInput.Length > 5) thrusterInput[5] = Mathf.Clamp(surge + yaw, -1f, 1f); // Horizontal2
        if (thrusterInput.Length > 6) thrusterInput[6] = Mathf.Clamp(-surge + yaw, -1f, 1f); // Horizontal3
        if (thrusterInput.Length > 7) thrusterInput[7] = Mathf.Clamp(-surge + yaw, -1f, 1f); // Horizontal4
    }

    public static void WriteHeuristicActionsFromManualInput(
        Unity.MLAgents.Actuators.ActionSegment<float> actionSegment,
        float[] scratchActions,
        float yawMixScale = 0.1f,
        float auxMixScale = 1.0f)
    {
        if (scratchActions == null)
        {
            return;
        }

        FillThrusterInputFromManualState(ReadKeyboardManualInput(), scratchActions, yawMixScale, auxMixScale);
        int copyCount = Mathf.Min(actionSegment.Length, scratchActions.Length);
        for (int i = 0; i < actionSegment.Length; i++)
        {
            actionSegment[i] = i < copyCount ? scratchActions[i] : 0f;
        }
    }

    public static void ZeroThrusters(
        Thruster[] orderedThrusters,
        ThrusterCommandMode commandMode)
    {
        ZeroThrusters(orderedThrusters, commandMode, 1.0f);
    }

    public static void ZeroThrusters(
        Thruster[] orderedThrusters,
        ThrusterCommandMode commandMode,
        float forceScaleN)
    {
        for (int i = 0; i < orderedThrusters.Length; i++)
        {
            ApplyThrusterAction(orderedThrusters[i], 0f, commandMode, forceScaleN);
        }
    }

    public static void RandomizeEpisodeIfPresent(Component owner)
    {
        if (owner == null)
        {
            return;
        }

        DomainRandomizationCoordinator coordinator = owner.GetComponentInParent<DomainRandomizationCoordinator>();
        if (coordinator == null)
        {
            coordinator = UnityEngine.Object.FindFirstObjectByType<DomainRandomizationCoordinator>();
        }

        if (coordinator != null)
        {
            coordinator.RandomizeForEpisode();
            return;
        }

        RandomizeStandaloneTargets(owner);
    }

    static void RandomizeStandaloneTargets(Component owner)
    {
        Transform root = owner.transform.root != null ? owner.transform.root : owner.transform;
        MonoBehaviour[] behaviours = root.GetComponentsInChildren<MonoBehaviour>(true);
        if (behaviours == null || behaviours.Length == 0)
        {
            return;
        }

        int seed = 41023 + standaloneRandomizationEpisodeIndex;
        var context = new RandomizationContext(null, DomainRandomizationMode.Train, seed, standaloneRandomizationEpisodeIndex);
        var visited = new HashSet<IEpisodeRandomizable>();

        for (int i = 0; i < behaviours.Length; i++)
        {
            if (behaviours[i] is IEpisodeRandomizable target && visited.Add(target))
            {
                target.RandomizeForEpisode(context);
            }
        }

        standaloneRandomizationEpisodeIndex++;
    }

    public static Vector3 GetLocalLinearVelocity(Transform referenceTransform, Rigidbody rigidBody)
    {
        if (referenceTransform == null || rigidBody == null)
        {
            return Vector3.zero;
        }

        return referenceTransform.InverseTransformDirection(rigidBody.linearVelocity);
    }

    public static Vector3 GetLocalAngularVelocity(Transform referenceTransform, Rigidbody rigidBody)
    {
        if (referenceTransform == null || rigidBody == null)
        {
            return Vector3.zero;
        }

        return referenceTransform.InverseTransformDirection(rigidBody.angularVelocity);
    }

    public static Vector3 GetWorldLinearVelocity(Transform referenceTransform, Vector3 localLinearVelocity)
    {
        if (referenceTransform == null)
        {
            return localLinearVelocity;
        }

        return referenceTransform.TransformDirection(localLinearVelocity);
    }

    public static Vector3 GetWorldAngularVelocity(Transform referenceTransform, Vector3 localAngularVelocity)
    {
        if (referenceTransform == null)
        {
            return localAngularVelocity;
        }

        return referenceTransform.TransformDirection(localAngularVelocity);
    }

    static void ApplyThrusterAction(
        Thruster thruster,
        float normalizedAction,
        ThrusterCommandMode commandMode,
        float forceScaleN)
    {
        if (thruster == null)
        {
            return;
        }

        float clampedAction = Mathf.Clamp(normalizedAction, -1f, 1f);
        if (commandMode == ThrusterCommandMode.ScaledForceRequest)
        {
            thruster.ApplyForceRequest(clampedAction * Mathf.Max(0f, forceScaleN));
            return;
        }

        if (commandMode == ThrusterCommandMode.NormalizedMaxForceRequest)
        {
            float maxForceN = clampedAction >= 0f
                ? Mathf.Abs(thruster.MaxForwardForceN)
                : Mathf.Abs(thruster.MaxReverseForceN);
            thruster.ApplyForceRequest(clampedAction * maxForceN);
            return;
        }

        thruster.ApplyInput(clampedAction);
    }

    static float ReadSignedKeys(KeyCode positiveKey, KeyCode negativeKey, KeyCode altPositiveKey = KeyCode.None, KeyCode altNegativeKey = KeyCode.None)
    {
        float positive = Input.GetKey(positiveKey) || (altPositiveKey != KeyCode.None && Input.GetKey(altPositiveKey)) ? 1f : 0f;
        float negative = Input.GetKey(negativeKey) || (altNegativeKey != KeyCode.None && Input.GetKey(altNegativeKey)) ? 1f : 0f;
        return positive - negative;
    }
}
