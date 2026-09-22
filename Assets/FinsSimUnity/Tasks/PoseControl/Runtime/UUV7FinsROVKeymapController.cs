using System.Collections.Generic;
using NWH.DWP2.ShipController;
using UnityEngine;

[DisallowMultipleComponent]
[RequireComponent(typeof(AdvancedShipController))]
public class UUV7FinsROVKeymapController : MonoBehaviour
{
    [Header("Manual Control")]
    [SerializeField] bool manualControlEnabled = true;
    [SerializeField] bool applyInFixedUpdate = true;
    [SerializeField] float yawMixScale = 1.0f;
    [SerializeField] float auxMixScale = 1.0f;

    [Header("Runtime Debug")]
    [SerializeField] FinsROVManualInputState currentInputState;
    [SerializeField] float[] currentEngineInput = new float[FinsROVAgentRuntime.DefaultThrusterOrder.Length];

    readonly float[] engineInput = new float[FinsROVAgentRuntime.DefaultThrusterOrder.Length];
    readonly Dictionary<string, Engine> enginesByName = new Dictionary<string, Engine>();
    AdvancedShipController shipController;

    void Awake()
    {
        ResolveRuntimeReferences();
    }

    void OnEnable()
    {
        ResolveRuntimeReferences();
        SetDwp2ExternalInputMode(true);
    }

    void OnDisable()
    {
        SetEngineInputs(0f);
    }

    void Update()
    {
        if (!applyInFixedUpdate)
        {
            ApplyManualControl();
        }
    }

    void FixedUpdate()
    {
        if (applyInFixedUpdate)
        {
            ApplyManualControl();
        }
    }

    void ApplyManualControl()
    {
        if (!manualControlEnabled || !enabled)
        {
            return;
        }

        ResolveRuntimeReferences();
        if (shipController == null)
        {
            return;
        }

        SetDwp2ExternalInputMode(true);
        currentInputState = FinsROVAgentRuntime.ReadKeyboardManualInput();
        FinsROVAgentRuntime.FillThrusterInputFromManualState(
            currentInputState,
            engineInput,
            yawMixScale,
            auxMixScale);

        EnsureDebugArraySize();
        for (int i = 0; i < currentEngineInput.Length; i++)
        {
            currentEngineInput[i] = i < engineInput.Length ? engineInput[i] : 0f;
        }

        for (int i = 0; i < FinsROVAgentRuntime.DefaultThrusterOrder.Length; i++)
        {
            if (enginesByName.TryGetValue(FinsROVAgentRuntime.DefaultThrusterOrder[i], out Engine engine))
            {
                engine.externalThrottleInput = engineInput[i];
            }
        }
    }

    void ResolveRuntimeReferences()
    {
        if (shipController == null)
        {
            shipController = GetComponent<AdvancedShipController>();
        }

        enginesByName.Clear();
        if (shipController == null || shipController.engines == null)
        {
            return;
        }

        foreach (Engine engine in shipController.engines)
        {
            if (engine != null && !enginesByName.ContainsKey(engine.name))
            {
                enginesByName.Add(engine.name, engine);
            }
        }
    }

    void SetDwp2ExternalInputMode(bool useExternalInput)
    {
        if (shipController == null)
        {
            return;
        }

        shipController.input.autoSetInput = !useExternalInput;
        if (shipController.engines == null)
        {
            return;
        }

        foreach (Engine engine in shipController.engines)
        {
            if (engine != null)
            {
                engine.useExternalThrottleInput = useExternalInput;
            }
        }
    }

    void SetEngineInputs(float value)
    {
        if (shipController == null || shipController.engines == null)
        {
            return;
        }

        foreach (Engine engine in shipController.engines)
        {
            if (engine != null)
            {
                engine.externalThrottleInput = value;
            }
        }
    }

    void EnsureDebugArraySize()
    {
        if (currentEngineInput == null || currentEngineInput.Length != FinsROVAgentRuntime.DefaultThrusterOrder.Length)
        {
            currentEngineInput = new float[FinsROVAgentRuntime.DefaultThrusterOrder.Length];
        }
    }
}
