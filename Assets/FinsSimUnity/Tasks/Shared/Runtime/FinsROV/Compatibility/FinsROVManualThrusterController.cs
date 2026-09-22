using FinsSim.Actuators;
using UnityEngine;

[DisallowMultipleComponent]
public class FinsROVManualThrusterController : MonoBehaviour
{
    [Header("Manual Control")]
    [SerializeField] bool manualControlEnabled = true;
    [SerializeField] bool applyInFixedUpdate = true;
    [Tooltip("When enabled, manual control writes zero thruster commands every frame while no keys are pressed. Keep disabled so idle manual control does not overwrite ROS/RL commands.")]
    [SerializeField] bool applyZeroWhenIdle;
    [SerializeField] float yawMixScale = 0.1f;
    [SerializeField] float auxMixScale = 1.0f;
    [SerializeField] bool autoResolveThrustersFromChildren = true;
    [SerializeField] bool logThrusterResolution = true;

    [Header("Overrides")]
    [SerializeField] ThrusterController thrusterControllerOverride;

    [Header("Runtime Debug")]
    [SerializeField] FinsROVManualInputState currentInputState;
    [SerializeField] float[] currentThrusterInput = new float[FinsROVAgentRuntime.DefaultThrusterOrder.Length];

    readonly Thruster[] orderedThrusters = new Thruster[FinsROVAgentRuntime.DefaultThrusterOrder.Length];
    readonly float[] manualThrusterInput = new float[FinsROVAgentRuntime.DefaultThrusterOrder.Length];

    ThrusterController thrusterController;
    bool wasManualInputActive;

    void Awake()
    {
        ResolveRuntimeReferences();
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
        if (thrusterController == null)
        {
            return;
        }

        currentInputState = FinsROVAgentRuntime.ReadKeyboardManualInput();
        FinsROVAgentRuntime.FillThrusterInputFromManualState(
            currentInputState,
            manualThrusterInput,
            yawMixScale,
            auxMixScale);
        bool manualInputActive = HasNonZeroInput(manualThrusterInput);
        if (!applyZeroWhenIdle && !manualInputActive && !wasManualInputActive)
        {
            UpdateDebugInput();
            return;
        }

        EnsureDebugArraySize();
        UpdateDebugInput();
        thrusterController.ApplyInput(manualThrusterInput);
        wasManualInputActive = manualInputActive;
    }

    void ResolveRuntimeReferences()
    {
        if (thrusterController == null)
        {
            thrusterController = thrusterControllerOverride != null
                ? thrusterControllerOverride
                : GetComponent<ThrusterController>();
        }

        if (FinsROVAgentRuntime.TryResolveOrderedThrusters(
            this,
            thrusterController,
            autoResolveThrustersFromChildren,
            orderedThrusters,
            out string statusMessage))
        {
            FinsROVAgentRuntime.EnsureThrusterControllerOrder(thrusterController, orderedThrusters);
            if (logThrusterResolution)
            {
                Debug.Log($"[{nameof(FinsROVManualThrusterController)}] {statusMessage}", this);
                logThrusterResolution = false;
            }
        }
        else
        {
            Debug.LogError($"[{nameof(FinsROVManualThrusterController)}] {statusMessage}", this);
        }
    }

    void EnsureDebugArraySize()
    {
        if (currentThrusterInput == null || currentThrusterInput.Length != FinsROVAgentRuntime.DefaultThrusterOrder.Length)
        {
            currentThrusterInput = new float[FinsROVAgentRuntime.DefaultThrusterOrder.Length];
        }
    }

    void UpdateDebugInput()
    {
        EnsureDebugArraySize();
        for (int i = 0; i < currentThrusterInput.Length; i++)
        {
            currentThrusterInput[i] = i < manualThrusterInput.Length ? manualThrusterInput[i] : 0f;
        }
    }

    static bool HasNonZeroInput(float[] values)
    {
        if (values == null)
        {
            return false;
        }

        for (int i = 0; i < values.Length; i++)
        {
            if (Mathf.Abs(values[i]) > 1e-4f)
            {
                return true;
            }
        }

        return false;
    }
}
