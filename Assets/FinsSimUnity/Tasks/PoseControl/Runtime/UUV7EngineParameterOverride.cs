using NWH.DWP2.ShipController;
using UnityEngine;

[DisallowMultipleComponent]
[RequireComponent(typeof(AdvancedShipController))]
public class UUV7EngineParameterOverride : MonoBehaviour
{
    [Header("Apply Timing")]
    [SerializeField] bool applyOnAwake = true;
    [SerializeField] bool applyOnStart = true;

    [Header("Thrust")]
    [SerializeField] bool overrideMaxThrust = true;
    [SerializeField] float maxThrust = 50f;
    [SerializeField] bool overrideReverseThrustCoefficient;
    [SerializeField] float reverseThrustCoefficient = 0.3f;

    [Header("RPM")]
    [SerializeField] bool overrideMaxRPM;
    [SerializeField] float maxRPM = 3000f;
    [SerializeField] bool overrideMinRPM;
    [SerializeField] float minRPM = 500f;
    [SerializeField] bool overrideMaxSpeed;
    [SerializeField] float maxSpeed = 20f;

    [Header("Response")]
    [SerializeField] bool overrideSpinUpTime;
    [SerializeField] float spinUpTime = 2f;
    [SerializeField] bool overrideStartDuration;
    [SerializeField] float startDuration = 1.3f;
    [SerializeField] bool overrideStopDuration;
    [SerializeField] float stopDuration = 0.8f;

    [Header("Animation And Audio")]
    [SerializeField] bool overridePropellerRpmRatio;
    [SerializeField] float propellerRpmRatio = 5f;
    [SerializeField] bool overridePitch;
    [SerializeField] float pitch = 0.4f;
    [SerializeField] bool overridePitchRange;
    [SerializeField] float pitchRange = 0.6f;

    [Header("Runtime Debug")]
    [SerializeField] int appliedEngineCount;

    AdvancedShipController shipController;

    void Awake()
    {
        ResolveRuntimeReferences();
        if (applyOnAwake)
        {
            ApplyOverrides();
        }
    }

    void Start()
    {
        ResolveRuntimeReferences();
        if (applyOnStart)
        {
            ApplyOverrides();
        }
    }

    [ContextMenu("Apply Engine Overrides")]
    public void ApplyOverrides()
    {
        ResolveRuntimeReferences();
        appliedEngineCount = 0;

        if (shipController == null || shipController.engines == null)
        {
            return;
        }

        foreach (Engine engine in shipController.engines)
        {
            if (engine == null)
            {
                continue;
            }

            ApplyToEngine(engine);
            appliedEngineCount++;
        }
    }

    void ApplyToEngine(Engine engine)
    {
        if (overrideMaxThrust) engine.maxThrust = Mathf.Max(0f, maxThrust);
        if (overrideReverseThrustCoefficient) engine.reverseThrustCoefficient = Mathf.Max(0f, reverseThrustCoefficient);
        if (overrideMaxRPM) engine.maxRPM = Mathf.Max(0f, maxRPM);
        if (overrideMinRPM) engine.minRPM = Mathf.Max(0f, minRPM);
        if (overrideMaxSpeed) engine.maxSpeed = Mathf.Max(0f, maxSpeed);
        if (overrideSpinUpTime) engine.spinUpTime = Mathf.Max(0f, spinUpTime);
        if (overrideStartDuration) engine.startDuration = Mathf.Max(0f, startDuration);
        if (overrideStopDuration) engine.stopDuration = Mathf.Max(0f, stopDuration);
        if (overridePropellerRpmRatio) engine.propellerRpmRatio = propellerRpmRatio;
        if (overridePitch) engine.pitch = Mathf.Max(0f, pitch);
        if (overridePitchRange) engine.pitchRange = Mathf.Max(0f, pitchRange);
    }

    void ResolveRuntimeReferences()
    {
        if (shipController == null)
        {
            shipController = GetComponent<AdvancedShipController>();
        }
    }
}
