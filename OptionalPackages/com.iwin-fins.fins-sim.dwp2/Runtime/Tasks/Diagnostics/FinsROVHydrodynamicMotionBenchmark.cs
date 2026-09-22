using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using FinsSim.Hydrodynamics;
using FinsSim.Core.Spatial;
using UnityEngine;

[DisallowMultipleComponent]
[RequireComponent(typeof(Rigidbody))]
public class FinsROVHydrodynamicMotionBenchmark : MonoBehaviour
{
    public enum SixDofBenchmarkAxis
    {
        SurgeU = 0,
        SwayV = 1,
        HeaveW = 2,
        RollP = 3,
        PitchQ = 4,
        YawR = 5,
    }

    [Serializable]
    public class AxisTrialConfig
    {
        public bool enabled = true;
        public SixDofBenchmarkAxis axis = SixDofBenchmarkAxis.SurgeU;
        [Tooltip("N for Surge/Sway/Heave, Nm for Roll/Pitch/Yaw.")]
        public float amplitude = 1f;
        public bool includeNegative = true;
        [Min(0f)] public float baselineSeconds = 1f;
        [Min(0f)] public float excitationSeconds = 6f;
        [Min(0f)] public float restSeconds = 2f;
    }

    struct ComponentState
    {
        public MonoBehaviour Component;
        public bool Enabled;
    }

    [Header("References")]
    [SerializeField] Rigidbody targetRigidbody;
    [SerializeField] HydrodynamicsController hydrodynamicsController;
    [SerializeField] Transform bodyFrame;

    [Header("Backends")]
    [SerializeField] List<HydrodynamicsMode> backendModes = new List<HydrodynamicsMode>
    {
        HydrodynamicsMode.Fossen6Dof,
        HydrodynamicsMode.SurfaceGeometryHydro,
    };
    [SerializeField] bool disableLegacyDwp2HydroDuringBenchmark = true;

    [Header("Execution")]
    [SerializeField] bool autoRunOnStart;
    [SerializeField] bool resetPoseEachTrial = true;
    [SerializeField] Vector3 resetPosition = Vector3.zero;
    [SerializeField] Vector3 resetEulerAngles = Vector3.zero;
    [SerializeField] bool captureInitialPoseOnStart = true;
    [SerializeField] MonoBehaviour[] componentsToDisableDuringBenchmark;
    [SerializeField] bool verbose = true;

    [Header("Output")]
    [SerializeField] string outputDirectory = "/home/fins/UnderwaterSim/Code/FinsSim/ros2_ws/data/unity_hydrodynamic_benchmark";
    [SerializeField] string outputPrefix = "unity_hydro_backend_benchmark";
    [SerializeField] bool flushAtEndOfEachTrial = false;

    [Header("Trials")]
    [SerializeField] List<AxisTrialConfig> trials = new List<AxisTrialConfig>
    {
        new AxisTrialConfig { axis = SixDofBenchmarkAxis.SurgeU, amplitude = 6.0f, includeNegative = true },
        new AxisTrialConfig { axis = SixDofBenchmarkAxis.SwayV, amplitude = 4.5f, includeNegative = true },
        new AxisTrialConfig { axis = SixDofBenchmarkAxis.HeaveW, amplitude = 4.5f, includeNegative = true },
        new AxisTrialConfig { axis = SixDofBenchmarkAxis.RollP, amplitude = 0.18f, includeNegative = true, excitationSeconds = 2f },
        new AxisTrialConfig { axis = SixDofBenchmarkAxis.PitchQ, amplitude = 0.18f, includeNegative = true, excitationSeconds = 2f },
        new AxisTrialConfig { axis = SixDofBenchmarkAxis.YawR, amplitude = 0.60f, includeNegative = true },
    };

    readonly StringBuilder _samplesCsv = new StringBuilder(1024 * 1024);
    readonly StringBuilder _summaryCsv = new StringBuilder(64 * 1024);
    readonly List<ComponentState> _disabledComponentStates = new List<ComponentState>();
    readonly float[] _trialMaxAbsNu = new float[6];
    readonly float[] _trialMaxAbsNuDot = new float[6];
    readonly float[] _trialFinalNu = new float[6];
    readonly float[] _trialFinalNuDot = new float[6];

    Coroutine _runRoutine;
    HydrodynamicsMode _originalMode;
    bool _originalSuppressDwp2Interop;
    bool _hasOriginalControllerState;
    bool _isLogging;
    bool _applyActiveCommand;
    bool _hasPreviousVelocity;
    float _phaseStartTime;
    float _runStartTime;
    int _trialIndex;
    string _activeBackend = "none";
    string _activeAxis = "none";
    string _activePhase = "idle";
    float _activeLevel;
    SixDofVector _activeCommand = SixDofVector.Zero;
    SixDofVector _previousVelocity = SixDofVector.Zero;
    SixDofVector _latestAcceleration = SixDofVector.Zero;
    float _trialMaxHydroForce;
    float _trialMaxHydroTorque;
    string _samplesPath;
    string _summaryPath;

    protected virtual HydrodynamicsMode[] DefaultBackendModes => new[]
    {
        HydrodynamicsMode.Fossen6Dof,
        HydrodynamicsMode.SurfaceGeometryHydro,
    };

    protected virtual void Reset()
    {
        ResolveReferences();
        ApplyDefaultBackendModes();
        resetPosition = transform.position;
        resetEulerAngles = transform.eulerAngles;
    }

    void Awake()
    {
        ResolveReferences();
    }

    void Start()
    {
        if (captureInitialPoseOnStart)
        {
            resetPosition = transform.position;
            resetEulerAngles = transform.eulerAngles;
        }

        if (autoRunOnStart)
        {
            RunBenchmark();
        }
    }

    void OnDisable()
    {
        _applyActiveCommand = false;
        _isLogging = false;
        RestoreDisabledComponents();
        RestoreHydrodynamicsControllerState();
    }

    void FixedUpdate()
    {
        if (targetRigidbody == null)
        {
            return;
        }

        if (_applyActiveCommand)
        {
            ApplyCommandWrench(_activeCommand);
        }

        SixDofVector velocity = HydroMath.UnityWorldVelocityToFossen(
            BenchmarkFrame,
            BenchmarkAxisConvention,
            targetRigidbody.linearVelocity,
            targetRigidbody.angularVelocity);
        float dt = Mathf.Max(Time.fixedDeltaTime, 1e-5f);
        _latestAcceleration = _hasPreviousVelocity ? (velocity - _previousVelocity) / dt : SixDofVector.Zero;
        _previousVelocity = velocity;
        _hasPreviousVelocity = true;

        if (_isLogging)
        {
            RecordSample(velocity, _latestAcceleration);
        }
    }

    [ContextMenu("Run Benchmark")]
    public void RunBenchmark()
    {
        if (_runRoutine != null)
        {
            StopCoroutine(_runRoutine);
        }

        ResolveReferences();
        if (!ValidateSetup())
        {
            return;
        }

        _runRoutine = StartCoroutine(RunBenchmarkCoroutine());
    }

    [ContextMenu("Stop Benchmark")]
    public void StopBenchmark()
    {
        if (_runRoutine != null)
        {
            StopCoroutine(_runRoutine);
            _runRoutine = null;
        }

        _activeCommand = SixDofVector.Zero;
        _applyActiveCommand = false;
        _isLogging = false;
        WriteOutputs();
        RestoreDisabledComponents();
        RestoreHydrodynamicsControllerState();
    }

    public void SetSingleBackend(HydrodynamicsMode mode)
    {
        backendModes.Clear();
        backendModes.Add(mode);
    }

    public void ResolveReferences()
    {
        if (targetRigidbody == null)
        {
            targetRigidbody = GetComponent<Rigidbody>();
        }

        if (hydrodynamicsController == null)
        {
            hydrodynamicsController = GetComponent<HydrodynamicsController>();
        }

        if (hydrodynamicsController == null)
        {
            hydrodynamicsController = GetComponentInChildren<HydrodynamicsController>(true);
        }

        if (bodyFrame == null)
        {
            bodyFrame = transform;
        }
    }

    protected void ApplyDefaultBackendModes()
    {
        backendModes.Clear();
        backendModes.AddRange(DefaultBackendModes);
    }

    IEnumerator RunBenchmarkCoroutine()
    {
        BeginOutput();
        DisableConfiguredComponents();
        CaptureHydrodynamicsControllerState();
        _runStartTime = Time.fixedTime;
        _trialIndex = 0;

        foreach (HydrodynamicsMode backendMode in backendModes)
        {
            if (!IsBenchmarkBackend(backendMode))
            {
                Debug.LogWarning($"[FinsROVHydrodynamicMotionBenchmark] Skipping unsupported backend {backendMode}.");
                continue;
            }

            ConfigureBackend(backendMode);
            WarnIfSurfaceBackendHasNoMesh(backendMode);

            foreach (AxisTrialConfig config in trials)
            {
                if (config == null || !config.enabled || Mathf.Approximately(config.amplitude, 0f))
                {
                    continue;
                }

                foreach (float level in ExpandLevels(config))
                {
                    _trialIndex++;
                    yield return RunTrial(backendMode, config, level);
                }
            }
        }

        _activeCommand = SixDofVector.Zero;
        _applyActiveCommand = false;
        _isLogging = false;
        WriteOutputs();
        RestoreDisabledComponents();
        RestoreHydrodynamicsControllerState();
        _runRoutine = null;

        if (verbose)
        {
            Debug.Log($"[FinsROVHydrodynamicMotionBenchmark] Finished. CSV: {_samplesPath}");
        }
    }

    IEnumerator RunTrial(HydrodynamicsMode backendMode, AxisTrialConfig config, float level)
    {
        PrepareForTrial();
        StartTrialSummary();

        _activeBackend = backendMode.ToString();
        _activeAxis = AxisName(config.axis);
        _activeLevel = level;

        if (verbose)
        {
            Debug.Log(
                $"[FinsROVHydrodynamicMotionBenchmark] Trial {_trialIndex} backend={_activeBackend} " +
                $"axis={_activeAxis} level={level.ToString("F4", CultureInfo.InvariantCulture)}");
        }

        yield return RunPhase("baseline", config.baselineSeconds, SixDofVector.Zero, false);
        yield return RunPhase("excitation", config.excitationSeconds, CommandFor(config.axis, level), true);
        yield return RunPhase("rest", config.restSeconds, SixDofVector.Zero, false);

        AppendTrialSummary(config, level);
        if (flushAtEndOfEachTrial)
        {
            WriteOutputs();
        }
    }

    IEnumerator RunPhase(string phase, float seconds, SixDofVector command, bool applyCommand)
    {
        _activePhase = phase;
        _activeCommand = command;
        _applyActiveCommand = applyCommand;
        _phaseStartTime = Time.fixedTime;
        _isLogging = true;

        float duration = Mathf.Max(0f, seconds);
        while (Time.fixedTime - _phaseStartTime < duration)
        {
            yield return new WaitForFixedUpdate();
        }

        _applyActiveCommand = false;
    }

    void PrepareForTrial()
    {
        if (targetRigidbody == null)
        {
            return;
        }

        if (resetPoseEachTrial)
        {
            transform.SetPositionAndRotation(resetPosition, Quaternion.Euler(resetEulerAngles));
        }

        targetRigidbody.linearVelocity = Vector3.zero;
        targetRigidbody.angularVelocity = Vector3.zero;
        targetRigidbody.Sleep();
        targetRigidbody.WakeUp();
        Physics.SyncTransforms();

        _hasPreviousVelocity = false;
        _previousVelocity = SixDofVector.Zero;
        _latestAcceleration = SixDofVector.Zero;
        hydrodynamicsController?.ResetBackendState();
    }

    void ConfigureBackend(HydrodynamicsMode backendMode)
    {
        if (hydrodynamicsController == null)
        {
            return;
        }

        hydrodynamicsController.mode = backendMode;
        Dwp2HydrodynamicsInterop interop = hydrodynamicsController.GetComponent<Dwp2HydrodynamicsInterop>();
        if (disableLegacyDwp2HydroDuringBenchmark && interop != null)
        {
            interop.suppressDwp2ForParametricBackends = true;
        }

        hydrodynamicsController.ResetBackendState();
    }

    void ApplyCommandWrench(SixDofVector bodyWrench)
    {
        HydroMath.FossenBodyWrenchToUnityWorld(
            BenchmarkFrame,
            BenchmarkAxisConvention,
            bodyWrench,
            out Vector3 force,
            out Vector3 torque);
        if (force.sqrMagnitude > 0f)
        {
            targetRigidbody.AddForce(force, ForceMode.Force);
        }

        if (torque.sqrMagnitude > 0f)
        {
            targetRigidbody.AddTorque(torque, ForceMode.Force);
        }
    }

    void RecordSample(SixDofVector velocity, SixDofVector acceleration)
    {
        HydrodynamicsWrench hydroWrench = hydrodynamicsController != null
            ? hydrodynamicsController.LastWrench
            : HydrodynamicsWrench.Zero;
        SixDofVector relativeVelocity = hydrodynamicsController != null
            ? hydrodynamicsController.LastRelativeVelocity6Dof
            : SixDofVector.Zero;
        WaterKinematicsSample water = hydrodynamicsController != null
            ? hydrodynamicsController.LastWaterSample
            : WaterKinematicsSample.Flat(0f, Vector3.zero);

        HydroMath.FossenBodyWrenchToUnityWorld(
            BenchmarkFrame,
            BenchmarkAxisConvention,
            _activeCommand,
            out Vector3 commandWorldForce,
            out Vector3 commandWorldTorque);
        Vector3 euler = transform.eulerAngles;
        Quaternion rotation = transform.rotation;
        float phaseElapsed = Time.fixedTime - _phaseStartTime;
        float elapsed = Time.fixedTime - _runStartTime;

        UpdateTrialSummary(velocity, acceleration, hydroWrench);

        _samplesCsv.AppendFormat(
            CultureInfo.InvariantCulture,
            "{0:F5},{1},{2},{3},{4},{5:F5},{6:F5}," +
            "{7:F6},{8:F6},{9:F6},{10:F6},{11:F6},{12:F6}," +
            "{13:F6},{14:F6},{15:F6},{16:F6},{17:F6},{18:F6}," +
            "{19:F6},{20:F6},{21:F6},{22:F6},{23:F6},{24:F6},{25:F6},{26:F6},{27:F6},{28:F6}," +
            "{29:F6},{30:F6},{31:F6},{32:F6},{33:F6},{34:F6}," +
            "{35:F6},{36:F6},{37:F6},{38:F6},{39:F6},{40:F6}," +
            "{41:F6},{42:F6},{43:F6},{44:F6},{45:F6},{46:F6}," +
            "{47:F6},{48:F6},{49:F6},{50:F6},{51:F6},{52:F6}," +
            "{53:F6},{54:F6},{55:F6},{56:F6},{57:F6},{58:F6},{59:F6},{60:F6},{61:F6},{62:F6},{63:F6},{64:F6},{65:F6}\n",
            elapsed,
            _activeBackend,
            _trialIndex,
            _activeAxis,
            _activePhase,
            phaseElapsed,
            _activeLevel,
            _activeCommand.u,
            _activeCommand.v,
            _activeCommand.w,
            _activeCommand.p,
            _activeCommand.q,
            _activeCommand.r,
            commandWorldForce.x,
            commandWorldForce.y,
            commandWorldForce.z,
            commandWorldTorque.x,
            commandWorldTorque.y,
            commandWorldTorque.z,
            transform.position.x,
            transform.position.y,
            transform.position.z,
            euler.x,
            euler.y,
            euler.z,
            rotation.x,
            rotation.y,
            rotation.z,
            rotation.w,
            velocity.u,
            velocity.v,
            velocity.w,
            velocity.p,
            velocity.q,
            velocity.r,
            acceleration.u,
            acceleration.v,
            acceleration.w,
            acceleration.p,
            acceleration.q,
            acceleration.r,
            relativeVelocity.u,
            relativeVelocity.v,
            relativeVelocity.w,
            relativeVelocity.p,
            relativeVelocity.q,
            relativeVelocity.r,
            hydroWrench.BodyWrench.u,
            hydroWrench.BodyWrench.v,
            hydroWrench.BodyWrench.w,
            hydroWrench.BodyWrench.p,
            hydroWrench.BodyWrench.q,
            hydroWrench.BodyWrench.r,
            hydroWrench.WorldForce.x,
            hydroWrench.WorldForce.y,
            hydroWrench.WorldForce.z,
            hydroWrench.WorldTorque.x,
            hydroWrench.WorldTorque.y,
            hydroWrench.WorldTorque.z,
            water.FlowVelocity.x,
            water.FlowVelocity.y,
            water.FlowVelocity.z,
            water.Height,
            targetRigidbody.mass,
            targetRigidbody.inertiaTensor.x,
            targetRigidbody.inertiaTensor.y,
            targetRigidbody.inertiaTensor.z);
    }

    void BeginOutput()
    {
        Directory.CreateDirectory(outputDirectory);
        string stamp = DateTime.UtcNow.ToString("yyyyMMddTHHmmssZ", CultureInfo.InvariantCulture);
        string basePath = Path.Combine(outputDirectory, $"{outputPrefix}_{stamp}");
        _samplesPath = basePath + ".csv";
        _summaryPath = basePath + "_summary.csv";

        _samplesCsv.Clear();
        _summaryCsv.Clear();
        _samplesCsv.AppendLine(
            "time_sec,backend,trial_index,axis,phase,phase_elapsed_sec,command_level," +
            "cmd_X_N,cmd_Y_N,cmd_Z_N,cmd_K_Nm,cmd_M_Nm,cmd_N_Nm," +
            "cmd_world_force_x,cmd_world_force_y,cmd_world_force_z,cmd_world_torque_x,cmd_world_torque_y,cmd_world_torque_z," +
            "position_x,position_y,position_z,euler_x_deg,euler_y_deg,euler_z_deg,quat_x,quat_y,quat_z,quat_w," +
            "nu_u_mps,nu_v_mps,nu_w_mps,nu_p_radps,nu_q_radps,nu_r_radps," +
            "nudot_u_mps2,nudot_v_mps2,nudot_w_mps2,nudot_p_radps2,nudot_q_radps2,nudot_r_radps2," +
            "rel_u_mps,rel_v_mps,rel_w_mps,rel_p_radps,rel_q_radps,rel_r_radps," +
            "hydro_X_N,hydro_Y_N,hydro_Z_N,hydro_K_Nm,hydro_M_Nm,hydro_N_Nm," +
            "hydro_world_force_x,hydro_world_force_y,hydro_world_force_z,hydro_world_torque_x,hydro_world_torque_y,hydro_world_torque_z," +
            "water_flow_x,water_flow_y,water_flow_z,water_height,rb_mass_kg,rb_inertia_x,rb_inertia_y,rb_inertia_z");
        _summaryCsv.AppendLine(
            "backend,trial_index,axis,command_level,total_trial_sec,excitation_sec," +
            "max_abs_primary_velocity,final_primary_velocity,max_abs_primary_acceleration,final_primary_acceleration," +
            "max_abs_cross_velocity,max_abs_cross_acceleration,max_hydro_force_n,max_hydro_torque_nm");
    }

    void WriteOutputs()
    {
        if (string.IsNullOrEmpty(_samplesPath))
        {
            return;
        }

        Directory.CreateDirectory(Path.GetDirectoryName(_samplesPath));
        File.WriteAllText(_samplesPath, _samplesCsv.ToString());
        File.WriteAllText(_summaryPath, _summaryCsv.ToString());
    }

    void StartTrialSummary()
    {
        Array.Clear(_trialMaxAbsNu, 0, _trialMaxAbsNu.Length);
        Array.Clear(_trialMaxAbsNuDot, 0, _trialMaxAbsNuDot.Length);
        Array.Clear(_trialFinalNu, 0, _trialFinalNu.Length);
        Array.Clear(_trialFinalNuDot, 0, _trialFinalNuDot.Length);
        _trialMaxHydroForce = 0f;
        _trialMaxHydroTorque = 0f;
    }

    void UpdateTrialSummary(SixDofVector velocity, SixDofVector acceleration, HydrodynamicsWrench hydroWrench)
    {
        for (int i = 0; i < 6; i++)
        {
            _trialMaxAbsNu[i] = Mathf.Max(_trialMaxAbsNu[i], Mathf.Abs(velocity[i]));
            _trialMaxAbsNuDot[i] = Mathf.Max(_trialMaxAbsNuDot[i], Mathf.Abs(acceleration[i]));
            _trialFinalNu[i] = velocity[i];
            _trialFinalNuDot[i] = acceleration[i];
        }

        _trialMaxHydroForce = Mathf.Max(_trialMaxHydroForce, hydroWrench.WorldForce.magnitude);
        _trialMaxHydroTorque = Mathf.Max(_trialMaxHydroTorque, hydroWrench.WorldTorque.magnitude);
    }

    void AppendTrialSummary(AxisTrialConfig config, float level)
    {
        int primary = AxisIndex(config.axis);
        float maxCrossVelocity = 0f;
        float maxCrossAcceleration = 0f;
        for (int i = 0; i < 6; i++)
        {
            if (i == primary)
            {
                continue;
            }

            maxCrossVelocity = Mathf.Max(maxCrossVelocity, _trialMaxAbsNu[i]);
            maxCrossAcceleration = Mathf.Max(maxCrossAcceleration, _trialMaxAbsNuDot[i]);
        }

        _summaryCsv.AppendFormat(
            CultureInfo.InvariantCulture,
            "{0},{1},{2},{3:F6},{4:F6},{5:F6},{6:F6},{7:F6},{8:F6},{9:F6},{10:F6},{11:F6},{12:F6},{13:F6}\n",
            _activeBackend,
            _trialIndex,
            AxisName(config.axis),
            level,
            config.baselineSeconds + config.excitationSeconds + config.restSeconds,
            config.excitationSeconds,
            _trialMaxAbsNu[primary],
            _trialFinalNu[primary],
            _trialMaxAbsNuDot[primary],
            _trialFinalNuDot[primary],
            maxCrossVelocity,
            maxCrossAcceleration,
            _trialMaxHydroForce,
            _trialMaxHydroTorque);
    }

    IEnumerable<float> ExpandLevels(AxisTrialConfig config)
    {
        float magnitude = Mathf.Abs(config.amplitude);
        yield return magnitude;
        if (config.includeNegative)
        {
            yield return -magnitude;
        }
    }

    static SixDofVector CommandFor(SixDofBenchmarkAxis axis, float level)
    {
        var command = SixDofVector.Zero;
        command[AxisIndex(axis)] = level;
        return command;
    }

    static int AxisIndex(SixDofBenchmarkAxis axis)
    {
        return (int)axis;
    }

    static string AxisName(SixDofBenchmarkAxis axis)
    {
        switch (axis)
        {
            case SixDofBenchmarkAxis.SurgeU: return "surge_u";
            case SixDofBenchmarkAxis.SwayV: return "sway_v";
            case SixDofBenchmarkAxis.HeaveW: return "heave_w";
            case SixDofBenchmarkAxis.RollP: return "roll_p";
            case SixDofBenchmarkAxis.PitchQ: return "pitch_q";
            case SixDofBenchmarkAxis.YawR: return "yaw_r";
            default: return axis.ToString();
        }
    }

    static bool IsBenchmarkBackend(HydrodynamicsMode mode)
    {
        return mode == HydrodynamicsMode.Fossen6Dof ||
            mode == HydrodynamicsMode.SurfaceGeometryHydro ||
            mode == HydrodynamicsMode.FossenPlusSurfaceResidual;
    }

    bool ValidateSetup()
    {
        if (targetRigidbody == null)
        {
            Debug.LogError("[FinsROVHydrodynamicMotionBenchmark] Missing target Rigidbody.", this);
            return false;
        }

        if (hydrodynamicsController == null)
        {
            Debug.LogError("[FinsROVHydrodynamicMotionBenchmark] Missing HydrodynamicsController.", this);
            return false;
        }

        if (backendModes == null || backendModes.Count == 0)
        {
            Debug.LogError("[FinsROVHydrodynamicMotionBenchmark] No backend modes configured.", this);
            return false;
        }

        return true;
    }

    void WarnIfSurfaceBackendHasNoMesh(HydrodynamicsMode backendMode)
    {
        if (backendMode != HydrodynamicsMode.SurfaceGeometryHydro &&
            backendMode != HydrodynamicsMode.FossenPlusSurfaceResidual)
        {
            return;
        }

        bool hasMesh = hydrodynamicsController.surfaceHydroMeshOverride != null ||
            (hydrodynamicsController.surfaceHydroMeshFilter != null &&
                hydrodynamicsController.surfaceHydroMeshFilter.sharedMesh != null) ||
            (hydrodynamicsController.profile != null && hydrodynamicsController.profile.surfaceHydroMesh != null);
        if (!hasMesh)
        {
            Debug.LogWarning(
                "[FinsROVHydrodynamicMotionBenchmark] SurfaceGeometryHydro has no surface mesh configured. " +
                "Set HydrodynamicsController.surfaceHydroMeshFilter or HydrodynamicsProfile.surfaceHydroMesh.",
                this);
        }
    }

    void DisableConfiguredComponents()
    {
        _disabledComponentStates.Clear();
        if (componentsToDisableDuringBenchmark == null)
        {
            return;
        }

        foreach (MonoBehaviour component in componentsToDisableDuringBenchmark)
        {
            if (component == null || component == this || component == hydrodynamicsController)
            {
                continue;
            }

            _disabledComponentStates.Add(new ComponentState { Component = component, Enabled = component.enabled });
            component.enabled = false;
        }
    }

    void RestoreDisabledComponents()
    {
        foreach (ComponentState state in _disabledComponentStates)
        {
            if (state.Component != null)
            {
                state.Component.enabled = state.Enabled;
            }
        }

        _disabledComponentStates.Clear();
    }

    void CaptureHydrodynamicsControllerState()
    {
        if (hydrodynamicsController == null)
        {
            _hasOriginalControllerState = false;
            return;
        }

        _originalMode = hydrodynamicsController.mode;
        Dwp2HydrodynamicsInterop interop = hydrodynamicsController.GetComponent<Dwp2HydrodynamicsInterop>();
        _originalSuppressDwp2Interop = interop != null && interop.suppressDwp2ForParametricBackends;
        _hasOriginalControllerState = true;
    }

    void RestoreHydrodynamicsControllerState()
    {
        if (!_hasOriginalControllerState || hydrodynamicsController == null)
        {
            return;
        }

        hydrodynamicsController.mode = _originalMode;
        Dwp2HydrodynamicsInterop interop = hydrodynamicsController.GetComponent<Dwp2HydrodynamicsInterop>();
        if (interop != null)
        {
            interop.suppressDwp2ForParametricBackends = _originalSuppressDwp2Interop;
        }
        hydrodynamicsController.ResetBackendState();
        _hasOriginalControllerState = false;
    }

    Transform BenchmarkFrame => bodyFrame != null ? bodyFrame : transform;
    BodyAxisConvention BenchmarkAxisConvention => hydrodynamicsController != null
        ? hydrodynamicsController.bodyAxisConvention
        : BodyAxisConvention.LegacyHydroMathZForwardXRightYUp;
}
