using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using NWH.DWP2.WaterObjects;
using UnityEngine;

[DisallowMultipleComponent]
[DefaultExecutionOrder(100)]
public sealed class FinsROVDwp2WaterObjectMotionBenchmark : MonoBehaviour
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

    public enum BodyAxisConvention
    {
        FinsRovXForwardYUpZLeft = 0,
        LegacyHydroMathZForwardXRightYUp = 1,
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

    struct Vector6D
    {
        public float a0;
        public float a1;
        public float a2;
        public float a3;
        public float a4;
        public float a5;

        public Vector6D(float a0, float a1, float a2, float a3, float a4, float a5)
        {
            this.a0 = a0;
            this.a1 = a1;
            this.a2 = a2;
            this.a3 = a3;
            this.a4 = a4;
            this.a5 = a5;
        }

        public float this[int index]
        {
            get
            {
                switch (index)
                {
                    case 0: return a0;
                    case 1: return a1;
                    case 2: return a2;
                    case 3: return a3;
                    case 4: return a4;
                    case 5: return a5;
                    default: throw new IndexOutOfRangeException(nameof(index));
                }
            }
            set
            {
                switch (index)
                {
                    case 0:
                        a0 = value;
                        break;
                    case 1:
                        a1 = value;
                        break;
                    case 2:
                        a2 = value;
                        break;
                    case 3:
                        a3 = value;
                        break;
                    case 4:
                        a4 = value;
                        break;
                    case 5:
                        a5 = value;
                        break;
                    default:
                        throw new IndexOutOfRangeException(nameof(index));
                }
            }
        }

        public static Vector6D Zero => default;

        public static Vector6D operator -(Vector6D lhs, Vector6D rhs)
        {
            return new Vector6D(
                lhs.a0 - rhs.a0,
                lhs.a1 - rhs.a1,
                lhs.a2 - rhs.a2,
                lhs.a3 - rhs.a3,
                lhs.a4 - rhs.a4,
                lhs.a5 - rhs.a5);
        }

        public static Vector6D operator /(Vector6D value, float divisor)
        {
            return new Vector6D(
                value.a0 / divisor,
                value.a1 / divisor,
                value.a2 / divisor,
                value.a3 / divisor,
                value.a4 / divisor,
                value.a5 / divisor);
        }
    }

    struct ComponentState
    {
        public MonoBehaviour Component;
        public bool Enabled;
    }

    struct Dwp2Aggregate
    {
        public Vector3 WorldForce;
        public Vector3 WorldTorque;
        public Vector3 MainBodyWorldForce;
        public Vector3 MainBodyWorldTorque;
        public float SubmergedVolume;
        public float MainBodySubmergedVolume;
        public int WaterObjectCount;
        public int EnabledWaterObjectCount;
        public int TriangleCount;
        public int MainBodyTriangleCount;
        public bool HasMainBody;
    }

    struct PlotSample
    {
        public float Time;
        public float Velocity;
        public float Acceleration;
    }

    [Header("References")]
    [SerializeField] Rigidbody targetRigidbody;
    [SerializeField] Transform bodyFrame;
    [SerializeField] Transform waterObjectSearchRoot;

    [Header("Coordinate Mapping")]
    [SerializeField] BodyAxisConvention axisConvention = BodyAxisConvention.FinsRovXForwardYUpZLeft;

    [Header("Execution")]
    [SerializeField] bool autoRunOnStart;
    [SerializeField] bool placeAtBenchmarkStart = true;
    [SerializeField] Vector3 benchmarkStartPosition = new Vector3(0f, -3f, 0f);
    [SerializeField] bool resetPoseEachTrial = true;
    [SerializeField] Vector3 resetPosition = Vector3.zero;
    [SerializeField] Vector3 resetEulerAngles = Vector3.zero;
    [SerializeField] bool captureInitialPoseOnStart = true;
    [SerializeField] bool disableMarusHydrodynamicsControllers = true;
    [SerializeField] MonoBehaviour[] componentsToDisableDuringBenchmark;
    [SerializeField] bool verbose = true;

    [Header("Output")]
    [SerializeField] string outputDirectory = "/home/fins/UnderwaterSim/Code/FinsSim/ros2_ws/data/unity_dwp2_waterobject_benchmark";

    [Header("Plot")]
    [SerializeField] bool writeVelocityAccelerationPng = true;
    [SerializeField] int plotWidth = 1400;
    [SerializeField] int plotHeight = 900;

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
    readonly List<PlotSample> _plotSamples = new List<PlotSample>(4096);
    readonly float[] _trialMaxAbsNu = new float[6];
    readonly float[] _trialMaxAbsNuDot = new float[6];
    readonly float[] _trialFinalNu = new float[6];
    readonly float[] _trialFinalNuDot = new float[6];

    WaterObject[] _waterObjects = Array.Empty<WaterObject>();
    Coroutine _runRoutine;
    bool _isLogging;
    bool _applyActiveCommand;
    bool _hasPreviousVelocity;
    float _phaseStartTime;
    float _runStartTime;
    float _trialStartTime;
    int _trialIndex;
    int _activeAxisIndex;
    string _activeAxis = "none";
    string _activePhase = "idle";
    float _activeLevel;
    Vector6D _activeCommand = Vector6D.Zero;
    Vector6D _previousVelocity = Vector6D.Zero;
    Vector6D _latestAcceleration = Vector6D.Zero;
    float _trialMaxDwp2Force;
    float _trialMaxDwp2Torque;
    float _trialMaxSubmergedVolume;
    string _sampleHeader;
    string _summaryHeader;
    string _samplesPath;
    string _summaryPath;
    string _plotPath;

    Transform SimulationRoot => targetRigidbody != null ? targetRigidbody.transform : transform;
    Transform BenchmarkFrame => bodyFrame != null ? bodyFrame : SimulationRoot;

    void Reset()
    {
        ResolveReferences();
        resetPosition = SimulationRoot.position;
        resetEulerAngles = SimulationRoot.eulerAngles;
    }

    void Awake()
    {
        ResolveReferences();
    }

    void Start()
    {
        ResolveReferences();

        if (placeAtBenchmarkStart)
        {
            SimulationRoot.position = benchmarkStartPosition;
            Physics.SyncTransforms();
        }

        if (captureInitialPoseOnStart)
        {
            resetPosition = SimulationRoot.position;
            resetEulerAngles = SimulationRoot.eulerAngles;
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

        Vector6D velocity = UnityWorldVelocityToFossenBody(
            BenchmarkFrame,
            axisConvention,
            targetRigidbody.linearVelocity,
            targetRigidbody.angularVelocity);
        float dt = Mathf.Max(Time.fixedDeltaTime, 1e-5f);
        _latestAcceleration = _hasPreviousVelocity ? (velocity - _previousVelocity) / dt : Vector6D.Zero;
        _previousVelocity = velocity;
        _hasPreviousVelocity = true;

        if (_isLogging)
        {
            RecordSample(velocity, _latestAcceleration);
        }
    }

    [ContextMenu("Run DWP2 WaterObject Benchmark")]
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

    [ContextMenu("Stop DWP2 WaterObject Benchmark")]
    public void StopBenchmark()
    {
        if (_runRoutine != null)
        {
            StopCoroutine(_runRoutine);
            _runRoutine = null;
        }

        _activeCommand = Vector6D.Zero;
        _applyActiveCommand = false;
        _isLogging = false;
        WriteOutputs();
        RestoreDisabledComponents();
    }

    public void ResolveReferences()
    {
        if (targetRigidbody == null)
        {
            targetRigidbody = GetComponent<Rigidbody>();
        }

        if (targetRigidbody == null)
        {
            targetRigidbody = GetComponentInParent<Rigidbody>();
        }

        if (bodyFrame == null)
        {
            bodyFrame = targetRigidbody != null ? targetRigidbody.transform : transform;
        }

        if (waterObjectSearchRoot == null)
        {
            waterObjectSearchRoot = targetRigidbody != null ? targetRigidbody.transform : transform;
        }

        RefreshWaterObjects();
    }

    IEnumerator RunBenchmarkCoroutine()
    {
        BeginOutput();
        DisableConfiguredComponents();
        DisableMarusHydrodynamicsControllersIfRequested();
        RefreshWaterObjects();

        _runStartTime = Time.fixedTime;
        _trialIndex = 0;

        foreach (AxisTrialConfig config in trials)
        {
            if (config == null || !config.enabled || Mathf.Approximately(config.amplitude, 0f))
            {
                continue;
            }

            foreach (float level in ExpandLevels(config))
            {
                _trialIndex++;
                yield return RunTrial(config, level);
            }
        }

        _activeCommand = Vector6D.Zero;
        _applyActiveCommand = false;
        _isLogging = false;
        WriteOutputs();
        RestoreDisabledComponents();
        _runRoutine = null;

        if (verbose)
        {
            Debug.Log($"[FinsROVDwp2WaterObjectMotionBenchmark] Finished. CSV: {_samplesPath}");
        }
    }

    IEnumerator RunTrial(AxisTrialConfig config, float level)
    {
        PrepareForTrial();
        StartTrialSummary();

        _activeAxis = AxisName(config.axis);
        _activeAxisIndex = AxisIndex(config.axis);
        _activeLevel = level;
        BeginTrialOutput(_activeAxis);
        _trialStartTime = Time.fixedTime;

        if (verbose)
        {
            Debug.Log(
                $"[FinsROVDwp2WaterObjectMotionBenchmark] Trial {_trialIndex} backend=DWP2WaterObject " +
                $"axis={_activeAxis} level={level.ToString("F4", CultureInfo.InvariantCulture)}");
        }

        yield return RunPhase("baseline", config.baselineSeconds, Vector6D.Zero, false);
        yield return RunPhase("excitation", config.excitationSeconds, CommandFor(config.axis, level), true);
        yield return RunPhase("rest", config.restSeconds, Vector6D.Zero, false);

        AppendTrialSummary(config, level);
        WriteOutputs();
    }

    IEnumerator RunPhase(string phase, float seconds, Vector6D command, bool applyCommand)
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
            SimulationRoot.SetPositionAndRotation(resetPosition, Quaternion.Euler(resetEulerAngles));
        }

        targetRigidbody.linearVelocity = Vector3.zero;
        targetRigidbody.angularVelocity = Vector3.zero;
        targetRigidbody.Sleep();
        targetRigidbody.WakeUp();
        Physics.SyncTransforms();

        _hasPreviousVelocity = false;
        _previousVelocity = Vector6D.Zero;
        _latestAcceleration = Vector6D.Zero;
    }

    void ApplyCommandWrench(Vector6D bodyWrench)
    {
        FossenBodyWrenchToUnityWorld(BenchmarkFrame, axisConvention, bodyWrench, out Vector3 force, out Vector3 torque);
        if (force.sqrMagnitude > 0f)
        {
            targetRigidbody.AddForce(force, ForceMode.Force);
        }

        if (torque.sqrMagnitude > 0f)
        {
            targetRigidbody.AddTorque(torque, ForceMode.Force);
        }
    }

    void RecordSample(Vector6D velocity, Vector6D acceleration)
    {
        Dwp2Aggregate dwp2 = AggregateDwp2Forces();
        FossenBodyWrenchToUnityWorld(BenchmarkFrame, axisConvention, _activeCommand, out Vector3 commandWorldForce, out Vector3 commandWorldTorque);
        Vector6D dwp2BodyWrench = UnityWorldWrenchToFossenBody(BenchmarkFrame, axisConvention, dwp2.WorldForce, dwp2.WorldTorque);
        Vector6D mainBodyWrench = dwp2.HasMainBody
            ? UnityWorldWrenchToFossenBody(BenchmarkFrame, axisConvention, dwp2.MainBodyWorldForce, dwp2.MainBodyWorldTorque)
            : Vector6D.Zero;

        Transform root = SimulationRoot;
        Vector3 euler = root.eulerAngles;
        Quaternion rotation = root.rotation;
        float phaseElapsed = Time.fixedTime - _phaseStartTime;
        float elapsed = Time.fixedTime - _runStartTime;

        UpdateTrialSummary(velocity, acceleration, dwp2);
        _plotSamples.Add(new PlotSample
        {
            Time = Time.fixedTime - _trialStartTime,
            Velocity = velocity[_activeAxisIndex],
            Acceleration = acceleration[_activeAxisIndex],
        });

        bool first = true;
        AppendCsv(_samplesCsv, ref first, elapsed);
        AppendCsv(_samplesCsv, ref first, "DWP2WaterObject");
        AppendCsv(_samplesCsv, ref first, _trialIndex);
        AppendCsv(_samplesCsv, ref first, _activeAxis);
        AppendCsv(_samplesCsv, ref first, _activePhase);
        AppendCsv(_samplesCsv, ref first, phaseElapsed);
        AppendCsv(_samplesCsv, ref first, _activeLevel);
        AppendVector6(_samplesCsv, ref first, _activeCommand);
        AppendCsv(_samplesCsv, ref first, commandWorldForce.x);
        AppendCsv(_samplesCsv, ref first, commandWorldForce.y);
        AppendCsv(_samplesCsv, ref first, commandWorldForce.z);
        AppendCsv(_samplesCsv, ref first, commandWorldTorque.x);
        AppendCsv(_samplesCsv, ref first, commandWorldTorque.y);
        AppendCsv(_samplesCsv, ref first, commandWorldTorque.z);
        AppendCsv(_samplesCsv, ref first, root.position.x);
        AppendCsv(_samplesCsv, ref first, root.position.y);
        AppendCsv(_samplesCsv, ref first, root.position.z);
        AppendCsv(_samplesCsv, ref first, euler.x);
        AppendCsv(_samplesCsv, ref first, euler.y);
        AppendCsv(_samplesCsv, ref first, euler.z);
        AppendCsv(_samplesCsv, ref first, rotation.x);
        AppendCsv(_samplesCsv, ref first, rotation.y);
        AppendCsv(_samplesCsv, ref first, rotation.z);
        AppendCsv(_samplesCsv, ref first, rotation.w);
        AppendVector6(_samplesCsv, ref first, velocity);
        AppendVector6(_samplesCsv, ref first, acceleration);
        AppendVector6(_samplesCsv, ref first, dwp2BodyWrench);
        AppendCsv(_samplesCsv, ref first, dwp2.WorldForce.x);
        AppendCsv(_samplesCsv, ref first, dwp2.WorldForce.y);
        AppendCsv(_samplesCsv, ref first, dwp2.WorldForce.z);
        AppendCsv(_samplesCsv, ref first, dwp2.WorldTorque.x);
        AppendCsv(_samplesCsv, ref first, dwp2.WorldTorque.y);
        AppendCsv(_samplesCsv, ref first, dwp2.WorldTorque.z);
        AppendCsv(_samplesCsv, ref first, dwp2.SubmergedVolume);
        AppendCsv(_samplesCsv, ref first, dwp2.WaterObjectCount);
        AppendCsv(_samplesCsv, ref first, dwp2.EnabledWaterObjectCount);
        AppendCsv(_samplesCsv, ref first, dwp2.TriangleCount);
        AppendVector6(_samplesCsv, ref first, mainBodyWrench);
        AppendCsv(_samplesCsv, ref first, dwp2.MainBodyWorldForce.x);
        AppendCsv(_samplesCsv, ref first, dwp2.MainBodyWorldForce.y);
        AppendCsv(_samplesCsv, ref first, dwp2.MainBodyWorldForce.z);
        AppendCsv(_samplesCsv, ref first, dwp2.MainBodyWorldTorque.x);
        AppendCsv(_samplesCsv, ref first, dwp2.MainBodyWorldTorque.y);
        AppendCsv(_samplesCsv, ref first, dwp2.MainBodyWorldTorque.z);
        AppendCsv(_samplesCsv, ref first, dwp2.MainBodySubmergedVolume);
        AppendCsv(_samplesCsv, ref first, dwp2.MainBodyTriangleCount);
        AppendCsv(_samplesCsv, ref first, dwp2.HasMainBody ? 1 : 0);
        AppendCsv(_samplesCsv, ref first, targetRigidbody.mass);
        AppendCsv(_samplesCsv, ref first, targetRigidbody.inertiaTensor.x);
        AppendCsv(_samplesCsv, ref first, targetRigidbody.inertiaTensor.y);
        AppendCsv(_samplesCsv, ref first, targetRigidbody.inertiaTensor.z);
        AppendCsv(_samplesCsv, ref first, targetRigidbody.inertiaTensorRotation.x);
        AppendCsv(_samplesCsv, ref first, targetRigidbody.inertiaTensorRotation.y);
        AppendCsv(_samplesCsv, ref first, targetRigidbody.inertiaTensorRotation.z);
        AppendCsv(_samplesCsv, ref first, targetRigidbody.inertiaTensorRotation.w);
        AppendCsv(_samplesCsv, ref first, targetRigidbody.worldCenterOfMass.x);
        AppendCsv(_samplesCsv, ref first, targetRigidbody.worldCenterOfMass.y);
        AppendCsv(_samplesCsv, ref first, targetRigidbody.worldCenterOfMass.z);
        _samplesCsv.Append('\n');
    }

    Dwp2Aggregate AggregateDwp2Forces()
    {
        var aggregate = new Dwp2Aggregate
        {
            WaterObjectCount = _waterObjects != null ? _waterObjects.Length : 0,
        };

        if (_waterObjects == null)
        {
            return aggregate;
        }

        for (int i = 0; i < _waterObjects.Length; i++)
        {
            WaterObject waterObject = _waterObjects[i];
            if (waterObject == null || !waterObject.enabled)
            {
                continue;
            }

            aggregate.EnabledWaterObjectCount++;
            aggregate.WorldForce += waterObject.ResultForce;
            aggregate.WorldTorque += waterObject.ResultTorque;
            aggregate.SubmergedVolume += waterObject.submergedVolume;
            aggregate.TriangleCount += waterObject.triangleCount;

            if (waterObject.name.Equals("mainbody", StringComparison.OrdinalIgnoreCase))
            {
                aggregate.HasMainBody = true;
                aggregate.MainBodyWorldForce += waterObject.ResultForce;
                aggregate.MainBodyWorldTorque += waterObject.ResultTorque;
                aggregate.MainBodySubmergedVolume += waterObject.submergedVolume;
                aggregate.MainBodyTriangleCount += waterObject.triangleCount;
            }
        }

        return aggregate;
    }

    void BeginOutput()
    {
        Directory.CreateDirectory(outputDirectory);

        _sampleHeader =
            "time_sec,backend,trial_index,axis,phase,phase_elapsed_sec,command_level," +
            "cmd_X_N,cmd_Y_N,cmd_Z_N,cmd_K_Nm,cmd_M_Nm,cmd_N_Nm," +
            "cmd_world_force_x,cmd_world_force_y,cmd_world_force_z,cmd_world_torque_x,cmd_world_torque_y,cmd_world_torque_z," +
            "position_x,position_y,position_z,euler_x_deg,euler_y_deg,euler_z_deg,quat_x,quat_y,quat_z,quat_w," +
            "nu_u_mps,nu_v_mps,nu_w_mps,nu_p_radps,nu_q_radps,nu_r_radps," +
            "nudot_u_mps2,nudot_v_mps2,nudot_w_mps2,nudot_p_radps2,nudot_q_radps2,nudot_r_radps2," +
            "dwp2_X_N,dwp2_Y_N,dwp2_Z_N,dwp2_K_Nm,dwp2_M_Nm,dwp2_N_Nm," +
            "dwp2_world_force_x,dwp2_world_force_y,dwp2_world_force_z,dwp2_world_torque_x,dwp2_world_torque_y,dwp2_world_torque_z," +
            "dwp2_submerged_volume_m3,dwp2_waterobject_count,dwp2_enabled_waterobject_count,dwp2_triangle_count," +
            "mainbody_X_N,mainbody_Y_N,mainbody_Z_N,mainbody_K_Nm,mainbody_M_Nm,mainbody_N_Nm," +
            "mainbody_world_force_x,mainbody_world_force_y,mainbody_world_force_z,mainbody_world_torque_x,mainbody_world_torque_y,mainbody_world_torque_z," +
            "mainbody_submerged_volume_m3,mainbody_triangle_count,mainbody_found," +
            "rb_mass_kg,rb_inertia_x,rb_inertia_y,rb_inertia_z,rb_inertia_rot_x,rb_inertia_rot_y,rb_inertia_rot_z,rb_inertia_rot_w," +
            "rb_world_com_x,rb_world_com_y,rb_world_com_z";
        _summaryHeader =
            "backend,trial_index,axis,command_level,total_trial_sec,excitation_sec," +
            "max_abs_primary_velocity,final_primary_velocity,max_abs_primary_acceleration,final_primary_acceleration," +
            "max_abs_cross_velocity,max_abs_cross_acceleration,max_dwp2_force_n,max_dwp2_torque_nm,max_submerged_volume_m3";
    }

    void BeginTrialOutput(string axisName)
    {
        string axisDirectory = Path.Combine(outputDirectory, axisName);
        Directory.CreateDirectory(axisDirectory);

        int runIndex = NextAxisRunIndex(axisDirectory, axisName);
        string runName = $"{axisName}_{runIndex:000}";
        string trialDirectory = Path.Combine(axisDirectory, runName);
        Directory.CreateDirectory(trialDirectory);

        _samplesPath = Path.Combine(trialDirectory, $"{runName}_dwp2_motion.csv");
        _summaryPath = Path.Combine(trialDirectory, $"{runName}_dwp2_summary.csv");
        _plotPath = Path.Combine(trialDirectory, $"{runName}_velocity_acceleration.png");

        _samplesCsv.Clear();
        _summaryCsv.Clear();
        _plotSamples.Clear();
        _samplesCsv.AppendLine(_sampleHeader);
        _summaryCsv.AppendLine(_summaryHeader);
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
        if (writeVelocityAccelerationPng)
        {
            WriteVelocityAccelerationPlot();
        }
    }

    void StartTrialSummary()
    {
        Array.Clear(_trialMaxAbsNu, 0, _trialMaxAbsNu.Length);
        Array.Clear(_trialMaxAbsNuDot, 0, _trialMaxAbsNuDot.Length);
        Array.Clear(_trialFinalNu, 0, _trialFinalNu.Length);
        Array.Clear(_trialFinalNuDot, 0, _trialFinalNuDot.Length);
        _trialMaxDwp2Force = 0f;
        _trialMaxDwp2Torque = 0f;
        _trialMaxSubmergedVolume = 0f;
    }

    void WriteVelocityAccelerationPlot()
    {
        if (_plotSamples.Count < 2 || string.IsNullOrEmpty(_plotPath))
        {
            return;
        }

        int width = Mathf.Max(640, plotWidth);
        int height = Mathf.Max(420, plotHeight);
        Texture2D texture = new Texture2D(width, height, TextureFormat.RGBA32, false);
        Color32[] pixels = new Color32[width * height];
        Color32 white = new Color32(255, 255, 255, 255);
        for (int i = 0; i < pixels.Length; i++)
        {
            pixels[i] = white;
        }

        texture.SetPixels32(pixels);

        Rect velocityRect = new Rect(125, height / 2f + 40, width - 175, height / 2f - 105);
        Rect accelerationRect = new Rect(125, 80, width - 175, height / 2f - 105);
        DrawPlotPanel(texture, velocityRect, PlotValueKind.Velocity, new Color32(21, 96, 189, 255));
        DrawPlotPanel(texture, accelerationRect, PlotValueKind.Acceleration, new Color32(204, 81, 37, 255));

        texture.Apply();
        Directory.CreateDirectory(Path.GetDirectoryName(_plotPath));
        File.WriteAllBytes(_plotPath, texture.EncodeToPNG());
        Destroy(texture);
    }

    enum PlotValueKind
    {
        Velocity,
        Acceleration,
    }

    void DrawPlotPanel(Texture2D texture, Rect rect, PlotValueKind valueKind, Color32 lineColor)
    {
        Color32 axisColor = new Color32(35, 35, 35, 255);
        Color32 gridColor = new Color32(220, 225, 230, 255);
        Color32 zeroColor = new Color32(150, 155, 160, 255);

        float minTime = _plotSamples[0].Time;
        float maxTime = _plotSamples[_plotSamples.Count - 1].Time;
        if (maxTime <= minTime)
        {
            maxTime = minTime + Mathf.Max(Time.fixedDeltaTime, 0.02f);
        }

        GetPlotRange(valueKind, out float minValue, out float maxValue);
        if (Mathf.Approximately(minValue, maxValue))
        {
            minValue -= 1f;
            maxValue += 1f;
        }

        for (int i = 0; i <= 5; i++)
        {
            float time = Mathf.Lerp(minTime, maxTime, i / 5f);
            int x = Mathf.RoundToInt(Mathf.Lerp(rect.xMin, rect.xMax, i / 5f));
            DrawLine(texture, x, Mathf.RoundToInt(rect.yMin), x, Mathf.RoundToInt(rect.yMax), gridColor);
            DrawTextCentered(texture, FormatPlotNumber(time), x, Mathf.RoundToInt(rect.yMin) - 30, axisColor, 2);
        }

        for (int i = 0; i <= 4; i++)
        {
            float value = Mathf.Lerp(minValue, maxValue, i / 4f);
            int y = Mathf.RoundToInt(Mathf.Lerp(rect.yMin, rect.yMax, i / 4f));
            DrawLine(texture, Mathf.RoundToInt(rect.xMin), y, Mathf.RoundToInt(rect.xMax), y, gridColor);
            DrawTextRight(texture, FormatPlotNumber(value), Mathf.RoundToInt(rect.xMin) - 12, y - 7, axisColor, 2);
        }

        if (minValue <= 0f && maxValue >= 0f)
        {
            int zeroY = ValueToPixelY(rect, 0f, minValue, maxValue);
            DrawLine(texture, Mathf.RoundToInt(rect.xMin), zeroY, Mathf.RoundToInt(rect.xMax), zeroY, zeroColor);
        }

        DrawRect(texture, rect, axisColor);
        DrawText(texture, PlotPanelTitle(valueKind), Mathf.RoundToInt(rect.xMin), Mathf.RoundToInt(rect.yMax) + 16, lineColor, 2);
        DrawTextCentered(texture, "time (s)", Mathf.RoundToInt((rect.xMin + rect.xMax) * 0.5f), Mathf.RoundToInt(rect.yMin) - 58, axisColor, 2);

        int previousX = TimeToPixelX(rect, _plotSamples[0].Time, minTime, maxTime);
        int previousY = ValueToPixelY(rect, GetPlotValue(_plotSamples[0], valueKind), minValue, maxValue);
        for (int i = 1; i < _plotSamples.Count; i++)
        {
            PlotSample sample = _plotSamples[i];
            int x = TimeToPixelX(rect, sample.Time, minTime, maxTime);
            int y = ValueToPixelY(rect, GetPlotValue(sample, valueKind), minValue, maxValue);
            DrawLine(texture, previousX, previousY, x, y, lineColor);
            previousX = x;
            previousY = y;
        }
    }

    string PlotPanelTitle(PlotValueKind valueKind)
    {
        bool angular = _activeAxisIndex >= 3;
        if (valueKind == PlotValueKind.Velocity)
        {
            return angular ? $"{_activeAxis} angular velocity (rad/s)" : $"{_activeAxis} velocity (m/s)";
        }

        return angular ? $"{_activeAxis} angular acceleration (rad/s2)" : $"{_activeAxis} acceleration (m/s2)";
    }

    void GetPlotRange(PlotValueKind valueKind, out float minValue, out float maxValue)
    {
        minValue = 0f;
        maxValue = 0f;
        for (int i = 0; i < _plotSamples.Count; i++)
        {
            float value = GetPlotValue(_plotSamples[i], valueKind);
            minValue = Mathf.Min(minValue, value);
            maxValue = Mathf.Max(maxValue, value);
        }

        float span = maxValue - minValue;
        float padding = Mathf.Max(span * 0.08f, 1e-3f);
        minValue -= padding;
        maxValue += padding;
    }

    static float GetPlotValue(PlotSample sample, PlotValueKind valueKind)
    {
        return valueKind == PlotValueKind.Velocity ? sample.Velocity : sample.Acceleration;
    }

    static int TimeToPixelX(Rect rect, float time, float minTime, float maxTime)
    {
        float t = Mathf.InverseLerp(minTime, maxTime, time);
        return Mathf.RoundToInt(Mathf.Lerp(rect.xMin, rect.xMax, t));
    }

    static int ValueToPixelY(Rect rect, float value, float minValue, float maxValue)
    {
        float t = Mathf.InverseLerp(minValue, maxValue, value);
        return Mathf.RoundToInt(Mathf.Lerp(rect.yMin, rect.yMax, t));
    }

    static void DrawRect(Texture2D texture, Rect rect, Color32 color)
    {
        int left = Mathf.RoundToInt(rect.xMin);
        int right = Mathf.RoundToInt(rect.xMax);
        int bottom = Mathf.RoundToInt(rect.yMin);
        int top = Mathf.RoundToInt(rect.yMax);
        DrawLine(texture, left, bottom, right, bottom, color);
        DrawLine(texture, left, top, right, top, color);
        DrawLine(texture, left, bottom, left, top, color);
        DrawLine(texture, right, bottom, right, top, color);
    }

    static void DrawLine(Texture2D texture, int x0, int y0, int x1, int y1, Color32 color)
    {
        int dx = Mathf.Abs(x1 - x0);
        int sx = x0 < x1 ? 1 : -1;
        int dy = -Mathf.Abs(y1 - y0);
        int sy = y0 < y1 ? 1 : -1;
        int error = dx + dy;

        while (true)
        {
            SetPixelSafe(texture, x0, y0, color);
            if (x0 == x1 && y0 == y1)
            {
                break;
            }

            int e2 = 2 * error;
            if (e2 >= dy)
            {
                error += dy;
                x0 += sx;
            }

            if (e2 <= dx)
            {
                error += dx;
                y0 += sy;
            }
        }
    }

    static void SetPixelSafe(Texture2D texture, int x, int y, Color32 color)
    {
        if (x < 0 || x >= texture.width || y < 0 || y >= texture.height)
        {
            return;
        }

        texture.SetPixel(x, y, color);
    }

    static string FormatPlotNumber(float value)
    {
        float abs = Mathf.Abs(value);
        string format = abs >= 100f ? "F0" : abs >= 10f ? "F1" : "F2";
        return value.ToString(format, CultureInfo.InvariantCulture);
    }

    static void DrawTextCentered(Texture2D texture, string text, int centerX, int y, Color32 color, int scale)
    {
        DrawText(texture, text, centerX - TextPixelWidth(text, scale) / 2, y, color, scale);
    }

    static void DrawTextRight(Texture2D texture, string text, int rightX, int y, Color32 color, int scale)
    {
        DrawText(texture, text, rightX - TextPixelWidth(text, scale), y, color, scale);
    }

    static int TextPixelWidth(string text, int scale)
    {
        if (string.IsNullOrEmpty(text))
        {
            return 0;
        }

        return text.Length * 6 * scale;
    }

    static void DrawText(Texture2D texture, string text, int x, int y, Color32 color, int scale)
    {
        if (string.IsNullOrEmpty(text))
        {
            return;
        }

        int cursor = x;
        for (int i = 0; i < text.Length; i++)
        {
            DrawGlyph(texture, char.ToUpperInvariant(text[i]), cursor, y, color, scale);
            cursor += 6 * scale;
        }
    }

    static void DrawGlyph(Texture2D texture, char c, int x, int y, Color32 color, int scale)
    {
        string[] glyph = GlyphRows(c);
        for (int row = 0; row < glyph.Length; row++)
        {
            string rowData = glyph[row];
            for (int col = 0; col < rowData.Length; col++)
            {
                if (rowData[col] != '1')
                {
                    continue;
                }

                int px = x + col * scale;
                int py = y + (glyph.Length - 1 - row) * scale;
                for (int sx = 0; sx < scale; sx++)
                {
                    for (int sy = 0; sy < scale; sy++)
                    {
                        SetPixelSafe(texture, px + sx, py + sy, color);
                    }
                }
            }
        }
    }

    static string[] GlyphRows(char c)
    {
        switch (c)
        {
            case '0': return new[] { "01110", "10001", "10011", "10101", "11001", "10001", "01110" };
            case '1': return new[] { "00100", "01100", "00100", "00100", "00100", "00100", "01110" };
            case '2': return new[] { "01110", "10001", "00001", "00010", "00100", "01000", "11111" };
            case '3': return new[] { "11110", "00001", "00001", "01110", "00001", "00001", "11110" };
            case '4': return new[] { "00010", "00110", "01010", "10010", "11111", "00010", "00010" };
            case '5': return new[] { "11111", "10000", "10000", "11110", "00001", "00001", "11110" };
            case '6': return new[] { "01110", "10000", "10000", "11110", "10001", "10001", "01110" };
            case '7': return new[] { "11111", "00001", "00010", "00100", "01000", "01000", "01000" };
            case '8': return new[] { "01110", "10001", "10001", "01110", "10001", "10001", "01110" };
            case '9': return new[] { "01110", "10001", "10001", "01111", "00001", "00001", "01110" };
            case 'A': return new[] { "01110", "10001", "10001", "11111", "10001", "10001", "10001" };
            case 'B': return new[] { "11110", "10001", "10001", "11110", "10001", "10001", "11110" };
            case 'C': return new[] { "01110", "10001", "10000", "10000", "10000", "10001", "01110" };
            case 'D': return new[] { "11110", "10001", "10001", "10001", "10001", "10001", "11110" };
            case 'E': return new[] { "11111", "10000", "10000", "11110", "10000", "10000", "11111" };
            case 'F': return new[] { "11111", "10000", "10000", "11110", "10000", "10000", "10000" };
            case 'G': return new[] { "01110", "10001", "10000", "10111", "10001", "10001", "01110" };
            case 'H': return new[] { "10001", "10001", "10001", "11111", "10001", "10001", "10001" };
            case 'I': return new[] { "01110", "00100", "00100", "00100", "00100", "00100", "01110" };
            case 'J': return new[] { "00001", "00001", "00001", "00001", "10001", "10001", "01110" };
            case 'K': return new[] { "10001", "10010", "10100", "11000", "10100", "10010", "10001" };
            case 'L': return new[] { "10000", "10000", "10000", "10000", "10000", "10000", "11111" };
            case 'M': return new[] { "10001", "11011", "10101", "10101", "10001", "10001", "10001" };
            case 'N': return new[] { "10001", "11001", "10101", "10011", "10001", "10001", "10001" };
            case 'O': return new[] { "01110", "10001", "10001", "10001", "10001", "10001", "01110" };
            case 'P': return new[] { "11110", "10001", "10001", "11110", "10000", "10000", "10000" };
            case 'Q': return new[] { "01110", "10001", "10001", "10001", "10101", "10010", "01101" };
            case 'R': return new[] { "11110", "10001", "10001", "11110", "10100", "10010", "10001" };
            case 'S': return new[] { "01111", "10000", "10000", "01110", "00001", "00001", "11110" };
            case 'T': return new[] { "11111", "00100", "00100", "00100", "00100", "00100", "00100" };
            case 'U': return new[] { "10001", "10001", "10001", "10001", "10001", "10001", "01110" };
            case 'V': return new[] { "10001", "10001", "10001", "10001", "10001", "01010", "00100" };
            case 'W': return new[] { "10001", "10001", "10001", "10101", "10101", "10101", "01010" };
            case 'X': return new[] { "10001", "10001", "01010", "00100", "01010", "10001", "10001" };
            case 'Y': return new[] { "10001", "10001", "01010", "00100", "00100", "00100", "00100" };
            case 'Z': return new[] { "11111", "00001", "00010", "00100", "01000", "10000", "11111" };
            case '.': return new[] { "00000", "00000", "00000", "00000", "00000", "01100", "01100" };
            case '-': return new[] { "00000", "00000", "00000", "11110", "00000", "00000", "00000" };
            case '+': return new[] { "00000", "00100", "00100", "11111", "00100", "00100", "00000" };
            case '/': return new[] { "00001", "00010", "00010", "00100", "01000", "01000", "10000" };
            case '(' : return new[] { "00010", "00100", "01000", "01000", "01000", "00100", "00010" };
            case ')' : return new[] { "01000", "00100", "00010", "00010", "00010", "00100", "01000" };
            case '_': return new[] { "00000", "00000", "00000", "00000", "00000", "00000", "11111" };
            case ' ': return new[] { "00000", "00000", "00000", "00000", "00000", "00000", "00000" };
            default: return new[] { "11111", "00001", "00010", "00100", "00100", "00000", "00100" };
        }
    }

    void UpdateTrialSummary(Vector6D velocity, Vector6D acceleration, Dwp2Aggregate dwp2)
    {
        for (int i = 0; i < 6; i++)
        {
            _trialMaxAbsNu[i] = Mathf.Max(_trialMaxAbsNu[i], Mathf.Abs(velocity[i]));
            _trialMaxAbsNuDot[i] = Mathf.Max(_trialMaxAbsNuDot[i], Mathf.Abs(acceleration[i]));
            _trialFinalNu[i] = velocity[i];
            _trialFinalNuDot[i] = acceleration[i];
        }

        _trialMaxDwp2Force = Mathf.Max(_trialMaxDwp2Force, dwp2.WorldForce.magnitude);
        _trialMaxDwp2Torque = Mathf.Max(_trialMaxDwp2Torque, dwp2.WorldTorque.magnitude);
        _trialMaxSubmergedVolume = Mathf.Max(_trialMaxSubmergedVolume, dwp2.SubmergedVolume);
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

        bool first = true;
        AppendCsv(_summaryCsv, ref first, "DWP2WaterObject");
        AppendCsv(_summaryCsv, ref first, _trialIndex);
        AppendCsv(_summaryCsv, ref first, AxisName(config.axis));
        AppendCsv(_summaryCsv, ref first, level);
        AppendCsv(_summaryCsv, ref first, config.baselineSeconds + config.excitationSeconds + config.restSeconds);
        AppendCsv(_summaryCsv, ref first, config.excitationSeconds);
        AppendCsv(_summaryCsv, ref first, _trialMaxAbsNu[primary]);
        AppendCsv(_summaryCsv, ref first, _trialFinalNu[primary]);
        AppendCsv(_summaryCsv, ref first, _trialMaxAbsNuDot[primary]);
        AppendCsv(_summaryCsv, ref first, _trialFinalNuDot[primary]);
        AppendCsv(_summaryCsv, ref first, maxCrossVelocity);
        AppendCsv(_summaryCsv, ref first, maxCrossAcceleration);
        AppendCsv(_summaryCsv, ref first, _trialMaxDwp2Force);
        AppendCsv(_summaryCsv, ref first, _trialMaxDwp2Torque);
        AppendCsv(_summaryCsv, ref first, _trialMaxSubmergedVolume);
        _summaryCsv.Append('\n');
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

    void RefreshWaterObjects()
    {
        Transform root = waterObjectSearchRoot != null ? waterObjectSearchRoot : SimulationRoot;
        _waterObjects = root != null
            ? root.GetComponentsInChildren<WaterObject>(true)
            : Array.Empty<WaterObject>();
    }

    bool ValidateSetup()
    {
        if (targetRigidbody == null)
        {
            Debug.LogError("[FinsROVDwp2WaterObjectMotionBenchmark] Missing target Rigidbody. Put the script on the ROV root/mainbody or assign Target Rigidbody.");
            return false;
        }

        RefreshWaterObjects();
        if (_waterObjects.Length == 0)
        {
            Debug.LogError("[FinsROVDwp2WaterObjectMotionBenchmark] No DWP2 WaterObject found under Water Object Search Root.");
            return false;
        }

        int enabledCount = 0;
        for (int i = 0; i < _waterObjects.Length; i++)
        {
            if (_waterObjects[i] != null && _waterObjects[i].enabled)
            {
                enabledCount++;
            }
        }

        if (enabledCount == 0)
        {
            Debug.LogWarning("[FinsROVDwp2WaterObjectMotionBenchmark] WaterObjects exist, but all are disabled.");
        }

        return true;
    }

    void DisableConfiguredComponents()
    {
        if (componentsToDisableDuringBenchmark == null)
        {
            return;
        }

        for (int i = 0; i < componentsToDisableDuringBenchmark.Length; i++)
        {
            CaptureAndDisable(componentsToDisableDuringBenchmark[i]);
        }
    }

    void DisableMarusHydrodynamicsControllersIfRequested()
    {
        if (!disableMarusHydrodynamicsControllers)
        {
            return;
        }

        Transform root = waterObjectSearchRoot != null ? waterObjectSearchRoot : SimulationRoot;
        if (root == null)
        {
            return;
        }

        MonoBehaviour[] components = root.GetComponentsInChildren<MonoBehaviour>(true);
        for (int i = 0; i < components.Length; i++)
        {
            MonoBehaviour component = components[i];
            if (component == null)
            {
                continue;
            }

            Type type = component.GetType();
            if (type.Name == "HydrodynamicsController" && type.Namespace == "FinsSim.Hydrodynamics")
            {
                CaptureAndDisable(component);
            }
        }
    }

    void CaptureAndDisable(MonoBehaviour component)
    {
        if (component == null || component == this)
        {
            return;
        }

        for (int i = 0; i < _disabledComponentStates.Count; i++)
        {
            if (_disabledComponentStates[i].Component == component)
            {
                return;
            }
        }

        _disabledComponentStates.Add(new ComponentState
        {
            Component = component,
            Enabled = component.enabled,
        });
        component.enabled = false;
    }

    void RestoreDisabledComponents()
    {
        for (int i = _disabledComponentStates.Count - 1; i >= 0; i--)
        {
            ComponentState state = _disabledComponentStates[i];
            if (state.Component != null)
            {
                state.Component.enabled = state.Enabled;
            }
        }

        _disabledComponentStates.Clear();
    }

    static Vector6D CommandFor(SixDofBenchmarkAxis axis, float level)
    {
        var command = Vector6D.Zero;
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

    static int NextAxisRunIndex(string axisDirectory, string axisName)
    {
        int maxIndex = 0;
        string prefix = axisName + "_";
        foreach (string directory in Directory.GetDirectories(axisDirectory, prefix + "*", SearchOption.TopDirectoryOnly))
        {
            string name = Path.GetFileName(directory);
            if (name == null || !name.StartsWith(prefix, StringComparison.OrdinalIgnoreCase))
            {
                continue;
            }

            if (int.TryParse(name.Substring(prefix.Length), NumberStyles.Integer, CultureInfo.InvariantCulture, out int index))
            {
                maxIndex = Mathf.Max(maxIndex, index);
            }
        }

        return maxIndex + 1;
    }

    static Vector6D UnityWorldVelocityToFossenBody(
        Transform frame,
        BodyAxisConvention convention,
        Vector3 worldLinearVelocity,
        Vector3 worldAngularVelocity)
    {
        Vector3 localLinear = frame.InverseTransformDirection(worldLinearVelocity);
        Vector3 localAngular = frame.InverseTransformDirection(worldAngularVelocity);
        if (convention == BodyAxisConvention.FinsRovXForwardYUpZLeft)
        {
            return new Vector6D(
                localLinear.x,
                localLinear.z,
                -localLinear.y,
                localAngular.x,
                localAngular.z,
                localAngular.y);
        }

        return new Vector6D(
            localLinear.z,
            localLinear.x,
            -localLinear.y,
            localAngular.z,
            localAngular.x,
            -localAngular.y);
    }

    static void FossenBodyWrenchToUnityWorld(
        Transform frame,
        BodyAxisConvention convention,
        Vector6D bodyWrench,
        out Vector3 worldForce,
        out Vector3 worldTorque)
    {
        Vector3 localForce;
        Vector3 localTorque;
        if (convention == BodyAxisConvention.FinsRovXForwardYUpZLeft)
        {
            localForce = new Vector3(bodyWrench[0], -bodyWrench[2], bodyWrench[1]);
            localTorque = new Vector3(bodyWrench[3], bodyWrench[5], bodyWrench[4]);
        }
        else
        {
            localForce = new Vector3(bodyWrench[1], -bodyWrench[2], bodyWrench[0]);
            localTorque = new Vector3(bodyWrench[4], -bodyWrench[5], bodyWrench[3]);
        }

        worldForce = frame.TransformDirection(localForce);
        worldTorque = frame.TransformDirection(localTorque);
    }

    static Vector6D UnityWorldWrenchToFossenBody(
        Transform frame,
        BodyAxisConvention convention,
        Vector3 worldForce,
        Vector3 worldTorque)
    {
        Vector3 localForce = frame.InverseTransformDirection(worldForce);
        Vector3 localTorque = frame.InverseTransformDirection(worldTorque);
        if (convention == BodyAxisConvention.FinsRovXForwardYUpZLeft)
        {
            return new Vector6D(
                localForce.x,
                localForce.z,
                -localForce.y,
                localTorque.x,
                localTorque.z,
                localTorque.y);
        }

        return new Vector6D(
            localForce.z,
            localForce.x,
            -localForce.y,
            localTorque.z,
            localTorque.x,
            -localTorque.y);
    }

    static void AppendVector6(StringBuilder builder, ref bool first, Vector6D value)
    {
        AppendCsv(builder, ref first, value[0]);
        AppendCsv(builder, ref first, value[1]);
        AppendCsv(builder, ref first, value[2]);
        AppendCsv(builder, ref first, value[3]);
        AppendCsv(builder, ref first, value[4]);
        AppendCsv(builder, ref first, value[5]);
    }

    static void AppendCsv(StringBuilder builder, ref bool first, string value)
    {
        if (!first)
        {
            builder.Append(',');
        }

        first = false;
        value = value ?? string.Empty;
        if (value.IndexOfAny(new[] { ',', '"', '\n', '\r' }) < 0)
        {
            builder.Append(value);
            return;
        }

        builder.Append('"');
        builder.Append(value.Replace("\"", "\"\""));
        builder.Append('"');
    }

    static void AppendCsv(StringBuilder builder, ref bool first, int value)
    {
        if (!first)
        {
            builder.Append(',');
        }

        first = false;
        builder.Append(value.ToString(CultureInfo.InvariantCulture));
    }

    static void AppendCsv(StringBuilder builder, ref bool first, float value)
    {
        if (!first)
        {
            builder.Append(',');
        }

        first = false;
        builder.Append(value.ToString("F6", CultureInfo.InvariantCulture));
    }
}
