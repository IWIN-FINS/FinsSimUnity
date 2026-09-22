using System;
using System.Collections;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Text;
using NWH.DWP2.ShipController;
using UnityEngine;

public class HydrodynamicAxisIdentifier : MonoBehaviour
{
    public enum AxisSelectionMode
    {
        AllConfiguredAxes = 0,
        SingleAxis = 1,
    }

    public enum LinearAxis
    {
        SurgeX = 0,
        HeaveY = 1,
        SwayZ = 2,
    }

    [Serializable]
    public class EngineWeight
    {
        public string engineName = string.Empty;
        [Range(-1f, 1f)]
        public float weight = 0f;
    }

    [Serializable]
    public class AxisThrusterMap
    {
        public LinearAxis axis = LinearAxis.SurgeX;
        public List<EngineWeight> engineWeights = new List<EngineWeight>();
    }

    [Serializable]
    public class AxisExcitationConfig
    {
        public LinearAxis axis = LinearAxis.SurgeX;
        public float baselineSeconds = 1.0f;
        public float holdSeconds = 4.0f;
        public float restSeconds = 1.5f;
        public List<float> throttleLevels = new List<float> { 0.25f, 0.5f, 0.75f, 1.0f };
        public bool includeNegativeMirror = true;
    }

    [Header("References")]
    [SerializeField] private Rigidbody rigidBody;
    [SerializeField] private AdvancedShipController advancedShipController;
    [SerializeField] private Transform bodyFrame;
    [SerializeField] private RandomizedWaterCurrentProvider waterCurrentProvider;

    [Header("Execution")]
    [SerializeField] private bool autoRunOnStart = false;
    [SerializeField] private AxisSelectionMode axisSelectionMode = AxisSelectionMode.SingleAxis;
    [SerializeField] private LinearAxis selectedAxis = LinearAxis.SurgeX;
    [SerializeField] private bool disableRandomizedWaterCurrent = true;
    [SerializeField] private float settleSeconds = 1.0f;
    [SerializeField] private bool resetPoseEachTrial = true;
    [SerializeField] private Vector3 resetPosition = new Vector3(0f, -4f, 0f);
    [SerializeField] private Vector3 resetEulerAngles = Vector3.zero;
    [SerializeField] private bool verbose = true;

    [Header("Output")]
    [SerializeField] private string outputDirectory = "/home/fins/UnderwaterSim/Code/RL/results";
    [SerializeField] private string outputFileName = "hydrodynamic_identification.csv";
    [SerializeField] private bool flushAfterEachTrial = true;

    [Header("Excitation Setup")]
    [SerializeField] private List<AxisExcitationConfig> excitationConfigs = new List<AxisExcitationConfig>
    {
        new AxisExcitationConfig { axis = LinearAxis.SurgeX },
        new AxisExcitationConfig { axis = LinearAxis.HeaveY },
        new AxisExcitationConfig { axis = LinearAxis.SwayZ },
    };

    [Header("Thruster Mapping")]
    [SerializeField] private List<AxisThrusterMap> axisThrusterMaps = new List<AxisThrusterMap>();

    private readonly Dictionary<string, Engine> _engineMap = new Dictionary<string, Engine>(StringComparer.Ordinal);
    private readonly Dictionary<LinearAxis, AxisThrusterMap> _axisMap = new Dictionary<LinearAxis, AxisThrusterMap>();
    private readonly StringBuilder _csvBuffer = new StringBuilder(128 * 1024);
    private static readonly string[] AllocatorEngineOrder =
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

    // Pseudo-inverse of underwater_rl.models.thrust_allocator.ThrustAllocator.A.
    // Converts ordered engine thrusts [T1..T8] back to [Fx, Fy, Fz, Mx, My, Mz].
    private static readonly float[,] AllocatorPseudoInverse =
    {
        { 0.000000000f, 0.000000000f, 0.000000000f, 0.000000000f, 0.707013575f, 0.707013575f, -0.707013575f, -0.707013575f },
        { 1.002405774f, 1.002405774f, 1.002405774f, 1.002405774f, 0.000000000f, 0.000000000f, 0.000000000f, 0.000000000f },
        { 0.000000000f, 0.000000000f, 0.000000000f, 0.000000000f, -0.707013575f, 0.707013575f, 0.707013575f, -0.707013575f },
        { -0.156113401f, -0.156113401f, 0.156113401f, 0.156113401f, 0.000000000f, 0.000000000f, 0.000000000f, 0.000000000f },
        { 0.000000000f, 0.000000000f, 0.000000000f, 0.000000000f, 0.262881178f, 0.262881178f, 0.262881178f, 0.262881178f },
        { 0.132724570f, -0.132724570f, -0.132724570f, 0.132724570f, 0.000000000f, 0.000000000f, 0.000000000f, 0.000000000f },
    };

    private Coroutine _runRoutine;
    private bool _isLogging;
    private string _activePhase = "idle";
    private int _trialId;
    private float _phaseElapsed;
    private float _commandLevel;
    private LinearAxis _activeAxis;
    private Vector3 _previousLocalVelocity;
    private bool _hasPreviousVelocity;
    private string _outputPath = string.Empty;

    private void Reset()
    {
        rigidBody = GetComponent<Rigidbody>();
        advancedShipController = GetComponent<AdvancedShipController>();
        bodyFrame = transform;
    }

    private void Start()
    {
        ResolveReferences();
        BuildEngineMaps();
        EnsureDefaultAxisMaps();
        InitializeCsvBuffer();

        if (autoRunOnStart)
        {
            RunIdentification();
        }
    }

    private void FixedUpdate()
    {
        if (rigidBody == null)
        {
            return;
        }

        Vector3 localVelocity = GetBodyFrame().InverseTransformDirection(rigidBody.linearVelocity);
        float dt = Mathf.Max(Time.fixedDeltaTime, 1e-5f);
        Vector3 localAcceleration = _hasPreviousVelocity ? (localVelocity - _previousLocalVelocity) / dt : Vector3.zero;
        _previousLocalVelocity = localVelocity;
        _hasPreviousVelocity = true;

        if (_isLogging)
        {
            AppendSample(localVelocity, localAcceleration);
            _phaseElapsed += dt;
        }
    }

    [ContextMenu("Run Identification")]
    public void RunIdentification()
    {
        if (_runRoutine != null)
        {
            StopCoroutine(_runRoutine);
        }

        ResolveReferences();
        BuildEngineMaps();
        EnsureDefaultAxisMaps();
        InitializeCsvBuffer();
        _outputPath = CreateUniqueOutputPath();
        _runRoutine = StartCoroutine(RunIdentificationCoroutine());
    }

    [ContextMenu("Stop Identification")]
    public void StopIdentification()
    {
        if (_runRoutine != null)
        {
            StopCoroutine(_runRoutine);
            _runRoutine = null;
        }

        _isLogging = false;
        _activePhase = "stopped";
        ApplyZeroThrottle();
    }

    private IEnumerator RunIdentificationCoroutine()
    {
        if (verbose)
        {
            Debug.Log($"[HydrodynamicAxisIdentifier] Writing CSV to: {_outputPath}");
        }

        _trialId = 0;
        foreach (AxisExcitationConfig config in GetSelectedExcitationConfigs())
        {
            foreach (float level in ExpandThrottleLevels(config))
            {
                _trialId++;
                yield return RunSingleTrial(config, level);
            }
        }

        PersistCsv();
        _runRoutine = null;

        if (verbose)
        {
            Debug.Log($"[HydrodynamicAxisIdentifier] Finished. CSV saved to {_outputPath}");
        }
    }

    private IEnumerator RunSingleTrial(AxisExcitationConfig config, float level)
    {
        PrepareForTrial();
        yield return WaitForFixedUpdates(2);

        _activeAxis = config.axis;
        _commandLevel = level;
        _phaseElapsed = 0f;
        _activePhase = "settle";
        _isLogging = true;
        ApplyZeroThrottle();
        yield return WaitForSecondsRealtimeSafe(settleSeconds);

        _phaseElapsed = 0f;
        _activePhase = "baseline";
        ApplyZeroThrottle();
        yield return WaitForSecondsRealtimeSafe(config.baselineSeconds);

        _phaseElapsed = 0f;
        _activePhase = "excitation";
        ApplyAxisThrottle(config.axis, level);
        yield return WaitForSecondsRealtimeSafe(config.holdSeconds);

        _phaseElapsed = 0f;
        _activePhase = "rest";
        ApplyZeroThrottle();
        yield return WaitForSecondsRealtimeSafe(config.restSeconds);

        _isLogging = false;
        _activePhase = "idle";
        ApplyZeroThrottle();

        if (flushAfterEachTrial)
        {
            PersistCsv();
        }

        if (verbose)
        {
            Debug.Log($"[HydrodynamicAxisIdentifier] Trial {_trialId} axis={config.axis} level={level:F3} complete.");
        }
    }

    private void PrepareForTrial()
    {
        if (disableRandomizedWaterCurrent && waterCurrentProvider != null)
        {
            waterCurrentProvider.minCurrentSpeed = 0f;
            waterCurrentProvider.maxCurrentSpeed = 0f;
            waterCurrentProvider.RandomizeForEpisode();
        }

        if (resetPoseEachTrial)
        {
            transform.position = resetPosition;
            transform.rotation = Quaternion.Euler(resetEulerAngles);
        }

        rigidBody.linearVelocity = Vector3.zero;
        rigidBody.angularVelocity = Vector3.zero;
        ApplyZeroThrottle();
        _previousLocalVelocity = Vector3.zero;
        _hasPreviousVelocity = false;
    }

    private IEnumerable<AxisExcitationConfig> GetSelectedExcitationConfigs()
    {
        if (axisSelectionMode == AxisSelectionMode.AllConfiguredAxes)
        {
            foreach (AxisExcitationConfig config in excitationConfigs)
            {
                yield return config;
            }

            yield break;
        }

        AxisExcitationConfig selectedConfig = null;
        foreach (AxisExcitationConfig config in excitationConfigs)
        {
            if (config.axis == selectedAxis)
            {
                selectedConfig = config;
                break;
            }
        }

        if (selectedConfig != null)
        {
            yield return selectedConfig;
            yield break;
        }

        if (verbose)
        {
            Debug.LogWarning(
                $"[HydrodynamicAxisIdentifier] No excitation config found for selected axis {selectedAxis}. " +
                "Falling back to all configured axes."
            );
        }

        foreach (AxisExcitationConfig config in excitationConfigs)
        {
            yield return config;
        }
    }

    private IEnumerable<float> ExpandThrottleLevels(AxisExcitationConfig config)
    {
        foreach (float rawLevel in config.throttleLevels)
        {
            float level = Mathf.Clamp(rawLevel, -1f, 1f);
            if (Mathf.Abs(level) < 1e-4f)
            {
                continue;
            }

            yield return level;
            if (config.includeNegativeMirror && level > 0f)
            {
                yield return -level;
            }
        }
    }

    private IEnumerator WaitForSecondsRealtimeSafe(float seconds)
    {
        float remaining = Mathf.Max(0f, seconds);
        while (remaining > 0f)
        {
            yield return new WaitForFixedUpdate();
            remaining -= Time.fixedDeltaTime;
        }
    }

    private IEnumerator WaitForFixedUpdates(int count)
    {
        int remaining = Mathf.Max(0, count);
        while (remaining > 0)
        {
            yield return new WaitForFixedUpdate();
            remaining--;
        }
    }

    private void AppendSample(Vector3 localVelocity, Vector3 localAcceleration)
    {
        float[] orderedEngineThrusts = GetOrderedEngineThrusts();
        Vector3 bodyForce = ComputeAllocatorBodyForce(orderedEngineThrusts);
        Vector3 bodyTorque = ComputeAllocatorBodyTorque(orderedEngineThrusts);
        Vector3 bodyFlow = GetBodyWaterFlow();
        Vector3 position = GetBodyFrame().position;
        int axisIndex = (int)_activeAxis;
        float tauAxis = axisIndex == 0 ? bodyForce.x : axisIndex == 1 ? bodyForce.y : bodyForce.z;
        float velAxis = axisIndex == 0 ? localVelocity.x : axisIndex == 1 ? localVelocity.y : localVelocity.z;
        float accAxis = axisIndex == 0 ? localAcceleration.x : axisIndex == 1 ? localAcceleration.y : localAcceleration.z;

        AppendCsvValue(DateTime.UtcNow.ToString("o", CultureInfo.InvariantCulture));
        AppendCsvValue(_trialId);
        AppendCsvValue(_activeAxis.ToString());
        AppendCsvValue(_activePhase);
        AppendCsvValue(_commandLevel);
        AppendCsvValue(_phaseElapsed);
        AppendCsvValue(tauAxis);
        AppendCsvValue(velAxis);
        AppendCsvValue(accAxis);
        AppendCsvValue(localVelocity.x);
        AppendCsvValue(localVelocity.y);
        AppendCsvValue(localVelocity.z);
        AppendCsvValue(localAcceleration.x);
        AppendCsvValue(localAcceleration.y);
        AppendCsvValue(localAcceleration.z);
        AppendCsvValue(bodyForce.x);
        AppendCsvValue(bodyForce.y);
        AppendCsvValue(bodyForce.z);
        AppendCsvValue(bodyTorque.x);
        AppendCsvValue(bodyTorque.y);
        AppendCsvValue(bodyTorque.z);
        foreach (float engineThrust in orderedEngineThrusts)
        {
            AppendCsvValue(engineThrust);
        }
        AppendCsvValue(bodyFlow.x);
        AppendCsvValue(bodyFlow.y);
        AppendCsvValue(bodyFlow.z);
        AppendCsvValue(position.x);
        AppendCsvValue(position.y);
        AppendCsvValue(position.z, last: true);
    }

    private void InitializeCsvBuffer()
    {
        _csvBuffer.Clear();
        _csvBuffer.AppendLine(
            "utc_time,trial_id,axis,phase,command_level,phase_elapsed_s,tau_axis_N,vel_axis_mps,acc_axis_mps2," +
            "vel_body_x,vel_body_y,vel_body_z,acc_body_x,acc_body_y,acc_body_z," +
            "force_body_x_N,force_body_y_N,force_body_z_N,torque_body_x_Nm,torque_body_y_Nm,torque_body_z_Nm," +
            "thrust_vertical1_N,thrust_vertical2_N,thrust_vertical3_N,thrust_vertical4_N," +
            "thrust_horizontal1_N,thrust_horizontal2_N,thrust_horizontal3_N,thrust_horizontal4_N," +
            "water_body_x,water_body_y,water_body_z,position_x,position_y,position_z"
        );
    }

    private void PersistCsv()
    {
        if (string.IsNullOrWhiteSpace(_outputPath))
        {
            return;
        }

        string directory = Path.GetDirectoryName(_outputPath);
        if (!string.IsNullOrWhiteSpace(directory))
        {
            Directory.CreateDirectory(directory);
        }
        File.WriteAllText(_outputPath, _csvBuffer.ToString());
    }

    private string CreateUniqueOutputPath()
    {
        string directory = GetResolvedOutputDirectory();
        string configuredFileName = string.IsNullOrWhiteSpace(outputFileName)
            ? "hydrodynamic_identification.csv"
            : outputFileName;

        string configuredDirectory = Path.GetDirectoryName(configuredFileName);
        if (Path.IsPathRooted(configuredFileName))
        {
            directory = configuredDirectory;
        }
        else if (!string.IsNullOrWhiteSpace(configuredDirectory))
        {
            directory = Path.Combine(directory, configuredDirectory);
        }

        if (string.IsNullOrWhiteSpace(directory))
        {
            directory = Application.persistentDataPath;
        }

        Directory.CreateDirectory(directory);

        string fileName = Path.GetFileName(configuredFileName);
        string stem = Path.GetFileNameWithoutExtension(fileName);
        if (string.IsNullOrWhiteSpace(stem))
        {
            stem = "hydrodynamic_identification";
        }

        string extension = Path.GetExtension(fileName);
        if (string.IsNullOrWhiteSpace(extension))
        {
            extension = ".csv";
        }
        extension = SanitizeFileNameComponent(extension);
        if (!extension.StartsWith(".", StringComparison.Ordinal))
        {
            extension = $".{extension}";
        }

        string axisLabel = SanitizeFileNameComponent(GetOutputAxisLabel());
        string sanitizedStem = SanitizeFileNameComponent(stem);
        string suffix = $"_axis_{axisLabel}{extension}";
        int trainingIndex = GetNextTrainingIndex(directory, sanitizedStem, extension);

        while (true)
        {
            string candidateFileName = $"{sanitizedStem}_train{trainingIndex:000}{suffix}";
            string candidatePath = Path.Combine(directory, candidateFileName);
            if (!File.Exists(candidatePath))
            {
                return candidatePath;
            }

            trainingIndex++;
        }
    }

    private int GetNextTrainingIndex(string directory, string stem, string extension)
    {
        int maxTrainingIndex = 0;
        string prefix = $"{stem}_train";
        string axisMarker = "_axis_";
        string searchPattern = $"{prefix}*{axisMarker}*{extension}";
        string[] existingFiles = Directory.GetFiles(directory, searchPattern);

        foreach (string existingFile in existingFiles)
        {
            string fileName = Path.GetFileName(existingFile);
            if (!fileName.StartsWith(prefix, StringComparison.Ordinal) ||
                !fileName.EndsWith(extension, StringComparison.Ordinal))
            {
                continue;
            }

            int axisMarkerIndex = fileName.IndexOf(axisMarker, prefix.Length, StringComparison.Ordinal);
            if (axisMarkerIndex < 0)
            {
                continue;
            }

            int numberStart = prefix.Length;
            int numberLength = axisMarkerIndex - numberStart;
            if (numberLength <= 0)
            {
                continue;
            }

            string numberText = fileName.Substring(numberStart, numberLength);
            if (int.TryParse(numberText, NumberStyles.None, CultureInfo.InvariantCulture, out int trainingIndex))
            {
                maxTrainingIndex = Mathf.Max(maxTrainingIndex, trainingIndex);
            }
        }

        return maxTrainingIndex + 1;
    }

    private string GetOutputAxisLabel()
    {
        List<string> axisNames = new List<string>();
        foreach (AxisExcitationConfig config in GetSelectedExcitationConfigs())
        {
            string axisName = config.axis.ToString();
            if (!axisNames.Contains(axisName))
            {
                axisNames.Add(axisName);
            }
        }

        if (axisNames.Count == 0)
        {
            return "None";
        }

        return string.Join("-", axisNames.ToArray());
    }

    private string SanitizeFileNameComponent(string value)
    {
        if (string.IsNullOrWhiteSpace(value))
        {
            return "unnamed";
        }

        char[] invalidChars = Path.GetInvalidFileNameChars();
        StringBuilder builder = new StringBuilder(value.Length);
        foreach (char currentChar in value)
        {
            bool isInvalid = false;
            foreach (char invalidChar in invalidChars)
            {
                if (currentChar == invalidChar)
                {
                    isInvalid = true;
                    break;
                }
            }

            bool isWildcard = currentChar == '*' || currentChar == '?';
            builder.Append(isInvalid || isWildcard || char.IsWhiteSpace(currentChar) ? '_' : currentChar);
        }

        return builder.Length == 0 ? "unnamed" : builder.ToString();
    }

    private string GetResolvedOutputDirectory()
    {
        if (!string.IsNullOrWhiteSpace(outputDirectory))
        {
            return outputDirectory;
        }

        return Application.persistentDataPath;
    }

    private void ApplyZeroThrottle()
    {
        foreach (KeyValuePair<string, Engine> item in _engineMap)
        {
            item.Value.externalThrottleInput = 0f;
        }
    }

    private void ApplyAxisThrottle(LinearAxis axis, float level)
    {
        ApplyZeroThrottle();
        if (!_axisMap.TryGetValue(axis, out AxisThrusterMap map))
        {
            Debug.LogWarning($"[HydrodynamicAxisIdentifier] Missing thruster map for axis {axis}.");
            return;
        }

        foreach (EngineWeight engineWeight in map.engineWeights)
        {
            if (!_engineMap.TryGetValue(engineWeight.engineName, out Engine engine))
            {
                Debug.LogWarning($"[HydrodynamicAxisIdentifier] Missing engine '{engineWeight.engineName}' in controller.");
                continue;
            }

            float throttle = Mathf.Clamp(engineWeight.weight * level, -1f, 1f);
            engine.useExternalThrottleInput = true;
            engine.externalThrottleInput = throttle;
            if (Mathf.Abs(throttle) > 1e-4f)
            {
                engine.StartEngine();
            }
        }
    }

    private float[] GetOrderedEngineThrusts()
    {
        float[] thrusts = new float[AllocatorEngineOrder.Length];
        for (int i = 0; i < AllocatorEngineOrder.Length; i++)
        {
            if (_engineMap.TryGetValue(AllocatorEngineOrder[i], out Engine engine))
            {
                thrusts[i] = engine.Thrust;
            }
        }

        return thrusts;
    }

    private Vector3 ComputeAllocatorBodyForce(float[] orderedEngineThrusts)
    {
        Vector3 force = Vector3.zero;
        force.x = DotAllocatorRow(0, orderedEngineThrusts);
        force.y = DotAllocatorRow(1, orderedEngineThrusts);
        force.z = DotAllocatorRow(2, orderedEngineThrusts);
        return force;
    }

    private Vector3 ComputeAllocatorBodyTorque(float[] orderedEngineThrusts)
    {
        Vector3 torque = Vector3.zero;
        torque.x = DotAllocatorRow(3, orderedEngineThrusts);
        torque.y = DotAllocatorRow(4, orderedEngineThrusts);
        torque.z = DotAllocatorRow(5, orderedEngineThrusts);
        return torque;
    }

    private float DotAllocatorRow(int row, float[] orderedEngineThrusts)
    {
        float value = 0f;
        int count = Mathf.Min(orderedEngineThrusts.Length, AllocatorEngineOrder.Length);
        for (int col = 0; col < count; col++)
        {
            value += AllocatorPseudoInverse[row, col] * orderedEngineThrusts[col];
        }

        return value;
    }

    private Vector3 ComputeNetBodyForce()
    {
        Vector3 totalForce = Vector3.zero;
        Transform frame = GetBodyFrame();

        foreach (KeyValuePair<string, Engine> item in _engineMap)
        {
            Engine engine = item.Value;
            float thrust = engine.Thrust;
            if (Mathf.Abs(thrust) < 1e-5f)
            {
                continue;
            }

            Vector3 worldForce = engine.ThrustDirection * thrust;
            Vector3 bodyForce = frame.InverseTransformDirection(worldForce);
            totalForce += bodyForce;
        }

        return totalForce;
    }

    private Vector3 ComputeNetBodyTorque()
    {
        Vector3 totalTorque = Vector3.zero;
        Transform frame = GetBodyFrame();

        foreach (KeyValuePair<string, Engine> item in _engineMap)
        {
            Engine engine = item.Value;
            float thrust = engine.Thrust;
            if (Mathf.Abs(thrust) < 1e-5f)
            {
                continue;
            }

            Vector3 worldForce = engine.ThrustDirection * thrust;
            Vector3 bodyForce = frame.InverseTransformDirection(worldForce);
            Vector3 worldPoint = engine.ThrustPosition;
            Vector3 bodyPoint = frame.InverseTransformPoint(worldPoint);
            totalTorque += Vector3.Cross(bodyPoint, bodyForce);
        }

        return totalTorque;
    }

    private Vector3 GetBodyWaterFlow()
    {
        if (waterCurrentProvider == null)
        {
            return Vector3.zero;
        }

        return GetBodyFrame().InverseTransformDirection(waterCurrentProvider.CurrentFlow);
    }

    private Transform GetBodyFrame()
    {
        return bodyFrame != null ? bodyFrame : transform;
    }

    private void ResolveReferences()
    {
        if (rigidBody == null)
        {
            rigidBody = GetComponent<Rigidbody>();
        }

        if (advancedShipController == null)
        {
            advancedShipController = GetComponent<AdvancedShipController>();
        }

        if (bodyFrame == null)
        {
            bodyFrame = transform;
        }

        if (waterCurrentProvider == null)
        {
            waterCurrentProvider = FindObjectOfType<RandomizedWaterCurrentProvider>();
        }

        if (rigidBody == null)
        {
            Debug.LogError("[HydrodynamicAxisIdentifier] Missing Rigidbody.");
        }

        if (advancedShipController == null)
        {
            Debug.LogError("[HydrodynamicAxisIdentifier] Missing AdvancedShipController.");
        }
    }

    private void BuildEngineMaps()
    {
        _engineMap.Clear();
        _axisMap.Clear();

        if (advancedShipController == null)
        {
            return;
        }

        foreach (Engine engine in advancedShipController.engines)
        {
            if (engine == null)
            {
                continue;
            }

            engine.useExternalThrottleInput = true;
            if (!_engineMap.ContainsKey(engine.name))
            {
                _engineMap.Add(engine.name, engine);
            }
        }

        foreach (AxisThrusterMap map in axisThrusterMaps)
        {
            _axisMap[map.axis] = map;
        }
    }

    private void EnsureDefaultAxisMaps()
    {
        if (_axisMap.Count > 0)
        {
            return;
        }

        axisThrusterMaps = new List<AxisThrusterMap>
        {
            new AxisThrusterMap
            {
                axis = LinearAxis.HeaveY,
                engineWeights = new List<EngineWeight>
                {
                    new EngineWeight { engineName = "Vertical1", weight = 1f },
                    new EngineWeight { engineName = "Vertical2", weight = 1f },
                    new EngineWeight { engineName = "Vertical3", weight = 1f },
                    new EngineWeight { engineName = "Vertical4", weight = 1f },
                },
            },
            new AxisThrusterMap
            {
                axis = LinearAxis.SurgeX,
                engineWeights = new List<EngineWeight>
                {
                    new EngineWeight { engineName = "Horizontal1", weight = -1f },
                    new EngineWeight { engineName = "Horizontal2", weight = -1f },
                    new EngineWeight { engineName = "Horizontal3", weight = 1f },
                    new EngineWeight { engineName = "Horizontal4", weight = 1f },
                },
            },
            new AxisThrusterMap
            {
                axis = LinearAxis.SwayZ,
                engineWeights = new List<EngineWeight>
                {
                    new EngineWeight { engineName = "Horizontal1", weight = 1f },
                    new EngineWeight { engineName = "Horizontal2", weight = -1f },
                    new EngineWeight { engineName = "Horizontal3", weight = -1f },
                    new EngineWeight { engineName = "Horizontal4", weight = 1f },
                },
            },
        };

        _axisMap.Clear();
        foreach (AxisThrusterMap map in axisThrusterMaps)
        {
            _axisMap[map.axis] = map;
        }
    }

    private void AppendCsvValue(string value, bool last = false)
    {
        _csvBuffer.Append(value);
        _csvBuffer.Append(last ? '\n' : ',');
    }

    private void AppendCsvValue(int value, bool last = false)
    {
        AppendCsvValue(value.ToString(CultureInfo.InvariantCulture), last);
    }

    private void AppendCsvValue(float value, bool last = false)
    {
        AppendCsvValue(value.ToString("G9", CultureInfo.InvariantCulture), last);
    }
}
