using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Threading;
using Google.Protobuf;
using Grpc.Core;
using FinsSim.Actuators;
using FinsSim.Core;
using Remotecontrol;
using Sensor;
using Sensorstreaming;
using Std;
using UnityEngine;
using static Remotecontrol.RemoteControl;
using static Sensorstreaming.SensorStreaming;
using MsgPoint = Geometry.Point;
using MsgPose = Geometry.Pose;
using MsgPoseWithCovariance = Geometry.PoseWithCovariance;
using MsgPoseWithCovarianceStamped = Geometry.PoseWithCovarianceStamped;
using MsgQuaternion = Geometry.Quaternion;
using MsgTwist = Geometry.Twist;
using MsgTwistWithCovariance = Geometry.TwistWithCovariance;
using MsgTwistWithCovarianceStamped = Geometry.TwistWithCovarianceStamped;
using MarusThruster = FinsSim.Actuators.Thruster;

namespace FinsSim.Networking
{
    public enum VehicleRosThrusterCommandMode
    {
        ForceN,
        NormalizedDirect
    }

    public enum VehicleRosStatePublishMode
    {
        SimTruth,
        RawFusion,
        RawAndSimTruth
    }

    [DefaultExecutionOrder(55)]
    [DisallowMultipleComponent]
    [RequireComponent(typeof(Rigidbody))]
    public class VehicleRosBridge : MonoBehaviour
    {
        const int ThrusterCount = 8;

        static readonly string[][] CanonicalThrusterNameAliases =
        {
            new[] { "V_LF", "Vertical1" },
            new[] { "V_LB", "Vertical2" },
            new[] { "V_RB", "Vertical3" },
            new[] { "V_RF", "Vertical4" },
            new[] { "H_LF", "Horizontal1" },
            new[] { "H_LB", "Horizontal2" },
            new[] { "H_RB", "Horizontal3" },
            new[] { "H_RF", "Horizontal4" },
        };

        [Header("Bridge Name")]
        public string bridgeName = "finsrov";

        [Header("ROS Topics")]
        public string thrusterTopic = "/finsrov/thrusters_out";
        public string poseTopic = "/finsrov/pose";
        public string imuTopic = "/finsrov/imu_link";
        public string dvlTopic = "/finsrov/dvl_link";
        public string depthTopic = "/finsrov/depth_link";
        public string resetTopic = "/finsrov/reset";
        public string controllerPoseTopic = "/finsrov/controller/pose";
        public string controllerImuTopic = "/finsrov/controller/imu";
        public string controllerDvlTopic = "/finsrov/controller/dvl";
        public string controllerDepthTopic = "/finsrov/controller/depth";

        [Header("Frame IDs")]
        public string poseFrameId = "FinsROV/pose_frame";
        public string imuFrameId = "FinsROV/imu_link_frame";
        public string dvlFrameId = "FinsROV/dvl_link_frame";
        public string depthFrameId = "FinsROV/depth_link_frame";
        public string controllerWorldFrameId = "controller_world";
        public string controllerBodyFrameId = "controller_body";

        [Header("Bridge Switches")]
        public bool subscribeThrusters = true;
        public bool subscribeReset = true;
        public bool publishPose = true;
        public bool publishImu = true;
        public bool publishDvl = true;
        public bool publishDepth = true;
        [Tooltip("Publish the actual force and torque applied by all eight thrusters. The DVL message uses linear=[Fx,Fy,Fz] N and angular=[Mx,My,Mz] N*m in controllerBodyFrameId.")]
        public bool publishThrusterAppliedWrench = false;
        public VehicleRosStatePublishMode statePublishMode = VehicleRosStatePublishMode.SimTruth;

        [Header("Controller Sim Truth")]
        public float controllerWaterSurfaceY = 0f;

        [Header("Thruster Command")]
        public VehicleRosThrusterCommandMode thrusterCommandMode = VehicleRosThrusterCommandMode.ForceN;
        public bool autoResolveThrustersFromChildren = true;
        public bool zeroThrustersWhenDisabled = true;
        public bool logThrusterResolution = true;
        [Tooltip("Re-apply the last received ROS thruster command every FixedUpdate. This keeps force commands active even if the bridge receives a constant ROS array only once.")]
        public bool reapplyLastThrusterCommandInFixedUpdate = true;

        [Header("gRPC Stream Health")]
        [Tooltip("Seconds between checks that recreate ROS command streams if they ended after an adapter restart.")]
        public float remoteControlStreamHealthCheckSec = 1f;

        [Header("Control Conflicts")]
        public bool disableManualControllerOnEnable = true;

        [Header("Publishing Rates")]
        public float posePublishHz = 30f;
        public float imuPublishHz = 30f;
        public float dvlPublishHz = 30f;
        public float depthPublishHz = 30f;
        public string thrusterAppliedWrenchTopic = "/finsrov/debug/thruster_applied_wrench";
        public float thrusterAppliedWrenchPublishHz = 30f;

        [Header("Queue Sizes")]
        public int poseQueueSize = 64;
        public int imuQueueSize = 64;
        public int dvlQueueSize = 64;
        public int depthQueueSize = 64;
        public int thrusterAppliedWrenchQueueSize = 64;

        [Header("IMU Settings")]
        public bool removeGravityFromAcceleration = true;

        [Header("Optional Normalized Fallback")]
        [Tooltip("Optional plugin-specific normalized-throttle target. For example, the private DWP2 extension supplies one for AdvancedShipController.")]
        public MonoBehaviour thrusterCommandFallbackOverride;

        [Header("Runtime Debug")]
        [SerializeField] int receivedThrusterCommandCount;
        [SerializeField] float lastReceivedThrusterCommandTime;
        [SerializeField] float lastReceivedThrusterCommandMaxAbs;

        [Header("Overrides")]
        public ThrusterController thrusterControllerOverride;

        Rigidbody _rigidBody;
        ThrusterController _thrusterController;
        IVehicleRosThrusterFallback _thrusterCommandFallback;
        Behaviour _manualThrusterController;
        readonly MarusThruster[] _orderedThrusters = new MarusThruster[ThrusterCount];

        StreamWriterWorker<PoseStreamingRequest> _poseWriter;
        StreamWriterWorker<ImuStreamingRequest> _imuWriter;
        StreamWriterWorker<DvlStreamingRequest> _dvlWriter;
        StreamWriterWorker<DepthStreamingRequest> _depthWriter;
        StreamWriterWorker<DvlStreamingRequest> _thrusterAppliedWrenchWriter;
        ServerStreamer<ForceResponse> _thrusterStreamer;
        ServerStreamer<ForceResponse> _resetStreamer;
        RosConnection _rosConnection;

        double _lastPosePublishTime;
        double _lastImuPublishTime;
        double _lastDvlPublishTime;
        double _lastDepthPublishTime;
        double _lastThrusterAppliedWrenchPublishTime;
        double _lastImuSampleTime;
        Vector3 _lastLocalVelocity;
        bool _loggedFallbackForceWarning;
        bool _manualControllerWasEnabled;
        bool _manualControllerDisabledByBridge;
        bool _hasLastThrusterCommand;
        readonly float[] _lastThrusterCommandValues = new float[ThrusterCount];
        float _nextRemoteControlStreamHealthCheckTime;

        Vector3 _initialPosition;
        Quaternion _initialRotation;
        Vector3 _initialLinearVelocity;
        Vector3 _initialAngularVelocity;

        public static string NormalizeBridgeName(string value, string fallbackObjectName)
        {
            var raw = string.IsNullOrWhiteSpace(value) ? fallbackObjectName : value;
            raw = string.IsNullOrWhiteSpace(raw) ? "vehicle" : raw.Trim();
            return raw.Trim('/').ToLowerInvariant();
        }

        public static string BuildDefaultTopic(string bridgeName, string suffix)
        {
            return NormalizeTopic("", $"{bridgeName.Trim('/')}/{suffix.Trim('/')}");
        }

        void Reset()
        {
            NormalizeConfiguration();
        }

        void Awake()
        {
            if (!enabled || Application.isBatchMode)
            {
                Debug.LogWarning($"{nameof(VehicleRosBridge)} is disabled for this run. Skipping Awake initialization.", this);
                return;
            }

            _rigidBody = GetComponent<Rigidbody>();
            ResolveRuntimeReferences();
            NormalizeConfiguration();
            CacheInitialState();
            InitializeObservationState();
        }

        void OnEnable()
        {
            if (Application.isBatchMode)
            {
                return;
            }

            ResolveRuntimeReferences();
            DisableManualControllerIfNeeded();
        }

        void Start()
        {
            if (Application.isBatchMode)
            {
                return;
            }

            _rosConnection = RosConnection.Instance;
            _rosConnection.OnConnected += HandleRosConnected;
            if (_rosConnection.IsConnected)
            {
                HandleRosConnected(_rosConnection.StreamingChannel);
            }
        }

        void HandleRosConnected(Grpc.Core.Channel channel)
        {
            if (!enabled || Application.isBatchMode || channel == null)
            {
                return;
            }

            StopWriters();
            StopStream(ref _thrusterStreamer, "thruster");
            StopStream(ref _resetStreamer, "reset");

            var sensorClient = _rosConnection.GetClient<SensorStreamingClient>();

            if (publishPose)
            {
                _poseWriter = new StreamWriterWorker<PoseStreamingRequest>(
                    sensorClient.StreamPoseSensor,
                    Mathf.Max(1, poseQueueSize),
                    "pose");
                _poseWriter.Start(_rosConnection.CancellationToken);
            }

            if (publishImu)
            {
                _imuWriter = new StreamWriterWorker<ImuStreamingRequest>(
                    sensorClient.StreamImuSensor,
                    Mathf.Max(1, imuQueueSize),
                    "imu");
                _imuWriter.Start(_rosConnection.CancellationToken);
            }

            if (publishDvl)
            {
                _dvlWriter = new StreamWriterWorker<DvlStreamingRequest>(
                    sensorClient.StreamDvlSensor,
                    Mathf.Max(1, dvlQueueSize),
                    "dvl");
                _dvlWriter.Start(_rosConnection.CancellationToken);
            }

            if (publishDepth)
            {
                _depthWriter = new StreamWriterWorker<DepthStreamingRequest>(
                    sensorClient.StreamDepthSensor,
                    Mathf.Max(1, depthQueueSize),
                    "depth");
                _depthWriter.Start(_rosConnection.CancellationToken);
            }

            if (publishThrusterAppliedWrench)
            {
                _thrusterAppliedWrenchWriter = new StreamWriterWorker<DvlStreamingRequest>(
                    sensorClient.StreamDvlSensor,
                    Mathf.Max(1, thrusterAppliedWrenchQueueSize),
                    "thruster_applied_wrench");
                _thrusterAppliedWrenchWriter.Start(_rosConnection.CancellationToken);
                Debug.Log(
                    $"[{nameof(VehicleRosBridge)}] Publishing applied thruster wrench on " +
                    $"{thrusterAppliedWrenchTopic} at {thrusterAppliedWrenchPublishHz:F1} Hz.",
                    this);
            }

            EnsureRemoteControlStreams(forceRestart: true);
        }

        void FixedUpdate()
        {
            if (!RosConnection.Instance.IsConnected)
            {
                return;
            }

            // Accelerated lockstep can execute several FixedUpdate calls
            // before Unity reaches Update.  Drain the remote command stream
            // here as well so the controller command acknowledged for the
            // preceding /clock tick is available before the next physics
            // integration.  The normal frame-driven path retains its Update
            // handling below.
            if (_rosConnection != null && _rosConnection.Ros2ControlLockstep)
            {
                _thrusterStreamer?.HandleNewMessages();
                _resetStreamer?.HandleNewMessages();
            }

            var now = Time.fixedTimeAsDouble;
            var publishRawState = statePublishMode == VehicleRosStatePublishMode.RawFusion
                || statePublishMode == VehicleRosStatePublishMode.RawAndSimTruth;
            var publishSimTruthState = statePublishMode == VehicleRosStatePublishMode.SimTruth
                || statePublishMode == VehicleRosStatePublishMode.RawAndSimTruth;

            if (publishPose && ShouldPublish(now, ref _lastPosePublishTime, posePublishHz))
            {
                if (publishRawState)
                {
                    _poseWriter?.Enqueue(BuildPoseRequest());
                }
                if (publishSimTruthState)
                {
                    _poseWriter?.Enqueue(BuildControllerPoseRequest());
                }
            }

            if (publishImu && ShouldPublish(now, ref _lastImuPublishTime, imuPublishHz))
            {
                var imuSample = SampleImu(now);
                if (publishRawState)
                {
                    _imuWriter?.Enqueue(BuildImuRequest(imuSample));
                }
                if (publishSimTruthState)
                {
                    _imuWriter?.Enqueue(BuildControllerImuRequest(imuSample));
                }
            }

            if (publishDvl && ShouldPublish(now, ref _lastDvlPublishTime, dvlPublishHz))
            {
                if (publishRawState)
                {
                    _dvlWriter?.Enqueue(BuildDvlRequest());
                }
                if (publishSimTruthState)
                {
                    _dvlWriter?.Enqueue(BuildControllerDvlRequest());
                }
            }

            if (publishThrusterAppliedWrench
                && ShouldPublish(now, ref _lastThrusterAppliedWrenchPublishTime, thrusterAppliedWrenchPublishHz))
            {
                _thrusterAppliedWrenchWriter?.Enqueue(BuildThrusterAppliedWrenchRequest());
            }

            if (publishDepth && ShouldPublish(now, ref _lastDepthPublishTime, depthPublishHz))
            {
                if (publishRawState)
                {
                    _depthWriter?.Enqueue(BuildDepthRequest());
                }
                if (publishSimTruthState)
                {
                    _depthWriter?.Enqueue(BuildControllerDepthRequest());
                }
            }

            if (reapplyLastThrusterCommandInFixedUpdate && _hasLastThrusterCommand)
            {
                ApplyStoredThrusterCommand();
            }
        }

        void Update()
        {
            EnsureRemoteControlStreams(forceRestart: false);
            _thrusterStreamer?.HandleNewMessages();
            _resetStreamer?.HandleNewMessages();
        }

        void OnDisable()
        {
            if (_rosConnection != null)
            {
                _rosConnection.OnConnected -= HandleRosConnected;
                _rosConnection = null;
            }

            StopWriters();
            StopStream(ref _thrusterStreamer, "thruster");
            StopStream(ref _resetStreamer, "reset");

            if (zeroThrustersWhenDisabled)
            {
                ZeroAllThrusters();
            }

            RestoreManualControllerState();
        }

        void OnDestroy()
        {
            if (_rosConnection != null)
            {
                _rosConnection.OnConnected -= HandleRosConnected;
                _rosConnection = null;
            }

            StopWriters();
            StopStream(ref _thrusterStreamer, "thruster");
            StopStream(ref _resetStreamer, "reset");
            RestoreManualControllerState();
        }

        public bool ApplyReceivedThrusterValues(IList<float> values)
        {
            if (values == null || values.Count != ThrusterCount)
            {
                Debug.LogWarning(
                    $"[{nameof(VehicleRosBridge)}] Drop thruster command: expected {ThrusterCount} values, got {values?.Count ?? 0}.",
                    this);
                return false;
            }

            for (int i = 0; i < values.Count; i++)
            {
                if (float.IsNaN(values[i]) || float.IsInfinity(values[i]))
                {
                    Debug.LogWarning($"[{nameof(VehicleRosBridge)}] Drop thruster command: non-finite value at index {i}.", this);
                    return false;
                }
            }

            receivedThrusterCommandCount++;
            lastReceivedThrusterCommandTime = Time.time;
            lastReceivedThrusterCommandMaxAbs = 0f;
            for (int i = 0; i < values.Count; i++)
            {
                lastReceivedThrusterCommandMaxAbs = Mathf.Max(lastReceivedThrusterCommandMaxAbs, Mathf.Abs(values[i]));
            }

            if (receivedThrusterCommandCount <= 5 || receivedThrusterCommandCount % 50 == 0)
            {
                Debug.Log(
                    $"[{nameof(VehicleRosBridge)}] Received thruster command #{receivedThrusterCommandCount} " +
                    $"from `{thrusterTopic}`, maxAbs={lastReceivedThrusterCommandMaxAbs:F3}.",
                    this);
            }

            StoreThrusterCommand(values);

            ResolveRuntimeReferences();
            if (_thrusterController != null && TryResolveOrderedThrusters())
            {
                ApplyThrusterControllerCommand(values);
                return true;
            }

            if (_thrusterCommandFallback != null && _thrusterCommandFallback.IsAvailable)
            {
                if (thrusterCommandMode == VehicleRosThrusterCommandMode.ForceN)
                {
                    if (!_loggedFallbackForceWarning)
                    {
                        Debug.LogWarning(
                            $"[{nameof(VehicleRosBridge)}] ForceN commands require Marus ThrusterController. " +
                            "The optional normalized-throttle fallback only supports NormalizedDirect.",
                            this);
                        _loggedFallbackForceWarning = true;
                    }
                    return false;
                }

                return _thrusterCommandFallback.ApplyNormalized(values);
            }

            Debug.LogWarning($"[{nameof(VehicleRosBridge)}] No supported thruster controller found.", this);
            return false;
        }

        void StoreThrusterCommand(IList<float> values)
        {
            for (int i = 0; i < ThrusterCount; i++)
            {
                _lastThrusterCommandValues[i] = values[i];
            }

            _hasLastThrusterCommand = true;
        }

        void ApplyStoredThrusterCommand()
        {
            ResolveRuntimeReferences();
            if (_thrusterController != null && TryResolveOrderedThrusters())
            {
                ApplyThrusterControllerCommand(_lastThrusterCommandValues);
                return;
            }

            if (_thrusterCommandFallback != null && _thrusterCommandFallback.IsAvailable &&
                thrusterCommandMode == VehicleRosThrusterCommandMode.NormalizedDirect)
            {
                _thrusterCommandFallback.ApplyNormalized(_lastThrusterCommandValues);
            }
        }

        public IReadOnlyList<MarusThruster> GetResolvedOrderedThrustersForTests()
        {
            ResolveRuntimeReferences();
            TryResolveOrderedThrusters();
            return _orderedThrusters;
        }

        void ResolveRuntimeReferences()
        {
            if (_rigidBody == null)
            {
                _rigidBody = GetComponent<Rigidbody>();
            }

            _thrusterController = thrusterControllerOverride != null
                ? thrusterControllerOverride
                : GetComponent<ThrusterController>();
            if (_thrusterController == null)
            {
                _thrusterController = GetComponentInChildren<ThrusterController>(true);
            }

            _thrusterCommandFallback = ResolveThrusterCommandFallback();

            if (_manualThrusterController == null)
            {
                _manualThrusterController = GetComponent("FinsROVManualThrusterController") as Behaviour;
            }

        }

        IVehicleRosThrusterFallback ResolveThrusterCommandFallback()
        {
            if (thrusterCommandFallbackOverride is IVehicleRosThrusterFallback overrideFallback)
            {
                return overrideFallback;
            }

            foreach (MonoBehaviour behaviour in GetComponents<MonoBehaviour>())
            {
                if (behaviour is IVehicleRosThrusterFallback fallback)
                {
                    return fallback;
                }
            }

            return null;
        }

        void DisableManualControllerIfNeeded()
        {
            if (!disableManualControllerOnEnable || _manualThrusterController == null)
            {
                return;
            }

            _manualControllerWasEnabled = _manualThrusterController.enabled;
            if (_manualControllerWasEnabled)
            {
                _manualThrusterController.enabled = false;
                _manualControllerDisabledByBridge = true;
            }
        }

        void RestoreManualControllerState()
        {
            if (!_manualControllerDisabledByBridge || _manualThrusterController == null)
            {
                return;
            }

            _manualThrusterController.enabled = _manualControllerWasEnabled;
            _manualControllerDisabledByBridge = false;
        }

        void NormalizeConfiguration()
        {
            var normalizedBridgeName = NormalizeBridgeName(bridgeName, gameObject.name);
            bridgeName = normalizedBridgeName;

            thrusterTopic = NormalizeTopic(thrusterTopic, $"{normalizedBridgeName}/thrusters_out");
            poseTopic = NormalizeTopic(poseTopic, $"{normalizedBridgeName}/pose");
            imuTopic = NormalizeTopic(imuTopic, $"{normalizedBridgeName}/imu_link");
            dvlTopic = NormalizeTopic(dvlTopic, $"{normalizedBridgeName}/dvl_link");
            depthTopic = NormalizeTopic(depthTopic, $"{normalizedBridgeName}/depth_link");
            resetTopic = NormalizeTopic(resetTopic, $"{normalizedBridgeName}/reset");
            controllerPoseTopic = NormalizeTopic(controllerPoseTopic, $"{normalizedBridgeName}/controller/pose");
            controllerImuTopic = NormalizeTopic(controllerImuTopic, $"{normalizedBridgeName}/controller/imu");
            controllerDvlTopic = NormalizeTopic(controllerDvlTopic, $"{normalizedBridgeName}/controller/dvl");
            controllerDepthTopic = NormalizeTopic(controllerDepthTopic, $"{normalizedBridgeName}/controller/depth");
            thrusterAppliedWrenchTopic = NormalizeTopic(thrusterAppliedWrenchTopic, $"{normalizedBridgeName}/debug/thruster_applied_wrench");

            var objectName = string.IsNullOrWhiteSpace(gameObject.name) ? normalizedBridgeName : gameObject.name;
            poseFrameId = NormalizeFrameId(poseFrameId, $"{objectName}/pose_frame");
            imuFrameId = NormalizeFrameId(imuFrameId, $"{objectName}/imu_link_frame");
            dvlFrameId = NormalizeFrameId(dvlFrameId, $"{objectName}/dvl_link_frame");
            depthFrameId = NormalizeFrameId(depthFrameId, $"{objectName}/depth_link_frame");
            controllerWorldFrameId = NormalizeFrameId(controllerWorldFrameId, "controller_world");
            controllerBodyFrameId = NormalizeFrameId(controllerBodyFrameId, "controller_body");

            posePublishHz = Mathf.Max(1f, posePublishHz);
            imuPublishHz = Mathf.Max(1f, imuPublishHz);
            dvlPublishHz = Mathf.Max(1f, dvlPublishHz);
            depthPublishHz = Mathf.Max(1f, depthPublishHz);
            thrusterAppliedWrenchPublishHz = Mathf.Max(1f, thrusterAppliedWrenchPublishHz);

            poseQueueSize = Mathf.Max(1, poseQueueSize);
            imuQueueSize = Mathf.Max(1, imuQueueSize);
            dvlQueueSize = Mathf.Max(1, dvlQueueSize);
            depthQueueSize = Mathf.Max(1, depthQueueSize);
            thrusterAppliedWrenchQueueSize = Mathf.Max(1, thrusterAppliedWrenchQueueSize);

        }

        static string NormalizeTopic(string topic, string fallback)
        {
            var normalized = string.IsNullOrWhiteSpace(topic) ? fallback : topic.Trim();
            return normalized.StartsWith("/") ? normalized : $"/{normalized}";
        }

        static string NormalizeFrameId(string frameId, string fallback)
        {
            var normalized = string.IsNullOrWhiteSpace(frameId) ? fallback : frameId.Trim();
            return normalized.TrimStart('/');
        }

        bool TryResolveOrderedThrusters()
        {
            Array.Clear(_orderedThrusters, 0, _orderedThrusters.Length);

            var uniqueThrusters = new List<MarusThruster>();
            var thrustersByName = new Dictionary<string, MarusThruster>(StringComparer.Ordinal);

            void AddThruster(MarusThruster thruster)
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

            if (_thrusterController != null && _thrusterController.thrusters != null)
            {
                foreach (var thruster in _thrusterController.thrusters)
                {
                    AddThruster(thruster);
                }
            }

            if (autoResolveThrustersFromChildren)
            {
                foreach (var thruster in GetComponentsInChildren<MarusThruster>(true))
                {
                    AddThruster(thruster);
                }
            }

            if (uniqueThrusters.Count == 0)
            {
                return false;
            }

            var usedThrusters = new HashSet<MarusThruster>();
            for (int i = 0; i < ThrusterCount; i++)
            {
                foreach (var alias in CanonicalThrusterNameAliases[i])
                {
                    if (thrustersByName.TryGetValue(alias, out var thruster))
                    {
                        _orderedThrusters[i] = thruster;
                        usedThrusters.Add(thruster);
                        break;
                    }
                }
            }

            int fallbackIndex = 0;
            for (int i = 0; i < ThrusterCount; i++)
            {
                if (_orderedThrusters[i] != null)
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

                _orderedThrusters[i] = uniqueThrusters[fallbackIndex];
                usedThrusters.Add(uniqueThrusters[fallbackIndex]);
                fallbackIndex++;
            }

            int resolvedCount = 0;
            for (int i = 0; i < ThrusterCount; i++)
            {
                if (_orderedThrusters[i] != null)
                {
                    resolvedCount++;
                }
            }

            if (resolvedCount != ThrusterCount)
            {
                Debug.LogWarning($"[{nameof(VehicleRosBridge)}] Resolved {resolvedCount}/{ThrusterCount} thrusters.", this);
                return false;
            }

            if (_thrusterController != null)
            {
                if (_thrusterController.thrusters == null)
                {
                    _thrusterController.thrusters = new List<MarusThruster>();
                }
                _thrusterController.thrusters.Clear();
                _thrusterController.thrusters.AddRange(_orderedThrusters);
            }

            if (logThrusterResolution)
            {
                Debug.Log($"[{nameof(VehicleRosBridge)}] Resolved {resolvedCount}/{ThrusterCount} thrusters in canonical order.", this);
                logThrusterResolution = false;
            }
            return true;
        }

        void ApplyThrusterControllerCommand(IList<float> values)
        {
            if (thrusterCommandMode == VehicleRosThrusterCommandMode.NormalizedDirect)
            {
                var normalized = new float[ThrusterCount];
                for (int i = 0; i < ThrusterCount; i++)
                {
                    normalized[i] = Mathf.Clamp(values[i], -1f, 1f);
                }
                _thrusterController.ApplyInput(normalized);
                return;
            }

            for (int i = 0; i < ThrusterCount; i++)
            {
                _orderedThrusters[i].ApplyForceRequest(values[i]);
            }
        }

        void ApplyThrusterCommand(ForceResponse response)
        {
            if (response?.Pwm == null)
            {
                return;
            }

            var values = new float[response.Pwm.Data.Count];
            for (int i = 0; i < response.Pwm.Data.Count; i++)
            {
                values[i] = response.Pwm.Data[i];
            }
            ApplyReceivedThrusterValues(values);
        }

        void CacheInitialState()
        {
            _initialPosition = transform.position;
            _initialRotation = transform.rotation;
            if (_rigidBody == null)
            {
                return;
            }

            _initialLinearVelocity = _rigidBody.linearVelocity;
            _initialAngularVelocity = _rigidBody.angularVelocity;
        }

        void ApplyReset()
        {
            ZeroAllThrusters();
            if (_rigidBody == null)
            {
                return;
            }

            _rigidBody.position = _initialPosition;
            _rigidBody.rotation = _initialRotation;
            transform.SetPositionAndRotation(_initialPosition, _initialRotation);
            _rigidBody.linearVelocity = _initialLinearVelocity;
            _rigidBody.angularVelocity = _initialAngularVelocity;
            _rigidBody.Sleep();
            _rigidBody.WakeUp();
            ZeroAllThrusters();

            Debug.Log($"[{nameof(VehicleRosBridge)}] Reset vehicle state from `{resetTopic}`.", this);
        }

        void ZeroAllThrusters()
        {
            ResolveRuntimeReferences();
            if (_thrusterController != null && TryResolveOrderedThrusters())
            {
                for (int i = 0; i < ThrusterCount; i++)
                {
                    _orderedThrusters[i].ApplyForceRequest(0f);
                }
            }

            _thrusterCommandFallback?.Zero();
        }

        void InitializeObservationState()
        {
            var now = Time.timeAsDouble;
            _lastImuSampleTime = now;
            _lastLocalVelocity = GetBodyLinearVelocity();
        }

        Vector3 GetBodyLinearVelocity()
        {
            return _rigidBody != null ? transform.InverseTransformDirection(_rigidBody.linearVelocity) : Vector3.zero;
        }

        static bool ShouldPublish(double now, ref double lastPublishTime, float publishHz)
        {
            if (publishHz <= 0f)
            {
                return false;
            }

            var period = 1.0 / publishHz;
            if (lastPublishTime <= 0.0 || now - lastPublishTime >= period)
            {
                lastPublishTime = now;
                return true;
            }

            return false;
        }

        PoseStreamingRequest BuildPoseRequest()
        {
            var positionEnu = transform.position.Unity2Map();
            var orientationEnu = transform.rotation.Unity2Map();

            return new PoseStreamingRequest
            {
                Address = poseTopic,
                Data = new MsgPoseWithCovarianceStamped
                {
                    Header = new Header
                    {
                        FrameId = poseFrameId,
                        Timestamp = TimeHandler.Instance.TimeDouble,
                    },
                    Pose = new MsgPoseWithCovariance
                    {
                        Pose = new MsgPose
                        {
                            Position = new MsgPoint
                            {
                                X = positionEnu.x,
                                Y = positionEnu.y,
                                Z = positionEnu.z,
                            },
                            Orientation = orientationEnu.AsMsg(),
                        },
                    },
                },
            };
        }

        PoseStreamingRequest BuildControllerPoseRequest()
        {
            var position = transform.position;
            position.y -= controllerWaterSurfaceY;

            return new PoseStreamingRequest
            {
                Address = controllerPoseTopic,
                Data = new MsgPoseWithCovarianceStamped
                {
                    Header = new Header
                    {
                        FrameId = controllerWorldFrameId,
                        Timestamp = TimeHandler.Instance.TimeDouble,
                    },
                    Pose = new MsgPoseWithCovariance
                    {
                        Pose = new MsgPose
                        {
                            Position = new MsgPoint
                            {
                                X = position.x,
                                Y = position.y,
                                Z = position.z,
                            },
                            Orientation = transform.rotation.AsMsg(),
                        },
                    },
                },
            };
        }

        ImuSample SampleImu(double now)
        {
            var localVelocity = GetBodyLinearVelocity();
            var dt = Mathf.Max((float)(now - _lastImuSampleTime), 1e-6f);
            var linearAcceleration = (localVelocity - _lastLocalVelocity) / dt;
            if (removeGravityFromAcceleration)
            {
                linearAcceleration -= transform.InverseTransformDirection(Physics.gravity);
            }

            _lastImuSampleTime = now;
            _lastLocalVelocity = localVelocity;

            return new ImuSample(linearAcceleration);
        }

        ImuStreamingRequest BuildImuRequest(ImuSample sample)
        {
            return new ImuStreamingRequest
            {
                Address = imuTopic,
                Data = new Imu
                {
                    Header = new Header
                    {
                        FrameId = imuFrameId,
                        Timestamp = TimeHandler.Instance.TimeDouble,
                    },
                    Orientation = transform.rotation.Unity2Map().AsMsg(),
                    AngularVelocity = (-_rigidBody.angularVelocity).Unity2Body().AsMsg(),
                    LinearAcceleration = sample.LocalLinearAcceleration.Unity2Body().AsMsg(),
                },
            };
        }

        ImuStreamingRequest BuildControllerImuRequest(ImuSample sample)
        {
            return new ImuStreamingRequest
            {
                Address = controllerImuTopic,
                Data = new Imu
                {
                    Header = new Header
                    {
                        FrameId = controllerBodyFrameId,
                        Timestamp = TimeHandler.Instance.TimeDouble,
                    },
                    Orientation = transform.rotation.AsMsg(),
                    AngularVelocity = transform.InverseTransformDirection(_rigidBody.angularVelocity).AsMsg(),
                    LinearAcceleration = sample.LocalLinearAcceleration.AsMsg(),
                },
            };
        }

        DvlStreamingRequest BuildDvlRequest()
        {
            var localVelocity = GetBodyLinearVelocity();
            return new DvlStreamingRequest
            {
                Address = dvlTopic,
                Data = new MsgTwistWithCovarianceStamped
                {
                    Header = new Header
                    {
                        FrameId = dvlFrameId,
                        Timestamp = TimeHandler.Instance.TimeDouble,
                    },
                    Twist = new MsgTwistWithCovariance
                    {
                        Twist = new MsgTwist
                        {
                            Linear = localVelocity.Unity2Body().AsMsg(),
                        },
                    },
                },
            };
        }

        DvlStreamingRequest BuildControllerDvlRequest()
        {
            var localVelocity = GetBodyLinearVelocity();
            return new DvlStreamingRequest
            {
                Address = controllerDvlTopic,
                Data = new MsgTwistWithCovarianceStamped
                {
                    Header = new Header
                    {
                        FrameId = controllerBodyFrameId,
                        Timestamp = TimeHandler.Instance.TimeDouble,
                    },
                    Twist = new MsgTwistWithCovariance
                    {
                        Twist = new MsgTwist
                        {
                            Linear = localVelocity.AsMsg(),
                        },
                    },
                },
            };
        }

        DvlStreamingRequest BuildThrusterAppliedWrenchRequest()
        {
            ResolveRuntimeReferences();
            TryResolveOrderedThrusters();

            Vector3 worldForce = Vector3.zero;
            Vector3 worldTorque = Vector3.zero;
            Vector3 torqueOrigin = _rigidBody != null ? _rigidBody.worldCenterOfMass : transform.position;
            for (int index = 0; index < ThrusterCount; index++)
            {
                MarusThruster thruster = _orderedThrusters[index];
                if (thruster == null)
                {
                    continue;
                }

                Vector3 force = thruster.GetAppliedWorldForce();
                worldForce += force;
                worldTorque += Vector3.Cross(thruster.GetAppliedForcePosition() - torqueOrigin, force);
            }

            return new DvlStreamingRequest
            {
                Address = thrusterAppliedWrenchTopic,
                Data = new MsgTwistWithCovarianceStamped
                {
                    Header = new Header
                    {
                        FrameId = controllerBodyFrameId,
                        Timestamp = TimeHandler.Instance.TimeDouble,
                    },
                    Twist = new MsgTwistWithCovariance
                    {
                        Twist = new MsgTwist
                        {
                            Linear = transform.InverseTransformDirection(worldForce).AsMsg(),
                            Angular = transform.InverseTransformDirection(worldTorque).AsMsg(),
                        },
                    },
                },
            };
        }

        DepthStreamingRequest BuildDepthRequest()
        {
            var depthMeters = -transform.position.y;
            return new DepthStreamingRequest
            {
                Address = depthTopic,
                Data = new MsgPoseWithCovarianceStamped
                {
                    Header = new Header
                    {
                        FrameId = depthFrameId,
                        Timestamp = TimeHandler.Instance.TimeDouble,
                    },
                    Pose = new MsgPoseWithCovariance
                    {
                        Pose = new MsgPose
                        {
                            Position = new MsgPoint
                            {
                                X = 0.0,
                                Y = 0.0,
                                Z = depthMeters,
                            },
                            Orientation = new MsgQuaternion(),
                        },
                    },
                },
            };
        }

        DepthStreamingRequest BuildControllerDepthRequest()
        {
            var position = transform.position;
            position.y -= controllerWaterSurfaceY;
            return new DepthStreamingRequest
            {
                Address = controllerDepthTopic,
                Data = new MsgPoseWithCovarianceStamped
                {
                    Header = new Header
                    {
                        FrameId = controllerWorldFrameId,
                        Timestamp = TimeHandler.Instance.TimeDouble,
                    },
                    Pose = new MsgPoseWithCovariance
                    {
                        Pose = new MsgPose
                        {
                            Position = new MsgPoint
                            {
                                X = 0.0,
                                Y = position.y,
                                Z = 0.0,
                            },
                            Orientation = new MsgQuaternion(),
                        },
                    },
                },
            };
        }

        readonly struct ImuSample
        {
            public readonly Vector3 LocalLinearAcceleration;

            public ImuSample(Vector3 localLinearAcceleration)
            {
                LocalLinearAcceleration = localLinearAcceleration;
            }
        }

        void StopWriters()
        {
            _poseWriter?.Stop();
            _imuWriter?.Stop();
            _dvlWriter?.Stop();
            _depthWriter?.Stop();
            _thrusterAppliedWrenchWriter?.Stop();
            _poseWriter = null;
            _imuWriter = null;
            _dvlWriter = null;
            _depthWriter = null;
            _thrusterAppliedWrenchWriter = null;
        }

        void EnsureRemoteControlStreams(bool forceRestart)
        {
            if (!enabled || Application.isBatchMode || _rosConnection == null || !_rosConnection.IsConnected)
            {
                return;
            }

            if (!forceRestart && Time.unscaledTime < _nextRemoteControlStreamHealthCheckTime)
            {
                return;
            }

            _nextRemoteControlStreamHealthCheckTime = Time.unscaledTime + Mathf.Max(0.2f, remoteControlStreamHealthCheckSec);

            RemoteControlClient remoteControlClient;
            try
            {
                remoteControlClient = _rosConnection.GetClient<RemoteControlClient>();
            }
            catch (Exception exception)
            {
                Debug.LogWarning($"[{nameof(VehicleRosBridge)}] Remote control client is not available: {exception.Message}", this);
                return;
            }

            if (subscribeThrusters && (forceRestart || _thrusterStreamer == null || !_thrusterStreamer.IsStreaming))
            {
                StopStream(ref _thrusterStreamer, "thruster");
                StartThrusterStream(remoteControlClient);
            }

            if (subscribeReset && (forceRestart || _resetStreamer == null || !_resetStreamer.IsStreaming))
            {
                StopStream(ref _resetStreamer, "reset");
                StartResetStream(remoteControlClient);
            }
        }

        void StartThrusterStream(RemoteControlClient remoteControlClient)
        {
            try
            {
                _thrusterStreamer = new ServerStreamer<ForceResponse>(ApplyThrusterCommand);
                _thrusterStreamer.StartStream(
                    remoteControlClient.ApplyForce(
                        new ForceRequest { Address = thrusterTopic },
                        cancellationToken: _rosConnection.CancellationToken));
                Debug.Log($"[{nameof(VehicleRosBridge)}] Listening for thrusters on {thrusterTopic}.", this);
            }
            catch (Exception exception)
            {
                Debug.LogWarning($"[{nameof(VehicleRosBridge)}] Failed to start thruster stream on {thrusterTopic}: {exception.Message}", this);
                _thrusterStreamer = null;
            }
        }

        void StartResetStream(RemoteControlClient remoteControlClient)
        {
            try
            {
                _resetStreamer = new ServerStreamer<ForceResponse>(_ => ApplyReset());
                _resetStreamer.StartStream(
                    remoteControlClient.ApplyForce(
                        new ForceRequest { Address = resetTopic },
                        cancellationToken: _rosConnection.CancellationToken));
                Debug.Log($"[{nameof(VehicleRosBridge)}] Listening for reset on {resetTopic}.", this);
            }
            catch (Exception exception)
            {
                Debug.LogWarning($"[{nameof(VehicleRosBridge)}] Failed to start reset stream on {resetTopic}: {exception.Message}", this);
                _resetStreamer = null;
            }
        }

        void StopStream(ref ServerStreamer<ForceResponse> streamer, string streamName)
        {
            if (streamer == null)
            {
                return;
            }

            try
            {
                if (streamer.IsStreaming)
                {
                    streamer.StopStream();
                }
            }
            catch (Exception exception)
            {
                Debug.LogWarning($"[{nameof(VehicleRosBridge)}] Failed to stop {streamName} stream: {exception.Message}", this);
            }

            streamer = null;
        }

        sealed class StreamWriterWorker<TRequest> where TRequest : class, IMessage
        {
            readonly Func<Metadata, DateTime?, CancellationToken, AsyncClientStreamingCall<TRequest, Empty>> _streamFactory;
            readonly ConcurrentQueue<TRequest> _queue = new ConcurrentQueue<TRequest>();
            readonly int _maxQueueSize;
            readonly string _streamName;

            AsyncClientStreamingCall<TRequest, Empty> _streamHandle;
            Thread _writerThread;
            volatile bool _stopRequested;

            public StreamWriterWorker(
                Func<Metadata, DateTime?, CancellationToken, AsyncClientStreamingCall<TRequest, Empty>> streamFactory,
                int maxQueueSize,
                string streamName)
            {
                _streamFactory = streamFactory;
                _maxQueueSize = Mathf.Max(1, maxQueueSize);
                _streamName = streamName;
            }

            public void Start(CancellationToken cancellationToken)
            {
                _streamHandle = _streamFactory(null, null, cancellationToken);
                _stopRequested = false;
                _writerThread = new Thread(() => WriteLoop(cancellationToken))
                {
                    IsBackground = true,
                    Name = $"{nameof(VehicleRosBridge)}-{_streamName}-writer",
                };
                _writerThread.Start();
            }

            public void Enqueue(TRequest request)
            {
                if (request == null)
                {
                    return;
                }

                while (_queue.Count >= _maxQueueSize && _queue.TryDequeue(out _))
                {
                }

                _queue.Enqueue(request);
            }

            public void Stop()
            {
                _stopRequested = true;
                var streamHandle = _streamHandle;
                try
                {
                    // Do not synchronously wait for gRPC from Unity's main thread:
                    // OnDisable is also called while Unity is reloading the domain.
                    _ = streamHandle?.RequestStream.CompleteAsync();
                }
                catch (Exception exception)
                {
                    Debug.LogWarning($"[{nameof(VehicleRosBridge)}] Failed to complete `{_streamName}` stream: {exception.Message}");
                }

                if (_writerThread != null && _writerThread.IsAlive)
                {
                    _writerThread.Join(100);
                }

                _streamHandle = null;
                _writerThread = null;
            }

            void WriteLoop(CancellationToken cancellationToken)
            {
                while (!_stopRequested && !cancellationToken.IsCancellationRequested)
                {
                    if (!_queue.TryDequeue(out var request))
                    {
                        Thread.Sleep(2);
                        continue;
                    }

                    try
                    {
                        var streamHandle = _streamHandle;
                        if (streamHandle == null)
                        {
                            break;
                        }

                        streamHandle.RequestStream.WriteAsync(request).Wait(cancellationToken);
                    }
                    catch (Exception exception)
                    {
                        if (!_stopRequested && !cancellationToken.IsCancellationRequested)
                        {
                            Debug.LogWarning(
                                $"[{nameof(VehicleRosBridge)}] Failed to write `{_streamName}` stream: {exception.Message}");
                            RosConnection.Instance.ReportConnectionFailure(
                                $"{nameof(VehicleRosBridge)} `{_streamName}` stream write failed: {exception.Message}");
                        }
                        _stopRequested = true;
                        Thread.Sleep(50);
                    }
                }
            }
        }
    }
}
