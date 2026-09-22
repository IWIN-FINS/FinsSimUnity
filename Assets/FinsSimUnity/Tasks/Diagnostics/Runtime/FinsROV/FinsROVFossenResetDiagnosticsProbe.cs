using System;
using System.Collections.Concurrent;
using System.Threading;
using Google.Protobuf;
using Grpc.Core;
using FinsSim.Actuators;
using FinsSim.Hydrodynamics;
using FinsSim.Core.Spatial;
using FinsSim.Networking;
using Remotecontrol;
using Sensor;
using Sensorstreaming;
using Std;
using UnityEngine;
using static Remotecontrol.RemoteControl;
using static Sensorstreaming.SensorStreaming;
using MsgTwist = Geometry.Twist;
using MsgTwistWithCovariance = Geometry.TwistWithCovariance;
using MsgTwistWithCovarianceStamped = Geometry.TwistWithCovarianceStamped;

/// <summary>
/// Scene-local diagnostic probe for reproducing and inspecting a Fossen reset.
/// It publishes DVL-shaped debug messages because that is a supported outbound
/// gRPC stream in the existing Unity-to-ROS adapter.
/// </summary>
[DefaultExecutionOrder(60)]
[DisallowMultipleComponent]
[RequireComponent(typeof(Rigidbody))]
public sealed class FinsROVFossenResetDiagnosticsProbe : MonoBehaviour
{
    public enum HoldResetTriggerMode
    {
        ToggleComponent,
        DirectOnEpisodeBegin,
    }

    const string TopicPrefix = "/finsrov/debug/fossen";

    [Header("References")]
    public HydrodynamicsController hydrodynamicsController;
    public ThrusterController thrusterController;

    [Header("ROS")]
    [Tooltip("Publish any Float32MultiArray to this topic to request one reset. The gRPC adapter applies /sim automatically.")]
    public string resetTopic = TopicPrefix + "_reset";
    [Range(1f, 30f)] public float publishHz = 20f;
    [Range(1, 32)] public int perTopicQueueSize = 4;

    [Header("Reset Experiment")]
    [Tooltip("Use the HoldForPosition agent rather than the probe's fallback rigidbody reset.")]
    public bool useHoldForPositionEpisodeReset = true;
    [Tooltip("ToggleComponent exactly mirrors disabling then enabling HoldForPosition in the Inspector. DirectOnEpisodeBegin is retained only for the earlier controlled reset experiment.")]
    public HoldResetTriggerMode holdResetTriggerMode = HoldResetTriggerMode.ToggleComponent;
    [Tooltip("When disabled, reset preserves the current pose and clears only velocities, thrusters, and Fossen history.")]
    public bool resetPositionAndRotation;
    public Vector3 resetWorldPosition = new Vector3(0f, -2f, 0f);
    public Vector3 resetEulerDegrees = Vector3.zero;
    public Vector3 resetLocalLinearVelocity = Vector3.zero;
    public Vector3 resetLocalAngularVelocity = Vector3.zero;
    [Min(0f)] public float freezeThrustersAfterResetSec = 0.5f;

    [Header("Runtime Debug")]
    [SerializeField] int resetCount;
    [SerializeField] float lastResetTime;
    [SerializeField] Vector3 lastTransformAngularRateWorld;
    [SerializeField] Vector3 lastAngularVelocityMismatchWorld;
    [SerializeField] Vector3 lastUpdateLinearVelocityWorld;
    [SerializeField] Vector3 lastUpdateAngularVelocityWorld;
    [SerializeField] bool hasFossenDiagnostics;

    readonly Thruster[] orderedThrusters = new Thruster[FinsROVAgentRuntime.DefaultThrusterOrder.Length];
    readonly DebugDvlWriter[] writers = new DebugDvlWriter[11];
    readonly string[] topicSuffixes =
    {
        "state_world",
        "transform_rate_world",
        "hydro_total_world",
        "hydrostatic_body",
        "damping_body",
        "added_mass_acceleration_body",
        "added_mass_coriolis_body",
        "relative_velocity_body",
        "relative_acceleration_body",
        "reset_initial_state_world",
        "state_update_world",
    };

    Rigidbody body;
    HoldForPosition holdForPosition;
    RosConnection rosConnection;
    ServerStreamer<ForceResponse> resetStreamer;
    Quaternion previousRotation;
    Vector3 previousPosition;
    Vector3 lastTransformLinearVelocityWorld;
    bool hasPreviousTransform;
    bool resetRequested;
    float nextPublishTime;
    float nextUpdatePublishTime;
    float freezeThrustersUntil;
    float nextStreamHealthCheckTime;

    void Awake()
    {
        body = GetComponent<Rigidbody>();
        ResolveReferences();
        resetTopic = NormalizeTopic(resetTopic, TopicPrefix + "_reset");
        previousRotation = transform.rotation;
        previousPosition = transform.position;
        hasPreviousTransform = true;
    }

    void Start()
    {
        rosConnection = RosConnection.Instance;
        rosConnection.OnConnected += HandleRosConnected;
        if (rosConnection.IsConnected)
        {
            HandleRosConnected(rosConnection.StreamingChannel);
        }
    }

    void Update()
    {
        resetStreamer?.HandleNewMessages();
        if (rosConnection != null && rosConnection.IsConnected && Time.unscaledTime >= nextStreamHealthCheckTime)
        {
            nextStreamHealthCheckTime = Time.unscaledTime + 1f;
            EnsureResetStream();
        }

        // VehicleRosBridge publishes IMU from Update. Capture the same Rigidbody
        // phase separately from FixedUpdate to expose any between-step writer.
        if (rosConnection != null && rosConnection.IsConnected && Time.unscaledTime >= nextUpdatePublishTime)
        {
            nextUpdatePublishTime = Time.unscaledTime + 1f / Mathf.Max(1f, publishHz);
            lastUpdateLinearVelocityWorld = body.linearVelocity;
            lastUpdateAngularVelocityWorld = body.angularVelocity;
            PublishWorld(10, lastUpdateLinearVelocityWorld, lastUpdateAngularVelocityWorld, "world");
        }
    }

    void FixedUpdate()
    {
        if (resetRequested)
        {
            resetRequested = false;
            ExecuteReset();
        }

        if (Time.time < freezeThrustersUntil)
        {
            ZeroThrusters();
        }

        UpdateTransformKinematics();
        if (rosConnection != null && rosConnection.IsConnected && Time.fixedTime >= nextPublishTime)
        {
            nextPublishTime = Time.fixedTime + 1f / Mathf.Max(1f, publishHz);
            PublishDiagnostics();
        }
    }

    void OnDestroy()
    {
        if (rosConnection != null)
        {
            rosConnection.OnConnected -= HandleRosConnected;
        }

        StopResetStream();
        foreach (DebugDvlWriter writer in writers)
        {
            writer?.Stop();
        }
    }

    void ResolveReferences()
    {
        if (hydrodynamicsController == null)
        {
            hydrodynamicsController = GetComponent<HydrodynamicsController>();
        }

        if (thrusterController == null)
        {
            thrusterController = GetComponent<ThrusterController>();
        }

        if (holdForPosition == null)
        {
            holdForPosition = GetComponent<HoldForPosition>();
        }
    }

    void HandleRosConnected(Grpc.Core.Channel channel)
    {
        if (!enabled || channel == null)
        {
            return;
        }

        StopResetStream();
        foreach (DebugDvlWriter writer in writers)
        {
            writer?.Stop();
        }

        SensorStreamingClient sensorClient = rosConnection.GetClient<SensorStreamingClient>();
        for (int index = 0; index < writers.Length; index++)
        {
            writers[index] = new DebugDvlWriter(sensorClient, Mathf.Max(1, perTopicQueueSize), topicSuffixes[index]);
            writers[index].Start(rosConnection.CancellationToken);
        }

        EnsureResetStream();
    }

    void EnsureResetStream()
    {
        if (resetStreamer != null && resetStreamer.IsStreaming)
        {
            return;
        }

        StopResetStream();
        try
        {
            RemoteControlClient remoteClient = rosConnection.GetClient<RemoteControlClient>();
            resetStreamer = new ServerStreamer<ForceResponse>(_ => resetRequested = true);
            resetStreamer.StartStream(remoteClient.ApplyForce(
                new ForceRequest { Address = resetTopic },
                cancellationToken: rosConnection.CancellationToken));
            Debug.Log($"[{nameof(FinsROVFossenResetDiagnosticsProbe)}] Listening for reset on `{resetTopic}`.", this);
        }
        catch (Exception exception)
        {
            Debug.LogWarning($"[{nameof(FinsROVFossenResetDiagnosticsProbe)}] Failed to start reset stream: {exception.Message}", this);
            resetStreamer = null;
        }
    }

    void StopResetStream()
    {
        if (resetStreamer == null)
        {
            return;
        }

        try
        {
            resetStreamer.StopStream();
        }
        catch (Exception exception)
        {
            Debug.LogWarning($"[{nameof(FinsROVFossenResetDiagnosticsProbe)}] Failed to stop reset stream: {exception.Message}", this);
        }
        finally
        {
            resetStreamer = null;
        }
    }

    void ExecuteReset()
    {
        ResolveReferences();

        if (useHoldForPositionEpisodeReset && holdForPosition != null)
        {
            if (holdResetTriggerMode == HoldResetTriggerMode.ToggleComponent)
            {
                // This deliberately has no extra velocity, thruster, or Fossen-state
                // manipulation: it reproduces the user's Inspector toggle exactly.
                holdForPosition.enabled = false;
                holdForPosition.enabled = true;
            }
            else
            {
                holdForPosition.OnEpisodeBegin();
            }

            previousRotation = transform.rotation;
            previousPosition = transform.position;
            lastTransformLinearVelocityWorld = body.linearVelocity;
            hasPreviousTransform = true;
            freezeThrustersUntil = holdResetTriggerMode == HoldResetTriggerMode.ToggleComponent
                ? Time.time
                : Time.time + Mathf.Max(0f, freezeThrustersAfterResetSec);
            resetCount++;
            lastResetTime = Time.time;
            PublishWorld(9, body.linearVelocity, body.angularVelocity, "world");
            Debug.Log($"[{nameof(FinsROVFossenResetDiagnosticsProbe)}] Reset #{resetCount} executed through HoldForPosition ({holdResetTriggerMode}).", this);
            return;
        }

        ZeroThrusters();

        if (resetPositionAndRotation)
        {
            body.position = resetWorldPosition;
            body.rotation = Quaternion.Euler(resetEulerDegrees);
            Physics.SyncTransforms();
        }

        body.linearVelocity = transform.TransformDirection(resetLocalLinearVelocity);
        body.angularVelocity = transform.TransformDirection(resetLocalAngularVelocity);
        body.WakeUp();
        hydrodynamicsController?.ResetBackendState();

        previousRotation = transform.rotation;
        previousPosition = transform.position;
        lastTransformLinearVelocityWorld = body.linearVelocity;
        hasPreviousTransform = true;
        freezeThrustersUntil = Time.time + Mathf.Max(0f, freezeThrustersAfterResetSec);
        resetCount++;
        lastResetTime = Time.time;
        PublishWorld(9, body.linearVelocity, body.angularVelocity, "world");
        Debug.Log($"[{nameof(FinsROVFossenResetDiagnosticsProbe)}] Reset #{resetCount} executed.", this);
    }

    void ZeroThrusters()
    {
        if (FinsROVAgentRuntime.TryResolveOrderedThrusters(
            this,
            thrusterController,
            autoResolveFromChildren: true,
            orderedThrusters,
            out _))
        {
            FinsROVAgentRuntime.ZeroThrusters(orderedThrusters, ThrusterCommandMode.ScaledForceRequest, 1f);
        }
    }

    void UpdateTransformKinematics()
    {
        if (!hasPreviousTransform)
        {
            previousRotation = transform.rotation;
            previousPosition = transform.position;
            hasPreviousTransform = true;
            lastTransformLinearVelocityWorld = Vector3.zero;
            lastTransformAngularRateWorld = Vector3.zero;
            lastAngularVelocityMismatchWorld = body.angularVelocity;
            return;
        }

        float dt = Mathf.Max(Time.fixedDeltaTime, 1e-6f);
        lastTransformLinearVelocityWorld = (transform.position - previousPosition) / dt;
        lastTransformAngularRateWorld = QuaternionDeltaToAngularVelocity(previousRotation, transform.rotation, dt);
        lastAngularVelocityMismatchWorld = body.angularVelocity - lastTransformAngularRateWorld;
        previousRotation = transform.rotation;
        previousPosition = transform.position;
    }

    void PublishDiagnostics()
    {
        Fossen6DofDiagnostics diagnostics = Fossen6DofDiagnostics.Zero;
        hasFossenDiagnostics = hydrodynamicsController != null && hydrodynamicsController.TryGetFossenDiagnostics(out diagnostics);
        HydrodynamicsWrench totalWrench = hydrodynamicsController != null
            ? hydrodynamicsController.LastWrench
            : HydrodynamicsWrench.Zero;

        PublishWorld(0, body.linearVelocity, body.angularVelocity, "world");
        PublishWorld(1, lastTransformLinearVelocityWorld, lastTransformAngularRateWorld, "world");
        PublishWorld(2, totalWrench.WorldForce, totalWrench.WorldTorque, "world");
        PublishBody(3, diagnostics.HydrostaticWrench.BodyWrench);
        PublishBody(4, diagnostics.DampingBodyWrench);
        PublishBody(5, diagnostics.AddedMassAccelerationBodyWrench);
        PublishBody(6, diagnostics.AddedMassCoriolisBodyWrench);
        PublishBody(7, diagnostics.RelativeVelocity);
        PublishBody(8, diagnostics.FilteredRelativeAcceleration);
    }

    void PublishWorld(int writerIndex, Vector3 linear, Vector3 angular, string frameId)
    {
        writers[writerIndex]?.Enqueue(BuildDvlRequest(TopicPrefix + "/" + topicSuffixes[writerIndex], frameId, linear, angular));
    }

    void PublishBody(int writerIndex, SixDofVector value)
    {
        writers[writerIndex]?.Enqueue(BuildDvlRequest(
            TopicPrefix + "/" + topicSuffixes[writerIndex],
            "finsrov_fossen_body",
            value.Linear,
            value.Angular));
    }

    static DvlStreamingRequest BuildDvlRequest(string address, string frameId, Vector3 linear, Vector3 angular)
    {
        return new DvlStreamingRequest
        {
            Address = address,
            Data = new MsgTwistWithCovarianceStamped
            {
                Header = new Header
                {
                    FrameId = frameId,
                    Timestamp = FinsSim.Core.TimeHandler.Instance.TimeDouble,
                },
                Twist = new MsgTwistWithCovariance
                {
                    Twist = new MsgTwist
                    {
                        Linear = linear.AsMsg(),
                        Angular = angular.AsMsg(),
                    },
                },
            },
        };
    }

    static Vector3 QuaternionDeltaToAngularVelocity(Quaternion previous, Quaternion current, float dt)
    {
        Quaternion delta = current * Quaternion.Inverse(previous);
        delta.ToAngleAxis(out float degrees, out Vector3 axis);
        if (degrees > 180f)
        {
            degrees -= 360f;
        }

        return axis.sqrMagnitude <= 1e-8f
            ? Vector3.zero
            : axis.normalized * (degrees * Mathf.Deg2Rad / dt);
    }

    static string NormalizeTopic(string topic, string fallback)
    {
        string normalized = string.IsNullOrWhiteSpace(topic) ? fallback : topic.Trim();
        return normalized.StartsWith("/") ? normalized : "/" + normalized;
    }

    sealed class DebugDvlWriter
    {
        readonly SensorStreamingClient sensorClient;
        readonly ConcurrentQueue<DvlStreamingRequest> queue = new ConcurrentQueue<DvlStreamingRequest>();
        readonly int maxQueueSize;
        readonly string streamName;

        AsyncClientStreamingCall<DvlStreamingRequest, Empty> streamHandle;
        Thread writerThread;
        volatile bool stopRequested;

        public DebugDvlWriter(SensorStreamingClient sensorClient, int maxQueueSize, string streamName)
        {
            this.sensorClient = sensorClient;
            this.maxQueueSize = maxQueueSize;
            this.streamName = streamName;
        }

        public void Start(CancellationToken cancellationToken)
        {
            streamHandle = sensorClient.StreamDvlSensor(null, null, cancellationToken);
            stopRequested = false;
            writerThread = new Thread(() => WriteLoop(cancellationToken))
            {
                IsBackground = true,
                Name = $"{nameof(FinsROVFossenResetDiagnosticsProbe)}-{streamName}",
            };
            writerThread.Start();
        }

        public void Enqueue(DvlStreamingRequest request)
        {
            while (queue.Count >= maxQueueSize && queue.TryDequeue(out _))
            {
            }

            queue.Enqueue(request);
        }

        public void Stop()
        {
            stopRequested = true;
            try
            {
                streamHandle?.RequestStream.CompleteAsync().Wait(250);
            }
            catch (Exception)
            {
            }

            if (writerThread != null && writerThread.IsAlive)
            {
                writerThread.Join(250);
            }

            streamHandle = null;
            writerThread = null;
        }

        void WriteLoop(CancellationToken cancellationToken)
        {
            while (!stopRequested && !cancellationToken.IsCancellationRequested)
            {
                if (!queue.TryDequeue(out DvlStreamingRequest request))
                {
                    Thread.Sleep(2);
                    continue;
                }

                try
                {
                    streamHandle.RequestStream.WriteAsync(request).Wait(cancellationToken);
                }
                catch (Exception exception)
                {
                    if (!stopRequested && !cancellationToken.IsCancellationRequested)
                    {
                        Debug.LogWarning($"[{nameof(FinsROVFossenResetDiagnosticsProbe)}] `{streamName}` write failed: {exception.Message}");
                    }
                    stopRequested = true;
                }
            }
        }
    }
}
