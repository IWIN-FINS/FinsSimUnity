using System;
using Grpc.Core;
using FinsSim.Networking;
using NWH.DWP2.ShipController;
using Remotecontrol;
using UnityEngine;
using static Remotecontrol.RemoteControl;

[DefaultExecutionOrder(60)]
[DisallowMultipleComponent]
[RequireComponent(typeof(Rigidbody))]
[RequireComponent(typeof(AdvancedShipController))]
public class FinsROVResetRosBridge : MonoBehaviour
{
    [Header("ROS Reset")]
    public string resetTopic = "/finsrov/reset";
    public bool subscribeReset = true;

    [Header("Reset Behaviour")]
    public bool zeroThrustersBeforeReset = true;
    public bool zeroThrustersAfterReset = true;
    public bool restoreInitialLinearVelocity = true;
    public bool restoreInitialAngularVelocity = true;

    private Rigidbody _rigidBody;
    private AdvancedShipController _advancedShipController;
    private ServerStreamer<ForceResponse> _resetStreamer;
    private RosConnection _rosConnection;

    private Vector3 _initialPosition;
    private Quaternion _initialRotation;
    private Vector3 _initialLinearVelocity;
    private Vector3 _initialAngularVelocity;

    private void Reset()
    {
        if (string.IsNullOrWhiteSpace(resetTopic))
        {
            var vehicleName = string.IsNullOrWhiteSpace(gameObject.name) ? "finsrov" : gameObject.name.ToLowerInvariant();
            resetTopic = $"/{vehicleName}/reset";
        }
    }

    private void Awake()
    {
        _rigidBody = GetComponent<Rigidbody>();
        _advancedShipController = GetComponent<AdvancedShipController>();

        CacheInitialState();
        if (string.IsNullOrWhiteSpace(resetTopic))
        {
            var vehicleName = string.IsNullOrWhiteSpace(gameObject.name) ? "finsrov" : gameObject.name.ToLowerInvariant();
            resetTopic = $"/{vehicleName}/reset";
        }
    }

    private void Start()
    {
        if (!subscribeReset)
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

    private void HandleRosConnected(Channel channel)
    {
        if (!enabled || channel == null || !subscribeReset)
        {
            return;
        }

        StopResetStream();

        var remoteControlClient = _rosConnection.GetClient<RemoteControlClient>();
        _resetStreamer = new ServerStreamer<ForceResponse>(HandleResetRequest);
        _resetStreamer.StartStream(
            remoteControlClient.ApplyForce(
                new ForceRequest { Address = resetTopic },
                cancellationToken: _rosConnection.CancellationToken));

        Debug.Log($"[{nameof(FinsROVResetRosBridge)}] ROS reset stream connected: reset=`{resetTopic}`.");
    }

    private void Update()
    {
        _resetStreamer?.HandleNewMessages();
    }

    private void OnDisable()
    {
        if (_rosConnection != null)
        {
            _rosConnection.OnConnected -= HandleRosConnected;
            _rosConnection = null;
        }

        StopResetStream();
    }

    private void CacheInitialState()
    {
        _initialPosition = transform.position;
        _initialRotation = transform.rotation;
        _initialLinearVelocity = _rigidBody.linearVelocity;
        _initialAngularVelocity = _rigidBody.angularVelocity;
    }

    private void HandleResetRequest(ForceResponse response)
    {
        if (response == null)
        {
            return;
        }

        ApplyReset();
    }

    private void ApplyReset()
    {
        if (zeroThrustersBeforeReset)
        {
            ZeroAllThrusters();
        }

        _rigidBody.position = _initialPosition;
        _rigidBody.rotation = _initialRotation;
        transform.SetPositionAndRotation(_initialPosition, _initialRotation);

        if (restoreInitialLinearVelocity)
        {
            _rigidBody.linearVelocity = _initialLinearVelocity;
        }

        if (restoreInitialAngularVelocity)
        {
            _rigidBody.angularVelocity = _initialAngularVelocity;
        }

        _rigidBody.Sleep();
        _rigidBody.WakeUp();

        if (zeroThrustersAfterReset)
        {
            ZeroAllThrusters();
        }

        Debug.Log($"[{nameof(FinsROVResetRosBridge)}] Reset vehicle state from `{resetTopic}`.");
    }

    private void ZeroAllThrusters()
    {
        if (_advancedShipController == null || _advancedShipController.engines == null)
        {
            return;
        }

        foreach (var engine in _advancedShipController.engines)
        {
            if (engine == null)
            {
                continue;
            }

            engine.externalThrottleInput = 0f;
        }
    }

    private void StopResetStream()
    {
        if (_resetStreamer == null)
        {
            return;
        }

        try
        {
            if (_resetStreamer.IsStreaming)
            {
                _resetStreamer.StopStream();
            }
        }
        catch (Exception exception)
        {
            Debug.LogWarning($"[{nameof(FinsROVResetRosBridge)}] Failed to stop reset stream: {exception.Message}");
        }

        _resetStreamer = null;
    }
}
