// Copyright 2022 Laboratory for Underwater Systems and Technologies (LABUST)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

using System;
using System.Collections.Generic;
using Grpc.Core;
using UnityEngine;
using System.Threading;
using FinsSim.Core;
using static Tf.Tf;
using static Sensorstreaming.SensorStreaming;
using static Remotecontrol.RemoteControl;
using static Ping.Ping;
using static Simulationcontrol.SimulationControl;
using static Visualization.Visualization;

using System.Collections;
using Simulationcontrol;
using FinsSim.Utils;
using FinsSim.Visualization;
using FinsSim.CustomInspector;
using static Acoustictransmission.AcousticTransmission;
using static Rfcommunication.LoraTransmission;
using Ping;

namespace FinsSim.Networking
{
    /// <summary>
    /// Singleton class for configuring and connecting to
    /// ROS server
    /// </summary>
    [DefaultExecutionOrder(-1)]
    public class RosConnection : Singleton<RosConnection>
    {
        [Header("Server info")]
        public string serverIP = "localhost";
        public int serverPort = 30052;
        [HideInRuntimeInspector]
        public int connectionTimeout = 5;

        [Tooltip("Seconds between reconnect attempts after the ROS gRPC server is unavailable.")]
        public float reconnectInterval = 1f;

        [Tooltip("Seconds between lightweight ping checks while connected.")]
        public float healthCheckInterval = 1f;


        [Header("Simulation")]
        public bool DisplayTf = false;

        [HideInRuntimeInspector]
        public bool RealtimeSimulation = true;

        [ConditionalHideInInspector("RealtimeSimulation", true)]
        [HideInRuntimeInspector]
        public float SimulationSpeed = 1;

        /// <summary>
        /// When enabled, a simulation physics step is gated by a synchronous
        /// SimulationControl.Step RPC.  The ROS2 adapter returns from that RPC
        /// only after the configured motion controller acknowledges the
        /// current /clock tick.  This is intentionally an evaluation-only
        /// mode; normal MARUS stepped simulation remains frame-driven.
        /// </summary>
        [HideInRuntimeInspector]
        public bool Ros2ControlLockstep = false;


        [Header("Earth origin frame")]
        public string OriginFrameName = "map";
        public string OriginFrameLatitude = "/d2/LocalOriginLat";
        public string OriginFrameLongitude = "/d2/LocalOriginLon";

        public double DefaultLatitude = 45;
        public double DefaultLongitude = 15;

        Channel _streamingChannel;

        public Channel StreamingChannel => _streamingChannel;
        Dictionary<Type, ClientBase> _grpcClients;
        volatile bool _connected;
        public bool IsConnected => _connected;

        volatile bool _isConnecting;
        public bool IsConnecting => _isConnecting;
        volatile bool _connectedNotificationPending;
        volatile bool _shutdownRequested;

        CancellationToken _cancellationToken;
        public CancellationToken CancellationToken => _cancellationToken;

        private SimulationControlClient _simulationController;
        readonly object _connectionLock = new object();
        List<ChannelOption> _channelOptions;


        public event Action<Channel> OnConnected;

        /// <summary>
        /// Adds new client of type if it does not currently exists.
        /// </summary>
        /// <typeparam name="T"></typeparam>
        /// <returns></returns>
        public T AddNewClient<T>() where T : ClientBase
        {
            var t = typeof(T);
            if (_grpcClients.TryGetValue(t, out var clientBase))
                return clientBase as T;

            var client = Activator.CreateInstance(typeof(T), _streamingChannel) as T;
            _grpcClients.Add(t, client);
            return client as T;
        }

        /// <summary>
        /// Get gRPC client of given type.
        /// Clients are shared on the application instance
        /// </summary>
        /// <typeparam name="T"></typeparam>
        /// <returns></returns>
        public T GetClient<T>() where T : ClientBase
        {
            if (_grpcClients.TryGetValue(typeof(T), out var client))
                return client as T;
            throw new Exception($"Client of type {typeof(T).Name} does not exist.");
        }

        private void Awake()
        {
            _shutdownRequested = false;
            ThreadPool.SetMinThreads(12, 100);
            try
            {
                Grpc.Core.GrpcEnvironment.SetThreadPoolSize(10);
            }
            catch (InvalidOperationException)
            {
                // ML-Agents may initialize gRPC before the bridge singleton is created.
                // In that case the existing gRPC environment is still usable.
                Debug.Log("[RosConnection] gRPC environment already initialized; reusing it.");
            }
            _channelOptions = new List<ChannelOption>
            {
                new ChannelOption(ChannelOptions.MaxSendMessageLength, 1024*1024*100),
                new ChannelOption(ChannelOptions.MaxReceiveMessageLength, 1024*1024*100),
                // Unity is commonly launched from a shell that exports an HTTP
                // proxy.  The ROS adapter is always a local gRPC endpoint
                // (localhost or a LAN address), and routing it through that
                // proxy can leave the native Grpc.Core client retrying until
                // its RPC deadline.  Keep this channel strictly direct.
                new ChannelOption("grpc.enable_http_proxy", 0),
            };
            CreateChannelAndClients(_channelOptions);


            CreateSingletons();
            Connect();

            StartCoroutine(ConnectionSupervisor());
        }


        public void Connect()
        {
            if (!_shutdownRequested && !_connected && !_isConnecting)
            {
                _isConnecting = true;
                var t = new Thread(() =>
                {
                    try
                    {
                        if (_shutdownRequested)
                        {
                            return;
                        }

                        _connected = TryConnect();
                        _connectedNotificationPending = _connected && !_shutdownRequested;
                    }
                    finally
                    {
                        _isConnecting = false;
                    }
                    // maybe add what to do after failed connection
                    // maybe ping server repeatedly
                });
                t.IsBackground = true;
                t.Start();
            }
        }

        void CreateSingletons()
        {
            var paramServer = ParamServerHandler.Instance;
            var tfHandler = TfHandler.Instance;
            var timeHandler = TimeHandler.Instance;
            var visualizationRos = VisualizationROS.Instance;
            var pcHandler = PointCloudRosVisualizer.Instance;
        }

        IEnumerator ConnectionSupervisor()
        {
            var nextHealthCheckTime = 0f;
            while (true)
            {
                if (_shutdownRequested)
                {
                    yield break;
                }

                if (!_connected && !_isConnecting)
                {
                    Connect();
                }

                if (_connectedNotificationPending)
                {
                    _connectedNotificationPending = false;
                    var connectedChannel = _streamingChannel;
                    OnRosConnected();
                    NotifyConnectedListeners(connectedChannel);
                }

                if (_connected && Time.unscaledTime >= nextHealthCheckTime)
                {
                    nextHealthCheckTime = Time.unscaledTime + Mathf.Max(0.2f, healthCheckInterval);
                    if (!PingServer())
                    {
                        Debug.LogWarning("Lost connection to ROS Server. Recreating gRPC channel.");
                        MarkDisconnected();
                        RecreateChannel(_channelOptions);
                    }
                }

                yield return new WaitForSecondsRealtime(Mathf.Max(0.1f, reconnectInterval));
            }
        }

        void OnRosConnected()
        {
            try
            {
                if (!RealtimeSimulation)
                {
                    GetSimulationController();
                }
            }
            catch (Exception e)
            {
                Debug.LogWarning($"ROS connection setup failed: {e.Message}");
            }
        }

        void NotifyConnectedListeners(Channel channel)
        {
            var connectedHandlers = OnConnected;
            if (connectedHandlers == null)
            {
                return;
            }

            foreach (Action<Channel> handler in connectedHandlers.GetInvocationList())
            {
                try
                {
                    handler(channel);
                }
                catch (Exception e)
                {
                    string target = handler.Target != null
                        ? handler.Target.GetType().Name
                        : handler.Method.Name;
                    Debug.LogWarning($"ROS OnConnected listener `{target}` failed: {e.Message}");
                }
            }
        }

        void Update()
        {
            if (!RealtimeSimulation && !Ros2ControlLockstep && IsConnected)
            {
                StartCoroutine(RosStep());
            }
        }

        /// <summary>
        /// Advance the ROS2 simulation clock once and wait for the adapter's
        /// response.  In lockstep mode the response is an acknowledgement
        /// that the ROS2 controller has completed the matching control-tick
        /// callback (or has explicitly held its previous command).
        /// </summary>
        public bool StepRos2ControlLockstep()
        {
            if (RealtimeSimulation || !Ros2ControlLockstep || !IsConnected)
            {
                return false;
            }

            try
            {
                if (_simulationController == null)
                {
                    GetSimulationController();
                }

                var response = _simulationController.Step(
                    new StepRequest
                    {
                        TotalTimeSecs = TimeHandler.Instance.TotalTimeSecs,
                        TotalTimeNsecs = TimeHandler.Instance.TotalTimeNsecs
                    }
                );
                if (response != null && response.Success)
                {
                    return true;
                }

                Debug.LogError("[RosConnection] ROS2 lockstep control acknowledgement failed; pausing simulation.");
                return false;
            }
            catch (Exception exception)
            {
                Debug.LogError($"[RosConnection] ROS2 lockstep Step RPC failed: {exception.Message}");
                return false;
            }
        }

        IEnumerator RosStep()
        {
            yield return new WaitForEndOfFrame();

            if (_simulationController != null)
            {
                _simulationController.Step(
                    new StepRequest
                    {
                        TotalTimeSecs = TimeHandler.Instance.TotalTimeSecs,
                        TotalTimeNsecs = TimeHandler.Instance.TotalTimeNsecs
                    }
                );
            }
        }

        private void GetSimulationController()
        {
            _simulationController = new SimulationControlClient(_streamingChannel);
            _simulationController.SetStartTime(
                new SetStartTimeRequest
                {
                    TimeSecs = TimeHandler.Instance.StartTimeSecs,
                    TimeNsecs = TimeHandler.Instance.StartTimeNsecs
                }
            );
        }

        void CreateChannelAndClients(List<ChannelOption> options)
        {
            _streamingChannel = new Channel(serverIP, serverPort, ChannelCredentials.Insecure, options);
            InitializeClients();
            _cancellationToken = _streamingChannel.ShutdownToken;
        }

        void RecreateChannel(List<ChannelOption> options)
        {
            if (_shutdownRequested)
            {
                return;
            }

            lock (_connectionLock)
            {
                try
                {
                    ShutdownChannel(_streamingChannel, 1000);
                }
                catch (Exception e)
                {
                    Debug.LogWarning($"Failed to shut down stale ROS gRPC channel: {e.Message}");
                }

                CreateChannelAndClients(options);
            }
        }

        void MarkDisconnected()
        {
            _connected = false;
            _isConnecting = false;
            _connectedNotificationPending = false;
            _simulationController = null;
        }

        public void ReportConnectionFailure(string reason)
        {
            if (_shutdownRequested || !_connected)
            {
                return;
            }

            Debug.LogWarning($"ROS gRPC connection failure reported: {reason}");
            MarkDisconnected();
            RecreateChannel(_channelOptions);
        }


        public SensorStreamingClient GetNewSensorStreamingClient()
        {
            return new SensorStreamingClient(_streamingChannel);
        }

        /// <summary>
        /// Initialize client registry
        /// </summary>
        private void InitializeClients()
        {
            _grpcClients = new Dictionary<Type, ClientBase>()
            {
                {
                    typeof(SensorStreamingClient),
                    new SensorStreamingClient(_streamingChannel)
                },
                {
                    typeof(RemoteControlClient),
                    new RemoteControlClient(_streamingChannel)
                },
                {
                    typeof(PingClient),
                    new PingClient(_streamingChannel)
                },
                {
                    typeof(TfClient),
                    new TfClient(_streamingChannel)
                },
                {
                    typeof(AcousticTransmissionClient),
                    new AcousticTransmissionClient(_streamingChannel)
                },
                {
                    typeof(VisualizationClient),
                    new VisualizationClient(_streamingChannel)
                },
                {
                    typeof(LoraTransmissionClient),
                    new LoraTransmissionClient(_streamingChannel)
                }
            };
        }

        /// <summary>
        /// Ping gRPC service to see if connection if established.
        /// </summary>
        /// <returns></returns>
        bool TryConnect()
        {
            // sleep until connected
            Debug.Log("Awaiting connection with ROS Server...");
            return PingServer(logFailure: true);
        }

        bool PingServer(bool logFailure = false)
        {
            try
            {
                var pingClinent = GetClient<PingClient>();
                var response = pingClinent.Ping(new PingMsg(), deadline: DateTime.UtcNow.AddSeconds(connectionTimeout));
                if (response.Value == 1)
                {
                    if (logFailure)
                    {
                        Debug.Log("Connected to the ROS Server");
                    }
                    return true;
                }
            }
            catch (RpcException e)
            {
                if (logFailure)
                {
                    Debug.Log($"Could not establish a connection to ROS Server. {e.Message}");
                }
            }
            catch (Exception e)
            {
                if (logFailure)
                {
                    Debug.Log($"Could not establish a connection to ROS Server. {e.Message}");
                }
            }
            return false;
        }

        void OnDisable()
        {
            _shutdownRequested = true;
            StopAllCoroutines();
            MarkDisconnected();

            var channel = _streamingChannel;
            _streamingChannel = null;
            _grpcClients = null;
            _simulationController = null;

            if (channel == null)
            {
                return;
            }

            try
            {
                // Domain reload invokes OnDisable on Unity's main thread. A
                // synchronous gRPC shutdown can wait forever for a worker that is
                // itself being torn down, leaving the Editor at Reloading Domain.
                // Starting shutdown is sufficient; the channel owns the task.
                _ = channel.ShutdownAsync();
            }
            catch (Exception e)
            {
                Debug.LogWarning($"Failed to start ROS gRPC channel shutdown: {e.Message}");
            }
        }

        static bool ShutdownChannel(Channel channel, int timeoutMs)
        {
            if (channel == null)
            {
                return true;
            }

            try
            {
                var shutdownTask = channel.ShutdownAsync();
                return shutdownTask.Wait(Mathf.Max(1, timeoutMs));
            }
            catch (Exception e)
            {
                Debug.LogWarning($"Failed to shut down ROS gRPC channel: {e.Message}");
                return false;
            }
        }
    }

}
