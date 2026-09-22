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
using System.Collections.Concurrent;
using System.Linq;
using System.Threading;
using Google.Protobuf;
using Grpc.Core;

namespace FinsSim.Networking
{
    /// <summary>
    /// Class that implements wrapper for streaming generic <T> messages from the server
    /// Instantiate it with callback action and call StartSteram method
    /// to begin streming
    /// </summary>
    /// <typeparam name="T"></typeparam>
    public class ServerStreamer<T> where T : IMessage
    {
        private Action<T> _onMessage;
        public ServerStreamer(Action<T> onMessage)
        {
            _onMessage = onMessage;
        }

        public enum MessageHandleMode
        {
            DropAndTakeLast = 1,
            Sequential,
        }

        public MessageHandleMode mode = MessageHandleMode.DropAndTakeLast;
        public bool IsStreaming { get; private set; }

        /// <summary>
        /// Set this in the Awake() method of the sensor script.
        /// Instantiate appropriate client service
        /// </summary>
        protected AsyncServerStreamingCall<T> _streamHandle;

        /// <summary>
        /// Internal buffer for storing response messages.
        /// </summary>
        ConcurrentQueue<T> _responseBuffer = new ConcurrentQueue<T>();


        Thread _handleStreamThread;
        volatile bool _stopRequested;

        /// <summary>
        /// Start streaming given stream
        ///
        /// On every message, call given callback method
        /// </summary>
        /// <param name="streamHandle"></param>
        /// <param name="onResponseMsg"></param>
        public void StartStream(AsyncServerStreamingCall<T> streamHandle)
        {
            StopStream();
            _stopRequested = false;
            _streamHandle = streamHandle;
            IsStreaming = true;
            _handleStreamThread = new Thread(_HandleResponse);
            _handleStreamThread.IsBackground = true;
            _handleStreamThread.Start();
        }

        /// <summary>
        /// Stop stream that is running
        /// </summary>
        public void StopStream()
        {
            _stopRequested = true;
            IsStreaming = false;
            var streamHandle = _streamHandle;
            _streamHandle = null;
            if (streamHandle == null)
            {
                return;
            }

            try
            {
                // Dispose cancels MoveNext without waiting for the gRPC worker. This
                // method is called from Unity's main thread during domain reload.
                streamHandle.Dispose();
            }
            catch (ObjectDisposedException)
            {
            }
            catch (InvalidOperationException)
            {
            }
        }

        /// <summary>
        /// Worker thread method to handle the response from stream
        /// </summary>
        /// <returns></returns>
        async void _HandleResponse()
        {
            if (_stopRequested)
            {
                return;
            }

            var streamHandle = _streamHandle;
            if (streamHandle == null
                || !RosConnection.TryGetInstance(out var rosConnection)
                || rosConnection == null
                || !rosConnection.IsConnected)
            {
                IsStreaming = false;
                return;
            }

            // invoke rpc call
            var stream = streamHandle.ResponseStream;

            try
            {
                while (!_stopRequested && await stream.MoveNext(rosConnection.CancellationToken))
                {
                    var current = stream.Current;
                    _responseBuffer.Enqueue(current);
                }
            }
            catch (Exception exception)
            {
                if (IsStreaming && !_stopRequested)
                {
                    try
                    {
                        rosConnection.ReportConnectionFailure($"server stream failed: {exception.Message}");
                    }
                    catch (Exception reportException)
                    {
                        UnityEngine.Debug.LogWarning(
                            $"Server stream failure could not be reported during shutdown: {reportException.Message}");
                    }
                }
            }
            finally
            {
                IsStreaming = false;
            }
        }

        /// <summary>
        /// Handle newly received messages from stream
        /// Call given callback method with received message
        /// </summary>
        /// <param name="onResponseMsg"></param>
        public void HandleNewMessages()
        {
            if (_responseBuffer.Count > 0)
            {
                if (mode == MessageHandleMode.Sequential)
                {
                    if (_responseBuffer.TryDequeue(out var result))
                    {
                        _onMessage(result);
                    }
                }
                else if (mode == MessageHandleMode.DropAndTakeLast)
                {
                    var last = _responseBuffer.LastOrDefault();
                    if (last != null)
                    {
                        _onMessage(last);
                        // clear queue, leave last element
                        while (_responseBuffer.Count > 0 && _responseBuffer.TryDequeue(out var item))
                        {
                            // do nothing
                        }
                    }
                }
            }
        }
    }
}
