using System;
using System.Linq;
using Grpc.Core;
using UnityEngine;

namespace FinsSim.EditorValidation
{
    /// <summary>
    /// Verifies that the ML-Agents communicator and FinsSim ROS transport share
    /// the controlled Grpc.Core v2 assembly without requiring a ROS server.
    /// </summary>
    public static class FinsSimGrpcDependencySmoke
    {
        public static void VerifyGrpcAssemblyUnification()
        {
            var grpcAssemblies = AppDomain.CurrentDomain.GetAssemblies()
                .Where(assembly => string.Equals(assembly.GetName().Name, "Grpc.Core", StringComparison.Ordinal))
                .ToArray();

            if (grpcAssemblies.Length != 1)
            {
                throw new InvalidOperationException($"Expected exactly one loaded Grpc.Core assembly, found {grpcAssemblies.Length}.");
            }

            Version grpcVersion = grpcAssemblies[0].GetName().Version;
            if (grpcVersion == null || grpcVersion.Major != 2)
            {
                throw new InvalidOperationException($"Expected FinsSim Grpc.Core v2, found {grpcVersion}.");
            }

            // Touch ML-Agents generated protobuf/gRPC metadata. This proves its
            // communicator assembly resolves against the selected Grpc.Core.
            string mlAgentsService = Unity.MLAgents.CommunicatorObjects.UnityToExternalReflection
                .Descriptor
                .Services[0]
                .FullName;
            string finsSimService = Remotecontrol.RemoteControl.Descriptor.FullName;

            var channel = new Channel("127.0.0.1", 1, ChannelCredentials.Insecure);
            try
            {
                // Construction forces the managed transport to bind its native
                // Linux plugin, without attempting an RPC to the endpoint.
            }
            finally
            {
                channel.ShutdownAsync().GetAwaiter().GetResult();
            }

            Debug.Log(
                $"[FinsSimDependencySmoke] PASS grpc={grpcVersion}; " +
                $"mlagents={mlAgentsService}; finssim={finsSimService}");
        }
    }
}
