# Third-party notices

This package incorporates platform source historically obtained from MARUS,
which remains licensed under Apache-2.0. Generated protobuf types preserve
their protocol namespaces.

The package owns the following controlled managed runtime dependencies under
`Runtime/Plugins`:

| Component | Role | Upstream license |
| --- | --- | --- |
| gRPC C# Core / API | Unity-to-ROS transport; selected `Grpc.Core` runtime for the project | Apache-2.0 |
| Google.Protobuf | FinsSim ROS protobuf generated types | BSD-3-Clause |
| Math.NET Numerics 5.x | FinsSim thruster algebra and OpenSourceCloth solver algebra | MIT |

`System.Diagnostics.DiagnosticSource` and the disabled legacy
`System.Runtime.CompilerServices.Unsafe` compatibility binary retain their
respective Microsoft upstream terms. Unity Collections supplies the active
Unsafe implementation. ML-Agents retains its separate
`Google.Protobuf_Packed` implementation; its old `Grpc.Core` importer is
disabled so it resolves the controlled gRPC C# Core runtime above.

DWP2 is an optional Unity Asset Store dependency and is not redistributed.
