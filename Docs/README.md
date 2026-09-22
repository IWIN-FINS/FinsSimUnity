# FinsSimUnity documentation

This directory contains the maintained, public-facing technical documentation
for the Unity simulation project. The repository-level README is the project
entry point; this page is the documentation map.

## Architecture and platform

- [Project + embedded UPM package architecture](Architecture/UPM_Architecture.zh-CN.md)
- [MARUS platform core overview](Platform/MARUS_Core_Overview.md)

## Hydrodynamics

- [Hydrodynamics backends and domain randomization](HydroDynamics/Hydrodynamics_Backends_And_Domain_Randomization.zh-CN.md)
- [DWP2 empirical-drag domain-randomization pipeline](HydroDynamics/DWP2_EmpiricalDrag_DomainRandomization_Pipeline.zh-CN.md)
- [Fossen learn-to-swim domain randomization](HydroDynamics/Fossen_LearnToSwim_DomainRandomization.zh-CN.md)
- [CFD coefficient pipeline](HydroDynamics/CFD_Hydrodynamic_Coefficient_Pipeline.zh-CN.md)
- [Yaw damping compensation](HydroDynamics/Yaw_Damping_Compensation_Pipeline.zh-CN.md)
- [Manual thruster control](HydroDynamics/FinsROV_Manual_Thruster_Control.md)
- [HDRP water guide](HydroDynamics/HDRP_Water_Guide.zh-CN.md)
- [Water-dynamics comparison](HydroDynamics/water_dynamics_comparison.html)

## Integration and guides

- [MARUS ROS integration](Integration/MARUS_ROS_Integration_Guide.md)
- [VehicleRosBridge ROS 2 contract](Integration/VehicleRosBridge_ROS2_Contract.md)
- [Simulation data collection and annotation](Guides/Simulation_Data_Collection_Annotation.md)
- [RL decision frequency](Guides/RL_Decision_Frequency.zh-CN.md)

## Governance and third-party material

- [License and provenance audit](Governance/LICENSE_AUDIT.zh-CN.md)
- [MARUS license](ThirdParty/MARUS-LICENSE)
- [MARUS upstream README](ThirdParty/MARUS-README.md)

## Operational and historical material

Machine-specific build commands, remote-operation notes, incident postmortems,
and superseded research notes live in the private `Docs/_Archive/` tree. They
are retained in private development history but excluded from public release
snapshots by `Scripts/Release/public-excludes.txt`. Current agent-facing build
and training commands are maintained in the repository root `AGENTS.md`.
