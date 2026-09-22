# FinsSimUnity

> **A Unity simulation component of [FinsSim](https://github.com/IWIN-FINS/FinsSim).**
> It's developed based on [MARUS](https://github.com/MARUSimulator/marus-example.git).
> For Python training workflows, ROS 2 integration, experiment configurations,
> hardware bridges, and the documentation map, start from the
> [FinsSim](https://github.com/IWIN-FINS/FinsSim)  workspace.

FinsSimUnity provides Unity-based underwater simulation and learning
environments for FinsROV. The reusable platform is the embedded UPM package
[`com.iwin-fins.fins-sim`](Packages/com.iwin-fins.fins-sim/); task-specific
scenes and training environments live under `Assets/FinsSimUnity/Tasks/`.

## What is here

- `Packages/com.iwin-fins.fins-sim/` — reusable Core and Platform code:
  sensors, actuators, ROS/gRPC communication, environments, visualization, and
  hydrodynamics.
- `OptionalPackages/com.iwin-fins.fins-sim.dwp2/` — an opt-in DWP2 adapter;
  it is outside Unity's automatic package scan and is never enabled by a
  public release manifest.
- `Assets/FinsSimUnity/Tasks/` — task-specific scenes, agents, rewards, and
  Editor preparation tools.
- `Docs/` — architecture, hydrodynamics, build, and training documentation.
- `artifacts/unity_builds/` — local build outputs; do not commit generated
  players.

## Requirements

- **Recommended Unity version:** Unity `6000.3.20f1`
  (revision `c9ba695d4f07`), as recorded in
  [`ProjectSettings/ProjectVersion.txt`](ProjectSettings/ProjectVersion.txt).
  This is the version currently used to author and verify the project, and is
  recommended for reproducible imports and builds. Other Unity versions may
  work but have not necessarily been tested by this project.
- A local checkout of [FinsSim](https://github.com/IWIN-FINS/FinsSim) when
  building training environments or running Python/ROS 2 workflows.
- An available X display for Linux Editor builds. Do not pass `-nographics` to
  the Unity Editor build command.

## Water-system dependencies

FinsSim provides maintained, open-source hydrodynamics and water-provider
backends, including the Fossen-based path used by the released training scene.
They do not require Dynamic Water Physics 2 (DWP2) or Crest HDRP.

The optional **DWP2 + Crest Water HDRP** workflow requires separately purchased
commercial assets. DWP2 and Crest Water HDRP are Unity Asset Store products;
they are excluded from public release snapshots and are not covered by the
FinsSim license. See [DWP2](https://assetstore.unity.com/packages/tools/physics/dynamic-water-physics-2-147990)
and [Crest Water 4 HDRP](https://assetstore.unity.com/packages/tools/particles-effects/crest-water-4-hdrp-ocean-rivers-lakes-164158).

### Enable the optional DWP2 adapter

The FinsSim-owned adapter source is retained at
[`OptionalPackages/com.iwin-fins.fins-sim.dwp2/`](OptionalPackages/com.iwin-fins.fins-sim.dwp2/),
but Unity does not load folders outside `Packages/` unless a manifest explicitly
references them. It contains no DWP2 source, DLLs, or Asset Store content.

1. Obtain and install compatible, user-licensed DWP2/NWH packages first. Some
   water-rendering workflows additionally require a separately licensed Crest
   Water HDRP installation.
2. In Unity, use **Window > Package Management > Package Manager > + > Add
   package from disk**, then select the optional package's `package.json`; or
   add the following local dependency to `Packages/manifest.json`:

   ```json
   "com.iwin-fins.fins-sim.dwp2": "file:../OptionalPackages/com.iwin-fins.fins-sim.dwp2"
   ```

3. Reopen the project after Package Manager resolves the dependency. Remove the
   package through Package Manager when returning to a non-DWP2 workflow.

Private `release-candidate` checkouts carry this local dependency by default.
Public release snapshots deliberately remove it, so a public checkout imports
without DWP2. Do not move this adapter under `Packages/` unless its commercial
dependencies are already installed: Unity treats every package in that folder
as an embedded package and compiles it automatically.

`Assets/Crest/` is different: it contains the MIT-licensed upstream
[Crest Water](https://github.com/wave-harmonic/crest) Built-in Render Pipeline
implementation, retained outside the FinsSim UPM package as source/reference
material. It is not compatible with HDRP. For a Crest-based HDRP project,
purchase and install Crest Water HDRP; do not treat the bundled open-source
Crest directory as an HDRP substitute. Its local note and license are at
[`Assets/Crest/README.md`](Assets/Crest/README.md) and
[`Assets/Crest/Crest/LICENSE`](Assets/Crest/Crest/LICENSE).

## Build a Linux training server

Training must use a Linux Server / Dedicated Server build. A normal Linux
Player launched with `-nographics` is not a supported replacement.

From the project root, build the Rot6D16 pose-control server:

```bash
env DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority XDG_RUNTIME_DIR=/run/user/1000 \
/path/to/Unity/Editor/Unity \
  -batchmode -quit \
  -projectPath "$PWD" \
  -executeMethod FinsSimTaskBuild.BuildFromCommandLine \
  -finsSimTask pose-control/new-server \
  -logFile /tmp/finssim-unity-build.log
```

The task build registry selects the correct scene preparation and output path.
For the task catalog and current build commands, see
[Unity binary build commands](Docs/RL/Unity_Binary_Build_Commands.zh-CN.md).

## Train from the FinsSim workspace

Set the resulting server executable as `unity.env_path` in the selected FinsSim
YAML configuration, then run its trainer from the FinsSim workspace:

```bash
cd /path/to/FinsSim
uv run --package finssim-cli finssim rl train -c <config>.yaml
```

ML-Agents training communication remains enabled through the project’s managed
gRPC dependency boundary. Use the Unity server build for a one-environment
smoke test before starting a long run.

## Documentation

- [Documentation index](Docs/README.md) — maintained public documentation and
  the boundary with private archived material.
- [UPM architecture](Docs/Architecture/UPM_Architecture.zh-CN.md)
- [Linux Server build runbook](Docs/RL/RL_Linux_Server_Build_Runbook.zh-CN.md)
- [Hydrodynamics backends and domain randomization](Docs/HydroDynamics/Hydrodynamics_Backends_And_Domain_Randomization.zh-CN.md)

## License

FinsSim-owned portions are licensed under [Apache-2.0](LICENSE), with separate
commercial terms available as described in [COMMERCIAL-LICENSE.md](COMMERCIAL-LICENSE.md).
Third-party and Unity Asset Store components retain their own terms; see
[THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md).

## Citation

```bibtex
@article{zhang2026finssim,
  title   = {FinsSim: A Reality-Aligned Integrated Simulation Platform for Underwater Robot Learning},
  author  = {Zhang, Yu and Song, Yuanmingqing and Rao, Xiangyun and Fong, Pangkit and Zhang, Kunhao and Fang, Chongrong and He, Jianping},
  journal = {arXiv preprint arXiv:2609.23943},
  year    = {2026},
  doi     = {10.48550/arXiv.2609.23943},
  url     = {https://arxiv.org/abs/2609.23943}
}
```
