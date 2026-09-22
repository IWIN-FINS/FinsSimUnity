# marus-example Agent Notes

## Project Overview

- Unity project path: `~/UnderwaterSim/Code/FinsSimUnity`
- Related FinsSim workspace: `~/UnderwaterSim/Code/FinsSim`
- Unity editor used for the current Linux builds: `~/Unity/Hub/Editor/6000.3.20f1/Editor/Unity`
- RL training scene: `Assets/FinsSimUnity/Tasks/PoseControl/Scenes/ControlForPosition_new.unity`
- Smooth-near-target manual RL scene: `Assets/FinsSimUnity/Tasks/PoseControl/Scenes/ControlForPosition_smooth_near_target_manual.unity`
- Maintained public documentation index: `Docs/README.md`
- Historical operational runbooks and postmortems: `Docs/_Archive/` (private development reference only)
- Main training Agent components available on `FinsROV`:
  - `ControlForPosition_IncrementalReward`
  - `ControlForPosition_SmoothNearTargetReward`
  - `ControlForPosition`
- ML-Agents behavior name: `ControlForPosition`
- Expected observation/action spaces for the current pose-control setup:
  - Vector observation size: `16`
  - Continuous actions: `8`
  - Observation layout:
    - local target offset: 3
    - relative target rotation 6D: 6
    - local linear velocity: 3
    - local angular velocity: 3
    - normalized target distance: 1

## Unity Editor Play Mode Control

The Unity Editor can be controlled from the terminal through the project-local
command bridge:

```bash
cd /home/fins/UnderwaterSim/Code/UnityProject/marus-example
./tools/unity/unity-play stop
./tools/unity/unity-play start
./tools/unity/unity-play status
```

`Assets/FinsSimUnity/Editor/Automation/ExternalPlayController.cs` listens for commands written to
`/tmp/unity_play_command` and writes Editor state to
`/tmp/unity_play_status.json`. Prefer this bridge over GUI automation. Do not
launch another Unity Editor process just to enter or exit Play Mode.

Before modifying C# scripts, stop Play Mode. After modifying scripts, wait for
Unity to finish compiling and check that there are no compile errors before
starting Play Mode again.

`/home/fins/.local/bin/unity-play` points at the same project-local script and
can be used as a shorthand because `/home/fins/.local/bin` is on PATH. If
`/usr/local/bin/unity-play` is also present, it should point at the same script.

## ROS gRPC Adapter Notes

When using Unity Editor Play mode with ROS2, start the adapter with the `/sim`
prefix if the real vehicle ROS stack is also present:

```bash
cd /home/fins/UnderwaterSim/Code/FinsSim/ros2_ws
./scripts/launch_grpc_ros_adapter.sh topic_prefix:=/sim
```

The prefix is applied inside the ROS adapter only. Unity scene/prefab topic
fields should normally remain `/finsrov/...`; ROS then sees them as
`/sim/finsrov/...`.

Unity `RosConnection` and `VehicleRosBridge` need a few seconds to connect or
reconnect after the adapter starts. Initial `DeadlineExceeded` messages in the
Unity Editor log are normal while the adapter is still absent or starting. Do
not begin ROS-side tests until these checks pass:

```bash
cd /home/fins/UnderwaterSim/Code/FinsSim/ros2_ws
./scripts/run_ros2_uv.sh ros2 topic list | sort | grep /sim/finsrov
./scripts/run_ros2_uv.sh ros2 topic info /sim/finsrov/thrusters_out -v
./scripts/run_ros2_uv.sh ros2 topic hz /sim/finsrov/controller/imu --window 20
```

Expected state:

- `/sim/finsrov/thrusters_out` has the adapter as subscriber.
- `/sim/finsrov/controller/imu`, `/sim/finsrov/controller/dvl`, `/sim/finsrov/controller/depth`, and `/sim/finsrov/controller/pose` have the adapter as publisher.
- Unity Editor log contains `Connected to the ROS Server` and
  `[VehicleRosBridge] Listening for thrusters on /finsrov/thrusters_out.`

For hydrodynamic identification against `VehicleRosBridge` controller topics,
use Unity controller IMU/DVL as Unity-local body-frame data. Do not reuse the
real-vehicle `[x,z,y]` remap. Pass identity mapping:

```bash
-p ros_to_body_basis_indices:="[0,1,2]" -p ros_to_body_basis_signs:="[1.0,1.0,1.0]"
```

If the mapping is wrong, a yaw test can appear in `nu_pitch_z_radps` instead of
`nu_yaw_radps`, which makes the yaw response look artificially zero.

`FinsROVDwp2YawRuntimeProbe` is diagnostic only. It confirms at runtime that the
DWP2 WaterObject yaw patch is active and logs raw/scaled/target/applied yaw
hydrodynamic torque plus thruster yaw torque. It is not required for ROS data
collection and does not replace IMU/DVL sampling.

For Unity yaw curve comparison against real `actual_tau`, do not use the
identifier CSV `tau_fit_my_yaw_nm` unless Unity publishes
`/sim/finsrov/hardware/motor_rpm_raw`. Without that feedback, the identifier
falls back to commanded/target torque. Use
`FinsROVDwp2YawRuntimeProbe.thruster_applied_yaw_tau_nm` for Unity applied yaw
torque; this matches the real summary's RPM-derived `actual_tau` definition.

`FinsROVManualThrusterController` should stay passive while idle. It may remain
enabled for manual testing, but it must not write zero thruster commands every
frame with no key input, otherwise it overwrites ROS/RL thruster commands and
yaw sweeps appear to have no response.

## Build Outputs

Training should use the Linux Server/Dedicated Server build:

```text
/home/fins/UnderwaterSim/Code/FinsSimUnity/artifacts/unity_builds/rl/linux/ControlForPosition_Rot6D16_10Hz_Server/ControlForPosition.x86_64
```

Smooth-near-target manual training should use:

```text
/home/fins/UnderwaterSim/Code/FinsSimUnity/artifacts/unity_builds/rl/linux/ControlForPosition_SmoothNearTarget_Manual_10Hz_Server/ControlForPosition.x86_64
```

Graphics/debug builds use the normal Linux player:

```text
/home/fins/UnderwaterSim/Code/FinsSimUnity/artifacts/unity_builds/rl/linux/ControlForPosition_Rot6D16_10Hz/ControlForPosition.x86_64
```

Do not use the normal Linux player with `-nographics` for RL training. In this project/version combination it reaches `NullGfxDevice` and then SIGSEGVs shortly after ML-Agents registers the communicator. The server build uses `NullGfxDevice` without the same crash and has been verified to enter PPO training.

## Unity Build Commands

Preferred command-line build path is the C# build script below. It runs the required scene preparation before building.

### Command Registry Maintenance

The `Portable task-build commands` section in this file is authoritative for build invocation. Whenever a task build is added or changed, update its task id, scene, output executable and command here; do not use archived scratchpads as a command registry.

Before copying commands, confirm the real X display:

```bash
printf 'DISPLAY=%s\nXAUTHORITY=%s\nXDG_RUNTIME_DIR=%s\n' "$DISPLAY" "$XAUTHORITY" "$XDG_RUNTIME_DIR"
ls -la /tmp/.X11-unix
who
```

On the current machine, the verified build environment is:

```text
DISPLAY=:0
XAUTHORITY=/run/user/1000/gdm/Xauthority
XDG_RUNTIME_DIR=/run/user/1000
```

Do not pass `-nographics` to the Unity Editor build command. `-nographics` is for the built server executable during training/smoke only.

Preferred build path: use the worktree build daemon. `build_in_worktree.sh` defaults to the daemon, starts it on demand, syncs the main project into the build worktree, sends a build request, and waits for the result. The first build still pays Unity Editor startup cost; subsequent builds can reuse the same background Editor.

```bash
cd /home/fins/UnderwaterSim/Code/UnityProject/marus-example
./tools/unity/build_in_worktree.sh \
  --execute-method FinsSimLinuxBuild.BuildOneChaseOneVisual \
  --log-file /tmp/finsim-1chase1-visual-build.log
```

Build progress/logging:

- By default, daemon builds print a compact wait heartbeat while Unity is still building.
- Add `--verbose` to stream newly written Unity log lines while waiting:

```bash
./tools/unity/build_in_worktree.sh \
  --execute-method FinsSimLinuxBuild.BuildOneChaseOneVisual \
  --log-file /tmp/finsim-1chase1-visual-build.log \
  --verbose
```

Default daemon build chain:

```text
build_in_worktree.sh
  -> build_daemon_request.sh
     -> create/verify /home/fins/UnderwaterSim/Code/UnityProject/marus-example-build
     -> rsync main project source into the build worktree
     -> start build_daemon_start.sh if no build daemon is running
        -> Unity Editor opens the build worktree in batchmode
        -> FinsSimBuildDaemon.Start waits until EditorApplication is idle
        -> writes /tmp/finsim-unity-build-daemon/ready
     -> write /tmp/finsim-unity-build-daemon/request.json
     -> FinsSimBuildDaemon polls request.json
     -> invokes the requested static build method by reflection
     -> writes result.json when the method returns
```

Important daemon caveat: `BuildPipeline.BuildPlayer` can write a successful player but not return cleanly to the Editor C# caller in the persistent batchmode daemon. `build_daemon_request.sh` therefore also watches the daemon log. If it sees:

```text
Build Finished, Result: Success.
```

it first waits 30 seconds for the daemon to write its JSON result. If no result arrives during that grace period, it treats the build as successful, stops the daemon, and lets the next build restart it cleanly. This avoids racing a healthy `BuildPipeline.BuildPlayer` return while still preventing request commands from hanging forever after Unity has already produced the build output.

Daemon controls:

```bash
./tools/unity/build_daemon_start.sh
./tools/unity/build_daemon_request.sh FinsSimLinuxBuild.BuildOneChaseOneVisual
./tools/unity/build_daemon_stop.sh
```

Useful direct daemon checks:

```bash
./tools/unity/build_daemon_start.sh --log-file /tmp/finsim-unity-build-daemon.log
./tools/unity/build_daemon_request.sh FinsSimBuildDaemon.Ping --skip-sync
./tools/unity/build_daemon_stop.sh
```

Cold-start fallback, for CI-like clean builds or when the daemon is suspected stale:

```bash
./tools/unity/build_in_worktree.sh \
  --cold-start \
  --execute-method FinsSimLinuxBuild.BuildOneChaseOneVisual \
  --log-file /tmp/finsim-1chase1-visual-build.log
```

The server build method is the one to use for RL training. It builds with `StandaloneBuildSubtarget.Server` and outputs to the `_Server` directory listed above. Do not use a normal Linux Player plus `-nographics` as the training build.

1Chase1 build methods:

```text
FinsSimLinuxBuild.BuildOneChaseOneVisual
  -> normal Linux Player
  -> output: /home/fins/UnderwaterSim/Code/FinsSimUnity/artifacts/unity_builds/rl/linux/FinsROV/1Chase1_visual/1Chase1.x86_64
  -> used for visible eval
  -> visual scene preparation keeps Camera, Light, Renderer, and Sky/Fog enabled

FinsSimLinuxBuild.BuildOneChaseOneServer
  -> Linux Server/Dedicated Server
  -> output: /home/fins/UnderwaterSim/Code/FinsSimUnity/artifacts/unity_builds/rl/linux/FinsROV/1Chase1_headless/1Chase1.x86_64
  -> used for training/headless smoke
  -> training scene preparation can disable graphics-only components
```

If Unity reports that another instance has the project open, check for existing editor processes first:

```bash
ps -eo pid,ppid,stat,etime,cmd | grep -F '/home/fins/Unity' | grep -v grep
```

Only terminate a stale Unity editor process after confirming it is not actively being used.

## Unity Editor Build Profile

Unity Editor can also create a Linux Server/Dedicated Server build manually:

1. Open the project:

   ```text
   /home/fins/UnderwaterSim/Code/UnityProject/marus-example
   ```
2. Open `File -> Build Profiles`.
3. Select or create a Linux build profile.
4. Set platform/target:

   ```text
   Target Platform: Linux
   Architecture: x86_64
   Subtarget / Build Type: Server or Dedicated Server
   ```
5. Include only the RL training scene:

   ```text
   Assets/FinsSimUnity/Tasks/PoseControl/Scenes/ControlForPosition_new.unity
   ```
6. Build to:

   ```text
   /home/fins/UnderwaterSim/Code/FinsSimUnity/artifacts/unity_builds/rl/linux/ControlForPosition_Rot6D16_10Hz_Server/
   ```

Important: the manual Editor build profile only chooses the build target. It does not automatically run `FinsSimScenePreparation.PrepareControlForPositionFlatWater()` unless you invoke the C# build method. For this project, prefer `FinsSimLinuxBuild.BuildControlForPositionNewServer` for training builds because it also:

- Disables `Ocean`.
- Creates/configures `FlatWaterProvider` at water height `0`.
- Removes training-irrelevant camera/HDRP/ROS/debug components.
- Keeps DWP2/NWH physics components such as `WaterObject`, `MassFromVolume`, `VariableCenterOfMass`, `Thruster`, and `ThrusterController`.

If you manually use Build Profiles, make sure the scene has already been prepared or you may reintroduce timeout, SIGSEGV, or graphics/GPU-memory issues.

## Scene Preparation

The build script calls `FinsSimScenePreparation.PrepareControlForPositionFlatWater()` before building.

Smooth-near-target manual builds call `FinsSimScenePreparation.PrepareControlForPositionSmoothNearTargetManualFlatWater()` before building. This is required; direct Build Profile builds can skip scene preparation and reintroduce headless timeout/SIGSEGV issues.

Important behavior:

- `Ocean` is disabled for the training scene.
- `FlatWaterProvider` is created/enabled under `Environment`.
- `PhysicalWaveWaterDataProvider` is set to `FlatFallback` with water height `0`.
- DWP2/NWH physics initialization is preserved:
  - Do not skip `WaterObject` initialization.
  - Do not skip `MassFromVolume` / `VariableCenterOfMass` initialization.
  - Do not disable `WaterObject`, `Rigidbody`, `Collider`, `Thruster`, `ThrusterController`, `ControlForPosition_IncrementalReward`, or `ControlForPosition`.
- Training-only visual/network/debug components are disabled or removed from the RL build scene:
  - Cameras, audio listeners, lights, renderers.
  - HDRP `Volume`, `HDAdditionalCameraData`, `HDAdditionalLightData`.
  - DWP2 `UnderwaterFog` and `WaterParticleSystem`.
  - ROS bridge and ROS sensor streamer components.
  - Manual/debug controls such as `FinsROVManualThrusterController`, `DWP2DebugController`, `VelocityDebugDisplay`, `DragObject`.
- Reward variants should be switched deliberately. For smooth-near-target manual training, use `ControlForPosition_SmoothNearTargetReward`; avoid leaving multiple enabled Agent components competing on the same `FinsROV`.
- ML-Agents child sensors are disabled on BehaviorParameters for training, so the current RL observation is the explicit 16-float vector from `CollectObservations`.

## Recommended Training Config

For headless training, configs should point to the server build and keep graphics off:

```yaml
unity:
  env_path: ../../artifacts/unity_builds/rl/linux/ControlForPosition_Rot6D16_10Hz_Server/ControlForPosition.x86_64
  no_graphics: true
  timeout_wait: 180

trainer:
  overrides:
    device: cuda
```

Current configs using this:

- `/home/fins/UnderwaterSim/Code/FinsSim/configs/rl/ppo_control_for_pose_rot6d16.yaml`
- `/home/fins/UnderwaterSim/Code/FinsSim/configs/rl/ppo_control_for_pose_smooth_near_target.yaml`

Smooth-near-target manual config must point to:

```yaml
unity:
  env_path: ../../artifacts/unity_builds/rl/linux/ControlForPosition_SmoothNearTarget_Manual_10Hz_Server/ControlForPosition.x86_64
  no_graphics: true
  timeout_wait: 180
```

Do not wrap the server build training command with `xvfb-run`; it is unnecessary.

Local test command:

```bash
cd /home/fins/UnderwaterSim/Code/FinsSim
unset DISPLAY HTTP_PROXY HTTPS_PROXY ALL_PROXY http_proxy https_proxy all_proxy
export NO_PROXY=localhost,127.0.0.1,::1
export no_proxy=localhost,127.0.0.1,::1
export CUDA_VISIBLE_DEVICES=1
uv run --package finssim-cli finssim rl train -c configs/rl/ppo_control_for_pose_rot6d16.yaml
```

Smooth-near-target manual command:

```bash
cd /home/fins/UnderwaterSim/Code/FinsSim
unset DISPLAY HTTP_PROXY HTTPS_PROXY ALL_PROXY http_proxy https_proxy all_proxy
export NO_PROXY=localhost,127.0.0.1,::1
export no_proxy=localhost,127.0.0.1,::1
export CUDA_VISIBLE_DEVICES=1
uv run --package finssim-cli finssim rl train -c configs/rl/ppo_control_for_pose_smooth_near_target.yaml
```

For smoke/debug commands and expected output, use the `Portable task-build commands` section below.

## Known Build/Smoke Pitfalls

- `ConnectionResetError: [Errno 104] Connection reset by peer` is a Python-side symptom. The Unity worker crashed, timed out, loaded a broken scene, or failed before returning spaces. Check the Player/build log and run the 1-env smoke in the runbook before changing PPO code.
- `Assimp does not support platform: LinuxServer` is non-blocking when the scene does not dynamically import URDF/mesh at runtime. It appeared in successful 1-env and 4-env smooth-near-target smoke tests.
- `Couldn't connect to trainer... Will perform inference instead` is expected when a Unity executable is launched manually without a Python trainer listening on `--mlagents-port`.
- `Unknown side channel data received ... a1d8f7b7...` means Python did not register `StatsSideChannel`; this causes log storms. FinsSim Python currently registers it in `finssim_rl.models.make_unity_env()`.
- Do not remove marus-core gRPC/Protobuf DLLs to quiet warnings. ML-Agents gRPC is training-critical; MARUS/ROS gRPC scene components can be disabled for RL builds, but the underlying assemblies may still be required for compile/runtime.
- Do not hand-edit large `.unity` YAML blocks unless unavoidable. The 2026-07-26 manual scene issue was caused by damaged `PrefabInstance` YAML around modification/removal data and only surfaced as Linux Server NullGfx crash/reset failure.

## Remote 4090 Server Notes

Remote server:

```text
zy@1.tcp.vip.cpolar.top -p 12514
```

Remote FinsSim workspace:

```text
/home/zy/code/FinsSim-private
```

Sync the server build:

```bash
rsync -Paz -e 'ssh -p 12514' \
/home/fins/UnderwaterSim/Code/FinsSimUnity/artifacts/unity_builds/rl/linux/ControlForPosition_Rot6D16_10Hz_Server/ \
zy@1.tcp.vip.cpolar.top:/home/zy/code/FinsSim-private/artifacts/unity_builds/rl/linux/ControlForPosition_Rot6D16_10Hz_Server/
```

Sync RL configs:

```bash
rsync -Paz -e 'ssh -p 12514' \
/home/fins/UnderwaterSim/Code/FinsSim/configs/rl/ppo_control_for_pose_rot6d16.yaml \
zy@1.tcp.vip.cpolar.top:/home/zy/code/FinsSim-private/configs/rl/
```

Remote test command:

```bash
cd /home/zy/code/FinsSim-private
export PATH=/home/zy/.local/bin:$PATH
source .venv/bin/activate
unset DISPLAY HTTP_PROXY HTTPS_PROXY ALL_PROXY http_proxy https_proxy all_proxy
export NO_PROXY=localhost,127.0.0.1,::1
export no_proxy=localhost,127.0.0.1,::1
export CUDA_VISIBLE_DEVICES=0
finssim rl train -c configs/rl/ppo_control_for_pose_rot6d16.yaml
```

If `uv` is not found over non-interactive SSH, make sure `/home/zy/.local/bin` is in `PATH`.

If ML-Agents reports worker `0` or port `5005` is in use, check for stale processes:

```bash
ss -ltnp | grep ':5005'
pgrep -af 'ControlForPosition|finssim|mlagents|train.py'
```

Do not kill unrelated training jobs. During the latest debugging session, an unrelated remote process existed:

```text
python train.py --config-json sim_online_per_bs_big_resume_latest.json
```

## Verified Behavior

Observed behavior during debugging:

- Normal Linux player with graphics at 320x240 still used roughly hundreds of MiB VRAM per Unity environment.
- Normal Linux player with `-nographics` crashed with SIGSEGV after `Registered Communicator in Agent`.
- Linux Server build with `no_graphics: true` reachecank
  - `Registered Communicator in Agent`
  - `Observation Space: Box(-inf, inf, (16,), float32)`
  - `Action Space: Box(-1.0, 1.0, (8,), float32)`
  - `Using cuda device`
  - `Starting training`
- The server build was verified locally and on the remote 4090 server through at least one PPO iteration, `512` timesteps.
- In server/headless mode, Unity training environments did not appear as GPU memory consumers in `nvidia-smi`; remaining observed GPU memory was from desktop remote services and an unrelated Python job.

## Release branches and remotes

- `origin-private` is the private development remote. Push its `main`, the
  single moving `release-candidate`, and internal `release/<version>` branches here.
- `origin` is the public release remote. Its `main` always represents the
  current history-free release snapshot; never push private `main` or a
  candidate branch there.
- Private `main` retains full development history. Use one moving
  `release-candidate` branch to prepare publication exclusions.
- `Scripts/Release/release_public.sh` creates `release/<version>` as an orphan
  root commit from the committed candidate tree, updates `origin/main` using
  `--force-with-lease`, and publishes an immutable annotated `v<version>` tag.
- The entire `Scripts/Release/` directory is excluded from public snapshots;
  the private publisher invokes its manifest-cleanup hook before writing the
  public tree.
- Do not merge a release back into private main or rewrite a published tag in normal operation. Only an approved, low-risk corrective reissue may use `--amend-release` to rebuild the same version, update `origin/main`, and rewrite its tag; record the reason in the commit/release notes. New features or behavior changes require a new version.
- Mark candidate revisions with immutable annotated tags such as
  `v0.1.0-rc.1`, rather than encoding the version in the branch name.

```bash
git push origin-private main
git push origin-private release-candidate
Scripts/Release/release_public.sh plan --version 0.1.0
Scripts/Release/release_public.sh publish --version 0.1.0
```

## Portable task-build commands

This section is the portable replacement for the old engineering command blocks
that no longer live in the top-level README. Run these commands from the Unity
project root unless noted otherwise. Do not substitute host-specific absolute
paths: resolve the two adjacent workspaces and the Unity Editor once per shell.

```bash
export FINSSIM_UNITY_ROOT="$(git rev-parse --show-toplevel)"
export FINSSIM_ROOT="${FINSSIM_ROOT:-$(cd "$FINSSIM_UNITY_ROOT/../FinsSim" && pwd)}"
export UNITY_EDITOR="${UNITY_EDITOR:-$HOME/Unity/Hub/Editor/6000.3.20f1/Editor/Unity}"
```

Before building, check for an Editor/trainer already using the project and
verify the display for the current login:

```bash
pgrep -af 'Editor/Unity|UnityShaderCompiler|ControlForPosition|finssim-rl|mlagents|train.py'
printf 'DISPLAY=%s\nXAUTHORITY=%s\nXDG_RUNTIME_DIR=%s\n' "$DISPLAY" "$XAUTHORITY" "$XDG_RUNTIME_DIR"
ls -la /tmp/.X11-unix
who
```

Use Linux Server / Dedicated Server for RL. Never pass `-nographics` to the
Unity **Editor** build command. The task registry runs the required scene
preparation.

```bash
# Smooth-near-target manual server build
env DISPLAY="$DISPLAY" XAUTHORITY="$XAUTHORITY" XDG_RUNTIME_DIR="$XDG_RUNTIME_DIR" \
  "$UNITY_EDITOR" -batchmode -quit -projectPath "$FINSSIM_UNITY_ROOT" \
  -executeMethod FinsSimTaskBuild.BuildFromCommandLine \
  -finsSimTask pose-control/smooth-near-target-manual-server \
  -logFile /tmp/finsim-smooth-manual-server-build.log

# Rot6D16 baseline server build
env DISPLAY="$DISPLAY" XAUTHORITY="$XAUTHORITY" XDG_RUNTIME_DIR="$XDG_RUNTIME_DIR" \
  "$UNITY_EDITOR" -batchmode -quit -projectPath "$FINSSIM_UNITY_ROOT" \
  -executeMethod FinsSimTaskBuild.BuildFromCommandLine \
  -finsSimTask pose-control/new-server \
  -logFile /tmp/finsim-rot6d16-server-build.log

# Normal Linux Player: graphical debug only, never headless RL training.
env DISPLAY="$DISPLAY" XAUTHORITY="$XAUTHORITY" XDG_RUNTIME_DIR="$XDG_RUNTIME_DIR" \
  "$UNITY_EDITOR" -batchmode -quit -projectPath "$FINSSIM_UNITY_ROOT" \
  -executeMethod FinsSimTaskBuild.BuildFromCommandLine \
  -finsSimTask pose-control/new-player \
  -logFile /tmp/finsim-rot6d16-player-build.log
```

Verify `Build Finished, Result: Success.` and find the executable before
changing `unity.env_path` in a trainer configuration:

```bash
tail -n 120 /tmp/finsim-smooth-manual-server-build.log
find "$FINSSIM_UNITY_ROOT/artifacts/unity_builds" -type f -name '*.x86_64' -printf '%p\n'
```

One-environment smoke test (set `ENV_PATH` to the server executable emitted by
the chosen task):

```bash
export ENV_PATH="$FINSSIM_UNITY_ROOT/artifacts/unity_builds/pose-control/smooth-near-target-manual-server/FinsSimUnity.x86_64"
cd "$FINSSIM_ROOT/python/finssim_rl"
env -u DISPLAY timeout 360s uv run python -c "from mlagents_envs.environment import UnityEnvironment; env=UnityEnvironment(file_name='$ENV_PATH', no_graphics=True, worker_id=0, base_port=11205, timeout_wait=240, additional_args=['-logFile','/tmp/finssim_smooth_manual_smoke.log']); print('INITIALIZED'); env.reset(); print('RESET_OK'); env.close(); print('CLOSED')"
```

Expected output includes `INITIALIZED`, `RESET_OK`, and `CLOSED`. For the
parallel four-environment smoke recipe and the scene-specific output catalog,
use the `Portable task-build commands` section in this file.

Launch training from the adjacent FinsSim workspace; the selected YAML must
point `unity.env_path` at a server executable:

```bash
cd "$FINSSIM_ROOT"
unset DISPLAY HTTP_PROXY HTTPS_PROXY ALL_PROXY http_proxy https_proxy all_proxy
export NO_PROXY=localhost,127.0.0.1,::1
export no_proxy=localhost,127.0.0.1,::1
uv run --package finssim-cli finssim rl train \
  -c configs/rl/ppo_control_for_pose_smooth_near_target.yaml
```
