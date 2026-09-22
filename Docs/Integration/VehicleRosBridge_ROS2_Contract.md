# VehicleRosBridge ROS2 Contract

`VehicleRosBridge` connects Unity vehicles to ROS2 through the existing gRPC adapter. Unity remains a gRPC client; `grpc_ros_adapter` subscribes to ROS2 topics and streams matching messages to Unity.

## Thruster Topic

Default topic:

```text
/{bridge_name}/thrusters_out
```

For `FinsROV`, the default is:

```text
/finsrov/thrusters_out
```

Message type:

```text
std_msgs/Float32MultiArray
```

Contract:

- `data` must contain exactly 8 finite floats.
- Default semantic mode is `force_n`.
- In `force_n`, each value is a force request in newtons.
- The semantic mode is not encoded in the message body. It must be configured consistently in Unity, `finssim_motion_control`, and `finsrov_hardware_bridge`.

Canonical order:

```text
[V_LF, V_LB, V_RB, V_RF, H_LF, H_LB, H_RB, H_RF]
```

Unity also accepts the current FinsROV names in the same slots:

```text
[Vertical1, Vertical2, Vertical3, Vertical4, Horizontal1, Horizontal2, Horizontal3, Horizontal4]
```

## Unity Behavior

`VehicleRosBridge.thrusterCommandMode` defaults to `ForceN`.

When a Marus `ThrusterController` is present, Unity consumes `force_n` by calling `Thruster.ApplyForceRequest(valueN)` for each thruster. This directly applies the requested simulation force and does not run the real-vehicle RPM/thruster curve conversion.

`NormalizedDirect` remains available for old normalized `[-1, 1]` workflows. DWP2 `AdvancedShipController` fallback only supports `NormalizedDirect`; for accurate `force_n` simulation, use Marus `ThrusterController` or add an equivalent force-request adapter.

## ROS2 Behavior

`finssim_motion_control` should publish force requests with:

```yaml
thruster_topic: /finsrov/thrusters_out
thruster_output_mode: force_n
```

`pwm_topic` is still accepted as a compatibility alias, but `thruster_topic` is the preferred name.

For real hardware, `finsrov_hardware_bridge` should consume the same topic with:

```yaml
input_thruster_topic: /finsrov/thrusters_out
command_mode: force_n
```

The hardware bridge converts `force_n -> target RPM -> normalized MCU command`, then applies `motor_order` and `motor_signs`.

## Navigation Status Gate

The controller keeps `require_fusion_ready: true` in both simulation and real-vehicle runs. The status contract remains:

```text
/finsrov/state/status
```

Message type:

```text
std_msgs/String
```

The string payload is JSON and must include at least:

```json
{"ready":true,"initialized":true,"vision_mode":"fresh","reject_reason":""}
```

For Unity `sim_truth` runs, do not extend the gRPC adapter for this string topic. Start ROS2 with:

```bash
ros2 launch finssim_motion_control motion_controller.launch.py state_input_mode:=sim_truth
```

That launch mode starts `sim_truth_state_status`, which watches the Unity-published pose/IMU/depth/DVL topics and publishes `/finsrov/state/status`. If any required Unity stream is missing or stale, the shim publishes `ready=false` and the controller continues to output zero thrust.

For future `raw_fusion` simulation, disable the shim and run the real `finsrov_state_estimation` fusion node instead. Unity should then provide raw perception/sensor inputs, and `state_fusion_node` remains the only publisher of `/finsrov/state/status`.
