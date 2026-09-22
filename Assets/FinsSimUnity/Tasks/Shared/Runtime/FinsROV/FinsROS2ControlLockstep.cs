using FinsSim.Networking;
using UnityEngine;

/// <summary>
/// Evaluation-only ROS2 lockstep driver.
///
/// The component is installed by ExternalPlayController before Play mode.  It
/// executes after the vehicle bridge has sampled and queued the current state,
/// asks RosConnection to publish the matching /clock value, and blocks until
/// the ROS2 adapter reports that the controller has completed that tick.  A
/// controller that runs more slowly than the Unity physics rate acknowledges
/// intermediate ticks while holding its last command, preserving its original
/// zero-order-hold action rate.
/// </summary>
[DefaultExecutionOrder(1000)]
[DisallowMultipleComponent]
public sealed class FinsROS2ControlLockstep : MonoBehaviour
{
    [SerializeField] private float simulationTimeScale = 1f;
    private RosConnection rosConnection;
    private bool halted;

    public void Configure(float requestedTimeScale)
    {
        simulationTimeScale = Mathf.Max(requestedTimeScale, 0.001f);
    }

    private void Awake()
    {
        rosConnection = GetComponent<RosConnection>();
        if (rosConnection == null)
        {
            Debug.LogError($"[{nameof(FinsROS2ControlLockstep)}] RosConnection is missing.", this);
            enabled = false;
            return;
        }

        // Do not let physics advance before the gRPC connection and the
        // controller-side acknowledgement path are available.  Update keeps
        // running at timeScale=0 and resumes the first fixed step after the
        // connection is ready.
        Time.timeScale = 0f;
        Debug.Log(
            $"[{nameof(FinsROS2ControlLockstep)}] waiting for ROS2 control acknowledgement path at " +
            $"requested time scale {simulationTimeScale:F2}x.",
            this);
    }

    private void Update()
    {
        if (halted || rosConnection == null || !rosConnection.Ros2ControlLockstep)
        {
            return;
        }

        if (rosConnection.IsConnected && Time.timeScale <= 0f)
        {
            Time.timeScale = simulationTimeScale;
        }
    }

    private void FixedUpdate()
    {
        if (halted || rosConnection == null || !rosConnection.Ros2ControlLockstep)
        {
            return;
        }

        if (!rosConnection.IsConnected)
        {
            Time.timeScale = 0f;
            return;
        }

        if (!rosConnection.StepRos2ControlLockstep())
        {
            halted = true;
            Time.timeScale = 0f;
            Debug.LogError(
                $"[{nameof(FinsROS2ControlLockstep)}] ROS2 did not acknowledge the current control tick; " +
                "simulation is paused for inspection.",
                this);
        }
    }
}
