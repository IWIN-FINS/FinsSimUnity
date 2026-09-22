using FinsSim.Networking;
using UnityEngine;

/// <summary>
/// Per-scene runtime clock override installed by the ROS2 simulation runner.
///
/// The training Fossen scene keeps its RosConnection disabled.  During a
/// hardware-aligned evaluation the editor activates that object and attaches
/// this component before entering Play mode.  Its execution order is earlier
/// than RosConnection's (-1), so TimeHandler is initialized with the intended
/// stepped simulation clock rather than the real-time defaults.
/// </summary>
[DefaultExecutionOrder(-1000)]
public sealed class FinsROSEvaluationRuntimeClock : MonoBehaviour
{
    [SerializeField] private float simulationTimeScale = 1f;
    [SerializeField] private bool ros2ControlLockstep;

    public void Configure(float requestedTimeScale, bool requestedRos2ControlLockstep = false)
    {
        simulationTimeScale = Mathf.Max(requestedTimeScale, 0.001f);
        ros2ControlLockstep = requestedRos2ControlLockstep;
    }

    private void Awake()
    {
        var rosConnection = GetComponent<RosConnection>();
        if (rosConnection == null)
        {
            Debug.LogError($"[{nameof(FinsROSEvaluationRuntimeClock)}] RosConnection is missing.", this);
            return;
        }

        rosConnection.RealtimeSimulation = false;
        rosConnection.SimulationSpeed = simulationTimeScale;
        rosConnection.Ros2ControlLockstep = ros2ControlLockstep;
        Time.timeScale = 1f;
        Debug.Log(
            $"[{nameof(FinsROSEvaluationRuntimeClock)}] enabled stepped /clock at {simulationTimeScale:F2}x; " +
            $"ros2ControlLockstep={ros2ControlLockstep}.",
            this);
    }
}
