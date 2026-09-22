using UnityEngine;

public class VelocityTrainingDebugGUI : TrainingDebugGUIBase
{
    [SerializeField] ControlForVelocity_IncrementalReward agent;

    protected override void Awake()
    {
        base.Awake();
        TryFindAgent();
    }

    protected override string GetWindowTitle()
    {
        return "Velocity RL Debug";
    }

    protected override void DrawContent()
    {
        if (agent == null)
        {
            TryFindAgent();
        }

        if (agent == null)
        {
            DrawStatus("Agent not found: ControlForVelocity_IncrementalReward", false);
            return;
        }

        DrawTitle("Velocity Task");
        DrawRow("Step Reward", agent.LastStepReward.ToString("F4"));
        DrawRow("Cumulative Reward", agent.GetCumulativeReward().ToString("F4"));
        DrawRow("Target Linear Vel", FormatVector3(agent.TargetLinearVelocity));
        DrawRow("Current Linear Vel", FormatVector3(agent.CurrentLinearVelocity));
        DrawRow("Target Angular Vel", FormatVector3(agent.TargetAngularVelocity));
        DrawRow("Current Angular Vel", FormatVector3(agent.CurrentAngularVelocity));
        DrawRow("Linear Error01", agent.CurrentLinearError01.ToString("F3"));
        DrawRow("Angular Error01", agent.CurrentAngularError01.ToString("F3"));
        DrawRow("Stable Steps", $"{agent.StableTrackingSteps}/{agent.RequiredStableSteps}");

        bool stableEnough =
            agent.CurrentLinearError01 < agent.LinearSuccessError01 &&
            agent.CurrentAngularError01 < agent.AngularSuccessError01;
        DrawStatus(stableEnough ? "Status: stable tracking window" : "Status: converging", stableEnough);
    }

    string FormatVector3(Vector3 value)
    {
        return $"({value.x:F2}, {value.y:F2}, {value.z:F2})";
    }

    void TryFindAgent()
    {
        if (agent == null)
        {
            agent = FindObjectOfType<ControlForVelocity_IncrementalReward>();
        }
    }
}
