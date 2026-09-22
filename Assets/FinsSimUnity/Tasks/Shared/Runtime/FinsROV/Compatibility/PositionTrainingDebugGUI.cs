using UnityEngine;

public class PositionTrainingDebugGUI : TrainingDebugGUIBase
{
    [SerializeField] ControlForPosition_IncrementalReward agent;

    protected override void Awake()
    {
        base.Awake();
        TryFindAgent();
    }

    protected override string GetWindowTitle()
    {
        return "Position RL Debug";
    }

    protected override void DrawContent()
    {
        if (agent == null)
        {
            TryFindAgent();
        }

        if (agent == null)
        {
            DrawStatus("Agent not found: ControlForPosition_IncrementalReward", false);
            return;
        }

        DrawTitle("Position Task");
        DrawRow("Step Reward", agent.LastStepReward.ToString("F4"));
        DrawRow("Cumulative Reward", agent.GetCumulativeReward().ToString("F4"));
        DrawRow("Distance To Target", agent.CurrentDistanceToTarget.ToString("F3"));
        DrawRow("Success Distance", agent.SuccessDistance.ToString("F3"));
        DrawRow("Heading Error", agent.CurrentHeadingErrorDeg.ToString("F2") + " deg");
        DrawRow("Success Heading", agent.SuccessHeadingAngleDeg.ToString("F2") + " deg");
        DrawRow("Direction Alignment", agent.CurrentDirectionAlignment.ToString("F3"));
        DrawRow("Approach Speed", agent.CurrentForwardApproachSpeed.ToString("F3"));
        DrawRow("Near Target Ratio", agent.CurrentNearTargetRatio.ToString("F3"));
        DrawRow("Local Linear Vel", FormatVector3(agent.CurrentLocalLinearVelocity));
        DrawRow("Local Angular Vel", FormatVector3(agent.CurrentLocalAngularVelocity));

        bool closeEnough =
            agent.CurrentDistanceToTarget < agent.SuccessDistance &&
            agent.CurrentHeadingErrorDeg < agent.SuccessHeadingAngleDeg;
        DrawStatus(closeEnough ? "Status: success window reached" : "Status: tracking", closeEnough);
    }

    string FormatVector3(Vector3 value)
    {
        return $"({value.x:F2}, {value.y:F2}, {value.z:F2})";
    }

    void TryFindAgent()
    {
        if (agent == null)
        {
            agent = FindObjectOfType<ControlForPosition_IncrementalReward>();
        }
    }
}
