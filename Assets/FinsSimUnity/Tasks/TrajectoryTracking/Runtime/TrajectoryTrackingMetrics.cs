using System;
using FinsSim.Actuators;
using Unity.MLAgents;
using UnityEngine;

/// <summary>Low-overhead aggregate train/eval metrics for the T2 task.</summary>
public static class TrajectoryTrackingMetrics
{
    const int SamplesPerReport = 16384;
    const float SaturationThreshold = 0.98f;
    const string Prefix = "FinsROV/trajectory_tracking/";
    static readonly float[] TrackingErrors = new float[SamplesPerReport];
    static readonly float[] AngularSpeeds = new float[SamplesPerReport];
    static readonly float[] ActionDeltaRms = new float[SamplesPerReport];
    static readonly float[] ActualSaturationCounts = new float[FinsROVAgentRuntime.DefaultThrusterOrder.Length];
    static int sampleCount;
    static float errorSum;
    static float errorMax;
    static float angularSpeedSum;
    static float actionRmsSum;
    static float actionDeltaSum;
    static float policyToThrusterDeltaSum;
    static float completionSum;

    public static void Record(
        TrajectoryTrackingAgent agent,
        float error,
        Vector3 angularVelocityBody,
        float[] action,
        float[] previousAction,
        float[] thrusterAction)
    {
        int index = sampleCount;
        TrackingErrors[index] = error;
        AngularSpeeds[index] = angularVelocityBody.magnitude;
        errorSum += error;
        errorMax = Mathf.Max(errorMax, error);
        angularSpeedSum += AngularSpeeds[index];
        float rms = 0f;
        float actionDeltaSquared = 0f;
        float policyToThrusterDelta = 0f;
        int actionCount = agent.PolicyActionCount;
        for (int actionIndex = 0; actionIndex < actionCount; actionIndex++)
        {
            rms += action[actionIndex] * action[actionIndex];
            float previous = previousAction != null && actionIndex < previousAction.Length ? previousAction[actionIndex] : 0f;
            float delta = action[actionIndex] - previous;
            actionDeltaSquared += delta * delta;
            if (thrusterAction != null && actionIndex < thrusterAction.Length)
            {
                policyToThrusterDelta += Mathf.Abs(action[actionIndex] - thrusterAction[actionIndex]);
            }
        }
        actionRmsSum += Mathf.Sqrt(rms / Mathf.Max(actionCount, 1));
        ActionDeltaRms[index] = Mathf.Sqrt(actionDeltaSquared / Mathf.Max(actionCount, 1));
        actionDeltaSum += ActionDeltaRms[index];
        policyToThrusterDeltaSum += policyToThrusterDelta / Mathf.Max(actionCount, 1);
        completionSum += agent.MaxStep > 0 ? Mathf.Clamp01((float)agent.StepCount / agent.MaxStep) : 0f;
        RecordThrusterSaturation(agent.OrderedThrusters, thrusterAction);

        sampleCount++;
        if (sampleCount >= SamplesPerReport) FlushPending();
    }

    public static void FlushPending()
    {
        if (sampleCount == 0 || Academy.Instance == null) return;
        float inverse = 1f / sampleCount;
        Add("tracking_error_mean_m", errorSum * inverse);
        Add("tracking_error_p95_m", Percentile95(TrackingErrors, sampleCount));
        Add("tracking_error_max_m", errorMax);
        Add("angular_speed_mean_radps", angularSpeedSum * inverse);
        Add("angular_speed_p95_radps", Percentile95(AngularSpeeds, sampleCount));
        Add("policy_action_rms", actionRmsSum * inverse);
        Add("policy_action_delta_rms_mean", actionDeltaSum * inverse);
        Add("policy_action_delta_rms_p95", Percentile95(ActionDeltaRms, sampleCount));
        Add("policy_to_thruster_abs_delta_mean", policyToThrusterDeltaSum * inverse);
        Add("trajectory_progress", completionSum * inverse);
        for (int thrusterIndex = 0; thrusterIndex < ActualSaturationCounts.Length; thrusterIndex++)
        {
            Add($"thruster_actual_saturation_rate/{FinsROVAgentRuntime.DefaultThrusterOrder[thrusterIndex]}", ActualSaturationCounts[thrusterIndex] * inverse);
        }
        Reset();
    }

    static void RecordThrusterSaturation(Thruster[] thrusters, float[] action)
    {
        for (int index = 0; index < ActualSaturationCounts.Length; index++)
        {
            Thruster thruster = thrusters != null && index < thrusters.Length ? thrusters[index] : null;
            float command = action != null && index < action.Length ? action[index] : 0f;
            if (thruster == null || Mathf.Abs(command) <= 1e-5f) continue;
            float limit = command >= 0f ? Mathf.Abs(thruster.MaxForwardForceN) : Mathf.Abs(thruster.MaxReverseForceN);
            if (limit > 1e-5f && Mathf.Abs(thruster.LastForceRequest) >= SaturationThreshold * limit) ActualSaturationCounts[index] += 1f;
        }
    }

    static void Add(string name, float value) => Academy.Instance.StatsRecorder.Add(Prefix + name, value, StatAggregationMethod.MostRecent);

    static float Percentile95(float[] values, int count)
    {
        Array.Sort(values, 0, count);
        return values[Mathf.Clamp(Mathf.CeilToInt(count * 0.95f) - 1, 0, count - 1)];
    }

    static void Reset()
    {
        sampleCount = 0;
        errorSum = 0f;
        errorMax = 0f;
        angularSpeedSum = 0f;
        actionRmsSum = 0f;
        actionDeltaSum = 0f;
        policyToThrusterDeltaSum = 0f;
        completionSum = 0f;
        Array.Clear(ActualSaturationCounts, 0, ActualSaturationCounts.Length);
    }
}
