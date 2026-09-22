using System;
using FinsSim.Actuators;
using Unity.MLAgents;
using UnityEngine;

/// <summary>
/// Batches telemetry from all replicated HoldForPosition areas before emitting it
/// through ML-Agents. This keeps high-area training observable without sending a
/// side-channel message for every agent on every physics tick.
/// </summary>
static class HoldForPositionTrainingMetrics
{
    const int SamplesPerReport = 16384;
    const float SaturationThreshold = 0.98f;
    const string MetricPrefix = "FinsROV/hold/";

    static readonly float[] PositionErrors = new float[SamplesPerReport];
    static readonly float[] AttitudeErrorsDeg = new float[SamplesPerReport];
    static readonly float[] NearTargetPositionErrors = new float[SamplesPerReport];
    static readonly float[] AngularSpeeds = new float[SamplesPerReport];
    static readonly float[] ActionDeltaRmsValues = new float[SamplesPerReport];
    static readonly float[] ActualSaturationCounts = new float[FinsROVAgentRuntime.DefaultThrusterOrder.Length];
    static readonly float[] CommandClampCounts = new float[FinsROVAgentRuntime.DefaultThrusterOrder.Length];

    static int sampleCount;
    static int nearTargetSampleCount;
    static float positionErrorSum;
    static float attitudeErrorDegSum;
    static float angularSpeedSum;
    static float actionDeltaRmsSum;
    static float stableHoldCount;
    static float nearTargetCount;
    static Vector3 angularVelocityAbsSum;

    public static void Record(
        float positionError,
        float attitudeErrorDeg,
        Vector3 localAngularVelocity,
        float actionDeltaRms,
        bool insideTargetHoldWindow,
        float nearTargetRadius,
        Thruster[] orderedThrusters,
        float[] actions,
        ThrusterController thrusterController)
    {
        int index = sampleCount;
        PositionErrors[index] = positionError;
        AttitudeErrorsDeg[index] = attitudeErrorDeg;
        AngularSpeeds[index] = localAngularVelocity.magnitude;
        ActionDeltaRmsValues[index] = actionDeltaRms;
        positionErrorSum += positionError;
        attitudeErrorDegSum += attitudeErrorDeg;
        angularSpeedSum += AngularSpeeds[index];
        actionDeltaRmsSum += actionDeltaRms;
        angularVelocityAbsSum += new Vector3(
            Mathf.Abs(localAngularVelocity.x),
            Mathf.Abs(localAngularVelocity.y),
            Mathf.Abs(localAngularVelocity.z));
        stableHoldCount += insideTargetHoldWindow ? 1f : 0f;

        if (positionError <= Mathf.Max(0.001f, nearTargetRadius))
        {
            NearTargetPositionErrors[nearTargetSampleCount++] = positionError;
            nearTargetCount += 1f;
        }

        RecordThrusterSaturation(orderedThrusters, actions, thrusterController);

        sampleCount++;
        if (sampleCount >= SamplesPerReport)
        {
            Flush();
        }
    }

    static void RecordThrusterSaturation(Thruster[] orderedThrusters, float[] actions, ThrusterController controller)
    {
        for (int index = 0; index < ActualSaturationCounts.Length; index++)
        {
            Thruster thruster = orderedThrusters != null && index < orderedThrusters.Length
                ? orderedThrusters[index]
                : null;
            float action = actions != null && index < actions.Length ? actions[index] : 0f;
            if (thruster == null || Mathf.Abs(action) <= 1e-5f)
            {
                continue;
            }

            float forceScale = controller != null ? Mathf.Max(0f, controller.ForceScaleMultiplier) : 1f;
            float targetForceLimit = action >= 0f
                ? Mathf.Abs(thruster.MaxForwardForceN) * forceScale
                : Mathf.Abs(thruster.MaxReverseForceN) * forceScale;
            if (targetForceLimit <= 1e-5f)
            {
                continue;
            }

            if (Mathf.Abs(thruster.TargetForceRequest) >= SaturationThreshold * targetForceLimit)
            {
                CommandClampCounts[index] += 1f;
            }

            float signedAppliedLimit = controller != null
                ? controller.ApplyForceResponseCompression(Mathf.Sign(action) * targetForceLimit)
                : Mathf.Sign(action) * targetForceLimit;
            float appliedForceLimit = Mathf.Min(targetForceLimit, Mathf.Abs(signedAppliedLimit));
            if (appliedForceLimit > 1e-5f &&
                Mathf.Abs(thruster.LastForceRequest) >= SaturationThreshold * appliedForceLimit)
            {
                ActualSaturationCounts[index] += 1f;
            }
        }
    }

    static void Flush()
    {
        if (sampleCount <= 0 || Academy.Instance == null)
        {
            Reset();
            return;
        }

        float inverseCount = 1f / sampleCount;
        Add("samples_per_report", sampleCount);
        Add("position_error_mean_m", positionErrorSum * inverseCount);
        Add("position_error_p95_m", Percentile95(PositionErrors, sampleCount));
        Add("attitude_error_mean_deg", attitudeErrorDegSum * inverseCount);
        Add("attitude_error_p95_deg", Percentile95(AttitudeErrorsDeg, sampleCount));
        Add("near_target_sample_ratio", nearTargetCount * inverseCount);
        Add("near_target_position_error_mean_m", nearTargetSampleCount > 0
            ? Sum(NearTargetPositionErrors, nearTargetSampleCount) / nearTargetSampleCount
            : 0f);
        Add("near_target_position_error_p95_m", nearTargetSampleCount > 0 ? Percentile95(NearTargetPositionErrors, nearTargetSampleCount) : 0f);
        Add("stable_hold_window_ratio", stableHoldCount * inverseCount);
        Add("angular_speed_mean_radps", angularSpeedSum * inverseCount);
        Add("angular_speed_p95_radps", Percentile95(AngularSpeeds, sampleCount));
        Add("angular_velocity_abs_roll_mean_radps", angularVelocityAbsSum.x * inverseCount);
        Add("angular_velocity_abs_pitch_mean_radps", angularVelocityAbsSum.y * inverseCount);
        Add("angular_velocity_abs_yaw_mean_radps", angularVelocityAbsSum.z * inverseCount);
        Add("action_delta_rms_mean", actionDeltaRmsSum * inverseCount);
        Add("action_delta_rms_p95", Percentile95(ActionDeltaRmsValues, sampleCount));

        float actualSaturationCount = 0f;
        for (int index = 0; index < ActualSaturationCounts.Length; index++)
        {
            float actualRate = ActualSaturationCounts[index] * inverseCount;
            float clampRate = CommandClampCounts[index] * inverseCount;
            Add($"thruster_actual_saturation_rate/{FinsROVAgentRuntime.DefaultThrusterOrder[index]}", actualRate);
            Add($"thruster_command_clamp_rate/{FinsROVAgentRuntime.DefaultThrusterOrder[index]}", clampRate);
            actualSaturationCount += ActualSaturationCounts[index];
        }
        Add("thruster_actual_saturation_rate", actualSaturationCount / (sampleCount * ActualSaturationCounts.Length));

        Reset();
    }

    static void Add(string name, float value)
    {
        Academy.Instance.StatsRecorder.Add(MetricPrefix + name, value, StatAggregationMethod.MostRecent);
    }

    static float Percentile95(float[] values, int count)
    {
        if (count <= 0)
        {
            return 0f;
        }

        Array.Sort(values, 0, count);
        return values[Mathf.Clamp(Mathf.CeilToInt(count * 0.95f) - 1, 0, count - 1)];
    }

    static float Sum(float[] values, int count)
    {
        float sum = 0f;
        for (int index = 0; index < count; index++)
        {
            sum += values[index];
        }
        return sum;
    }

    static void Reset()
    {
        sampleCount = 0;
        nearTargetSampleCount = 0;
        positionErrorSum = 0f;
        attitudeErrorDegSum = 0f;
        angularSpeedSum = 0f;
        actionDeltaRmsSum = 0f;
        stableHoldCount = 0f;
        nearTargetCount = 0f;
        angularVelocityAbsSum = Vector3.zero;
        Array.Clear(ActualSaturationCounts, 0, ActualSaturationCounts.Length);
        Array.Clear(CommandClampCounts, 0, CommandClampCounts.Length);
    }
}
