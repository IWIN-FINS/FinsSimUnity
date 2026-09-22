using System;
using UnityEngine;

[Serializable]
public struct LowYawPoseRewardResult
{
    public float Reward;
    public float DistanceProgress;
    public float HeadingProgress;
    public float DirectionAlignment;
    public float ForwardApproachSpeed;
    public float NearTargetRatio;
    public float NearTargetGain;
    public float AngularSpeed;
    public float YawRate;
    public float NearTargetAngularPenalty;
    public bool InSuccessPose;
    public bool StableForSuccess;
}

[Serializable]
public sealed class LowYawPoseReward
{
    [Header("Success Threshold")]
    [Min(0f)] public float successDistance = 0.2f;
    [Range(0f, 180f)] public float successHeadingAngleDeg = 25f;
    [Min(0.01f)] public float nearTargetDistance = 0.6f;
    [Min(0f)] public float stableSuccessLinearVelocity = 0.1f;
    [Min(0f)] public float stableSuccessAngularVelocity = 0.2f;
    [Min(1)] public int stableSuccessStepsRequired = 10;

    [Header("Reward Weights")]
    public float perStepPenalty = -0.001f;
    [Min(0f)] public float distanceProgressRewardScale = 1.3f;
    [Min(0f)] public float headingProgressRewardScale = 0.05f;
    [Min(0f)] public float directionAlignmentRewardScale = 0.01f;
    [Min(0f)] public float approachVelocityRewardScale = 0.04f;
    [Min(0f)] public float stationKeepingPositionRewardScale = 0.1f;
    [Min(0f)] public float nearTargetYawAlignmentRewardScale = 0.02f;
    [Min(0f)] public float stableHoldRewardScale = 0.05f;
    [Min(0f)] public float angularVelocityPenaltyScale = 0.012f;
    [Min(0f)] public float nearTargetAngularVelocityPenaltyScale = 0.1f;
    [Min(0f)] public float nearTargetYawRatePenaltyScale = 0.18f;
    [Min(0f)] public float speedNearTargetPenaltyScale = 0.05f;
    [Min(0f)] public float actionEnergyPenaltyScale = 0.0015f;
    [Min(0f)] public float actionChangePenaltyScale = 0.0012f;
    public float successReward = 2f;
    public float outOfBoundsPenalty = -1f;

    public LowYawPoseRewardResult Evaluate(
        float previousDistanceToTarget,
        float previousHeadingError01,
        float distanceToTarget,
        float headingError01,
        Vector3 directionToTarget,
        Vector3 referenceForward,
        Vector3 worldLinearVelocity,
        Vector3 localLinearVelocity,
        Vector3 localAngularVelocity,
        float actionEnergy,
        float actionDelta)
    {
        Vector3 directionFlat = new Vector3(directionToTarget.x, 0f, directionToTarget.z);
        Vector3 referenceForwardFlat = new Vector3(referenceForward.x, 0f, referenceForward.z);
        float directionAlignment = 0f;
        if (directionFlat.sqrMagnitude > 1e-6f && referenceForwardFlat.sqrMagnitude > 1e-6f)
        {
            directionAlignment = Vector3.Dot(referenceForwardFlat.normalized, directionFlat.normalized);
        }

        float forwardApproachSpeed = directionToTarget.sqrMagnitude > 1e-6f
            ? Vector3.Dot(worldLinearVelocity, directionToTarget)
            : 0f;
        float nearTargetRatio = 1f - Mathf.Clamp01(distanceToTarget / Mathf.Max(nearTargetDistance, 0.01f));
        float nearTargetGain = nearTargetRatio * nearTargetRatio * (3f - 2f * nearTargetRatio);
        float angularSpeed = localAngularVelocity.magnitude;
        float yawRate = Mathf.Abs(localAngularVelocity.y);
        float distanceProgress = previousDistanceToTarget - distanceToTarget;
        float headingProgress = previousHeadingError01 - headingError01;
        float nearTargetAngularPenalty =
            nearTargetGain * (
                nearTargetAngularVelocityPenaltyScale * angularSpeed +
                nearTargetYawRatePenaltyScale * yawRate
            );

        bool inSuccessPose =
            distanceToTarget < Mathf.Max(successDistance, 0f) &&
            headingError01 < Mathf.Clamp(successHeadingAngleDeg / 180f, 0f, 1f);
        bool stableForSuccess =
            localLinearVelocity.magnitude < Mathf.Max(stableSuccessLinearVelocity, 0f) &&
            angularSpeed < Mathf.Max(stableSuccessAngularVelocity, 0f);

        float reward =
            perStepPenalty +
            distanceProgressRewardScale * distanceProgress +
            headingProgressRewardScale * headingProgress +
            directionAlignmentRewardScale * directionAlignment +
            approachVelocityRewardScale * Mathf.Clamp(forwardApproachSpeed, -1f, 1f) +
            stationKeepingPositionRewardScale * nearTargetRatio +
            nearTargetYawAlignmentRewardScale * nearTargetGain * (1f - headingError01) +
            stableHoldRewardScale * nearTargetGain * (stableForSuccess ? 1f : 0f) -
            angularVelocityPenaltyScale * angularSpeed -
            nearTargetAngularPenalty -
            speedNearTargetPenaltyScale * nearTargetRatio * localLinearVelocity.magnitude -
            actionEnergyPenaltyScale * actionEnergy -
            actionChangePenaltyScale * actionDelta;

        return new LowYawPoseRewardResult
        {
            Reward = reward,
            DistanceProgress = distanceProgress,
            HeadingProgress = headingProgress,
            DirectionAlignment = directionAlignment,
            ForwardApproachSpeed = forwardApproachSpeed,
            NearTargetRatio = nearTargetRatio,
            NearTargetGain = nearTargetGain,
            AngularSpeed = angularSpeed,
            YawRate = yawRate,
            NearTargetAngularPenalty = nearTargetAngularPenalty,
            InSuccessPose = inSuccessPose,
            StableForSuccess = stableForSuccess,
        };
    }
}
