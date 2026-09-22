using UnityEngine;

/// <summary>
/// Shared definition of the T2 reference curves. Positions are in the FinsROV
/// controller-world convention: X forward, Y up, Z left. The ROS controller
/// mirrors the equations and observation layout in trajectory_tracking.py.
/// </summary>
public static class TrajectoryTrackingMath
{
    public enum Profile
    {
        Curriculum = 0,
        StraightLine = 1,
        Circle = 2,
        Lemniscate = 3,
        Helix = 4,
    }

    public struct Parameters
    {
        public Profile profile;
        public Vector3 origin;
        public float yawDeg;
        public Vector3 scale;
        public float angularSpeed;
        public float phase;
        public float lemniscateC;
        /// <summary>Episode duration used by finite, non-periodic profiles.</summary>
        public float durationSec;
    }

    public static void Evaluate(in Parameters parameters, float timeSec, out Vector3 position, out Vector3 velocity)
    {
        float phase = parameters.phase + parameters.angularSpeed * Mathf.Max(0f, timeSec);
        Vector3 localPosition;
        Vector3 localVelocity;
        float speed = parameters.angularSpeed;
        switch (parameters.profile)
        {
            case Profile.StraightLine:
                // A line is a finite 30 s reference, rather than phase integrated
                // motion. The previous expression grew without bound and ended
                // every straight-line episode at the area safety limit.
                float progress = Mathf.Clamp01(timeSec / Mathf.Max(parameters.durationSec, 1e-4f));
                float direction = Mathf.Cos(parameters.phase) >= 0f ? 1f : -1f;
                float startX = -direction * parameters.scale.x;
                float endX = direction * parameters.scale.x;
                localPosition = new Vector3(Mathf.Lerp(startX, endX, progress), 0f, 0f);
                localVelocity = timeSec < parameters.durationSec
                    ? new Vector3((endX - startX) / Mathf.Max(parameters.durationSec, 1e-4f), 0f, 0f)
                    : Vector3.zero;
                break;
            case Profile.Circle:
                localPosition = new Vector3(
                    parameters.scale.x * Mathf.Cos(phase),
                    0f,
                    parameters.scale.z * Mathf.Sin(phase));
                localVelocity = new Vector3(
                    -parameters.scale.x * Mathf.Sin(phase) * speed,
                    0f,
                    parameters.scale.z * Mathf.Cos(phase) * speed);
                break;
            case Profile.Helix:
                localPosition = new Vector3(
                    parameters.scale.x * Mathf.Cos(phase),
                    parameters.scale.y * phase / (2f * Mathf.PI),
                    parameters.scale.z * Mathf.Sin(phase));
                localVelocity = new Vector3(
                    -parameters.scale.x * Mathf.Sin(phase) * speed,
                    parameters.scale.y * speed / (2f * Mathf.PI),
                    parameters.scale.z * Mathf.Cos(phase) * speed);
                break;
            case Profile.Lemniscate:
            default:
                // Bernoulli lemniscate with a small centre-offset c, matching
                // MarineGym's family while remaining numerically well-behaved.
                float denominator = 1f + Mathf.Sin(phase) * Mathf.Sin(phase);
                float root = Mathf.Sqrt(Mathf.Max(denominator, 1e-6f));
                float x = Mathf.Cos(phase) / root;
                float z = Mathf.Sin(phase) * Mathf.Cos(phase) / denominator;
                float epsilon = 0.001f;
                float nextPhase = phase + epsilon;
                float nextDenominator = 1f + Mathf.Sin(nextPhase) * Mathf.Sin(nextPhase);
                float nextX = Mathf.Cos(nextPhase) / Mathf.Sqrt(Mathf.Max(nextDenominator, 1e-6f));
                float nextZ = Mathf.Sin(nextPhase) * Mathf.Cos(nextPhase) / nextDenominator;
                localPosition = new Vector3(
                    parameters.scale.x * x,
                    parameters.scale.y * parameters.lemniscateC * Mathf.Sin(phase),
                    parameters.scale.z * z);
                localVelocity = new Vector3(
                    parameters.scale.x * (nextX - x) / epsilon * speed,
                    parameters.scale.y * parameters.lemniscateC * Mathf.Cos(phase) * speed,
                    parameters.scale.z * (nextZ - z) / epsilon * speed);
                break;
        }

        Quaternion yaw = Quaternion.AngleAxis(parameters.yawDeg, Vector3.up);
        position = parameters.origin + yaw * localPosition;
        velocity = yaw * localVelocity;
    }

    public static Profile ResolveCurriculumProfile(int curriculumStage, System.Random random)
    {
        int stage = Mathf.Clamp(curriculumStage, 0, 2);
        int choice = random.Next(0, stage == 0 ? 2 : stage == 1 ? 3 : 4);
        if (stage == 0)
        {
            return choice == 0 ? Profile.StraightLine : Profile.Circle;
        }
        if (stage == 1)
        {
            return choice == 0 ? Profile.Circle : Profile.Lemniscate;
        }
        return choice switch
        {
            0 => Profile.Circle,
            1 => Profile.Lemniscate,
            _ => Profile.Helix,
        };
    }
}
