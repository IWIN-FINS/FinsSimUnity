using UnityEngine;

public static class ControllerBodyFrame
{
    public static Vector3 WorldToBodyVector(Transform reference, Vector3 worldVector)
    {
        if (reference == null)
        {
            return Vector3.zero;
        }

        return new Vector3(
            // Canonical FinsROV propulsion axes: Unity local +X is surge,
            // +Y is heave, and +Z is left/sway.
            Vector3.Dot(worldVector, reference.right),
            Vector3.Dot(worldVector, reference.up),
            Vector3.Dot(worldVector, reference.forward)
        );
    }

    public static Vector3 WorldToBodyPositionDelta(Transform reference, Vector3 worldPositionDelta)
    {
        return WorldToBodyVector(reference, worldPositionDelta);
    }

    public static Vector3 WorldLinearVelocityToBody(Transform reference, Vector3 worldLinearVelocity)
    {
        return WorldToBodyVector(reference, worldLinearVelocity);
    }

    public static Vector3 WorldAngularVelocityToBody(Transform reference, Vector3 worldAngularVelocity)
    {
        return WorldToBodyVector(reference, worldAngularVelocity);
    }

    public static float HorizontalBearingRad(Vector3 bodyVector)
    {
        return Mathf.Atan2(bodyVector.z, bodyVector.x);
    }
}
