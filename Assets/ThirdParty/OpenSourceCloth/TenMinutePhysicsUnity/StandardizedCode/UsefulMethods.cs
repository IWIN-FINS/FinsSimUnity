using UnityEngine;

// Minimal helper subset required by Intersections.cs. The original file used
// an HDRP-internal namespace that no longer exists in Unity 6.
public static class UsefulMethods
{
    public const float EPSILON = 0.00001f;

    public static Vector3 GetClosestPointOnRay(Vector3 point, Ray ray)
    {
        Vector3 direction = ray.direction;
        float denominator = Vector3.Dot(direction, direction);
        if (denominator <= 0f)
            return ray.origin;
        float t = Mathf.Max(0f, Vector3.Dot(point - ray.origin, direction) / denominator);
        return ray.origin + direction * t;
    }
}
