using UnityEngine;

// Minimal interface required by the imported ClothSimulationTutorial source.
public interface IGrabbable
{
    void StartGrab(Vector3 grabPos);
    void MoveGrabbed(Vector3 grabPos);
    void EndGrab(Vector3 grabPos, Vector3 velocity);
    void IsRayHittingBody(Ray ray, out CustomHit hit);
    Vector3 GetGrabbedPos();
}
