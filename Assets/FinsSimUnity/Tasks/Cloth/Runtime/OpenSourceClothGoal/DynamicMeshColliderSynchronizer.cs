using UnityEngine;

/// <summary>Keeps a non-convex collider synchronized with a runtime-deformed mesh.</summary>
[DefaultExecutionOrder(200)]
[RequireComponent(typeof(MeshFilter), typeof(MeshCollider))]
public sealed class DynamicMeshColliderSynchronizer : MonoBehaviour
{
    private MeshFilter meshFilter;
    private MeshCollider meshCollider;

    private void Awake()
    {
        meshFilter = GetComponent<MeshFilter>();
        meshCollider = GetComponent<MeshCollider>();
    }

    private void FixedUpdate()
    {
        Synchronize();
    }

    private void LateUpdate()
    {
        Synchronize();
    }

    private void Synchronize()
    {
        Mesh mesh = meshFilter.sharedMesh;
        if (mesh == null)
            return;

        // Reassigning after clearing forces PhysX to read the updated vertices.
        meshCollider.sharedMesh = null;
        meshCollider.sharedMesh = mesh;
        Physics.SyncTransforms();
    }
}
