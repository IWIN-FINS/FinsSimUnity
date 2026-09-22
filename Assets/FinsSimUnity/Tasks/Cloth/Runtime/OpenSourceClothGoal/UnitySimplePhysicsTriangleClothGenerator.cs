using System;
using System.Collections.Generic;
using UnitySimpleCloth = OpenSourceCloth.UnitySimplePhysics.UnitySimplePhysicsCloth;
using UnityEngine;

/// <summary>
/// Binds a pre-authored, three-corner triangular topology to UnitySimplePhysics
/// Cloth.  Points and joints are serialized in the scene; this component does
/// not create them at runtime.
/// </summary>
[DefaultExecutionOrder(-100)]
[RequireComponent(typeof(UnitySimpleCloth))]
public sealed class UnitySimplePhysicsTriangleClothGenerator : MonoBehaviour
{
    public Transform topAnchor;
    public Transform leftAnchor;
    public Transform rightAnchor;

    [Range(2, 16)] public int subdivisions = 8;
    [Min(0.001f)] public float nodeMassKg = 0.025f;
    [Min(0f)] public float nodeLinearDamping = 1.8f;
    [Min(0.001f)] public float nodeColliderRadiusM = 0.035f;
    [Min(0.0001f)] public float jointProjectionDistanceM = 0.003f;
    public Material lineMaterial;
    public Material clothMaterial;

    [SerializeField] private List<Transform> points = new();
    [SerializeField] private int[] triangles;
    [SerializeField] private Vector2Int[] edges;

    private void Awake()
    {
        if (topAnchor == null || leftAnchor == null || rightAnchor == null)
            throw new InvalidOperationException("UnitySimplePhysicsTriangleClothGenerator needs top, left, and right anchors.");

        if (points.Count < 3 || triangles == null || triangles.Length < 3 || edges == null || edges.Length == 0)
            throw new InvalidOperationException("UnitySimplePhysicsTriangleClothGenerator requires serialized scene topology.");

        UnitySimpleCloth cloth = GetComponent<UnitySimpleCloth>();
        cloth.showLineRenderers = false;
        cloth.showMesh = true;
        cloth.addMeshCollider = true;
        cloth.meshColliderConvex = false;
        cloth.relayCollisionImpulse = true;
        cloth.impulseNearestN = 8;
        cloth.impulseScale = 1f;
        cloth.InitTriangular(points, triangles, new List<Vector2Int>(edges), lineMaterial, clothMaterial);
    }

    public void SetStaticTopology(List<Transform> scenePoints, int[] sceneTriangles, List<Vector2Int> sceneEdges)
    {
        points = scenePoints;
        triangles = sceneTriangles;
        edges = sceneEdges.ToArray();
    }

    public static void CreateDistanceJoint(Rigidbody first, Rigidbody second, float projectionDistanceM)
    {
        if (first == null || second == null)
            throw new InvalidOperationException("Every UnitySimplePhysics cloth point needs a Rigidbody.");

        ConfigurableJoint joint = first.gameObject.AddComponent<ConfigurableJoint>();
        joint.connectedBody = second;
        joint.autoConfigureConnectedAnchor = true;
        joint.anchor = Vector3.zero;
        joint.xMotion = ConfigurableJointMotion.Locked;
        joint.yMotion = ConfigurableJointMotion.Locked;
        joint.zMotion = ConfigurableJointMotion.Locked;
        joint.angularXMotion = ConfigurableJointMotion.Free;
        joint.angularYMotion = ConfigurableJointMotion.Free;
        joint.angularZMotion = ConfigurableJointMotion.Free;
        joint.projectionMode = JointProjectionMode.PositionAndRotation;
        joint.projectionDistance = projectionDistanceM;
        joint.projectionAngle = 1f;
        joint.enableCollision = false;
    }

    public static List<Vector2Int> BuildEdges(int[] triangles)
    {
        HashSet<Vector2Int> uniqueEdges = new();
        for (int i = 0; i < triangles.Length; i += 3)
        {
            AddEdge(uniqueEdges, triangles[i], triangles[i + 1]);
            AddEdge(uniqueEdges, triangles[i + 1], triangles[i + 2]);
            AddEdge(uniqueEdges, triangles[i + 2], triangles[i]);
        }
        return new List<Vector2Int>(uniqueEdges);
    }

    private static void AddEdge(HashSet<Vector2Int> edges, int a, int b)
    {
        edges.Add(a < b ? new Vector2Int(a, b) : new Vector2Int(b, a));
    }
}
