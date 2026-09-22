using System;
using UnityEngine;

/// <summary>XPBD triangular-net driver with explicit anchors and Target contact.</summary>
[DefaultExecutionOrder(-100)]
[RequireComponent(typeof(MeshFilter))]
public sealed class HabradorTriangleTowDriver : MonoBehaviour
{
    public Transform topAnchor;
    public Transform leftAnchor;
    public Transform rightAnchor;
    [Range(2, 16)] public int subdivisions = 8;
    [Min(0)] public int solverSubsteps = 8;
    [Min(0f)] public float stretchingCompliance = 0.00002f;
    [Min(0f)] public float bendingCompliance = 0.001f;
    [Range(0f, 20f)] public float velocityDamping = 2f;
    [Tooltip("Scales the tutorial cloth's default inverse masses. A small value gives the anchored net enough effective mass to tow the Target instead of locally yielding around it.")]
    [Range(0.00001f, 1f)] public float dynamicParticleInverseMassScale = 0.0002f;
    [Header("FinsROV Cloth Attachments")]
    [Tooltip("Relaxed length of each elastic FinsROV-to-cloth corner lead.")]
    [Min(0f)] public float anchorSpringRestLengthM = 0.10f;
    [Tooltip("XPBD compliance of each FinsROV-to-cloth lead. Higher values make the connection softer.")]
    [Min(0f)] public float anchorSpringCompliance = 0.000001f;
    public bool enableTutorialFloorCollision = false;
    public Rigidbody targetBody;
    [Tooltip("Optional real Target shape. When assigned (or found on Target), XPBD uses capsule-triangle contact instead of a bounding sphere.")]
    public CapsuleCollider targetCapsuleCollider;
    [Min(0.001f)] public float targetRadiusM = 0.3f;
    [Min(0f)] public float targetFriction = 0.8f;
    [HideInInspector] public int topParticleIndex;
    [HideInInspector] public int leftParticleIndex;
    [HideInInspector] public int rightParticleIndex;

    private ClothSimulationTutorial simulation;
    private Mesh restMesh;
    private MeshFilter meshFilter;

    private void Awake()
    {
        meshFilter = GetComponent<MeshFilter>();
        Mesh initialMesh = meshFilter.sharedMesh;
        if (initialMesh == null)
            throw new InvalidOperationException("HabradorTriangleTowDriver requires a serialized triangle mesh in the scene.");
        if (topAnchor == null || leftAnchor == null || rightAnchor == null || targetBody == null)
            throw new InvalidOperationException("HabradorTriangleTowDriver requires three anchors and the target Rigidbody.");

        restMesh = Instantiate(initialMesh);
        InitializeSimulation(meshFilter);
    }

    /// <summary>Restores the XPBD net to its serialized rest shape after an episode reset.</summary>
    public void ResetSimulation()
    {
        if (restMesh == null)
            return;
        InitializeSimulation(GetComponent<MeshFilter>());
    }

    public bool TryGetTriangleVertices(out Vector3 top, out Vector3 left, out Vector3 right)
    {
        top = topAnchor != null ? topAnchor.position : Vector3.zero;
        left = leftAnchor != null ? leftAnchor.position : Vector3.zero;
        right = rightAnchor != null ? rightAnchor.position : Vector3.zero;
        return topAnchor != null && leftAnchor != null && rightAnchor != null &&
            Vector3.Cross(left - top, right - top).sqrMagnitude > 1e-10f;
    }

    public Vector3 NetCenter => (topAnchor.position + leftAnchor.position + rightAnchor.position) / 3f;

    /// <summary>
    /// Gets the shortest separation between a collider and the currently simulated
    /// XPBD cloth surface. The mesh is refreshed after every solver step, so this
    /// intentionally reads the runtime mesh rather than the serialized rest mesh.
    /// </summary>
    public bool TryGetClosestSurfaceDistance(Collider collider, out float distance)
    {
        distance = float.PositiveInfinity;
        if (collider == null)
            return false;

        Mesh mesh = meshFilter != null ? meshFilter.sharedMesh : null;
        if (mesh == null)
            return false;

        Vector3[] vertices = mesh.vertices;
        int[] triangles = mesh.triangles;
        if (vertices.Length == 0 || triangles.Length < 3)
            return false;

        Vector3 colliderCenter = collider.bounds.center;
        for (int i = 0; i + 2 < triangles.Length; i += 3)
        {
            int indexA = triangles[i];
            int indexB = triangles[i + 1];
            int indexC = triangles[i + 2];
            if (indexA < 0 || indexB < 0 || indexC < 0 ||
                indexA >= vertices.Length || indexB >= vertices.Length || indexC >= vertices.Length)
            {
                continue;
            }

            Vector3 a = transform.TransformPoint(vertices[indexA]);
            Vector3 b = transform.TransformPoint(vertices[indexB]);
            Vector3 c = transform.TransformPoint(vertices[indexC]);

            // Project from both shapes once. This gives the actual collider surface
            // distance for the capsule Target rather than centre-to-cloth distance.
            Vector3 clothPoint = ClosestPointOnTriangle(colliderCenter, a, b, c);
            Vector3 targetPoint = collider.ClosestPoint(clothPoint);
            clothPoint = ClosestPointOnTriangle(targetPoint, a, b, c);
            targetPoint = collider.ClosestPoint(clothPoint);
            distance = Mathf.Min(distance, Vector3.Distance(clothPoint, targetPoint));
        }

        return !float.IsPositiveInfinity(distance);
    }

    private static Vector3 ClosestPointOnTriangle(Vector3 point, Vector3 a, Vector3 b, Vector3 c)
    {
        Vector3 ab = b - a;
        Vector3 ac = c - a;
        Vector3 ap = point - a;
        float d1 = Vector3.Dot(ab, ap);
        float d2 = Vector3.Dot(ac, ap);
        if (d1 <= 0f && d2 <= 0f)
            return a;

        Vector3 bp = point - b;
        float d3 = Vector3.Dot(ab, bp);
        float d4 = Vector3.Dot(ac, bp);
        if (d3 >= 0f && d4 <= d3)
            return b;

        float vc = d1 * d4 - d3 * d2;
        if (vc <= 0f && d1 >= 0f && d3 <= 0f)
            return a + d1 / (d1 - d3) * ab;

        Vector3 cp = point - c;
        float d5 = Vector3.Dot(ab, cp);
        float d6 = Vector3.Dot(ac, cp);
        if (d6 >= 0f && d5 <= d6)
            return c;

        float vb = d5 * d2 - d1 * d6;
        if (vb <= 0f && d2 >= 0f && d6 <= 0f)
            return a + d2 / (d2 - d6) * ac;

        float va = d3 * d6 - d5 * d4;
        if (va <= 0f && d4 - d3 >= 0f && d5 - d6 >= 0f)
            return b + (d4 - d3) / ((d4 - d3) + (d5 - d6)) * (c - b);

        float denominator = 1f / (va + vb + vc);
        return a + ab * (vb * denominator) + ac * (vc * denominator);
    }

    private void InitializeSimulation(MeshFilter meshFilter)
    {
        if (meshFilter == null || restMesh == null)
            return;

        simulation = new ClothSimulationTutorial(
            meshFilter,
            new TriangleTowClothData(restMesh),
            Vector3.zero,
            meshScale: 1f,
            stretchingCompliance: stretchingCompliance,
            bendingCompliance: bendingCompliance,
            numSubSteps: solverSubsteps,
            velocityDamping: velocityDamping,
            enableFloorCollision: enableTutorialFloorCollision,
            pinTutorialRoofCorners: false);
        simulation.SetGravity(Vector3.zero);
        simulation.ScaleDynamicParticleInverseMass(dynamicParticleInverseMassScale);
        CapsuleCollider capsule = targetCapsuleCollider != null
            ? targetCapsuleCollider
            : targetBody.GetComponent<CapsuleCollider>();
        if (capsule != null)
            simulation.SetCapsuleContact(targetBody, capsule, targetFriction);
        else
            simulation.SetSphereContact(targetBody, targetRadiusM, targetFriction);
    }

    private void FixedUpdate()
    {
        if (simulation == null)
            return;
        simulation.SetSpringAttachment(
            topParticleIndex, topAnchor.position, anchorSpringRestLengthM, anchorSpringCompliance);
        simulation.SetSpringAttachment(
            leftParticleIndex, leftAnchor.position, anchorSpringRestLengthM, anchorSpringCompliance);
        simulation.SetSpringAttachment(
            rightParticleIndex, rightAnchor.position, anchorSpringRestLengthM, anchorSpringCompliance);
        simulation.MyFixedUpdate();
        simulation.RefreshMesh();
    }

    private void Update()
    {
        if (simulation != null)
            simulation.MyUpdate();
    }

    private sealed class TriangleTowClothData : ClothData
    {
        private readonly float[] vertices;
        private readonly int[] triangles;
        public override float[] GetVerts => vertices;
        public override int[] GetFaceTriIds => triangles;

        public TriangleTowClothData(Mesh mesh)
        {
            Vector3[] meshVertices = mesh.vertices;
            vertices = new float[meshVertices.Length * 3];
            for (int i = 0; i < meshVertices.Length; i++)
            {
                vertices[3 * i] = meshVertices[i].x;
                vertices[3 * i + 1] = meshVertices[i].y;
                vertices[3 * i + 2] = meshVertices[i].z;
            }
            triangles = mesh.triangles;
        }
    }
}
