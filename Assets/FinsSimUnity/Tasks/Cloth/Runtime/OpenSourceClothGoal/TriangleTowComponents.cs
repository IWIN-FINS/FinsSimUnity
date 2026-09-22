using System;
using System.Collections.Generic;
using UnityEngine;

/// <summary>Moves three kinematic towing particles together along the world +X axis.</summary>
[DefaultExecutionOrder(-200)]
public sealed class TriangleTowAnchorDriver : MonoBehaviour
{
    public Transform top;
    public Transform left;
    public Transform right;
    [Min(0f)] public float speedMps = 0.55f;
    [Min(0f)] public float travelDistanceM = 4f;

    private Transform[] anchors;
    private Vector3[] startPositions;
    private float elapsed;
    public bool IsComplete { get; private set; }

    private void Awake()
    {
        anchors = new[] { top, left, right };
        startPositions = new Vector3[anchors.Length];
        for (int i = 0; i < anchors.Length; i++)
        {
            if (anchors[i] == null)
                throw new InvalidOperationException("TriangleTowAnchorDriver requires exactly three anchor transforms.");
            startPositions[i] = anchors[i].position;
        }
    }

    private void FixedUpdate()
    {
        if (IsComplete)
            return;

        elapsed += Time.fixedDeltaTime;
        float distance = Mathf.Min(travelDistanceM, elapsed * speedMps);
        for (int i = 0; i < anchors.Length; i++)
        {
            Vector3 next = startPositions[i] + Vector3.right * distance;
            Rigidbody body = anchors[i].GetComponent<Rigidbody>();
            if (body != null)
                body.MovePosition(next);
            else
                anchors[i].position = next;
        }

        // After the scripted travel, leave the particles completely under
        // scene/user control.  Do not keep restoring their final positions.
        if (distance >= travelDistanceM)
            IsComplete = true;
    }
}

/// <summary>Shared topology helper for all three triangular-net comparison scenes.</summary>
public sealed class TriangleTowLattice
{
    public Vector3[] Vertices { get; private set; }
    public int[] Triangles { get; private set; }
    public int TopIndex { get; private set; }
    public int LeftIndex { get; private set; }
    public int RightIndex { get; private set; }

    public static TriangleTowLattice Create(Vector3 top, Vector3 left, Vector3 right, int subdivisions)
    {
        int n = Mathf.Max(2, subdivisions);
        int count = (n + 1) * (n + 2) / 2;
        Vector3[] vertices = new Vector3[count];

        for (int row = 0; row <= n; row++)
        {
            for (int column = 0; column <= n - row; column++)
            {
                float u = column / (float)n;
                float v = row / (float)n;
                vertices[IndexOf(n, row, column)] = top * (1f - u - v) + left * u + right * v;
            }
        }

        List<int> triangles = new();
        for (int row = 0; row < n; row++)
        {
            for (int column = 0; column < n - row; column++)
            {
                int a = IndexOf(n, row, column);
                int b = IndexOf(n, row, column + 1);
                int c = IndexOf(n, row + 1, column);
                triangles.Add(a);
                triangles.Add(b);
                triangles.Add(c);

                if (column < n - row - 1)
                {
                    triangles.Add(b);
                    triangles.Add(IndexOf(n, row + 1, column + 1));
                    triangles.Add(c);
                }
            }
        }

        return new TriangleTowLattice
        {
            Vertices = vertices,
            Triangles = triangles.ToArray(),
            TopIndex = IndexOf(n, 0, 0),
            LeftIndex = IndexOf(n, 0, n),
            RightIndex = IndexOf(n, n, 0),
        };
    }

    /// <summary>
    /// Creates a triangular net whose attachment boundary exactly matches the
    /// three supplied anchors, while only its interior has spare material.
    /// The pocket reaches <paramref name="rearwardPocketDepthM"/> along world
    /// -X at the barycentric centre and is exactly zero at all edges/corners.
    /// </summary>
    public static TriangleTowLattice CreateRearwardPocket(
        Vector3 top,
        Vector3 left,
        Vector3 right,
        int subdivisions,
        float rearwardPocketDepthM)
    {
        TriangleTowLattice lattice = Create(top, left, right, subdivisions);
        int n = Mathf.Max(2, subdivisions);
        float depth = Mathf.Max(0f, rearwardPocketDepthM);

        for (int row = 0; row <= n; row++)
        {
            for (int column = 0; column <= n - row; column++)
            {
                float u = column / (float)n;
                float v = row / (float)n;
                float w = 1f - u - v;
                // 27*u*v*w has unit peak at u=v=w=1/3 and vanishes on every boundary.
                float pocketWeight = 27f * u * v * w;
                int index = IndexOf(n, row, column);
                lattice.Vertices[index] += Vector3.left * (depth * pocketWeight);
            }
        }

        return lattice;
    }

    /// <summary>
    /// Creates a Reuleaux-triangle-like net: the three attachment vertices
    /// stay fixed, while each boundary edge becomes the circular arc centred
    /// on the opposite vertex. For the equilateral FinsROV formation this is
    /// an exact Reuleaux triangle. A smooth barycentric blend carries those
    /// three boundary arcs through the mesh interior.
    /// </summary>
    public static TriangleTowLattice CreateReuleauxRearwardPocket(
        Vector3 top,
        Vector3 left,
        Vector3 right,
        int subdivisions,
        float rearwardPocketDepthM)
    {
        TriangleTowLattice lattice = Create(top, left, right, subdivisions);
        int n = Mathf.Max(2, subdivisions);
        float pocketDepth = Mathf.Max(0f, rearwardPocketDepthM);

        for (int row = 0; row <= n; row++)
        {
            for (int column = 0; column <= n - row; column++)
            {
                float u = column / (float)n;
                float v = row / (float)n;
                float w = 1f - u - v;
                int index = IndexOf(n, row, column);
                Vector3 basePoint = lattice.Vertices[index];

                // Reuleaux arc on Left--Right, centred on Top.
                float leftRightWeight = u + v;
                Vector3 leftRightDeviation = leftRightWeight > 0f
                    ? ReuleauxArcDeviation(left, right, top, v / leftRightWeight)
                    : Vector3.zero;

                // Reuleaux arc on Top--Right, centred on Left.
                float topRightWeight = w + v;
                Vector3 topRightDeviation = topRightWeight > 0f
                    ? ReuleauxArcDeviation(top, right, left, v / topRightWeight)
                    : Vector3.zero;

                // Reuleaux arc on Top--Left, centred on Right.
                float topLeftWeight = w + u;
                Vector3 topLeftDeviation = topLeftWeight > 0f
                    ? ReuleauxArcDeviation(top, left, right, u / topLeftWeight)
                    : Vector3.zero;

                // Each curved-boundary displacement has unit influence at its
                // own edge and becomes zero at the opposite corner.
                Vector3 curvedPoint = basePoint
                    + leftRightWeight * leftRightDeviation
                    + topRightWeight * topRightDeviation
                    + topLeftWeight * topLeftDeviation;

                // Extra mesh material remains inside the face: no boundary
                // or FinsROV attachment point receives a -X offset.
                float pocketWeight = 27f * u * v * w;
                lattice.Vertices[index] = curvedPoint + Vector3.left * (pocketDepth * pocketWeight);
            }
        }

        return lattice;
    }

    private static Vector3 ReuleauxArcDeviation(Vector3 start, Vector3 end, Vector3 arcCenter, float t)
    {
        t = Mathf.Clamp01(t);
        Vector3 straightPoint = Vector3.Lerp(start, end, t);
        Vector3 startRadius = start - arcCenter;
        Vector3 endRadius = end - arcCenter;
        if (startRadius.sqrMagnitude <= 1e-10f || endRadius.sqrMagnitude <= 1e-10f)
            return Vector3.zero;

        // The current formation is equilateral, so Slerp traces the exact
        // constant-radius Reuleaux arc. It also degrades gracefully if a
        // later reset perturbs the three anchors slightly.
        Vector3 arcPoint = arcCenter + Vector3.Slerp(startRadius, endRadius, t);
        return arcPoint - straightPoint;
    }

    public static Mesh CreateMesh(string name, TriangleTowLattice lattice)
    {
        Mesh mesh = new() { name = name };
        mesh.vertices = lattice.Vertices;
        mesh.triangles = lattice.Triangles;
        Vector2[] uv = new Vector2[lattice.Vertices.Length];
        for (int i = 0; i < uv.Length; i++)
        {
            Vector3 vertex = lattice.Vertices[i];
            uv[i] = new Vector2(vertex.y, vertex.z);
        }
        mesh.uv = uv;
        mesh.RecalculateNormals();
        mesh.RecalculateBounds();
        return mesh;
    }

    private static int IndexOf(int subdivisions, int row, int column)
    {
        return row * (subdivisions + 1) - row * (row - 1) / 2 + column;
    }
}
