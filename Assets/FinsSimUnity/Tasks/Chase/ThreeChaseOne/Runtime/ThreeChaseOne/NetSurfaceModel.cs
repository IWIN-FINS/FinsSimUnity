using System.Collections.Generic;
using UnityEngine;

[DisallowMultipleComponent]
public class NetSurfaceModel : MonoBehaviour
{
    [Header("Generated Net Hierarchy")]
    [SerializeField] private string topLayerName = "Layer1";
    [SerializeField] private string middleLayerName = "Layer2";
    [SerializeField] private string bottomLayerName = "Layer3";
    [SerializeField] private string horizontalLayerName = "Horizontal";

    [Header("Target Geometry")]
    [SerializeField] private float targetNetWidth = 4.8f;
    [SerializeField] private float targetNetHeight = 1.0f;
    [SerializeField] private float widthTolerance = 1.8f;
    [SerializeField] private float heightTolerance = 0.45f;

    [Header("Triangular Surface")]
    [SerializeField] private bool triangularTopology;
    [SerializeField] private Transform triangleA;
    [SerializeField] private Transform triangleB;
    [SerializeField] private Transform triangleC;
    [SerializeField] private List<Transform> triangularNodes = new List<Transform>();
    [SerializeField] private int triangularResolution;

    private Transform topLeft;
    private Transform topRight;
    private Transform middleLeft;
    private Transform middleRight;
    private Transform bottomLeft;
    private Transform bottomRight;
    private Transform centerBrace;
    private Transform[,] surfaceNodes = new Transform[0, 0];
    private int surfaceColumnCount;
    private int surfaceRowCount;
    private readonly List<SurfaceSegment> surfaceSegments = new List<SurfaceSegment>();

    private struct SurfaceSegment
    {
        public Transform a;
        public Transform b;
    }

    public Transform TopLeft => topLeft;
    public Transform TopRight => topRight;
    public Transform MiddleLeft => middleLeft;
    public Transform MiddleRight => middleRight;
    public Transform BottomLeft => bottomLeft;
    public Transform BottomRight => bottomRight;
    public Transform CenterBrace => centerBrace;
    public float TargetNetWidth => targetNetWidth;
    public float TargetNetHeight => targetNetHeight;
    public float WidthTolerance => widthTolerance;
    public float HeightTolerance => heightTolerance;
    public int SurfaceColumnCount => surfaceColumnCount;
    public int SurfaceRowCount => surfaceRowCount;
    public bool HasSurfaceGrid => surfaceColumnCount >= 2 && surfaceRowCount >= 2;

    public void SetTargetGeometry(float width, float height)
    {
        targetNetWidth = Mathf.Max(0.1f, width);
        targetNetHeight = Mathf.Max(0.1f, height);
    }

    public void SetGeometryTolerances(float width, float height)
    {
        widthTolerance = Mathf.Max(0.1f, width);
        heightTolerance = Mathf.Max(0.1f, height);
    }

    public void ConfigureTriangularSurface(
        Transform apex,
        Transform baseLeft,
        Transform baseRight,
        List<Transform> nodes,
        int subdivisions)
    {
        triangularTopology = true;
        triangleA = apex;
        triangleB = baseLeft;
        triangleC = baseRight;
        triangularNodes = nodes != null ? new List<Transform>(nodes) : new List<Transform>();
        triangularResolution = Mathf.Max(1, subdivisions);
        surfaceColumnCount = triangularResolution + 1;
        surfaceRowCount = triangularResolution + 1;
        surfaceSegments.Clear();
    }

    private void Awake()
    {
        ResolveHierarchy();
    }

    private void OnValidate()
    {
        targetNetWidth = Mathf.Max(0.1f, targetNetWidth);
        targetNetHeight = Mathf.Max(0.1f, targetNetHeight);
        widthTolerance = Mathf.Max(0.1f, widthTolerance);
        heightTolerance = Mathf.Max(0.1f, heightTolerance);
        ResolveHierarchy();
    }

    public void ResolveHierarchy()
    {
        if (triangularTopology)
        {
            surfaceNodes = new Transform[0, 0];
            surfaceColumnCount = triangularResolution + 1;
            surfaceRowCount = triangularResolution + 1;
            surfaceSegments.Clear();
            return;
        }

        List<Transform> indexedLayers = GetIndexedNamedChildren("Layer");
        Transform topLayer = indexedLayers.Count > 0 ? indexedLayers[0] : transform.Find(topLayerName);
        Transform middleLayer = indexedLayers.Count > 0 ? indexedLayers[indexedLayers.Count / 2] : transform.Find(middleLayerName);
        Transform bottomLayer = indexedLayers.Count > 0 ? indexedLayers[indexedLayers.Count - 1] : transform.Find(bottomLayerName);
        Transform horizontalLayer = transform.Find(horizontalLayerName);

        List<Transform> topSegments = GetIndexedChildren(topLayer, 'L');
        List<Transform> middleSegments = GetIndexedChildren(middleLayer, 'L');
        List<Transform> bottomSegments = GetIndexedChildren(bottomLayer, 'L');
        List<Transform> horizontalMembers = GetIndexedChildren(horizontalLayer, 'H');
        ResolveSurfaceGrid(indexedLayers);

        topLeft = topSegments.Count > 0 ? topSegments[0] : null;
        topRight = topSegments.Count > 0 ? topSegments[topSegments.Count - 1] : null;
        middleLeft = middleSegments.Count > 0 ? middleSegments[0] : null;
        middleRight = middleSegments.Count > 0 ? middleSegments[middleSegments.Count - 1] : null;
        bottomLeft = bottomSegments.Count > 0 ? bottomSegments[0] : null;
        bottomRight = bottomSegments.Count > 0 ? bottomSegments[bottomSegments.Count - 1] : null;
        centerBrace = horizontalMembers.Count > 0 ? horizontalMembers[horizontalMembers.Count / 2] : null;
    }

    private void ResolveSurfaceGrid(List<Transform> indexedLayers)
    {
        surfaceNodes = new Transform[0, 0];
        surfaceColumnCount = 0;
        surfaceRowCount = 0;
        surfaceSegments.Clear();

        if (indexedLayers == null || indexedLayers.Count < 2)
        {
            return;
        }

        List<List<Transform>> rows = new List<List<Transform>>();
        int columnCount = int.MaxValue;
        for (int i = 0; i < indexedLayers.Count; ++i)
        {
            List<Transform> row = GetIndexedChildren(indexedLayers[i], 'L');
            if (row.Count < 2)
            {
                return;
            }

            rows.Add(row);
            columnCount = Mathf.Min(columnCount, row.Count);
        }

        if (columnCount < 2)
        {
            return;
        }

        surfaceColumnCount = columnCount;
        surfaceRowCount = rows.Count;
        surfaceNodes = new Transform[surfaceColumnCount, surfaceRowCount];
        for (int y = 0; y < surfaceRowCount; ++y)
        {
            for (int x = 0; x < surfaceColumnCount; ++x)
            {
                surfaceNodes[x, y] = rows[y][x];
            }
        }

        for (int y = 0; y < surfaceRowCount; ++y)
        {
            for (int x = 0; x < surfaceColumnCount - 1; ++x)
            {
                AddSurfaceSegment(surfaceNodes[x, y], surfaceNodes[x + 1, y]);
            }
        }

        for (int x = 0; x < surfaceColumnCount; ++x)
        {
            for (int y = 0; y < surfaceRowCount - 1; ++y)
            {
                AddSurfaceSegment(surfaceNodes[x, y], surfaceNodes[x, y + 1]);
            }
        }
    }

    private void AddSurfaceSegment(Transform a, Transform b)
    {
        if (a == null || b == null)
        {
            return;
        }

        surfaceSegments.Add(new SurfaceSegment { a = a, b = b });
    }

    public bool IsReady()
    {
        if (triangularTopology)
        {
            return triangleA != null && triangleB != null && triangleC != null;
        }

        return topLeft != null && topRight != null && bottomLeft != null && bottomRight != null;
    }

    /// <summary>Returns the current three anchor positions for a triangular net.</summary>
    public bool TryGetTriangleVertices(out Vector3 a, out Vector3 b, out Vector3 c)
    {
        a = Vector3.zero;
        b = Vector3.zero;
        c = Vector3.zero;
        if (!triangularTopology || !IsReady())
        {
            return false;
        }

        a = triangleA.position;
        b = triangleB.position;
        c = triangleC.position;
        return Vector3.Cross(b - a, c - a).sqrMagnitude > 1e-10f;
    }

    /// <summary>
    /// Evaluates containment against the deformed triangle anchor plane. The
    /// barycentric values are signed; a negative component means outside.
    /// Edge margin is measured in the projected triangle plane.
    /// </summary>
    public bool TryGetTriangularSurfaceMetrics(
        Vector3 worldPoint,
        out Vector3 projectedPoint,
        out float surfaceDistance,
        out Vector3 barycentric,
        out float minimumEdgeMargin)
    {
        return TryGetTriangularSurfaceMetrics(
            worldPoint,
            out projectedPoint,
            out surfaceDistance,
            out _,
            out barycentric,
            out minimumEdgeMargin);
    }

    /// <summary>
    /// Same as the legacy triangular metric query, while also returning the
    /// signed point-to-plane distance. The sign is defined by the stable
    /// configured triangle anchor order A -> B -> C.
    /// </summary>
    public bool TryGetTriangularSurfaceMetrics(
        Vector3 worldPoint,
        out Vector3 projectedPoint,
        out float surfaceDistance,
        out float signedSurfaceDistance,
        out Vector3 barycentric,
        out float minimumEdgeMargin)
    {
        projectedPoint = transform.position;
        surfaceDistance = 0f;
        signedSurfaceDistance = 0f;
        barycentric = Vector3.zero;
        minimumEdgeMargin = 0f;
        if (!TryGetTriangleVertices(out Vector3 a, out Vector3 b, out Vector3 c))
        {
            return false;
        }

        Vector3 normal = Vector3.Cross(b - a, c - a);
        float twiceArea = normal.magnitude;
        if (twiceArea < 1e-6f)
        {
            return false;
        }
        normal /= twiceArea;
        float signedDistance = Vector3.Dot(worldPoint - a, normal);
        projectedPoint = worldPoint - signedDistance * normal;
        surfaceDistance = Mathf.Abs(signedDistance);
        signedSurfaceDistance = signedDistance;

        Vector3 v0 = b - a;
        Vector3 v1 = c - a;
        Vector3 v2 = projectedPoint - a;
        float d00 = Vector3.Dot(v0, v0);
        float d01 = Vector3.Dot(v0, v1);
        float d11 = Vector3.Dot(v1, v1);
        float d20 = Vector3.Dot(v2, v0);
        float d21 = Vector3.Dot(v2, v1);
        float denominator = d00 * d11 - d01 * d01;
        if (Mathf.Abs(denominator) < 1e-8f)
        {
            return false;
        }

        float baryB = (d11 * d20 - d01 * d21) / denominator;
        float baryC = (d00 * d21 - d01 * d20) / denominator;
        float baryA = 1f - baryB - baryC;
        barycentric = new Vector3(baryA, baryB, baryC);

        float altitudeA = twiceArea / Vector3.Distance(b, c);
        float altitudeB = twiceArea / Vector3.Distance(c, a);
        float altitudeC = twiceArea / Vector3.Distance(a, b);
        minimumEdgeMargin = Mathf.Min(baryA * altitudeA, baryB * altitudeB, baryC * altitudeC);
        return true;
    }

    public Vector3 GetNetCenter()
    {
        if (!IsReady())
        {
            return transform.position;
        }

        return triangularTopology
            ? (triangleA.position + triangleB.position + triangleC.position) / 3f
            : 0.25f * (topLeft.position + topRight.position + bottomLeft.position + bottomRight.position);
    }

    public float GetNetWidth()
    {
        if (!IsReady())
        {
            return 0f;
        }

        if (triangularTopology)
        {
            return Mathf.Max(
                Vector3.Distance(triangleA.position, triangleB.position),
                Vector3.Distance(triangleB.position, triangleC.position),
                Vector3.Distance(triangleC.position, triangleA.position));
        }

        float topWidth = Vector3.Distance(topLeft.position, topRight.position);
        float bottomWidth = Vector3.Distance(bottomLeft.position, bottomRight.position);
        return 0.5f * (topWidth + bottomWidth);
    }

    public float GetNetHeight()
    {
        if (!IsReady())
        {
            return 0f;
        }

        if (triangularTopology)
        {
            Vector3 baseDirection = triangleC.position - triangleB.position;
            if (baseDirection.sqrMagnitude < 1e-8f)
            {
                return 0f;
            }

            Vector3 apexOffset = triangleA.position - triangleB.position;
            return (apexOffset - Vector3.Project(apexOffset, baseDirection)).magnitude;
        }

        float leftHeight = Vector3.Distance(topLeft.position, bottomLeft.position);
        float rightHeight = Vector3.Distance(topRight.position, bottomRight.position);
        return 0.5f * (leftHeight + rightHeight);
    }

    public float GetWidthError()
    {
        return Mathf.Abs(GetNetWidth() - targetNetWidth);
    }

    public float GetHeightError()
    {
        return Mathf.Abs(GetNetHeight() - targetNetHeight);
    }

    public Vector3 EvaluateSurfacePoint(float u, float v)
    {
        if (!IsReady())
        {
            return transform.position;
        }

        if (triangularTopology)
        {
            float baseU = Mathf.Clamp01(u);
            float baseV = Mathf.Clamp01(v);
            if (baseU + baseV > 1f)
            {
                float sum = baseU + baseV;
                baseU /= sum;
                baseV /= sum;
            }

            return triangleA.position * (1f - baseU - baseV) + triangleB.position * baseU + triangleC.position * baseV;
        }

        Vector3 top = Vector3.Lerp(topLeft.position, topRight.position, Mathf.Clamp01(u));
        Vector3 bottom = Vector3.Lerp(bottomLeft.position, bottomRight.position, Mathf.Clamp01(u));
        Vector3 point = Vector3.Lerp(top, bottom, Mathf.Clamp01(v));

        if (centerBrace == null)
        {
            return point;
        }

        float braceWeight = 1f - Mathf.Abs(0.5f - u) * 2f;
        braceWeight = Mathf.Clamp01(braceWeight) * 0.18f;
        return Vector3.Lerp(point, centerBrace.position, braceWeight);
    }

    public bool TryGetFrame(out Vector3 origin, out Vector3 right, out Vector3 up, out Vector3 normal, out float width, out float height)
    {
        if (!IsReady())
        {
            origin = transform.position;
            right = Vector3.right;
            up = Vector3.up;
            normal = Vector3.forward;
            width = 0f;
            height = 0f;
            return false;
        }

        origin = GetNetCenter();

        if (triangularTopology)
        {
            Vector3 baseAxis = triangleC.position - triangleB.position;
            Vector3 apexAxis = triangleA.position - 0.5f * (triangleB.position + triangleC.position);
            width = baseAxis.magnitude;
            if (width < 1e-6f)
            {
                right = Vector3.right;
                up = Vector3.up;
                normal = Vector3.forward;
                height = 0f;
                return false;
            }

            right = baseAxis / width;
            normal = Vector3.Cross(right, apexAxis);
            if (normal.sqrMagnitude < 1e-6f)
            {
                up = Vector3.up;
                normal = Vector3.forward;
                height = 0f;
                return false;
            }

            normal.Normalize();
            up = Vector3.Cross(normal, right).normalized;
            height = Mathf.Abs(Vector3.Dot(apexAxis, up));
            return height >= 1e-6f;
        }

        Vector3 topEdge = topRight.position - topLeft.position;
        Vector3 bottomEdge = bottomRight.position - bottomLeft.position;
        Vector3 leftEdge = bottomLeft.position - topLeft.position;
        Vector3 rightEdge = bottomRight.position - topRight.position;

        Vector3 rightAxis = (topEdge + bottomEdge) * 0.5f;
        Vector3 upAxis = (topLeft.position - bottomLeft.position + topRight.position - bottomRight.position) * 0.5f;
        if (upAxis.sqrMagnitude < 1e-6f)
        {
            upAxis = -(leftEdge + rightEdge) * 0.5f;
        }

        width = rightAxis.magnitude;
        height = upAxis.magnitude;
        if (width < 1e-6f || height < 1e-6f)
        {
            right = Vector3.right;
            up = Vector3.up;
            normal = Vector3.forward;
            return false;
        }

        right = rightAxis / width;
        up = upAxis / height;
        normal = Vector3.Cross(right, up);
        if (normal.sqrMagnitude < 1e-6f)
        {
            normal = Vector3.forward;
        }
        else
        {
            normal.Normalize();
        }

        return true;
    }

    public bool TryProjectPoint(Vector3 worldPoint, out float u, out float v, out float signedDistance)
    {
        if (!TryGetFrame(out Vector3 origin, out Vector3 right, out Vector3 up, out Vector3 normal, out float width, out float height))
        {
            u = 0f;
            v = 0f;
            signedDistance = 0f;
            return false;
        }

        Vector3 offset = worldPoint - origin;
        signedDistance = Vector3.Dot(offset, normal);
        u = Mathf.InverseLerp(-width * 0.5f, width * 0.5f, Vector3.Dot(offset, right));
        v = Mathf.InverseLerp(-height * 0.5f, height * 0.5f, Vector3.Dot(offset, up));
        return true;
    }

    public bool TryGetClosestSurfacePoint(Vector3 worldPoint, out Vector3 closestPoint, out float distance)
    {
        if (triangularTopology)
        {
            return TryGetClosestTriangularSurfacePoint(worldPoint, out closestPoint, out distance);
        }

        if (!HasSurfaceGrid || !IsReady())
        {
            ResolveHierarchy();
        }

        closestPoint = transform.position;
        distance = 0f;

        bool foundPoint = false;
        float closestSqrDistance = float.PositiveInfinity;

        if (surfaceColumnCount >= 2 && surfaceRowCount >= 2)
        {
            for (int y = 0; y < surfaceRowCount - 1; ++y)
            {
                for (int x = 0; x < surfaceColumnCount - 1; ++x)
                {
                    Transform nodeTopLeft = surfaceNodes[x, y];
                    Transform nodeTopRight = surfaceNodes[x + 1, y];
                    Transform nodeBottomLeft = surfaceNodes[x, y + 1];
                    Transform nodeBottomRight = surfaceNodes[x + 1, y + 1];

                    if (nodeTopLeft == null || nodeTopRight == null || nodeBottomLeft == null || nodeBottomRight == null)
                    {
                        continue;
                    }

                    AccumulateClosestTrianglePoint(
                        worldPoint,
                        nodeTopLeft.position,
                        nodeTopRight.position,
                        nodeBottomLeft.position,
                        ref closestPoint,
                        ref closestSqrDistance,
                        ref foundPoint);

                    AccumulateClosestTrianglePoint(
                        worldPoint,
                        nodeTopRight.position,
                        nodeBottomRight.position,
                        nodeBottomLeft.position,
                        ref closestPoint,
                        ref closestSqrDistance,
                        ref foundPoint);
                }
            }
        }

        for (int i = 0; i < surfaceSegments.Count; ++i)
        {
            SurfaceSegment segment = surfaceSegments[i];
            if (segment.a == null || segment.b == null)
            {
                continue;
            }

            AccumulateClosestSegmentPoint(
                worldPoint,
                segment.a.position,
                segment.b.position,
                ref closestPoint,
                ref closestSqrDistance,
                ref foundPoint);
        }

        if (!foundPoint && IsReady())
        {
            AccumulateClosestTrianglePoint(
                worldPoint,
                topLeft.position,
                topRight.position,
                bottomLeft.position,
                ref closestPoint,
                ref closestSqrDistance,
                ref foundPoint);

            AccumulateClosestTrianglePoint(
                worldPoint,
                topRight.position,
                bottomRight.position,
                bottomLeft.position,
                ref closestPoint,
                ref closestSqrDistance,
                ref foundPoint);
        }

        if (!foundPoint)
        {
            return false;
        }

        distance = Mathf.Sqrt(closestSqrDistance);
        return true;
    }

    private bool TryGetClosestTriangularSurfacePoint(Vector3 worldPoint, out Vector3 closestPoint, out float distance)
    {
        if (!IsReady() || triangularResolution < 1 || triangularNodes == null ||
            triangularNodes.Count != (triangularResolution + 1) * (triangularResolution + 2) / 2)
        {
            closestPoint = transform.position;
            distance = 0f;
            return false;
        }

        bool foundPoint = false;
        float closestSqrDistance = float.PositiveInfinity;
        closestPoint = transform.position;
        for (int row = 0; row < triangularResolution; row++)
        {
            for (int column = 0; column <= row; column++)
            {
                Transform upper = GetTriangularNode(row, column);
                Transform lowerLeft = GetTriangularNode(row + 1, column);
                Transform lowerRight = GetTriangularNode(row + 1, column + 1);
                if (upper == null || lowerLeft == null || lowerRight == null)
                {
                    continue;
                }

                AccumulateClosestTrianglePoint(
                    worldPoint, upper.position, lowerLeft.position, lowerRight.position,
                    ref closestPoint, ref closestSqrDistance, ref foundPoint);

                if (column < row)
                {
                    Transform nextUpper = GetTriangularNode(row, column + 1);
                    if (nextUpper != null)
                    {
                        AccumulateClosestTrianglePoint(
                            worldPoint, upper.position, lowerRight.position, nextUpper.position,
                            ref closestPoint, ref closestSqrDistance, ref foundPoint);
                    }
                }
            }
        }

        distance = foundPoint ? Mathf.Sqrt(closestSqrDistance) : 0f;
        return foundPoint;
    }

    private Transform GetTriangularNode(int row, int column)
    {
        if (row < 0 || column < 0 || column > row)
        {
            return null;
        }

        int index = row * (row + 1) / 2 + column;
        return index >= 0 && index < triangularNodes.Count ? triangularNodes[index] : null;
    }

    private static void AccumulateClosestSegmentPoint(
        Vector3 point,
        Vector3 a,
        Vector3 b,
        ref Vector3 closestPoint,
        ref float closestSqrDistance,
        ref bool foundPoint)
    {
        Vector3 candidate = ClosestPointOnSegment(point, a, b);
        float sqrDistance = (point - candidate).sqrMagnitude;
        if (!foundPoint || sqrDistance < closestSqrDistance)
        {
            closestPoint = candidate;
            closestSqrDistance = sqrDistance;
            foundPoint = true;
        }
    }

    private static void AccumulateClosestTrianglePoint(
        Vector3 point,
        Vector3 a,
        Vector3 b,
        Vector3 c,
        ref Vector3 closestPoint,
        ref float closestSqrDistance,
        ref bool foundPoint)
    {
        Vector3 candidate = ClosestPointOnTriangle(point, a, b, c);
        float sqrDistance = (point - candidate).sqrMagnitude;
        if (!foundPoint || sqrDistance < closestSqrDistance)
        {
            closestPoint = candidate;
            closestSqrDistance = sqrDistance;
            foundPoint = true;
        }
    }

    private static Vector3 ClosestPointOnTriangle(Vector3 point, Vector3 a, Vector3 b, Vector3 c)
    {
        Vector3 ab = b - a;
        Vector3 ac = c - a;
        if (Vector3.Cross(ab, ac).sqrMagnitude < 1e-10f)
        {
            Vector3 closest = ClosestPointOnSegment(point, a, b);
            float closestSqrDistance = (point - closest).sqrMagnitude;

            Vector3 candidate = ClosestPointOnSegment(point, b, c);
            float candidateSqrDistance = (point - candidate).sqrMagnitude;
            if (candidateSqrDistance < closestSqrDistance)
            {
                closest = candidate;
                closestSqrDistance = candidateSqrDistance;
            }

            candidate = ClosestPointOnSegment(point, c, a);
            candidateSqrDistance = (point - candidate).sqrMagnitude;
            if (candidateSqrDistance < closestSqrDistance)
            {
                closest = candidate;
            }

            return closest;
        }

        Vector3 ap = point - a;

        float d1 = Vector3.Dot(ab, ap);
        float d2 = Vector3.Dot(ac, ap);
        if (d1 <= 0f && d2 <= 0f)
        {
            return a;
        }

        Vector3 bp = point - b;
        float d3 = Vector3.Dot(ab, bp);
        float d4 = Vector3.Dot(ac, bp);
        if (d3 >= 0f && d4 <= d3)
        {
            return b;
        }

        float vc = d1 * d4 - d3 * d2;
        if (vc <= 0f && d1 >= 0f && d3 <= 0f)
        {
            float v = d1 / (d1 - d3);
            return a + ab * v;
        }

        Vector3 cp = point - c;
        float d5 = Vector3.Dot(ab, cp);
        float d6 = Vector3.Dot(ac, cp);
        if (d6 >= 0f && d5 <= d6)
        {
            return c;
        }

        float vb = d5 * d2 - d1 * d6;
        if (vb <= 0f && d2 >= 0f && d6 <= 0f)
        {
            float w = d2 / (d2 - d6);
            return a + ac * w;
        }

        float va = d3 * d6 - d5 * d4;
        if (va <= 0f && d4 - d3 >= 0f && d5 - d6 >= 0f)
        {
            float w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            return b + (c - b) * w;
        }

        float denom = 1f / (va + vb + vc);
        float baryB = vb * denom;
        float baryC = vc * denom;
        return a + ab * baryB + ac * baryC;
    }

    private static Vector3 ClosestPointOnSegment(Vector3 point, Vector3 a, Vector3 b)
    {
        Vector3 ab = b - a;
        float lengthSqr = ab.sqrMagnitude;
        if (lengthSqr < 1e-10f)
        {
            return a;
        }

        float t = Mathf.Clamp01(Vector3.Dot(point - a, ab) / lengthSqr);
        return a + ab * t;
    }

    private static List<Transform> GetIndexedChildren(Transform parent, char expectedPrefix)
    {
        List<Transform> indexedChildren = new List<Transform>();
        if (parent == null)
        {
            return indexedChildren;
        }

        foreach (Transform child in parent)
        {
            if (TryParseIndexedName(child.name, expectedPrefix, out _))
            {
                indexedChildren.Add(child);
            }
        }

        indexedChildren.Sort((a, b) =>
        {
            TryParseIndexedName(a.name, expectedPrefix, out int indexA);
            TryParseIndexedName(b.name, expectedPrefix, out int indexB);
            return indexA.CompareTo(indexB);
        });
        return indexedChildren;
    }

    private static bool TryParseIndexedName(string objectName, char expectedPrefix, out int index)
    {
        index = -1;
        if (string.IsNullOrEmpty(objectName) || objectName[0] != expectedPrefix)
        {
            return false;
        }

        int separator = objectName.LastIndexOf('_');
        string numericSuffix = separator >= 0 ? objectName.Substring(separator + 1) : objectName.Substring(1);
        return int.TryParse(numericSuffix, out index);
    }

    private List<Transform> GetIndexedNamedChildren(string expectedPrefix)
    {
        List<Transform> indexedChildren = new List<Transform>();
        foreach (Transform child in transform)
        {
            if (TryParseIndexedName(child.name, expectedPrefix, out _))
            {
                indexedChildren.Add(child);
            }
        }

        indexedChildren.Sort((a, b) =>
        {
            TryParseIndexedName(a.name, expectedPrefix, out int indexA);
            TryParseIndexedName(b.name, expectedPrefix, out int indexB);
            return indexA.CompareTo(indexB);
        });
        return indexedChildren;
    }

    private static bool TryParseIndexedName(string objectName, string expectedPrefix, out int index)
    {
        index = -1;
        if (string.IsNullOrEmpty(objectName) || !objectName.StartsWith(expectedPrefix, System.StringComparison.Ordinal))
        {
            return false;
        }

        return int.TryParse(objectName.Substring(expectedPrefix.Length), out index);
    }
}
