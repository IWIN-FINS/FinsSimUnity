using System.Collections.Generic;
using UnityEngine;

[DisallowMultipleComponent]
[RequireComponent(typeof(NetSurfaceModel), typeof(MeshFilter), typeof(MeshRenderer))]
public class NetVisualSurface : MonoBehaviour
{
    [Header("Topology")]
    [SerializeField] private int verticalLines = 6;
    [SerializeField] private int horizontalLines = 4;

    [Header("Visual")]
    [SerializeField] private float strandThickness = 0.03f;

    [Header("Density Sync")]
    [SerializeField] private float targetCellWidth = 0.96f;
    [SerializeField] private float targetCellHeight = 0.33333334f;

    private MeshFilter meshFilter;
    private MeshRenderer meshRenderer;
    private Mesh dynamicMesh;
    private NetSurfaceModel model;

    private void Awake()
    {
        EnsureComponents();
        EnsureDynamicMesh();
    }

    private void OnEnable()
    {
        EnsureComponents();
        EnsureDynamicMesh();

        if (meshRenderer != null)
        {
            meshRenderer.enabled = true;
        }
    }

    private void Start()
    {
        ApplyReferenceMaterial();
    }

    private void OnDisable()
    {
        if (dynamicMesh != null)
        {
            dynamicMesh.Clear();
        }

        if (meshRenderer != null)
        {
            meshRenderer.enabled = false;
        }
    }

    private void LateUpdate()
    {
        RebuildMesh();
    }

    private void OnValidate()
    {
        verticalLines = Mathf.Max(2, verticalLines);
        horizontalLines = Mathf.Max(2, horizontalLines);
        strandThickness = Mathf.Max(0.001f, strandThickness);
        targetCellWidth = Mathf.Max(0.01f, targetCellWidth);
        targetCellHeight = Mathf.Max(0.01f, targetCellHeight);

        if (!Application.isPlaying)
        {
            EnsureComponents();
            EnsureDynamicMesh();
            RebuildMesh();
        }
    }

    public void CaptureCellSizeFromCurrentGeometry()
    {
        EnsureComponents();
        if (model == null || !model.IsReady())
        {
            return;
        }

        targetCellWidth = ComputeCellSize(model.GetNetWidth(), verticalLines);
        targetCellHeight = ComputeCellSize(model.GetNetHeight(), horizontalLines);
    }

    public void RecalculateLineDensityFromTargetGeometry()
    {
        EnsureComponents();
        if (model == null)
        {
            return;
        }

        verticalLines = ComputeLineCount(model.TargetNetWidth, targetCellWidth);
        horizontalLines = ComputeLineCount(model.TargetNetHeight, targetCellHeight);
    }

    public void RefreshSurface()
    {
        EnsureComponents();
        EnsureDynamicMesh();
        RebuildMesh();
    }

    public float GetTargetCellWidth()
    {
        return targetCellWidth;
    }

    public float GetTargetCellHeight()
    {
        return targetCellHeight;
    }

    public int GetVerticalLineCount()
    {
        return verticalLines;
    }

    public int GetHorizontalLineCount()
    {
        return horizontalLines;
    }

    private void EnsureComponents()
    {
        if (meshFilter == null)
        {
            meshFilter = GetComponent<MeshFilter>();
        }

        if (meshRenderer == null)
        {
            meshRenderer = GetComponent<MeshRenderer>();
        }

        if (model == null)
        {
            model = GetComponent<NetSurfaceModel>();
        }
    }

    private void EnsureDynamicMesh()
    {
        if (meshFilter == null)
        {
            return;
        }

        if (dynamicMesh == null)
        {
            dynamicMesh = new Mesh { name = "NetVisualSurfaceMesh" };
            dynamicMesh.MarkDynamic();
        }

        if (meshFilter.sharedMesh != dynamicMesh)
        {
            meshFilter.sharedMesh = dynamicMesh;
        }
    }

    private void RebuildMesh()
    {
        if (dynamicMesh == null)
        {
            return;
        }

        if (model == null || !model.IsReady())
        {
            dynamicMesh.Clear();
            return;
        }

        List<Vector3> vertices = new List<Vector3>();
        List<int> triangles = new List<int>();

        int safeVerticalLines = Mathf.Max(2, verticalLines);
        int safeHorizontalLines = Mathf.Max(2, horizontalLines);
        int segmentsPerStrip = 10;

        for (int i = 0; i < safeVerticalLines; i++)
        {
            float u = (float)i / (safeVerticalLines - 1);
            BuildStrip(vertices, triangles, u, true, segmentsPerStrip);
        }

        for (int i = 0; i < safeHorizontalLines; i++)
        {
            float v = (float)i / (safeHorizontalLines - 1);
            BuildStrip(vertices, triangles, v, false, segmentsPerStrip);
        }

        dynamicMesh.Clear();
        dynamicMesh.SetVertices(vertices);
        dynamicMesh.SetTriangles(triangles, 0, true);
        dynamicMesh.RecalculateNormals();
        dynamicMesh.RecalculateBounds();
    }

    private void ApplyReferenceMaterial()
    {
        if (meshRenderer == null || model == null)
        {
            return;
        }

        MeshRenderer referenceRenderer = FindReferenceRenderer();
        if (referenceRenderer == null)
        {
            return;
        }

        meshRenderer.sharedMaterials = referenceRenderer.sharedMaterials;
        meshRenderer.shadowCastingMode = referenceRenderer.shadowCastingMode;
        meshRenderer.receiveShadows = referenceRenderer.receiveShadows;
        meshRenderer.lightProbeUsage = referenceRenderer.lightProbeUsage;
        meshRenderer.reflectionProbeUsage = referenceRenderer.reflectionProbeUsage;
    }

    private MeshRenderer FindReferenceRenderer()
    {
        Transform[] candidates =
        {
            model.TopLeft,
            model.TopRight,
            model.BottomLeft,
            model.BottomRight,
            model.CenterBrace
        };

        foreach (Transform candidate in candidates)
        {
            if (candidate == null)
            {
                continue;
            }

            MeshRenderer candidateRenderer = candidate.GetComponent<MeshRenderer>();
            if (candidateRenderer != null && candidateRenderer.sharedMaterials != null && candidateRenderer.sharedMaterials.Length > 0)
            {
                return candidateRenderer;
            }
        }

        return null;
    }

    private void BuildStrip(List<Vector3> vertices, List<int> triangles, float lineValue, bool vertical, int segments)
    {
        Vector3 previousLeft = Vector3.zero;
        Vector3 previousRight = Vector3.zero;
        bool hasPrevious = false;

        for (int i = 0; i <= segments; i++)
        {
            float t = (float)i / segments;
            Vector3 current = vertical
                ? model.EvaluateSurfacePoint(lineValue, t)
                : model.EvaluateSurfacePoint(t, lineValue);
            Vector3 next = vertical
                ? model.EvaluateSurfacePoint(lineValue, Mathf.Min(1f, t + 1f / segments))
                : model.EvaluateSurfacePoint(Mathf.Min(1f, t + 1f / segments), lineValue);

            Vector3 tangent = (next - current).normalized;
            if (tangent.sqrMagnitude < 1e-8f)
            {
                tangent = Vector3.right;
            }

            Vector3 toCenter = model.GetNetCenter() - current;
            Vector3 side = Vector3.Cross(tangent, toCenter).normalized;
            if (side.sqrMagnitude < 1e-8f)
            {
                side = Vector3.up;
            }

            Vector3 offset = side * (strandThickness * 0.5f);
            Vector3 left = transform.InverseTransformPoint(current - offset);
            Vector3 right = transform.InverseTransformPoint(current + offset);

            if (hasPrevious)
            {
                int startIndex = vertices.Count;
                vertices.Add(previousLeft);
                vertices.Add(previousRight);
                vertices.Add(left);
                vertices.Add(right);

                triangles.Add(startIndex + 0);
                triangles.Add(startIndex + 2);
                triangles.Add(startIndex + 1);
                triangles.Add(startIndex + 1);
                triangles.Add(startIndex + 2);
                triangles.Add(startIndex + 3);
            }

            previousLeft = left;
            previousRight = right;
            hasPrevious = true;
        }
    }

    private static float ComputeCellSize(float span, int lineCount)
    {
        return lineCount > 1 ? Mathf.Max(0.01f, span / (lineCount - 1)) : Mathf.Max(0.01f, span);
    }

    private static int ComputeLineCount(float span, float desiredCellSize)
    {
        if (span <= 0.01f || desiredCellSize <= 0.01f)
        {
            return 2;
        }

        return Mathf.Max(2, Mathf.RoundToInt(span / desiredCellSize) + 1);
    }
}
