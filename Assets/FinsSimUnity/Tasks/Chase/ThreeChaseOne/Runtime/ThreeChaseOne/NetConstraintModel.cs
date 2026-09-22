using System.Collections.Generic;
using UnityEngine;

[DisallowMultipleComponent]
public class NetConstraintModel : MonoBehaviour
{
    [Header("Hydrodynamic Frame Members")]
    [SerializeField] private string topLayerName = "Layer1";
    [SerializeField] private string middleLayerName = "Layer2";
    [SerializeField] private string bottomLayerName = "Layer3";
    [SerializeField] private string horizontalLayerName = "Horizontal";

    private Transform topLeft;
    private Transform topRight;
    private Transform middleLeft;
    private Transform middleRight;
    private Transform bottomLeft;
    private Transform bottomRight;
    private Transform centerBrace;

    public Transform TopLeft => topLeft;
    public Transform TopRight => topRight;
    public Transform MiddleLeft => middleLeft;
    public Transform MiddleRight => middleRight;
    public Transform BottomLeft => bottomLeft;
    public Transform BottomRight => bottomRight;
    public Transform CenterBrace => centerBrace;

    private void Awake()
    {
        ResolveHierarchy();
    }

    private void OnValidate()
    {
        ResolveHierarchy();
    }

    public void ResolveHierarchy()
    {
        List<Transform> indexedLayers = GetIndexedNamedChildren("Layer");
        Transform topLayer = indexedLayers.Count > 0 ? indexedLayers[0] : transform.Find(topLayerName);
        Transform middleLayer = indexedLayers.Count > 0 ? indexedLayers[indexedLayers.Count / 2] : transform.Find(middleLayerName);
        Transform bottomLayer = indexedLayers.Count > 0 ? indexedLayers[indexedLayers.Count - 1] : transform.Find(bottomLayerName);
        Transform horizontalLayer = transform.Find(horizontalLayerName);

        List<Transform> topSegments = GetIndexedChildren(topLayer, 'L');
        List<Transform> middleSegments = GetIndexedChildren(middleLayer, 'L');
        List<Transform> bottomSegments = GetIndexedChildren(bottomLayer, 'L');
        List<Transform> horizontalMembers = GetIndexedChildren(horizontalLayer, 'H');

        topLeft = topSegments.Count > 0 ? topSegments[0] : null;
        topRight = topSegments.Count > 0 ? topSegments[topSegments.Count - 1] : null;
        middleLeft = middleSegments.Count > 0 ? middleSegments[0] : null;
        middleRight = middleSegments.Count > 0 ? middleSegments[middleSegments.Count - 1] : null;
        bottomLeft = bottomSegments.Count > 0 ? bottomSegments[0] : null;
        bottomRight = bottomSegments.Count > 0 ? bottomSegments[bottomSegments.Count - 1] : null;
        centerBrace = horizontalMembers.Count > 0 ? horizontalMembers[horizontalMembers.Count / 2] : null;
    }

    public bool IsReady()
    {
        return topLeft != null && topRight != null && bottomLeft != null && bottomRight != null;
    }

    public Vector3 GetNetCenter()
    {
        if (!IsReady())
        {
            return transform.position;
        }

        return 0.25f * (topLeft.position + topRight.position + bottomLeft.position + bottomRight.position);
    }

    public float GetNetWidth()
    {
        if (!IsReady())
        {
            return 0f;
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

        float leftHeight = Vector3.Distance(topLeft.position, bottomLeft.position);
        float rightHeight = Vector3.Distance(topRight.position, bottomRight.position);
        return 0.5f * (leftHeight + rightHeight);
    }

    public Vector3 EvaluateSurfacePoint(float u, float v)
    {
        if (!IsReady())
        {
            return transform.position;
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
