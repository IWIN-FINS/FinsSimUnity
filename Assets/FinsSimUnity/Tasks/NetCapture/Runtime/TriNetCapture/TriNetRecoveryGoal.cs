using UnityEngine;

[DisallowMultipleComponent]
public sealed class TriNetRecoveryGoal : MonoBehaviour
{
    [SerializeField] private BoxCollider recoveryVolume;
    [Tooltip("Vertical offset from the recovery volume's submerged lower face used for transport guidance.")]
    [SerializeField, Min(0f)] private float guidanceBottomClearance = 0.01f;
    [SerializeField] private Color gizmoFillColor = new Color(0.10f, 0.85f, 0.75f, 0.18f);
    [SerializeField] private Color gizmoOutlineColor = new Color(0.05f, 1.00f, 0.85f, 0.95f);

    public BoxCollider RecoveryVolume => recoveryVolume;
    public Vector3 Anchor => transform.position;
    /// <summary>
    /// Submerged point used by controllers and distance metrics. Anchor remains
    /// at the waterline so the Goal root is easy to edit in the scene.
    /// </summary>
    public Vector3 GuidancePoint
    {
        get
        {
            if (recoveryVolume == null)
            {
                return Anchor;
            }

            Bounds bounds = recoveryVolume.bounds;
            return new Vector3(
                bounds.center.x,
                Mathf.Min(bounds.max.y, bounds.min.y + guidanceBottomClearance),
                bounds.center.z);
        }
    }

    public void Configure(BoxCollider volume)
    {
        recoveryVolume = volume;
        OnValidate();
    }

    private void Reset()
    {
        recoveryVolume = GetComponent<BoxCollider>();
    }

    private void OnValidate()
    {
        if (recoveryVolume == null)
        {
            recoveryVolume = GetComponent<BoxCollider>();
        }
        if (recoveryVolume != null)
        {
            recoveryVolume.isTrigger = true;
        }
        guidanceBottomClearance = Mathf.Max(0f, guidanceBottomClearance);
    }

    public bool Contains(Vector3 worldPoint)
    {
        if (recoveryVolume == null)
        {
            return false;
        }
        Vector3 local = recoveryVolume.transform.InverseTransformPoint(worldPoint) - recoveryVolume.center;
        Vector3 half = recoveryVolume.size * 0.5f;
        return Mathf.Abs(local.x) <= half.x && Mathf.Abs(local.y) <= half.y && Mathf.Abs(local.z) <= half.z;
    }

    private void OnDrawGizmos()
    {
        if (recoveryVolume == null)
        {
            return;
        }

        Matrix4x4 previousMatrix = Gizmos.matrix;
        Color previousColor = Gizmos.color;
        Gizmos.matrix = recoveryVolume.transform.localToWorldMatrix;
        Gizmos.color = gizmoFillColor;
        Gizmos.DrawCube(recoveryVolume.center, recoveryVolume.size);
        Gizmos.color = gizmoOutlineColor;
        Gizmos.DrawWireCube(recoveryVolume.center, recoveryVolume.size);
        Gizmos.matrix = previousMatrix;
        Gizmos.color = previousColor;

        Gizmos.color = gizmoOutlineColor;
        Gizmos.DrawSphere(Anchor, 0.025f);
        Gizmos.color = Color.yellow;
        Gizmos.DrawSphere(GuidancePoint, 0.035f);
        Gizmos.color = previousColor;
    }
}
