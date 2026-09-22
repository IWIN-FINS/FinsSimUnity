using UnityEngine;

[DisallowMultipleComponent]
public sealed class TriNetTarget : MonoBehaviour
{
    [SerializeField] private Rigidbody targetRigidbody;
    [SerializeField] private Collider targetCollider;

    public Rigidbody Rigidbody => targetRigidbody;
    public Collider Collider => targetCollider;

    public void Configure(Rigidbody rigidbody, Collider collider)
    {
        targetRigidbody = rigidbody;
        targetCollider = collider;
    }

    private void Reset()
    {
        targetRigidbody = GetComponent<Rigidbody>();
        targetCollider = GetComponent<Collider>();
    }

    private void Awake()
    {
        if (targetRigidbody == null)
        {
            targetRigidbody = GetComponent<Rigidbody>();
        }
        if (targetCollider == null)
        {
            targetCollider = GetComponent<Collider>();
        }
    }

    public void ResetTo(Vector3 worldPosition, Quaternion worldRotation)
    {
        transform.SetPositionAndRotation(worldPosition, worldRotation);
        if (targetRigidbody != null)
        {
            targetRigidbody.linearVelocity = Vector3.zero;
            targetRigidbody.angularVelocity = Vector3.zero;
            targetRigidbody.WakeUp();
        }
    }

    public Vector3 GetCapturePoint(Vector3 netCenter)
    {
        return targetCollider != null ? targetCollider.ClosestPoint(netCenter) : transform.position;
    }
}
