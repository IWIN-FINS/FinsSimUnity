using UnityEngine;

[DisallowMultipleComponent]
[RequireComponent(typeof(NetConstraintModel))]
public class NetHydrodynamicProxy : MonoBehaviour
{
    [SerializeField] private float tangentialDragCoefficient = 12f;
    [SerializeField] private float normalDragCoefficient = 20f;
    [SerializeField] private float maxForcePerSample = 35f;
    [SerializeField] private int widthSamples = 4;
    [SerializeField] private int heightSamples = 3;
    [SerializeField] private Vector3 waterVelocity = Vector3.zero;

    private NetConstraintModel model;
    private Rigidbody topLeftRb;
    private Rigidbody topRightRb;
    private Rigidbody bottomLeftRb;
    private Rigidbody bottomRightRb;

    private void Awake()
    {
        model = GetComponent<NetConstraintModel>();
    }

    private void Start()
    {
        if (model != null)
        {
            model.ResolveHierarchy();
        }
        CacheRigidbodies();
    }

    private void CacheRigidbodies()
    {
        topLeftRb = model != null && model.TopLeft != null ? model.TopLeft.GetComponent<Rigidbody>() : null;
        topRightRb = model != null && model.TopRight != null ? model.TopRight.GetComponent<Rigidbody>() : null;
        bottomLeftRb = model != null && model.BottomLeft != null ? model.BottomLeft.GetComponent<Rigidbody>() : null;
        bottomRightRb = model != null && model.BottomRight != null ? model.BottomRight.GetComponent<Rigidbody>() : null;
    }

    private void FixedUpdate()
    {
        long finsSimProfileStart = FinsSimRuntimeProfiler.Begin();
        UnityEngine.Profiling.Profiler.BeginSample("FinsSim.NetHydrodynamicProxy.FixedUpdate");
        try
        {
            if ((topLeftRb == null || topRightRb == null || bottomLeftRb == null || bottomRightRb == null) && model != null)
            {
                CacheRigidbodies();
            }

            if (model == null || !model.TryGetFrame(out _, out _, out _, out Vector3 normal, out float width, out float height))
            {
                return;
            }

            int safeWidthSamples = Mathf.Max(2, widthSamples);
            int safeHeightSamples = Mathf.Max(2, heightSamples);
            float areaPerSample = Mathf.Max(0.01f, width * height / ((safeWidthSamples - 1) * (safeHeightSamples - 1)));

            for (int y = 0; y < safeHeightSamples; y++)
            {
                float v = safeHeightSamples == 1 ? 0.5f : (float)y / (safeHeightSamples - 1);
                for (int x = 0; x < safeWidthSamples; x++)
                {
                    float u = safeWidthSamples == 1 ? 0.5f : (float)x / (safeWidthSamples - 1);
                    ApplySampleForce(u, v, normal, areaPerSample);
                }
            }
        }
        finally
        {
            UnityEngine.Profiling.Profiler.EndSample();
            FinsSimRuntimeProfiler.End("FinsSim.NetHydrodynamicProxy.FixedUpdate", finsSimProfileStart);
        }
    }

    private void ApplySampleForce(float u, float v, Vector3 normal, float areaPerSample)
    {
        if (topLeftRb == null || topRightRb == null || bottomLeftRb == null || bottomRightRb == null)
        {
            return;
        }

        float wTopLeft = (1f - u) * (1f - v);
        float wTopRight = u * (1f - v);
        float wBottomLeft = (1f - u) * v;
        float wBottomRight = u * v;

        Vector3 topLeftPos = model.TopLeft.position;
        Vector3 topRightPos = model.TopRight.position;
        Vector3 bottomLeftPos = model.BottomLeft.position;
        Vector3 bottomRightPos = model.BottomRight.position;
        Vector3 samplePosition = model.EvaluateSurfacePoint(u, v);

        Vector3 sampleVelocity =
            topLeftRb.GetPointVelocity(topLeftPos) * wTopLeft +
            topRightRb.GetPointVelocity(topRightPos) * wTopRight +
            bottomLeftRb.GetPointVelocity(bottomLeftPos) * wBottomLeft +
            bottomRightRb.GetPointVelocity(bottomRightPos) * wBottomRight;

        Vector3 relativeVelocity = sampleVelocity - waterVelocity;
        float normalSpeed = Vector3.Dot(relativeVelocity, normal);
        Vector3 normalVelocity = normal * normalSpeed;
        Vector3 tangentVelocity = relativeVelocity - normalVelocity;

        Vector3 force =
            -tangentVelocity * tangentVelocity.magnitude * tangentialDragCoefficient * areaPerSample
            - normalVelocity * Mathf.Abs(normalSpeed) * normalDragCoefficient * areaPerSample;

        force = Vector3.ClampMagnitude(force, maxForcePerSample);

        AddWeightedForce(topLeftRb, force * wTopLeft, samplePosition);
        AddWeightedForce(topRightRb, force * wTopRight, samplePosition);
        AddWeightedForce(bottomLeftRb, force * wBottomLeft, samplePosition);
        AddWeightedForce(bottomRightRb, force * wBottomRight, samplePosition);
    }

    private static void AddWeightedForce(Rigidbody rb, Vector3 force, Vector3 position)
    {
        if (rb == null || force.sqrMagnitude < 1e-8f)
        {
            return;
        }

        rb.AddForceAtPosition(force, position, ForceMode.Force);
    }
}
