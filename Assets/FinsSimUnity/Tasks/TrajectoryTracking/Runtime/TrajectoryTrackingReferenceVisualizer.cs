using UnityEngine;

/// <summary>Editor-visible, collider-free preview of a T2 reference trajectory.</summary>
[DisallowMultipleComponent]
public sealed class TrajectoryTrackingReferenceVisualizer : MonoBehaviour
{
    [SerializeField] TrajectoryTrackingAgent agent;
    [SerializeField, Min(8)] int sampleCount = 96;
    [SerializeField] Color lineColor = new Color(0.1f, 0.8f, 1f, 0.8f);
    LineRenderer line;

    public void Bind(TrajectoryTrackingAgent value) => agent = value;

    void Awake()
    {
        line = GetComponent<LineRenderer>();
        if (line == null) line = gameObject.AddComponent<LineRenderer>();
        line.useWorldSpace = true;
        line.widthMultiplier = 0.015f;
        line.startColor = lineColor;
        line.endColor = lineColor;
        line.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        line.receiveShadows = false;
    }

    void LateUpdate()
    {
        if (agent == null || line == null) return;
        int count = Mathf.Max(sampleCount, 8);
        line.positionCount = count;
        float duration = agent.EpisodeDurationSec;
        for (int index = 0; index < count; index++)
        {
            agent.EvaluateReference(duration * index / (count - 1), out Vector3 point, out _);
            line.SetPosition(index, point);
        }
    }
}
