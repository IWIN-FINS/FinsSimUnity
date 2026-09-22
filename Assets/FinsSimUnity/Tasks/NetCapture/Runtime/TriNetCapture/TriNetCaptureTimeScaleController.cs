using Unity.MLAgents;
using UnityEngine;

/// <summary>
/// Keeps TriNetCapture's requested ML-Agents time scale authoritative.
///
/// MARUS simulation-clock components may set <see cref="Time.timeScale"/>
/// after ML-Agents' EngineConfigurationChannel has applied it. The Python
/// launcher mirrors its requested value into this environment parameter so
/// the scene has one final, explicit owner of the runtime rate.
/// </summary>
[DefaultExecutionOrder(10000)]
public sealed class TriNetCaptureTimeScaleController : MonoBehaviour
{
    public const string RuntimeTimeScaleParameter = "finssim_runtime_time_scale";

    [SerializeField, Min(0.001f)] private float editorFallbackTimeScale = 1f;

    private float requestedTimeScale;

    private void Awake()
    {
        requestedTimeScale = Mathf.Max(editorFallbackTimeScale, 0.001f);
    }

    private void OnEnable()
    {
        Academy academy = Academy.Instance;
        if (academy == null)
        {
            return;
        }

        ApplyTimeScale(academy.EnvironmentParameters.GetWithDefault(
            RuntimeTimeScaleParameter,
            requestedTimeScale));
        academy.EnvironmentParameters.RegisterCallback(
            RuntimeTimeScaleParameter,
            ApplyTimeScale);
    }

    // Run after simulation-clock writers in FixedUpdate, and also keep the
    // editor-visible rate correct when a third-party component writes it in
    // Update/LateUpdate.
    private void FixedUpdate() => Time.timeScale = requestedTimeScale;

    private void LateUpdate() => Time.timeScale = requestedTimeScale;

    private void OnValidate()
    {
        editorFallbackTimeScale = Mathf.Max(editorFallbackTimeScale, 0.001f);
        requestedTimeScale = Mathf.Max(requestedTimeScale, 0.001f);
    }

    private void ApplyTimeScale(float value)
    {
        requestedTimeScale = Mathf.Max(value, 0.001f);
        Time.timeScale = requestedTimeScale;
    }
}
