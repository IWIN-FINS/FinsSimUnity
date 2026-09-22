using System;
using System.Collections.Generic;
using FinsSim.Hydrodynamics;
using UnityEngine;

[DisallowMultipleComponent]
public sealed class PositionDropoutHoldRandomizer : MonoBehaviour, IEpisodeRandomizable
{
    struct DropoutWindow
    {
        public float StartSec;
        public float EndSec;
    }

    [Header("Schedule")]
    public bool enableDropout = true;
    [Min(0)] public int minDropoutsPerEpisode = 1;
    [Min(0)] public int maxDropoutsPerEpisode = 2;
    public Vector2 dropoutDurationSeconds = new Vector2(0.5f, 1.5f);
    public Vector2 dropoutStartTimeSeconds = new Vector2(1f, 24f);
    [Min(0f)] public float minimumGapSeconds = 0.5f;
    public bool randomizePerEpisode = true;

    [Header("Observation Hold")]
    public bool freezeReferencePosition = true;
    public bool freezeReferenceRotation;
    public bool freezeTargetPosition;
    public bool freezeTargetRotation;

    [Header("Debug")]
    public bool logDropoutWindows;
    [SerializeField] int scheduledDropoutCount;
    [SerializeField] float episodeElapsedSeconds;
    [SerializeField] bool dropoutActive;
    [SerializeField] int activeDropoutIndex = -1;

    readonly List<DropoutWindow> dropoutWindows = new List<DropoutWindow>();
    float[] heldThrusterActions = new float[0];
    bool hasHeldThrusterActions;
    bool hasCachedPose;
    Vector3 cachedReferencePosition;
    Quaternion cachedReferenceRotation = Quaternion.identity;
    Vector3 cachedTargetPosition;
    Quaternion cachedTargetRotation = Quaternion.identity;
    int lastScheduledFrame = -1;

    public bool IsDropoutActive => enableDropout && dropoutActive;
    public int ScheduledDropoutCount => scheduledDropoutCount;
    public float EpisodeElapsedSeconds => episodeElapsedSeconds;
    public int ActiveDropoutIndex => activeDropoutIndex;

    public void BeginEpisode()
    {
        if (lastScheduledFrame == Time.frameCount)
        {
            episodeElapsedSeconds = 0f;
            dropoutActive = false;
            activeDropoutIndex = -1;
            hasCachedPose = false;
            hasHeldThrusterActions = false;
            return;
        }

        ScheduleEpisode(() => UnityEngine.Random.value);
    }

    public void RandomizeForEpisode(RandomizationContext context)
    {
        if (!randomizePerEpisode)
        {
            ClearSchedule();
            lastScheduledFrame = Time.frameCount;
            return;
        }

        ScheduleEpisode(context.Value);
    }

    public void Tick(float deltaTime)
    {
        if (!enableDropout)
        {
            dropoutActive = false;
            activeDropoutIndex = -1;
            return;
        }

        if (lastScheduledFrame < 0)
        {
            ScheduleEpisode(() => UnityEngine.Random.value);
        }

        episodeElapsedSeconds += Mathf.Max(0f, deltaTime);
        UpdateActiveState();
    }

    public void CaptureFreshPose(Transform referenceTransform, Transform targetTransform)
    {
        if (referenceTransform == null)
        {
            return;
        }

        cachedReferencePosition = referenceTransform.position;
        cachedReferenceRotation = referenceTransform.rotation;
        if (targetTransform != null)
        {
            cachedTargetPosition = targetTransform.position;
            cachedTargetRotation = targetTransform.rotation;
        }
        else
        {
            cachedTargetPosition = cachedReferencePosition;
            cachedTargetRotation = Quaternion.identity;
        }

        hasCachedPose = true;
    }

    public void ResolveObservedPose(
        Transform referenceTransform,
        Transform targetTransform,
        out Vector3 observedReferencePosition,
        out Quaternion observedReferenceRotation,
        out Vector3 observedTargetPosition,
        out Quaternion observedTargetRotation)
    {
        Vector3 currentReferencePosition = referenceTransform != null ? referenceTransform.position : transform.position;
        Quaternion currentReferenceRotation = referenceTransform != null ? referenceTransform.rotation : transform.rotation;
        Vector3 currentTargetPosition = targetTransform != null ? targetTransform.position : currentReferencePosition;
        Quaternion currentTargetRotation = targetTransform != null ? targetTransform.rotation : Quaternion.identity;

        if (!IsDropoutActive || !hasCachedPose)
        {
            cachedReferencePosition = currentReferencePosition;
            cachedReferenceRotation = currentReferenceRotation;
            cachedTargetPosition = currentTargetPosition;
            cachedTargetRotation = currentTargetRotation;
            hasCachedPose = true;

            observedReferencePosition = currentReferencePosition;
            observedReferenceRotation = currentReferenceRotation;
            observedTargetPosition = currentTargetPosition;
            observedTargetRotation = currentTargetRotation;
            return;
        }

        observedReferencePosition = freezeReferencePosition ? cachedReferencePosition : currentReferencePosition;
        observedReferenceRotation = freezeReferenceRotation ? cachedReferenceRotation : currentReferenceRotation;
        observedTargetPosition = freezeTargetPosition ? cachedTargetPosition : currentTargetPosition;
        observedTargetRotation = freezeTargetRotation ? cachedTargetRotation : currentTargetRotation;
    }

    public void CaptureThrusterActions(float[] normalizedActions)
    {
        if (IsDropoutActive || normalizedActions == null)
        {
            return;
        }

        EnsureHeldActionCapacity(normalizedActions.Length);
        Array.Copy(normalizedActions, heldThrusterActions, normalizedActions.Length);
        hasHeldThrusterActions = true;
    }

    public bool TryWriteHeldThrusterActions(float[] destination)
    {
        if (!IsDropoutActive || !hasHeldThrusterActions || destination == null)
        {
            return false;
        }

        int copyCount = Mathf.Min(destination.Length, heldThrusterActions.Length);
        for (int i = 0; i < destination.Length; i++)
        {
            destination[i] = i < copyCount ? heldThrusterActions[i] : 0f;
        }

        return true;
    }

    void ScheduleEpisode(Func<float> nextUnitSample)
    {
        ClearSchedule();
        lastScheduledFrame = Time.frameCount;

        if (!enableDropout)
        {
            return;
        }

        int lowerCount = Mathf.Max(0, Mathf.Min(minDropoutsPerEpisode, maxDropoutsPerEpisode));
        int upperCount = Mathf.Max(0, Mathf.Max(minDropoutsPerEpisode, maxDropoutsPerEpisode));
        if (upperCount == 0)
        {
            return;
        }

        int dropoutCount = SampleInclusiveInt(lowerCount, upperCount, nextUnitSample());
        float startLower = Mathf.Min(dropoutStartTimeSeconds.x, dropoutStartTimeSeconds.y);
        float startUpper = Mathf.Max(dropoutStartTimeSeconds.x, dropoutStartTimeSeconds.y);
        float durationLower = Mathf.Min(dropoutDurationSeconds.x, dropoutDurationSeconds.y);
        float durationUpper = Mathf.Max(dropoutDurationSeconds.x, dropoutDurationSeconds.y);

        for (int i = 0; i < dropoutCount; i++)
        {
            float start = Mathf.Lerp(startLower, startUpper, Mathf.Clamp01(nextUnitSample()));
            float duration = Mathf.Lerp(durationLower, durationUpper, Mathf.Clamp01(nextUnitSample()));
            duration = Mathf.Max(0f, duration);
            dropoutWindows.Add(new DropoutWindow
            {
                StartSec = start,
                EndSec = start + duration,
            });
        }

        dropoutWindows.Sort((left, right) => left.StartSec.CompareTo(right.StartSec));
        for (int i = 1; i < dropoutWindows.Count; i++)
        {
            DropoutWindow previous = dropoutWindows[i - 1];
            DropoutWindow current = dropoutWindows[i];
            float duration = Mathf.Max(0f, current.EndSec - current.StartSec);
            float minStart = previous.EndSec + Mathf.Max(0f, minimumGapSeconds);
            if (current.StartSec < minStart)
            {
                current.StartSec = minStart;
                current.EndSec = current.StartSec + duration;
                dropoutWindows[i] = current;
            }
        }

        scheduledDropoutCount = dropoutWindows.Count;
        if (logDropoutWindows)
        {
            for (int i = 0; i < dropoutWindows.Count; i++)
            {
                DropoutWindow window = dropoutWindows[i];
                Debug.Log(
                    $"[{nameof(PositionDropoutHoldRandomizer)}] dropout[{i}] start={window.StartSec:F2}s duration={window.EndSec - window.StartSec:F2}s",
                    this);
            }
        }
    }

    void ClearSchedule()
    {
        dropoutWindows.Clear();
        scheduledDropoutCount = 0;
        episodeElapsedSeconds = 0f;
        dropoutActive = false;
        activeDropoutIndex = -1;
        hasCachedPose = false;
        hasHeldThrusterActions = false;
    }

    void UpdateActiveState()
    {
        bool nextActive = false;
        int nextIndex = -1;
        for (int i = 0; i < dropoutWindows.Count; i++)
        {
            DropoutWindow window = dropoutWindows[i];
            if (episodeElapsedSeconds >= window.StartSec && episodeElapsedSeconds < window.EndSec)
            {
                nextActive = true;
                nextIndex = i;
                break;
            }
        }

        if (dropoutActive != nextActive && logDropoutWindows)
        {
            string state = nextActive ? "begin" : "end";
            Debug.Log(
                $"[{nameof(PositionDropoutHoldRandomizer)}] {state} dropout at t={episodeElapsedSeconds:F2}s",
                this);
        }

        dropoutActive = nextActive;
        activeDropoutIndex = nextIndex;
    }

    void EnsureHeldActionCapacity(int length)
    {
        if (heldThrusterActions.Length == length)
        {
            return;
        }

        heldThrusterActions = new float[Mathf.Max(0, length)];
    }

    static int SampleInclusiveInt(int lower, int upper, float unitSample)
    {
        if (upper <= lower)
        {
            return lower;
        }

        float scaled = Mathf.Lerp(lower, upper + 0.999f, Mathf.Clamp01(unitSample));
        return Mathf.Clamp(Mathf.FloorToInt(scaled), lower, upper);
    }
}
