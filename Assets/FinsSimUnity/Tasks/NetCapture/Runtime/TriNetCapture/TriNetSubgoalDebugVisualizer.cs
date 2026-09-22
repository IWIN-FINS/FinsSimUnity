using System;
using System.Collections.Generic;
using System.Text;
using Unity.MLAgents.SideChannels;
using Unity.MLAgents.Policies;
using UnityEngine;

/// <summary>
/// Receives the Python TriNet transport baseline's formation targets and
/// pre-PID subgoals.
/// This is diagnostic-only: it never changes ML-Agents actions or vehicle state.
/// </summary>
[DisallowMultipleComponent]
public sealed class TriNetSubgoalDebugVisualizer : MonoBehaviour
{
    private const int RovCount = 3;

    private sealed class TriNetSubgoalDebugSideChannel : SideChannel
    {
        private static readonly Guid DebugChannelId = new Guid("d90d4182-49cb-4e1f-8319-43423249c46d");

        public readonly struct Payload
        {
            public Payload(int stepIndex, int areaIndex, Vector3[] sharedTranslation,
                Vector3[] formationCorrection, Vector3[] bodySubgoal, float[] yawErrorDeg, int[] phase)
            {
                StepIndex = stepIndex;
                AreaIndex = areaIndex;
                SharedTranslation = sharedTranslation;
                FormationCorrection = formationCorrection;
                BodySubgoal = bodySubgoal;
                YawErrorDeg = yawErrorDeg;
                Phase = phase;
            }

            public int StepIndex { get; }
            public int AreaIndex { get; }
            public Vector3[] SharedTranslation { get; }
            public Vector3[] FormationCorrection { get; }
            public Vector3[] BodySubgoal { get; }
            public float[] YawErrorDeg { get; }
            public int[] Phase { get; }
        }

        public event Action<Payload> MessageReceived;

        public TriNetSubgoalDebugSideChannel()
        {
            ChannelId = DebugChannelId;
        }

        protected override void OnMessageReceived(IncomingMessage msg)
        {
            int stepIndex = msg.ReadInt32();
            int areaIndex = msg.ReadInt32();
            Vector3[] sharedTranslation = new Vector3[RovCount];
            Vector3[] formationCorrection = new Vector3[RovCount];
            Vector3[] bodySubgoal = new Vector3[RovCount];
            float[] yawErrorDeg = new float[RovCount];
            int[] phase = new int[RovCount];

            for (int index = 0; index < RovCount; index++)
            {
                sharedTranslation[index] = ReadVector3(msg.ReadFloatList());
                formationCorrection[index] = ReadVector3(msg.ReadFloatList());
                bodySubgoal[index] = ReadVector3(msg.ReadFloatList());
                yawErrorDeg[index] = msg.ReadFloat32();
                phase[index] = msg.ReadInt32();
            }

            MessageReceived?.Invoke(new Payload(
                stepIndex, areaIndex, sharedTranslation, formationCorrection, bodySubgoal, yawErrorDeg, phase));
        }

        private static Vector3 ReadVector3(IList<float> values)
        {
            return values != null && values.Count >= 3
                ? new Vector3(values[0], values[1], values[2])
                : Vector3.zero;
        }
    }

    [Header("Source")]
    [Tooltip("The replicated TrainingArea index to display. The foreground editor has area 0.")]
    [SerializeField, Min(0)] private int debugAreaIndex;

    [Header("Console")]
    [SerializeField] private bool logReceivedSubgoals = true;
    [SerializeField, Min(1)] private int logEveryMessages = 20;

    [Header("Scene Gizmos")]
    [SerializeField] private bool drawSubgoalGizmos = true;
    [SerializeField, Min(0.01f)] private float gizmoMarkerRadius = 0.08f;
    [SerializeField] private Color subgoalColor = new Color(1f, 0.78f, 0.08f, 0.95f);
    [Tooltip("Cyan: full Target-adjacent formation vertex. Yellow: clipped PID look-ahead.")]
    [SerializeField] private Color translationColor = new Color(0.1f, 0.9f, 1f, 0.8f);

    [Header("Runtime State")]
    [SerializeField] private bool hasReceivedSubgoal;
    [SerializeField] private int lastStepIndex = -1;
    [SerializeField] private int receivedMessageCount;
    [SerializeField] private Vector3[] lastSharedTranslation = new Vector3[RovCount];
    [SerializeField] private Vector3[] lastFormationCorrection = new Vector3[RovCount];
    [SerializeField] private Vector3[] lastBodySubgoal = new Vector3[RovCount];
    [SerializeField] private float[] lastYawErrorDeg = new float[RovCount];
    [SerializeField] private int[] lastPhase = new int[RovCount];
    [SerializeField] private Vector3[] lastWorldFormationWaypoint = new Vector3[RovCount];
    [SerializeField] private Vector3[] lastWorldSubgoal = new Vector3[RovCount];

    private static TriNetSubgoalDebugVisualizer activeInstance;

    private readonly TriNetCaptureAgent[] chasers = new TriNetCaptureAgent[RovCount];
    private readonly StringBuilder logBuilder = new StringBuilder(512);
    private TriNetSubgoalDebugSideChannel sideChannel;
    private bool hasStarted;

    private void Awake()
    {
        ResolveChasers();
        EnsureRuntimeArrays();
    }

    private void OnEnable()
    {
        if (activeInstance != null && activeInstance != this)
        {
            // Training-area clones share one process-side channel. The original
            // area hosts the single receiver and displays debugAreaIndex.
            enabled = false;
            return;
        }

        activeInstance = this;
        ResolveChasers();
        if (hasStarted)
        {
            RegisterSideChannel();
        }
    }

    private void Start()
    {
        hasStarted = true;
        RegisterSideChannel();
    }

    private void OnDisable()
    {
        if (sideChannel != null)
        {
            sideChannel.MessageReceived -= HandlePayload;
            SideChannelManager.UnregisterSideChannel(sideChannel);
            sideChannel = null;
        }

        if (activeInstance == this)
        {
            activeInstance = null;
        }
    }

    private void OnDrawGizmos()
    {
        if (!drawSubgoalGizmos || !hasReceivedSubgoal)
        {
            return;
        }

        ResolveChasers();
        for (int index = 0; index < RovCount; index++)
        {
            TriNetCaptureAgent chaser = chasers[index];
            if (chaser == null)
            {
                continue;
            }

            Transform reference = chaser.PoseTransform;
            Gizmos.color = translationColor;
            Gizmos.DrawLine(reference.position, lastWorldFormationWaypoint[index]);
            Gizmos.DrawWireSphere(lastWorldFormationWaypoint[index], gizmoMarkerRadius * 1.35f);
            Gizmos.color = subgoalColor;
            Gizmos.DrawLine(reference.position, lastWorldSubgoal[index]);
            Gizmos.DrawSphere(lastWorldSubgoal[index], gizmoMarkerRadius);
            Gizmos.DrawWireSphere(lastWorldSubgoal[index], gizmoMarkerRadius * 1.35f);
        }
    }

    private void RegisterSideChannel()
    {
        if (sideChannel != null)
        {
            return;
        }

        sideChannel = new TriNetSubgoalDebugSideChannel();
        sideChannel.MessageReceived += HandlePayload;
        SideChannelManager.RegisterSideChannel(sideChannel);
    }

    private void HandlePayload(TriNetSubgoalDebugSideChannel.Payload payload)
    {
        if (payload.AreaIndex != debugAreaIndex)
        {
            return;
        }

        ResolveChasers();
        hasReceivedSubgoal = true;
        lastStepIndex = payload.StepIndex;
        receivedMessageCount++;
        for (int index = 0; index < RovCount; index++)
        {
            lastSharedTranslation[index] = payload.SharedTranslation[index];
            lastFormationCorrection[index] = payload.FormationCorrection[index];
            lastBodySubgoal[index] = payload.BodySubgoal[index];
            lastYawErrorDeg[index] = payload.YawErrorDeg[index];
            lastPhase[index] = payload.Phase[index];

            TriNetCaptureAgent chaser = chasers[index];
            Transform reference = chaser != null ? chaser.PoseTransform : transform;
            lastWorldFormationWaypoint[index] = reference.position + BodyDeltaToWorld(reference, lastSharedTranslation[index]);
            lastWorldSubgoal[index] = reference.position + BodyDeltaToWorld(reference, lastBodySubgoal[index]);
        }

        if (logReceivedSubgoals && receivedMessageCount % Mathf.Max(1, logEveryMessages) == 0)
        {
            LogPayload();
        }
    }

    private void ResolveChasers()
    {
        TriNetCaptureAgent[] discovered = GetComponentsInChildren<TriNetCaptureAgent>(true);
        Array.Sort(discovered, (left, right) => GetFinsRovOrder(left).CompareTo(GetFinsRovOrder(right)));
        for (int index = 0; index < RovCount; index++)
        {
            chasers[index] = index < discovered.Length ? discovered[index] : null;
        }
    }

    private static int GetFinsRovOrder(TriNetCaptureAgent chaser)
    {
        BehaviorParameters behaviour = chaser != null ? chaser.GetComponent<BehaviorParameters>() : null;
        string name = behaviour != null ? behaviour.BehaviorName : chaser != null ? chaser.name : string.Empty;
        if (name.Contains("FinsROV_Fossen_Left")) return 0;
        if (name.Contains("FinsROV_Fossen_Top")) return 1;
        if (name.Contains("FinsROV_Fossen_Right")) return 2;
        return 3;
    }

    private void EnsureRuntimeArrays()
    {
        if (lastSharedTranslation == null || lastSharedTranslation.Length != RovCount) lastSharedTranslation = new Vector3[RovCount];
        if (lastFormationCorrection == null || lastFormationCorrection.Length != RovCount) lastFormationCorrection = new Vector3[RovCount];
        if (lastBodySubgoal == null || lastBodySubgoal.Length != RovCount) lastBodySubgoal = new Vector3[RovCount];
        if (lastYawErrorDeg == null || lastYawErrorDeg.Length != RovCount) lastYawErrorDeg = new float[RovCount];
        if (lastPhase == null || lastPhase.Length != RovCount) lastPhase = new int[RovCount];
        if (lastWorldFormationWaypoint == null || lastWorldFormationWaypoint.Length != RovCount) lastWorldFormationWaypoint = new Vector3[RovCount];
        if (lastWorldSubgoal == null || lastWorldSubgoal.Length != RovCount) lastWorldSubgoal = new Vector3[RovCount];
    }

    private void LogPayload()
    {
        logBuilder.Clear();
        logBuilder.Append("[TriNetSubgoalDebug] step=").Append(lastStepIndex)
            .Append(" area=").Append(debugAreaIndex);
        for (int index = 0; index < RovCount; index++)
        {
            string label = chasers[index] != null ? chasers[index].name : $"ROV{index}";
            logBuilder.Append(" | ").Append(label)
                .Append(" phase=").Append(PhaseName(lastPhase[index]))
                .Append(" formation_waypoint=").Append(lastSharedTranslation[index].ToString("F3"))
                .Append(" formation=").Append(lastFormationCorrection[index].ToString("F3"))
                .Append(" body_subgoal=").Append(lastBodySubgoal[index].ToString("F3"))
                .Append(" yaw_deg=").Append(lastYawErrorDeg[index].ToString("F1"));
        }
        Debug.Log(logBuilder.ToString(), this);
    }

    private static Vector3 BodyDeltaToWorld(Transform reference, Vector3 bodyDelta)
    {
        return reference.right * bodyDelta.x + reference.up * bodyDelta.y + reference.forward * bodyDelta.z;
    }

    private static string PhaseName(int phase)
    {
        return phase switch
        {
            0 => "FixedInitialFormation",
            1 => "FormSymmetric",
            2 => "RaiseNet",
            3 => "TowToGoal",
            4 => "Hold",
            _ => $"Unknown({phase})",
        };
    }
}
