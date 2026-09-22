using System;
using System.Collections.Generic;
using Unity.MLAgents.SideChannels;
using UnityEngine;

[DisallowMultipleComponent]
public sealed class OneChaseOneHighLevelTargetVisualizer : MonoBehaviour
{
    private sealed class HighLevelTargetDebugSideChannel : SideChannel
    {
        private static readonly Guid DebugChannelId = new Guid("6f5d8f67-3d91-4f69-9acb-18d90e2f4a31");

        public readonly struct Payload
        {
            public Payload(int stepIndex, Vector3 localTargetDelta, Vector3 targetWorld, Vector3 currentWorld)
            {
                StepIndex = stepIndex;
                LocalTargetDelta = localTargetDelta;
                TargetWorld = targetWorld;
                CurrentWorld = currentWorld;
            }

            public int StepIndex { get; }
            public Vector3 LocalTargetDelta { get; }
            public Vector3 TargetWorld { get; }
            public Vector3 CurrentWorld { get; }
        }

        public event Action<Payload> MessageReceived;

        public HighLevelTargetDebugSideChannel()
        {
            ChannelId = DebugChannelId;
        }

        protected override void OnMessageReceived(IncomingMessage msg)
        {
            int stepIndex = msg.ReadInt32();
            Vector3 localTargetDelta = ReadVector3(msg.ReadFloatList());
            Vector3 targetWorld = ReadVector3(msg.ReadFloatList());
            Vector3 currentWorld = ReadVector3(msg.ReadFloatList());
            MessageReceived?.Invoke(new Payload(stepIndex, localTargetDelta, targetWorld, currentWorld));
        }

        private static Vector3 ReadVector3(IList<float> values)
        {
            if (values == null || values.Count < 3)
            {
                return Vector3.zero;
            }

            return new Vector3(values[0], values[1], values[2]);
        }
    }

    [Header("References")]
    [SerializeField] private OneChaseOnePoseAgent targetAgent;
    [SerializeField] private Transform referenceTransform;

    [Header("Scene Debug")]
    [SerializeField] private bool showSceneGizmos = true;
    [SerializeField] private bool drawRuntimeDebugLine = true;
    [SerializeField] private float staleTimeoutSeconds = 0.5f;
    [SerializeField] private float targetMarkerRadius = 0.12f;
    [SerializeField] private Color freshTargetColor = new Color(0.10f, 0.95f, 1.0f, 0.95f);
    [SerializeField] private Color staleTargetColor = new Color(0.40f, 0.52f, 0.58f, 0.70f);
    [SerializeField] private bool verboseLogging = false;

    [Header("Runtime State")]
    [SerializeField] private bool hasReceivedTarget;
    [SerializeField] private bool targetLive;
    [SerializeField] private int lastStepIndex = -1;
    [SerializeField] private int receivedMessageCount;
    [SerializeField] private float lastTargetAgeSeconds = -1f;
    [SerializeField] private Vector3 lastTargetWorld = Vector3.zero;
    [SerializeField] private Vector3 lastLocalTargetDelta = Vector3.zero;
    [SerializeField] private Vector3 lastReportedCurrentWorld = Vector3.zero;

    private static OneChaseOneHighLevelTargetVisualizer activeInstance;

    private HighLevelTargetDebugSideChannel sideChannel;
    private float lastReceiveTime = float.NegativeInfinity;
    private float lastAgentPushTime = float.NegativeInfinity;
    private bool lastPushedLiveState;
    private bool hasStarted;

    private void Awake()
    {
        ResolveReferences();
    }

    private void OnEnable()
    {
        if (activeInstance != null && activeInstance != this)
        {
            // Replicated training areas only need one process-local debug receiver.
            enabled = false;
            return;
        }

        activeInstance = this;
        ResolveReferences();
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

    private void RegisterSideChannel()
    {
        if (sideChannel != null)
        {
            return;
        }
        sideChannel = new HighLevelTargetDebugSideChannel();
        sideChannel.MessageReceived += HandlePayload;
        SideChannelManager.RegisterSideChannel(sideChannel);
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

    private void Update()
    {
        ResolveReferences();

        if (!hasReceivedTarget)
        {
            return;
        }

        lastTargetAgeSeconds = Time.unscaledTime - lastReceiveTime;
        targetLive = lastTargetAgeSeconds <= staleTimeoutSeconds;

        if (drawRuntimeDebugLine && targetLive)
        {
            Vector3 currentWorld = ResolveCurrentWorldPosition();
            Debug.DrawLine(currentWorld, lastTargetWorld, freshTargetColor, 0f, false);
        }

        bool shouldPushToAgent =
            targetAgent != null &&
            (targetLive != lastPushedLiveState || Time.unscaledTime - lastAgentPushTime >= 0.15f);
        if (shouldPushToAgent)
        {
            PushDebugStateToAgent();
        }
    }

    private void OnDrawGizmos()
    {
        if (!showSceneGizmos || !hasReceivedTarget)
        {
            return;
        }

        Color color = targetLive ? freshTargetColor : staleTargetColor;
        Gizmos.color = color;
        Gizmos.DrawSphere(lastTargetWorld, targetMarkerRadius);
        Gizmos.DrawWireSphere(lastTargetWorld, targetMarkerRadius * 1.35f);

        Vector3 currentWorld = ResolveCurrentWorldPosition();
        Gizmos.DrawLine(currentWorld, lastTargetWorld);
    }

    private void ResolveReferences()
    {
        if (targetAgent == null)
        {
            targetAgent = GetComponent<OneChaseOnePoseAgent>();
        }

        if (referenceTransform == null)
        {
            if (targetAgent != null && targetAgent.selfTransform != null)
            {
                referenceTransform = targetAgent.selfTransform;
            }
            else
            {
                referenceTransform = transform;
            }
        }
    }

    private void HandlePayload(HighLevelTargetDebugSideChannel.Payload payload)
    {
        ResolveReferences();

        hasReceivedTarget = true;
        targetLive = true;
        lastStepIndex = payload.StepIndex;
        lastLocalTargetDelta = payload.LocalTargetDelta;
        lastReportedCurrentWorld = ResolveCurrentWorldPosition();
        lastTargetWorld = ResolveWorldTargetFromLocalDelta(lastLocalTargetDelta);
        receivedMessageCount += 1;
        lastReceiveTime = Time.unscaledTime;
        lastTargetAgeSeconds = 0f;

        if (verboseLogging)
        {
            Debug.Log(
                $"[{nameof(OneChaseOneHighLevelTargetVisualizer)}] step={lastStepIndex}, "
                + $"target={lastTargetWorld}, local_delta={lastLocalTargetDelta}",
                this);
        }

        PushDebugStateToAgent();
    }

    private void PushDebugStateToAgent()
    {
        if (targetAgent == null)
        {
            return;
        }

        targetAgent.SetHighLevelTargetDebugState(
            targetLive,
            lastStepIndex,
            lastTargetAgeSeconds,
            lastTargetWorld,
            lastLocalTargetDelta,
            lastReportedCurrentWorld);
        lastPushedLiveState = targetLive;
        lastAgentPushTime = Time.unscaledTime;
    }

    private Vector3 ResolveCurrentWorldPosition()
    {
        if (referenceTransform != null)
        {
            return referenceTransform.position;
        }

        if (targetAgent != null && targetAgent.selfTransform != null)
        {
            return targetAgent.selfTransform.position;
        }

        return hasReceivedTarget ? lastReportedCurrentWorld : transform.position;
    }

    private Vector3 ResolveWorldTargetFromLocalDelta(Vector3 localTargetDelta)
    {
        if (referenceTransform == null)
        {
            return ResolveCurrentWorldPosition() + localTargetDelta;
        }

        return referenceTransform.position
            + referenceTransform.right * localTargetDelta.x
            + referenceTransform.up * localTargetDelta.y
            + referenceTransform.forward * localTargetDelta.z;
    }
}
