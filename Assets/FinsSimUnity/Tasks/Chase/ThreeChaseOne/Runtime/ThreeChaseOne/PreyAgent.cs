using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class PreyAgent : Agent
{
    public Transform[] chasers; // 引用所有追方
    public CatchAreaManager areaManager;
    private Rigidbody rb;
    public float moveSpeed = 6f; // 通常鱼要比追方稍微灵活一点
    [Header("Control Model")]
    [Tooltip("鱼的垂直速度相对水平速度的缩放。小于 1 可以避免鱼用上下窜动逃逸。")]
    [Range(0f, 1f)]
    public float verticalSpeedScale = 0.25f;
    [Tooltip("是否把鱼限制在水柱内，防止随机策略或策略网络把鱼带出水面。")]
    public bool constrainToWaterColumn = true;
    [Tooltip("水面世界坐标 Y。当前场景的 Dynamic Water Physics 默认水面为 0。")]
    public float waterSurfaceY = 0f;
    [Tooltip("鱼中心点允许接近水面的距离。0 表示中心点不允许高于水面。")]
    public float surfaceClearance = 0f;
    [Tooltip("鱼允许到达的最低世界坐标 Y。")]
    public float minWaterY = -6f;
    [Tooltip("离上下边界多近时开始压低对应方向的垂直速度。")]
    public float verticalBoundaryBuffer = 0.35f;
    [Tooltip("越界后拉回水柱的速度增益。")]
    public float verticalBoundaryCorrectionGain = 4f;
    [Header("Reward Tuning")]
    [Tooltip("每步轻微生存奖励，鼓励延迟被捕获。")]
    public float survivalReward = 0.0005f;
    [Tooltip("鱼整体拉开与追方平均距离时的进度奖励系数。")]
    public float averageSeparationProgressRewardScale = 0.05f;
    [Tooltip("鱼拉开与最近威胁距离时的进度奖励系数。")]
    public float nearestThreatProgressRewardScale = 0.12f;
    private float previousAverageDistanceToChasers;
    private float previousNearestDistanceToChaser;
    private readonly float[] baselineOverrideActions = new float[3];
    private readonly float[] appliedContinuousActions = new float[3];

    public float LastStepReward { get; private set; }
    public float LastSurvivalReward { get; private set; }
    public float LastAverageSeparationReward { get; private set; }
    public float LastNearestThreatReward { get; private set; }

    private static float GetBoundedIncrease(float previousValue, float currentValue)
    {
        return Mathf.Clamp(currentValue - previousValue, -1f, 1f);
    }

    private void ClearRewardDebugBreakdown()
    {
        LastStepReward = 0f;
        LastSurvivalReward = 0f;
        LastAverageSeparationReward = 0f;
        LastNearestThreatReward = 0f;
    }

    private float GetAverageDistanceToChasers()
    {
        if (chasers == null || chasers.Length == 0)
        {
            return 0f;
        }

        float totalDistance = 0f;
        foreach (Transform chaser in chasers)
        {
            if (chaser != null)
            {
                totalDistance += Vector3.Distance(transform.position, chaser.position);
            }
        }

        return totalDistance / chasers.Length;
    }

    private float GetNearestDistanceToChaser()
    {
        if (chasers == null || chasers.Length == 0)
        {
            return 0f;
        }

        float nearestDistance = float.PositiveInfinity;
        foreach (Transform chaser in chasers)
        {
            if (chaser != null)
            {
                nearestDistance = Mathf.Min(nearestDistance, Vector3.Distance(transform.position, chaser.position));
            }
        }

        return float.IsPositiveInfinity(nearestDistance) ? 0f : nearestDistance;
    }

    private void ResetRewardTracking()
    {
        ClearRewardDebugBreakdown();
        previousAverageDistanceToChasers = GetAverageDistanceToChasers();
        previousNearestDistanceToChaser = GetNearestDistanceToChaser();
    }

    public override void Initialize()
    {
        rb = GetComponent<Rigidbody>();
    }

    public override void OnEpisodeBegin()
    {
        // 鱼通常不需要重置环境，它只需要响应重置
        ResetRewardTracking();
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(transform.localPosition);
        sensor.AddObservation(rb.linearVelocity);

        // 观察所有追方的位置
        foreach (var chaser in chasers)
        {
            sensor.AddObservation(chaser.localPosition);
        }

        // 总共的观察维度 = 3 (自身位置) + 3 (自身速度) + 3 * N (每个追方位置) = 15
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        if (areaManager != null && areaManager.TryResolveCapture())
        {
            return;
        }

        ResolveAppliedContinuousActions(actions);

        rb.linearVelocity = BuildDesiredVelocity();

        // --- 奖励逻辑 ---

        float reward = survivalReward;
        ClearRewardDebugBreakdown();
        LastSurvivalReward = survivalReward;

        float averageDistanceToChasers = GetAverageDistanceToChasers();
        float nearestDistanceToChaser = GetNearestDistanceToChaser();
        float averageSeparationReward = averageSeparationProgressRewardScale
                                        * GetBoundedIncrease(previousAverageDistanceToChasers, averageDistanceToChasers);
        float nearestThreatReward = nearestThreatProgressRewardScale
                                    * GetBoundedIncrease(previousNearestDistanceToChaser, nearestDistanceToChaser);
        reward += averageSeparationReward;
        reward += nearestThreatReward;
        LastAverageSeparationReward = averageSeparationReward;
        LastNearestThreatReward = nearestThreatReward;

        previousAverageDistanceToChasers = averageDistanceToChasers;
        previousNearestDistanceToChaser = nearestDistanceToChaser;
        SetReward(reward);
        LastStepReward = reward;

        // Debug.Log("Prey Step reward: " + reward);

        // // 撞墙惩罚
        // if (transform.localPosition.x > 10f || transform.localPosition.x < -10f ||
        //     transform.localPosition.z > 10f || transform.localPosition.z < -10f)
        // {
        //     reward = -0.5f;
        //     SetReward(-0.5f);
        //     EndEpisode();
        // }
    }

    private void ResolveAppliedContinuousActions(ActionBuffers actions)
    {
        for (int i = 0; i < appliedContinuousActions.Length; i++)
        {
            appliedContinuousActions[i] = i < actions.ContinuousActions.Length
                ? Mathf.Clamp(actions.ContinuousActions[i], -1f, 1f)
                : 0f;
        }

        ThreeChaseOneBaselineController baselineController =
            ThreeChaseOneBaselineController.ResolveActive(areaManager);
        if (baselineController == null
            || !baselineController.ShouldOverridePreyActions()
            || !baselineController.TryGetPreyOverrideActions(this, baselineOverrideActions))
        {
            return;
        }

        for (int i = 0; i < appliedContinuousActions.Length; i++)
        {
            appliedContinuousActions[i] = Mathf.Clamp(baselineOverrideActions[i], -1f, 1f);
        }
    }

    private Vector3 BuildDesiredVelocity()
    {
        Vector2 horizontalAction = Vector2.ClampMagnitude(
            new Vector2(appliedContinuousActions[0], appliedContinuousActions[2]),
            1f);
        float verticalAction = Mathf.Clamp(appliedContinuousActions[1], -1f, 1f) * verticalSpeedScale;

        Vector3 desiredVelocity = new Vector3(
            horizontalAction.x,
            verticalAction,
            horizontalAction.y) * moveSpeed;

        return constrainToWaterColumn
            ? ApplyWaterColumnConstraint(desiredVelocity)
            : desiredVelocity;
    }

    private Vector3 ApplyWaterColumnConstraint(Vector3 desiredVelocity)
    {
        float upperY = waterSurfaceY - Mathf.Max(0f, surfaceClearance);
        float lowerY = Mathf.Min(minWaterY, upperY);
        float buffer = Mathf.Max(0f, verticalBoundaryBuffer);
        float currentY = transform.position.y;

        if (buffer > 1e-4f)
        {
            if (desiredVelocity.y > 0f && currentY > upperY - buffer)
            {
                desiredVelocity.y *= Mathf.InverseLerp(upperY, upperY - buffer, currentY);
            }
            else if (desiredVelocity.y < 0f && currentY < lowerY + buffer)
            {
                desiredVelocity.y *= Mathf.InverseLerp(lowerY, lowerY + buffer, currentY);
            }
        }

        if (currentY > upperY)
        {
            float correctionVelocity = -(currentY - upperY) * verticalBoundaryCorrectionGain;
            desiredVelocity.y = Mathf.Min(desiredVelocity.y, correctionVelocity);
        }
        else if (currentY < lowerY)
        {
            float correctionVelocity = (lowerY - currentY) * verticalBoundaryCorrectionGain;
            desiredVelocity.y = Mathf.Max(desiredVelocity.y, correctionVelocity);
        }

        return desiredVelocity;
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        var continuousActionsOut = actionsOut.ContinuousActions;
        for (int i = 0; i < continuousActionsOut.Length; i++)
        {
            continuousActionsOut[i] = 0f;
        }
    }

    // 碰撞检测：鱼与网发生碰撞
    private void OnCollisionEnter(Collision collision)
    {
        if (IsNetCollision(collision))
        {
            areaManager.OnNetCollisionWithFish();
        }
    }

    // 可选：持续碰撞检测，确保碰撞状态持续
    private void OnCollisionStay(Collision collision)
    {
        if (IsNetCollision(collision))
        {
            areaManager.OnNetCollisionWithFish();
        }
    }

    private bool IsNetCollision(Collision collision)
    {
        Transform netRoot = areaManager != null ? areaManager.net : null;
        if (collision == null || netRoot == null)
        {
            return false;
        }

        Transform other = collision.transform;
        return other == netRoot || other.IsChildOf(netRoot);
    }
}
