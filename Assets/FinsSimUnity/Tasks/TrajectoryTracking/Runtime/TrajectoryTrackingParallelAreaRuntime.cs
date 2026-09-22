using System;
using System.Collections.Generic;
using FinsSim.Hydrodynamics;
using UnityEngine;

/// <summary>Area-local bindings for replicated T2 trajectory training areas.</summary>
[DefaultExecutionOrder(-4)]
public sealed class TrajectoryTrackingParallelAreaRuntime : MonoBehaviour
{
    [Min(0.01f)] public float areaSeparation = 32f;
    public float localWaterHeight;
    public int seedStride = 1_000_003;

    void Awake() => ConfigureArea();

    public void ConfigureArea()
    {
        MonoBehaviour provider = FindAreaWaterProvider();
        if (provider == null) throw new InvalidOperationException($"{name} has no {nameof(IAreaWaterKinematicsProvider)}.");
        float waterHeight = transform.position.y + localWaterHeight;
        ((IAreaWaterKinematicsProvider)provider).SetAreaWaterHeight(waterHeight);

        Transform reference = transform.Find("TrajectoryReference");
        if (reference == null) throw new InvalidOperationException($"{name} has no TrajectoryReference.");
        foreach (TrajectoryTrackingAgent agent in GetComponentsInChildren<TrajectoryTrackingAgent>(true)) agent.ConfigureForParallelTrainingArea(transform, reference);
        foreach (HydrodynamicsController controller in GetComponentsInChildren<HydrodynamicsController>(true)) controller.waterProviderBehaviour = provider;
        foreach (YawDampingCompensationTarget target in GetComponentsInChildren<YawDampingCompensationTarget>(true)) target.waterProviderBehaviour = provider;

        int areaSeedOffset = StableAreaSeedOffset(transform.position, areaSeparation);
        foreach (DomainRandomizationCoordinator coordinator in GetComponentsInChildren<DomainRandomizationCoordinator>(true))
        {
            coordinator.baseSeed = unchecked(coordinator.baseSeed + areaSeedOffset * seedStride);
            coordinator.targetBehaviours = BuildTargets(coordinator, provider);
        }
    }

    static List<MonoBehaviour> BuildTargets(DomainRandomizationCoordinator coordinator, MonoBehaviour provider)
    {
        var targets = new List<MonoBehaviour>();
        foreach (MonoBehaviour behaviour in coordinator.GetComponentsInChildren<MonoBehaviour>(true)) if (behaviour is IEpisodeRandomizable) targets.Add(behaviour);
        if (provider is IEpisodeRandomizable && !targets.Contains(provider)) targets.Add(provider);
        return targets;
    }

    MonoBehaviour FindAreaWaterProvider()
    {
        foreach (MonoBehaviour behaviour in GetComponentsInChildren<MonoBehaviour>(true))
        {
            if (behaviour is IAreaWaterKinematicsProvider)
            {
                return behaviour;
            }
        }

        return null;
    }

    static int StableAreaSeedOffset(Vector3 position, float separation)
    {
        int x = Mathf.RoundToInt(position.x / separation);
        int y = Mathf.RoundToInt(position.y / separation);
        int z = Mathf.RoundToInt(position.z / separation);
        unchecked { return x * 73_856_093 ^ y * 19_349_663 ^ z * 83_492_791; }
    }
}
