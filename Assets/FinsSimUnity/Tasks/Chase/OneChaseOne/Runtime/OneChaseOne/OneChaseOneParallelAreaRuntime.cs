using System;
using System.Collections.Generic;
using FinsSim.Hydrodynamics;
using NWH.DWP2.WaterData;
using UnityEngine;

/// <summary>
/// Repairs the scene-local references that must not cross TrainingAreaReplicator
/// clone boundaries in the 1Chase1 scene.
/// </summary>
[DefaultExecutionOrder(-4)]
public sealed class OneChaseOneParallelAreaRuntime : MonoBehaviour
{
    [Min(0.01f)] public float areaSeparation = 32f;
    public float localWaterSurfaceY = 0f;
    public float localMinWaterY = -6f;
    public int seedStride = 1_000_003;

    void Awake()
    {
        ConfigureArea();
    }

    void OnValidate()
    {
        areaSeparation = Mathf.Max(0.01f, areaSeparation);
        localMinWaterY = Mathf.Min(localMinWaterY, localWaterSurfaceY);
    }

    public void ConfigureArea()
    {
        OneChaseOnePoseAgent[] agents = GetComponentsInChildren<OneChaseOnePoseAgent>(true);
        OneChaseOnePreyController[] preyControllers = GetComponentsInChildren<OneChaseOnePreyController>(true);
        if (agents.Length != 1 || preyControllers.Length != 1)
        {
            throw new InvalidOperationException(
                $"{name} must contain exactly one {nameof(OneChaseOnePoseAgent)} and one {nameof(OneChaseOnePreyController)}.");
        }

        PhysicalWaveWaterDataProvider provider = GetComponentInChildren<PhysicalWaveWaterDataProvider>(true);
        if (provider == null)
        {
            throw new InvalidOperationException($"{name} has no {nameof(PhysicalWaveWaterDataProvider)}.");
        }

        float waterHeight = transform.position.y + localWaterSurfaceY;
        provider.stillWaterHeight = waterHeight;
        provider.fallbackWaterHeight = waterHeight;

        OneChaseOnePoseAgent agent = agents[0];
        OneChaseOnePreyController prey = preyControllers[0];
        agent.ConfigureForParallelTrainingArea(transform, prey.transform, prey, waterHeight);
        prey.ConfigureForParallelTrainingArea(transform, agent.transform, localWaterSurfaceY, localMinWaterY);

        foreach (HydrodynamicsController controller in GetComponentsInChildren<HydrodynamicsController>(true))
        {
            controller.waterProviderBehaviour = provider;
        }

        foreach (YawDampingCompensationTarget target in GetComponentsInChildren<YawDampingCompensationTarget>(true))
        {
            target.waterProviderBehaviour = provider;
        }

        int areaSeedOffset = StableAreaSeedOffset(transform.position, areaSeparation);
        foreach (DomainRandomizationCoordinator coordinator in GetComponentsInChildren<DomainRandomizationCoordinator>(true))
        {
            coordinator.baseSeed = unchecked(coordinator.baseSeed + areaSeedOffset * seedStride);
            coordinator.targetBehaviours = BuildTargets(coordinator, provider);
            coordinator.ResolveTargets();
        }
    }

    static List<MonoBehaviour> BuildTargets(
        DomainRandomizationCoordinator coordinator,
        PhysicalWaveWaterDataProvider provider)
    {
        var targets = new List<MonoBehaviour>();
        foreach (MonoBehaviour behaviour in coordinator.GetComponentsInChildren<MonoBehaviour>(true))
        {
            if (behaviour is IEpisodeRandomizable)
            {
                targets.Add(behaviour);
            }
        }
        if (provider is IEpisodeRandomizable && !targets.Contains(provider))
        {
            targets.Add(provider);
        }
        return targets;
    }

    static int StableAreaSeedOffset(Vector3 position, float separation)
    {
        int x = Mathf.RoundToInt(position.x / separation);
        int y = Mathf.RoundToInt(position.y / separation);
        int z = Mathf.RoundToInt(position.z / separation);
        unchecked
        {
            return x * 73_856_093 ^ y * 19_349_663 ^ z * 83_492_791;
        }
    }
}
