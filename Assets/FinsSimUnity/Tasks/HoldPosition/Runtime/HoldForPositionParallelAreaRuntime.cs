using System;
using System.Collections.Generic;
using FinsSim.Hydrodynamics;
using UnityEngine;

/// <summary>
/// Repairs references which must stay inside one replicated HoldForPosition
/// training area.  TrainingAreaReplicator uses a 3D grid, hence each copied
/// water provider needs a matching world-space water height.
/// </summary>
[DefaultExecutionOrder(-4)]
public sealed class HoldForPositionParallelAreaRuntime : MonoBehaviour
{
    [Min(0.01f)] public float areaSeparation = 32f;
    public float localWaterHeight = 0f;
    public int seedStride = 1_000_003;

    void Awake()
    {
        ConfigureArea();
    }

    void OnValidate()
    {
        areaSeparation = Mathf.Max(0.01f, areaSeparation);
    }

    public void ConfigureArea()
    {
        MonoBehaviour provider = FindAreaWaterProvider();
        if (provider == null)
        {
            throw new InvalidOperationException($"{name} has no {nameof(IAreaWaterKinematicsProvider)}.");
        }
        // TrainingAreaReplicator offsets copies on all three axes. The Fossen
        // source scene uses the physical provider's flat fallback surface, so
        // make its water plane follow the Y lattice coordinate of this area.
        float waterHeight = transform.position.y + localWaterHeight;
        ((IAreaWaterKinematicsProvider)provider).SetAreaWaterHeight(waterHeight);

        Transform target = transform.Find("Target");
        if (target == null)
        {
            throw new InvalidOperationException($"{name} has no child named Target.");
        }

        int areaSeedOffset = StableAreaSeedOffset(transform.position, areaSeparation);
        foreach (HoldForPosition agent in GetComponentsInChildren<HoldForPosition>(true))
        {
            agent.ConfigureForParallelTrainingArea(transform, target, areaSeedOffset);
        }

        foreach (HydrodynamicsController controller in GetComponentsInChildren<HydrodynamicsController>(true))
        {
            controller.waterProviderBehaviour = provider;
        }

        foreach (YawDampingCompensationTarget targetComponent in GetComponentsInChildren<YawDampingCompensationTarget>(true))
        {
            targetComponent.waterProviderBehaviour = provider;
        }

        foreach (DomainRandomizationCoordinator coordinator in GetComponentsInChildren<DomainRandomizationCoordinator>(true))
        {
            // DomainRandomizationCoordinator has already applied the optional
            // -fins-dr-seed command-line override at execution order -5.
            coordinator.baseSeed = unchecked(coordinator.baseSeed + areaSeedOffset * seedStride);
            coordinator.targetBehaviours = BuildTargets(coordinator, provider);
        }
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

    static List<MonoBehaviour> BuildTargets(DomainRandomizationCoordinator coordinator, MonoBehaviour provider)
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
