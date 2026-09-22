using System;
using System.Collections.Generic;
using Obi;
using FinsSim.Actuators;
using FinsSim.Hydrodynamics;
using UnityEngine;

/// <summary>
/// Rebinds references that must remain within one TrainingAreaReplicator copy
/// of the 3Chase1 scene.
/// </summary>
[DefaultExecutionOrder(-4)]
public sealed class ThreeChaseOneParallelAreaRuntime : MonoBehaviour
{
    [Min(0.01f)] public float areaSeparation = 96f;
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
        CatchAreaManager[] managers = GetComponentsInChildren<CatchAreaManager>(true);
        ChaserAgent[] chasers = GetComponentsInChildren<ChaserAgent>(true);
        PreyAgent[] preyAgents = GetComponentsInChildren<PreyAgent>(true);
        if (managers.Length != 1 || chasers.Length != 3 || preyAgents.Length != 1)
        {
            throw new InvalidOperationException(
                $"{name} must contain exactly one {nameof(CatchAreaManager)}, three {nameof(ChaserAgent)} instances, and one {nameof(PreyAgent)}.");
        }

        CatchAreaManager manager = managers[0];
        PreyAgent prey = preyAgents[0];
        ChaserAgent[] netters = Array.FindAll(chasers, chaser => chaser.role == AgentRole.Netter);
        ChaserAgent[] herders = Array.FindAll(chasers, chaser => chaser.role == AgentRole.Herder);
        if (netters.Length != 2 || herders.Length != 1)
        {
            throw new InvalidOperationException(
                $"{name} must contain two Netter and one Herder {nameof(ChaserAgent)} instances.");
        }

        manager.netter1 = netters[0];
        manager.netter2 = netters[1];
        manager.herder = herders[0];
        manager.fish = prey;
        manager.showRewardDebug = false;

        foreach (ChaserAgent chaser in chasers)
        {
            chaser.areaManager = manager;
            chaser.fish = prey.transform;
            if (chaser.selfTransform == null || !chaser.selfTransform.IsChildOf(chaser.transform))
            {
                chaser.selfTransform = FinsROVAgentRuntime.ResolveReferenceTransform(chaser.transform, null);
            }
        }
        netters[0].partnerNetter = netters[1];
        netters[1].partnerNetter = netters[0];
        herders[0].partnerNetter = null;

        prey.areaManager = manager;
        prey.chasers = new[] { netters[0].transform, netters[1].transform, herders[0].transform };

        FishNetGenerator[] generators = manager.GetComponentsInChildren<FishNetGenerator>(true);
        if (generators.Length != 1)
        {
            throw new InvalidOperationException($"{name} must contain exactly one {nameof(FishNetGenerator)}.");
        }
        FishNetGenerator generator = generators[0];
        generator.areaManager = manager;
        generator.netter1 = netters[0].transform;
        generator.netter2 = netters[1].transform;
        generator.herder = herders[0].transform;
        if (generator.netter1MarkPosition == null || !generator.netter1MarkPosition.IsChildOf(netters[0].transform))
        {
            generator.netter1MarkPosition = FinsROVAgentRuntime.ResolveReferenceTransform(netters[0].transform, null);
        }
        if (generator.netter2MarkPosition == null || !generator.netter2MarkPosition.IsChildOf(netters[1].transform))
        {
            generator.netter2MarkPosition = FinsROVAgentRuntime.ResolveReferenceTransform(netters[1].transform, null);
        }
        if (generator.herderMarkPosition == null || !generator.herderMarkPosition.IsChildOf(herders[0].transform))
        {
            generator.herderMarkPosition = FinsROVAgentRuntime.ResolveReferenceTransform(herders[0].transform, null);
        }
        RebindStaticLeadRopes(generator, netters[0].transform, netters[1].transform, herders[0].transform);
        if (generator.NetRoot != null)
        {
            manager.net = generator.NetRoot;
        }

        WaterCurrentDomainRandomizer current = GetComponentInChildren<WaterCurrentDomainRandomizer>(true);
        if (current == null)
        {
            throw new InvalidOperationException($"{name} has no scene-local {nameof(WaterCurrentDomainRandomizer)}.");
        }
        current.targetRigidbodies.Clear();
        foreach (ChaserAgent chaser in chasers)
        {
            Rigidbody rigidbody = chaser.GetComponent<Rigidbody>();
            if (rigidbody != null)
            {
                current.targetRigidbodies.Add(rigidbody);
            }
        }

        foreach (HydrodynamicsController controller in GetComponentsInChildren<HydrodynamicsController>(true))
        {
            controller.waterProviderBehaviour = current;
        }

        int areaSeedOffset = StableAreaSeedOffset(transform.position, areaSeparation);
        foreach (DomainRandomizationCoordinator coordinator in GetComponentsInChildren<DomainRandomizationCoordinator>(true))
        {
            coordinator.baseSeed = unchecked(coordinator.baseSeed + areaSeedOffset * seedStride);
            coordinator.targetBehaviours = BuildTargets(coordinator, current);
            coordinator.ResolveTargets();
        }
    }

    static List<MonoBehaviour> BuildTargets(
        DomainRandomizationCoordinator coordinator,
        WaterCurrentDomainRandomizer current)
    {
        var targets = new List<MonoBehaviour>();
        foreach (MonoBehaviour behaviour in coordinator.GetComponentsInChildren<MonoBehaviour>(true))
        {
            if (behaviour is IEpisodeRandomizable)
            {
                targets.Add(behaviour);
            }
        }
        if (!targets.Contains(current))
        {
            targets.Add(current);
        }
        return targets;
    }

    static void RebindStaticLeadRopes(
        FishNetGenerator generator,
        Transform netter1,
        Transform netter2,
        Transform herder)
    {
        Transform net = generator != null ? generator.NetRoot : null;
        if (net == null)
        {
            return;
        }

        foreach (ObiParticleAttachment attachment in net.GetComponentsInChildren<ObiParticleAttachment>(true))
        {
            ObiParticleAttachment[] attachments = attachment.GetComponents<ObiParticleAttachment>();
            if (attachments.Length == 0 || attachment != attachments[0])
            {
                continue;
            }

            if (attachment.gameObject.name == "LeadRope_Left")
            {
                attachment.target = netter1;
                attachment.attachmentType = ObiParticleAttachment.AttachmentType.Dynamic;
            }
            else if (attachment.gameObject.name == "LeadRope_Right")
            {
                attachment.target = netter2;
                attachment.attachmentType = ObiParticleAttachment.AttachmentType.Dynamic;
            }
            else if (attachment.gameObject.name == "LeadRope_Herder")
            {
                attachment.target = herder;
                attachment.attachmentType = ObiParticleAttachment.AttachmentType.Dynamic;
            }
            else if (attachment.gameObject.name == "LeadRope_Netter1")
            {
                attachment.target = netter1;
                attachment.attachmentType = ObiParticleAttachment.AttachmentType.Dynamic;
            }
            else if (attachment.gameObject.name == "LeadRope_Netter2")
            {
                attachment.target = netter2;
                attachment.attachmentType = ObiParticleAttachment.AttachmentType.Dynamic;
            }
        }
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
