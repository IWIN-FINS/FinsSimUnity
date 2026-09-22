using FinsSim.Actuators;
using FinsSim.Hydrodynamics;
using UnityEngine;

[DisallowMultipleComponent]
[RequireComponent(typeof(Rigidbody))]
public class FinsROVHydrodynamicsSetup : MonoBehaviour
{
    [Header("Core")]
    public HydrodynamicsController hydrodynamicsController;
    public HydrodynamicsProfile hydrodynamicsProfile;
    public DomainRandomizationCoordinator domainRandomizationCoordinator;
    public DomainRandomizationProfile domainRandomizationProfile;

    [Header("Water")]
    public MonoBehaviour waterProviderBehaviour;
    public bool disableLegacyCurrentDirectForces = true;

    [Header("Randomization Targets")]
    public bool ensureRigidbodyRandomizationTarget = false;
    public bool skipRigidbodyRandomizationWhenExternalOwnerExists = true;
    public bool ensureThrusterRandomizationTarget = true;
    public bool ensureInitialStateRandomizationTarget = false;

    void Reset()
    {
        ResolveReferences();
    }

    void Awake()
    {
        ResolveReferences();
        ApplyConfiguration();
    }

    void OnValidate()
    {
        ResolveReferences();
    }

    public void ResolveReferences()
    {
        if (hydrodynamicsController == null)
        {
            hydrodynamicsController = GetComponent<HydrodynamicsController>();
        }

        if (hydrodynamicsController == null)
        {
            hydrodynamicsController = GetComponentInChildren<HydrodynamicsController>(true);
        }

        if (domainRandomizationCoordinator == null)
        {
            domainRandomizationCoordinator = GetComponent<DomainRandomizationCoordinator>();
        }

        if (waterProviderBehaviour == null)
        {
            foreach (MonoBehaviour behaviour in GetComponentsInChildren<MonoBehaviour>(true))
            {
                if (behaviour is IWaterKinematicsProvider)
                {
                    waterProviderBehaviour = behaviour;
                    break;
                }
            }
        }
    }

    public void ApplyConfiguration()
    {
        if (hydrodynamicsController != null)
        {
            hydrodynamicsController.profile = hydrodynamicsProfile != null
                ? hydrodynamicsProfile
                : hydrodynamicsController.profile;
            hydrodynamicsController.waterProviderBehaviour = waterProviderBehaviour;
            hydrodynamicsController.mode = hydrodynamicsController.mode == HydrodynamicsMode.Off
                ? HydrodynamicsMode.Fossen6Dof
                : hydrodynamicsController.mode;
        }

        if (domainRandomizationCoordinator != null)
        {
            domainRandomizationCoordinator.profile = domainRandomizationProfile != null
                ? domainRandomizationProfile
                : domainRandomizationCoordinator.profile;
        }

        if (disableLegacyCurrentDirectForces && waterProviderBehaviour is WaterCurrentDomainRandomizer randomizer)
        {
            randomizer.applyForcesToRigidbodies = false;
        }

        EnsureRandomizationTargets();
    }

    void EnsureRandomizationTargets()
    {
        if (domainRandomizationCoordinator == null)
        {
            return;
        }

        bool hasExternalRigidbodyRandomizationOwner = false;
        if (skipRigidbodyRandomizationWhenExternalOwnerExists)
        {
            foreach (MonoBehaviour behaviour in GetComponentsInChildren<MonoBehaviour>(true))
            {
                if (behaviour is IRigidbodyRandomizationOwner)
                {
                    hasExternalRigidbodyRandomizationOwner = true;
                    break;
                }
            }
        }

        if (ensureRigidbodyRandomizationTarget &&
            !hasExternalRigidbodyRandomizationOwner &&
            GetComponent<RigidbodyRandomizationTarget>() == null)
        {
            gameObject.AddComponent<RigidbodyRandomizationTarget>();
        }

        if (ensureThrusterRandomizationTarget && GetComponent<ThrusterRandomizationTarget>() == null)
        {
            var target = gameObject.AddComponent<ThrusterRandomizationTarget>();
            target.thrusters = GetComponentsInChildren<Thruster>(true);
        }

        if (ensureInitialStateRandomizationTarget && GetComponent<InitialStateRandomizationTarget>() == null)
        {
            gameObject.AddComponent<InitialStateRandomizationTarget>();
        }

        domainRandomizationCoordinator.ResolveTargets();
    }
}
