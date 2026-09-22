using FinsSim.Hydrodynamics;
using FinsSim.Networking;
using NWH.Common.CoM;
using NWH.DWP2.ShipController;
using NWH.DWP2.WaterObjects;
using UnityEngine;

namespace FinsSim.Dwp2
{
    /// <summary>
    /// Preserves existing private scenes while their DWP2 compatibility components
    /// move out of the public core package. It only runs when this private package
    /// and the upstream NWH packages are installed.
    /// </summary>
    static class Dwp2CompatibilityBootstrap
    {
        [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.AfterSceneLoad)]
        static void InstallCompatibilityComponents()
        {
            foreach (HydrodynamicsController controller in Object.FindObjectsByType<HydrodynamicsController>(FindObjectsInactive.Include, FindObjectsSortMode.None))
            {
                if (controller.GetComponent<Dwp2HydrodynamicsInterop>() == null &&
                    controller.GetComponentInChildren<WaterObject>(true) != null)
                {
                    controller.gameObject.AddComponent<Dwp2HydrodynamicsInterop>();
                }
            }

            foreach (AdvancedShipController shipController in Object.FindObjectsByType<AdvancedShipController>(FindObjectsInactive.Include, FindObjectsSortMode.None))
            {
                if (shipController.GetComponent<Dwp2ThrusterCommandFallback>() == null)
                {
                    shipController.gameObject.AddComponent<Dwp2ThrusterCommandFallback>();
                }
            }

            foreach (VariableCenterOfMass centerOfMass in Object.FindObjectsByType<VariableCenterOfMass>(FindObjectsInactive.Include, FindObjectsSortMode.None))
            {
                if (centerOfMass.GetComponent<Dwp2VariableCenterOfMassSynchronizer>() == null &&
                    centerOfMass.GetComponent<Rigidbody>() != null)
                {
                    centerOfMass.gameObject.AddComponent<Dwp2VariableCenterOfMassSynchronizer>();
                }
            }
        }
    }
}
