using NWH.DWP2.WaterObjects;
using FinsSim.Hydrodynamics;
using UnityEngine;

[DisallowMultipleComponent]
public sealed class FinsROVDwp2ForceReadout : MonoBehaviour
{
    [Header("References")]
    [SerializeField] Rigidbody targetRigidbody;
    [SerializeField] HydrodynamicsController hydrodynamicsController;
    [SerializeField] Transform waterObjectSearchRoot;
    [SerializeField] string mainBodyWaterObjectName = "mainbody";

    [Header("Readout")]
    [SerializeField] int waterObjectCount;
    [SerializeField] int enabledWaterObjectCount;
    [SerializeField] Vector3 totalDwp2Force;
    [SerializeField] Vector3 totalDwp2Torque;
    [SerializeField] float totalSubmergedVolume;
    [SerializeField] Vector3 mainBodyForce;
    [SerializeField] Vector3 mainBodyTorque;
    [SerializeField] float mainBodySubmergedVolume;
    [SerializeField] Vector3 marusHydroForce;
    [SerializeField] Vector3 marusHydroTorque;
    [SerializeField] float rigidbodyMassKg;
    [SerializeField] float gravityForceY;
    [SerializeField] float netVerticalForceY;
    [SerializeField] float currentMainBodyBuoyantForceCoefficient;
    [SerializeField] float estimatedNeutralMainBodyBuoyantForceCoefficient;
    [SerializeField] float estimatedNeutralUniformBuoyantForceCoefficient;
    [SerializeField] float verticalForceErrorY;

    WaterObject[] _waterObjects;

    void Reset()
    {
        ResolveReferences();
    }

    void Awake()
    {
        ResolveReferences();
    }

    void LateUpdate()
    {
        RefreshReadout();
    }

    [ContextMenu("Resolve References")]
    public void ResolveReferences()
    {
        if (targetRigidbody == null)
        {
            targetRigidbody = GetComponentInParent<Rigidbody>();
        }

        if (targetRigidbody == null)
        {
            targetRigidbody = GetComponentInChildren<Rigidbody>(true);
        }

        if (hydrodynamicsController == null)
        {
            hydrodynamicsController = targetRigidbody != null
                ? targetRigidbody.GetComponent<HydrodynamicsController>()
                : GetComponentInChildren<HydrodynamicsController>(true);
        }

        if (waterObjectSearchRoot == null)
        {
            waterObjectSearchRoot = targetRigidbody != null ? targetRigidbody.transform : transform;
        }

        RefreshWaterObjects();
    }

    [ContextMenu("Refresh Water Objects")]
    public void RefreshWaterObjects()
    {
        _waterObjects = waterObjectSearchRoot != null
            ? waterObjectSearchRoot.GetComponentsInChildren<WaterObject>(true)
            : GetComponentsInChildren<WaterObject>(true);
    }

    void RefreshReadout()
    {
        if (_waterObjects == null)
        {
            RefreshWaterObjects();
        }

        waterObjectCount = _waterObjects != null ? _waterObjects.Length : 0;
        enabledWaterObjectCount = 0;
        totalDwp2Force = Vector3.zero;
        totalDwp2Torque = Vector3.zero;
        totalSubmergedVolume = 0f;
        mainBodyForce = Vector3.zero;
        mainBodyTorque = Vector3.zero;
        mainBodySubmergedVolume = 0f;
        currentMainBodyBuoyantForceCoefficient = 0f;

        if (_waterObjects != null)
        {
            for (int i = 0; i < _waterObjects.Length; i++)
            {
                WaterObject waterObject = _waterObjects[i];
                if (waterObject == null || !waterObject.isActiveAndEnabled)
                {
                    continue;
                }

                enabledWaterObjectCount++;
                totalDwp2Force += waterObject.ResultForce;
                totalDwp2Torque += waterObject.ResultTorque;
                totalSubmergedVolume += waterObject.submergedVolume;

                if (waterObject.name.Equals(mainBodyWaterObjectName, System.StringComparison.OrdinalIgnoreCase))
                {
                    mainBodyForce += waterObject.ResultForce;
                    mainBodyTorque += waterObject.ResultTorque;
                    mainBodySubmergedVolume += waterObject.submergedVolume;
                    currentMainBodyBuoyantForceCoefficient = waterObject.buoyantForceCoefficient;
                }
            }
        }

        marusHydroForce = hydrodynamicsController != null ? hydrodynamicsController.LastWrench.WorldForce : Vector3.zero;
        marusHydroTorque = hydrodynamicsController != null ? hydrodynamicsController.LastWrench.WorldTorque : Vector3.zero;
        rigidbodyMassKg = targetRigidbody != null ? targetRigidbody.mass : 0f;
        gravityForceY = targetRigidbody != null ? targetRigidbodyMassForceY() : 0f;
        netVerticalForceY = totalDwp2Force.y + marusHydroForce.y + gravityForceY;
        verticalForceErrorY = -netVerticalForceY;
        estimatedNeutralMainBodyBuoyantForceCoefficient = EstimateNeutralMainBodyBfc();
        estimatedNeutralUniformBuoyantForceCoefficient = EstimateNeutralUniformBfc();
    }

    float targetRigidbodyMassForceY()
    {
        return Physics.gravity.y * targetRigidbody.mass;
    }

    float EstimateNeutralMainBodyBfc()
    {
        if (Mathf.Abs(mainBodyForce.y) < 1e-4f || currentMainBodyBuoyantForceCoefficient <= 0f)
        {
            return 0f;
        }

        float scale = 1f - netVerticalForceY / mainBodyForce.y;
        return Mathf.Max(0f, currentMainBodyBuoyantForceCoefficient * scale);
    }

    float EstimateNeutralUniformBfc()
    {
        if (Mathf.Abs(totalDwp2Force.y) < 1e-4f || currentMainBodyBuoyantForceCoefficient <= 0f)
        {
            return 0f;
        }

        float scale = 1f - netVerticalForceY / totalDwp2Force.y;
        return Mathf.Max(0f, currentMainBodyBuoyantForceCoefficient * scale);
    }
}
