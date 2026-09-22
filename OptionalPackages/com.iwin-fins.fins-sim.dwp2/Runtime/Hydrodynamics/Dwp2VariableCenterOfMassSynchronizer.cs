using NWH.Common.CoM;
using UnityEngine;

namespace FinsSim.Hydrodynamics
{
    /// <summary>Keeps NWH's cached mass/inertia state aligned with Unity Rigidbody changes.</summary>
    [DefaultExecutionOrder(110)]
    [DisallowMultipleComponent]
    [RequireComponent(typeof(Rigidbody))]
    public sealed class Dwp2VariableCenterOfMassSynchronizer : MonoBehaviour
    {
        public Rigidbody targetRigidbody;
        public VariableCenterOfMass variableCenterOfMass;

        void Awake()
        {
            ResolveReferences();
        }

        void FixedUpdate()
        {
            ResolveReferences();
            if (targetRigidbody == null || variableCenterOfMass == null)
            {
                return;
            }

            variableCenterOfMass.baseMass = targetRigidbody.mass;
            variableCenterOfMass.combinedMass = targetRigidbody.mass;
            if (!variableCenterOfMass.useDefaultInertia)
            {
                variableCenterOfMass.inertiaTensor = targetRigidbody.inertiaTensor;
                variableCenterOfMass.combinedInertiaTensor = targetRigidbody.inertiaTensor;
                variableCenterOfMass.MarkDirty();
            }
        }

        void ResolveReferences()
        {
            if (targetRigidbody == null)
            {
                targetRigidbody = GetComponent<Rigidbody>();
            }

            if (variableCenterOfMass == null)
            {
                variableCenterOfMass = GetComponent<VariableCenterOfMass>();
            }
        }
    }
}
