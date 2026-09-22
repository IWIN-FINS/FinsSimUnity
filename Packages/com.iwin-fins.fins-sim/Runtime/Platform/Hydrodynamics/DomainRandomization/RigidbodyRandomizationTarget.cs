using UnityEngine;

namespace FinsSim.Hydrodynamics
{
    [RequireComponent(typeof(Rigidbody))]
    public class RigidbodyRandomizationTarget : MonoBehaviour, IEpisodeRandomizable
    {
        public Rigidbody targetRigidbody;

        float _baseMass;
        Vector3 _baseInertiaTensor;
        bool _captured;

        void Awake()
        {
            CaptureBaseline();
        }

        public void CaptureBaseline()
        {
            if (targetRigidbody == null)
            {
                targetRigidbody = GetComponent<Rigidbody>();
            }

            if (targetRigidbody == null)
            {
                return;
            }

            _baseMass = targetRigidbody.mass;
            _baseInertiaTensor = targetRigidbody.inertiaTensor;
            _captured = true;
        }

        public void RandomizeForEpisode(RandomizationContext context)
        {
            if (context.Profile == null || !context.Profile.randomizeBody)
            {
                return;
            }

            if (!_captured)
            {
                CaptureBaseline();
            }

            if (targetRigidbody == null)
            {
                return;
            }

            float massScale = context.Range(context.Profile.massScale, "body.mass-scale");
            targetRigidbody.mass = Mathf.Max(0.001f, _baseMass * massScale);
            float inertiaScale = context.Range(context.Profile.inertiaScale, "body.inertia-scale");
            if (context.Profile.coupleMassAndVolumeForNeutralBuoyancy)
            {
                inertiaScale *= massScale;
            }
            targetRigidbody.inertiaTensor = _baseInertiaTensor * Mathf.Max(0.001f, inertiaScale);

        }
    }
}
