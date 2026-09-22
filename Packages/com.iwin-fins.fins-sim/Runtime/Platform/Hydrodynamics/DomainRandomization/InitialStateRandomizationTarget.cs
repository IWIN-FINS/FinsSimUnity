using UnityEngine;

namespace FinsSim.Hydrodynamics
{
    [RequireComponent(typeof(Rigidbody))]
    public class InitialStateRandomizationTarget : MonoBehaviour, IEpisodeRandomizable
    {
        public Rigidbody targetRigidbody;
        public bool useInitialTransformAsBase = true;
        public Vector3 basePosition;
        public Quaternion baseRotation = Quaternion.identity;

        void Awake()
        {
            if (targetRigidbody == null)
            {
                targetRigidbody = GetComponent<Rigidbody>();
            }

            if (useInitialTransformAsBase)
            {
                basePosition = transform.position;
                baseRotation = transform.rotation;
            }
        }

        public void RandomizeForEpisode(RandomizationContext context)
        {
            if (context.Profile == null || !context.Profile.randomizeInitialState)
            {
                return;
            }

            if (targetRigidbody == null)
            {
                targetRigidbody = GetComponent<Rigidbody>();
            }

            Vector3 position = basePosition + context.Range(context.Profile.initialPositionOffset);
            Quaternion rotation = baseRotation * Quaternion.Euler(context.Range(context.Profile.initialEulerOffsetDeg));
            transform.SetPositionAndRotation(position, rotation);

            if (targetRigidbody != null)
            {
                targetRigidbody.position = position;
                targetRigidbody.rotation = rotation;
                targetRigidbody.linearVelocity = transform.TransformDirection(context.Range(context.Profile.initialLinearVelocity));
                targetRigidbody.angularVelocity = transform.TransformDirection(context.Range(context.Profile.initialAngularVelocity));
                targetRigidbody.WakeUp();
            }
        }
    }
}
