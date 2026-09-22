using UnityEngine;

namespace FinsSim.Hydrodynamics
{
    public class StochasticCurrentProvider : FlatWaterKinematicsProvider, IEpisodeRandomizable
    {
        [Header("Episode Current")]
        public bool randomizeOnStart = true;
        public FloatRange speedRange = new FloatRange(0f, 0.35f);
        public bool allowVerticalCurrent;
        public Vector3 meanDirection = Vector3.forward;
        [Range(0f, 180f)] public float maxAngleFromMeanDegrees = 180f;

        [Header("Per-step Noise")]
        public bool addPerStepNoise = true;
        public Vector3 noiseAmplitude = new Vector3(0.02f, 0f, 0.02f);

        [Header("Runtime Debug")]
        [SerializeField] Vector3 episodeCurrent;
        [SerializeField] Vector3 lastNoise;

        System.Random _random = new System.Random(12345);

        void Start()
        {
            if (randomizeOnStart)
            {
                RandomizeLocal(new RandomizationContext(null, DomainRandomizationMode.Train, Random.Range(0, int.MaxValue), 0));
            }
        }

        public override WaterKinematicsSample Sample(Vector3 worldPoint)
        {
            Vector3 flow = episodeCurrent;
            if (addPerStepNoise)
            {
                lastNoise = new Vector3(
                    RandomSigned() * noiseAmplitude.x,
                    RandomSigned() * noiseAmplitude.y,
                    RandomSigned() * noiseAmplitude.z);
                flow += lastNoise;
            }

            return new WaterKinematicsSample
            {
                IsValid = true,
                Height = waterHeight,
                Normal = Vector3.up,
                FlowVelocity = flow,
                AngularFlowVelocity = angularCurrentVelocity,
            };
        }

        public void RandomizeForEpisode(RandomizationContext context)
        {
            if (context.Profile != null && context.Profile.randomizeWater)
            {
                speedRange = context.Profile.currentSpeed;
                allowVerticalCurrent = context.Profile.allowVerticalCurrent;
                meanDirection = context.Profile.meanCurrent.sqrMagnitude > 1e-8f
                    ? context.Profile.meanCurrent.normalized
                    : meanDirection;
                maxAngleFromMeanDegrees = context.Profile.maxAngleFromMeanDegrees;
                noiseAmplitude = context.Range(context.Profile.currentNoiseAmplitude);
            }

            RandomizeLocal(context);
        }

        void RandomizeLocal(RandomizationContext context)
        {
            _random = new System.Random(context.Seed);
            float speed = context.Profile != null
                ? context.Range(speedRange)
                : speedRange.ClampSample((float)_random.NextDouble());
            Vector3 direction = context.Profile != null
                ? context.CurrentDirection()
                : SampleDirection();
            episodeCurrent = direction * Mathf.Max(0f, speed);
            currentVelocity = episodeCurrent;
        }

        Vector3 SampleDirection()
        {
            Vector3 mean = meanDirection.sqrMagnitude > 1e-8f ? meanDirection.normalized : Vector3.forward;
            if (maxAngleFromMeanDegrees >= 179.9f)
            {
                float yaw = Mathf.Lerp(0f, Mathf.PI * 2f, (float)_random.NextDouble());
                Vector3 direction = new Vector3(Mathf.Cos(yaw), 0f, Mathf.Sin(yaw));
                if (allowVerticalCurrent)
                {
                    direction.y = Mathf.Lerp(-1f, 1f, (float)_random.NextDouble());
                }
                return direction.normalized;
            }

            float angle = Mathf.Lerp(-maxAngleFromMeanDegrees, maxAngleFromMeanDegrees, (float)_random.NextDouble());
            Vector3 axis = allowVerticalCurrent ? Random.onUnitSphere : Vector3.up;
            Vector3 sampled = Quaternion.AngleAxis(angle, axis) * mean;
            if (!allowVerticalCurrent)
            {
                sampled.y = 0f;
            }

            return sampled.sqrMagnitude > 1e-8f ? sampled.normalized : Vector3.forward;
        }

        float RandomSigned()
        {
            return Mathf.Lerp(-1f, 1f, (float)_random.NextDouble());
        }
    }
}
