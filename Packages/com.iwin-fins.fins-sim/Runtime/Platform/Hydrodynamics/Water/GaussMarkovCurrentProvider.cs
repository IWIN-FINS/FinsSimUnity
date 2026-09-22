using UnityEngine;

namespace FinsSim.Hydrodynamics
{
    public class GaussMarkovCurrentProvider : FlatWaterKinematicsProvider, IEpisodeRandomizable
    {
        public Vector3 meanCurrent = Vector3.zero;
        public Vector3 maxDeviation = new Vector3(0.2f, 0f, 0.2f);
        [Min(0.01f)] public float timeConstantSeconds = 15f;
        public int seed = 12345;

        System.Random _random;
        Vector3 _state;

        void Awake()
        {
            _random = new System.Random(seed);
            _state = meanCurrent;
        }

        void FixedUpdate()
        {
            float alpha = 1f - Mathf.Exp(-Time.fixedDeltaTime / Mathf.Max(0.01f, timeConstantSeconds));
            Vector3 target = meanCurrent + new Vector3(
                RandomSigned() * maxDeviation.x,
                RandomSigned() * maxDeviation.y,
                RandomSigned() * maxDeviation.z);
            _state = Vector3.Lerp(_state, target, alpha);
            currentVelocity = _state;
        }

        public void RandomizeForEpisode(RandomizationContext context)
        {
            if (context.Profile != null && context.Profile.randomizeWater)
            {
                seed = context.Seed;
                meanCurrent = context.CurrentDirection() * context.Range(context.Profile.currentSpeed);
                maxDeviation = context.Range(context.Profile.currentNoiseAmplitude);
                timeConstantSeconds = context.Range(context.Profile.currentDriftTimeConstant);
            }

            _random = new System.Random(seed);
            _state = meanCurrent;
            currentVelocity = _state;
        }

        float RandomSigned()
        {
            if (_random == null)
            {
                _random = new System.Random(seed);
            }

            return Mathf.Lerp(-1f, 1f, (float)_random.NextDouble());
        }
    }
}
