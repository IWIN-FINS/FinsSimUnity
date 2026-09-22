using UnityEngine;

using FinsSim.Core.Spatial;

namespace FinsSim.Hydrodynamics
{
    public readonly struct RandomizationContext
    {
        readonly System.Random _random;

        public readonly DomainRandomizationProfile Profile;
        public readonly DomainRandomizationMode Mode;
        public readonly int Seed;
        public readonly int EpisodeIndex;

        public RandomizationContext(
            DomainRandomizationProfile profile,
            DomainRandomizationMode mode,
            int seed,
            int episodeIndex)
        {
            Profile = profile;
            Mode = mode;
            Seed = seed;
            EpisodeIndex = episodeIndex;
            _random = new System.Random(seed);
        }

        public float Value()
        {
            return (float)_random.NextDouble();
        }

        public float Range(FloatRange range)
        {
            return range.ClampSample(Value());
        }

        /// <summary>
        /// Returns a sample that is independent of target discovery/order.
        /// This lets multiple randomization targets share one physical latent,
        /// such as the mass scale used by both Rigidbody and buoyancy.
        /// </summary>
        public float Value(string key)
        {
            unchecked
            {
                uint hash = 2166136261u;
                hash ^= (uint)Seed;
                hash *= 16777619u;
                hash ^= (uint)EpisodeIndex;
                hash *= 16777619u;

                if (!string.IsNullOrEmpty(key))
                {
                    for (int i = 0; i < key.Length; i++)
                    {
                        hash ^= key[i];
                        hash *= 16777619u;
                    }
                }

                // Wang hash finalizer gives a stable, well-spread uint without
                // depending on runtime string or System.Random hash behavior.
                hash = (hash ^ 61u) ^ (hash >> 16);
                hash *= 9u;
                hash ^= hash >> 4;
                hash *= 0x27d4eb2du;
                hash ^= hash >> 15;
                return hash / 4294967295f;
            }
        }

        public float Range(FloatRange range, string key)
        {
            return range.ClampSample(Value(key));
        }

        public Vector3 Range(Vector3Range range)
        {
            return new Vector3(
                Mathf.Lerp(range.min.x, range.max.x, Value()),
                Mathf.Lerp(range.min.y, range.max.y, Value()),
                Mathf.Lerp(range.min.z, range.max.z, Value()));
        }

        public SixDofVector Range(SixDofVector min, SixDofVector max)
        {
            return new SixDofVector(
                Mathf.Lerp(min.u, max.u, Value()),
                Mathf.Lerp(min.v, max.v, Value()),
                Mathf.Lerp(min.w, max.w, Value()),
                Mathf.Lerp(min.p, max.p, Value()),
                Mathf.Lerp(min.q, max.q, Value()),
                Mathf.Lerp(min.r, max.r, Value()));
        }

        public int Range(IntRange range)
        {
            return range.ClampSample(Value());
        }

        public Vector3 CurrentDirection()
        {
            if (Profile == null)
            {
                return Vector3.forward;
            }

            Vector3 mean = Profile.meanCurrent.sqrMagnitude > 1e-8f
                ? Profile.meanCurrent.normalized
                : Vector3.forward;

            if (Profile.maxAngleFromMeanDegrees >= 179.9f)
            {
                if (Profile.allowVerticalCurrent)
                {
                    return RandomUnitVector();
                }

                float yaw = Mathf.Lerp(0f, Mathf.PI * 2f, Value());
                return new Vector3(Mathf.Cos(yaw), 0f, Mathf.Sin(yaw)).normalized;
            }

            Vector3 axis = Profile.allowVerticalCurrent ? RandomUnitVector() : Vector3.up;
            float angle = Mathf.Lerp(-Profile.maxAngleFromMeanDegrees, Profile.maxAngleFromMeanDegrees, Value());
            Vector3 direction = Quaternion.AngleAxis(angle, axis) * mean;
            if (!Profile.allowVerticalCurrent)
            {
                direction.y = 0f;
            }

            return direction.sqrMagnitude > 1e-8f ? direction.normalized : Vector3.forward;
        }

        Vector3 RandomUnitVector()
        {
            float z = Mathf.Lerp(-1f, 1f, Value());
            float theta = Mathf.Lerp(0f, Mathf.PI * 2f, Value());
            float radius = Mathf.Sqrt(Mathf.Max(0f, 1f - z * z));
            return new Vector3(radius * Mathf.Cos(theta), z, radius * Mathf.Sin(theta));
        }
    }
}
