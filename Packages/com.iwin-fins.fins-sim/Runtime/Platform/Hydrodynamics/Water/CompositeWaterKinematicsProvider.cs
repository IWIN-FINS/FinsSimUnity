using System.Collections.Generic;
using UnityEngine;

namespace FinsSim.Hydrodynamics
{
    public class CompositeWaterKinematicsProvider : MonoBehaviour, IWaterKinematicsProvider, IEpisodeRandomizable
    {
        public List<MonoBehaviour> providerBehaviours = new List<MonoBehaviour>();
        public bool autoFindChildProviders = true;
        public float fallbackHeight = 0f;

        readonly List<IWaterKinematicsProvider> _providers = new List<IWaterKinematicsProvider>();
        readonly List<IEpisodeRandomizable> _randomizables = new List<IEpisodeRandomizable>();

        void Awake()
        {
            ResolveProviders();
        }

        public WaterKinematicsSample Sample(Vector3 worldPoint)
        {
            ResolveProviders();
            if (_providers.Count == 0)
            {
                return WaterKinematicsSample.Flat(fallbackHeight, Vector3.zero);
            }

            bool hasHeight = false;
            float height = fallbackHeight;
            Vector3 normal = Vector3.zero;
            Vector3 flow = Vector3.zero;
            Vector3 angularFlow = Vector3.zero;

            for (int i = 0; i < _providers.Count; i++)
            {
                WaterKinematicsSample sample = _providers[i].Sample(worldPoint);
                if (!sample.IsValid)
                {
                    continue;
                }

                if (!hasHeight)
                {
                    height = sample.Height;
                    hasHeight = true;
                }

                normal += sample.Normal.sqrMagnitude > 1e-8f ? sample.Normal : Vector3.up;
                flow += sample.FlowVelocity;
                angularFlow += sample.AngularFlowVelocity;
            }

            return new WaterKinematicsSample
            {
                IsValid = true,
                Height = height,
                Normal = normal.sqrMagnitude > 1e-8f ? normal.normalized : Vector3.up,
                FlowVelocity = flow,
                AngularFlowVelocity = angularFlow,
            };
        }

        public void RandomizeForEpisode(RandomizationContext context)
        {
            ResolveProviders();
            for (int i = 0; i < _randomizables.Count; i++)
            {
                _randomizables[i].RandomizeForEpisode(context);
            }
        }

        public void ResolveProviders()
        {
            _providers.Clear();
            _randomizables.Clear();

            for (int i = 0; i < providerBehaviours.Count; i++)
            {
                AddProvider(providerBehaviours[i]);
            }

            if (!autoFindChildProviders)
            {
                return;
            }

            foreach (MonoBehaviour behaviour in GetComponentsInChildren<MonoBehaviour>(true))
            {
                if (behaviour == this)
                {
                    continue;
                }

                AddProvider(behaviour);
            }
        }

        void AddProvider(MonoBehaviour behaviour)
        {
            if (behaviour is IWaterKinematicsProvider provider && !_providers.Contains(provider))
            {
                _providers.Add(provider);
            }

            if (behaviour is IEpisodeRandomizable randomizable && !_randomizables.Contains(randomizable))
            {
                _randomizables.Add(randomizable);
            }
        }
    }
}
