using NWH.DWP2.WaterData;
using UnityEngine;

namespace FinsSim.Hydrodynamics
{
    public class Dwp2WaterKinematicsAdapter : MonoBehaviour, IWaterKinematicsProvider
    {
        public WaterDataProvider provider;
        public float fallbackHeight = 0f;
        public Vector3 fallbackNormal = Vector3.up;
        public Vector3 fallbackFlow = Vector3.zero;

        Vector3[] _points = new Vector3[1];
        float[] _heights = new float[1];
        Vector3[] _vectors = new Vector3[1];

        void Awake()
        {
            if (provider == null)
            {
                provider = GetComponent<WaterDataProvider>();
            }
        }

        public WaterKinematicsSample Sample(Vector3 worldPoint)
        {
            if (provider == null)
            {
                return WaterKinematicsSample.Flat(fallbackHeight, fallbackFlow);
            }

            _points[0] = worldPoint;
            float height = fallbackHeight;
            Vector3 normal = fallbackNormal;
            Vector3 flow = fallbackFlow;

            if (provider.SupportsWaterHeightQueries())
            {
                provider.GetWaterHeights(null, ref _points, ref _heights);
                height = _heights[0];
            }

            if (provider.SupportsWaterNormalQueries())
            {
                provider.GetWaterNormals(null, ref _points, ref _vectors);
                normal = _vectors[0].sqrMagnitude > 1e-8f ? _vectors[0].normalized : Vector3.up;
            }

            if (provider.SupportsWaterFlowQueries())
            {
                provider.GetWaterFlows(null, ref _points, ref _vectors);
                flow = _vectors[0];
            }

            return new WaterKinematicsSample
            {
                IsValid = true,
                Height = height,
                Normal = normal,
                FlowVelocity = flow,
                AngularFlowVelocity = Vector3.zero,
            };
        }
    }
}
