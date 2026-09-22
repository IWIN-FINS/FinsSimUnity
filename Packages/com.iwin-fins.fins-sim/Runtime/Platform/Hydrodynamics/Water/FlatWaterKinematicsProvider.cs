using UnityEngine;

namespace FinsSim.Hydrodynamics
{
    public class FlatWaterKinematicsProvider : MonoBehaviour, IAreaWaterKinematicsProvider
    {
        public float waterHeight = 0f;
        public Vector3 currentVelocity = Vector3.zero;
        public Vector3 angularCurrentVelocity = Vector3.zero;

        public virtual WaterKinematicsSample Sample(Vector3 worldPoint)
        {
            return new WaterKinematicsSample
            {
                IsValid = true,
                Height = waterHeight,
                Normal = Vector3.up,
                FlowVelocity = currentVelocity,
                AngularFlowVelocity = angularCurrentVelocity,
            };
        }

        public void SetCurrent(Vector3 worldVelocity)
        {
            currentVelocity = worldVelocity;
        }

        public void SetAreaWaterHeight(float height)
        {
            waterHeight = height;
        }
    }
}
