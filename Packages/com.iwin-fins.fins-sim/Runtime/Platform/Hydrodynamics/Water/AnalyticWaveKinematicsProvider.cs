using System;
using System.Collections.Generic;
using UnityEngine;

namespace FinsSim.Hydrodynamics
{
    public class AnalyticWaveKinematicsProvider : MonoBehaviour, IWaterKinematicsProvider
    {
        [Serializable]
        public class WaveComponent
        {
            [Min(0f)] public float amplitude = 0.03f;
            [Min(0.1f)] public float wavelength = 8f;
            [Min(0.1f)] public float period = 5f;
            public float directionDeg = 0f;
            public float phaseRad = 0f;
        }

        public float stillWaterHeight = 0f;
        public Vector3 steadyCurrent = Vector3.zero;
        public bool includeWaveNormals = true;
        public bool includeWaveFlow = true;
        public bool includeVerticalOrbitalFlow = true;
        [Min(0f)] public float flowScale = 1f;
        [Min(0f)] public float maxFlowSpeed = 0.25f;
        public List<WaveComponent> waves = new List<WaveComponent>
        {
            new WaveComponent { amplitude = 0.03f, wavelength = 8f, period = 5f, directionDeg = 0f },
            new WaveComponent { amplitude = 0.015f, wavelength = 3.5f, period = 2.8f, directionDeg = 55f, phaseRad = 1.7f },
        };

        public WaterKinematicsSample Sample(Vector3 worldPoint)
        {
            float height = Evaluate(worldPoint, out Vector3 normal, out Vector3 flow);
            return new WaterKinematicsSample
            {
                IsValid = true,
                Height = height,
                Normal = includeWaveNormals ? normal : Vector3.up,
                FlowVelocity = flow,
                AngularFlowVelocity = Vector3.zero,
            };
        }

        float Evaluate(Vector3 point, out Vector3 normal, out Vector3 flow)
        {
            float height = stillWaterHeight;
            float slopeX = 0f;
            float slopeZ = 0f;
            flow = steadyCurrent;
            float time = Application.isPlaying ? Time.time : 0f;

            for (int i = 0; i < waves.Count; i++)
            {
                WaveComponent wave = waves[i];
                float amplitude = Mathf.Max(0f, wave.amplitude);
                float wavelength = Mathf.Max(0.1f, wave.wavelength);
                float period = Mathf.Max(0.1f, wave.period);
                float waveNumber = 2f * Mathf.PI / wavelength;
                float angularFrequency = 2f * Mathf.PI / period;
                Vector2 direction = DirectionFromDegrees(wave.directionDeg);
                float phase = waveNumber * (direction.x * point.x + direction.y * point.z) -
                    angularFrequency * time +
                    wave.phaseRad;

                float cosPhase = Mathf.Cos(phase);
                float sinPhase = Mathf.Sin(phase);
                height += amplitude * cosPhase;

                float slope = -amplitude * waveNumber * sinPhase;
                slopeX += slope * direction.x;
                slopeZ += slope * direction.y;

                if (includeWaveFlow)
                {
                    float depthBelowSurface = Mathf.Max(0f, height - point.y);
                    float decay = Mathf.Exp(-waveNumber * depthBelowSurface);
                    float orbitalSpeed = amplitude * angularFrequency * decay * flowScale;
                    flow += new Vector3(
                        direction.x * orbitalSpeed * cosPhase,
                        includeVerticalOrbitalFlow ? orbitalSpeed * sinPhase : 0f,
                        direction.y * orbitalSpeed * cosPhase);
                }
            }

            if (maxFlowSpeed > 0f)
            {
                flow = Vector3.ClampMagnitude(flow, maxFlowSpeed);
            }

            normal = new Vector3(-slopeX, 1f, -slopeZ).normalized;
            return height;
        }

        static Vector2 DirectionFromDegrees(float directionDeg)
        {
            float radians = directionDeg * Mathf.Deg2Rad;
            return new Vector2(Mathf.Cos(radians), Mathf.Sin(radians)).normalized;
        }
    }
}
