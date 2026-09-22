using System;
using UnityEngine;

using FinsSim.Core.Spatial;

namespace FinsSim.Hydrodynamics
{
    public enum DomainRandomizationMode
    {
        Disabled,
        Train,
        Evaluate
    }

    [Serializable]
    public struct FloatRange
    {
        public float min;
        public float max;

        public FloatRange(float min, float max)
        {
            this.min = min;
            this.max = max;
        }

        public float ClampSample(float t)
        {
            return Mathf.Lerp(Mathf.Min(min, max), Mathf.Max(min, max), Mathf.Clamp01(t));
        }
    }

    [Serializable]
    public struct IntRange
    {
        public int min;
        public int max;

        public IntRange(int min, int max)
        {
            this.min = min;
            this.max = max;
        }

        public int ClampSample(float t)
        {
            int lower = Mathf.Min(min, max);
            int upper = Mathf.Max(min, max);
            return Mathf.RoundToInt(Mathf.Lerp(lower, upper, Mathf.Clamp01(t)));
        }
    }

    [Serializable]
    public struct Vector3Range
    {
        public Vector3 min;
        public Vector3 max;

        public Vector3Range(Vector3 min, Vector3 max)
        {
            this.min = min;
            this.max = max;
        }
    }

    [CreateAssetMenu(menuName = "Marus/Hydrodynamics/Domain Randomization Profile", fileName = "DomainRandomizationProfile")]
    public class DomainRandomizationProfile : ScriptableObject
    {
        [Header("Body")]
        public bool randomizeBody = true;
        public FloatRange massScale = new FloatRange(0.8f, 1.2f);
        public FloatRange volumeScale = new FloatRange(0.9f, 1.1f);
        public FloatRange inertiaScale = new FloatRange(0.8f, 1.2f);
        [Tooltip("When enabled, displaced volume uses the same episode mass scale so the baseline buoyancy ratio is preserved.")]
        public bool coupleMassAndVolumeForNeutralBuoyancy;
        [Tooltip("Optional trim applied on top of the coupled mass scale. Keep at 1 for neutral buoyancy.")]
        public FloatRange buoyancyTrimScale = new FloatRange(1f, 1f);
        public Vector3Range centerOfMassOffset = new Vector3Range(Vector3.zero, Vector3.zero);
        public Vector3Range centerOfBuoyancyOffset = new Vector3Range(new Vector3(0f, -0.01f, 0f), new Vector3(0f, 0.01f, 0f));

        [Header("Hydrodynamics")]
        public bool randomizeHydrodynamics = true;
        public FloatRange addedMassScale = new FloatRange(0.5f, 1.0f);
        public FloatRange linearDampingScale = new FloatRange(0.5f, 1.0f);
        public FloatRange quadraticDampingScale = new FloatRange(0.5f, 1.0f);

        [Header("Hydrodynamics Axis Scales [u, v, w, p, q, r]")]
        public bool randomizeHydrodynamicAxisScales;
        public SixDofVector addedMassAxisScaleMin = new SixDofVector(1f, 1f, 1f, 1f, 1f, 1f);
        public SixDofVector addedMassAxisScaleMax = new SixDofVector(1f, 1f, 1f, 1f, 1f, 1f);
        public SixDofVector linearDampingAxisScaleMin = new SixDofVector(1f, 1f, 1f, 1f, 1f, 0.8f);
        public SixDofVector linearDampingAxisScaleMax = new SixDofVector(1f, 1f, 1f, 1f, 1f, 1.4f);
        public SixDofVector quadraticDampingAxisScaleMin = new SixDofVector(1f, 1f, 1f, 1f, 1f, 0.6f);
        public SixDofVector quadraticDampingAxisScaleMax = new SixDofVector(1f, 1f, 1f, 1f, 1f, 2.0f);

        [Header("Thrusters")]
        public bool randomizeThrusters = true;
        [Tooltip("Common all-thruster efficiency / supply scale. The per-thruster maxForceScale adds independent asymmetry on top.")]
        public FloatRange commonMaxForceScale = new FloatRange(1f, 1f);
        public FloatRange maxForceScale = new FloatRange(0.8f, 1.2f);
        public FloatRange forceConstantScale = new FloatRange(0.8f, 1.2f);
        public FloatRange timeConstantScale = new FloatRange(0.8f, 1.2f);
        public FloatRange delayScale = new FloatRange(0.8f, 1.2f);
        public FloatRange slewRateScale = new FloatRange(0.8f, 1.2f);

        [Header("Water Current")]
        public bool randomizeWater = true;
        public FloatRange currentSpeed = new FloatRange(0f, 0.35f);
        public bool allowVerticalCurrent;
        public Vector3 meanCurrent = Vector3.zero;
        [Range(0f, 180f)] public float maxAngleFromMeanDegrees = 180f;
        public Vector3Range currentNoiseAmplitude = new Vector3Range(Vector3.zero, new Vector3(0.05f, 0.05f, 0.05f));
        public FloatRange currentDriftTimeConstant = new FloatRange(8f, 30f);

        [Header("Water Waves")]
        public bool randomizeWaves = true;
        public IntRange waveComponentCount = new IntRange(1, 3);
        public FloatRange waveAmplitude = new FloatRange(0f, 0.05f);
        public FloatRange waveWavelength = new FloatRange(2.5f, 12f);
        public FloatRange wavePeriod = new FloatRange(2f, 7f);
        public FloatRange waveDirectionDeg = new FloatRange(0f, 360f);
        public bool randomizeWavePhase = true;
        public FloatRange waveFlowScale = new FloatRange(0.5f, 1.2f);
        public FloatRange waveMaxFlowSpeed = new FloatRange(0.1f, 0.35f);

        [Header("Initial State")]
        public bool randomizeInitialState = true;
        public Vector3Range initialPositionOffset = new Vector3Range(new Vector3(-1f, -0.5f, -1f), new Vector3(1f, 0.5f, 1f));
        public Vector3Range initialEulerOffsetDeg = new Vector3Range(new Vector3(-5f, -180f, -5f), new Vector3(5f, 180f, 5f));
        public Vector3Range initialLinearVelocity = new Vector3Range(new Vector3(-0.1f, -0.05f, -0.1f), new Vector3(0.1f, 0.05f, 0.1f));
        public Vector3Range initialAngularVelocity = new Vector3Range(new Vector3(-0.05f, -0.05f, -0.05f), new Vector3(0.05f, 0.05f, 0.05f));
    }
}
