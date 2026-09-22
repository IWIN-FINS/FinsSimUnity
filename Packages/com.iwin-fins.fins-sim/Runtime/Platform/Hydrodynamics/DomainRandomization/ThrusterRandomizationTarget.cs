using FinsSim.Actuators;
using UnityEngine;

namespace FinsSim.Hydrodynamics
{
    public class ThrusterRandomizationTarget : MonoBehaviour, IEpisodeRandomizable
    {
        public Thruster[] thrusters;
        ThrusterBaseline[] _baselines;

        struct ThrusterBaseline
        {
            public float MaxForwardForceN;
            public float MaxReverseForceN;
            public float C1Forward;
            public float C1Reverse;
            public float CommandDelaySec;
            public float ForceTimeConstantSec;
            public float MaxForceSlewRateNPerSec;
        }

        void Awake()
        {
            CaptureBaseline();
        }

        public void CaptureBaseline()
        {
            if (thrusters == null || thrusters.Length == 0)
            {
                thrusters = GetComponentsInChildren<Thruster>(true);
            }

            _baselines = new ThrusterBaseline[thrusters.Length];
            for (int i = 0; i < thrusters.Length; i++)
            {
                Thruster thruster = thrusters[i];
                if (thruster == null)
                {
                    continue;
                }

                _baselines[i] = new ThrusterBaseline
                {
                    MaxForwardForceN = thruster.MaxForwardForceN,
                    MaxReverseForceN = thruster.MaxReverseForceN,
                    C1Forward = thruster.C1Forward,
                    C1Reverse = thruster.C1Reverse,
                    CommandDelaySec = thruster.CommandDelaySec,
                    ForceTimeConstantSec = thruster.ForceTimeConstantSec,
                    MaxForceSlewRateNPerSec = thruster.MaxForceSlewRateNPerSec,
                };
            }
        }

        public void RandomizeForEpisode(RandomizationContext context)
        {
            if (context.Profile == null || !context.Profile.randomizeThrusters)
            {
                return;
            }

            if (thrusters == null || _baselines == null || _baselines.Length != thrusters.Length)
            {
                CaptureBaseline();
            }

            for (int i = 0; i < thrusters.Length; i++)
            {
                Thruster thruster = thrusters[i];
                if (thruster == null)
                {
                    continue;
                }

                ThrusterBaseline baseline = _baselines[i];
                float commonMaxForceScale = context.Range(
                    context.Profile.commonMaxForceScale,
                    "thrusters.common-max-force-scale");
                float individualMaxForceScale = context.Range(
                    context.Profile.maxForceScale,
                    $"thrusters.{i}.max-force-scale");
                float maxForceScale = commonMaxForceScale * individualMaxForceScale;
                float forceConstantScale = context.Range(
                    context.Profile.forceConstantScale,
                    $"thrusters.{i}.force-constant-scale");
                thruster.MaxForwardForceN = baseline.MaxForwardForceN * maxForceScale;
                thruster.MaxReverseForceN = baseline.MaxReverseForceN * maxForceScale;
                thruster.C1Forward = baseline.C1Forward * forceConstantScale;
                thruster.C1Reverse = baseline.C1Reverse * forceConstantScale;
                thruster.CommandDelaySec = baseline.CommandDelaySec * context.Range(
                    context.Profile.delayScale,
                    $"thrusters.{i}.delay-scale");
                thruster.ForceTimeConstantSec = baseline.ForceTimeConstantSec * context.Range(
                    context.Profile.timeConstantScale,
                    $"thrusters.{i}.time-constant-scale");
                thruster.MaxForceSlewRateNPerSec = baseline.MaxForceSlewRateNPerSec * context.Range(
                    context.Profile.slewRateScale,
                    $"thrusters.{i}.slew-rate-scale");
            }
        }
    }
}
