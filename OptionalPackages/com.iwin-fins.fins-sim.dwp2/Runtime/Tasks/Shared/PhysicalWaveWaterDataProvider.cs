using System;
using System.Collections.Generic;
using FinsSim.Hydrodynamics;
using NWH.DWP2.WaterObjects;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Rendering.HighDefinition;

namespace NWH.DWP2.WaterData
{
    [DefaultExecutionOrder(-45)]
    public class PhysicalWaveWaterDataProvider : WaterDataProvider, IAreaWaterKinematicsProvider, IEpisodeRandomizable
    {
        public enum WavePhysicsMode
        {
            MinimalHDRPHeightOnly = 0,
            AnalyticPhysicalWave = 1,
            FlatFallback = 2,
        }

        [Serializable]
        public class WaveComponent
        {
            [Min(0f)] public float amplitude = 0.03f;
            [Min(0.1f)] public float wavelength = 8f;
            [Min(0.1f)] public float period = 5f;
            public float directionDeg = 0f;
            public float phaseRad = 0f;
        }

        [Header("Mode")]
        public WavePhysicsMode mode = WavePhysicsMode.AnalyticPhysicalWave;

        [Header("Minimal HDRP Height Only")]
        public WaterSurface targetSurfaceOverride;
        public float fallbackWaterHeight = 0f;
        [Min(0.001f)] public float hdrpSearchError = 0.01f;
        [Min(1)] public int hdrpSearchIterations = 8;
        [Min(0f)] public float maxAllowedSearchError = 0.25f;
        [Min(0f)] public float maxProjectedHorizontalError = 0.05f;

        [Header("Analytic Physical Waves")]
        public float stillWaterHeight = 0f;
        public bool includeWaveNormals = true;
        public bool includeWaveFlow = true;
        public bool includeVerticalOrbitalFlow = true;
        [Min(0f)] public float flowScale = 1f;
        [Min(0f)] public float maxFlowSpeed = 0.25f;
        public Vector3 steadyCurrent = Vector3.zero;
        public List<WaveComponent> waves = new List<WaveComponent>
        {
            new WaveComponent { amplitude = 0.03f, wavelength = 8f, period = 5f, directionDeg = 0f, phaseRad = 0f },
            new WaveComponent { amplitude = 0.015f, wavelength = 3.5f, period = 2.8f, directionDeg = 55f, phaseRad = 1.7f },
        };

        [Header("Episode Domain Randomization")]
        public bool randomizeWaterCurrentFromProfile = true;
        public bool randomizeWavesFromProfile = true;
        public bool forceAnalyticModeWhenRandomizingWaves = true;
        public bool logEpisodeRandomization;

        [Header("Debug")]
        [SerializeField] int lastSearchFailureCount;
        [SerializeField] Vector2 lastWaterHeightRange;
        [SerializeField] Vector3 lastWaterFlow;
        [SerializeField] Vector3 lastRandomizedSteadyCurrent;
        [SerializeField] int lastRandomizedWaveCount;

        WaterSurface targetSurface;
        WaterSearchParameters searchParameters;

        public override void Awake()
        {
            base.Awake();
            ResolveTargetSurface();
        }

        public override bool SupportsWaterHeightQueries()
        {
            return true;
        }

        public override bool SupportsWaterNormalQueries()
        {
            return mode == WavePhysicsMode.AnalyticPhysicalWave && includeWaveNormals;
        }

        public override bool SupportsWaterFlowQueries()
        {
            return mode == WavePhysicsMode.AnalyticPhysicalWave && includeWaveFlow;
        }

        public override float GetWaterHeightSingle(WaterObject waterObject, Vector3 point)
        {
            return mode switch
            {
                WavePhysicsMode.MinimalHDRPHeightOnly => GetHDRPHeight(waterObject, point),
                WavePhysicsMode.AnalyticPhysicalWave => EvaluateAnalyticWave(point, out _, out _),
                _ => GetFallbackHeight(waterObject),
            };
        }

        public WaterKinematicsSample Sample(Vector3 worldPoint)
        {
            float height = GetWaterHeightSingle(null, worldPoint);
            Vector3 normal = Vector3.up;
            Vector3 flow = Vector3.zero;

            if (mode == WavePhysicsMode.AnalyticPhysicalWave)
            {
                height = EvaluateAnalyticWave(worldPoint, out normal, out flow);
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

        public void SetAreaWaterHeight(float height)
        {
            stillWaterHeight = height;
            fallbackWaterHeight = height;
        }

        public override void GetWaterHeights(WaterObject waterObject, ref Vector3[] points, ref float[] waterHeights)
        {
            int count = Mathf.Min(points.Length, waterHeights.Length);
            float minHeight = float.PositiveInfinity;
            float maxHeight = float.NegativeInfinity;
            int failureCount = 0;

            for (int i = 0; i < count; i++)
            {
                float height;
                if (mode == WavePhysicsMode.MinimalHDRPHeightOnly)
                {
                    bool success = TryGetHDRPHeight(points[i], out height);
                    if (!success)
                    {
                        height = GetFallbackHeight(waterObject);
                        failureCount++;
                    }
                }
                else if (mode == WavePhysicsMode.AnalyticPhysicalWave)
                {
                    height = EvaluateAnalyticWave(points[i], out _, out _);
                }
                else
                {
                    height = GetFallbackHeight(waterObject);
                }

                waterHeights[i] = height;
                minHeight = Mathf.Min(minHeight, height);
                maxHeight = Mathf.Max(maxHeight, height);
            }

            lastSearchFailureCount = failureCount;
            lastWaterHeightRange = count > 0 ? new Vector2(minHeight, maxHeight) : Vector2.zero;
        }

        public override void GetWaterNormals(WaterObject waterObject, ref Vector3[] points, ref Vector3[] waterNormals)
        {
            int count = Mathf.Min(points.Length, waterNormals.Length);
            for (int i = 0; i < count; i++)
            {
                EvaluateAnalyticWave(points[i], out Vector3 normal, out _);
                waterNormals[i] = normal;
            }
        }

        public override void GetWaterFlows(WaterObject waterObject, ref Vector3[] points, ref Vector3[] waterFlows)
        {
            int count = Mathf.Min(points.Length, waterFlows.Length);
            for (int i = 0; i < count; i++)
            {
                EvaluateAnalyticWave(points[i], out _, out Vector3 flow);
                waterFlows[i] = flow;
                lastWaterFlow = flow;
            }
        }

        public void RandomizeForEpisode(RandomizationContext context)
        {
            DomainRandomizationProfile profile = context.Profile;
            if (profile == null)
            {
                return;
            }

            if (randomizeWaterCurrentFromProfile && profile.randomizeWater)
            {
                RandomizeSteadyCurrent(context);
            }

            if (randomizeWavesFromProfile && profile.randomizeWaves)
            {
                RandomizeWaves(context);
            }

            if (logEpisodeRandomization)
            {
                Debug.Log(
                    $"[{nameof(PhysicalWaveWaterDataProvider)}] episode={context.EpisodeIndex}, "
                    + $"mode={mode}, steady_current={steadyCurrent:F3}, waves={waves.Count}, "
                    + $"flow_scale={flowScale:F2}, max_flow={maxFlowSpeed:F2}",
                    this);
            }
        }

        void ResolveTargetSurface()
        {
            targetSurface = targetSurfaceOverride != null
                ? targetSurfaceOverride
                : GetComponent<WaterSurface>();

            if (targetSurface == null && mode == WavePhysicsMode.MinimalHDRPHeightOnly)
            {
                targetSurface = FindAnyObjectByType<WaterSurface>();
            }
        }

        float GetHDRPHeight(WaterObject waterObject, Vector3 point)
        {
            return TryGetHDRPHeight(point, out float height) ? height : GetFallbackHeight(waterObject);
        }

        bool TryGetHDRPHeight(Vector3 point, out float height)
        {
            height = fallbackWaterHeight;

            if (targetSurface == null)
            {
                ResolveTargetSurface();
                if (targetSurface == null)
                {
                    return false;
                }
            }

            searchParameters.startPositionWS = point;
            searchParameters.targetPositionWS = point;
            searchParameters.error = hdrpSearchError;
            searchParameters.maxIterations = hdrpSearchIterations;

            if (!targetSurface.ProjectPointOnWaterSurface(searchParameters, out WaterSearchResult result))
            {
                return false;
            }

            if (!IsValidHeightResult(point, result.projectedPositionWS, result.error))
            {
                return false;
            }

            height = result.projectedPositionWS.y;
            return true;
        }

        float EvaluateAnalyticWave(Vector3 point, out Vector3 normal, out Vector3 flow)
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
                float phase = waveNumber * (direction.x * point.x + direction.y * point.z)
                    - angularFrequency * time
                    + wave.phaseRad;

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
                    Vector3 orbitalFlow = new Vector3(
                        direction.x * orbitalSpeed * cosPhase,
                        includeVerticalOrbitalFlow ? orbitalSpeed * sinPhase : 0f,
                        direction.y * orbitalSpeed * cosPhase);
                    flow += orbitalFlow;
                }
            }

            normal = new Vector3(-slopeX, 1f, -slopeZ).normalized;

            if (maxFlowSpeed > 0f && flow.magnitude > maxFlowSpeed)
            {
                flow = flow.normalized * maxFlowSpeed;
            }

            return height;
        }

        void RandomizeSteadyCurrent(RandomizationContext context)
        {
            DomainRandomizationProfile profile = context.Profile;
            Vector3 current = context.CurrentDirection() * Mathf.Max(0f, context.Range(profile.currentSpeed));
            if (!profile.allowVerticalCurrent)
            {
                current.y = 0f;
            }

            steadyCurrent = current;
            lastRandomizedSteadyCurrent = current;
        }

        void RandomizeWaves(RandomizationContext context)
        {
            DomainRandomizationProfile profile = context.Profile;
            if (forceAnalyticModeWhenRandomizingWaves)
            {
                mode = WavePhysicsMode.AnalyticPhysicalWave;
            }

            flowScale = Mathf.Max(0f, context.Range(profile.waveFlowScale));
            maxFlowSpeed = Mathf.Max(0f, context.Range(profile.waveMaxFlowSpeed));

            int waveCount = Mathf.Max(0, context.Range(profile.waveComponentCount));
            waves.Clear();

            for (int i = 0; i < waveCount; i++)
            {
                waves.Add(new WaveComponent
                {
                    amplitude = Mathf.Max(0f, context.Range(profile.waveAmplitude)),
                    wavelength = Mathf.Max(0.1f, context.Range(profile.waveWavelength)),
                    period = Mathf.Max(0.1f, context.Range(profile.wavePeriod)),
                    directionDeg = NormalizeDegrees(context.Range(profile.waveDirectionDeg)),
                    phaseRad = profile.randomizeWavePhase ? context.Value() * Mathf.PI * 2f : 0f,
                });
            }

            lastRandomizedWaveCount = waves.Count;
        }

        float GetFallbackHeight(WaterObject waterObject)
        {
            return waterObject != null ? waterObject.defaultWaterHeight : fallbackWaterHeight;
        }

        bool IsValidHeightResult(Vector3 targetPoint, float3 projectedPoint, float error)
        {
            float height = projectedPoint.y;
            if (!IsFinite(height) || !IsFinite(error) || error > maxAllowedSearchError)
            {
                return false;
            }

            Vector2 targetXZ = new Vector2(targetPoint.x, targetPoint.z);
            Vector2 projectedXZ = new Vector2(projectedPoint.x, projectedPoint.z);
            return Vector2.Distance(targetXZ, projectedXZ) <= maxProjectedHorizontalError;
        }

        static Vector2 DirectionFromDegrees(float degrees)
        {
            float radians = degrees * Mathf.Deg2Rad;
            return new Vector2(Mathf.Cos(radians), Mathf.Sin(radians)).normalized;
        }

        static float NormalizeDegrees(float degrees)
        {
            degrees %= 360f;
            return degrees < 0f ? degrees + 360f : degrees;
        }

        static bool IsFinite(float value)
        {
            return !float.IsNaN(value) && !float.IsInfinity(value);
        }

        void OnValidate()
        {
            hdrpSearchIterations = Mathf.Max(1, hdrpSearchIterations);
            hdrpSearchError = Mathf.Max(0.001f, hdrpSearchError);
            maxAllowedSearchError = Mathf.Max(0f, maxAllowedSearchError);
            maxProjectedHorizontalError = Mathf.Max(0f, maxProjectedHorizontalError);
            maxFlowSpeed = Mathf.Max(0f, maxFlowSpeed);
            flowScale = Mathf.Max(0f, flowScale);

            for (int i = 0; i < waves.Count; i++)
            {
                waves[i].amplitude = Mathf.Max(0f, waves[i].amplitude);
                waves[i].wavelength = Mathf.Max(0.1f, waves[i].wavelength);
                waves[i].period = Mathf.Max(0.1f, waves[i].period);
            }
        }
    }
}
