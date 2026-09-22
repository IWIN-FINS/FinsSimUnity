using FinsSim.Hydrodynamics;
using NWH.DWP2.WaterObjects;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;
using UnityEngine.Rendering.HighDefinition;

namespace NWH.DWP2.WaterData
{
    [DefaultExecutionOrder(-50)]
    public class UnityHDRPWaterDataProvider : WaterDataProvider, IWaterKinematicsProvider
    {
        const int StartBufferSize = 64;

        int bufferSize;
        NativeArray<float3> targetPositionBuffer;
        NativeArray<float3> projectedPositionBuffer;
        NativeArray<float3> candidatePositionBuffer;
        NativeArray<float3> directionBuffer;
        NativeArray<float> errorBuffer;
        NativeArray<int> stepCountBuffer;
        WaterSearchParameters searchParameters;
        WaterSearchResult searchResult;
        WaterSurface targetSurface;

        [SerializeField] WaterSurface targetSurfaceOverride;
        [SerializeField] bool sampleDynamicWaterHeight;
        [SerializeField] float fallbackWaterHeight = 0f;
        [SerializeField] float maxAllowedSearchError = 0.25f;
        [SerializeField] float maxProjectedHorizontalError = 0.05f;
        [SerializeField] bool logSearchFailures;
        [SerializeField] int lastSearchFailureCount;
        [SerializeField] Vector2 lastWaterHeightRange;

        public override void Awake()
        {
            base.Awake();

            targetSurface = targetSurfaceOverride != null
                ? targetSurfaceOverride
                : GetComponent<WaterSurface>();

            if (targetSurface == null && sampleDynamicWaterHeight)
            {
                targetSurface = FindAnyObjectByType<WaterSurface>();
                if (targetSurface == null)
                {
                    Debug.LogError(
                        $"{nameof(UnityHDRPWaterDataProvider)} requires an HDRP WaterSurface when dynamic sampling is enabled.",
                        this);
                }
            }

            bufferSize = StartBufferSize;
            targetPositionBuffer = new NativeArray<float3>(bufferSize, Allocator.Persistent);
            projectedPositionBuffer = new NativeArray<float3>(bufferSize, Allocator.Persistent);
            candidatePositionBuffer = new NativeArray<float3>(bufferSize, Allocator.Persistent);
            directionBuffer = new NativeArray<float3>(bufferSize, Allocator.Persistent);
            errorBuffer = new NativeArray<float>(bufferSize, Allocator.Persistent);
            stepCountBuffer = new NativeArray<int>(bufferSize, Allocator.Persistent);
        }

        public override bool SupportsWaterFlowQueries()
        {
            return false;
        }

        public override bool SupportsWaterHeightQueries()
        {
            return true;
        }

        public override bool SupportsWaterNormalQueries()
        {
            return false;
        }

        public override float GetWaterHeightSingle(WaterObject waterObject, Vector3 point)
        {
            if (!sampleDynamicWaterHeight)
            {
                return waterObject != null ? waterObject.defaultWaterHeight : fallbackWaterHeight;
            }

            if (targetSurface == null)
            {
                return GetFallbackHeight(waterObject);
            }

            searchParameters.startPositionWS = point;
            searchParameters.targetPositionWS = point;
            searchParameters.error = 0.01f;
            searchParameters.maxIterations = 8;

            bool success = targetSurface.ProjectPointOnWaterSurface(searchParameters, out searchResult);
            return success && IsValidHeightResult(point, searchResult.projectedPositionWS, searchResult.error)
                ? searchResult.projectedPositionWS.y
                : GetFallbackHeight(waterObject);
        }

        public WaterKinematicsSample Sample(Vector3 worldPoint)
        {
            return new WaterKinematicsSample
            {
                IsValid = true,
                Height = GetWaterHeightSingle(null, worldPoint),
                Normal = Vector3.up,
                FlowVelocity = Vector3.zero,
                AngularFlowVelocity = Vector3.zero,
            };
        }

        public override void GetWaterHeights(WaterObject waterObject, ref Vector3[] points, ref float[] waterHeights)
        {
            if (!sampleDynamicWaterHeight)
            {
                FillFallbackHeights(waterObject, waterHeights);
                return;
            }

            if (targetSurface == null)
            {
                FillFallbackHeights(waterObject, waterHeights);
                return;
            }

            var simulationData = new WaterSimSearchData();
            if (!targetSurface.FillWaterSearchData(ref simulationData))
            {
                FillFallbackHeights(waterObject, waterHeights);
                return;
            }

            int pointCount = points.Length;
            EnsureBufferSize(pointCount);

            for (int i = 0; i < pointCount; i++)
            {
                targetPositionBuffer[i] = points[i];
            }

            var job = new WaterSimulationSearchJob
            {
                simSearchData = simulationData,
                targetPositionWSBuffer = targetPositionBuffer,
                startPositionWSBuffer = targetPositionBuffer,
                projectedPositionWSBuffer = projectedPositionBuffer,
                errorBuffer = errorBuffer,
                candidateLocationWSBuffer = candidatePositionBuffer,
                directionBuffer = directionBuffer,
                stepCountBuffer = stepCountBuffer,
                error = 0.01f,
                maxIterations = 8,
            };

            JobHandle handle = job.Schedule(pointCount, 1);
            handle.Complete();

            int failureCount = 0;
            float minHeight = float.PositiveInfinity;
            float maxHeight = float.NegativeInfinity;

            for (int i = 0; i < pointCount; i++)
            {
                Vector3 point = points[i];
                float3 projected = projectedPositionBuffer[i];
                float height = projected.y;
                bool valid = IsValidHeightResult(point, projected, errorBuffer[i]);

                if (!valid && TryProjectSingle(point, out float singleHeight))
                {
                    height = singleHeight;
                    valid = true;
                }

                if (!valid)
                {
                    height = GetFallbackHeight(waterObject);
                    failureCount++;
                }

                waterHeights[i] = height;
                minHeight = Mathf.Min(minHeight, height);
                maxHeight = Mathf.Max(maxHeight, height);
            }

            lastSearchFailureCount = failureCount;
            lastWaterHeightRange = pointCount > 0 ? new Vector2(minHeight, maxHeight) : Vector2.zero;

            if (logSearchFailures && failureCount > 0)
            {
                Debug.LogWarning(
                    $"{nameof(UnityHDRPWaterDataProvider)} used fallback water height for {failureCount}/{pointCount} points.",
                    this);
            }
        }

        void EnsureBufferSize(int requestedSize)
        {
            if (requestedSize <= bufferSize)
            {
                return;
            }

            int newSize = requestedSize >= bufferSize * 2 ? requestedSize : bufferSize * 2;
            DisposeBuffers();
            bufferSize = newSize;
            targetPositionBuffer = new NativeArray<float3>(bufferSize, Allocator.Persistent);
            projectedPositionBuffer = new NativeArray<float3>(bufferSize, Allocator.Persistent);
            candidatePositionBuffer = new NativeArray<float3>(bufferSize, Allocator.Persistent);
            directionBuffer = new NativeArray<float3>(bufferSize, Allocator.Persistent);
            errorBuffer = new NativeArray<float>(bufferSize, Allocator.Persistent);
            stepCountBuffer = new NativeArray<int>(bufferSize, Allocator.Persistent);
        }

        void FillFallbackHeights(WaterObject waterObject, float[] waterHeights)
        {
            float height = GetFallbackHeight(waterObject);
            for (int i = 0; i < waterHeights.Length; i++)
            {
                waterHeights[i] = height;
            }

            lastSearchFailureCount = sampleDynamicWaterHeight ? waterHeights.Length : 0;
            lastWaterHeightRange = waterHeights.Length > 0
                ? new Vector2(height, height)
                : Vector2.zero;
        }

        bool TryProjectSingle(Vector3 point, out float height)
        {
            height = fallbackWaterHeight;

            searchParameters.startPositionWS = point;
            searchParameters.targetPositionWS = point;
            searchParameters.error = 0.01f;
            searchParameters.maxIterations = 8;

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

        static bool IsFinite(float value)
        {
            return !float.IsNaN(value) && !float.IsInfinity(value);
        }

        void OnDestroy()
        {
            DisposeBuffers();
        }

        void DisposeBuffers()
        {
            if (targetPositionBuffer.IsCreated) targetPositionBuffer.Dispose();
            if (projectedPositionBuffer.IsCreated) projectedPositionBuffer.Dispose();
            if (candidatePositionBuffer.IsCreated) candidatePositionBuffer.Dispose();
            if (directionBuffer.IsCreated) directionBuffer.Dispose();
            if (errorBuffer.IsCreated) errorBuffer.Dispose();
            if (stepCountBuffer.IsCreated) stepCountBuffer.Dispose();
        }
    }
}
