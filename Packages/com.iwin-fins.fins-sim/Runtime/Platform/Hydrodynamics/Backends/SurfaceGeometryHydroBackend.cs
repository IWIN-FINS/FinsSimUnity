using System.Collections.Generic;
using UnityEngine;

using FinsSim.Core.Spatial;

namespace FinsSim.Hydrodynamics
{
    /// <summary>
    /// Stonefish-style geometry backend: closed-hull volume supplies hydrostatics while
    /// distributed triangles supply form and skin drag. Added mass remains a Fossen concern.
    /// </summary>
    public sealed class SurfaceGeometryHydroBackend : HydrodynamicsBackendBase
    {
        const float VolumeEpsilon = 1e-7f;

        readonly bool _includeHydrostatics;
        readonly List<SubmergedVertex> _polygon = new List<SubmergedVertex>(3);
        readonly List<SubmergedVertex> _clipped = new List<SubmergedVertex>(4);

        Mesh _cachedMesh;
        MeshProperties _cachedProperties;
        Mesh _warnedMesh;

        struct SubmergedVertex
        {
            public Vector3 Position;
            public float Depth;

            public SubmergedVertex(Vector3 position, float depth)
            {
                Position = position;
                Depth = depth;
            }
        }

        struct MeshProperties
        {
            public bool IsClosedEnough;
            public float SignedVolume;
            public Vector3 LocalCenterOfBuoyancy;
        }

        public SurfaceGeometryHydroBackend(bool includeHydrostatics = true)
        {
            _includeHydrostatics = includeHydrostatics;
        }

        public override HydrodynamicsWrench Compute(in HydrodynamicsContext context)
        {
            HydrodynamicsProfile profile = context.Profile;
            Mesh mesh = ResolvePhysicsMesh(context, profile);
            if (profile == null || mesh == null || context.Body == null)
            {
                return _includeHydrostatics ? ComputeHydrostatic(context) : HydrodynamicsWrench.Zero;
            }

            Transform meshTransform = Controller != null ? Controller.SurfaceHydroTransform : context.Transform;
            Vector3[] vertices = mesh.vertices;
            int[] triangles = mesh.triangles;
            int triangleCount = triangles.Length / 3;
            if (vertices.Length == 0 || triangleCount == 0)
            {
                return _includeHydrostatics ? ComputeHydrostatic(context) : HydrodynamicsWrench.Zero;
            }

            MeshProperties meshProperties = GetMeshProperties(mesh);
            float orientation = (meshProperties.SignedVolume < 0f ? -1f : 1f) *
                GetTransformOrientation(meshTransform);
            bool fullySubmerged = IsFullySubmerged(context, meshTransform, vertices);

            float submergedVolume = 0f;
            Vector3 submergedCenter = context.Transform.TransformPoint(profile.centerOfBuoyancy);
            float wettedArea = 0f;

            if (_includeHydrostatics)
            {
                if (profile.useMeshVolumeForBuoyancy && meshProperties.IsClosedEnough)
                {
                    if (fullySubmerged)
                    {
                        submergedVolume = ComputeWorldVolume(meshProperties.SignedVolume, meshTransform);
                        submergedCenter = meshTransform.TransformPoint(meshProperties.LocalCenterOfBuoyancy);
                    }
                    else
                    {
                        ComputePartialSubmergedVolume(
                            context,
                            meshTransform,
                            vertices,
                            triangles,
                            orientation,
                            out submergedVolume,
                            out submergedCenter);
                    }
                }

                // A non-closed render mesh must never make a vehicle sink indefinitely.
                if (submergedVolume <= VolumeEpsilon)
                {
                    float submergence = fullySubmerged ? 1f : profile.ComputeSubmergence(context.WorldCenterOfMass, context.WaterSample);
                    submergedVolume = profile.displacedVolume * submergence;
                    submergedCenter = context.Transform.TransformPoint(profile.centerOfBuoyancy);
                }

                submergedVolume *= Mathf.Max(0f, profile.meshBuoyancyVolumeScale) *
                    Mathf.Max(0f, profile.pressureBuoyancyScale);
            }

            Vector3 formForce = Vector3.zero;
            Vector3 formTorque = Vector3.zero;
            Vector3 skinForce = Vector3.zero;
            Vector3 skinTorque = Vector3.zero;
            int dynamicCount = Mathf.Min(triangleCount, Mathf.Max(1, profile.maxSurfaceTriangles));
            float dynamicWeight = triangleCount / (float)dynamicCount;
            if (triangleCount > dynamicCount && _warnedMesh != mesh)
            {
                Debug.LogWarning(
                    $"[{nameof(SurfaceGeometryHydroBackend)}] Mesh `{mesh.name}` has {triangleCount} triangles; " +
                    $"sampling {dynamicCount} evenly for drag. Hydrostatic volume still uses the complete physics mesh.",
                    Controller);
                _warnedMesh = mesh;
            }

            int nextSample = 0;
            int nextTriangle = 0;
            for (int triangleIndex = 0; triangleIndex < triangleCount && nextSample < dynamicCount; triangleIndex++)
            {
                if (triangleIndex != nextTriangle)
                {
                    continue;
                }

                int baseIndex = triangleIndex * 3;
                SubmergedVertex a = CreateVertex(context, meshTransform.TransformPoint(vertices[triangles[baseIndex]]));
                SubmergedVertex b = CreateVertex(context, meshTransform.TransformPoint(vertices[triangles[baseIndex + 1]]));
                SubmergedVertex c = CreateVertex(context, meshTransform.TransformPoint(vertices[triangles[baseIndex + 2]]));
                AccumulateDragForClippedTriangle(
                    context,
                    profile,
                    a,
                    b,
                    c,
                    orientation,
                    dynamicWeight,
                    ref formForce,
                    ref formTorque,
                    ref skinForce,
                    ref skinTorque,
                    ref wettedArea);

                nextSample++;
                nextTriangle = nextSample * triangleCount / dynamicCount;
            }

            Vector3 dynamicForce = ScaleWorldForceByBodyAxis(context, formForce, profile.formDragAxisScale) +
                ScaleWorldForceByBodyAxis(context, skinForce, profile.skinDragAxisScale);
            Vector3 dynamicTorque = ScaleWorldTorqueByBodyAxis(context, formTorque, profile.formDragTorqueAxisScale) +
                ScaleWorldTorqueByBodyAxis(context, skinTorque, profile.skinDragTorqueAxisScale);

            HydrodynamicsWrench result = _includeHydrostatics
                ? ComputeHydrostatic(context, submergedVolume, submergedCenter)
                : HydrodynamicsWrench.Zero;
            result.WorldForce += dynamicForce;
            result.WorldTorque += dynamicTorque;
            result.BodyWrench = HydroMath.UnityWorldVelocityToFossen(
                context.Transform,
                context.AxisConvention,
                result.WorldForce,
                result.WorldTorque);
            result.SubmergedVolume = _includeHydrostatics ? submergedVolume : 0f;
            result.WettedArea = wettedArea;
            return result;
        }

        Mesh ResolvePhysicsMesh(in HydrodynamicsContext context, HydrodynamicsProfile profile)
        {
            return context.SurfaceMesh;
        }

        MeshProperties GetMeshProperties(Mesh mesh)
        {
            if (_cachedMesh == mesh)
            {
                return _cachedProperties;
            }

            Vector3[] vertices = mesh.vertices;
            int[] triangles = mesh.triangles;
            double signedVolume = 0d;
            Vector3 centroidMoment = Vector3.zero;
            var edgeCounts = new Dictionary<ulong, int>(triangles.Length);

            for (int i = 0; i + 2 < triangles.Length; i += 3)
            {
                int ia = triangles[i];
                int ib = triangles[i + 1];
                int ic = triangles[i + 2];
                if (ia < 0 || ib < 0 || ic < 0 || ia >= vertices.Length || ib >= vertices.Length || ic >= vertices.Length)
                {
                    continue;
                }

                AddEdge(edgeCounts, ia, ib);
                AddEdge(edgeCounts, ib, ic);
                AddEdge(edgeCounts, ic, ia);

                Vector3 a = vertices[ia];
                Vector3 b = vertices[ib];
                Vector3 c = vertices[ic];
                float tetraVolume = Vector3.Dot(a, Vector3.Cross(b, c)) / 6f;
                signedVolume += tetraVolume;
                centroidMoment += (a + b + c) * (tetraVolume * 0.25f);
            }

            float volume = (float)signedVolume;
            bool isClosed = true;
            foreach (int count in edgeCounts.Values)
            {
                if (count != 2)
                {
                    isClosed = false;
                    break;
                }
            }
            _cachedProperties = new MeshProperties
            {
                IsClosedEnough = isClosed && Mathf.Abs(volume) > VolumeEpsilon,
                SignedVolume = volume,
                LocalCenterOfBuoyancy = Mathf.Abs(volume) > VolumeEpsilon
                    ? centroidMoment / volume
                    : Vector3.zero,
            };
            _cachedMesh = mesh;
            return _cachedProperties;
        }

        static void AddEdge(Dictionary<ulong, int> edgeCounts, int first, int second)
        {
            uint a = (uint)Mathf.Min(first, second);
            uint b = (uint)Mathf.Max(first, second);
            ulong key = ((ulong)a << 32) | b;
            edgeCounts.TryGetValue(key, out int count);
            edgeCounts[key] = count + 1;
        }

        static float ComputeWorldVolume(float localSignedVolume, Transform meshTransform)
        {
            Vector3 x = meshTransform.TransformVector(Vector3.right);
            Vector3 y = meshTransform.TransformVector(Vector3.up);
            Vector3 z = meshTransform.TransformVector(Vector3.forward);
            float determinant = Vector3.Dot(x, Vector3.Cross(y, z));
            return Mathf.Abs(localSignedVolume * determinant);
        }

        static float GetTransformOrientation(Transform meshTransform)
        {
            Vector3 x = meshTransform.TransformVector(Vector3.right);
            Vector3 y = meshTransform.TransformVector(Vector3.up);
            Vector3 z = meshTransform.TransformVector(Vector3.forward);
            return Vector3.Dot(x, Vector3.Cross(y, z)) < 0f ? -1f : 1f;
        }

        bool IsFullySubmerged(in HydrodynamicsContext context, Transform meshTransform, Vector3[] vertices)
        {
            for (int i = 0; i < vertices.Length; i++)
            {
                SubmergedVertex vertex = CreateVertex(context, meshTransform.TransformPoint(vertices[i]));
                if (vertex.Depth < 0f)
                {
                    return false;
                }
            }

            return true;
        }

        static SubmergedVertex CreateVertex(in HydrodynamicsContext context, Vector3 point)
        {
            WaterKinematicsSample water = context.WaterProvider != null
                ? context.WaterProvider.Sample(point)
                : context.WaterSample;
            return new SubmergedVertex(point, water.Height - point.y);
        }

        void ComputePartialSubmergedVolume(
            in HydrodynamicsContext context,
            Transform meshTransform,
            Vector3[] vertices,
            int[] triangles,
            float orientation,
            out float submergedVolume,
            out Vector3 submergedCenter)
        {
            Vector3 referencePoint = context.WorldCenterOfMass;
            WaterKinematicsSample referenceWater = context.WaterProvider != null
                ? context.WaterProvider.Sample(referencePoint)
                : context.WaterSample;
            referencePoint.y = referenceWater.Height;

            float volume = 0f;
            Vector3 centroidMoment = Vector3.zero;
            for (int i = 0; i + 2 < triangles.Length; i += 3)
            {
                SubmergedVertex a = CreateVertex(context, meshTransform.TransformPoint(vertices[triangles[i]]));
                SubmergedVertex b = CreateVertex(context, meshTransform.TransformPoint(vertices[triangles[i + 1]]));
                SubmergedVertex c = CreateVertex(context, meshTransform.TransformPoint(vertices[triangles[i + 2]]));
                ClipTriangle(a, b, c);

                for (int fan = 1; fan < _clipped.Count - 1; fan++)
                {
                    Vector3 p0 = _clipped[0].Position;
                    Vector3 p1 = _clipped[fan].Position;
                    Vector3 p2 = _clipped[fan + 1].Position;
                    float tetraVolume = orientation * Vector3.Dot(
                        p0 - referencePoint,
                        Vector3.Cross(p1 - referencePoint, p2 - referencePoint)) / 6f;
                    volume += tetraVolume;
                    centroidMoment += (referencePoint + p0 + p1 + p2) * (tetraVolume * 0.25f);
                }
            }

            submergedVolume = Mathf.Max(0f, volume);
            submergedCenter = submergedVolume > VolumeEpsilon
                ? centroidMoment / submergedVolume
                : context.Transform.TransformPoint(context.Profile.centerOfBuoyancy);
        }

        void AccumulateDragForClippedTriangle(
            in HydrodynamicsContext context,
            HydrodynamicsProfile profile,
            SubmergedVertex a,
            SubmergedVertex b,
            SubmergedVertex c,
            float orientation,
            float sampleWeight,
            ref Vector3 formForceTotal,
            ref Vector3 formTorqueTotal,
            ref Vector3 skinForceTotal,
            ref Vector3 skinTorqueTotal,
            ref float wettedArea)
        {
            ClipTriangle(a, b, c);
            for (int fan = 1; fan < _clipped.Count - 1; fan++)
            {
                Vector3 p0 = _clipped[0].Position;
                Vector3 p1 = _clipped[fan].Position;
                Vector3 p2 = _clipped[fan + 1].Position;
                Vector3 cross = Vector3.Cross(p1 - p0, p2 - p0);
                float doubleArea = cross.magnitude;
                if (doubleArea < profile.minTriangleArea * 2f)
                {
                    continue;
                }

                float area = doubleArea * 0.5f;
                Vector3 normal = orientation * cross / doubleArea;
                Vector3 center = (p0 + p1 + p2) / 3f;
                WaterKinematicsSample water = context.WaterProvider != null
                    ? context.WaterProvider.Sample(center)
                    : context.WaterSample;
                Vector3 relativeVelocity = water.FlowVelocity - context.Body.GetPointVelocity(center);
                float normalVelocity = Vector3.Dot(relativeVelocity, normal);
                Vector3 formForce = Vector3.zero;
                Vector3 skinForce = Vector3.zero;

                // Same decomposition as Stonefish: approaching normal flow is quadratic;
                // tangential flow is linear skin friction. Coefficients apply after geometry integration.
                if (normalVelocity < -1e-5f)
                {
                    float speed = relativeVelocity.magnitude;
                    formForce = 0.5f * profile.waterDensity * profile.formDragCoefficient *
                        relativeVelocity * speed * (-normalVelocity) * area;
                }

                Vector3 tangentVelocity = relativeVelocity - normalVelocity * normal;
                if (tangentVelocity.sqrMagnitude > 1e-8f)
                {
                    skinForce = profile.waterDensity * profile.skinDragCoefficient * tangentVelocity * area;
                }

                formForce *= sampleWeight;
                skinForce *= sampleWeight;
                formForceTotal += formForce;
                formTorqueTotal += Vector3.Cross(center - context.WorldCenterOfMass, formForce);
                skinForceTotal += skinForce;
                skinTorqueTotal += Vector3.Cross(center - context.WorldCenterOfMass, skinForce);
                wettedArea += area * sampleWeight;
            }
        }

        static Vector3 ScaleWorldForceByBodyAxis(
            in HydrodynamicsContext context,
            Vector3 worldForce,
            Vector3 axisScale)
        {
            SixDofVector body = HydroMath.UnityWorldVelocityToFossen(
                context.Transform,
                context.AxisConvention,
                worldForce,
                Vector3.zero);
            body.u *= Mathf.Max(0f, axisScale.x);
            body.v *= Mathf.Max(0f, axisScale.y);
            body.w *= Mathf.Max(0f, axisScale.z);
            HydroMath.FossenBodyWrenchToUnityWorld(
                context.Transform,
                context.AxisConvention,
                body,
                out Vector3 scaledForce,
                out _);
            return scaledForce;
        }

        static Vector3 ScaleWorldTorqueByBodyAxis(
            in HydrodynamicsContext context,
            Vector3 worldTorque,
            Vector3 axisScale)
        {
            SixDofVector body = HydroMath.UnityWorldVelocityToFossen(
                context.Transform,
                context.AxisConvention,
                Vector3.zero,
                worldTorque);
            body.p *= Mathf.Max(0f, axisScale.x);
            body.q *= Mathf.Max(0f, axisScale.y);
            body.r *= Mathf.Max(0f, axisScale.z);
            HydroMath.FossenBodyWrenchToUnityWorld(
                context.Transform,
                context.AxisConvention,
                body,
                out _,
                out Vector3 scaledTorque);
            return scaledTorque;
        }

        void ClipTriangle(SubmergedVertex a, SubmergedVertex b, SubmergedVertex c)
        {
            _polygon.Clear();
            _polygon.Add(a);
            _polygon.Add(b);
            _polygon.Add(c);
            _clipped.Clear();

            SubmergedVertex previous = _polygon[_polygon.Count - 1];
            bool previousInside = previous.Depth >= 0f;
            for (int i = 0; i < _polygon.Count; i++)
            {
                SubmergedVertex current = _polygon[i];
                bool currentInside = current.Depth >= 0f;
                if (currentInside != previousInside)
                {
                    float denominator = previous.Depth - current.Depth;
                    float t = Mathf.Approximately(denominator, 0f)
                        ? 0f
                        : Mathf.Clamp01(previous.Depth / denominator);
                    _clipped.Add(new SubmergedVertex(
                        Vector3.Lerp(previous.Position, current.Position, t),
                        0f));
                }

                if (currentInside)
                {
                    _clipped.Add(current);
                }

                previous = current;
                previousInside = currentInside;
            }
        }
    }
}
