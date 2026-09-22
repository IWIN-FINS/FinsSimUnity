using NWH.DWP2.WaterObjects;
using UnityEngine;

namespace FinsSim.Hydrodynamics
{
    /// <summary>
    /// Private DWP2 companion for <see cref="HydrodynamicsController"/>.
    /// It prevents duplicate water forces when a FinsSim parametric backend
    /// shares a vehicle with DWP2 WaterObjects, and can select DWP2's processed
    /// simulation mesh for the geometry backend.
    /// </summary>
    [DefaultExecutionOrder(19)]
    [DisallowMultipleComponent]
    [RequireComponent(typeof(HydrodynamicsController))]
    public sealed class Dwp2HydrodynamicsInterop : MonoBehaviour
    {
        [Tooltip("Disable DWP2 coefficients when the selected FinsSim backend owns the corresponding water forces.")]
        public bool suppressDwp2ForParametricBackends = true;
        public bool zeroDwp2HydrodynamicForceCoefficient;
        public bool zeroDwp2BuoyantForceCoefficient;
        public bool useDwp2SimulationMeshWhenProfileRequestsIt = true;

        HydrodynamicsController _controller;
        WaterObject[] _waterObjects;
        Baseline[] _baselines;

        struct Baseline
        {
            public WaterObject WaterObject;
            public float HydrodynamicForceCoefficient;
            public float BuoyantForceCoefficient;
        }

        void Awake()
        {
            _controller = GetComponent<HydrodynamicsController>();
            RefreshBaselines();
            ResolveSimulationMesh();
        }

        void FixedUpdate()
        {
            EnsureBaselines();
            ResolveSimulationMesh();
            ApplyCoefficientPolicy();
        }

        void OnDisable()
        {
            RestoreAll();
        }

        void ResolveSimulationMesh()
        {
            if (!useDwp2SimulationMeshWhenProfileRequestsIt || _controller == null ||
                _controller.RuntimeProfile == null || !_controller.RuntimeProfile.preferDwp2SimulationMesh ||
                _controller.surfaceHydroMeshOverride != null || _controller.surfaceHydroMeshFilter == null)
            {
                return;
            }

            WaterObject waterObject = _controller.surfaceHydroMeshFilter.GetComponent<WaterObject>();
            if (waterObject != null && waterObject.SimulationMesh != null)
            {
                _controller.surfaceHydroMeshOverride = waterObject.SimulationMesh;
            }
        }

        void ApplyCoefficientPolicy()
        {
            if (_controller == null || !suppressDwp2ForParametricBackends ||
                _controller.mode == HydrodynamicsMode.Off)
            {
                RestoreAll();
                return;
            }

            bool equivalentBoxOwnsAllWaterForces =
                _controller.mode == HydrodynamicsMode.LearningToSwimEquivalentBox;
            bool suppressHydrodynamics = equivalentBoxOwnsAllWaterForces ||
                zeroDwp2HydrodynamicForceCoefficient;
            bool suppressBuoyancy = equivalentBoxOwnsAllWaterForces ||
                zeroDwp2BuoyantForceCoefficient;
            if (!suppressHydrodynamics && !suppressBuoyancy)
            {
                RestoreAll();
                return;
            }

            for (int i = 0; i < _baselines.Length; i++)
            {
                Baseline baseline = _baselines[i];
                if (baseline.WaterObject == null)
                {
                    continue;
                }

                baseline.WaterObject.hydrodynamicForceCoefficient = suppressHydrodynamics
                    ? 0f : baseline.HydrodynamicForceCoefficient;
                baseline.WaterObject.buoyantForceCoefficient = suppressBuoyancy
                    ? 0f : baseline.BuoyantForceCoefficient;
            }
        }

        void EnsureBaselines()
        {
            WaterObject[] current = GetComponentsInChildren<WaterObject>(true);
            if (_waterObjects == null || _waterObjects.Length != current.Length)
            {
                RefreshBaselines();
                return;
            }

            for (int i = 0; i < current.Length; i++)
            {
                if (current[i] != _waterObjects[i])
                {
                    RefreshBaselines();
                    return;
                }
            }
        }

        void RefreshBaselines()
        {
            _waterObjects = GetComponentsInChildren<WaterObject>(true);
            _baselines = new Baseline[_waterObjects.Length];
            for (int i = 0; i < _waterObjects.Length; i++)
            {
                WaterObject waterObject = _waterObjects[i];
                _baselines[i] = new Baseline
                {
                    WaterObject = waterObject,
                    HydrodynamicForceCoefficient = waterObject != null ? waterObject.hydrodynamicForceCoefficient : 0f,
                    BuoyantForceCoefficient = waterObject != null ? waterObject.buoyantForceCoefficient : 0f,
                };
            }
        }

        void RestoreAll()
        {
            if (_baselines == null)
            {
                return;
            }

            for (int i = 0; i < _baselines.Length; i++)
            {
                Baseline baseline = _baselines[i];
                if (baseline.WaterObject == null)
                {
                    continue;
                }

                baseline.WaterObject.hydrodynamicForceCoefficient = baseline.HydrodynamicForceCoefficient;
                baseline.WaterObject.buoyantForceCoefficient = baseline.BuoyantForceCoefficient;
            }
        }
    }
}
