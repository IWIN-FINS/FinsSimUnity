using System;
using System.Collections.Generic;
using NWH.DWP2.ShipController;
using UnityEngine;

namespace FinsSim.Networking
{
    /// <summary>Private DWP2 implementation of the core ROS normalized-throttle fallback.</summary>
    [DisallowMultipleComponent]
    public sealed class Dwp2ThrusterCommandFallback : MonoBehaviour, IVehicleRosThrusterFallback
    {
        static readonly string[] DefaultEngineOrder =
        {
            "Vertical1", "Vertical2", "Vertical3", "Vertical4",
            "Horizontal1", "Horizontal2", "Horizontal3", "Horizontal4",
        };

        public AdvancedShipController advancedShipController;
        public string[] engineOrder = DefaultEngineOrder;

        readonly Dictionary<string, Engine> _enginesByName = new Dictionary<string, Engine>(StringComparer.Ordinal);
        bool _loggedMissingEngineWarning;

        public bool IsAvailable
        {
            get
            {
                ResolveController();
                return advancedShipController != null && advancedShipController.engines != null;
            }
        }

        void Awake()
        {
            ResolveController();
            CacheEngines();
        }

        public bool ApplyNormalized(IList<float> values)
        {
            if (!IsAvailable || values == null)
            {
                return false;
            }

            CacheEngines();
            Zero();
            for (int index = 0; index < engineOrder.Length && index < values.Count; index++)
            {
                string engineName = engineOrder[index];
                if (!_enginesByName.TryGetValue(engineName, out Engine engine))
                {
                    if (!_loggedMissingEngineWarning)
                    {
                        Debug.LogWarning($"[{nameof(Dwp2ThrusterCommandFallback)}] Missing engine `{engineName}` on {name}.", this);
                        _loggedMissingEngineWarning = true;
                    }
                    continue;
                }

                engine.useExternalThrottleInput = true;
                engine.externalThrottleInput = Mathf.Clamp(values[index], -1f, 1f);
            }

            return true;
        }

        public void Zero()
        {
            if (!IsAvailable)
            {
                return;
            }

            foreach (Engine engine in advancedShipController.engines)
            {
                if (engine != null)
                {
                    engine.externalThrottleInput = 0f;
                }
            }
        }

        void ResolveController()
        {
            if (advancedShipController == null)
            {
                advancedShipController = GetComponent<AdvancedShipController>();
            }
        }

        void CacheEngines()
        {
            _enginesByName.Clear();
            if (advancedShipController == null || advancedShipController.engines == null)
            {
                return;
            }

            foreach (Engine engine in advancedShipController.engines)
            {
                if (engine != null && !string.IsNullOrWhiteSpace(engine.name))
                {
                    _enginesByName[engine.name] = engine;
                    engine.useExternalThrottleInput = true;
                }
            }
        }
    }
}
