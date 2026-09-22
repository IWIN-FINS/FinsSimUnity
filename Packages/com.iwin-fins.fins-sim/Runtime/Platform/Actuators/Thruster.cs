// Copyright 2022 Laboratory for Underwater Systems and Technologies (LABUST)
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

using FinsSim.Logger;
using FinsSim.Utils;
using System.Collections.Generic;
using UnityEngine;
namespace FinsSim.Actuators
{

    public class Thruster : MonoBehaviour
    {
        public enum InputModel
        {
            DatasheetCurve,
            NormalizedForce,
            NormalizedOmega
        }

        public enum ActuatorDynamicsModel
        {
            None,
            FirstOrderDelaySlew,
            PureDelay
        }

        public float LastForceRequest;
        public float TargetForceRequest;
        public float TimeSinceForceRequest = 0.0f;
        public ThrusterAsset ThrusterAsset;
        public InputModel Model = InputModel.NormalizedForce;

        [Header("Force request model")]
        public Rigidbody TargetRigidbody;
        public float MaxForwardForceN = 7.0f;
        public float MaxReverseForceN = -7.0f;
        public Vector3 LocalForceDirection = Vector3.forward;

        [Header("Runtime Debug")]
        [Tooltip("Enable debug vectors shown in the Thruster inspector. Keep disabled during normal training/playback.")]
        public bool EnableRuntimeDebugState = false;
        [Tooltip("Enable MARUS DataLogger records for every thruster command. This is expensive with many thrusters.")]
        public bool EnableRuntimeDataLogging = false;
        public string AppliedBodyName = "";
        public Vector3 LastAppliedWorldForce;
        public Vector3 LastAppliedForcePosition;

        [Header("Actuator dynamics")]
        public ActuatorDynamicsModel DynamicsModel = ActuatorDynamicsModel.None;
        public float ForceHoldSec = 1.0f;
        public float CommandDelaySec = 0.05f;
        public float ForceTimeConstantSec = 0.12f;
        public float MaxForceSlewRateNPerSec = 500.0f;
        public bool RandomizeDynamicsOnStart = false;
        public Vector2 DelayScaleRange = new Vector2(0.8f, 1.2f);
        public Vector2 TimeConstantScaleRange = new Vector2(0.8f, 1.2f);
        public Vector2 SlewRateScaleRange = new Vector2(0.8f, 1.2f);

        [Header("Omega thrust model")]
        public float MaxForwardOmegaRadS = 260.0f;
        public float MaxReverseOmegaRadS = 260.0f;
        public float C1Forward = 9.0e-5f;
        public float C1Reverse = 9.0e-5f;
        public bool RandomizeC1OnStart = false;
        public Vector2 C1ScaleRange = new Vector2(0.9f, 1.1f);

        float _commandedForceRequest;
        float _commandedNormalizedInput;
        float _c1ForwardScale = 1.0f;
        float _c1ReverseScale = 1.0f;
        float _delayScale = 1.0f;
        float _timeConstantScale = 1.0f;
        float _slewRateScale = 1.0f;
        readonly Queue<DelayedForceCommand> _pendingForceCommands = new Queue<DelayedForceCommand>();
        Rigidbody _vehicleBody;
        Transform _vehicle;
        ThrusterController _thrusterController;
        GameObjectLogger<LogRecord> _logger;

        struct DelayedForceCommand
        {
            public float DueTime;
            public float ForceN;
            public float NormalizedInput;
        }

        Transform vehicle
        {
            get
            {
                if (ResolveVehicleBody() != null)
                {
                    return _vehicleBody.transform;
                }

                return transform;
            }
        }

        void Start()
        {
            if (Application.isBatchMode)
            {
                ResolveVehicleBody();
                return;
            }

            if (RandomizeC1OnStart)
            {
                _c1ForwardScale = Random.Range(C1ScaleRange.x, C1ScaleRange.y);
                _c1ReverseScale = Random.Range(C1ScaleRange.x, C1ScaleRange.y);
            }
            if (RandomizeDynamicsOnStart)
            {
                _delayScale = Random.Range(DelayScaleRange.x, DelayScaleRange.y);
                _timeConstantScale = Random.Range(TimeConstantScaleRange.x, TimeConstantScaleRange.y);
                _slewRateScale = Random.Range(SlewRateScaleRange.x, SlewRateScaleRange.y);
            }
            ResolveVehicleBody();
            if (EnableRuntimeDataLogging)
            {
                _logger = DataLogger.Instance.GetLogger<LogRecord>($"{vehicle.transform.name}/{name}");
            }
        }

        /// <summary>
        /// Apply force to the thruster location from datasheet and standardized input
        /// </summary>
        /// <param name="normalizedInput"> -1 - 1 value</param>
        /// <returns></returns>
        public Vector3 ApplyInput(float normalizedInput)
        {
            float clampedInput = Mathf.Clamp(normalizedInput, -1.0f, 1.0f);
            float forceRequest = ForceFromNormalizedInput(clampedInput);
            return ApplyForceRequest(forceRequest, clampedInput);
        }

        public Vector3 ApplyForceRequest(float forceN)
        {
            return ApplyForceRequest(ClampForceRequest(forceN), 0.0f);
        }

        Vector3 ApplyForceRequest(float forceN, float normalizedInput)
        {
            TargetForceRequest = ClampForceRequest(forceN);
            TimeSinceForceRequest = 0.0f;

            if (UsesCommandDelay() && EffectiveCommandDelaySec() > 0.0f)
            {
                _pendingForceCommands.Enqueue(new DelayedForceCommand
                {
                    DueTime = Time.time + EffectiveCommandDelaySec(),
                    ForceN = TargetForceRequest,
                    NormalizedInput = normalizedInput
                });
            }
            else
            {
                SetCommandedForce(TargetForceRequest, normalizedInput);
            }

            if (EnableRuntimeDataLogging)
            {
                if (_logger == null)
                {
                    _logger = DataLogger.Instance.GetLogger<LogRecord>($"{vehicle.transform.name}/{name}");
                }

                _logger?.Log(new LogRecord
                {
                    NormalizedInput = normalizedInput,
                    TargetForceRequestN = TargetForceRequest,
                    AppliedForceRequestN = LastForceRequest,
                    Force = GetWorldForceDirection() * LastForceRequest
                });
            }
            return GetWorldForceDirection() * TargetForceRequest;
        }

        public float GetInputFromForce(float force)
        {
            switch (Model)
            {
                case InputModel.NormalizedForce:
                    if (force >= 0.0f)
                    {
                        float maxForwardForce = EffectiveMaxForwardForceN();
                        return maxForwardForce <= 0.0f ? 0.0f : Mathf.Clamp(force / maxForwardForce, 0.0f, 1.0f);
                    }
                    float maxReverseForce = EffectiveMaxReverseForceN();
                    return maxReverseForce <= 0.0f ? 0.0f : Mathf.Clamp(force / maxReverseForce, -1.0f, 0.0f);

                case InputModel.NormalizedOmega:
                    return InputFromOmegaForce(force);

                case InputModel.DatasheetCurve:
                default:
                    // from N to kgf
                    force /= 9.80665f;
                    var input_value = ThrusterAsset.inversedCurve.Evaluate(force);
                    return input_value + 1.0f;
            }
        }

        float ForceFromNormalizedInput(float normalizedInput)
        {
            switch (Model)
            {
                case InputModel.NormalizedForce:
                    return normalizedInput >= 0.0f
                        ? normalizedInput * EffectiveMaxForwardForceN()
                        : normalizedInput * EffectiveMaxReverseForceN();

                case InputModel.NormalizedOmega:
                    return ForceFromOmegaInput(normalizedInput);

                case InputModel.DatasheetCurve:
                default:
                    float value = ThrusterAsset.curve.Evaluate(normalizedInput);
                    // from kgf to N
                    return value * 9.80665f * EffectiveForceScaleMultiplier();
            }
        }

        float ForceFromOmegaInput(float normalizedInput)
        {
            float omega = normalizedInput >= 0.0f
                ? normalizedInput * Mathf.Abs(MaxForwardOmegaRadS)
                : normalizedInput * Mathf.Abs(MaxReverseOmegaRadS);
            float c1 = normalizedInput >= 0.0f
                ? Mathf.Abs(C1Forward) * _c1ForwardScale
                : Mathf.Abs(C1Reverse) * _c1ReverseScale;
            return c1 * omega * Mathf.Abs(omega) * EffectiveForceScaleMultiplier();
        }

        float InputFromOmegaForce(float forceN)
        {
            if (Mathf.Approximately(forceN, 0.0f))
            {
                return 0.0f;
            }

            bool forward = forceN > 0.0f;
            forceN /= EffectiveForceScaleMultiplier();
            float c1 = forward ? Mathf.Abs(C1Forward) * _c1ForwardScale : Mathf.Abs(C1Reverse) * _c1ReverseScale;
            float maxOmega = forward ? Mathf.Abs(MaxForwardOmegaRadS) : Mathf.Abs(MaxReverseOmegaRadS);
            if (c1 <= 0.0f || maxOmega <= 0.0f)
            {
                return 0.0f;
            }

            float omega = Mathf.Sqrt(Mathf.Abs(forceN) / c1);
            float normalized = Mathf.Clamp(omega / maxOmega, 0.0f, 1.0f);
            return forward ? normalized : -normalized;
        }

        float ClampForceRequest(float forceN)
        {
            return Mathf.Clamp(forceN, -EffectiveMaxReverseForceN(), EffectiveMaxForwardForceN());
        }

        float EffectiveMaxForwardForceN()
        {
            return Mathf.Abs(MaxForwardForceN) * EffectiveForceScaleMultiplier();
        }

        float EffectiveMaxReverseForceN()
        {
            return Mathf.Abs(MaxReverseForceN) * EffectiveForceScaleMultiplier();
        }

        float EffectiveForceScaleMultiplier()
        {
            if (_thrusterController == null)
            {
                _thrusterController = GetComponentInParent<ThrusterController>();
            }

            return _thrusterController != null
                ? Mathf.Max(0.0f, _thrusterController.ForceScaleMultiplier)
                : 1.0f;
        }

        float ApplyForceResponseCompression(float forceN)
        {
            if (_thrusterController == null)
            {
                _thrusterController = GetComponentInParent<ThrusterController>();
            }

            return _thrusterController != null
                ? _thrusterController.ApplyForceResponseCompression(forceN)
                : forceN;
        }

        void SetCommandedForce(float forceN, float normalizedInput)
        {
            _commandedForceRequest = ClampForceRequest(ApplyForceResponseCompression(forceN));
            _commandedNormalizedInput = normalizedInput;
        }

        void ProcessDelayedCommands()
        {
            if (!UsesCommandDelay())
            {
                _pendingForceCommands.Clear();
                return;
            }

            while (_pendingForceCommands.Count > 0 && _pendingForceCommands.Peek().DueTime <= Time.time)
            {
                DelayedForceCommand command = _pendingForceCommands.Dequeue();
                SetCommandedForce(command.ForceN, command.NormalizedInput);
            }
        }

        void UpdateAppliedForce(float deltaTime)
        {
            switch (DynamicsModel)
            {
                case ActuatorDynamicsModel.FirstOrderDelaySlew:
                    ApplyFirstOrderDelaySlew(deltaTime);
                    break;

                case ActuatorDynamicsModel.PureDelay:
                case ActuatorDynamicsModel.None:
                default:
                    LastForceRequest = _commandedForceRequest;
                    break;
            }
        }

        void ApplyFirstOrderDelaySlew(float deltaTime)
        {
            float target = _commandedForceRequest;
            float tau = EffectiveForceTimeConstantSec();
            float filtered = tau <= 0.0f
                ? target
                : Mathf.Lerp(LastForceRequest, target, 1.0f - Mathf.Exp(-deltaTime / tau));

            float slewRate = EffectiveMaxForceSlewRateNPerSec();
            if (slewRate > 0.0f)
            {
                LastForceRequest = Mathf.MoveTowards(LastForceRequest, filtered, slewRate * deltaTime);
            }
            else
            {
                LastForceRequest = filtered;
            }
        }

        bool UsesCommandDelay()
        {
            return DynamicsModel == ActuatorDynamicsModel.FirstOrderDelaySlew
                || DynamicsModel == ActuatorDynamicsModel.PureDelay;
        }

        float EffectiveCommandDelaySec()
        {
            return Mathf.Max(0.0f, CommandDelaySec * _delayScale);
        }

        float EffectiveForceTimeConstantSec()
        {
            return Mathf.Max(0.0f, ForceTimeConstantSec * _timeConstantScale);
        }

        float EffectiveMaxForceSlewRateNPerSec()
        {
            return Mathf.Max(0.0f, MaxForceSlewRateNPerSec * _slewRateScale);
        }

        Vector3 GetWorldForceDirection()
        {
            Vector3 localDirection = LocalForceDirection.sqrMagnitude > 1e-8f
                ? LocalForceDirection.normalized
                : Vector3.forward;
            return transform.TransformDirection(localDirection).normalized;
        }

        /// <summary>
        /// Returns the force currently passed to Rigidbody.AddForceAtPosition.
        /// This includes command delay, response compression, and actuator
        /// dynamics, unlike TargetForceRequest which is only the requested N.
        /// </summary>
        public Vector3 GetAppliedWorldForce()
        {
            return GetWorldForceDirection() * LastForceRequest;
        }

        public Vector3 GetAppliedForcePosition()
        {
            return transform.position;
        }

        Rigidbody ResolveVehicleBody()
        {
            if (TargetRigidbody != null)
            {
                _vehicleBody = TargetRigidbody;
                AppliedBodyName = _vehicleBody.name;
                return _vehicleBody;
            }

            if (_vehicleBody != null)
            {
                AppliedBodyName = _vehicleBody.name;
                return _vehicleBody;
            }

            _vehicle = Helpers.GetVehicle(transform);
            if (_vehicle != null)
            {
                _vehicleBody = _vehicle.GetComponent<Rigidbody>();
            }

            if (_vehicleBody == null)
            {
                _vehicleBody = Helpers.GetParentRigidBody(transform);
            }

            AppliedBodyName = _vehicleBody != null ? _vehicleBody.name : "<missing>";
            return _vehicleBody;
        }

        void FixedUpdate()
        {
            long finsSimProfileStart = FinsSimRuntimeProfiler.Begin();
            UnityEngine.Profiling.Profiler.BeginSample("FinsSim.Thruster.FixedUpdate");
            try
            {
                ProcessDelayedCommands();
                if (TimeSinceForceRequest > ForceHoldSec && _pendingForceCommands.Count == 0)
                {
                    TargetForceRequest = 0.0f;
                    SetCommandedForce(0.0f, 0.0f);
                }

                UpdateAppliedForce(Time.fixedDeltaTime);
                if (Mathf.Abs(LastForceRequest) > 1e-4f)
                {
                    Vector3 force = GetWorldForceDirection() * LastForceRequest;
                    if (EnableRuntimeDebugState)
                    {
                        LastAppliedWorldForce = force;
                        LastAppliedForcePosition = transform.position;
                    }

                    Rigidbody body = ResolveVehicleBody();
                    if (body != null)
                    {
                        body.WakeUp();
                        body.AddForceAtPosition(force, transform.position, ForceMode.Force);
                    }
                }
                else
                {
                    if (EnableRuntimeDebugState)
                    {
                        LastAppliedWorldForce = Vector3.zero;
                        LastAppliedForcePosition = transform.position;
                    }
                }
                TimeSinceForceRequest += Time.fixedDeltaTime;
            }
            finally
            {
                UnityEngine.Profiling.Profiler.EndSample();
                FinsSimRuntimeProfiler.End("FinsSim.Thruster.FixedUpdate", finsSimProfileStart);
            }
        }

        private class LogRecord
        {
            public float NormalizedInput { get; set; }
            public float TargetForceRequestN { get; set; }
            public float AppliedForceRequestN { get; set; }
            public Vector3 Force { get; set; }
        }
    }
}
