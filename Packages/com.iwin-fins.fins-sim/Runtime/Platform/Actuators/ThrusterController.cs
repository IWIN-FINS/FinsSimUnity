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

using System.Collections.Generic;
using MathNet.Numerics.LinearAlgebra;
using UnityEngine;

namespace FinsSim.Actuators
{
    [System.Serializable]
    public class ThrusterRuntimeDebugInfo
    {
        public string ThrusterName;
        public float TargetForceRequestN;
        public float AppliedForceRequestN;
        public float TimeSinceForceRequestSec;
    }

    public class ThrusterController : MonoBehaviour
    {
        [Min(0f)]
        public float ForceScaleMultiplier = 1.0f;
        [Header("Force Response Compression")]
        [Tooltip("Map requested force to a lower applied force with a smooth saturation curve. TargetForceRequest keeps the raw request; LastForceRequest is the compressed applied force.")]
        public bool ForceResponseCompressionEnabled = false;
        [Tooltip("Linear gain near zero for applied = sign(request) * gain * abs(request) / (1 + saturation * abs(request)).")]
        [Min(0f)]
        public float ForceResponseCompressionLinearGain = 0.917f;
        [Tooltip("Saturation coefficient in 1/N for applied = sign(request) * gain * abs(request) / (1 + saturation * abs(request)).")]
        [Min(0f)]
        public float ForceResponseCompressionSaturationInvN = 0.079f;
        [Tooltip("Use a measured piecewise force-response curve before falling back to the two-parameter saturation formula.")]
        public bool ForceResponseCalibrationEnabled = true;
        [Tooltip("Measured mapping from requested force magnitude [N] to applied force magnitude [N]. Used only when ForceResponseCompressionEnabled and ForceResponseCalibrationEnabled are both true.")]
        public List<Vector2> ForceResponseCalibrationPoints = new List<Vector2>
        {
            new Vector2(0.0000f, 0.0000f),
            new Vector2(0.2853f, 0.4228f),
            new Vector2(0.5706f, 0.5694f),
            new Vector2(0.8559f, 0.7946f),
            new Vector2(1.1412f, 1.0165f),
            new Vector2(1.4265f, 1.2317f),
            new Vector2(1.7118f, 1.4267f),
            new Vector2(2.2824f, 1.8091f),
            new Vector2(2.8530f, 2.1813f),
            new Vector2(3.4236f, 2.5345f),
            new Vector2(4.2795f, 3.0169f),
            new Vector2(5.2305f, 3.5357f),
            new Vector2(6.1815f, 4.0155f),
            new Vector2(7.1325f, 4.4580f),
            new Vector2(7.4022f, 4.8309f),
        };
        public List<Thruster> thrusters = new List<Thruster>();

        public float ApplyForceResponseCompression(float requestedForceN)
        {
            if (!ForceResponseCompressionEnabled)
            {
                return requestedForceN;
            }

            float absForce = Mathf.Abs(requestedForceN);
            if (absForce <= 1e-6f)
            {
                return 0f;
            }

            float appliedAbsForce = ForceResponseCalibrationEnabled && ForceResponseCalibrationPoints != null && ForceResponseCalibrationPoints.Count >= 2
                ? EvaluateForceResponseCalibration(absForce)
                : EvaluateRationalForceResponseCompression(absForce);

            return Mathf.Sign(requestedForceN) * appliedAbsForce;
        }

        float EvaluateRationalForceResponseCompression(float absForce)
        {
            float denominator = 1f + Mathf.Max(0f, ForceResponseCompressionSaturationInvN) * absForce;
            return Mathf.Max(0f, ForceResponseCompressionLinearGain) * absForce / denominator;
        }

        float EvaluateForceResponseCalibration(float absForce)
        {
            Vector2 previous = ForceResponseCalibrationPoints[0];
            if (absForce <= previous.x)
            {
                return Mathf.Max(0f, previous.y);
            }

            for (int i = 1; i < ForceResponseCalibrationPoints.Count; i++)
            {
                Vector2 next = ForceResponseCalibrationPoints[i];
                if (next.x <= previous.x)
                {
                    previous = next;
                    continue;
                }

                if (absForce <= next.x)
                {
                    float t = Mathf.InverseLerp(previous.x, next.x, absForce);
                    return Mathf.Max(0f, Mathf.Lerp(previous.y, next.y, t));
                }

                previous = next;
            }

            return Mathf.Max(0f, previous.y);
        }

        [Header("Runtime Debug")]
        [Tooltip("Enable Inspector-facing per-thruster command statistics. Keep disabled during normal training/playback.")]
        public bool EnableRuntimeThrusterStats = false;
        [Tooltip("Enable per-frame net force/torque aggregation for debugging. Keep disabled during normal training/playback.")]
        public bool EnableRuntimeNetWrenchStats = false;
        [SerializeField] List<ThrusterRuntimeDebugInfo> runtimeThrusterStates = new List<ThrusterRuntimeDebugInfo>();
        [SerializeField] Vector3 runtimeNetForce;
        [SerializeField] Vector3 runtimeNetTorque;
        [SerializeField] Vector3 runtimeLocalNetTorque;
        [SerializeField] float runtimeYawTorqueY;
        [SerializeField] float runtimeLocalYawTorqueY;
        bool lastAppliedThrusterDebugState;

        public void ApplyInput(float[] array)
        {
            for (int i = 0; i < thrusters.Count; i++)
            {
                if (i < array.Length)
                {
                    thrusters[i].ApplyInput(array[i]);
                }
                else
                {
                    break;
                }
            }
        }

        public void ApplyInput(Vector3 tau, Matrix<double> inverseAllocationMatrix)
        {
            var vec = CreateVector.Dense<double>(3);
            vec[0] = tau.x;
            vec[1] = tau.y;
            vec[2] = tau.z;
            var forces = inverseAllocationMatrix.Multiply(vec);

            for (int i = 0; i < thrusters.Count; i++)
            {
                if (i < forces.Count)
                {
                    thrusters[i].ApplyForceRequest((float)forces[i]);
                }
                else
                {
                    break;
                }
            }

        }

        void LateUpdate()
        {
            if (!EnableRuntimeThrusterStats && !EnableRuntimeNetWrenchStats)
            {
                ApplyThrusterDebugStateIfNeeded(false);
                return;
            }

            if (EnableRuntimeThrusterStats)
            {
                UpdateRuntimeThrusterStates();
            }

            if (EnableRuntimeNetWrenchStats)
            {
                ApplyThrusterDebugStateIfNeeded(true);
                UpdateNetForceDebug();
            }
            else
            {
                ApplyThrusterDebugStateIfNeeded(false);
            }
        }

        void ApplyThrusterDebugStateIfNeeded(bool enabled)
        {
            if (lastAppliedThrusterDebugState == enabled)
            {
                return;
            }

            lastAppliedThrusterDebugState = enabled;
            for (int i = 0; i < thrusters.Count; i++)
            {
                if (thrusters[i] != null)
                {
                    thrusters[i].EnableRuntimeDebugState = enabled;
                }
            }
        }

        void UpdateRuntimeThrusterStates()
        {
            if (runtimeThrusterStates == null)
            {
                runtimeThrusterStates = new List<ThrusterRuntimeDebugInfo>();
            }

            while (runtimeThrusterStates.Count < thrusters.Count)
            {
                runtimeThrusterStates.Add(new ThrusterRuntimeDebugInfo());
            }

            while (runtimeThrusterStates.Count > thrusters.Count)
            {
                runtimeThrusterStates.RemoveAt(runtimeThrusterStates.Count - 1);
            }

            for (int i = 0; i < thrusters.Count; i++)
            {
                Thruster thruster = thrusters[i];
                ThrusterRuntimeDebugInfo state = runtimeThrusterStates[i];
                if (thruster == null)
                {
                    state.ThrusterName = "<missing>";
                    state.TargetForceRequestN = 0f;
                    state.AppliedForceRequestN = 0f;
                    state.TimeSinceForceRequestSec = 0f;
                    continue;
                }

                state.ThrusterName = thruster.name;
                state.TargetForceRequestN = thruster.TargetForceRequest;
                state.AppliedForceRequestN = thruster.LastForceRequest;
                state.TimeSinceForceRequestSec = thruster.TimeSinceForceRequest;
            }
        }

        void UpdateNetForceDebug()
        {
            runtimeNetForce = Vector3.zero;
            runtimeNetTorque = Vector3.zero;

            Rigidbody body = GetComponent<Rigidbody>();
            Vector3 torqueOrigin = body != null ? body.worldCenterOfMass : transform.position;

            for (int i = 0; i < thrusters.Count; i++)
            {
                Thruster thruster = thrusters[i];
                if (thruster == null)
                {
                    continue;
                }

                Vector3 force = thruster.LastAppliedWorldForce;
                Vector3 leverArm = thruster.LastAppliedForcePosition - torqueOrigin;
                runtimeNetForce += force;
                runtimeNetTorque += Vector3.Cross(leverArm, force);
            }

            runtimeYawTorqueY = runtimeNetTorque.y;
            runtimeLocalNetTorque = transform.InverseTransformDirection(runtimeNetTorque);
            runtimeLocalYawTorqueY = runtimeLocalNetTorque.y;
        }
    }
}
