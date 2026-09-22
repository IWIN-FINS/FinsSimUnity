using UnityEngine;

using FinsSim.Core.Spatial;

namespace FinsSim.Hydrodynamics
{
    public static class HydroMath
    {
        public static SixDofVector UnityWorldVelocityToFossen(
            Transform bodyTransform,
            Vector3 worldLinearVelocity,
            Vector3 worldAngularVelocity)
        {
            return UnityWorldVelocityToFossen(
                bodyTransform,
                BodyAxisConvention.LegacyHydroMathZForwardXRightYUp,
                worldLinearVelocity,
                worldAngularVelocity);
        }

        public static SixDofVector UnityWorldVelocityToFossen(
            Transform bodyTransform,
            BodyAxisConvention axisConvention,
            Vector3 worldLinearVelocity,
            Vector3 worldAngularVelocity)
        {
            Vector3 localLinear = bodyTransform != null
                ? bodyTransform.InverseTransformDirection(worldLinearVelocity)
                : worldLinearVelocity;
            Vector3 localAngular = bodyTransform != null
                ? bodyTransform.InverseTransformDirection(worldAngularVelocity)
                : worldAngularVelocity;

            if (axisConvention == BodyAxisConvention.FinsRovXForwardYUpZLeft)
            {
                return new SixDofVector(
                    localLinear.x,
                    localLinear.z,
                    -localLinear.y,
                    localAngular.x,
                    localAngular.z,
                    localAngular.y);
            }

            return new SixDofVector(
                localLinear.z,
                localLinear.x,
                -localLinear.y,
                localAngular.z,
                localAngular.x,
                -localAngular.y);
        }

        public static void FossenBodyWrenchToUnityWorld(
            Transform bodyTransform,
            SixDofVector bodyWrench,
            out Vector3 worldForce,
            out Vector3 worldTorque)
        {
            FossenBodyWrenchToUnityWorld(
                bodyTransform,
                BodyAxisConvention.LegacyHydroMathZForwardXRightYUp,
                bodyWrench,
                out worldForce,
                out worldTorque);
        }

        public static void FossenBodyWrenchToUnityWorld(
            Transform bodyTransform,
            BodyAxisConvention axisConvention,
            SixDofVector bodyWrench,
            out Vector3 worldForce,
            out Vector3 worldTorque)
        {
            Vector3 localForce;
            Vector3 localTorque;
            if (axisConvention == BodyAxisConvention.FinsRovXForwardYUpZLeft)
            {
                localForce = new Vector3(bodyWrench.u, -bodyWrench.w, bodyWrench.v);
                localTorque = new Vector3(bodyWrench.p, bodyWrench.r, bodyWrench.q);
            }
            else
            {
                localForce = new Vector3(bodyWrench.v, -bodyWrench.w, bodyWrench.u);
                localTorque = new Vector3(bodyWrench.q, -bodyWrench.r, bodyWrench.p);
            }

            if (bodyTransform != null)
            {
                worldForce = bodyTransform.TransformDirection(localForce);
                worldTorque = bodyTransform.TransformDirection(localTorque);
            }
            else
            {
                worldForce = localForce;
                worldTorque = localTorque;
            }
        }

        public static SixDofVector ComputeDiagonalDamping(
            SixDofVector linearDamping,
            SixDofVector quadraticDamping,
            SixDofVector forwardSpeedDamping,
            SixDofVector relativeVelocity)
        {
            SixDofVector absNu = relativeVelocity.Abs();
            float absU = Mathf.Abs(relativeVelocity.u);
            var result = SixDofVector.Zero;

            for (int i = 0; i < 6; i++)
            {
                float coefficient =
                    Mathf.Abs(linearDamping[i]) +
                    Mathf.Abs(quadraticDamping[i]) * absNu[i] +
                    Mathf.Abs(forwardSpeedDamping[i]) * absU;
                result[i] = -coefficient * relativeVelocity[i];
            }

            return result;
        }

        public static SixDofVector BuildFullAddedMassCoriolisProduct(
            SixDofMatrix positiveAddedMass,
            SixDofVector velocity)
        {
            Vector3 v1 = velocity.Linear;
            Vector3 v2 = velocity.Angular;

            Vector3 a1 = new Vector3(
                positiveAddedMass[0, 0] * v1.x + positiveAddedMass[0, 1] * v1.y + positiveAddedMass[0, 2] * v1.z +
                positiveAddedMass[0, 3] * v2.x + positiveAddedMass[0, 4] * v2.y + positiveAddedMass[0, 5] * v2.z,
                positiveAddedMass[1, 0] * v1.x + positiveAddedMass[1, 1] * v1.y + positiveAddedMass[1, 2] * v1.z +
                positiveAddedMass[1, 3] * v2.x + positiveAddedMass[1, 4] * v2.y + positiveAddedMass[1, 5] * v2.z,
                positiveAddedMass[2, 0] * v1.x + positiveAddedMass[2, 1] * v1.y + positiveAddedMass[2, 2] * v1.z +
                positiveAddedMass[2, 3] * v2.x + positiveAddedMass[2, 4] * v2.y + positiveAddedMass[2, 5] * v2.z);

            Vector3 a2 = new Vector3(
                positiveAddedMass[3, 0] * v1.x + positiveAddedMass[3, 1] * v1.y + positiveAddedMass[3, 2] * v1.z +
                positiveAddedMass[3, 3] * v2.x + positiveAddedMass[3, 4] * v2.y + positiveAddedMass[3, 5] * v2.z,
                positiveAddedMass[4, 0] * v1.x + positiveAddedMass[4, 1] * v1.y + positiveAddedMass[4, 2] * v1.z +
                positiveAddedMass[4, 3] * v2.x + positiveAddedMass[4, 4] * v2.y + positiveAddedMass[4, 5] * v2.z,
                positiveAddedMass[5, 0] * v1.x + positiveAddedMass[5, 1] * v1.y + positiveAddedMass[5, 2] * v1.z +
                positiveAddedMass[5, 3] * v2.x + positiveAddedMass[5, 4] * v2.y + positiveAddedMass[5, 5] * v2.z);

            Vector3 linear = -Vector3.Cross(a1, v2);
            Vector3 angular = -Vector3.Cross(a1, v1) - Vector3.Cross(a2, v2);
            return new SixDofVector(linear.x, linear.y, linear.z, angular.x, angular.y, angular.z);
        }

        public static float ClampFinite(float value, float fallback = 0f)
        {
            return float.IsNaN(value) || float.IsInfinity(value) ? fallback : value;
        }

        public static Vector3 ClampFinite(Vector3 value)
        {
            return new Vector3(
                ClampFinite(value.x),
                ClampFinite(value.y),
                ClampFinite(value.z));
        }

        public static SixDofVector ClampFinite(SixDofVector value)
        {
            for (int i = 0; i < 6; i++)
            {
                value[i] = ClampFinite(value[i]);
            }

            return value;
        }

        public static HydrodynamicsWrench ClampWrench(HydrodynamicsWrench wrench, float maxForce, float maxTorque)
        {
            if (maxForce > 0f)
            {
                wrench.WorldForce = Vector3.ClampMagnitude(wrench.WorldForce, maxForce);
            }

            if (maxTorque > 0f)
            {
                wrench.WorldTorque = Vector3.ClampMagnitude(wrench.WorldTorque, maxTorque);
            }

            wrench.WorldForce = ClampFinite(wrench.WorldForce);
            wrench.WorldTorque = ClampFinite(wrench.WorldTorque);
            wrench.BodyWrench = ClampFinite(wrench.BodyWrench);
            return wrench;
        }
    }
}
