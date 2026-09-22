using System;
using UnityEngine;

namespace FinsSim.Core.Spatial
{
    /// <summary>
    /// Fossen/SNAME body vector in [u, v, w, p, q, r] order.
    /// u: surge forward, v: sway starboard, w: heave down,
    /// p: roll about surge, q: pitch about sway, r: yaw about down.
    /// </summary>
    [Serializable]
    public struct SixDofVector
    {
        public float u;
        public float v;
        public float w;
        public float p;
        public float q;
        public float r;

        public SixDofVector(float u, float v, float w, float p, float q, float r)
        {
            this.u = u;
            this.v = v;
            this.w = w;
            this.p = p;
            this.q = q;
            this.r = r;
        }

        public static SixDofVector Zero => new SixDofVector(0f, 0f, 0f, 0f, 0f, 0f);

        public float this[int index]
        {
            get
            {
                switch (index)
                {
                    case 0: return u;
                    case 1: return v;
                    case 2: return w;
                    case 3: return p;
                    case 4: return q;
                    case 5: return r;
                    default: throw new IndexOutOfRangeException(nameof(index));
                }
            }
            set
            {
                switch (index)
                {
                    case 0: u = value; break;
                    case 1: v = value; break;
                    case 2: w = value; break;
                    case 3: p = value; break;
                    case 4: q = value; break;
                    case 5: r = value; break;
                    default: throw new IndexOutOfRangeException(nameof(index));
                }
            }
        }

        public Vector3 Linear => new Vector3(u, v, w);
        public Vector3 Angular => new Vector3(p, q, r);

        public float Dot(SixDofVector other)
        {
            return u * other.u + v * other.v + w * other.w + p * other.p + q * other.q + r * other.r;
        }

        public SixDofVector Abs()
        {
            return new SixDofVector(
                Mathf.Abs(u),
                Mathf.Abs(v),
                Mathf.Abs(w),
                Mathf.Abs(p),
                Mathf.Abs(q),
                Mathf.Abs(r));
        }

        public SixDofVector ClampMagnitude(float maxLinearMagnitude, float maxAngularMagnitude)
        {
            Vector3 linear = Vector3.ClampMagnitude(Linear, maxLinearMagnitude);
            Vector3 angular = Vector3.ClampMagnitude(Angular, maxAngularMagnitude);
            return new SixDofVector(linear.x, linear.y, linear.z, angular.x, angular.y, angular.z);
        }

        public static SixDofVector Scale(SixDofVector a, SixDofVector b)
        {
            return new SixDofVector(a.u * b.u, a.v * b.v, a.w * b.w, a.p * b.p, a.q * b.q, a.r * b.r);
        }

        public static SixDofVector operator +(SixDofVector a, SixDofVector b)
        {
            return new SixDofVector(a.u + b.u, a.v + b.v, a.w + b.w, a.p + b.p, a.q + b.q, a.r + b.r);
        }

        public static SixDofVector operator -(SixDofVector a, SixDofVector b)
        {
            return new SixDofVector(a.u - b.u, a.v - b.v, a.w - b.w, a.p - b.p, a.q - b.q, a.r - b.r);
        }

        public static SixDofVector operator -(SixDofVector a)
        {
            return new SixDofVector(-a.u, -a.v, -a.w, -a.p, -a.q, -a.r);
        }

        public static SixDofVector operator *(SixDofVector a, float scalar)
        {
            return new SixDofVector(a.u * scalar, a.v * scalar, a.w * scalar, a.p * scalar, a.q * scalar, a.r * scalar);
        }

        public static SixDofVector operator *(float scalar, SixDofVector a)
        {
            return a * scalar;
        }

        public static SixDofVector operator /(SixDofVector a, float scalar)
        {
            return Mathf.Approximately(scalar, 0f) ? Zero : a * (1f / scalar);
        }

        public override string ToString()
        {
            return $"[{u:F4}, {v:F4}, {w:F4}, {p:F4}, {q:F4}, {r:F4}]";
        }
    }
}
