using System;
using System.Collections.Generic;
using System.Globalization;
using System.Text;
using FinsSim.Actuators;
using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;
using UnityEngine.SceneManagement;

public static class FinsROVChaseStaticAudit
{
    const string OneChaseOneScenePath = "Assets/FinsSimUnity/Tasks/Chase/OneChaseOne/Scenes/1Chase1.unity";
    static readonly string[] CanonicalThrusterNames =
    {
        "Vertical1", "Vertical2", "Vertical3", "Vertical4",
        "Horizontal1", "Horizontal2", "Horizontal3", "Horizontal4",
    };

    [MenuItem("FinsSim/Audit/1Chase1 Direct Thruster Axes")]
    public static void AuditOneChaseOneDirectThrusterAxes()
    {
        EditorSceneManager.OpenScene(OneChaseOneScenePath, OpenSceneMode.Single);
        OneChaseOnePoseAgent agent = UnityEngine.Object.FindFirstObjectByType<OneChaseOnePoseAgent>();
        if (agent == null || agent.selfTransform == null)
        {
            throw new InvalidOperationException("1Chase1 must contain an enabled OneChaseOnePoseAgent with selfTransform assigned.");
        }

        Rigidbody body = agent.GetComponent<Rigidbody>();
        if (body == null)
        {
            throw new InvalidOperationException("OneChaseOnePoseAgent must share its FinsROV Rigidbody.");
        }

        Thruster[] thrusters = ResolveCanonicalThrusters(agent);
        BodyWrench negativeYaw = ComputeBodyWrench(
            agent.selfTransform,
            body.worldCenterOfMass,
            thrusters,
            new[] { 0f, 0f, 0f, 0f, -1f, -1f, -1f, -1f });
        BodyWrench forward = ComputeBodyWrench(
            agent.selfTransform,
            body.worldCenterOfMass,
            thrusters,
            new[] { 0f, 0f, 0f, 0f, 1f, 1f, -1f, -1f });
        BodyWrench positiveHeave = ComputeBodyWrench(
            agent.selfTransform,
            body.worldCenterOfMass,
            thrusters,
            new[] { 1f, 1f, 1f, 1f, 0f, 0f, 0f, 0f });

        float horizontalForceBound = SumHorizontalForceMagnitudes(thrusters);
        float horizontalForceTolerance = Mathf.Max(1e-3f, horizontalForceBound * 0.02f);
        float yawTorqueBound = SumYawTorqueMagnitudes(body.worldCenterOfMass, thrusters);
        float yawTorqueTolerance = Mathf.Max(1e-3f, yawTorqueBound * 0.02f);

        Debug.Log(
            "[FinsROVChaseStaticAudit] measured " +
            $"reference={agent.selfTransform.name}; negativeYaw={negativeYaw}; forward={forward}; positiveHeave={positiveHeave}; " +
            $"forceTolerance={horizontalForceTolerance.ToString("F6", CultureInfo.InvariantCulture)}; " +
            $"yawTolerance={yawTorqueTolerance.ToString("F6", CultureInfo.InvariantCulture)}\n" +
            DescribeThrusters(agent.selfTransform, body.worldCenterOfMass, thrusters) +
            DescribeAllocationColumns(agent.selfTransform, body.worldCenterOfMass, thrusters));

        RequireNearZero(negativeYaw.force.x, horizontalForceTolerance, "negative-yaw surge");
        RequireNearZero(negativeYaw.force.z, horizontalForceTolerance, "negative-yaw sway");
        Require(negativeYaw.torque.y < -yawTorqueTolerance,
            $"all-negative horizontal commands must produce negative body yaw, got {negativeYaw.torque.y:F6} Nm.");
        Require(forward.force.x > horizontalForceTolerance,
            $"forward command must produce positive body surge, got {forward.force.x:F6} N.");
        RequireNearZero(forward.force.z, horizontalForceTolerance, "forward sway");
        RequireNearZero(forward.torque.y, yawTorqueTolerance, "forward yaw");
        Require(positiveHeave.force.y > horizontalForceTolerance,
            $"positive vertical commands must produce positive body heave, got {positiveHeave.force.y:F6} N.");

        Debug.Log(
            "[FinsROVChaseStaticAudit] PASS " +
            $"reference={agent.selfTransform.name}; negativeYaw={negativeYaw}; forward={forward}; positiveHeave={positiveHeave}; " +
            $"forceTolerance={horizontalForceTolerance.ToString("F6", CultureInfo.InvariantCulture)}; " +
            $"yawTolerance={yawTorqueTolerance.ToString("F6", CultureInfo.InvariantCulture)}\n" +
            DescribeThrusters(agent.selfTransform, body.worldCenterOfMass, thrusters) +
            DescribeAllocationColumns(agent.selfTransform, body.worldCenterOfMass, thrusters));
    }

    static Thruster[] ResolveCanonicalThrusters(OneChaseOnePoseAgent agent)
    {
        var byName = new Dictionary<string, Thruster>(StringComparer.Ordinal);
        foreach (Thruster thruster in agent.GetComponentsInChildren<Thruster>(true))
        {
            if (thruster != null && !byName.ContainsKey(thruster.name))
            {
                byName.Add(thruster.name, thruster);
            }
        }

        var result = new Thruster[CanonicalThrusterNames.Length];
        for (int i = 0; i < CanonicalThrusterNames.Length; i++)
        {
            if (!byName.TryGetValue(CanonicalThrusterNames[i], out Thruster thruster))
            {
                throw new InvalidOperationException($"Missing canonical thruster {CanonicalThrusterNames[i]} in 1Chase1.");
            }

            result[i] = thruster;
        }

        return result;
    }

    static BodyWrench ComputeBodyWrench(Transform reference, Vector3 centerOfMass, Thruster[] thrusters, float[] actions)
    {
        Vector3 worldForce = Vector3.zero;
        Vector3 worldTorque = Vector3.zero;
        for (int i = 0; i < thrusters.Length; i++)
        {
            Thruster thruster = thrusters[i];
            float action = i < actions.Length ? Mathf.Clamp(actions[i], -1f, 1f) : 0f;
            float maxForce = action >= 0f
                ? Mathf.Abs(thruster.MaxForwardForceN)
                : Mathf.Abs(thruster.MaxReverseForceN);
            Vector3 localDirection = thruster.LocalForceDirection.sqrMagnitude > 1e-8f
                ? thruster.LocalForceDirection.normalized
                : Vector3.forward;
            Vector3 force = thruster.transform.TransformDirection(localDirection).normalized * (action * maxForce);
            worldForce += force;
            worldTorque += Vector3.Cross(thruster.transform.position - centerOfMass, force);
        }

        return new BodyWrench(
            ControllerBodyFrame.WorldToBodyVector(reference, worldForce),
            ControllerBodyFrame.WorldToBodyVector(reference, worldTorque)
        );
    }

    static float SumHorizontalForceMagnitudes(Thruster[] thrusters)
    {
        float sum = 0f;
        for (int i = 4; i < thrusters.Length; i++)
        {
            sum += Mathf.Max(Mathf.Abs(thrusters[i].MaxForwardForceN), Mathf.Abs(thrusters[i].MaxReverseForceN));
        }

        return sum;
    }

    static float SumYawTorqueMagnitudes(Vector3 centerOfMass, Thruster[] thrusters)
    {
        float sum = 0f;
        for (int i = 4; i < thrusters.Length; i++)
        {
            float force = Mathf.Max(Mathf.Abs(thrusters[i].MaxForwardForceN), Mathf.Abs(thrusters[i].MaxReverseForceN));
            sum += Vector3.Distance(thrusters[i].transform.position, centerOfMass) * force;
        }

        return sum;
    }

    static string DescribeThrusters(Transform reference, Vector3 centerOfMass, Thruster[] thrusters)
    {
        var report = new StringBuilder();
        for (int i = 4; i < thrusters.Length; i++)
        {
            Thruster thruster = thrusters[i];
            Vector3 direction = thruster.transform.TransformDirection(thruster.LocalForceDirection).normalized;
            Vector3 bodyDirection = ControllerBodyFrame.WorldToBodyVector(reference, direction);
            Vector3 bodyPosition = ControllerBodyFrame.WorldToBodyVector(reference, thruster.transform.position - centerOfMass);
            report.AppendLine(
                $"  {thruster.name}: bodyPosition={bodyPosition}, bodyForceDirection={bodyDirection}, " +
                $"maxForward={thruster.MaxForwardForceN:F3}, maxReverse={thruster.MaxReverseForceN:F3}");
        }

        return report.ToString();
    }

    static string DescribeAllocationColumns(Transform reference, Vector3 centerOfMass, Thruster[] thrusters)
    {
        var report = new StringBuilder("  physicalAllocationColumns (+1 action, [Fx,Fy,Fz,Mx,My,Mz]):\n");
        for (int i = 0; i < thrusters.Length; i++)
        {
            var action = new float[thrusters.Length];
            action[i] = 1f;
            BodyWrench wrench = ComputeBodyWrench(reference, centerOfMass, thrusters, action);
            report.AppendLine($"    {CanonicalThrusterNames[i]}: {wrench}");
        }

        return report.ToString();
    }

    static void RequireNearZero(float value, float tolerance, string label)
    {
        Require(Mathf.Abs(value) <= tolerance, $"{label} must be near zero, got {value:F6} (tolerance {tolerance:F6}).");
    }

    static void Require(bool condition, string message)
    {
        if (!condition)
        {
            throw new InvalidOperationException("[FinsROVChaseStaticAudit] " + message);
        }
    }

    readonly struct BodyWrench
    {
        public readonly Vector3 force;
        public readonly Vector3 torque;

        public BodyWrench(Vector3 force, Vector3 torque)
        {
            this.force = force;
            this.torque = torque;
        }

        public override string ToString()
        {
            return $"force={force}, torque={torque}";
        }
    }
}
