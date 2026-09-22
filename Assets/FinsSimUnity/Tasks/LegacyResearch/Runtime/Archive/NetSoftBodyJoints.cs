using System.Collections.Generic;
using UnityEngine;

[RequireComponent(typeof(Rigidbody))]
public class NetSoftBodyJoints : MonoBehaviour
{
    [Header("节点物理属性")]
    public float nodeMass = 0.12f;
    public float drag = 1.5f;
    public float angularDrag = 9f;

    [Header("弹簧刚度与阻尼")]
    public float spring = 90f;
    public float damper = 20f;
    public float maxDistanceMultiplier = 1.2f;

    [Header("节点连接范围与最大连接数")]
    public float connectRange = 0.7f;
    public int maxConnectionsPerNode = 4;

    private List<Transform> _nodes = new List<Transform>();
    private Rigidbody _rootRb;

    void Awake()
    {
        _rootRb = GetComponent<Rigidbody>();
        CollectAllNodes();
        SetNodesPhysics();
        CreateSpringConnections();
    }

    /// <summary>
    /// 收集所有带SphereCollider的子节点
    /// </summary>
    void CollectAllNodes()
    {
        _nodes.Clear();
        foreach (Transform t in GetComponentsInChildren<Transform>())
        {
            if (t != transform && t.GetComponent<SphereCollider>() != null)
                _nodes.Add(t);
        }
        Debug.Log($"收集到物理节点数量：{_nodes.Count}");
    }

    /// <summary>
    /// 给节点统一设置刚体属性，开启连续碰撞、插值
    /// </summary>
    void SetNodesPhysics()
    {
        foreach (Transform node in _nodes)
        {
            Rigidbody rb = node.GetComponent<Rigidbody>();
            if (rb == null)
                rb = node.gameObject.AddComponent<Rigidbody>();

            rb.mass = nodeMass;
            rb.linearDamping = drag;
            rb.angularDamping = angularDrag;
            rb.useGravity = true;
            rb.isKinematic = false;
            rb.interpolation = RigidbodyInterpolation.Interpolate;
            rb.collisionDetectionMode = CollisionDetectionMode.ContinuousDynamic;
        }
    }

    /// <summary>
    /// 为每个节点创建邻近弹簧连接，并连接回根物体
    /// </summary>
    void CreateSpringConnections()
    {
        foreach (Transform nodeA in _nodes)
        {
            Rigidbody rbA = nodeA.GetComponent<Rigidbody>();
            int connectedCount = 0;

            // 连接邻近节点
            foreach (Transform nodeB in _nodes)
            {
                if (nodeA == nodeB || connectedCount >= maxConnectionsPerNode)
                    continue;

                float dist = Vector3.Distance(nodeA.position, nodeB.position);
                if (dist < connectRange)
                {
                    SpringJoint sj = nodeA.gameObject.AddComponent<SpringJoint>();
                    sj.connectedBody = nodeB.GetComponent<Rigidbody>();
                    sj.anchor = Vector3.zero;
                    sj.connectedAnchor = Vector3.zero;
                    sj.spring = spring;
                    sj.damper = damper;
                    sj.maxDistance = dist * maxDistanceMultiplier;
                    sj.enableCollision = true;

                    connectedCount++;
                }
            }

            // 每个节点都连回根刚体，防止整体散架
            SpringJoint sjRoot = nodeA.gameObject.AddComponent<SpringJoint>();
            sjRoot.connectedBody = _rootRb;
            sjRoot.anchor = Vector3.zero;
            sjRoot.connectedAnchor = nodeA.localPosition;
            sjRoot.spring = spring * 0.4f;
            sjRoot.damper = damper * 0.4f;
            sjRoot.maxDistance = connectRange;
            sjRoot.enableCollision = true;
        }
    }
}
