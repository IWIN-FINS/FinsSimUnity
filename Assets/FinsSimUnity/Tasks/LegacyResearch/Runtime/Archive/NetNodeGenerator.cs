using UnityEngine;

public class NetNodeGenerator : MonoBehaviour
{
    [Header("节点碰撞体设置")]
    public float nodeRadius = 0.06f;
    [Header("顶点采样步长，数值越大节点越少")]
    public int sampleStep = 100;
    [Header("手动创建的节点根容器（拖入Net_Nodes）")]
    public Transform nodeRoot; // 关键：拖入步骤1创建的Net_Nodes

    void Start()
    {
        // 校验：必须指定节点根容器
        if (nodeRoot == null)
        {
            Debug.LogError("请先手动创建Net_Nodes空物体，再拖入nodeRoot参数！");
            return;
        }

        MeshFilter mf = GetComponent<MeshFilter>();
        if (mf == null || mf.mesh == null)
        {
            Debug.LogError("渔网根物体未找到MeshFilter或Mesh！");
            return;
        }

        Vector3[] vertices = mf.mesh.vertices;
        int nodeCount = 0;

        // 清空容器原有节点（避免重复生成）
        foreach (Transform child in nodeRoot)
        {
            Destroy(child.gameObject);
        }

        // 按顶点采样生成节点
        for (int i = 0; i < vertices.Length; i += sampleStep)
        {
            GameObject node = new GameObject($"Node_{i}");
            node.transform.SetParent(nodeRoot, false); // 设为Net_Nodes子物体，继承局部坐标
            node.transform.localPosition = vertices[i]; // 用局部坐标，匹配Mesh顶点位置

            SphereCollider col = node.AddComponent<SphereCollider>();
            col.radius = nodeRadius;
            col.isTrigger = false;

            nodeCount++;
        }

        Debug.Log($"节点生成完成，共生成：{nodeCount} 个，已存入Net_Nodes容器");
        Destroy(this); // 生成完毕自毁，不占用性能
    }
}
