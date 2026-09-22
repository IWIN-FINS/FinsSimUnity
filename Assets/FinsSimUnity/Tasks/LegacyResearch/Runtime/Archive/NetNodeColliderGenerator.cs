using UnityEngine;

/// <summary>
/// 单渔网物体节点Sphere Collider批量生成脚本
/// 自动读取Mesh顶点，生成节点碰撞体，无独立Rigidbody，继承父物体物理
/// </summary>
public class NetNodeColliderGenerator : MonoBehaviour
{
    [Header("节点碰撞体配置（直接用默认值）")]
    public float sphereRadius = 0.06f; // 绳结碰撞体半径，适配渔网大小
    [Range(20, 80)] public int maxNodeCount = 50; // 最大节点数，控制性能（30-50最佳）
    public string nodeGroupName = "NetNodes"; // 节点统一父物体，方便管理

    private MeshFilter _meshFilter;
    private Mesh _netMesh;

    void Start()
    {
        // 自动获取组件，无需手动赋值
        _meshFilter = GetComponent<MeshFilter>();
        CheckValid(); // 校验前置条件，避免报错
        GenerateAllNodes(); // 批量生成节点碰撞体
        Debug.Log("渔网节点Sphere Collider生成完成！共生成" + transform.Find(nodeGroupName).childCount + "个节点");
    }

    /// <summary>
    /// 校验前置条件，提示错误原因
    /// </summary>
    private void CheckValid()
    {
        if (_meshFilter == null || _meshFilter.mesh == null)
        {
            Debug.LogError("渔网物体缺少MeshFilter或未关联Mesh！");
            enabled = false;
        }
        else if (!_meshFilter.mesh.isReadable)
        {
            Debug.LogError("请先开启渔网FBX的Read/Write Enabled并Apply！");
            enabled = false;
        }
        _netMesh = _meshFilter.mesh;
    }

    /// <summary>
    /// 批量生成节点，创建统一父物体管理所有Sphere Collider
    /// </summary>
    private void GenerateAllNodes()
    {
        // 创建节点统一父物体（作为渔网子物体，无独立Rigidbody）
        GameObject nodeParent = new GameObject(nodeGroupName);
        nodeParent.transform.SetParent(transform, false);
        nodeParent.layer = gameObject.layer; // 与渔网同层，碰撞检测一致

        Vector3[] vertices = _netMesh.vertices; // 读取渔网Mesh顶点（局部坐标）
        int step = Mathf.Max(1, vertices.Length / maxNodeCount); // 计算步长，控制节点数量

        // 遍历顶点，按步长生成节点Sphere Collider
        for (int i = 0; i < vertices.Length; i += step)
        {
            GameObject node = new GameObject("NetNode_" + i);
            node.transform.SetParent(nodeParent.transform, false);
            node.transform.localPosition = vertices[i]; // 与渔网顶点完全重合

            // 添加Sphere Collider，无独立Rigidbody，继承父物体物理
            SphereCollider collider = node.AddComponent<SphereCollider>();
            collider.radius = sphereRadius;
            collider.isTrigger = false; // 非触发器，实现真实物理碰撞
        }
    }
}
