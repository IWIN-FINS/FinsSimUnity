using UnityEngine;
using System; // 解决Array不存在
#if UNITY_EDITOR
using UnityEditor; // 引入编辑器命名空间，仅编辑器生效
#endif
using System.Collections.Generic;

[DisallowMultipleComponent]
[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public class Net_MeshBinder : MonoBehaviour
{
    [Header("=== 形变参数（核心调软度）===")]
    [Tooltip("每个顶点受几个最近节点影响（2~4=最像布，必设>1）")]
    public int influenceNodes = 3;
    [Tooltip("距离衰减指数（1.0~2.0，越小越软、越易拉伸）")]
    public float falloffPower = 1.2f;
    [Tooltip("刷新间隔（0.01~0.03，越小形变越流畅）")]
    public float updateInterval = 0.02f;

    [Header("=== 贴合校准参数（解决节点与Mesh偏移）===")]
    [Tooltip("强制校准节点与Mesh的坐标空间（必开）")]
    public bool forceCalibratePos = true;
    [Tooltip("是否基于根物体缩放适配（解决Blender导出缩放问题）")]
    public bool adaptRootScale = true;

    // 核心组件与Mesh数据
    private MeshFilter _mf;
    private Mesh _dynamicMesh; // 强制新建的动态可写Mesh
    private Vector3[] _originVerts; // Mesh原始顶点（局部坐标）
    private Vector3[] _deformVerts; // 形变后的顶点（最终渲染用）
    private Vector3 _rootScale; // 根物体缩放缓存

    // 物理节点列表
    private List<Transform> _nodeList = new List<Transform>();
    private Transform _nodeRoot; // 节点根容器（Net_Nodes）

    // 顶点影响缓存：每个顶点对应[节点索引, 权重]，预计算一次避免每帧开销
    private struct VertInfluenceData
    {
        public int[] nodeIndices; // 受影响的节点索引数组
        public float[] nodeWeights; // 对应节点的权重（归一化）
    }
    private VertInfluenceData[] _allVertInfluence;

    // 刷新计时器
    private float _updateTimer;

    void Start()
    {
        // 1. 初始化核心组件，缺失则直接禁用
        _mf = GetComponent<MeshFilter>();
        if (_mf == null || _mf.sharedMesh == null)
        {
            Debug.LogError("[MeshBinder] 渔网根物体缺少MeshFilter或有效Mesh！");
#if UNITY_EDITOR
            EditorUtility.DisplayDialog("Mesh绑定错误", "渔网根物体缺少MeshFilter或有效Mesh！", "确定");
#endif
            enabled = false;
            return;
        }

        // 2. 初始化坐标校准与缩放适配
        _rootScale = adaptRootScale ? transform.localScale : Vector3.one;
        _nodeRoot = transform.Find("Net_Nodes");
        if (_nodeRoot == null)
        {
            Debug.LogError("[MeshBinder] 未找到节点根容器Net_Nodes！（必须是根物体直接子物体）");
#if UNITY_EDITOR
            EditorUtility.DisplayDialog("Mesh绑定错误", "未找到节点根容器Net_Nodes！\n必须是渔网根物体的直接子物体", "确定");
#endif
            enabled = false;
            return;
        }

        // 3. 收集物理节点，空则禁用
        CollectPhysicsNodes();
        if (_nodeList.Count == 0)
        {
            Debug.LogError("[MeshBinder] 未收集到有效物理节点！（节点需以Node_开头且带SphereCollider）");
#if UNITY_EDITOR
            EditorUtility.DisplayDialog("Mesh绑定错误", "未收集到有效物理节点！\n要求：1.节点以Node_开头 2.带有SphereCollider 3.非Trigger", "确定");
#endif
            enabled = false;
            return;
        }

        // 4. 初始化动态Mesh（强制可写，避免Unity静态缓存）
        InitDynamicMesh();

        // 5. 预计算顶点-节点影响关系（核心：实现形变的基础）
        PrecomputeVertNodeInfluence();

        Debug.Log($"[MeshBinder] 初始化成功！顶点数：{_originVerts.Length} | 物理节点数：{_nodeList.Count} | 每顶点影响数：{influenceNodes}");
    }

    void Update()
    {
        if (!Application.isPlaying || _nodeList.Count == 0 || _allVertInfluence == null)
            return;

        // 按间隔更新Mesh，平衡流畅度与性能
        _updateTimer += Time.deltaTime;
        if (_updateTimer < updateInterval)
            return;
        _updateTimer = 0;

        // 核心：更新Mesh顶点（先校准贴合，再计算形变）
        UpdateMeshDeformation();
    }

    /// <summary>
    /// 收集Net_Nodes下所有有效物理节点（Node_开头+带SphereCollider）
    /// </summary>
    private void CollectPhysicsNodes()
    {
        _nodeList.Clear();
        foreach (Transform child in _nodeRoot)
        {
            if (child.name.StartsWith("Node_") && child.GetComponent<SphereCollider>() != null && !child.GetComponent<SphereCollider>().isTrigger)
            {
                _nodeList.Add(child);
                // 强制校准节点局部位置（解决节点自身Transform偏移）
                if (forceCalibratePos)
                    child.localRotation = Quaternion.identity;
            }
        }
    }

    /// <summary>
    /// 初始化动态Mesh：强制新建、标记可写，避免Unity静态缓存导致修改不渲染
    /// </summary>
    private void InitDynamicMesh()
    {
        // 克隆原始Mesh并标记为动态可写
        _dynamicMesh = new Mesh();
        _dynamicMesh.name = "Net_DynamicMesh";
        _dynamicMesh.vertices = _mf.sharedMesh.vertices;
        _dynamicMesh.triangles = _mf.sharedMesh.triangles;
        _dynamicMesh.uv = _mf.sharedMesh.uv;
        _dynamicMesh.normals = _mf.sharedMesh.normals;
        _dynamicMesh.tangents = _mf.sharedMesh.tangents;
        _dynamicMesh.MarkDynamic(); // 关键：告诉Unity这是动态Mesh，允许每帧修改顶点

        // 初始化顶点数组
        _originVerts = _dynamicMesh.vertices;
        _deformVerts = new Vector3[_originVerts.Length];
        Array.Copy(_originVerts, _deformVerts, _originVerts.Length);

        // 赋值给MeshFilter，作为渲染用Mesh
        _mf.mesh = _dynamicMesh;
    }

    /// <summary>
    /// 预计算每个顶点受哪些节点加权影响（只执行一次，大幅提升性能）
    /// </summary>
    private void PrecomputeVertNodeInfluence()
    {
        _allVertInfluence = new VertInfluenceData[_originVerts.Length];
        int effectiveInfluence = Mathf.Clamp(influenceNodes, 2, _nodeList.Count); // 强制至少2个节点，避免刚性

        for (int vIdx = 0; vIdx < _originVerts.Length; vIdx++)
        {
            // 1. 计算当前顶点的世界坐标（叠加根缩放，解决坐标偏移）
            Vector3 vertLocal = _originVerts[vIdx];
            if (adaptRootScale) vertLocal = Vector3.Scale(vertLocal, _rootScale);
            Vector3 vertWorld = transform.TransformPoint(vertLocal);

            // 2. 找到所有节点中，距离当前顶点最近的N个
            List<(int nodeIdx, float dist)> nodeDistList = new List<(int, float)>();
            for (int nIdx = 0; nIdx < _nodeList.Count; nIdx++)
            {
                if (_nodeList[nIdx] == null) continue;
                float dist = Vector3.Distance(vertWorld, _nodeList[nIdx].position);
                nodeDistList.Add((nIdx, dist));
            }

            // 3. 按距离升序排序，取前N个节点
            nodeDistList.Sort((a, b) => a.dist.CompareTo(b.dist));
            int takeCount = Mathf.Min(effectiveInfluence, nodeDistList.Count);

            // 4. 计算每个节点的权重（距离越近，权重越大）
            float[] weights = new float[takeCount];
            float totalWeight = 0;
            int[] indices = new int[takeCount];
            for (int i = 0; i < takeCount; i++)
            {
                indices[i] = nodeDistList[i].nodeIdx;
                weights[i] = 1f / Mathf.Pow(Mathf.Max(nodeDistList[i].dist, 0.001f), falloffPower); // 避免除0
                totalWeight += weights[i];
            }

            // 5. 权重归一化（总和=1，避免顶点偏移）
            for (int i = 0; i < takeCount; i++)
                weights[i] /= totalWeight;

            // 6. 缓存当前顶点的影响数据
            _allVertInfluence[vIdx] = new VertInfluenceData
            {
                nodeIndices = indices,
                nodeWeights = weights
            };
        }
    }

    /// <summary>
    /// 核心：更新Mesh形变（先校准贴合，再按多节点加权计算顶点位置）
    /// </summary>
    private void UpdateMeshDeformation()
    {
        for (int vIdx = 0; vIdx < _originVerts.Length; vIdx++)
        {
            VertInfluenceData vertInf = _allVertInfluence[vIdx];
            Vector3 blendedWorldPos = Vector3.zero;

            // 遍历当前顶点受影响的所有节点，计算加权混合的世界坐标
            for (int i = 0; i < vertInf.nodeIndices.Length; i++)
            {
                int nIdx = vertInf.nodeIndices[i];
                if (nIdx < 0 || nIdx >= _nodeList.Count || _nodeList[nIdx] == null)
                    continue;

                blendedWorldPos += _nodeList[nIdx].position * vertInf.nodeWeights[i];
            }

            // 将混合后的世界坐标转换为Mesh局部坐标（适配根缩放，解决贴合偏移）
            Vector3 finalLocalPos = transform.InverseTransformPoint(blendedWorldPos);
            if (adaptRootScale)
                finalLocalPos = Vector3.Scale(finalLocalPos, new Vector3(1/_rootScale.x, 1/_rootScale.y, 1/_rootScale.z));

            _deformVerts[vIdx] = finalLocalPos;
        }

        // 更新动态Mesh顶点，并刷新法线/包围盒（确保渲染正常、碰撞准确）
        _dynamicMesh.vertices = _deformVerts;
        _dynamicMesh.RecalculateNormals();
        _dynamicMesh.RecalculateBounds();
    }

    /// <summary>
    /// 编辑器快捷检查：一键查看绑定状态（节点数、顶点数、贴合度）
    /// </summary>
    [ContextMenu("★ 检查Mesh-节点绑定状态 ★")]
    public void CheckBindState()
    {
        _nodeRoot = transform.Find("Net_Nodes");
        CollectPhysicsNodes();
        string stateInfo = $"=== 渔网绑定状态检查 ===\n";
        stateInfo += $"1. 根物体缩放：{transform.localScale}\n";
        stateInfo += $"2. 节点根容器：{( _nodeRoot == null ? "缺失（错误）" : "存在（正常）" )}\n";
        stateInfo += $"3. 有效物理节点数：{_nodeList.Count}\n";
        stateInfo += $"4. Mesh顶点数：{_mf?.sharedMesh?.vertexCount ?? 0}\n";
        stateInfo += $"5. 每顶点影响节点数：{influenceNodes}\n";
        stateInfo += $"6. 贴合校准：{( forceCalibratePos ? "开启（推荐）" : "关闭（易偏移）" )}\n";
        stateInfo += $"7. 缩放适配：{( adaptRootScale ? "开启（推荐）" : "关闭（易偏移）" )}";

        Debug.Log(stateInfo);
#if UNITY_EDITOR
        EditorUtility.DisplayDialog("绑定状态检查", stateInfo, "确定");
#endif
    }

    /// <summary>
    /// 运行时Gizmos可视化：在Scene视图显示顶点-节点绑定关系（青色线条）
    /// </summary>
    void OnDrawGizmosSelected()
    {
        if (!Application.isPlaying || _nodeList.Count == 0 || _allVertInfluence == null)
            return;

        Gizmos.color = Color.cyan;
        int showCount = Mathf.Min(200, _originVerts.Length); // 只显示前200个，避免卡顿
        for (int vIdx = 0; vIdx < showCount; vIdx++)
        {
            VertInfluenceData vertInf = _allVertInfluence[vIdx];
            Vector3 vertWorld = transform.TransformPoint(_originVerts[vIdx]);
            Gizmos.DrawSphere(vertWorld, 0.01f); // 绘制顶点小球

            // 绘制顶点到每个受影响节点的连线
            for (int i = 0; i < vertInf.nodeIndices.Length; i++)
            {
                int nIdx = vertInf.nodeIndices[i];
                if (nIdx < 0 || nIdx >= _nodeList.Count || _nodeList[nIdx] == null)
                    continue;

                Gizmos.DrawLine(vertWorld, _nodeList[nIdx].position);
            }
        }
    }
}
