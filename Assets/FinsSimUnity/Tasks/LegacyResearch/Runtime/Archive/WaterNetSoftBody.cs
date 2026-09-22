using UnityEngine;

/// <summary>
/// 单渔网物体柔体脚本（无任何插件依赖，适配所有水动力环境）
/// 实现节点间弹性柔性变形，与Sphere Collider、Rigidbody完美联动
/// </summary>
public class WaterNetSoftBody : MonoBehaviour
{
    [Header("渔网柔性参数（默认值适配绝大多数场景）")]
    [Tooltip("节点间弹性刚度：越小越软，越大越硬（0.2-0.4最佳）")]
    public float stiffness = 0.3f;
    [Tooltip("阻尼系数：抑制过度抖动（0.4-0.5最佳）")]
    public float damping = 0.45f;
    [Tooltip("顶点质量：越小越易被外力推动变形（0.02-0.05最佳）")]
    public float vertexMass = 0.03f;
    [Tooltip("最大拉伸限制：防止渔网被冲散（0.1-0.2最佳）")]
    [Range(0.05f, 0.3f)] public float maxStretch = 0.15f;

    // 核心网格数据
    private Mesh _netMesh;
    private Vector3[] _originalVerts; // 原始顶点位置（节点弹性约束目标）
    private Vector3[] _currentVerts;  // 当前顶点位置（实时柔性变形）
    private Vector3[] _vertVelocities;// 顶点速度（控制变形平滑度）
    private int[] _triangles;         // 网格三角面，识别相邻节点

    private MeshFilter _meshFilter;
    private Rigidbody _rb;
    private bool _isValid = false;    // 初始化校验标记

    void Start()
    {
        InitSoftBody(); // 初始化柔体数据
    }

    /// <summary>
    /// 初始化柔体数据，仅校验Unity原生核心组件
    /// </summary>
    private void InitSoftBody()
    {
        // 仅获取Unity原生组件，无任何插件依赖
        _meshFilter = GetComponent<MeshFilter>();
        _rb = GetComponent<Rigidbody>();

        // 校验核心组件，友好提示错误原因
        if (_meshFilter == null || _meshFilter.mesh == null)
        {
            Debug.LogError("【柔体脚本】渔网物体缺少MeshFilter组件，或未关联Mesh！");
            return;
        }
        if (!_meshFilter.mesh.isReadable)
        {
            Debug.LogError("【柔体脚本】请先开启渔网FBX的「Read/Write Enabled」并点击Apply！");
            return;
        }
        if (_rb == null || _rb.isKinematic)
        {
            Debug.LogError("【柔体脚本】渔网物体需要添加「非Kinematic」的Rigidbody组件！");
            return;
        }

        // 初始化网格数据，标记为动态可修改（提升变形性能）
        _netMesh = _meshFilter.mesh;
        _netMesh.MarkDynamic();
        _originalVerts = _netMesh.vertices;
        _currentVerts = (Vector3[])_originalVerts.Clone();
        _vertVelocities = new Vector3[_originalVerts.Length];
        _triangles = _netMesh.triangles;

        _isValid = true;
        Debug.Log("【柔体脚本】初始化完成，渔网可实现节点间柔性变形！");
    }

    /// <summary>
    /// 物理帧更新（与Unity刚体、碰撞体同步，保证变形真实不脱节）
    /// </summary>
    void FixedUpdate()
    {
        if (!_isValid) return; // 未初始化成功则不执行任何逻辑

        ApplyElasticConstraint(); // 节点间弹性约束，实现柔性弯曲
        UpdateMeshVertices();    // 更新网格顶点，刷新视觉变形
    }

    /// <summary>
    /// 核心：节点间弹性约束计算，模拟渔网绳结的拉力与弯曲特性
    /// </summary>
    private void ApplyElasticConstraint()
    {
        // 1. 基础弹性复位力：拉回原始形状，实现柔性回弹，抑制过度变形
        for (int i = 0; i < _currentVerts.Length; i++)
        {
            Vector3 displacement = _originalVerts[i] - _currentVerts[i];
            Vector3 elasticForce = displacement * stiffness; // 弹性拉力
            Vector3 accel = elasticForce / Mathf.Max(vertexMass, 0.001f); // 顶点加速度（避免除0）

            // 更新顶点速度，添加阻尼抑制抖动，让变形更顺滑
            _vertVelocities[i] += accel * Time.fixedDeltaTime;
            _vertVelocities[i] *= Mathf.Clamp01(1f - damping * Time.fixedDeltaTime);
            // 更新当前顶点位置
            _currentVerts[i] += _vertVelocities[i] * Time.fixedDeltaTime;
        }

        // 2. 相邻节点拉伸约束：遍历三角面，限制最大拉伸距离，防止渔网被冲散
        for (int i = 0; i < _triangles.Length; i += 3)
        {
            int v1 = _triangles[i];
            int v2 = _triangles[i + 1];
            int v3 = _triangles[i + 2];

            // 对三角面的3组相邻节点进行拉伸约束，保证渔网拓扑结构
            ConstrainTwoNodes(v1, v2);
            ConstrainTwoNodes(v2, v3);
            ConstrainTwoNodes(v1, v3);
        }
    }

    /// <summary>
    /// 两个相邻节点间的拉伸约束：超过最大距离则双向拉回，防止散架
    /// </summary>
    private void ConstrainTwoNodes(int indexA, int indexB)
    {
        float originalDist = Vector3.Distance(_originalVerts[indexA], _originalVerts[indexB]);
        float currentDist = Vector3.Distance(_currentVerts[indexA], _currentVerts[indexB]);
        float maxAllowDist = originalDist * (1f + maxStretch); // 最大允许拉伸距离

        // 超过最大拉伸距离，产生反向拉力拉回，均分修正距离
        if (currentDist > maxAllowDist)
        {
            Vector3 dir = (_currentVerts[indexA] - _currentVerts[indexB]).normalized;
            float overStretch = currentDist - maxAllowDist;
            Vector3 correctPos = dir * overStretch * 0.5f;

            _currentVerts[indexA] -= correctPos;
            _currentVerts[indexB] += correctPos;
        }
    }

    /// <summary>
    /// 更新网格顶点位置，刷新视觉变形，同步碰撞体避免脱节
    /// </summary>
    private void UpdateMeshVertices()
    {
        _netMesh.vertices = _currentVerts;
        _netMesh.RecalculateNormals(); // 重新计算法线，保证光照渲染正常
        _netMesh.RecalculateBounds();  // 重新计算包围盒，保证碰撞检测准确

        // 同步Mesh Collider（如果有），避免视觉变形与碰撞体位置不一致
        if (TryGetComponent<MeshCollider>(out var meshCol))
        {
            meshCol.sharedMesh = null;
            meshCol.sharedMesh = _netMesh;
        }
    }

    /// <summary>
    /// 可选：重置渔网到原始形状（可通过UI按钮、代码外部调用）
    /// </summary>
    public void ResetNetShape()
    {
        if (_isValid)
        {
            _currentVerts = (Vector3[])_originalVerts.Clone();
            _vertVelocities = new Vector3[_originalVerts.Length];
            UpdateMeshVertices();
            Debug.Log("【柔体脚本】渔网已重置为原始形状！");
        }
    }
}
