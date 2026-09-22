using System.Collections.Generic;
using System.IO;
using UnityEditor;
using UnityEngine;
using UnityEngine.Rendering;
 
namespace UniMeshCombiner
{
    // 自定义编辑器窗口类，继承自 UnityEditor.EditorWindow
    public class UniMeshCombinerWindow : EditorWindow
    {
        // 导出的目标目录（Project 视图中一个文件夹）
        private DefaultAsset _exportDirectory = null;
 
        // 要合并网格的目标 GameObject（包含 MeshFilter 的物体）
        private GameObject _combineTarget = null;
 
        // 是否导出合并后的 Mesh 为 .asset 文件
        private bool _exportMesh;

        // 是否根据材质合并
        private bool _mergeByMaterial = false;
 
        // 在 Unity 菜单栏添加一个窗口菜单项
        [MenuItem("Window/UniMeshCombiner")]
        static void Open()
        {
            // 创建并显示窗口，标题为 "UniMeshCombiner"
            GetWindow<UniMeshCombinerWindow>("UniMeshCombiner").Show();
        }
 
        // 编辑器窗口 GUI 绘制函数
        void OnGUI()
        {
            // 拖入或选择要合并网格的目标物体
            _combineTarget = (GameObject)EditorGUILayout.ObjectField("CombineTarget", _combineTarget, typeof(GameObject), true);
 
            // 选择是否导出合并结果为资源文件
            _exportMesh = EditorGUILayout.Toggle("Export Mesh", _exportMesh);
 
            // 拖入导出目录，类型为 DefaultAsset（文件夹）
            _exportDirectory = (DefaultAsset)EditorGUILayout.ObjectField("Export Directory", _exportDirectory, typeof(DefaultAsset), true);

            // 选择是否根据材质合并
            _mergeByMaterial = EditorGUILayout.Toggle("Merge by Material", _mergeByMaterial);

            // 合并按钮
            if (GUILayout.Button("Combine"))
            {
                // 若未指定目标，则不执行
                if (_combineTarget == null)
                {
                    return;
                }
 
                // 调用合并函数
                CombineMesh();
            }
        }
 
        // 执行网格合并逻辑
        void CombineMesh()
        {
            if (_mergeByMaterial)
            {
                CombineMeshByMaterial();
            }
            else
            {
                CombineMeshToSingle();
            }
        }

        // 根据材质分别合并网格
        void CombineMeshByMaterial()
        {
            // 获取目标对象下所有子对象的 MeshFilter 组件
            var meshFilters = _combineTarget.GetComponentsInChildren<MeshFilter>();
            var meshFilterCount = meshFilters.Length;
 
            Debug.Log($"[UniMeshCombiner] 开始按材质合并，共找到 {meshFilterCount} 个 MeshFilter");
 
            // 用字典记录每种材质对应的 CombineInstance 列表（分材质合并）
            var combineMeshInstanceDictionary = new Dictionary<Material, List<CombineInstance>>();
 
            // 遍历所有 MeshFilter
            var filterIndex = 0;
            foreach (var meshFilter_single in meshFilters)
            {
                filterIndex++;
                var progress = (float)filterIndex / meshFilterCount;
                EditorUtility.DisplayProgressBar("UniMeshCombiner - 按材质合并", 
                    $"处理中... ({filterIndex}/{meshFilterCount})", progress);

                var mesh = meshFilter_single.sharedMesh; // 获取 mesh 数据
                var vertices = new List<Vector3>();
                mesh.GetVertices(vertices); // 获取顶点数据
 
                var materials = meshFilter_single.GetComponent<Renderer>().sharedMaterials; // 获取材质数组
                var subMeshCount = meshFilter_single.sharedMesh.subMeshCount; // 获取子网格数量
 
                // 遍历每个子网格（按材质处理）
                for (var i = 0; i < subMeshCount; i++)
                {
                    var material = materials[i]; // 当前子网格使用的材质
                    var triangles = new List<int>();
                    mesh.GetTriangles(triangles, i); // 获取当前子网格的三角面索引
 
                    // 创建一个新 mesh，仅包含当前 submesh 的顶点和三角面
                    var newMesh = new Mesh
                    {
                        vertices = vertices.ToArray(),
                        triangles = triangles.ToArray(),
                        uv = mesh.uv,
                        normals = mesh.normals
                    };
 
                    // 若该材质尚未记录，则初始化列表
                    if (!combineMeshInstanceDictionary.ContainsKey(material))
                    {
                        combineMeshInstanceDictionary.Add(material, new List<CombineInstance>());
                    }
 
                    // 创建 CombineInstance 并加入字典
                    var combineInstance = new CombineInstance
                    {
                        transform = meshFilter_single.transform.localToWorldMatrix, // 记录世界坐标变换矩阵
                        mesh = newMesh
                    };
 
                    combineMeshInstanceDictionary[material].Add(combineInstance);
                }
            }
 
            // 隐藏原始对象
            _combineTarget.SetActive(false);
            
            Debug.Log($"[UniMeshCombiner] 数据收集完成，共 {combineMeshInstanceDictionary.Count} 种材质");
 
            // 遍历每种材质的合并实例列表
            var materialIndex = 0;
            var materialCount = combineMeshInstanceDictionary.Count;
            foreach (var kvp in combineMeshInstanceDictionary)
            {
                materialIndex++;
                var progress = (float)materialIndex / materialCount;
                EditorUtility.DisplayProgressBar("UniMeshCombiner - 按材质合并", 
                    $"合并材质 {kvp.Key.name}... ({materialIndex}/{materialCount})", progress);

                var material = kvp.Key;
                var combineList = kvp.Value;
 
                // 为该材质创建一个新的 GameObject（用于显示合并结果）
                var newObject = new GameObject(material.name);
 
                // 添加 MeshRenderer 和 MeshFilter 组件
                var meshRenderer = newObject.AddComponent<MeshRenderer>();
                var meshFilter = newObject.AddComponent<MeshFilter>();
 
                // 设置材质
                meshRenderer.material = material;
 
                // 创建新 Mesh 并进行合并
                var mesh = new Mesh();
                mesh.indexFormat = IndexFormat.UInt32;
                mesh.CombineMeshes(combineList.ToArray());
 
                // 自动生成第二套 UV（用于烘焙或光照贴图）
                Unwrapping.GenerateSecondaryUVSet(mesh);
 
                // 将合并后的 mesh 指定到 MeshFilter 上
                meshFilter.sharedMesh = mesh;
 
                // 设置合并结果的父物体为原目标的父物体
                newObject.transform.parent = _combineTarget.transform.parent;
 
                // 判断是否导出为 asset
                if (!_exportMesh || _exportDirectory == null)
                {
                    continue;
                }
 
                // 执行导出
                ExportMesh(mesh, material.name);
            }
            
            EditorUtility.ClearProgressBar();
            Debug.Log($"[UniMeshCombiner] 按材质合并完成！");
        }

        // 将所有网格合并为单一网格（不根据材质区分）
        void CombineMeshToSingle()
        {
            // 获取目标对象下所有子对象的 MeshFilter 组件
            var meshFilters = _combineTarget.GetComponentsInChildren<MeshFilter>();
            var meshFilterCount = meshFilters.Length;
            
            Debug.Log($"[UniMeshCombiner] 开始全局合并，共找到 {meshFilterCount} 个 MeshFilter");
 
            // 所有 CombineInstance 列表
            var combineList = new List<CombineInstance>();
            Material firstMaterial = null;
 
            // 遍历所有 MeshFilter
            var filterIndex = 0;
            foreach (var meshFilter_single in meshFilters)
            {
                filterIndex++;
                var progress = (float)filterIndex / meshFilterCount;
                EditorUtility.DisplayProgressBar("UniMeshCombiner - 全局合并", 
                    $"收集数据中... ({filterIndex}/{meshFilterCount})", progress);

                var mesh = meshFilter_single.sharedMesh; // 获取 mesh 数据
                var vertices = new List<Vector3>();
                mesh.GetVertices(vertices); // 获取顶点数据
 
                var materials = meshFilter_single.GetComponent<Renderer>().sharedMaterials; // 获取材质数组
                var subMeshCount = meshFilter_single.sharedMesh.subMeshCount; // 获取子网格数量
 
                // 记录第一个材质（用于单一合并网格）
                if (firstMaterial == null && materials.Length > 0)
                {
                    firstMaterial = materials[0];
                }
 
                // 遍历每个子网格
                for (var i = 0; i < subMeshCount; i++)
                {
                    var triangles = new List<int>();
                    mesh.GetTriangles(triangles, i); // 获取当前子网格的三角面索引
 
                    // 创建一个新 mesh，仅包含当前 submesh 的顶点和三角面
                    var newMesh = new Mesh
                    {
                        vertices = vertices.ToArray(),
                        triangles = triangles.ToArray(),
                        uv = mesh.uv,
                        normals = mesh.normals
                    };
 
                    // 创建 CombineInstance 并加入列表
                    var combineInstance = new CombineInstance
                    {
                        transform = meshFilter_single.transform.localToWorldMatrix, // 记录世界坐标变换矩阵
                        mesh = newMesh
                    };
 
                    combineList.Add(combineInstance);
                }
            }
 
            // 隐藏原始对象
            _combineTarget.SetActive(false);
            
            EditorUtility.DisplayProgressBar("UniMeshCombiner - 全局合并", "合并网格中...", 0.9f);
            Debug.Log($"[UniMeshCombiner] 数据收集完成，共 {combineList.Count} 个子网格");
 
            // 创建一个新的 GameObject（用于显示合并结果）
            var newObject = new GameObject("CombinedMesh");
 
            // 添加 MeshRenderer 和 MeshFilter 组件
            var meshRenderer = newObject.AddComponent<MeshRenderer>();
            var meshFilter = newObject.AddComponent<MeshFilter>();
 
            // 设置材质（使用第一个材质，或者可以指定其他默认材质）
            if (firstMaterial != null)
            {
                meshRenderer.material = firstMaterial;
            }
 
            // 创建新 Mesh 并进行合并
            var combinedMesh = new Mesh();
            combinedMesh.indexFormat = IndexFormat.UInt32;
            combinedMesh.CombineMeshes(combineList.ToArray());
 
            // 自动生成第二套 UV（用于烘焙或光照贴图）
            Unwrapping.GenerateSecondaryUVSet(combinedMesh);
 
            // 将合并后的 mesh 指定到 MeshFilter 上
            meshFilter.sharedMesh = combinedMesh;
 
            // 设置合并结果的父物体为原目标的父物体
            newObject.transform.parent = _combineTarget.transform.parent;
 
            // 判断是否导出为 asset
            if (_exportMesh && _exportDirectory != null)
            {
                ExportMesh(combinedMesh, "CombinedMesh");
            }
            
            EditorUtility.ClearProgressBar();
            Debug.Log($"[UniMeshCombiner] 全局合并完成！");
        }
 
        // 导出 mesh 为 .asset 文件
        void ExportMesh(Mesh mesh, string fileName)
        {
            // 获取文件夹在项目中的路径
            var exportDirectoryPath = AssetDatabase.GetAssetPath(_exportDirectory);
 
            // 确保文件名后缀为 .asset
            if (Path.GetExtension(fileName) != ".asset")
            {
                fileName += ".asset";
            }
 
            // 拼接完整导出路径
            var exportPath = Path.Combine(exportDirectoryPath, fileName);
 
            // 创建资源文件
            AssetDatabase.CreateAsset(mesh, exportPath);
        }
    }
}