using System.Collections.Generic;
using System.IO;
using UnityEditor;
using UnityEngine;
using UnityEngine.Rendering;

namespace UniMeshCombiner
{
    // 优化版本：使用Unity自带高效API的网格合并工具
    public class FastMeshCombinerWindow : EditorWindow
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
        [MenuItem("Window/FastMeshCombiner")]
        static void Open()
        {
            // 创建并显示窗口，标题为 "FastMeshCombiner"
            GetWindow<FastMeshCombinerWindow>("FastMeshCombiner").Show();
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
            if (GUILayout.Button("Combine (Fast)", GUILayout.Height(40)))
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
                CombineMeshByMaterialFast();
            }
            else
            {
                CombineMeshToSingleFast();
            }
        }

        // 优化版本：根据材质分别合并网格
        void CombineMeshByMaterialFast()
        {
            // 获取目标对象下所有子对象的 MeshFilter 组件
            var meshFilters = _combineTarget.GetComponentsInChildren<MeshFilter>();
            var meshFilterCount = meshFilters.Length;

            Debug.Log($"[FastMeshCombiner] 开始按材质合并，共找到 {meshFilterCount} 个 MeshFilter");

            // 用字典记录每种材质对应的 CombineInstance 列表
            var combineMeshInstanceDictionary = new Dictionary<Material, List<CombineInstance>>();

            // 收集所有 MeshFilter 信息
            var filterIndex = 0;
            foreach (var meshFilter_single in meshFilters)
            {
                filterIndex++;
                var progress = (float)filterIndex / meshFilterCount;
                EditorUtility.DisplayProgressBar("FastMeshCombiner - 按材质合并",
                    $"收集数据... ({filterIndex}/{meshFilterCount})", progress * 0.5f);

                var renderer = meshFilter_single.GetComponent<Renderer>();
                if (renderer == null || meshFilter_single.sharedMesh == null)
                    continue;

                var materials = renderer.sharedMaterials;
                var mesh = meshFilter_single.sharedMesh;

                // 对于每个材质的submesh，添加到对应的列表
                for (var i = 0; i < materials.Length && i < mesh.subMeshCount; i++)
                {
                    var material = materials[i];

                    if (!combineMeshInstanceDictionary.ContainsKey(material))
                    {
                        combineMeshInstanceDictionary.Add(material, new List<CombineInstance>());
                    }

                    // 创建 CombineInstance，直接使用原始 mesh
                    var combineInstance = new CombineInstance
                    {
                        mesh = mesh,
                        subMeshIndex = i,
                        transform = meshFilter_single.transform.localToWorldMatrix
                    };

                    combineMeshInstanceDictionary[material].Add(combineInstance);
                }
            }

            // 隐藏原始对象
            _combineTarget.SetActive(false);

            Debug.Log($"[FastMeshCombiner] 数据收集完成，共 {combineMeshInstanceDictionary.Count} 种材质");

            // 合并每种材质的网格
            var materialIndex = 0;
            var materialCount = combineMeshInstanceDictionary.Count;
            foreach (var kvp in combineMeshInstanceDictionary)
            {
                materialIndex++;
                var progress = 0.5f + (float)materialIndex / materialCount * 0.5f;
                EditorUtility.DisplayProgressBar("FastMeshCombiner - 按材质合并",
                    $"合并材质 {kvp.Key.name}... ({materialIndex}/{materialCount})", progress);

                var material = kvp.Key;
                var combineInstances = kvp.Value;

                // 为该材质创建一个新的 GameObject
                var newObject = new GameObject(material.name);

                // 添加 MeshRenderer 和 MeshFilter 组件
                var meshRenderer = newObject.AddComponent<MeshRenderer>();
                var meshFilter = newObject.AddComponent<MeshFilter>();

                // 设置材质
                meshRenderer.material = material;

                // 使用 CombineMeshes 直接合并，mergeSubMeshes=true 时会合并所有 submesh
                var combinedMesh = new Mesh();
                combinedMesh.name = material.name + "_Combined";
                combinedMesh.indexFormat = IndexFormat.UInt32;
                combinedMesh.CombineMeshes(combineInstances.ToArray(), true, true);

                // 自动生成第二套 UV
                Unwrapping.GenerateSecondaryUVSet(combinedMesh);

                // 将合并后的 mesh 指定到 MeshFilter 上
                meshFilter.sharedMesh = combinedMesh;

                // 设置合并结果的父物体
                newObject.transform.parent = _combineTarget.transform.parent;

                // 判断是否导出为 asset
                if (_exportMesh && _exportDirectory != null)
                {
                    ExportMesh(combinedMesh, material.name);
                }
            }

            EditorUtility.ClearProgressBar();
            Debug.Log($"[FastMeshCombiner] 按材质合并完成！");
        }

        // 优化版本：将所有网格合并为单一网格
        void CombineMeshToSingleFast()
        {
            // 获取目标对象下所有子对象的 MeshFilter 组件
            var meshFilters = _combineTarget.GetComponentsInChildren<MeshFilter>();
            var meshFilterCount = meshFilters.Length;

            Debug.Log($"[FastMeshCombiner] 开始全局合并，共找到 {meshFilterCount} 个 MeshFilter");

            // 收集所有 CombineInstance
            var combineList = new List<CombineInstance>();
            Material firstMaterial = null;

            var filterIndex = 0;
            foreach (var meshFilter_single in meshFilters)
            {
                filterIndex++;
                var progress = (float)filterIndex / meshFilterCount;
                EditorUtility.DisplayProgressBar("FastMeshCombiner - 全局合并",
                    $"收集数据... ({filterIndex}/{meshFilterCount})", progress * 0.5f);

                var renderer = meshFilter_single.GetComponent<Renderer>();
                if (renderer == null || meshFilter_single.sharedMesh == null)
                    continue;

                var mesh = meshFilter_single.sharedMesh;
                var materials = renderer.sharedMaterials;

                // 记录第一个材质
                if (firstMaterial == null && materials.Length > 0)
                {
                    firstMaterial = materials[0];
                }

                // 添加所有 submesh 到合并列表
                for (var i = 0; i < mesh.subMeshCount; i++)
                {
                    var combineInstance = new CombineInstance
                    {
                        mesh = mesh,
                        subMeshIndex = i,
                        transform = meshFilter_single.transform.localToWorldMatrix
                    };

                    combineList.Add(combineInstance);
                }
            }

            // 隐藏原始对象
            _combineTarget.SetActive(false);

            EditorUtility.DisplayProgressBar("FastMeshCombiner - 全局合并",
                $"合并网格中...", 0.9f);
            Debug.Log($"[FastMeshCombiner] 数据收集完成，共 {combineList.Count} 个子网格");

            // 创建一个新的 GameObject
            var newObject = new GameObject("CombinedMesh_Fast");

            // 添加 MeshRenderer 和 MeshFilter 组件
            var meshRenderer = newObject.AddComponent<MeshRenderer>();
            var meshFilter = newObject.AddComponent<MeshFilter>();

            // 设置材质
            if (firstMaterial != null)
            {
                meshRenderer.material = firstMaterial;
            }

            // 使用 CombineMeshes 直接合并所有 mesh
            var combinedMesh = new Mesh();
            combinedMesh.name = "CombinedMesh_Fast";
            combinedMesh.indexFormat = IndexFormat.UInt32;
            // mergeSubMeshes=false 保留原有的 submesh 结构，useMatrices=true 使用变换矩阵
            combinedMesh.CombineMeshes(combineList.ToArray(), false, true);

            // 自动生成第二套 UV
            Unwrapping.GenerateSecondaryUVSet(combinedMesh);

            // 将合并后的 mesh 指定到 MeshFilter 上
            meshFilter.sharedMesh = combinedMesh;

            // 设置合并结果的父物体
            newObject.transform.parent = _combineTarget.transform.parent;

            // 判断是否导出为 asset
            if (_exportMesh && _exportDirectory != null)
            {
                ExportMesh(combinedMesh, "CombinedMesh_Fast");
            }

            EditorUtility.ClearProgressBar();
            Debug.Log($"[FastMeshCombiner] 全局合并完成！");
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
