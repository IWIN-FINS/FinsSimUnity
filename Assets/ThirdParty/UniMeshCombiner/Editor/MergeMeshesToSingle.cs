using System.Collections.Generic;
using System.IO;
using UnityEditor;
using UnityEngine;
using UnityEngine.Rendering;

namespace UniMeshCombiner
{
    public class MergeMeshesToSingle : EditorWindow
    {
        // 要合并的父物体（包含多个 Mesh 的子物体）
        private GameObject _mergeTarget = null;

        // 是否导出 Mesh 为 .asset 文件
        private bool _exportMesh = true;

        // 导出目录
        private DefaultAsset _exportDirectory = null;

        [MenuItem("Window/MergeMeshesToSingle")]
        public static void Open()
        {
            GetWindow<MergeMeshesToSingle>("MergeMeshesToSingle").Show();
        }

        void OnGUI()
        {
            _mergeTarget = (GameObject)EditorGUILayout.ObjectField("Merge Target", _mergeTarget, typeof(GameObject), true);
            _exportMesh = EditorGUILayout.Toggle("Export Mesh", _exportMesh);
            _exportDirectory = (DefaultAsset)EditorGUILayout.ObjectField("Export Directory", _exportDirectory, typeof(DefaultAsset), true);

            if (GUILayout.Button("Merge into Single Mesh"))
            {
                if (_mergeTarget == null)
                {
                    Debug.LogWarning("Please assign a target GameObject!");
                    return;
                }

                MergeMeshes();
            }
        }

        void MergeMeshes()
        {
            var meshFilters = _mergeTarget.GetComponentsInChildren<MeshFilter>();
            if (meshFilters.Length == 0)
            {
                Debug.LogWarning("No MeshFilter found under the target GameObject.");
                return;
            }

            List<CombineInstance> combineList = new List<CombineInstance>();

            foreach (var mf in meshFilters)
            {
                if (mf.sharedMesh == null) continue;

                CombineInstance ci = new CombineInstance
                {
                    mesh = mf.sharedMesh,
                    transform = mf.transform.localToWorldMatrix
                };
                combineList.Add(ci);
            }

            // 创建新的 GameObject 显示合并结果
            GameObject combinedObj = new GameObject(_mergeTarget.name + "_SingleMesh");
            combinedObj.transform.parent = _mergeTarget.transform.parent;

            MeshFilter combinedMF = combinedObj.AddComponent<MeshFilter>();
            MeshRenderer combinedMR = combinedObj.AddComponent<MeshRenderer>();

            Mesh combinedMesh = new Mesh
            {
                name = _mergeTarget.name + "_Mesh",
                indexFormat = IndexFormat.UInt32 // 支持大顶点数
            };
            combinedMesh.CombineMeshes(combineList.ToArray(), true, true); // mergeSubMeshes=true, useMatrices=true

            // 自动生成第二套 UV（可选）
            Unwrapping.GenerateSecondaryUVSet(combinedMesh);

            combinedMF.sharedMesh = combinedMesh;

            // 材质处理：选第一个子物体的材质
            var firstRenderer = meshFilters[0].GetComponent<Renderer>();
            if (firstRenderer != null)
            {
                combinedMR.sharedMaterial = firstRenderer.sharedMaterial;
            }

            // 隐藏原物体
            _mergeTarget.SetActive(false);

            // 导出 Mesh
            if (_exportMesh && _exportDirectory != null)
            {
                ExportMesh(combinedMesh);
            }

            Debug.Log($"Merged {meshFilters.Length} meshes into a single mesh.");
        }

        void ExportMesh(Mesh mesh)
        {
            string exportDirectoryPath = AssetDatabase.GetAssetPath(_exportDirectory);
            string fileName = mesh.name + ".asset";
            string exportPath = Path.Combine(exportDirectoryPath, fileName);

            AssetDatabase.CreateAsset(mesh, exportPath);
            AssetDatabase.SaveAssets();
            AssetDatabase.Refresh();

            Debug.Log($"Exported combined mesh to: {exportPath}");
        }
    }
}
