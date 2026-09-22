using System.Collections.Generic;
using UnityEngine;

/// <summary>
/// Uses Habrador's original ClothSimulationTutorial solver to drive a goal-sized
/// triangular mesh. That solver has no native Unity Rigidbody contact solver;
/// the MeshCollider is deliberately added by the test scene as a comparison proxy.
/// </summary>
[RequireComponent(typeof(MeshFilter), typeof(MeshRenderer))]
public sealed class HabradorGoalClothDriver : MonoBehaviour
{
    [Min(2)] public int columns = 25;
    [Min(2)] public int rows = 15;
    public float width = 5f;
    public float height = 3f;
    public float bottomY = 0.25f;

    private ClothSimulationTutorial simulation;

    private void Awake()
    {
        MeshFilter filter = GetComponent<MeshFilter>();
        simulation = new ClothSimulationTutorial(
            filter,
            new GoalClothData(columns, rows, width, height, bottomY),
            Vector3.zero,
            1f,
            bendingCompliance: 0.02f);
    }

    private void FixedUpdate()
    {
        simulation.MyFixedUpdate();
    }

    private void Update()
    {
        simulation.MyUpdate();
    }

    private sealed class GoalClothData : ClothData
    {
        private readonly float[] vertices;
        private readonly int[] triangles;

        public override float[] GetVerts => vertices;
        public override int[] GetFaceTriIds => triangles;

        public GoalClothData(int cols, int rows, float width, float height, float bottomY)
        {
            vertices = new float[cols * rows * 3];
            for (int row = 0; row < rows; row++)
            {
                float v = row / (float)(rows - 1);
                for (int col = 0; col < cols; col++)
                {
                    float u = col / (float)(cols - 1);
                    int i = (row * cols + col) * 3;
                    vertices[i] = Mathf.Lerp(-width * 0.5f, width * 0.5f, u);
                    vertices[i + 1] = Mathf.Lerp(bottomY + height, bottomY, v);
                    vertices[i + 2] = 0f;
                }
            }

            triangles = new int[(rows - 1) * (cols - 1) * 6];
            int t = 0;
            for (int row = 0; row < rows - 1; row++)
            {
                for (int col = 0; col < cols - 1; col++)
                {
                    int a = row * cols + col;
                    int b = a + 1;
                    int c = a + cols;
                    int d = c + 1;
                    triangles[t++] = a;
                    triangles[t++] = b;
                    triangles[t++] = c;
                    triangles[t++] = b;
                    triangles[t++] = d;
                    triangles[t++] = c;
                }
            }
        }
    }
}
