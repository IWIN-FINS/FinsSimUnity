using UnityEngine;

public class VelocityDebugDisplay : MonoBehaviour
{
    public enum VelocitySpace
    {
        World,
        Local
    }

    [Header("目标数据")]
    public Vector3 targetLinearVelocity;
    public Vector3 targetAngularVelocity;

    [Header("当前数据")]
    public Vector3 currentLinearVelocity;
    public Vector3 currentAngularVelocity;

    [Header("Reward数据")]
    public float currentReward;
    public float stepReward;

    [Header("阈值设置")]
    public float linearThreshold = 0.15f;
    public float angularThreshold = 0.1f;

    [Header("绑定目标")]
    [Tooltip("局部速度箭头会相对于这个Transform转换到世界坐标并绘制")]
    public Transform targetTransform;

    [Header("坐标系")]
    public VelocitySpace velocitySpace = VelocitySpace.Local;

    private float velocityScale = 1.5f;
    private const int ArrowPointCount = 5;

    [Header("箭头显示")]
    public float arrowWidth = 0.02f;
    public float arrowHeadLength = 0.2f;
    public float arrowHeadWidth = 0.12f;

    [Header("旧版文字GUI")]
    [Tooltip("仅保留旧版Velocity文字调试面板时开启。默认关闭，避免和新的可拖动GUI重复显示。")]
    public bool showLegacyOnGUI = false;

    private LineRenderer targetArrowRenderer;
    private LineRenderer currentArrowRenderer;
    private Material lineMaterial;

    void Awake()
    {
        EnsureArrowRenderers();
    }

    void LateUpdate()
    {
        if (!Application.isPlaying) return;

        EnsureArrowRenderers();

        Transform referenceTransform = targetTransform != null ? targetTransform : transform;
        Vector3 center = referenceTransform.position;
        UpdateArrowRenderer(targetArrowRenderer, referenceTransform, center, targetLinearVelocity, Color.yellow);
        UpdateArrowRenderer(currentArrowRenderer, referenceTransform, center, currentLinearVelocity, Color.green);
    }

    void OnGUI()
    {
        if (!Application.isPlaying) return;
        if (!showLegacyOnGUI) return;

        float labelX = 10f;
        float labelY = 10f;
        float lineHeight = 20f;
        float labelWidth = 300f;

        GUIStyle titleStyle = new GUIStyle(GUI.skin.label);
        titleStyle.fontSize = 14;
        titleStyle.fontStyle = FontStyle.Bold;

        GUIStyle labelStyle = new GUIStyle(GUI.skin.label);
        labelStyle.fontSize = 12;

        GUIStyle targetStyle = new GUIStyle(GUI.skin.label);
        targetStyle.fontSize = 12;
        targetStyle.normal.textColor = Color.yellow;

        GUIStyle currentStyle = new GUIStyle(GUI.skin.label);
        currentStyle.fontSize = 12;
        currentStyle.normal.textColor = Color.green;

        GUIStyle rewardStyle = new GUIStyle(GUI.skin.label);
        rewardStyle.fontSize = 12;
        rewardStyle.normal.textColor = Color.cyan;

        string spaceLabel = velocitySpace == VelocitySpace.Local ? "Local" : "World";

        float linearError = (currentLinearVelocity - targetLinearVelocity).magnitude;
        float angularError = (currentAngularVelocity - targetAngularVelocity).magnitude;

        GUIStyle errorStyle = new GUIStyle(GUI.skin.label);
        errorStyle.fontSize = 12;
        errorStyle.normal.textColor = linearError < linearThreshold && angularError < angularThreshold ? Color.cyan : Color.red;

        // 标题
        GUI.Label(new Rect(labelX, labelY, labelWidth, lineHeight), "=== Velocity Control Debug ===", titleStyle);
        labelY += lineHeight * 1.5f;
        GUI.Label(new Rect(labelX, labelY, labelWidth, lineHeight), $"Velocity Space: {spaceLabel}", labelStyle);
        labelY += lineHeight * 1.2f;

        // Reward显示
        GUI.Label(new Rect(labelX, labelY, labelWidth, lineHeight), "Step Reward:", rewardStyle);
        labelY += lineHeight;
        GUI.Label(new Rect(labelX + 20, labelY, labelWidth, lineHeight),
            string.Format("  {0:F4}", stepReward), labelStyle);
        labelY += lineHeight;
        GUI.Label(new Rect(labelX, labelY, labelWidth, lineHeight), "Accumulated Reward:", rewardStyle);
        labelY += lineHeight;
        GUI.Label(new Rect(labelX + 20, labelY, labelWidth, lineHeight),
            string.Format("  {0:F4}", currentReward), labelStyle);
        labelY += lineHeight * 1.2f;

        // 目标速度
        GUI.Label(new Rect(labelX, labelY, labelWidth, lineHeight), $"Target Linear Velocity ({spaceLabel}):", targetStyle);
        labelY += lineHeight;
        GUI.Label(new Rect(labelX + 20, labelY, labelWidth, lineHeight),
            string.Format("  X: {0,8:F3} m/s", targetLinearVelocity.x), labelStyle);
        labelY += lineHeight;
        GUI.Label(new Rect(labelX + 20, labelY, labelWidth, lineHeight),
            string.Format("  Y: {0,8:F3} m/s", targetLinearVelocity.y), labelStyle);
        labelY += lineHeight;
        GUI.Label(new Rect(labelX + 20, labelY, labelWidth, lineHeight),
            string.Format("  Z: {0,8:F3} m/s", targetLinearVelocity.z), labelStyle);
        labelY += lineHeight * 1.2f;

        GUI.Label(new Rect(labelX, labelY, labelWidth, lineHeight), $"Target Angular Velocity ({spaceLabel}):", targetStyle);
        labelY += lineHeight;
        GUI.Label(new Rect(labelX + 20, labelY, labelWidth, lineHeight),
            string.Format("  X: {0,8:F3} rad/s", targetAngularVelocity.x), labelStyle);
        labelY += lineHeight;
        GUI.Label(new Rect(labelX + 20, labelY, labelWidth, lineHeight),
            string.Format("  Y: {0,8:F3} rad/s", targetAngularVelocity.y), labelStyle);
        labelY += lineHeight;
        GUI.Label(new Rect(labelX + 20, labelY, labelWidth, lineHeight),
            string.Format("  Z: {0,8:F3} rad/s", targetAngularVelocity.z), labelStyle);
        labelY += lineHeight * 1.5f;

        // 当前速度
        GUI.Label(new Rect(labelX, labelY, labelWidth, lineHeight), $"Current Linear Velocity ({spaceLabel}):", currentStyle);
        labelY += lineHeight;
        GUI.Label(new Rect(labelX + 20, labelY, labelWidth, lineHeight),
            string.Format("  X: {0,8:F3} m/s", currentLinearVelocity.x), labelStyle);
        labelY += lineHeight;
        GUI.Label(new Rect(labelX + 20, labelY, labelWidth, lineHeight),
            string.Format("  Y: {0,8:F3} m/s", currentLinearVelocity.y), labelStyle);
        labelY += lineHeight;
        GUI.Label(new Rect(labelX + 20, labelY, labelWidth, lineHeight),
            string.Format("  Z: {0,8:F3} m/s", currentLinearVelocity.z), labelStyle);
        labelY += lineHeight * 1.2f;

        GUI.Label(new Rect(labelX, labelY, labelWidth, lineHeight), $"Current Angular Velocity ({spaceLabel}):", currentStyle);
        labelY += lineHeight;
        GUI.Label(new Rect(labelX + 20, labelY, labelWidth, lineHeight),
            string.Format("  X: {0,8:F3} rad/s", currentAngularVelocity.x), labelStyle);
        labelY += lineHeight;
        GUI.Label(new Rect(labelX + 20, labelY, labelWidth, lineHeight),
            string.Format("  Y: {0,8:F3} rad/s", currentAngularVelocity.y), labelStyle);
        labelY += lineHeight;
        GUI.Label(new Rect(labelX + 20, labelY, labelWidth, lineHeight),
            string.Format("  Z: {0,8:F3} rad/s", currentAngularVelocity.z), labelStyle);
        labelY += lineHeight * 1.5f;

        // 误差显示
        GUI.Label(new Rect(labelX, labelY, labelWidth, lineHeight), string.Format("Linear Error: {0:F3}", linearError), labelStyle);
        labelY += lineHeight;
        GUI.Label(new Rect(labelX, labelY, labelWidth, lineHeight), string.Format("Angular Error: {0:F3}", angularError), labelStyle);
        labelY += lineHeight * 1.2f;

        if (linearError < linearThreshold && angularError < angularThreshold)
        {
            GUI.Label(new Rect(labelX, labelY, labelWidth, lineHeight * 2), "TARGET REACHED!", errorStyle);
        }
    }

    private void EnsureArrowRenderers()
    {
        if (targetArrowRenderer == null)
        {
            targetArrowRenderer = CreateArrowRenderer("TargetVelocityArrow", Color.yellow);
        }

        if (currentArrowRenderer == null)
        {
            currentArrowRenderer = CreateArrowRenderer("CurrentVelocityArrow", Color.green);
        }

        if (lineMaterial == null)
        {
            Shader shader = Shader.Find("Sprites/Default");
            if (shader != null)
            {
                lineMaterial = new Material(shader);
            }
        }
    }

    private LineRenderer CreateArrowRenderer(string objectName, Color color)
    {
        Transform existingChild = transform.Find(objectName);
        GameObject arrowObject = existingChild != null ? existingChild.gameObject : new GameObject(objectName);

        if (arrowObject.transform.parent != transform)
        {
            arrowObject.transform.SetParent(transform, false);
        }

        LineRenderer lineRenderer = arrowObject.GetComponent<LineRenderer>();
        if (lineRenderer == null)
        {
            lineRenderer = arrowObject.AddComponent<LineRenderer>();
        }

        lineRenderer.useWorldSpace = true;
        lineRenderer.loop = false;
        lineRenderer.positionCount = ArrowPointCount;
        lineRenderer.startWidth = arrowWidth;
        lineRenderer.endWidth = arrowWidth;
        lineRenderer.numCapVertices = 2;
        lineRenderer.numCornerVertices = 2;
        lineRenderer.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
        lineRenderer.receiveShadows = false;
        lineRenderer.alignment = LineAlignment.View;
        lineRenderer.textureMode = LineTextureMode.Stretch;
        lineRenderer.startColor = color;
        lineRenderer.endColor = color;

        if (lineMaterial != null)
        {
            lineRenderer.material = lineMaterial;
        }

        return lineRenderer;
    }

    private void UpdateArrowRenderer(LineRenderer lineRenderer, Transform referenceTransform, Vector3 origin, Vector3 velocity, Color color)
    {
        if (lineRenderer == null) return;

        Vector3 drawVelocity = velocity;
        if (velocitySpace == VelocitySpace.Local && referenceTransform != null)
        {
            drawVelocity = referenceTransform.TransformDirection(velocity);
        }

        if (lineMaterial != null && lineRenderer.material == null)
        {
            lineRenderer.material = lineMaterial;
        }

        lineRenderer.startColor = color;
        lineRenderer.endColor = color;

        if (drawVelocity.magnitude < 0.001f)
        {
            lineRenderer.enabled = false;
            return;
        }

        lineRenderer.enabled = true;

        Vector3 endPoint = origin + drawVelocity * velocityScale;
        Vector3 direction = drawVelocity.normalized;
        Vector3 left = Vector3.Cross(direction, Vector3.up).normalized * arrowHeadWidth;
        Vector3 right = Vector3.Cross(Vector3.up, direction).normalized * arrowHeadWidth;

        if (left.magnitude < 0.001f)
        {
            left = Vector3.Cross(direction, Vector3.forward).normalized * arrowHeadWidth;
            right = -left;
        }

        Vector3 headBase = endPoint - direction * arrowHeadLength;
        lineRenderer.SetPosition(0, origin);
        lineRenderer.SetPosition(1, endPoint);
        lineRenderer.SetPosition(2, headBase + left);
        lineRenderer.SetPosition(3, endPoint);
        lineRenderer.SetPosition(4, headBase + right);
    }
}
