using System;
using UnityEngine;

public abstract class TrainingDebugGUIBase : MonoBehaviour
{
    [Header("GUI Layout")]
    [SerializeField] protected Vector2 panelPosition = new Vector2(24f, 24f);
    [SerializeField] protected float panelWidth = 380f;
    [SerializeField] protected float panelHeight = 340f;
    [SerializeField] protected float lineHeight = 20f;
    [SerializeField] protected float titleBarHeight = 24f;
    [SerializeField] protected bool hideWhenNotPlaying = true;
    [SerializeField] protected bool clampToScreen = true;

    GUIStyle titleStyle;
    GUIStyle labelStyle;
    GUIStyle valueStyle;
    GUIStyle goodStyle;
    GUIStyle warnStyle;
    Rect windowRect;
    Vector2 scrollPosition;
    int windowId;

    protected virtual void Awake()
    {
        if (panelWidth <= 0f)
        {
            panelWidth = 380f;
        }
        if (panelHeight <= 0f)
        {
            panelHeight = 340f;
        }
        if (titleBarHeight <= 0f)
        {
            titleBarHeight = 24f;
        }

        windowId = GetInstanceID();
        windowRect = new Rect(panelPosition.x, panelPosition.y, panelWidth, panelHeight);
    }

    protected virtual void OnGUI()
    {
        if (hideWhenNotPlaying && !Application.isPlaying)
        {
            return;
        }

        EnsureStyles();
        if (windowRect.width <= 0f || windowRect.height <= 0f)
        {
            windowRect = new Rect(panelPosition.x, panelPosition.y, panelWidth, panelHeight);
        }

        windowRect = GUI.Window(windowId, windowRect, DrawWindow, GetWindowTitle());
        panelPosition = new Vector2(windowRect.x, windowRect.y);

        if (clampToScreen)
        {
            ClampWindowToScreen();
        }

        panelPosition = new Vector2(windowRect.x, windowRect.y);
    }

    protected abstract void DrawContent();
    protected abstract string GetWindowTitle();

    protected void DrawTitle(string text)
    {
        GUILayout.Label(text, titleStyle, GUILayout.Height(lineHeight));
    }

    protected void DrawRow(string label, string value)
    {
        GUILayout.BeginHorizontal();
        GUILayout.Label(label, labelStyle, GUILayout.Width(panelWidth * 0.52f), GUILayout.Height(lineHeight));
        GUILayout.Label(value, valueStyle, GUILayout.Height(lineHeight));
        GUILayout.EndHorizontal();
    }

    protected void DrawStatus(string text, bool good)
    {
        GUILayout.Label(text, good ? goodStyle : warnStyle, GUILayout.Height(lineHeight));
    }

    void DrawWindow(int id)
    {
        float contentHeight = Mathf.Max(80f, windowRect.height - titleBarHeight - 12f);
        float contentWidth = Mathf.Max(120f, windowRect.width - 12f);

        GUILayout.BeginVertical();
        try
        {
            GUILayout.Space(2f);
            scrollPosition = GUILayout.BeginScrollView(scrollPosition, false, true, GUILayout.Width(contentWidth), GUILayout.Height(contentHeight));
            try
            {
                DrawContent();
            }
            catch (Exception ex)
            {
                Debug.LogException(ex, this);
                GUILayout.Label($"Debug GUI error: {ex.GetType().Name}", warnStyle, GUILayout.Height(lineHeight));
            }
            finally
            {
                GUILayout.EndScrollView();
            }
        }
        finally
        {
            GUILayout.EndVertical();
        }

        GUI.DragWindow(new Rect(0f, 0f, windowRect.width, titleBarHeight));
    }

    void ClampWindowToScreen()
    {
        float maxX = Mathf.Max(0f, Screen.width - windowRect.width);
        float maxY = Mathf.Max(0f, Screen.height - windowRect.height);
        windowRect.x = Mathf.Clamp(windowRect.x, 0f, maxX);
        windowRect.y = Mathf.Clamp(windowRect.y, 0f, maxY);
    }

    void EnsureStyles()
    {
        if (titleStyle != null)
        {
            return;
        }

        titleStyle = new GUIStyle(GUI.skin.label)
        {
            fontSize = 14,
            fontStyle = FontStyle.Bold
        };
        titleStyle.normal.textColor = Color.white;

        labelStyle = new GUIStyle(GUI.skin.label)
        {
            fontSize = 12
        };
        labelStyle.normal.textColor = new Color(0.85f, 0.9f, 0.95f);

        valueStyle = new GUIStyle(GUI.skin.label)
        {
            fontSize = 12,
            alignment = TextAnchor.MiddleLeft
        };
        valueStyle.normal.textColor = Color.cyan;

        goodStyle = new GUIStyle(GUI.skin.label)
        {
            fontSize = 12,
            fontStyle = FontStyle.Bold
        };
        goodStyle.normal.textColor = Color.green;

        warnStyle = new GUIStyle(GUI.skin.label)
        {
            fontSize = 12,
            fontStyle = FontStyle.Bold
        };
        warnStyle.normal.textColor = new Color(1f, 0.55f, 0.2f);
    }
}
