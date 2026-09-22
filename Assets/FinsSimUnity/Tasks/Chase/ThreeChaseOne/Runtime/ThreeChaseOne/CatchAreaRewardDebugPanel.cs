using System.Text;
using Unity.MLAgents.Policies;
using UnityEngine;

/// <summary>
/// Renders a scrollable, section-based runtime HUD for catch-area reward debugging.
/// This keeps presentation logic out of CatchAreaManager so the data source and view stay separate.
/// </summary>
public sealed class CatchAreaRewardDebugPanel
{
    private const float OuterPadding = 14f;
    private const float SectionSpacing = 12f;
    private const float RowSpacing = 5f;
    private const float LabelColumnWidth = 240f;

    private Vector2 scrollPosition;
    private bool showCaptureSection = true;
    private bool showBaselineSection = true;
    private bool showStressTestSection = true;
    private bool showThrusterSection = true;
    private bool showChaserSection = true;
    private bool showPreySection = true;
    private bool showRoleHighlights;

    private GUIStyle panelStyle;
    private GUIStyle titleStyle;
    private GUIStyle subtitleStyle;
    private GUIStyle sectionStyle;
    private GUIStyle sectionTitleStyle;
    private GUIStyle sectionToggleButtonStyle;
    private GUIStyle cardStyle;
    private GUIStyle cardTitleStyle;
    private GUIStyle keyStyle;
    private GUIStyle valueStyle;
    private GUIStyle highlightLabelStyle;
    private Texture2D panelTexture;
    private Texture2D sectionTexture;
    private Texture2D cardTexture;
    private Texture2D highlightTexture;

    public void Draw(CatchAreaManager manager)
    {
        if (manager == null)
        {
            return;
        }

        EnsureStyles(manager.rewardDebugFontSize);
        Rect panelRect = new Rect(
            manager.rewardDebugPanelPosition.x,
            manager.rewardDebugPanelPosition.y,
            manager.rewardDebugPanelSize.x,
            manager.rewardDebugPanelSize.y);

        GUI.Box(panelRect, GUIContent.none, panelStyle);

        Rect contentRect = new Rect(
            panelRect.x + OuterPadding,
            panelRect.y + OuterPadding,
            panelRect.width - OuterPadding * 2f,
            panelRect.height - OuterPadding * 2f);

        GUILayout.BeginArea(contentRect);
        DrawHeader(manager);

        scrollPosition = GUILayout.BeginScrollView(scrollPosition, false, true, GUILayout.ExpandHeight(true));

        DrawCaptureSection(manager);
        DrawBaselineSection(manager);
        DrawStressTestSection(manager);
        DrawThrusterSection(manager);
        DrawChaserSection(manager);
        DrawPreySection(manager);

        GUILayout.EndScrollView();
        GUILayout.EndArea();

    }

    public void DrawPersistentOverlays(CatchAreaManager manager)
    {
        if (!showRoleHighlights || manager == null)
        {
            return;
        }

        EnsureStyles(manager.rewardDebugFontSize);
        DrawRoleHighlights(manager);
    }

    private void DrawHeader(CatchAreaManager manager)
    {
        ThreeChaseOneBaselineController baselineController =
            ThreeChaseOneBaselineController.ResolveActive(manager);
        string baselineKey = baselineController != null ? baselineController.toggleBaselineKey.ToString() : "F7";

        GUILayout.Label("Three Chase One Reward Debug Panel", titleStyle);
        GUILayout.Label(
            $"Toggle HUD: {manager.toggleRewardDebugKey}    Toggle Baseline: {baselineKey}    Scroll when content exceeds the panel height.",
            subtitleStyle);

        GUILayout.BeginHorizontal();
        if (GUILayout.Button("Expand All Sections", sectionToggleButtonStyle, GUILayout.Width(160f)))
        {
            SetAllSections(true);
        }

        if (GUILayout.Button("Collapse All Sections", sectionToggleButtonStyle, GUILayout.Width(160f)))
        {
            SetAllSections(false);
        }

        string highlightButtonLabel = showRoleHighlights ? "Role Highlights: On" : "Role Highlights: Off";
        if (GUILayout.Button(highlightButtonLabel, sectionToggleButtonStyle, GUILayout.Width(180f)))
        {
            showRoleHighlights = !showRoleHighlights;
        }
        GUILayout.EndHorizontal();

        GUILayout.Space(SectionSpacing);
    }

    private void DrawCaptureSection(CatchAreaManager manager)
    {
        BeginSection("Capture And Episode State", ref showCaptureSection);
        if (!showCaptureSection)
        {
            return;
        }

        DrawKeyValueRow("Net collision detected", FormatBool(manager.HasNetCollision));
        DrawKeyValueRow("Obi net collision detected", FormatBool(manager.HasObiNetCollision));
        DrawKeyValueRow("Obi net collision debug", manager.ObiNetCollisionDebug);
        DrawKeyValueRow("Capture criterion", manager.ActiveCaptureCriterion.ToString());
        DrawKeyValueRow("Capture already resolved", FormatBool(manager.CaptureResolved));
        DrawKeyValueRow("Last reward event", manager.LastRewardEventMessage);
        DrawKeyValueRow("Event age", $"{manager.LastRewardEventAgeSeconds:F1} seconds");
        DrawKeyValueRow("Net center (world position)", FormatVector(manager.GetNetCenter()));
        DrawKeyValueRow("Geometric capture progress", $"{manager.CaptureProgress01:P0}");
        DrawKeyValueRow("Net surface capture qualified", FormatBool(manager.NetSurfaceCaptureQualified));
        DrawKeyValueRow("Fish near net surface", FormatBool(manager.FishNearNetSurface));
        DrawKeyValueRow("Fish distance to net surface", $"{manager.FishNetSurfaceDistance:F2}");
        DrawKeyValueRow("Net surface capture debug", manager.NetSurfaceCaptureDebug);
        DrawKeyValueRow("Fish distance to nearest netter", $"{manager.NearestNetterFishDistance:F2}");
        DrawKeyValueRow("Fish within UUV capture distance", FormatBool(manager.FishWithinUuvCaptureDistance));
        DrawKeyValueRow("Net collision OR UUV proximity", FormatBool(manager.NetCollisionOrUuvProximityQualified));
        if (manager.NetSurfaceModel != null && manager.NetSurfaceModel.IsReady())
        {
            DrawKeyValueRow("Net width", $"{manager.NetSurfaceModel.GetNetWidth():F2}");
            DrawKeyValueRow("Net height", $"{manager.NetSurfaceModel.GetNetHeight():F2}");
        }
        EndSection();
    }

    private void DrawBaselineSection(CatchAreaManager manager)
    {
        BeginSection("Baseline Controller", ref showBaselineSection);
        if (!showBaselineSection)
        {
            return;
        }

        ThreeChaseOneBaselineController baselineController =
            ThreeChaseOneBaselineController.ResolveActive(manager);
        if (baselineController == null)
        {
            DrawKeyValueRow("Controller status", "No active ThreeChaseOneBaselineController was found in the scene.");
            EndSection();
            return;
        }

        DrawKeyValueRow("Baseline enabled", FormatBool(baselineController.baselineEnabled));
        DrawKeyValueRow("Baseline controls chasers", FormatBool(baselineController.baselineControlsChasers));
        DrawKeyValueRow("Baseline controls prey", FormatBool(baselineController.baselineControlsPrey));
        DrawKeyValueRow("Auto behavior-type switching", FormatBool(baselineController.autoSwitchBehaviorType));
        DrawKeyValueRow("Chaser baseline mode", NicifyName(baselineController.chaserMode.ToString()));
        DrawKeyValueRow("Prey baseline mode", NicifyName(baselineController.preyMode.ToString()));
        DrawKeyValueRow("Controller summary", baselineController.GetStatusSummary(manager));
        EndSection();
    }

    private void DrawThrusterSection(CatchAreaManager manager)
    {
        BeginSection("Thrusters And Terminal Rewards", ref showThrusterSection);
        if (!showThrusterSection)
        {
            return;
        }

        DrawKeyValueRow("Unified thruster override", FormatBool(manager.UseUnifiedThrusterMaxThrust));
        DrawKeyValueRow("Vertical thruster max thrust", $"{manager.UnifiedVerticalThrusterMaxThrust:F0}");
        DrawKeyValueRow("Horizontal thruster max thrust", $"{manager.UnifiedHorizontalThrusterMaxThrust:F0}");
        DrawKeyValueRow("Chaser terminal capture reward", $"{manager.chaserCaptureReward:F2}");
        DrawKeyValueRow("Fish terminal capture penalty", $"{manager.fishCapturePenalty:F2}");
        EndSection();
    }

    private void DrawStressTestSection(CatchAreaManager manager)
    {
        BeginSection("Net Separation Stress Test", ref showStressTestSection);
        if (!showStressTestSection)
        {
            return;
        }

        NetSeparationStressTest stressTest = NetSeparationStressTest.ResolveActive(manager);
        if (stressTest == null)
        {
            DrawKeyValueRow("Stress test status", "No active NetSeparationStressTest was found in the scene.");
            EndSection();
            return;
        }

        DrawKeyValueRow("Stress test enabled", FormatBool(stressTest.stressTestEnabled));
        DrawKeyValueRow("Currently providing actions", FormatBool(stressTest.IsProvidingActions));
        DrawKeyValueRow("Toggle key", stressTest.toggleTestKey.ToString());
        DrawKeyValueRow("Reset and restart key", stressTest.resetAndRestartKey.ToString());
        DrawKeyValueRow("Allow vertical separation", FormatBool(stressTest.allowVerticalSeparationMotion));
        DrawKeyValueRow("Continuous outward push", FormatBool(stressTest.pushContinuously));
        DrawKeyValueRow("Target outward distance", $"{stressTest.outwardPushDistance:F2}");
        DrawKeyValueRow(
            "Console logging compiled in",
            ThreeChaseOneRuntimeLog.ConsoleLogsCompiledIn
                ? "Yes (UNITY_EDITOR / DEVELOPMENT_BUILD / THREE_CHASE_ONE_RUNTIME_LOGS)"
                : "No");
        DrawKeyValueRow("Last status", stressTest.LastStatusMessage);
        DrawKeyValueRow("Status age", $"{stressTest.LastStatusAgeSeconds:F1} seconds");
        DrawKeyValueRow("Last snapshot", stressTest.LastSnapshotMessage);
        DrawKeyValueRow("Snapshot age", $"{stressTest.LastSnapshotAgeSeconds:F1} seconds");
        EndSection();
    }

    private void DrawChaserSection(CatchAreaManager manager)
    {
        BeginSection("Chaser Agents", ref showChaserSection);
        if (!showChaserSection)
        {
            return;
        }

        ThreeChaseOneBaselineController baselineController =
            ThreeChaseOneBaselineController.ResolveActive(manager);
        DrawChaserCard("Netter 1", manager, manager.netter1, manager.fish, baselineController, manager.GetNetCenter());
        DrawChaserCard("Netter 2", manager, manager.netter2, manager.fish, baselineController, manager.GetNetCenter());
        DrawChaserCard("Herder", manager, manager.herder, manager.fish, baselineController, manager.GetNetCenter());
        EndSection();
    }

    private void DrawPreySection(CatchAreaManager manager)
    {
        BeginSection("Prey Agent", ref showPreySection);
        if (!showPreySection)
        {
            return;
        }

        ThreeChaseOneBaselineController baselineController =
            ThreeChaseOneBaselineController.ResolveActive(manager);
        DrawPreyCard("Fish", manager.fish, baselineController);
        EndSection();
    }

    private void DrawChaserCard(
        string title,
        CatchAreaManager manager,
        ChaserAgent agent,
        PreyAgent fish,
        ThreeChaseOneBaselineController baselineController,
        Vector3 netCenter)
    {
        GUILayout.BeginVertical(cardStyle);
        GUILayout.Label(title, cardTitleStyle);

        if (agent == null)
        {
            DrawKeyValueRow("Reference", "This chaser reference is null.");
            GUILayout.EndVertical();
            GUILayout.Space(SectionSpacing);
            return;
        }

        float distanceToFish = fish != null ? Vector3.Distance(agent.transform.position, fish.transform.position) : 0f;
        DrawKeyValueRow("Behavior type", GetBehaviorTypeLabel(agent));
        DrawKeyValueRow("Role", NicifyName(agent.role.ToString()));
        DrawKeyValueRow("Reward mode", NicifyName(agent.rewardMode.ToString()));
        DrawKeyValueRow("Current world position", FormatVector(agent.transform.position));
        DrawKeyValueRow("Current step reward", $"{agent.LastStepReward:F3}");
        DrawKeyValueRow("Episode cumulative reward", $"{agent.GetCumulativeReward():F3}");
        DrawKeyValueRow("Distance to fish", $"{distanceToFish:F2}");
        DrawKeyValueRow("Time penalty contribution", $"{agent.LastStepPenaltyReward:F3}");
        DrawKeyValueRow("Angular velocity penalty", $"{agent.LastAngularVelocityPenaltyReward:F3}");
        DrawKeyValueRow("Linear velocity penalty", $"{agent.LastLinearVelocityPenaltyReward:F3}");
        DrawKeyValueRow("Action magnitude penalty", $"{agent.LastActionMagnitudePenaltyReward:F3}");
        DrawKeyValueRow("Action delta penalty", $"{agent.LastActionDeltaPenaltyReward:F3}");

        if (agent.rewardMode == ChaserRewardMode.SimpleChasePrey)
        {
            DrawKeyValueRow("Simple chase reward contribution", $"{agent.LastSimpleChaseReward:F3}");
            DrawKeyValueRow("Fish-closing progress contribution", $"{agent.LastFishClosingReward:F3}");
        }
        else if (agent.role == AgentRole.Netter)
        {
            float partnerSpacing = manager.NetSurfaceModel != null && manager.NetSurfaceModel.IsReady()
                ? manager.NetSurfaceModel.GetNetWidth()
                : (agent.partnerNetter != null
                    ? Vector3.Distance(agent.transform.position, agent.partnerNetter.transform.position)
                    : 0f);
            DrawKeyValueRow("Distance to partner netter", $"{partnerSpacing:F2}");
            DrawKeyValueRow("Fish-closing reward contribution", $"{agent.LastFishClosingReward:F3}");
            DrawKeyValueRow("Net-width reward contribution", $"{agent.LastNetterSpacingReward:F3}");
            DrawKeyValueRow("Net-width recovery progress reward", $"{agent.LastNetterSpacingProgressReward:F3}");
        }
        else
        {
            float fishToNetCenter = fish != null ? Vector3.Distance(fish.transform.position, netCenter) : 0f;
            DrawKeyValueRow("Fish distance to net center", $"{fishToNetCenter:F2}");
            DrawKeyValueRow("Fish-closing reward contribution", $"{agent.LastHerderFishClosingReward:F3}");
            DrawKeyValueRow("Herding progress reward contribution", $"{agent.LastHerdingProgressReward:F3}");
        }

        DrawKeyValueRow(
            "Baseline heuristic details",
            baselineController != null
                ? baselineController.GetChaserHeuristicSummary(agent)
                : "No active baseline controller is available.");

        GUILayout.EndVertical();
        GUILayout.Space(SectionSpacing);
    }

    private void DrawPreyCard(
        string title,
        PreyAgent preyAgent,
        ThreeChaseOneBaselineController baselineController)
    {
        GUILayout.BeginVertical(cardStyle);
        GUILayout.Label(title, cardTitleStyle);

        if (preyAgent == null)
        {
            DrawKeyValueRow("Reference", "This prey reference is null.");
            GUILayout.EndVertical();
            GUILayout.Space(SectionSpacing);
            return;
        }

        float averageDistance = 0f;
        float nearestDistance = float.PositiveInfinity;
        int count = 0;
        Transform[] chasers = preyAgent.chasers;
        if (chasers != null)
        {
            foreach (Transform chaser in chasers)
            {
                if (chaser == null)
                {
                    continue;
                }

                float distance = Vector3.Distance(preyAgent.transform.position, chaser.position);
                averageDistance += distance;
                nearestDistance = Mathf.Min(nearestDistance, distance);
                count++;
            }
        }

        averageDistance = count > 0 ? averageDistance / count : 0f;
        nearestDistance = float.IsPositiveInfinity(nearestDistance) ? 0f : nearestDistance;

        DrawKeyValueRow("Behavior type", GetBehaviorTypeLabel(preyAgent));
        DrawKeyValueRow("Current world position", FormatVector(preyAgent.transform.position));
        DrawKeyValueRow("Current step reward", $"{preyAgent.LastStepReward:F3}");
        DrawKeyValueRow("Episode cumulative reward", $"{preyAgent.GetCumulativeReward():F3}");
        DrawKeyValueRow("Average distance to chasers", $"{averageDistance:F2}");
        DrawKeyValueRow("Nearest chaser distance", $"{nearestDistance:F2}");
        DrawKeyValueRow("Survival reward contribution", $"{preyAgent.LastSurvivalReward:F3}");
        DrawKeyValueRow("Average-separation progress reward", $"{preyAgent.LastAverageSeparationReward:F3}");
        DrawKeyValueRow("Nearest-threat progress reward", $"{preyAgent.LastNearestThreatReward:F3}");
        DrawKeyValueRow(
            "Baseline heuristic details",
            baselineController != null
                ? baselineController.GetPreyHeuristicSummary(preyAgent)
                : "No active baseline controller is available.");

        GUILayout.EndVertical();
        GUILayout.Space(SectionSpacing);
    }

    private void BeginSection(string title, ref bool expanded)
    {
        GUILayout.BeginVertical(sectionStyle);
        GUILayout.BeginHorizontal();
        GUILayout.Label(title, sectionTitleStyle);
        GUILayout.FlexibleSpace();
        string buttonLabel = expanded ? "Hide" : "Show";
        if (GUILayout.Button(buttonLabel, sectionToggleButtonStyle, GUILayout.Width(80f)))
        {
            expanded = !expanded;
        }
        GUILayout.EndHorizontal();

        if (expanded)
        {
            GUILayout.Space(RowSpacing);
        }
        else
        {
            GUILayout.EndVertical();
            GUILayout.Space(SectionSpacing);
        }
    }

    private void EndSection()
    {
        GUILayout.EndVertical();
        GUILayout.Space(SectionSpacing);
    }

    private void DrawKeyValueRow(string label, string value)
    {
        GUILayout.BeginHorizontal();
        GUILayout.Label(label, keyStyle, GUILayout.Width(LabelColumnWidth));
        GUILayout.Label(string.IsNullOrEmpty(value) ? "-" : value, valueStyle, GUILayout.ExpandWidth(true));
        GUILayout.EndHorizontal();
        GUILayout.Space(RowSpacing);
    }

    private void DrawRoleHighlights(CatchAreaManager manager)
    {
        Camera camera = Camera.main;
        if (camera == null)
        {
            return;
        }

        DrawRoleHighlight(camera, "NETTER 1", manager.netter1, new Color(0.2f, 0.75f, 1f, 1f));
        DrawRoleHighlight(camera, "NETTER 2", manager.netter2, new Color(0.25f, 1f, 0.55f, 1f));
        DrawRoleHighlight(camera, "HERDER", manager.herder, new Color(1f, 0.78f, 0.25f, 1f));
        DrawRoleHighlight(camera, "PREY", manager.fish, new Color(1f, 0.25f, 0.4f, 1f));
    }

    private void DrawRoleHighlight(Camera camera, string label, Component target, Color color)
    {
        if (target == null)
        {
            return;
        }

        Vector3 worldPoint = GetHighlightWorldPoint(target.transform);
        Vector3 screenPoint = camera.WorldToScreenPoint(worldPoint);
        if (screenPoint.z <= 0f)
        {
            return;
        }

        float x = screenPoint.x;
        float y = Screen.height - screenPoint.y;
        Rect labelRect = new Rect(x - 54f, y - 42f, 108f, 24f);
        Rect horizontalRect = new Rect(x - 14f, y, 28f, 2f);
        Rect verticalRect = new Rect(x - 1f, y - 13f, 2f, 28f);

        Color previousColor = GUI.color;
        GUI.color = color;
        GUI.Box(labelRect, label, highlightLabelStyle);
        GUI.DrawTexture(horizontalRect, Texture2D.whiteTexture);
        GUI.DrawTexture(verticalRect, Texture2D.whiteTexture);
        GUI.color = previousColor;
    }

    private static Vector3 GetHighlightWorldPoint(Transform target)
    {
        Renderer[] renderers = target.GetComponentsInChildren<Renderer>();
        if (renderers.Length == 0)
        {
            return target.position + Vector3.up * 1.5f;
        }

        Bounds bounds = renderers[0].bounds;
        for (int i = 1; i < renderers.Length; i++)
        {
            bounds.Encapsulate(renderers[i].bounds);
        }

        return bounds.center + Vector3.up * (bounds.extents.y + 0.6f);
    }

    private void EnsureStyles(int fontSize)
    {
        if (panelTexture == null)
        {
            panelTexture = CreateTexture(new Color(0.08f, 0.1f, 0.14f, 0.95f));
            sectionTexture = CreateTexture(new Color(0.14f, 0.18f, 0.24f, 0.95f));
            cardTexture = CreateTexture(new Color(0.2f, 0.24f, 0.3f, 0.95f));
            highlightTexture = CreateTexture(new Color(0f, 0f, 0f, 0.72f));
        }

        if (panelStyle == null)
        {
            panelStyle = new GUIStyle(GUI.skin.box);
            panelStyle.padding = new RectOffset(12, 12, 12, 12);
            panelStyle.normal.background = panelTexture;
            panelStyle.border = new RectOffset(8, 8, 8, 8);
        }

        if (titleStyle == null)
        {
            titleStyle = new GUIStyle(GUI.skin.label);
            titleStyle.fontStyle = FontStyle.Bold;
            titleStyle.wordWrap = true;
            titleStyle.normal.textColor = Color.white;
        }

        if (subtitleStyle == null)
        {
            subtitleStyle = new GUIStyle(GUI.skin.label);
            subtitleStyle.wordWrap = true;
            subtitleStyle.normal.textColor = new Color(0.83f, 0.88f, 0.95f, 1f);
        }

        if (sectionStyle == null)
        {
            sectionStyle = new GUIStyle(GUI.skin.box);
            sectionStyle.padding = new RectOffset(14, 14, 12, 12);
            sectionStyle.margin = new RectOffset(0, 0, 0, 0);
            sectionStyle.normal.background = sectionTexture;
        }

        if (sectionTitleStyle == null)
        {
            sectionTitleStyle = new GUIStyle(GUI.skin.label);
            sectionTitleStyle.fontStyle = FontStyle.Bold;
            sectionTitleStyle.normal.textColor = Color.white;
        }

        if (sectionToggleButtonStyle == null)
        {
            sectionToggleButtonStyle = new GUIStyle(GUI.skin.button);
            sectionToggleButtonStyle.alignment = TextAnchor.MiddleCenter;
            sectionToggleButtonStyle.padding = new RectOffset(10, 10, 6, 6);
        }

        if (cardStyle == null)
        {
            cardStyle = new GUIStyle(GUI.skin.box);
            cardStyle.padding = new RectOffset(14, 14, 12, 12);
            cardStyle.margin = new RectOffset(0, 0, 0, 0);
            cardStyle.normal.background = cardTexture;
        }

        if (cardTitleStyle == null)
        {
            cardTitleStyle = new GUIStyle(GUI.skin.label);
            cardTitleStyle.fontStyle = FontStyle.Bold;
            cardTitleStyle.normal.textColor = Color.white;
        }

        if (keyStyle == null)
        {
            keyStyle = new GUIStyle(GUI.skin.label);
            keyStyle.fontStyle = FontStyle.Bold;
            keyStyle.wordWrap = true;
            keyStyle.normal.textColor = new Color(0.92f, 0.95f, 0.98f, 1f);
        }

        if (valueStyle == null)
        {
            valueStyle = new GUIStyle(GUI.skin.label);
            valueStyle.wordWrap = true;
            valueStyle.normal.textColor = Color.white;
        }

        if (highlightLabelStyle == null)
        {
            highlightLabelStyle = new GUIStyle(GUI.skin.box);
            highlightLabelStyle.alignment = TextAnchor.MiddleCenter;
            highlightLabelStyle.fontStyle = FontStyle.Bold;
            highlightLabelStyle.normal.textColor = Color.white;
            highlightLabelStyle.normal.background = highlightTexture;
            highlightLabelStyle.padding = new RectOffset(8, 8, 4, 4);
        }

        titleStyle.fontSize = fontSize + 4;
        subtitleStyle.fontSize = fontSize - 1;
        sectionTitleStyle.fontSize = fontSize + 1;
        cardTitleStyle.fontSize = fontSize;
        keyStyle.fontSize = fontSize;
        valueStyle.fontSize = fontSize;
        sectionToggleButtonStyle.fontSize = fontSize - 1;
        highlightLabelStyle.fontSize = fontSize;
    }

    private void SetAllSections(bool expanded)
    {
        showCaptureSection = expanded;
        showBaselineSection = expanded;
        showStressTestSection = expanded;
        showThrusterSection = expanded;
        showChaserSection = expanded;
        showPreySection = expanded;
    }

    private static string FormatBool(bool value)
    {
        return value ? "Yes" : "No";
    }

    private static string FormatVector(Vector3 value)
    {
        return $"({value.x:F1}, {value.y:F1}, {value.z:F1})";
    }

    private static string GetBehaviorTypeLabel(Component component)
    {
        if (component != null && component.TryGetComponent(out BehaviorParameters behaviorParameters))
        {
            return NicifyName(behaviorParameters.BehaviorType.ToString());
        }

        return "Not available";
    }

    private static string NicifyName(string value)
    {
        if (string.IsNullOrEmpty(value))
        {
            return "-";
        }

        StringBuilder builder = new StringBuilder(value.Length + 8);
        for (int i = 0; i < value.Length; i++)
        {
            char current = value[i];
            if (i > 0
                && char.IsUpper(current)
                && (char.IsLower(value[i - 1]) || char.IsDigit(value[i - 1])))
            {
                builder.Append(' ');
            }

            builder.Append(current);
        }

        return builder.ToString();
    }

    private static Texture2D CreateTexture(Color color)
    {
        Texture2D texture = new Texture2D(1, 1, TextureFormat.RGBA32, false);
        texture.hideFlags = HideFlags.HideAndDontSave;
        texture.SetPixel(0, 0, color);
        texture.Apply();
        return texture;
    }
}
