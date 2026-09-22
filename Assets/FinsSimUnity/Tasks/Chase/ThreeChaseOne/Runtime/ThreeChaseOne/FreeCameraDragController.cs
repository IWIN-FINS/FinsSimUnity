using UnityEngine;
using UnityEngine.EventSystems;
using UnityEngine.InputSystem;
using NWH.DWP2.ShipController;

public sealed class FreeCameraDragController : MonoBehaviour
{
    [SerializeField] private float moveSpeed = 8f;
    [SerializeField] private float fastMoveMultiplier = 3f;
    [SerializeField] private float mouseSensitivity = 0.12f;
    [SerializeField] private float minPitch = -89f;
    [SerializeField] private float maxPitch = 89f;
    [SerializeField] private bool ignoreInputWhenPointerOverUI = true;
    [SerializeField] private bool suppressShipInputProviders = true;
    [SerializeField] private bool showDebugOverlay = false;

    private readonly System.Collections.Generic.List<ShipInputProvider> suppressedShipInputProviders = new();
    private static int activeInputControllerCount;
    private float yaw;
    private float pitch;

    public static bool IsInputActive { get; private set; }

    private void OnEnable()
    {
        activeInputControllerCount++;
        IsInputActive = activeInputControllerCount > 0;
        Vector3 eulerAngles = transform.rotation.eulerAngles;
        yaw = eulerAngles.y;
        pitch = NormalizeAngle(eulerAngles.x);
        SuppressShipInputProviders();
    }

    private void OnDisable()
    {
        RestoreShipInputProviders();
        activeInputControllerCount = Mathf.Max(0, activeInputControllerCount - 1);
        IsInputActive = activeInputControllerCount > 0;
    }

    private void Update()
    {
        Keyboard keyboard = Keyboard.current;
        Mouse mouse = Mouse.current;
        if (keyboard == null || mouse == null)
        {
            return;
        }

        SuppressShipInputProviders();
        Move(keyboard);

        if (!mouse.leftButton.isPressed)
        {
            return;
        }

        if (ignoreInputWhenPointerOverUI && IsPointerOverUI())
        {
            return;
        }

        Vector2 mouseDelta = mouse.delta.ReadValue();
        yaw += mouseDelta.x * mouseSensitivity;
        pitch = Mathf.Clamp(pitch - mouseDelta.y * mouseSensitivity, minPitch, maxPitch);
        transform.rotation = Quaternion.Euler(pitch, yaw, 0f);
    }

    private void Move(Keyboard keyboard)
    {
        Vector3 localMovement = Vector3.zero;

        if (keyboard.wKey.isPressed)
        {
            localMovement.z += 1f;
        }

        if (keyboard.sKey.isPressed)
        {
            localMovement.z -= 1f;
        }

        if (keyboard.dKey.isPressed)
        {
            localMovement.x += 1f;
        }

        if (keyboard.aKey.isPressed)
        {
            localMovement.x -= 1f;
        }

        if (keyboard.eKey.isPressed || keyboard.spaceKey.isPressed)
        {
            localMovement.y += 1f;
        }

        if (keyboard.qKey.isPressed || keyboard.leftCtrlKey.isPressed || keyboard.rightCtrlKey.isPressed)
        {
            localMovement.y -= 1f;
        }

        if (localMovement.sqrMagnitude <= 0f)
        {
            return;
        }

        float speed = moveSpeed;
        if (keyboard.leftShiftKey.isPressed || keyboard.rightShiftKey.isPressed)
        {
            speed *= fastMoveMultiplier;
        }

        transform.position += transform.TransformDirection(localMovement.normalized) * speed * Time.deltaTime;
    }

    private static bool IsPointerOverUI()
    {
        return EventSystem.current != null && EventSystem.current.IsPointerOverGameObject();
    }

    private void SuppressShipInputProviders()
    {
        if (!suppressShipInputProviders)
        {
            return;
        }

        ShipInputProvider[] providers = FindObjectsByType<ShipInputProvider>(
            FindObjectsInactive.Exclude,
            FindObjectsSortMode.None);

        foreach (ShipInputProvider provider in providers)
        {
            if (provider == null || !provider.enabled || suppressedShipInputProviders.Contains(provider))
            {
                continue;
            }

            suppressedShipInputProviders.Add(provider);
            provider.enabled = false;
        }
    }

    private void RestoreShipInputProviders()
    {
        foreach (ShipInputProvider provider in suppressedShipInputProviders)
        {
            if (provider != null)
            {
                provider.enabled = true;
            }
        }

        suppressedShipInputProviders.Clear();
    }

    private void OnGUI()
    {
        if (!showDebugOverlay)
        {
            return;
        }

        const float width = 480f;
        const float height = 118f;
        GUILayout.BeginArea(new Rect(16f, 16f, width, height), GUI.skin.box);
        GUILayout.Label("FreeCamera");
        GUILayout.Label("Move: W/S forward/back, A/D left/right");
        GUILayout.Label("Up/Down: Space or E / Ctrl or Q");
        GUILayout.Label("Look: hold Left Mouse Button and drag");
        GUILayout.Label("Speed: hold Shift");
        GUILayout.EndArea();
    }

    private static float NormalizeAngle(float angle)
    {
        return angle > 180f ? angle - 360f : angle;
    }
}
