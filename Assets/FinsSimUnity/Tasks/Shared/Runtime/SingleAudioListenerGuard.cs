using UnityEngine;
using UnityEngine.SceneManagement;

public sealed class SingleAudioListenerGuard : MonoBehaviour
{
    private static SingleAudioListenerGuard instance;
    private int nextCheckFrame;

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
    private static void Initialize()
    {
        if (instance != null)
        {
            return;
        }

        var guardObject = new GameObject(nameof(SingleAudioListenerGuard));
        DontDestroyOnLoad(guardObject);
        instance = guardObject.AddComponent<SingleAudioListenerGuard>();
    }

    private void Awake()
    {
        if (instance != null && instance != this)
        {
            Destroy(gameObject);
            return;
        }

        instance = this;
        DontDestroyOnLoad(gameObject);
        EnforceSingleListener();
    }

    private void OnEnable()
    {
        SceneManager.sceneLoaded += OnSceneLoaded;
    }

    private void OnDisable()
    {
        SceneManager.sceneLoaded -= OnSceneLoaded;
    }

    private void LateUpdate()
    {
        if (Time.frameCount < nextCheckFrame)
        {
            return;
        }

        nextCheckFrame = Time.frameCount + 15;
        EnforceSingleListener();
    }

    private void OnSceneLoaded(Scene scene, LoadSceneMode mode)
    {
        EnforceSingleListener();
    }

    private static void EnforceSingleListener()
    {
        AudioListener primaryListener = null;
        var mainCamera = Camera.main;
        if (mainCamera != null)
        {
            primaryListener = mainCamera.GetComponent<AudioListener>();
            if (primaryListener != null && !primaryListener.isActiveAndEnabled)
            {
                primaryListener = null;
            }
        }

        foreach (var listener in FindObjectsOfType<AudioListener>())
        {
            if (listener == null || !listener.isActiveAndEnabled)
            {
                continue;
            }

            if (primaryListener == null)
            {
                primaryListener = listener;
                continue;
            }

            if (listener != primaryListener)
            {
                listener.enabled = false;
            }
        }
    }
}
