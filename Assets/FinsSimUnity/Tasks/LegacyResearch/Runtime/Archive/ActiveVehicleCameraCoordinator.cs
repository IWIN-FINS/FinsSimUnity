#if false
using System.Collections.Generic;
using NWH.Common.Cameras;
using NWH.Common.SceneManagement;
using NWH.Common.Vehicles;
using UnityEngine;

[DefaultExecutionOrder(600)]
public sealed class ActiveVehicleCameraCoordinator : MonoBehaviour
{
    private static ActiveVehicleCameraCoordinator instance;

    private VehicleChanger vehicleChanger;
    private int lastActiveVehicleIndex = -1;
    private int nextRefreshFrame;

    [RuntimeInitializeOnLoadMethod(RuntimeInitializeLoadType.BeforeSceneLoad)]
    private static void Initialize()
    {
        if (instance != null)
        {
            return;
        }

        var coordinatorObject = new GameObject(nameof(ActiveVehicleCameraCoordinator));
        DontDestroyOnLoad(coordinatorObject);
        instance = coordinatorObject.AddComponent<ActiveVehicleCameraCoordinator>();
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
    }

    private void LateUpdate()
    {
        if (vehicleChanger == null)
        {
            vehicleChanger = FindObjectOfType<VehicleChanger>();
            if (vehicleChanger == null)
            {
                return;
            }
        }

        bool activeVehicleChanged = lastActiveVehicleIndex != vehicleChanger.activeVehicleIndex;
        bool periodicRefreshDue = Time.frameCount >= nextRefreshFrame;
        if (!activeVehicleChanged && !periodicRefreshDue)
        {
            return;
        }

        nextRefreshFrame = Time.frameCount + 10;
        lastActiveVehicleIndex = vehicleChanger.activeVehicleIndex;
        SyncActiveVehicleCameras();
    }

    private void SyncActiveVehicleCameras()
    {
        if (vehicleChanger.cameraChangers != null && vehicleChanger.cameraChangers.Count > 0)
        {
            SyncCameraChangerTargets();
            return;
        }

        if (vehicleChanger.vehicles == null || vehicleChanger.vehicles.Count == 0)
        {
            return;
        }

        int activeIndex = Mathf.Clamp(vehicleChanger.activeVehicleIndex, 0, vehicleChanger.vehicles.Count - 1);
        for (int i = 0; i < vehicleChanger.vehicles.Count; i++)
        {
            Vehicle vehicle = vehicleChanger.vehicles[i];
            if (vehicle == null)
            {
                continue;
            }

            bool isActiveVehicle = i == activeIndex;
            CameraChanger[] cameraChangers = vehicle.GetComponentsInChildren<CameraChanger>(true);
            if (cameraChangers.Length == 0)
            {
                SetLooseVehicleCameras(vehicle, isActiveVehicle);
                continue;
            }

            foreach (CameraChanger cameraChanger in cameraChangers)
            {
                if (cameraChanger == null)
                {
                    continue;
                }

                List<GameObject> cameras = CollectCameras(cameraChanger);
                if (cameras.Count == 0)
                {
                    cameraChanger.enabled = false;
                    continue;
                }

                if (cameraChanger.currentCameraIndex < 0 || cameraChanger.currentCameraIndex >= cameras.Count)
                {
                    cameraChanger.currentCameraIndex = 0;
                }

                for (int cameraIndex = 0; cameraIndex < cameras.Count; cameraIndex++)
                {
                    GameObject cameraObject = cameras[cameraIndex];
                    if (cameraObject == null)
                    {
                        continue;
                    }

                    bool shouldBeActive = isActiveVehicle && cameraIndex == cameraChanger.currentCameraIndex;
                    if (cameraObject.activeSelf != shouldBeActive)
                    {
                        cameraObject.SetActive(shouldBeActive);
                    }

                    AudioListener listener = cameraObject.GetComponent<AudioListener>();
                    if (listener != null)
                    {
                        listener.enabled = shouldBeActive;
                    }
                }

                cameraChanger.enabled = isActiveVehicle;
            }
        }
    }

    private void SyncCameraChangerTargets()
    {
        int activeIndex = Mathf.Clamp(vehicleChanger.activeVehicleIndex, 0, vehicleChanger.cameraChangers.Count - 1);
        for (int i = 0; i < vehicleChanger.cameraChangers.Count; i++)
        {
            CameraChanger cameraChanger = vehicleChanger.cameraChangers[i];
            if (cameraChanger == null)
            {
                continue;
            }

            bool isActiveTarget = i == activeIndex;
            List<GameObject> cameras = CollectCameras(cameraChanger);
            if (cameras.Count == 0)
            {
                cameraChanger.enabled = false;
                continue;
            }

            if (cameraChanger.currentCameraIndex < 0 || cameraChanger.currentCameraIndex >= cameras.Count)
            {
                cameraChanger.currentCameraIndex = 0;
            }

            for (int cameraIndex = 0; cameraIndex < cameras.Count; cameraIndex++)
            {
                GameObject cameraObject = cameras[cameraIndex];
                if (cameraObject == null)
                {
                    continue;
                }

                bool shouldBeActive = isActiveTarget && cameraIndex == cameraChanger.currentCameraIndex;
                if (cameraObject.activeSelf != shouldBeActive)
                {
                    cameraObject.SetActive(shouldBeActive);
                }

                AudioListener listener = cameraObject.GetComponent<AudioListener>();
                if (listener != null)
                {
                    listener.enabled = shouldBeActive;
                }
            }

            cameraChanger.enabled = isActiveTarget;
        }
    }

    private static List<GameObject> CollectCameras(CameraChanger cameraChanger)
    {
        List<GameObject> cameras = new List<GameObject>();
        Camera[] childCameras = cameraChanger.GetComponentsInChildren<Camera>(true);
        foreach (Camera childCamera in childCameras)
        {
            if (childCamera != null)
            {
                cameras.Add(childCamera.gameObject);
            }
        }

        cameraChanger.cameras = cameras;
        return cameras;
    }

    private static void SetLooseVehicleCameras(Vehicle vehicle, bool isActiveVehicle)
    {
        Camera[] cameras = vehicle.GetComponentsInChildren<Camera>(true);
        for (int i = 0; i < cameras.Length; i++)
        {
            Camera cameraComponent = cameras[i];
            if (cameraComponent == null)
            {
                continue;
            }

            bool shouldBeActive = isActiveVehicle && i == 0;
            GameObject cameraObject = cameraComponent.gameObject;
            if (cameraObject.activeSelf != shouldBeActive)
            {
                cameraObject.SetActive(shouldBeActive);
            }

            AudioListener listener = cameraObject.GetComponent<AudioListener>();
            if (listener != null)
            {
                listener.enabled = shouldBeActive;
            }
        }
    }
}
#endif
