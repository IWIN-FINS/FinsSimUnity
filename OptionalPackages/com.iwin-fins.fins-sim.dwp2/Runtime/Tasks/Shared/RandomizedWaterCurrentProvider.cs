using FinsSim.Hydrodynamics;
using NWH.DWP2.WaterData;
using NWH.DWP2.WaterObjects;
using UnityEngine;

public class RandomizedWaterCurrentProvider : WaterDataProvider, IWaterKinematicsProvider, IEpisodeRandomizable
{
    [Header("Water Surface")]
    [Tooltip("Flat water surface height in world space.")]
    public float waterHeight = 0f;

    [Header("Episode Current Randomization")]
    [Tooltip("Minimum current speed sampled at the start of an episode.")]
    public float minCurrentSpeed = 0f;

    [Tooltip("Maximum current speed sampled at the start of an episode.")]
    public float maxCurrentSpeed = 0.2f;

    [Tooltip("Allow the sampled current to include a vertical component.")]
    public bool randomizeVerticalCurrent = false;

    [Tooltip("Sample one current when Play mode starts. Episode resets can sample again.")]
    public bool randomizeOnStart = true;

    [Header("Debug")]
    [SerializeField]
    private Vector3 currentFlow;

    public Vector3 CurrentFlow => currentFlow;

    private void Start()
    {
        if (randomizeOnStart)
        {
            RandomizeForEpisode();
        }
    }

    public void RandomizeForEpisode()
    {
        float minSpeed = Mathf.Max(0f, minCurrentSpeed);
        float maxSpeed = Mathf.Max(minSpeed, maxCurrentSpeed);
        float speed = Random.Range(minSpeed, maxSpeed);
        currentFlow = SampleDirection() * speed;
    }

    public void RandomizeForEpisode(RandomizationContext context)
    {
        if (context.Profile != null && context.Profile.randomizeWater)
        {
            minCurrentSpeed = context.Profile.currentSpeed.min;
            maxCurrentSpeed = context.Profile.currentSpeed.max;
            randomizeVerticalCurrent = context.Profile.allowVerticalCurrent;
            currentFlow = context.CurrentDirection() * Mathf.Max(0f, context.Range(context.Profile.currentSpeed));
            return;
        }

        RandomizeForEpisode();
    }

    public WaterKinematicsSample Sample(Vector3 worldPoint)
    {
        return new WaterKinematicsSample
        {
            IsValid = true,
            Height = waterHeight,
            Normal = Vector3.up,
            FlowVelocity = currentFlow,
            AngularFlowVelocity = Vector3.zero,
        };
    }

    public override bool SupportsWaterHeightQueries()
    {
        return true;
    }

    public override bool SupportsWaterNormalQueries()
    {
        return false;
    }

    public override bool SupportsWaterFlowQueries()
    {
        return true;
    }

    public override void GetWaterHeights(WaterObject waterObject, ref Vector3[] points, ref float[] waterHeights)
    {
        for (int i = 0; i < waterHeights.Length; i++)
        {
            waterHeights[i] = waterHeight;
        }
    }

    public override void GetWaterFlows(WaterObject waterObject, ref Vector3[] points, ref Vector3[] waterFlows)
    {
        for (int i = 0; i < waterFlows.Length; i++)
        {
            waterFlows[i] = currentFlow;
        }
    }

    private Vector3 SampleDirection()
    {
        if (randomizeVerticalCurrent)
        {
            return Random.onUnitSphere;
        }

        Vector2 horizontal = Random.insideUnitCircle;
        if (horizontal.sqrMagnitude < 0.0001f)
        {
            return Vector3.forward;
        }

        horizontal.Normalize();
        return new Vector3(horizontal.x, 0f, horizontal.y);
    }
}
