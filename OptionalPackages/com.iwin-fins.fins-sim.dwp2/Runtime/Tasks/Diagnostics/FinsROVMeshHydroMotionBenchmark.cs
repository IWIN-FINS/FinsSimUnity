using FinsSim.Hydrodynamics;

public sealed class FinsROVMeshHydroMotionBenchmark : FinsROVHydrodynamicMotionBenchmark
{
    protected override HydrodynamicsMode[] DefaultBackendModes => new[]
    {
        HydrodynamicsMode.SurfaceGeometryHydro,
    };
}
