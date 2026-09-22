using FinsSim.Hydrodynamics;

public sealed class FinsROVFossen6DofMotionBenchmark : FinsROVHydrodynamicMotionBenchmark
{
    protected override HydrodynamicsMode[] DefaultBackendModes => new[]
    {
        HydrodynamicsMode.Fossen6Dof,
    };
}
