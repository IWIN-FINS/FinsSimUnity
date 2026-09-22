# Fossen Learn-to-Swim Domain Randomization

## Purpose

This configuration randomizes the calibrated FinsROV Fossen model per RL
episode while preserving nominal neutral buoyancy. It is intended for the
30-second parallel hold-position environment, not for DWP2 training scenes.

## Assets And Scene

| Item | Path | Role |
| --- | --- | --- |
| Nominal calibrated profile | `Assets/Models/FinsROV/Hydrodynamics/FinsROV_HydrodynamicsProfile.asset` | Kept unchanged as the calibration source. |
| DR nominal clone | `Assets/Models/FinsROV/Hydrodynamics/FinsROV_HydrodynamicsProfile_LearnToSwimDR.asset` | Baseline cloned by `HydrodynamicsController` before each episode. |
| DR ranges | `Assets/Models/FinsROV/Hydrodynamics/DomainRandomizationProfile_Fossen_LearnToSwim.asset` | Learn-to-Swim domain-randomization ranges. |
| Training scene | `Assets/FinsSimUnity/Tasks/HoldPosition/Scenes/HoldForPosition_Fossen_Parallel_30s.unity` | References both DR assets. |

`FinsROV_Fossen.prefab` contains `HydrodynamicsController`,
`ThrusterRandomizationTarget`, and `RigidbodyRandomizationTarget`. The
scene-level `DomainRandomizationCoordinator` discovers and randomizes all
three at every episode reset.

## Neutral Buoyancy Coupling

The calibrated baseline is:

```text
mass             = 12.11 kg
water density    = 1027 kg/m^3
displaced volume = 0.0118 m^3
```

The profile enables `coupleMassAndVolumeForNeutralBuoyancy`. For each episode:

```text
s          ~ Uniform(0.98, 1.02)
mass       = mass_0 * s
volume     = volume_0 * s * trim
inertia    = inertia_0 * s * inertia_residual
trim       = 1.0
inertia_residual ~ Uniform(0.95, 1.05)
```

Consequently, `rho * volume / mass` remains at its calibrated value. The
policy does not receive an artificial persistent ascent or descent target.
`volumeScale` remains in the profile only for legacy uncoupled profiles; it is
ignored while the coupling option is enabled.

`RigidbodyRandomizationTarget` also synchronizes NWH
`VariableCenterOfMass` after applying mass and inertia. This prevents the NWH
component from restoring the nominal inertia on its next physics update.

## Enabled Randomization

### Hydrodynamics

The runtime profile is reset from the DR nominal clone, then applies a global
scale and a per-axis scale in Fossen order `[u, v, w, p, q, r]`:

```text
added mass:        global 0.90-1.10, per axis 0.90-1.10
linear damping:    global 0.85-1.15, yaw per-axis 0.75-1.35
quadratic damping: global 0.80-1.25, yaw per-axis 0.70-1.50
```

Yaw is intentionally broader because it remains the most uncertain calibrated
degree of freedom. Other axes have narrower residual ranges to retain the
measured surge, sway, heave, roll, and pitch behavior.

### Thrusters

Every episode samples a vehicle-wide propulsion scale and an independent scale
for each of the eight thrusters:

```text
common max-force scale:     0.95-1.05
per-thruster max-force:     0.95-1.05
effective max-force scale:  common * per-thruster
time constant:              0.70-1.30
command delay:              0.50-1.50
force slew rate:            0.80-1.20
```

The effective thrust range is approximately `0.9025-1.1025` of the calibrated
value. The common term represents supply voltage, water density, or global
thrust-curve error. The individual term creates left/right and
front/rear asymmetry. The FinsROV thrusters currently use `NormalizedForce`,
so `forceConstantScale` is deliberately fixed to one; it does not control the
active force path.

## Water, Waves, And Curriculum

The first training stage keeps `randomizeWater` and `randomizeWaves` disabled.
This isolates vehicle and actuator uncertainty while preserving a stable,
calibrated neutral-buoyancy task.

For the second stage, enable one effect at a time in
`DomainRandomizationProfile_Fossen_LearnToSwim.asset`:

```text
steady current:
  randomizeWater = true
  currentSpeed = 0.00-0.15 m/s
  allowVerticalCurrent = false
  maxAngleFromMeanDegrees = 180

waves:
  randomizeWaves = true
  waveComponentCount = 0-2
  waveAmplitude = 0.00-0.03 m
  waveWavelength = 3-8 m
  wavePeriod = 2.5-5 s
```

`PhysicalWaveWaterDataProvider` receives these episode values through the
same coordinator. To make its current and wave flow affect Fossen relative
velocity, its mode must be `AnalyticPhysicalWave`; `FlatFallback` provides a
flat height but does not expose the randomized flow through `Sample()`. Do not
enable vertical current or wave flow until the no-current policy converges.

## Exclusions

- Do not add `Dwp2MassBuoyancyRandomizer` to `FinsROV_Fossen`.
- Do not use DWP2 `buoyantForceCoefficient` for this Fossen profile.
- Do not modify the nominal calibrated profile during training; all coefficient
  changes are applied to a runtime clone.
- Do not enable a non-unit buoyancy trim until the policy is stable under the
  coupled mass-volume model.

## Verification

Build the configured scene with:

```bash
cd "$(git rev-parse --show-toplevel)"
./tools/unity/build_in_worktree.sh \
  --execute-method FinsSimLinuxBuild.BuildHoldForPositionFossenParallel30sServer \
  --log-file /tmp/hold-fossen-learn-to-swim-dr-build.log \
  --verbose
```

At runtime, inspect the FinsROV root object. It must contain one each of
`HydrodynamicsController`, `RigidbodyRandomizationTarget`, and
`ThrusterRandomizationTarget`; the area-local
`DomainRandomizationCoordinator` must be in `Train` mode and reference
`DomainRandomizationProfile_Fossen_LearnToSwim`.
