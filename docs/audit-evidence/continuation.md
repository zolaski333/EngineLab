# Continuation evidence

Evidence for [the audit register](../audit.md). File review, controlled execution, instrumented bounds checks and whole-suite results are distinct. No product fix or default-sound adjustment was made.

## Windows build and registered tests

Reference hardware observed: Intel i5-10600, 6 cores / 12 logical processors, 16 GB RAM; Windows 10 x64 10.0.19045. Build used MSVC14.44 x64 via `scripts/vsenv.ps1` and the authoritative VS2022 Release tree.

```powershell
. scripts/vsenv.ps1
cmake --build out/build/windows-vs2022 --config Release -- /m:2 /nologo /verbosity:minimal
ctest --test-dir out/build/windows-vs2022 -C Release -j 4 --output-on-failure --output-junit out/audit2/full-ctest.xml
ctest --test-dir out/build/windows-vs2022 -C Release -j 1 -R "^EngineLab\.(AudioRender|LoadedAccelerationCatalogDfcoResume)$" --output-on-failure --output-junit out/audit2/isolated-reruns.xml
```

Full build exit0. Initial108-test run:106PASS, AudioRender timeout180.02s, LoadedAccelerationCatalogDfcoResume timeout240.01s; total2084.44s. Those two reran sequentially alone and passed in159.74s and166.15s. Thus every registered test passed at least once; the first parallel invocation itself did not pass. No harness/compilation was run concurrently with either baseline or isolated reruns.

Passing the registered suite does not establish the stronger claims invalidated by the oracle findings. DesignedReference remains report-only and contains two FAIL result rows despite exit0.

| Test | Initial result | Initial seconds | Isolated retry |
|---|---|---:|---|
| EngineLab.Calibration | PASS | 0.116 | — |
| EngineLab.Exhaust | PASS | 0.147 | — |
| EngineLab.ExhaustPathTopologyEditor | PASS | 0.140 | — |
| EngineLab.ExhaustSimulation | PASS | 52.861 | — |
| EngineLab.GasDynamics | PASS | 1.351 | — |
| EngineLab.RenderSnapshot | PASS | 0.277 | — |
| EngineLab.Scripting | PASS | 0.281 | — |
| EngineLab.Core | PASS | 117.352 | — |
| EngineLab.AudioVoicing | PASS | 0.178 | — |
| EngineLab.PorousLinerLoss | PASS | 0.062 | — |
| EngineLab.AutoGainSoftClipper | PASS | 0.132 | — |
| EngineLab.ValveTurbulenceNoise | PASS | 0.232 | — |
| EngineLab.CombustionCycleVariation | PASS | 0.117 | — |
| EngineLab.VehicleDynamics | PASS | 0.175 | — |
| EngineLab.DynoEstimator | PASS | 0.080 | — |
| EngineLab.DynoQualityGate | PASS | 0.086 | — |
| EngineLab.DynoRunArchive | PASS | 0.080 | — |
| EngineLab.AudioUpsampling | PASS | 0.990 | — |
| EngineLab.ThreadScheduling | PASS | 0.039 | — |
| EngineLab.CylinderWorkerPool | PASS | 0.560 | — |
| EngineLab.StructuralNvh | PASS | 0.132 | — |
| EngineLab.PhysicsRegression | PASS | 0.116 | — |
| EngineLab.CombustionPhasing | PASS | 17.090 | — |
| EngineLab.IntakeTuning | PASS | 369.904 | — |
| EngineLab.IntakeAcousticConvergence | PASS | 0.305 | — |
| EngineLab.IntakeRunnerCourant | PASS | 0.391 | — |
| EngineLab.ExhaustJunctionStep | PASS | 0.090 | — |
| EngineLab.IntakePlenumPassivity | PASS | 2.965 | — |
| EngineLab.IntakeBoundaryAdmittance | PASS | 0.017 | — |
| EngineLab.IntakeOutletPredictionEquilibrium | PASS | 0.015 | — |
| EngineLab.IntakeCouplingContinuity | PASS | 0.542 | — |
| EngineLab.IntakeSplitReservoirContinuity | PASS | 1.772 | — |
| EngineLab.IntakeMovingBoundary | PASS | 1.301 | — |
| EngineLab.PartitionedExhaustNetwork | PASS | 0.183 | — |
| EngineLab.RestrictionFace | PASS | 0.522 | — |
| EngineLab.JointIntakeManifold | PASS | 36.075 | — |
| EngineLab.AngularStepResolution | PASS | 56.577 | — |
| EngineLab.BrakeCycleNumericalQuality | PASS | 1.208 | — |
| EngineLab.GasExchange | PASS | 31.486 | — |
| EngineLab.OverrunThermalRegression | PASS | 37.059 | — |
| EngineLab.PartLoadFuelling | PASS | 65.596 | — |
| EngineLab.IdleStabilityRegression | PASS | 908.941 | — |
| EngineLab.ShiftStyle | PASS | 135.891 | — |
| EngineLab.DriverCommand | PASS | 0.204 | — |
| EngineLab.ActiveLimits | PASS | 36.409 | — |
| EngineLab.TurboFit | PASS | 8.174 | — |
| EngineLab.EngineBrake | PASS | 36.141 | — |
| EngineLab.TruckDrive | PASS | 111.320 | — |
| EngineLab.DynoHold | PASS | 37.260 | — |
| EngineLab.EcuCalibration | PASS | 0.108 | — |
| EngineLab.SavedEngine | PASS | 267.755 | — |
| EngineLab.RealtimeRegression | PASS | 4.552 | — |
| EngineLab.ExhaustTransfer | PASS | 1.487 | — |
| EngineLab.LiveChange | PASS | 211.474 | — |
| EngineLab.LiveCylinderResize | PASS | 336.483 | — |
| EngineLab.LiveSettings | PASS | 137.974 | — |
| EngineLab.LiveIntake | PASS | 222.167 | — |
| EngineLab.LiveTuning | PASS | 350.605 | — |
| EngineLab.CameraMicrophone | PASS | 77.980 | — |
| EngineLab.OfflineAudioExport | PASS | 13.441 | — |
| EngineLab.LoopPackExport | PASS | 32.044 | — |
| EngineLab.ExhaustValve | PASS | 24.433 | — |
| EngineLab.AntiLag | PASS | 103.602 | — |
| EngineLab.Nitrous | PASS | 68.969 | — |
| EngineLab.PassByMicrophone | PASS | 5.561 | — |
| EngineLab.AudioWorkshop | PASS | 2.091 | — |
| EngineLab.CatalogPhysics | PASS | 300.039 | — |
| EngineLab.TurboDownstreamAuthority | PASS | 234.652 | — |
| EngineLab.DynoSchema | PASS | 0.057 | — |
| EngineLab.CatalogReference | PASS | 131.586 | — |
| EngineLab.CatalogBoostHold | PASS | 68.689 | — |
| EngineLab.DesignedEnginesBuild | PASS | 513.524 | — |
| EngineLab.DesignedReference | PASS | 169.767 | — |
| EngineLab.AudioRender | TIMEOUT | 180.020 | PASS 159.739s |
| EngineLab.AudioRuntimeDiscontinuity | PASS | 6.518 | — |
| EngineLab.AudioTransients | PASS | 30.444 | — |
| EngineLab.AudioShiftTransientNA | PASS | 32.493 | — |
| EngineLab.AudioShiftTransientBoosted | PASS | 32.311 | — |
| EngineLab.LoadedAccelerationCP2 | PASS | 5.482 | — |
| EngineLab.LoadedAccelerationAudi | PASS | 15.506 | — |
| EngineLab.LoadedAccelerationCatalogDfcoResume | TIMEOUT | 240.012 | PASS 166.154s |
| EngineLab.StandingStartCatalogue | PASS | 57.420 | — |
| EngineLab.StandingStartSettledIdle | PASS | 218.431 | — |
| EngineLab.UserDynoHighRpmStart | PASS | 1.101 | — |
| EngineLab.UserDynoRampCP2Complete | PASS | 7.565 | — |
| EngineLab.UserDynoRampLS3Complete | PASS | 7.767 | — |
| EngineLab.DieselControl | PASS | 30.194 | — |
| EngineLab.EngineModel3D | PASS | 55.751 | — |
| EngineLab.SourceEncoding | PASS | 0.334 | — |
| EngineLab.ValveProfileTransition | PASS | 23.991 | — |
| EngineLab.Units | PASS | 0.108 | — |
| EngineLab.CruiseControl | PASS | 0.077 | — |
| EngineLab.PassByRun | PASS | 67.085 | — |
| EngineLab.CruiseControlHold | PASS | 75.606 | — |
| EngineLab.ShiftSchedule | PASS | 165.886 | — |
| EngineLab.Cvt | PASS | 0.090 | — |
| EngineLab.KnockControl | PASS | 25.856 | — |
| EngineLab.PeakPressure | PASS | 65.893 | — |
| EngineLab.Turbocharger | PASS | 152.065 | — |
| EngineLab.LaunchControl | PASS | 0.093 | — |
| EngineLab.OverboostCut | PASS | 0.194 | — |
| EngineLab.DataLogger | PASS | 0.087 | — |
| EngineLab.CalibrationEditing | PASS | 0.081 | — |
| EngineLab.TwoStroke | PASS | 86.549 | — |
| EngineLab.CatalogRoundTrip | PASS | 0.842 | — |
| EngineLab.EngineDesign | PASS | 67.938 | — |
| EngineLab.Rotary | PASS | 35.875 | — |
| EngineLab.Vtec | PASS | 148.207 | — |

## CatalogReference.csv

Successful JUnit output is truncated to1KiB; these full tables were extracted from the preserved full-run LastTest.log.

```csv
engine,metric,target_rpm,actual_rpm,raw_rpm_span_percent,held_rpm_span_percent,held_rpm_drift_percent,held_rpm_stddev,measured,reference,error_percent,tolerance_percent,hold_result,source,result
"K20A-like 2.0 I4 VTEC",torque_nm,7000.000,6996.518,0.648,0.023,0.009,0.424,190.865,206.000,-7.347,15.000,PASS,"Honda K20A rated specification",PASS
"K20A-like 2.0 I4 VTEC",power_kw,8000.000,8000.494,0.782,0.064,0.028,1.340,146.172,162.000,-9.770,15.000,PASS,"Honda K20A rated specification",PASS
"2JZ-GTE-like 3.0 I6 Turbo",torque_nm,4000.000,3993.853,0.297,0.044,0.022,0.514,380.667,427.000,-10.851,15.000,PASS,"Toyota 2JZ-GTE export rated specification",PASS
"2JZ-GTE-like 3.0 I6 Turbo",power_kw,5600.000,5599.016,0.234,0.143,0.071,2.290,220.625,239.000,-7.688,15.000,PASS,"Toyota 2JZ-GTE export rated specification",PASS
"LS3-like 6.2 Crossplane V8",torque_nm,4600.000,4599.475,0.197,0.006,0.002,0.068,585.818,575.000,1.881,15.000,PASS,"GM LS3 rated specification",PASS
"LS3-like 6.2 Crossplane V8",power_kw,5900.000,5900.808,0.174,0.087,0.041,1.450,330.681,321.000,3.016,15.000,PASS,"GM LS3 rated specification",PASS
"EJ25-like 2.5 Flat-4 Turbo",torque_nm,4000.000,3998.900,0.793,0.018,0.004,0.155,361.223,393.000,-8.086,15.000,PASS,"Subaru 2021 WRX STI EJ257 rated specification",PASS
"EJ25-like 2.5 Flat-4 Turbo",power_kw,6000.000,5999.667,0.297,0.183,0.087,3.071,198.409,231.000,-14.109,15.000,PASS,"Subaru 2021 WRX STI EJ257 rated specification",PASS
"Audi I5-like 2.5 Turbo",torque_nm,4000.000,3992.773,1.276,0.097,0.025,0.995,468.113,500.000,-6.377,15.000,PASS,"Audi RS 3 2.5 TFSI rated specification",PASS
"Audi I5-like 2.5 Turbo",power_kw,5600.000,5599.041,0.649,0.145,0.071,2.344,256.376,294.000,-12.797,15.000,PASS,"Audi RS 3 2.5 TFSI rated specification",PASS
"Hayabusa-like 1.3 I4",torque_nm,7000.000,6999.619,0.529,0.012,0.000,0.220,140.914,138.200,1.964,15.000,PASS,"Suzuki 1999 GSX1300R rated specification",PASS
"Hayabusa-like 1.3 I4",power_kw,9800.000,9800.764,0.942,0.016,0.004,0.322,132.192,128.700,2.713,15.000,PASS,"Suzuki 1999 GSX1300R rated specification",PASS
"Big Twin-like 1.9 V2",torque_nm,2500.000,2502.651,7.862,0.303,-0.017,2.004,177.091,162.700,8.845,15.000,PASS,"Harley-Davidson Milwaukee-Eight 117 Classic J1349 specification",PASS
"Big Twin-like 1.9 V2",power_kw,4600.000,4600.060,2.658,0.267,0.108,3.004,70.529,73.000,-3.385,15.000,PASS,"Harley-Davidson Milwaukee-Eight 117 Classic J1349 specification",PASS
"Aircooled-like 3.6 Flat-6",torque_nm,4800.000,4801.253,0.193,0.039,-0.004,0.515,342.345,310.000,10.434,15.000,PASS,"Porsche 911 964 Carrera 2 rated specification",PASS
"Aircooled-like 3.6 Flat-6",power_kw,6100.000,6100.164,0.306,0.098,0.046,1.691,204.817,184.000,11.314,15.000,PASS,"Porsche 911 964 Carrera 2 rated specification",PASS
"Yamaha CP2 MT-07-like 689 Twin",torque_nm,6500.000,6500.497,2.362,0.036,-0.001,0.669,67.434,68.000,-0.832,15.000,PASS,"Yamaha MT-07 EU rated specification",PASS
"Yamaha CP2 MT-07-like 689 Twin",power_kw,8750.000,8750.104,1.284,0.094,0.033,1.788,52.181,54.000,-3.368,15.000,PASS,"Yamaha MT-07 EU rated specification",PASS
"Yamaha CP3 MT-09-like 890 Triple",torque_nm,7000.000,6999.957,1.331,0.010,0.000,0.216,95.733,93.000,2.938,15.000,PASS,"Yamaha MT-09 890 rated specification",PASS
"Yamaha CP3 MT-09-like 890 Triple",power_kw,10000.000,9999.927,0.666,0.022,0.008,0.542,88.010,87.500,0.583,15.000,PASS,"Yamaha MT-09 890 rated specification",PASS
"Yamaha CP4 MT-10-like 998 Crossplane I4",torque_nm,9000.000,8997.833,1.067,0.065,0.006,1.359,109.770,111.000,-1.108,15.000,PASS,"Yamaha MT-10 rated specification",PASS
"Yamaha CP4 MT-10-like 998 Crossplane I4",power_kw,11500.000,11500.779,0.633,0.031,0.007,0.776,120.691,118.000,2.280,15.000,PASS,"Yamaha MT-10 rated specification",PASS
"VW EA288-like 2.0 TDI I4",torque_nm,2000.000,1991.659,5.368,0.154,0.024,0.747,289.574,320.000,-9.508,15.000,PASS,"Volkswagen Golf EA288 2.0 TDI 110 kW rated specification",PASS
"VW EA288-like 2.0 TDI I4",power_kw,4000.000,3997.342,0.688,0.259,0.129,3.004,96.572,110.000,-12.207,15.000,PASS,"Volkswagen EA288 110 kW rated specification",PASS
```

## DesignedReference.csv

Successful JUnit output is truncated to1KiB; these full tables were extracted from the preserved full-run LastTest.log.

```csv
engine,metric,target_rpm,actual_rpm,raw_rpm_span_percent,held_rpm_span_percent,held_rpm_drift_percent,held_rpm_stddev,measured,reference,error_percent,tolerance_percent,hold_result,source,result
"D-CBR1000RR-R",torque_nm,12500.000,12497.726,0.419,0.017,0.003,0.515,98.307,113.000,-13.003,15.000,PASS,"Honda CBR1000RR-R Fireblade SP 2020 press kit",PASS
"D-CBR1000RR-R",power_kw,14500.000,14500.474,0.440,0.010,0.002,0.298,129.800,160.000,-18.875,15.000,PASS,"Honda CBR1000RR-R Fireblade SP 2020 press kit",FAIL
"D-CP4 MT-10",torque_nm,9000.000,8999.303,0.206,0.014,0.002,0.350,108.376,111.000,-2.364,15.000,PASS,"Yamaha MT-10 rated specification",PASS
"D-CP4 MT-10",power_kw,11500.000,11500.558,0.667,0.070,-0.008,1.463,122.582,118.000,3.883,15.000,PASS,"Yamaha MT-10 rated specification",PASS
"D-Hayabusa 1999",torque_nm,7000.000,6999.349,0.259,0.006,0.002,0.099,139.397,138.200,0.866,15.000,PASS,"Suzuki 1999 GSX1300R rated specification",PASS
"D-Hayabusa 1999",power_kw,9800.000,9800.646,0.465,0.026,0.004,0.441,130.897,128.700,1.707,15.000,PASS,"Suzuki 1999 GSX1300R rated specification",PASS
"D-CP2 MT-07",torque_nm,6500.000,6499.561,2.065,0.027,0.001,0.531,69.399,68.000,2.057,15.000,PASS,"Yamaha MT-07 EU rated specification",PASS
"D-CP2 MT-07",power_kw,8750.000,8750.501,1.291,0.063,0.024,1.356,52.986,54.000,-1.878,15.000,PASS,"Yamaha MT-07 EU rated specification",PASS
"D-CP3 MT-09",torque_nm,7000.000,7000.047,1.062,0.010,0.000,0.180,93.097,93.000,0.104,15.000,PASS,"Yamaha MT-09 890 rated specification",PASS
"D-CP3 MT-09",power_kw,10000.000,10000.768,0.513,0.055,0.026,1.621,79.196,87.500,-9.491,15.000,PASS,"Yamaha MT-09 890 rated specification",PASS
"D-K20A Euro-R",torque_nm,7000.000,6999.313,0.489,0.009,0.001,0.153,205.224,206.000,-0.377,15.000,PASS,"Honda K20A Accord Euro-R rated specification",PASS
"D-K20A Euro-R",power_kw,8000.000,8001.630,0.666,0.075,0.035,1.692,154.479,162.000,-4.643,15.000,PASS,"Honda K20A Accord Euro-R rated specification",PASS
"D-LS3",torque_nm,4600.000,4599.788,0.217,0.004,0.001,0.038,642.550,575.000,11.748,15.000,PASS,"GM LS3 rated specification",PASS
"D-LS3",power_kw,5900.000,5900.183,0.187,0.092,0.044,1.552,352.422,321.000,9.789,15.000,PASS,"GM LS3 rated specification",PASS
"D-M8 117",torque_nm,2500.000,2501.504,8.864,0.289,-0.010,2.201,193.265,162.700,18.786,15.000,PASS,"Harley-Davidson Milwaukee-Eight 117 J1349 specification",FAIL
"D-M8 117",power_kw,4600.000,4600.320,2.716,0.297,0.131,3.588,71.638,73.000,-1.866,15.000,PASS,"Harley-Davidson Milwaukee-Eight 117 J1349 specification",PASS
"D-LS7",torque_nm,4800.000,4799.626,0.159,0.007,0.002,0.076,716.098,637.000,12.417,15.000,PASS,"GM Powertrain LS7 Corvette Z06 SAE certified (control)",PASS
"D-LS7",power_kw,6300.000,6300.381,0.173,0.109,0.049,1.886,357.472,377.000,-5.180,15.000,PASS,"GM Powertrain LS7 Corvette Z06 SAE certified (control)",PASS
"D-Panigale 1299",torque_nm,8750.000,8747.298,2.376,0.065,-0.003,1.241,133.137,144.600,-7.928,15.000,PASS,"Ducati 1299 Panigale 2015 rated specification (control)",PASS
"D-Panigale 1299",power_kw,10500.000,10500.256,1.584,0.023,0.001,0.559,131.094,151.000,-13.183,15.000,PASS,"Ducati 1299 Panigale 2015 rated specification (control)",PASS
"D-SC59 CBR1000RR 2008",torque_nm,8500.000,8499.811,0.445,0.068,-0.002,1.713,106.511,112.000,-4.901,15.000,PASS,"Honda CBR1000RR 2008 rated specification (control)",PASS
"D-SC59 CBR1000RR 2008",power_kw,12000.000,12000.242,0.467,0.034,0.003,0.626,116.829,131.000,-10.818,15.000,PASS,"Honda CBR1000RR 2008 rated specification (control)",PASS
```

## Probe and tool output

These continuation probes use libraries/objects from the completed current-tree Windows Release build (the normal incremental command). Original physics/data/script/UI-audio probe executables were also recompiled and rerun after that build. Their results remain consistent. Fresh SceneExport confirms existing R01–R03. The hardened PartInspector diagnostic uses the unchanged owner source with an STL check in the scratch executable; exit86 is the probe handler, not a reported normal-app crash.

### physics/component_probe.log

```text
slug fresh_boundary_inflow_kg=1.3774354573929063e-07 returned_fuel_kg=1.5734445229799169e-09 extra_plenum_mass_kg=1.5734445230021141e-09 unchanged_oxygen=1
ring fine_delay_us=209.99999999999753 target_delay_us=625 max_error_us=415.00000000000261
ring between_knots_output_delta_us=0 input_delta_us=10
taper remap_before_mass_kg=0.0036623684972229772 after_mass_kg=0.0037112000771859508 relative_mass_change=0.013333333333333419 relative_energy_change=0.013333333333333419
executor completed=1 dispatches=3 maximum_dispatched_chunks=1 ducts=6
```
### physics/ecu_probe.log

```text
overboost_active=1 afterfire_active=1 afterfire_fuel=1 clean_fuel=0
```
### physics/persistence_probe.log

```text
four_metres_accepted=0 diagnostic=Final engine validation failed: Engine configuration contains non-finite or physically invalid values
hundred_millimetres_accepted=1 stored_metres=100
```
### physics/alias_probe.log

```text
plain_yaml_valid=1
entering_alias_conversion
child_exit_code=3221225725 hex=C00000FD
```
### audio/probe-output.txt

```text
crossover source_hz=40000 coupling_hz=5000 cutoff_hz=2350 history_seconds=0.0014 low_only_delay_seconds=0.000625 aligned_rms=0.692070051674 delayed_rms=0.212184695033 relative_db=-10.2687200265
valve open_b0=0.928171856503 initial_output=928.171875 closed_zero_input_1s_last=-71.8281402588 tail_energy=247645523.438
jet configured_mach=2 target_rms=10.304111142 measured_rms=202.919294521 excess_db=25.8862561868
paused_fi published_time_scale=0 shaft_rpm=12000 late_stem_rms=0.000506531096476
cancelled_export cancelled=1 partial_exists=1
cancelled_export_retry success=0 error=Stale partial export exists: C:/Users/t3anc/OneDrive/Bureau/EngineLab/out/audit2/audio/probe-run-20261116-1642\cancelled-export\engine-order-map.csv.partial
unicode_ir authored=1 loaded=0 warning=Impulse response not loaded (file not found): C:/Users/t3anc/OneDrive/Bureau/EngineLab/out/audit2/audio/probe-run-20261116-1642\ir-�\kernel.wav
datalog_rewind_times=0.000,1.000,1.020,0.000,0.020,
```
### audio/zero-match-cli.txt

```text
engine,metric,target_rpm,actual_rpm,raw_rpm_span_percent,held_rpm_span_percent,held_rpm_drift_percent,held_rpm_stddev,measured,reference,error_percent,tolerance_percent,hold_result,source,result
```
### audio/tool-text-evidence.json

```text
{
  "TL2-01": {
    "input_work_j": [
      0.0,
      100.0,
      0.0,
      100.0,
      0.0,
      100.0,
      0.0,
      100.0,
      0.0,
      100.0,
      0.0,
      100.0,
      0.0,
      100.0,
      0.0,
      100.0,
      0.0,
      100.0,
      0.0,
      100.0
    ],
    "retained_work_j": [
      100.0,
      100.0,
      100.0,
      100.0,
      100.0,
      100.0,
      100.0,
      100.0,
      100.0,
      100.0
    ],
    "reported_cov_from_retained": 0.0,
    "complete_sample_cov": 1.0259783520851542
  },
  "TL2-02": {
    "header_fields": 28,
    "row_format_fields": 33,
    "trailing_unnamed_fields": [
      "lambda",
      "airFuelRatioValid",
      "manifoldPressureKpa",
      "wastegateOpening",
      "injectorDutyCycle"
    ]
  },
  "TL2-04": {
    "shipped_bank_route_mm": 3000,
    "control_bank_route_mm": 1790,
    "deleted_pipe_length_per_bank_mm": 1210
  }
}
```
### audio/crossover-delay-math.json

```text
{
  "kind": "exact_coefficient_frequency_math_not_engine_measurement",
  "sample_rate_hz": 48000.0,
  "mechanical_source_rate_hz": 40000.0,
  "coupling_rate_hz": 5000.0,
  "crossover_hz": 2350.0,
  "history_span_seconds_for_eight_knots": 0.0014,
  "required_delay_seconds": 0.000625,
  "history_is_adequate": true,
  "low_magnitude_at_crossover": 0.5000000000000014,
  "high_magnitude_at_crossover": 0.5000000000000007,
  "ideal_lr_pair_sum_magnitude": 1.0000000000000022,
  "aligned_sum_with_same_upper_filter_magnitude": 0.9787348531899659,
  "production_delay_sum_with_same_upper_filter_magnitude": 0.3000744734435567,
  "relative_db_from_low_branch_delay_alone": -10.268720026495284
}
```
### app/model-output.log

```text
scalar validity=valid

editable validity=valid

scalar nodes=8 diagnostics=0

 route cylinder=1 length_mm=1727 delay_s=0.00334035169658

 route cylinder=2 length_mm=1350 delay_s=0.00261116085141

 route cylinder=3 length_mm=1350 delay_s=0.00261116085141

 route cylinder=4 length_mm=1350 delay_s=0.00261116085141

editable nodes=11 diagnostics=0

 route cylinder=1 length_mm=600 delay_s=0.00116051593396

 route cylinder=2 length_mm=600 delay_s=0.00116051593396

 route cylinder=3 length_mm=600 delay_s=0.00116051593396

 route cylinder=4 length_mm=600 delay_s=0.00116051593396

binding component=6 authored_mm=200 node=1 p=100777

binding component=1 authored_mm=400 node=1 p=100777

binding component=2 authored_mm=400 node=2 p=100400

binding component=3 authored_mm=400 node=3 p=100400

binding component=4 authored_mm=400 node=4 p=100400

multi_before validity=valid

 turbo_path=1 unit=0 turbo_path=2 unit=1 turbo_path=3 unit=0

remove accepted=1 error= note=

multi_after validity=valid

 turbo_path=2 unit=0 turbo_path=3 unit=1

gears validity=valid

gear=0 requested=1 rendered=1

gear=1 requested=0.1 rendered=0.219512195122

strobe rate=0 crank_deg_s=0 viewport_shaft_formula_deg_s=72

strobe rate=0.25 crank_deg_s=18 viewport_shaft_formula_deg_s=72

strobe rate=1 crank_deg_s=72 viewport_shaft_formula_deg_s=72
```
### app/component-output.log

```text
workshop_after_switch expected_cov=0.03 applied_cov=0.11

exhaust_property_editors=12

exhaust_cost before_cfl=0.05971 after_cfl=0.01 true_ratio=5.971

exhaust_status INFO  |  Configuration validated and applied to the engine.  Limiting CFL length 10.0 mm versus 10.0 mm before (solver cost unchanged).

telemetry_height=1136 vtec=1

telemetry_button Show in 3-D bounds=274 624 92 22

telemetry_button Fuel bounds=221 624 47 22

topbar_width=1280

 button Engine text=Engine bounds=156 6 320 42

 button A text=A bounds=606 10 33 34

 button Log text=Log bounds=599 10 47 34

 button Exhaust text=Exhaust bounds=600 10 93 34

 button Fuel text=Fuel bounds=677 10 71 34

 button Equipment text=Equipment bounds=754 10 109 34

 button Pass-by text=Pass-by bounds=869 10 94 34

 button Vehicle text=Vehicle bounds=969 10 89 34

 button ECU text=ECU bounds=1064 10 72 34

 button Audio text=Audio bounds=1142 10 80 34

 button  text= bounds=1228 10 36 34

topbar_width=1440

 button Engine text=Engine bounds=156 6 360 42

 button A text=A bounds=654 10 33 34

 button Log text=Log bounds=685 10 47 34

 button Exhaust text=Exhaust bounds=738 10 93 34

 button Fuel text=Fuel bounds=837 10 71 34

 button Equipment text=Equipment bounds=914 10 109 34

 button Pass-by text=Pass-by bounds=1029 10 94 34

 button Vehicle text=Vehicle bounds=1129 10 89 34

 button ECU text=ECU bounds=1224 10 72 34

 button Audio text=Audio bounds=1302 10 80 34

 button  text= bounds=1388 10 36 34

passby_default_window=579 206 762 628 content=0 0 762 628

passby_minimum_window=579 206 760 560 content=0 0 760 560

 minimum_label Ground bounds=18 518 150 28

 minimum_label Level bounds=18 546 150 0

passby_height=560

 label Ground bounds=18 518 150 28

 label Level bounds=18 546 150 0

passby_height=600

 label Ground bounds=18 518 150 30

 label Level bounds=18 556 150 30
```
### app/part-layout-output.log

```text
button=Reset bounds=204 158 64 26

button=Port ▾ bounds=70 158 128 26

button=Filtered ▾ bounds=108 128 160 26

button=- bounds=138 74 26 22

button=+ bounds=242 74 26 22

button=- bounds=138 100 26 22

button=+ bounds=242 100 26 22
```
### app/part-reentry-output.log

```text
after_stepper callbacks=0 old_last_value_click=203, 267

callback=1 value_count=8 requested_forward_gears=2

callback_replaced_fields=7; returning to original last-field click

CHECKED_VECTOR_FAILURE: vector subscript out of range
```
### app/oracle_witness.json

```text
{
  "scope": "Mathematical counterexamples only; no simulator/render/test execution",
  "TEST-01": {
    "target": 1000.0,
    "minimum": 900.0,
    "last_2s_mean": 2000.0,
    "final_sample": 2000.0,
    "slowCoastAllowed": true,
    "dipFailure": false,
    "returnFailure": false,
    "documentedFinalSampleBelowMean": false,
    "raisedIdlePredicateSameCondition": true
  },
  "TEST-02": {
    "original": [
      1.0,
      -1.0,
      0.0,
      0.0
    ],
    "changed": [
      1.0,
      0.0,
      -1.0,
      0.0
    ],
    "originalSummary": [
      0.7071067811865476,
      1.0
    ],
    "changedSummary": [
      0.7071067811865476,
      1.0
    ],
    "samplesEqual": false,
    "acceptedBySummaryEquality": true
  }
}
```
### root-probes.log

```text
{"case": "pop-metrics-silence", "pop_count": 0, "standard_json": false, "error": "nonstandard JSON NaN", "nan_tokens": 3}
{"case": "self-mode-label", "exit": 0, "heading": "# Blind A/B listening - Engine X through the output stage versus Engine X as rendered", "claims_two_catalogue_engines": true}
```
### contrast-probe.log

```text
[
  {
    "foreground": "5c6a65",
    "background_name": "background",
    "background": "0b0f0e",
    "ratio": 3.4021416897373724,
    "reaches_4_5": false
  },
  {
    "foreground": "5c6a65",
    "background_name": "panel",
    "background": "111716",
    "ratio": 3.19919090206678,
    "reaches_4_5": false
  },
  {
    "foreground": "5c6a65",
    "background_name": "panelRaised",
    "background": "161e1c",
    "ratio": 2.995808620484556,
    "reaches_4_5": false
  }
]
```
### root-cli-probes.log

```text
{"case": "invalid-tap-token", "args": ["--engine", "K20A", "--output", "C:\\Users\\t3anc\\OneDrive\\Bureau\\EngineLab\\out\\audit2\\root-fixtures\\tap-output", "--exhaust-tap", "bad:0.5"], "exit": 3221226505, "stdout": "", "stderr": ""}
{"case": "scene-index-write-failure", "args": ["--out", "C:\\Users\\t3anc\\OneDrive\\Bureau\\EngineLab\\out\\audit2\\root-fixtures\\blocked-scene", "--filter", "no-matching-engine-for-audit"], "exit": 0, "stdout": "total: clash 0, self 0, engine 0, bend 0, stretch 0\n", "stderr": ""}
{"case": "zero-sample-transfer", "args": ["--filter", "K20A", "--component", "100", "--seconds", ".000001", "--max-hz", "10"], "exit": 3221225477, "stdout": "", "stderr": ""}
{"case": "scene-control-character", "args": ["--out", "C:\\Users\\t3anc\\OneDrive\\Bureau\\EngineLab\\out\\audit2\\root-fixtures\\control-name-scene", "--catalog-root", "C:\\Users\\t3anc\\OneDrive\\Bureau\\EngineLab\\out\\audit2\\root-fixtures\\control-name-catalog"], "exit": 0, "stdout": "1.json   Audit\nEngine                                   clash   0  self   0  engine   0  bend   0  stretch   0\ntotal: clash 0, self 0, engine 0, bend 0, stretch 0\n", "stderr": ""}
{"case": "scene-json-standard", "accepted": false, "error": "Invalid control character at: line 1 column 16 (char 15)"}
```
### transfer-normal-control.log

```text
# duct node 4294967295 component 100 length_mm 760 inlet_d_mm 45 outlet_d_mm 45
# duct node 4294967294 component 101 length_mm 760 inlet_d_mm 45 outlet_d_mm 45
# duct node 4294967293 component 102 length_mm 760 inlet_d_mm 45 outlet_d_mm 45
# duct node 4294967292 component 103 length_mm 760 inlet_d_mm 45 outlet_d_mm 45
# duct node 4294967290 component 500 length_mm 2200 inlet_d_mm 63 outlet_d_mm 63
# duct node 4294967289 component 600 length_mm 300 inlet_d_mm 63 outlet_d_mm 63
# duct node 4294967288 component 700 length_mm 59.71 inlet_d_mm 70 outlet_d_mm 70
# component 100 fraction 0.5 density 0.55 sound_speed 520 mean_flow 0 far_field_density 1.196 last_sample_pa 0
frequency_hz,transfer_db,tap_db,microphone_db
5.00,-660.00,60.00,-600.00
10.00,-660.00,60.00,-600.00
```
### package-notices.json

```text
{
  "installedFiles": 71,
  "installedBytes": 23932213.0,
  "includesNlohmannCopyright": false,
  "includesYamlCppCopyright": false,
  "noticeFiles": [
    "LICENSE.md",
    "README.txt",
    "THIRD_PARTY_NOTICES.md",
    "assets\\ir\\LICENSE-es2d.txt",
    "assets\\ir\\README.md",
    "licenses\\OFL-Inter.txt",
    "licenses\\OFL-JetBrainsMono.txt"
  ]
}
```
### package-root-probe.json

```text
[
  {
    "mode": "default",
    "exit": 0,
    "seesPackagedSentinel": false,
    "seesRepositoryName": true,
    "stdout": "K20A-like 2.0 I4 VTEC\n2JZ-GTE-like 3.0 I6 Turbo\nLS3-like 6.2 Crossplane V8\nEJ25-like 2.5 Flat-4 Turbo\nAudi I5-like 2.5 Turbo\nHayabusa-like 1.3 I4\nBig Twin-like 1.9 V2\nMerlin-like 19.8 V12 Scaled\nAircooled-like 3.6 Flat-6\nRadial-like 6.5 R5\nYamaha CP2 MT-07-like 689 Twin\nYamaha CP3 MT-09-like 890 Triple\nYamaha CP4 MT-10-like 998 Crossplane I4\nVW EA288-like 2.0 TDI I4\nMT-07-like 689 Twin Full System\nAudio Physics Lab 689 Twin\nYamaha RD350LC-like 347 Two-Stroke Twin\n13B Renesis-like 2-Rotor\n",
    "stderr": ""
  },
  {
    "mode": "explicit",
    "exit": 0,
    "seesPackagedSentinel": true,
    "seesRepositoryName": false,
    "stdout": "Audit package catalogue sentinel\n2JZ-GTE-like 3.0 I6 Turbo\nLS3-like 6.2 Crossplane V8\nEJ25-like 2.5 Flat-4 Turbo\nAudi I5-like 2.5 Turbo\nHayabusa-like 1.3 I4\nBig Twin-like 1.9 V2\nMerlin-like 19.8 V12 Scaled\nAircooled-like 3.6 Flat-6\nRadial-like 6.5 R5\nYamaha CP2 MT-07-like 689 Twin\nYamaha CP3 MT-09-like 890 Triple\nYamaha CP4 MT-10-like 998 Crossplane I4\nVW EA288-like 2.0 TDI I4\nMT-07-like 689 Twin Full System\nAudio Physics Lab 689 Twin\nYamaha RD350LC-like 347 Two-Stroke Twin\n13B Renesis-like 2-Rotor\n",
    "stderr": ""
  }
]
```
### scenes.log

```text
1.json   K20A-like 2.0 I4 VTEC                          clash   0  self   0  engine   0  bend   0  stretch   0
2.json   2JZ-GTE-like 3.0 I6 Turbo                      clash   4  self   0  engine   0  bend   5  stretch   2
3.json   LS3-like 6.2 Crossplane V8                     clash   0  self   2  engine   0  bend  14  stretch   4
4.json   EJ25-like 2.5 Flat-4 Turbo                     clash   0  self   0  engine   0  bend   0  stretch   0
5.json   Audi I5-like 2.5 Turbo                         clash   0  self   0  engine   0  bend   0  stretch   0
6.json   Hayabusa-like 1.3 I4                           clash   0  self   1  engine   0  bend   4  stretch   0
7.json   Big Twin-like 1.9 V2                           clash   0  self   0  engine   0  bend   0  stretch   0
8.json   Merlin-like 19.8 V12 Scaled                    clash   0  self   0  engine   0  bend   0  stretch   0
9.json   Aircooled-like 3.6 Flat-6                      clash   0  self   0  engine   0  bend   0  stretch   0
10.json  Radial-like 6.5 R5                             clash   0  self   0  engine   0  bend   0  stretch   0
11.json  Yamaha CP2 MT-07-like 689 Twin                 clash   0  self   0  engine   0  bend   2  stretch   0
12.json  Yamaha CP3 MT-09-like 890 Triple               clash   0  self   0  engine   0  bend   3  stretch   0
13.json  Yamaha CP4 MT-10-like 998 Crossplane I4        clash   0  self   1  engine   0  bend   6  stretch   0
14.json  VW EA288-like 2.0 TDI I4                       clash   0  self   0  engine   0  bend   0  stretch   0
15.json  MT-07-like 689 Twin Full System                clash   0  self   0  engine   0  bend   2  stretch   0
16.json  Audio Physics Lab 689 Twin                     clash   0  self   0  engine   0  bend   2  stretch   0
17.json  Yamaha RD350LC-like 347 Two-Stroke Twin        clash   0  self   0  engine   0  bend   1  stretch   0
18.json  13B Renesis-like 2-Rotor                       clash   0  self   0  engine   0  bend   7  stretch   0
total: clash 4, self 4, engine 0, bend 46, stretch 6
```
### scene-summary.log

```text
2JZ-GTE-like 3.0 I6 Turbo {'ductClash': 4, 'selfClash': 0, 'engineClash': 0, 'tightBend': 5, 'stretched': 2}
{"issue": "ductClash", "duct": 18, "kind": 0, "path": 1, "element": 104, "other": 19, "value": 28.02, "authored_mm": 430.0, "drawn_mm": 440.944, "where_mm": [174.56, -2.47, -45.52]}
{"issue": "tightBend", "duct": 18, "kind": 0, "path": 1, "element": 104, "other": -1, "value": 0.52, "authored_mm": 430.0, "drawn_mm": 440.944, "where_mm": [165.53, -10.19, -49.81]}
{"issue": "stretched", "duct": 19, "kind": 0, "path": 1, "element": 105, "other": -1, "value": 1.28, "authored_mm": 430.0, "drawn_mm": 550.04, "where_mm": [153.94, 101.93, 134.38]}
LS3-like 6.2 Crossplane V8 {'ductClash': 0, 'selfClash': 2, 'engineClash': 0, 'tightBend': 14, 'stretched': 4}
{"issue": "selfClash", "duct": 27, "kind": 0, "path": 1, "element": 107, "other": -1, "value": 10.72, "authored_mm": 690.0, "drawn_mm": 690.202, "where_mm": [333.36, 18.55, 174.1]}
{"issue": "tightBend", "duct": 30, "kind": 0, "path": 1, "element": 212, "other": -1, "value": 0.53, "authored_mm": 140.0, "drawn_mm": 484.655, "where_mm": [-49.01, -27.74, 870.15]}
{"issue": "stretched", "duct": 30, "kind": 0, "path": 1, "element": 212, "other": -1, "value": 3.46, "authored_mm": 140.0, "drawn_mm": 484.655, "where_mm": [-191.33, -17.98, 942.09]}
Hayabusa-like 1.3 I4 {'ductClash': 0, 'selfClash': 1, 'engineClash': 0, 'tightBend': 4, 'stretched': 0}
{"issue": "selfClash", "duct": 17, "kind": 0, "path": 1, "element": 300, "other": -1, "value": 21.35, "authored_mm": 256.2, "drawn_mm": 255.568, "where_mm": [183.72, -42.79, 317.04]}
{"issue": "tightBend", "duct": 17, "kind": 0, "path": 1, "element": 300, "other": -1, "value": 0.61, "authored_mm": 256.2, "drawn_mm": 255.568, "where_mm": [239.37, -57.08, 330.69]}
Yamaha CP2 MT-07-like 689 Twin {'ductClash': 0, 'selfClash': 0, 'engineClash': 0, 'tightBend': 2, 'stretched': 0}
{"issue": "tightBend", "duct": 0, "kind": 2, "path": 1, "element": 1, "other": -1, "value": 0.61, "authored_mm": 120.0, "drawn_mm": 119.009, "where_mm": [-94.05, 250.78, -50.0]}
Yamaha CP3 MT-09-like 890 Triple {'ductClash': 0, 'selfClash': 0, 'engineClash': 0, 'tightBend': 3, 'stretched': 0}
{"issue": "tightBend", "duct": 0, "kind": 2, "path": 1, "element": 1, "other": -1, "value": 0.82, "authored_mm": 180.0, "drawn_mm": 179.17, "where_mm": [-113.65, 255.28, -97.5]}
Yamaha CP4 MT-10-like 998 Crossplane I4 {'ductClash': 0, 'selfClash': 1, 'engineClash': 0, 'tightBend': 6, 'stretched': 0}
{"issue": "selfClash", "duct": 17, "kind": 0, "path": 1, "element": 300, "other": -1, "value": 36.97, "authored_mm": 256.2, "drawn_mm": 255.696, "where_mm": [180.32, -44.67, 317.06]}
{"issue": "tightBend", "duct": 17, "kind": 0, "path": 1, "element": 300, "other": -1, "value": 0.5, "authored_mm": 256.2, "drawn_mm": 255.696, "where_mm": [240.33, -47.13, 328.14]}
MT-07-like 689 Twin Full System {'ductClash': 0, 'selfClash': 0, 'engineClash': 0, 'tightBend': 2, 'stretched': 0}
{"issue": "tightBend", "duct": 0, "kind": 2, "path": 1, "element": 1, "other": -1, "value": 0.61, "authored_mm": 120.0, "drawn_mm": 119.009, "where_mm": [-94.05, 250.78, -50.0]}
Audio Physics Lab 689 Twin {'ductClash': 0, 'selfClash': 0, 'engineClash': 0, 'tightBend': 2, 'stretched': 0}
{"issue": "tightBend", "duct": 0, "kind": 2, "path": 1, "element": 1, "other": -1, "value": 0.61, "authored_mm": 120.0, "drawn_mm": 119.009, "where_mm": [-94.05, 250.78, -50.0]}
Yamaha RD350LC-like 347 Two-Stroke Twin {'ductClash': 0, 'selfClash': 0, 'engineClash': 0, 'tightBend': 1, 'stretched': 0}
{"issue": "tightBend", "duct": 22, "kind": 0, "path": 2, "element": 201, "other": -1, "value": 0.75, "authored_mm": 260.0, "drawn_mm": 259.507, "where_mm": [165.46, 69.49, 3.7]}
13B Renesis-like 2-Rotor {'ductClash': 0, 'selfClash': 0, 'engineClash': 0, 'tightBend': 7, 'stretched': 0}
{"issue": "tightBend", "duct": 11, "kind": 0, "path": 1, "element": 101, "other": -1, "value": 0.95, "authored_mm": 450.0, "drawn_mm": 449.46, "where_mm": [212.37, 107.33, 65.85]}
```
### first-physics-refresh.log

```text
physics_probe.cpp
SimpleEcuModel.cpp
EcuCalibration.cpp
DrivelineModel.cpp
ShiftSchedule.cpp
VariatorControl.cpp
Génération de code en cours...
antilag_before_reset active=0 blockers=64
antilag_after_reset active=0 blockers=64
antilag_fresh active=1 blockers=0
backdrive slip_rpm=-100.924092831159 stick_fraction=0 engine_reaction_nm=100 engine_coupling_nm=100 reflected_inertia=0 wheel_torque_nm=-50 engine_rpm=900 wheel_rpm=1000.800067647 engine_power_w=9424.77796076938 wheel_power_w=-5240.17690038665 clutch_loss_w=1056.87462869528
aero_50_vs_120_kpa_max_speed_difference=0
infinite_knock_curve validation=accepted advance=-nan(ind) finite=0
infinite_knock_curve_yaml accepted=1 error=
descending_knock_curve validation=accepted
two_stroke_design cylinders=8 accepted=1 error=
two_stroke_design cylinders=9 accepted=0 error=Exhaust topology may contain at most 8 paths supported by audio
```
### first-data-refresh.log

```text
data-repro.cpp
Defining YAML_CPP_API for DLL import
base_valid=1
collision_paths_equal=1 first=A_B.json
first_save_error=none
second_save_error=none
first_file_now_contains=A*B
upper_ecu_name_error=none filename=My.ECU.json classified_as_calibration=1
reserved_name_error=none save_error=cannot replace CON.json
range_json_loaded=1 octane=130 warnings=1
range_yaml_loaded=0 error=range.yaml: Engine value out of range: octaneRating = 150, outside 70 to 130
oversized_schema_loaded=1 loaded_schema=9
fractional_valve_count_loaded=1 count=1
large_engine_valid=1
large_compact_bytes=812424 compact_decodes=1
large_save_error=none file_bytes=2617379
large_saved_file_loaded=0 error=large.json exceeds the 2 MiB limit
log_both_opened=1 first_log_row_preserved=0
```
### first-script-refresh.log

```text
script-repro.cpp
windows_ansi_code_page=1252
accented_include_exists=1 compile_success=0
ES302: Cannot read engine script 'C:\Users\t3anc\OneDrive\Bureau\EngineLab\out\audit\script-fixtures\é.engine'.
accented_calibration_exists=1 watcher_error=The watched calibration file was not found.
script_bytes=242 parentheses=100
returned=1 diagnostics=0
small_nesting_exit=0
script_bytes=20042 parentheses=10000
large_nesting_exit=-1073741571
```
### first-ui-refresh.log

```text
shift_conflict accepted=1 pause=1 throttle_idle=1 error=
reserved_save accepted=1 pause=1 error=
live_tuned_limit published=1 limit=3000
at_4000 live_limiter=1 export_default_limiter=0 config_limit=8700
tuner_store published=1
tuner_button 'LOAD' bounds=170 12 92 34
tuner_button 'SAVE' bounds=268 12 112 34
tuner_button 'RELOAD' bounds=386 12 58 34
tuner_button '-' bounds=12 382 38 28
tuner_button '+' bounds=54 382 38 28
tuner_button '-5 %' bounds=96 382 56 28
tuner_button '+5 %' bounds=156 382 56 28
tuner_button 'SMOOTH' bounds=224 382 80 28
tuner_button 'INTERPOLATE' bounds=308 382 108 28
tuner_button 'UNDO' bounds=428 382 64 28
tuner_button 'REDO' bounds=496 382 64 28
tuner_button 'FIT VTEC CAMS' bounds=584 12 124 34
tuner_button 'ANTI-LAG OFF' bounds=452 12 124 34
antilag_editor '0' bounds=480 110 140 26
antilag_editor '70' bounds=480 140 140 26
antilag_editor '40' bounds=480 170 140 26
antilag_editor '50' bounds=480 200 140 26
antilag_editor '33' bounds=480 230 140 26
antilag_editor '1050' bounds=480 260 140 26
antilag_editor '0.90' bounds=480 290 140 26
antilag_editor '6480' bounds=480 320 140 26
antilag_editor '4.0' bounds=480 350 140 26
antilag_editor '3240' bounds=480 380 140 26
antilag_editor '0' bounds=480 410 140 26
antilag_editor '25.0' bounds=480 440 140 26
antilag_editor_count=12 outside=1 overlapping_toolbar=3
failed_replace success=0 error=Could not replace C:\Users\t3anc\OneDrive\Bureau\EngineLab\out\audit2\first-ui-refresh-run\failed-replacement\scenario.json: The directory is not empty.
old_master_preserved=0 new_master_bytes=28844
tap duct=100
tap_export success=1 error=
old_tap_preserved=0 tap_bytes=19200
```
### audio-ab-refresh.json

```text
{
  "args": [
    "C:\\Users\\t3anc\\OneDrive\\Bureau\\EngineLab\\out\\build\\windows-vs2022\\tools\\Release\\EngineLabAudioAbHarness.exe",
    "--root",
    "C:\\Users\\t3anc\\OneDrive\\Bureau\\EngineLab",
    "--output",
    "C:\\Users\\t3anc\\OneDrive\\Bureau\\EngineLab\\out\\audit2\\audio-ab-refresh"
  ],
  "exit": 3221226505,
  "stdout": "Loaded IR: C:/Users/t3anc/OneDrive/Bureau/EngineLab\\assets\\ir\\exhaust_default.wav (33705 samples @ 44100 Hz)\nLoaded 18 catalogue engines from C:\\Users\\t3anc\\OneDrive\\Bureau\\EngineLab\\engines\nVoicing: convolution=0.45 (default), exhaustPreset=street (default)\n\n=== K20A-like 2.0 I4 VTEC  (idle 950, redline 8600 rpm) ===\n",
  "stderr": ""
}
```
### realtime-budget.log

```text
Realtime budget: simulated seconds produced per wall second by the
240 Hz EngineRuntime thread, each engine held at 60.000000% of redline.
FREE-RUN: the wall-clock sleep is removed, so the factor is CAPACITY.
1.0 is exactly break-even and leaves no margin for scheduler jitter.

AUDIO: production RealtimeEngineAudio consumer at 48000 Hz / 256 samples, outlet jet noise ON, FI band power ON, paced by accelerated simulated time.
A valid point requires no over-budget render, queue/reaction loss, late event, stolen voice, delay truncation, invalid boundary, legacy sample, voicing saturation, AGC, soft limiting, hard clamping or non-finite output. Scheduler wake misses remain diagnostic unless the render itself exceeds one block.

engine                      cyl   target  meanRpm   wrk    held   factor   sim/wall    overruns   maxLate    exhPeak    exhMean  bpWarn  tyreLim  audMean   audP99    miss   dropP    late  legacy     agc
K20A-like 2.0 I4 VTEC         4     5160     5159     2     yes    1.380        2/2           0     0.0ms      111.1       99.5      no       no    24.4%      37%       0       0       0       0       0
  gas view: 435 fields, 444 probe traces received
  audio gain min 1.0000, pre-limiter peak 0.1498
  observer peaks (Pa): exhaust 42.9, jet 2.4, intake 2.1, structure 4.9, forced induction 0.0
  audio contract: dropF=0 dropP=0 dropA=0 dropR=0 reactLimit=0 late=0 pending=0 stolen=0 trunc=0 boundary=0 legacy=0 leveler=0 saturation=0 softLimit=0 hardClamp=0 nonFinite=0 renderOver=0
  steady: torque 199.04 Nm, power 107.57 kW, VE 0.9869, exhaust substeps 22080 Hz
2JZ-GTE-like 3.0 I6 Turbo     6     4200     4202     2     yes    1.251        2/2           0     0.0ms      178.0      164.5      no       no    35.1%      49%       0       0       0       0       0
  gas view: 389 fields, 390 probe traces received
  audio gain min 1.0000, pre-limiter peak 0.2461
  observer peaks (Pa): exhaust 70.9, jet 12.9, intake 1.7, structure 5.0, forced induction 0.6
  audio contract: dropF=0 dropP=0 dropA=0 dropR=0 reactLimit=0 late=0 pending=0 stolen=0 trunc=0 boundary=0 legacy=0 leveler=0 saturation=0 softLimit=0 hardClamp=0 nonFinite=0 renderOver=0
  steady: torque 392.85 Nm, power 172.85 kW, VE 1.4379, exhaust substeps 26880 Hz
LS3-like 6.2 Crossplane V8    8     3960     3961     2     yes    1.636        3/2           0     0.0ms      105.5       95.7      no       no    41.8%      70%       0       0       0       0       0
  gas view: 333 fields, 333 probe traces received
  audio gain min 1.0000, pre-limiter peak 0.2240
  observer peaks (Pa): exhaust 63.3, jet 4.4, intake 3.8, structure 2.0, forced induction 0.0
  audio contract: dropF=0 dropP=0 dropA=0 dropR=0 reactLimit=0 late=0 pending=0 stolen=0 trunc=0 boundary=0 legacy=0 leveler=0 saturation=0 softLimit=0 hardClamp=0 nonFinite=0 renderOver=0
  steady: torque 575.05 Nm, power 238.50 kW, VE 0.9398, exhaust substeps 12720 Hz
EJ25-like 2.5 Flat-4 Turbo    4     4080     4080     2     yes    1.447        2/2           0     0.0ms      193.8      182.2      no       no    29.2%      46%       0       0       0       0       0
  gas view: 348 fields, 348 probe traces received
  audio gain min 1.0000, pre-limiter peak 0.2156
  observer peaks (Pa): exhaust 60.9, jet 11.3, intake 6.6, structure 3.2, forced induction 0.8
  audio contract: dropF=0 dropP=0 dropA=0 dropR=0 reactLimit=0 late=0 pending=0 stolen=0 trunc=0 boundary=0 legacy=0 leveler=0 saturation=0 softLimit=0 hardClamp=0 nonFinite=0 renderOver=0
  steady: torque 362.13 Nm, power 154.74 kW, VE 1.5571, exhaust substeps 26400 Hz
Audi I5-like 2.5 Turbo        5     4260     4260     2     yes    1.274        2/2           0     0.0ms      257.7      241.4      no       no    32.9%      50%       0       0       0       0       0
  gas view: 390 fields, 390 probe traces received
  audio gain min 1.0000, pre-limiter peak 0.1860
  observer peaks (Pa): exhaust 52.5, jet 23.0, intake 4.2, structure 7.8, forced induction 1.3
  audio contract: dropF=0 dropP=0 dropA=0 dropR=0 reactLimit=0 late=0 pending=0 stolen=0 trunc=0 boundary=0 legacy=0 leveler=0 saturation=0 softLimit=0 hardClamp=0 nonFinite=0 renderOver=0
  steady: torque 464.21 Nm, power 207.18 kW, VE 2.1646, exhaust substeps 27347 Hz
Hayabusa-like 1.3 I4          4     6720     6720     2     yes    1.350        2/2           0     0.0ms      134.3      101.4      no       no    27.8%      43%       0       0       0       0       0
  gas view: 370 fields, 370 probe traces received
  audio gain min 1.0000, pre-limiter peak 0.2772
  observer peaks (Pa): exhaust 81.8, jet 10.1, intake 2.3, structure 7.0, forced induction 0.0
  audio contract: dropF=0 dropP=0 dropA=0 dropR=0 reactLimit=0 late=0 pending=0 stolen=0 trunc=0 boundary=0 legacy=0 leveler=0 saturation=0 softLimit=0 hardClamp=0 nonFinite=0 renderOver=0
  steady: torque 139.88 Nm, power 98.43 kW, VE 1.0491, exhaust substeps 26285 Hz
INVALID: Merlin-like 19.8 V12 Scaled: audio realtime contract violation (dropped-pressure=1949)
Big Twin-like 1.9 V2          2     3360     3360     0     yes    2.611        5/2           0     0.0ms      119.0      100.8      no       no    20.9%      35%       0       0       0       0       0
  gas view: 255 fields, 255 probe traces received
  audio gain min 1.0000, pre-limiter peak 0.1503
  observer peaks (Pa): exhaust 44.1, jet 18.2, intake 2.8, structure 0.4, forced induction 0.0
  audio contract: dropF=0 dropP=0 dropA=0 dropR=0 reactLimit=0 late=0 pending=0 stolen=0 trunc=0 boundary=0 legacy=0 leveler=0 saturation=0 softLimit=0 hardClamp=0 nonFinite=0 renderOver=0
  steady: torque 185.37 Nm, power 65.24 kW, VE 1.0564, exhaust substeps 17646 Hz
Merlin-like 19.8 V12 Scaled   12     1920     1921     3 INVALID       --        3/2           0     0.0ms      101.3       98.5      no       no    64.6%      91%       0    1949       0       0       0
  gas view: 292 fields, 292 probe traces received
  audio gain min 1.0000, pre-limiter peak 0.1662
  observer peaks (Pa): exhaust 53.8, jet 0.0, intake 6.6, structure 1.5, forced induction 0.5
  audio contract: dropF=0 dropP=1949 dropA=0 dropR=0 reactLimit=0 late=0 pending=0 stolen=0 trunc=0 boundary=0 legacy=0 leveler=0 saturation=0 softLimit=0 hardClamp=0 nonFinite=0 renderOver=0
Aircooled-like 3.6 Flat-6     6     4440     4441     2     yes    1.326        2/2           0     0.0ms      101.7       96.8      no       no    33.5%      54%       0       0       0       0       0
  gas view: 372 fields, 372 probe traces received
  audio gain min 1.0000, pre-limiter peak 0.3825
  observer peaks (Pa): exhaust 136.0, jet 1.2, intake 5.4, structure 0.8, forced induction 0.0
  audio contract: dropF=0 dropP=0 dropA=0 dropR=0 reactLimit=0 late=0 pending=0 stolen=0 trunc=0 boundary=0 legacy=0 leveler=0 saturation=0 softLimit=0 hardClamp=0 nonFinite=0 renderOver=0
  steady: torque 334.12 Nm, power 155.38 kW, VE 1.0507, exhaust substeps 24311 Hz
Radial-like 6.5 R5            5     1440     1439     2     yes    2.748        5/2           0     0.0ms      101.3       98.4      no       no    23.9%      42%       0       0       0       0       0
  gas view: 256 fields, 256 probe traces received
  audio gain min 1.0000, pre-limiter peak 0.1334
  observer peaks (Pa): exhaust 16.3, jet 0.0, intake 2.5, structure 0.3, forced induction 0.0
  audio contract: dropF=0 dropP=0 dropA=0 dropR=0 reactLimit=0 late=0 pending=0 stolen=0 trunc=0 boundary=0 legacy=0 leveler=0 saturation=0 softLimit=0 hardClamp=0 nonFinite=0 renderOver=0
  steady: torque 504.94 Nm, power 76.23 kW, VE 0.9170, exhaust substeps 9592 Hz
Yamaha CP2 MT-07-like 689 Twin    2     6000     6003     0     yes    2.248        4/2           0     0.0ms      162.7      104.0      no       no    17.5%      28%       0       0       0       0       0
  gas view: 274 fields, 274 probe traces received
  audio gain min 1.0000, pre-limiter peak 0.1431
  observer peaks (Pa): exhaust 40.5, jet 10.2, intake 0.8, structure 2.3, forced induction 0.0
  audio contract: dropF=0 dropP=0 dropA=0 dropR=0 reactLimit=0 late=0 pending=0 stolen=0 trunc=0 boundary=0 legacy=0 leveler=0 saturation=0 softLimit=0 hardClamp=0 nonFinite=0 renderOver=0
  steady: torque 68.37 Nm, power 42.96 kW, VE 0.9925, exhaust substeps 22303 Hz
Yamaha CP3 MT-09-like 890 Triple    3     6600     6600     2     yes    1.836        3/2           0     0.0ms      109.5       93.0      no       no    21.4%      35%       0       0       0       0       0
  gas view: 315 fields, 316 probe traces received
  audio gain min 1.0000, pre-limiter peak 0.2543
  observer peaks (Pa): exhaust 74.0, jet 0.5, intake 1.4, structure 4.0, forced induction 0.0
  audio contract: dropF=0 dropP=0 dropA=0 dropR=0 reactLimit=0 late=0 pending=0 stolen=0 trunc=0 boundary=0 legacy=0 leveler=0 saturation=0 softLimit=0 hardClamp=0 nonFinite=0 renderOver=0
  steady: torque 94.91 Nm, power 65.60 kW, VE 1.0485, exhaust substeps 21362 Hz
Yamaha CP4 MT-10-like 998 Crossplane I4    4     7200     7202     2     yes    1.355        2/2           0     0.0ms      126.8       95.4      no       no    28.7%      43%       0       0       0       0       0
  gas view: 369 fields, 369 probe traces received
  audio gain min 1.0000, pre-limiter peak 0.1272
  observer peaks (Pa): exhaust 37.6, jet 0.7, intake 2.3, structure 6.9, forced induction 0.0
  audio contract: dropF=0 dropP=0 dropA=0 dropR=0 reactLimit=0 late=0 pending=0 stolen=0 trunc=0 boundary=0 legacy=0 leveler=0 saturation=0 softLimit=0 hardClamp=0 nonFinite=0 renderOver=0
  steady: torque 106.30 Nm, power 80.15 kW, VE 1.0285, exhaust substeps 23215 Hz
VW EA288-like 2.0 TDI I4      4     3000     3002     2     yes    2.259        4/2           0     0.0ms      184.3      187.7      no       no    25.5%      39%       0       0       0       0       0
  gas view: 277 fields, 277 probe traces received
  audio gain min 1.0000, pre-limiter peak 0.1828
  observer peaks (Pa): exhaust 53.0, jet 2.3, intake 1.6, structure 6.1, forced induction 0.4
  audio contract: dropF=0 dropP=0 dropA=0 dropR=0 reactLimit=0 late=0 pending=0 stolen=0 trunc=0 boundary=0 legacy=0 leveler=0 saturation=0 softLimit=0 hardClamp=0 nonFinite=0 renderOver=0
  steady: torque 268.90 Nm, power 84.53 kW, VE 1.5257, exhaust substeps 12960 Hz
MT-07-like 689 Twin Full System    2     6000     5998     0     yes    2.387        4/2           0     0.0ms      104.0       99.1      no       no    17.5%      29%       0       0       0       0       0
  gas view: 261 fields, 261 probe traces received
  audio gain min 1.0000, pre-limiter peak 0.1381
  observer peaks (Pa): exhaust 38.6, jet 6.2, intake 0.8, structure 2.3, forced induction 0.0
  audio contract: dropF=0 dropP=0 dropA=0 dropR=0 reactLimit=0 late=0 pending=0 stolen=0 trunc=0 boundary=0 legacy=0 leveler=0 saturation=0 softLimit=0 hardClamp=0 nonFinite=0 renderOver=0
  steady: torque 68.58 Nm, power 43.09 kW, VE 0.9922, exhaust substeps 18131 Hz
Audio Physics Lab 689 Twin    2     6000     6007     0     yes    2.198        4/2           0     0.0ms      118.1      102.6      no       no    19.0%      33%       0       0       0       0       0
  gas view: 270 fields, 270 probe traces received
  audio gain min 1.0000, pre-limiter peak 0.1454
  observer peaks (Pa): exhaust 41.0, jet 6.3, intake 0.9, structure 2.6, forced induction 0.0
1 invalid measurement(s); no comparison is allowed
0 engine(s) below the realtime floor
  audio contract: dropF=0 dropP=0 dropA=0 dropR=0 reactLimit=0 late=0 pending=0 stolen=0 trunc=0 boundary=0 legacy=0 leveler=0 saturation=0 softLimit=0 hardClamp=0 nonFinite=0 renderOver=0
  steady: torque 68.53 Nm, power 43.06 kW, VE 0.9922, exhaust substeps 18017 Hz
Yamaha RD350LC-like 347 Two-Stroke Twin    2     5700     5701     1     yes    1.536        3/2           0     0.0ms      108.1      109.2      no       no    37.8%      61%       0       0       0       0       0
  gas view: 352 fields, 352 probe traces received
  audio gain min 1.0000, pre-limiter peak 0.1083
  observer peaks (Pa): exhaust 33.0, jet 0.0, intake 0.6, structure 0.5, forced induction 0.0
  audio contract: dropF=0 dropP=0 dropA=0 dropR=0 reactLimit=0 late=0 pending=0 stolen=0 trunc=0 boundary=0 legacy=0 leveler=0 saturation=0 softLimit=0 hardClamp=0 nonFinite=0 renderOver=0
  steady: torque 22.53 Nm, power 13.45 kW, VE 0.4632, exhaust substeps 18299 Hz
13B Renesis-like 2-Rotor      6     5400     5399     2     yes    1.474        2/2           0     0.0ms      101.3       92.0      no       no    27.4%      44%       0       0       0       0       0
  gas view: 360 fields, 360 probe traces received
  audio gain min 1.0000, pre-limiter peak 0.1926
  observer peaks (Pa): exhaust 55.4, jet 7.8, intake 3.0, structure 13.6, forced induction 0.0
  audio contract: dropF=0 dropP=0 dropA=0 dropR=0 reactLimit=0 late=0 pending=0 stolen=0 trunc=0 boundary=0 legacy=0 leveler=0 saturation=0 softLimit=0 hardClamp=0 nonFinite=0 renderOver=0
  steady: torque 212.65 Nm, power 120.25 kW, VE 0.8828, exhaust substeps 21686 Hz

worst: 2JZ-GTE-like 3.0 I6 Turbo at 1.251
```
### realtime-merlin-paced.log

```text
Realtime budget: simulated seconds produced per wall second by the
240 Hz EngineRuntime thread, each engine held at 60.000000% of redline.
A factor below 1.0 means the simulation is running in slow motion.
It saturates at 1.0; use --free-run to see the headroom above it.

AUDIO: production RealtimeEngineAudio consumer at 48000 Hz / 256 samples, outlet jet noise ON, FI band power ON, paced by wall-clock deadlines.
A valid point requires no over-budget render, queue/reaction loss, late event, stolen voice, delay truncation, invalid boundary, legacy sample, voicing saturation, AGC, soft limiting, hard clamping or non-finite output. Scheduler wake misses remain diagnostic unless the render itself exceeds one block.

engine                      cyl   target  meanRpm   wrk    held   factor   sim/wall    overruns   maxLate    exhPeak    exhMean  bpWarn  tyreLim  audMean   audP99    miss   dropP    late  legacy     agc
Merlin-like 19.8 V12 Scaled   12     1920     1921     3     yes    0.991        4/5          28    22.5ms      101.3       98.5      no       no    54.7%      85%      16       0       0       0       0
  gas view: 641 fields, 642 probe traces received
  audio gain min 1.0000, pre-limiter peak 0.1579
  observer peaks (Pa): exhaust 54.1, jet 0.1, intake 3.0, structure 1.6, forced induction 0.6
  audio contract: dropF=0 dropP=0 dropA=0 dropR=0 reactLimit=0 late=0 pending=0 stolen=0 trunc=0 boundary=0 legacy=0 leveler=0 saturation=0 softLimit=0 hardClamp=0 nonFinite=0 renderOver=0
  steady: torque 2091.12 Nm, power 420.58 kW, VE 1.2781, exhaust substeps 14587 Hz

worst: Merlin-like 19.8 V12 Scaled at 0.991
```
### realtime-merlin-free.log

```text
Realtime budget: simulated seconds produced per wall second by the
240 Hz EngineRuntime thread, each engine held at 60.000000% of redline.
FREE-RUN: the wall-clock sleep is removed, so the factor is CAPACITY.
1.0 is exactly break-even and leaves no margin for scheduler jitter.

AUDIO: production RealtimeEngineAudio consumer at 48000 Hz / 256 samples, outlet jet noise ON, FI band power ON, paced by accelerated simulated time.
A valid point requires no over-budget render, queue/reaction loss, late event, stolen voice, delay truncation, invalid boundary, legacy sample, voicing saturation, AGC, soft limiting, hard clamping or non-finite output. Scheduler wake misses remain diagnostic unless the render itself exceeds one block.

engine                      cyl   target  meanRpm   wrk    held   factor   sim/wall    overruns   maxLate    exhPeak    exhMean  bpWarn  tyreLim  audMean   audP99    miss   dropP    late  legacy     agc
Merlin-like 19.8 V12 Scaled   12     1920     1920     3 INVALID       --       10/5           0     0.0ms      101.3       98.5      no       no    59.8%      91%       2   17475       0       0       0
  gas view: 466 fields, 466 probe traces received
  audio gain min 1.0000, pre-limiter peak 0.1537
  observer peaks (Pa): exhaust 47.3, jet 0.0, intake 8.7, structure 1.6, forced induction 0.5
  audio contract: dropF=0 dropP=17475 dropA=0 dropR=0 reactLimit=0 late=0 pending=0 stolen=0 trunc=0 boundary=0 legacy=0 leveler=0 saturation=0 softLimit=0 hardClamp=0 nonFinite=0 renderOver=2

worst: n/a (no valid points)
INVALID: Merlin-like 19.8 V12 Scaled: audio realtime contract violation (dropped-pressure=17475, render-over-budget=2)
1 invalid measurement(s); no comparison is allowed
0 engine(s) below the realtime floor
```

## Desktop inspection boundary

The freshly staged application was launched and exposed one EngineLab window. Native automation failed activation (`failed to activate captured window`); a refreshed selection and capture retry failed with `FrameArrived timed out: timed out waiting on channel`. No subsequent UI input was issued. Only the scratch-staged process launched for this check was stopped. This does not establish an application startup defect or a live tour. The app PNGs are software captures of actual JUCE components, not live desktop screenshots.


## Real-time scope and interpretation

The18-entry short capacity run used `--seconds 2 --relative-rpm0.6 --free-run --with-audio --audio-rate48000 --audio-block256 --gas-view --enforce1.0` (spaces between each option/value in the actual command). No other audit compilation/simulation ran. Physics, the production audio consumer and60Hz gas-field/probe capture were included; OpenGL drawing and a hardware audio device were absent. Seventeen held points passed their integrity counters, with individual capacity factors1.251–2.748. Merlin lost1949 pressure frames; that point and the complete-catalogue comparison are invalid, not a successful eighteen-engine capacity result.

A five-second Merlin control without `--free-run`, with floor0.99, returned0/factor0.991 and zero queue/audio-contract losses (28 runtime overruns/max22.5ms late and16 consumer wake misses remain diagnostics). A five-second accelerated repeat returned1 with17475 pressure drops and2 over-budget renders. The controlled paced result does not establish UI/device behavior or capacity headroom; accelerated loss alone is not evidence of a normal-speed application audio defect. No performance code was changed or adopted. Docker Desktop stop initially exceeded its30s CLI deadline; a subsequent process/status check found no Docker backend/frontend processes, and the targeted controls ran after that cleanup. Ordinary background applications were not controlled.

Fresh Linux scope and exact compiler failure are preserved in [linux.md](linux.md). The standard build did not pass; four independent tests did.


## Artifact integrity

Hashes bind the durable evidence to its original scratch output; executables and large scene arrays stay in ignored out/.

| Scratch artifact | SHA-256 |
|---|---|
| `out/audit2/physics/component_probe.log` | `f5bb8c20192b31a18ac3d648f7ec1a3dd9315ace1d0d8f58ba4f50221f25f773` |
| `out/audit2/physics/ecu_probe.log` | `d20aeb06b88281ce9e2a8c6454d6b50dcdd8e256c61fb8fd1201874575ebd553` |
| `out/audit2/physics/persistence_probe.log` | `6f80d9b7df7fd0095c380e5348f2fe199cbb2b5fafc3da17280d1b6c86188c4c` |
| `out/audit2/physics/alias_probe.log` | `4d90b92a795d64585e9499989a0efe3ed6dde45eb2d1740b28b961c1c032b652` |
| `out/audit2/audio/probe-output.txt` | `9a164fc54808e859796de16a897e41356f1fd8c77200410af95040e8739b37c3` |
| `out/audit2/audio/zero-match-cli.txt` | `11477d16a1856043a9acc2dfe923a1cd7c21476442672e46d02be100f8a7732b` |
| `out/audit2/audio/tool-text-evidence.json` | `9fb0ad453fda67c7adea4634092d97798f7ee4ca716d14e1de1c0ca3aa08d869` |
| `out/audit2/audio/crossover-delay-math.json` | `d9da725502b6d8a35ac5b1f1b7dd6d14d45186575e2a08424e173eb052d4c679` |
| `out/audit2/app/model-output.log` | `52b77cfee7ebf01c217c3afcbd7427675fd871a8ae193deb1ad875b5a8ca9429` |
| `out/audit2/app/component-output.log` | `124a14d40dbad4157f442eabfbe23db00be1aa1ecfba51ac563efa34172f17ec` |
| `out/audit2/app/part-layout-output.log` | `d5e2a280839c54f1124b8a3a68239ff752cc0bfbedbcb074b239a33284c5b4c1` |
| `out/audit2/app/part-reentry-output.log` | `6a3c91588788975a1d663c6aea50c35d7cdd9c3dc07ef1c77f32299b9062740f` |
| `out/audit2/app/oracle_witness.json` | `98adb66b91f61e7109b6b82c56d07c0c1a7a870d64e3a2fad58df25e320bd892` |
| `out/audit2/root-probes.log` | `cacc7783d091b8b8aa78428a7e8f8e9cec9727b8db050d37123288429fcd993b` |
| `out/audit2/contrast-probe.log` | `82366065fc812efd2741bfeebd59b6374ae85f19bd775cccae99dc49f48e3029` |
| `out/audit2/root-cli-probes.log` | `72298474360b1ddf4bc03b99c2e47c41d762b8d751cfc93879401f131dfd1e3e` |
| `out/audit2/transfer-normal-control.log` | `8b1363fcd256718bec066058744f64fda0b795312bf28b53cb74247bf8bdedbf` |
| `out/audit2/package-notices.json` | `52eb74109116af4d98e81406edf6ed873b475ac1063151e98ae0e63f59c0ce1f` |
| `out/audit2/package-root-probe.json` | `88e4cd58b7c506b60e686c0e08bd71250fd2266966f28e2b0c585286afd7d0df` |
| `out/audit2/scenes.log` | `c9e1fefb4cc9cc79b78cf0a6ebb38dd00ea5eb07f4fd7cd16c5697495a5558e3` |
| `out/audit2/scene-summary.log` | `2c431235d084c48e7ea308afa0e72eed31cc191a414fe052414e87e6a17ce014` |
| `out/audit2/first-physics-refresh.log` | `857b4d0ed502561d86074fdeb49f1f363620b9c056016f47705e8c195f129f51` |
| `out/audit2/first-data-refresh.log` | `95b0bf4967d8c1fba47b15e193ee8590c56b6420bb5edfbae8e747a7a0a413fb` |
| `out/audit2/first-script-refresh.log` | `4b48076391300f13aa8a6af40d768bc27397e73311c0ff48097c3a3ed354f1ff` |
| `out/audit2/first-ui-refresh.log` | `febf5a153546e14b281f46ec41ae220f2e562a0bde77b4ca1965e35b3fa7301a` |
| `out/audit2/audio-ab-refresh.json` | `7e5e380580e342c410c62bab350e361502c806f7a76ae11689bde2908be369ae` |
| `out/audit2/realtime-budget.log` | `6cfd2e4ff2021207738aef14f81bec4160506855ea6658743d290741a3a7b8ef` |
| `out/audit2/realtime-merlin-paced.log` | `74b30dc5d2f63a691925b5789d4ef03a057c93407b997252b36d47cc03c82da6` |
| `out/audit2/realtime-merlin-free.log` | `8961cf99616d9766f541a2225700474cc39ed3f8d532b0b5ecc782f7904cdd98` |
| `out/audit2/full-build.log` | `1e942e8840a4024656d5a8609ac27b26df371d51de0914f6ae2bd039ffbb7b8c` |
| `out/audit2/full-ctest.log` | `f346127312c815bb56f7d49ee7456c628cc2edf657a1b299c01a8dc33633bc7e` |
| `out/audit2/full-ctest.xml` | `1c129dc2605ce9706166c5ff03753d71f95da1d308bb2fef59b6fc3cb9cad9b5` |
| `out/audit2/full-LastTest.log` | `cb0205eef349d2a7a4fe808a0b4cd5bd4fd58589868f77ea30007e5dc3564630` |
| `out/audit2/isolated-reruns.log` | `bedbac118452158b57a473f781fd8a2ad9ae1131b96db1668dc87697d21b5c7b` |
| `out/audit2/isolated-reruns.xml` | `da6d36e7f4c21ac2f68ca61bedcb0ae4f8f43ce011a3143bf61dd37efa1db45c` |
| `out/audit2/package-files.json` | `ee8ed0c2b1cf88bed7cd9272e642afd12cd09bd833adf91f15f52da68ddf31f1` |

## Document and workspace checks

The completed register has 90 unique indexed findings (13 P1, 66 P2, 11 P3). Checks of eight Markdown artifacts found 588 valid relative links and no unclosed fences. Hash comparison against the 553-file starting inventory found only the audit journal entry changed among tracked files; every application source/test/data/build file retained its starting bytes, including all pre-existing local edits. Structured result: final-check.json. These checks validate artifact consistency, not every product behavior.

