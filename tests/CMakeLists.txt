add_executable(EngineLabCoreTests CoreTests.cpp)
target_link_libraries(EngineLabCoreTests PRIVATE
    EngineLab::Foundation EngineLab::Events EngineLab::Physics EngineLab::Ecu
    EngineLab::Exhaust EngineLab::Simulation EngineLab::Serialization EngineLab::Catalog EngineLab::Runtime)
target_link_libraries(EngineLabCoreTests PRIVATE EngineLab::Audio)
target_compile_definitions(EngineLabCoreTests PRIVATE ENGINELAB_CATALOG_ROOT="${CMAKE_SOURCE_DIR}")
enginelab_set_warnings(EngineLabCoreTests)
add_test(NAME EngineLab.Core COMMAND EngineLabCoreTests)

add_executable(EngineLabPhysicsRegressionTests PhysicsRegressionTests.cpp)
target_link_libraries(EngineLabPhysicsRegressionTests PRIVATE EngineLab::Foundation EngineLab::Physics)
enginelab_set_warnings(EngineLabPhysicsRegressionTests)
add_test(NAME EngineLab.PhysicsRegression COMMAND EngineLabPhysicsRegressionTests)

add_executable(EngineLabCombustionPhasingTests CombustionPhasingTests.cpp)
target_link_libraries(EngineLabCombustionPhasingTests PRIVATE
    EngineLab::Foundation EngineLab::Events EngineLab::Physics EngineLab::Ecu
    EngineLab::Exhaust EngineLab::Simulation)
enginelab_set_warnings(EngineLabCombustionPhasingTests)
add_test(NAME EngineLab.CombustionPhasing COMMAND EngineLabCombustionPhasingTests)

add_executable(EngineLabIntakeTuningTests IntakeTuningTests.cpp)
target_link_libraries(EngineLabIntakeTuningTests PRIVATE
    EngineLab::Foundation EngineLab::Events EngineLab::Physics EngineLab::Ecu
    EngineLab::Exhaust EngineLab::Simulation)
enginelab_set_warnings(EngineLabIntakeTuningTests)
# --enforce-tuning keeps the four literature criteria hard. --compare-oracle
# additionally constrains the realtime 95 mm/Euler/multirate reduction to 15%
# point-by-point against the former 30 mm/RK2/every-substep configuration.
# These are independent guards: real-engine curve shape and numerical drift.
add_test(NAME EngineLab.IntakeTuning COMMAND EngineLabIntakeTuningTests
    --enforce-tuning --compare-oracle)
set_tests_properties(EngineLab.IntakeTuning PROPERTIES TIMEOUT 600)

add_executable(EngineLabGasExchangeTests GasExchangeTests.cpp)
target_link_libraries(EngineLabGasExchangeTests PRIVATE
    EngineLab::Foundation EngineLab::Events EngineLab::Physics EngineLab::Ecu
    EngineLab::Exhaust EngineLab::Simulation)
enginelab_set_warnings(EngineLabGasExchangeTests)
# Registered WITHOUT --enforce-gas-exchange on purpose. The two literature
# pumping ceilings are frozen from Heywood ch. 13 and the simulator measures
# roughly three times the high-rpm one today, so they are reported and not
# gated; the always-on checks (sign, growth, net == gross + pmep, absorber
# holding, WOT MAP) do gate. Promotion is adding the flag here -- never
# relaxing a ceiling in the test. Same staging as EngineLab.IntakeTuning.
add_test(NAME EngineLab.GasExchange COMMAND EngineLabGasExchangeTests)
set_tests_properties(EngineLab.GasExchange PROPERTIES TIMEOUT 600)

add_executable(EngineLabOverrunThermalRegressionTests OverrunThermalRegressionTests.cpp)
target_link_libraries(EngineLabOverrunThermalRegressionTests PRIVATE
    EngineLab::Foundation EngineLab::Events EngineLab::Physics EngineLab::Ecu
    EngineLab::Exhaust EngineLab::Simulation EngineLab::Catalog)
target_compile_definitions(EngineLabOverrunThermalRegressionTests PRIVATE
    ENGINELAB_CATALOG_ROOT="${CMAKE_SOURCE_DIR}")
enginelab_set_warnings(EngineLabOverrunThermalRegressionTests)
add_test(NAME EngineLab.OverrunThermalRegression
    COMMAND EngineLabOverrunThermalRegressionTests)

add_executable(EngineLabIdleStabilityRegressionTests IdleStabilityRegressionTests.cpp)
target_link_libraries(EngineLabIdleStabilityRegressionTests PRIVATE
    EngineLab::Foundation EngineLab::Events EngineLab::Physics EngineLab::Ecu
    EngineLab::Exhaust EngineLab::Simulation EngineLab::Catalog)
target_compile_definitions(EngineLabIdleStabilityRegressionTests PRIVATE
    ENGINELAB_CATALOG_ROOT="${CMAKE_SOURCE_DIR}")
enginelab_set_warnings(EngineLabIdleStabilityRegressionTests)
add_test(NAME EngineLab.IdleStabilityRegression
    COMMAND EngineLabIdleStabilityRegressionTests)

add_executable(EngineLabEcuCalibrationIntegrationTests EcuCalibrationIntegrationTests.cpp)
target_link_libraries(EngineLabEcuCalibrationIntegrationTests PRIVATE
    EngineLab::Foundation EngineLab::Calibration EngineLab::Ecu)
enginelab_set_warnings(EngineLabEcuCalibrationIntegrationTests)
add_test(NAME EngineLab.EcuCalibration COMMAND EngineLabEcuCalibrationIntegrationTests)

add_executable(EngineLabRealtimeRegressionTests RealtimeRegressionTests.cpp)
target_link_libraries(EngineLabRealtimeRegressionTests PRIVATE EngineLab::Audio)
enginelab_set_warnings(EngineLabRealtimeRegressionTests)
add_test(NAME EngineLab.RealtimeRegression COMMAND EngineLabRealtimeRegressionTests)
if(MSVC)
    target_link_options(EngineLabRealtimeRegressionTests PRIVATE /STACK:8388608)
endif()

if(TARGET EngineLabComparisonHarness)
    add_test(NAME EngineLab.CatalogPhysics COMMAND EngineLabComparisonHarness
        --output "${CMAKE_BINARY_DIR}/catalog-physics-test"
        --catalog-root "${CMAKE_SOURCE_DIR}")
endif()
if(TARGET EngineLabDynoSweepHarness)
    # The header and each data row are generated from fixed-size arrays whose
    # sizes are tied by a static assertion. This fast invocation also catches a
    # stale executable or argument/parser regression before a multi-minute sweep.
    add_test(NAME EngineLab.DynoSchema COMMAND EngineLabDynoSweepHarness --schema-only)
    # Manufacturer-anchored rated points. Unlike CatalogPhysics, this is an
    # absolute torque/power gate: every point must be held within 2% rpm and
    # land within the committed 15% real-engine envelope.
    add_test(NAME EngineLab.CatalogReference
        COMMAND EngineLabDynoSweepHarness
            --catalog-root "${CMAKE_SOURCE_DIR}"
            --reference-file
                "${CMAKE_SOURCE_DIR}/tests/reference-data/catalog-manufacturer-points.csv")
    set_tests_properties(EngineLab.DynoSchema PROPERTIES
        PASS_REGULAR_EXPRESSION
            "exhaust_stroke_mep_bar,intake_stroke_mep_bar")
    set_tests_properties(EngineLab.CatalogReference PROPERTIES TIMEOUT 600)
endif()
if(TARGET EngineLabAudioRenderHarness)
    add_test(NAME EngineLab.AudioRender COMMAND EngineLabAudioRenderHarness)
    add_test(NAME EngineLab.AudioTransients
        COMMAND EngineLabAudioRenderHarness
            --transient-only
            --output "${CMAKE_BINARY_DIR}/audio-transient-test")
    if(ENGINELAB_ENABLE_SANITIZERS)
        set_tests_properties(EngineLab.AudioRender PROPERTIES TIMEOUT 600)
        set_tests_properties(EngineLab.AudioTransients PROPERTIES TIMEOUT 600)
    else()
        set_tests_properties(EngineLab.AudioRender PROPERTIES TIMEOUT 180)
        set_tests_properties(EngineLab.AudioTransients PROPERTIES TIMEOUT 180)
    endif()
endif()
if(TARGET EngineLabShiftTransientHarness)
    add_test(NAME EngineLab.AudioShiftTransientNA
        COMMAND EngineLabShiftTransientHarness
            --filter "K20"
            --audio-output "${CMAKE_BINARY_DIR}/audio-shift-transient-na-test")
    add_test(NAME EngineLab.AudioShiftTransientBoosted
        COMMAND EngineLabShiftTransientHarness
            --filter "2JZ"
            --audio-output "${CMAKE_BINARY_DIR}/audio-shift-transient-boosted-test")
    set_tests_properties(
        EngineLab.AudioShiftTransientNA
        EngineLab.AudioShiftTransientBoosted
        PROPERTIES TIMEOUT 180)
endif()
if(MSVC)
    target_link_options(EngineLabCoreTests PRIVATE /STACK:8388608)
endif()
if(MSVC AND ENGINELAB_ENABLE_SANITIZERS)
    get_filename_component(ENGINELAB_MSVC_RUNTIME_DIR "${CMAKE_CXX_COMPILER}" DIRECTORY)
    set_tests_properties(EngineLab.Core PROPERTIES
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:${ENGINELAB_MSVC_RUNTIME_DIR}")
endif()
