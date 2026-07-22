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

add_executable(EngineLabSubstepParallelTests SubstepParallelTests.cpp)
target_link_libraries(EngineLabSubstepParallelTests PRIVATE EngineLab::Simulation)
enginelab_set_warnings(EngineLabSubstepParallelTests)
add_test(NAME EngineLab.SubstepParallel COMMAND EngineLabSubstepParallelTests)
set_tests_properties(EngineLab.SubstepParallel PROPERTIES TIMEOUT 30)

add_executable(EngineLabCombustionPhasingTests CombustionPhasingTests.cpp)
target_link_libraries(EngineLabCombustionPhasingTests PRIVATE
    EngineLab::Foundation EngineLab::Events EngineLab::Physics EngineLab::Ecu
    EngineLab::Exhaust EngineLab::Simulation)
enginelab_set_warnings(EngineLabCombustionPhasingTests)
add_test(NAME EngineLab.CombustionPhasing COMMAND EngineLabCombustionPhasingTests)

add_executable(EngineLabOverrunThermalRegressionTests OverrunThermalRegressionTests.cpp)
target_link_libraries(EngineLabOverrunThermalRegressionTests PRIVATE
    EngineLab::Foundation EngineLab::Events EngineLab::Physics EngineLab::Ecu
    EngineLab::Exhaust EngineLab::Simulation EngineLab::Catalog)
target_compile_definitions(EngineLabOverrunThermalRegressionTests PRIVATE
    ENGINELAB_CATALOG_ROOT="${CMAKE_SOURCE_DIR}")
enginelab_set_warnings(EngineLabOverrunThermalRegressionTests)
add_test(NAME EngineLab.OverrunThermalRegression
    COMMAND EngineLabOverrunThermalRegressionTests)

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
if(TARGET EngineLabAudioRenderHarness)
    add_test(NAME EngineLab.AudioRender COMMAND EngineLabAudioRenderHarness)
    if(ENGINELAB_ENABLE_SANITIZERS)
        set_tests_properties(EngineLab.AudioRender PROPERTIES TIMEOUT 600)
    else()
        set_tests_properties(EngineLab.AudioRender PROPERTIES TIMEOUT 180)
    endif()
endif()
if(MSVC)
    target_link_options(EngineLabCoreTests PRIVATE /STACK:8388608)
endif()
if(MSVC AND ENGINELAB_ENABLE_SANITIZERS)
    get_filename_component(ENGINELAB_MSVC_RUNTIME_DIR "${CMAKE_CXX_COMPILER}" DIRECTORY)
    set_tests_properties(EngineLab.Core PROPERTIES
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:${ENGINELAB_MSVC_RUNTIME_DIR}")
endif()
