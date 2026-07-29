add_executable(EngineLabComparisonHarness ComparisonHarness.cpp)
target_link_libraries(EngineLabComparisonHarness PRIVATE
    EngineLab::Simulation EngineLab::Foundation EngineLab::Physics
    EngineLab::Ecu EngineLab::Events EngineLab::Exhaust EngineLab::Catalog)
enginelab_set_warnings(EngineLabComparisonHarness)

add_executable(EngineLabPhysicsPerfHarness PhysicsPerfHarness.cpp)
target_link_libraries(EngineLabPhysicsPerfHarness PRIVATE
    EngineLab::Simulation EngineLab::Foundation EngineLab::Physics
    EngineLab::Ecu EngineLab::Events EngineLab::Exhaust EngineLab::Catalog)
enginelab_set_warnings(EngineLabPhysicsPerfHarness)

add_executable(EngineLabDynoSweepHarness DynoSweepHarness.cpp)
target_link_libraries(EngineLabDynoSweepHarness PRIVATE
    EngineLab::Simulation EngineLab::Foundation EngineLab::Physics
    EngineLab::Ecu EngineLab::Events EngineLab::Exhaust EngineLab::Catalog)
enginelab_set_warnings(EngineLabDynoSweepHarness)

add_executable(EngineLabIntakeDuctBench IntakeDuctBench.cpp)
target_link_libraries(EngineLabIntakeDuctBench PRIVATE
    EngineLab::GasDynamics EngineLab::Physics EngineLab::Foundation)
enginelab_set_warnings(EngineLabIntakeDuctBench)

add_executable(EngineLabExhaustFlowBench ExhaustFlowBench.cpp)
target_link_libraries(EngineLabExhaustFlowBench PRIVATE
    EngineLab::GasDynamics EngineLab::Exhaust EngineLab::Foundation
    EngineLab::Physics EngineLab::Catalog)
enginelab_set_warnings(EngineLabExhaustFlowBench)

add_executable(EngineLabShiftTransientHarness ShiftTransientHarness.cpp)
target_link_libraries(EngineLabShiftTransientHarness PRIVATE
    EngineLab::Audio EngineLab::Runtime EngineLab::Simulation EngineLab::Foundation EngineLab::Physics
    EngineLab::Ecu EngineLab::Events EngineLab::Exhaust EngineLab::Catalog)
target_compile_definitions(EngineLabShiftTransientHarness PRIVATE ENGINELAB_CATALOG_ROOT="${CMAKE_SOURCE_DIR}")
if(MSVC)
    target_link_options(EngineLabShiftTransientHarness PRIVATE /STACK:8388608)
endif()
enginelab_set_warnings(EngineLabShiftTransientHarness)

add_executable(EngineLabAudioRenderHarness AudioRenderHarness.cpp)
target_link_libraries(EngineLabAudioRenderHarness PRIVATE
    EngineLab::Audio EngineLab::Simulation EngineLab::Foundation EngineLab::Physics
    EngineLab::Ecu EngineLab::Events EngineLab::Exhaust EngineLab::Catalog)
target_compile_definitions(EngineLabAudioRenderHarness PRIVATE ENGINELAB_CATALOG_ROOT="${CMAKE_SOURCE_DIR}")
if(MSVC)
    target_link_options(EngineLabAudioRenderHarness PRIVATE /STACK:8388608)
endif()
enginelab_set_warnings(EngineLabAudioRenderHarness)

add_executable(EngineLabRealtimeBudgetHarness RealtimeBudgetHarness.cpp)
target_link_libraries(EngineLabRealtimeBudgetHarness PRIVATE
    EngineLab::Audio EngineLab::Runtime EngineLab::Simulation EngineLab::Foundation EngineLab::Physics
    EngineLab::Ecu EngineLab::Events EngineLab::Exhaust EngineLab::Catalog
    EngineLab::Diagnostics)
target_compile_definitions(EngineLabRealtimeBudgetHarness PRIVATE ENGINELAB_CATALOG_ROOT="${CMAKE_SOURCE_DIR}")
enginelab_set_warnings(EngineLabRealtimeBudgetHarness)

add_executable(EngineLabAbClipRenderer AbClipRenderer.cpp)
target_link_libraries(EngineLabAbClipRenderer PRIVATE
    EngineLab::Audio EngineLab::Simulation EngineLab::Foundation EngineLab::Physics
    EngineLab::Ecu EngineLab::Events EngineLab::Exhaust EngineLab::Catalog
    juce::juce_cryptography nlohmann_json::nlohmann_json)
target_compile_definitions(EngineLabAbClipRenderer PRIVATE ENGINELAB_CATALOG_ROOT="${CMAKE_SOURCE_DIR}")
if(MSVC)
    target_link_options(EngineLabAbClipRenderer PRIVATE /STACK:8388608)
endif()
enginelab_set_warnings(EngineLabAbClipRenderer)

add_executable(EngineLabAudioAbHarness AudioAbHarness.cpp)
target_link_libraries(EngineLabAudioAbHarness PRIVATE
    EngineLab::Audio EngineLab::Simulation EngineLab::Foundation EngineLab::Physics
    EngineLab::Ecu EngineLab::Events EngineLab::Exhaust EngineLab::Catalog)
target_compile_definitions(EngineLabAudioAbHarness PRIVATE ENGINELAB_CATALOG_ROOT="${CMAKE_SOURCE_DIR}")
if(MSVC)
    target_link_options(EngineLabAudioAbHarness PRIVATE /STACK:8388608)
endif()
enginelab_set_warnings(EngineLabAudioAbHarness)
