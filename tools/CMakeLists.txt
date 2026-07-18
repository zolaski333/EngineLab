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

add_executable(EngineLabAudioRenderHarness AudioRenderHarness.cpp)
target_link_libraries(EngineLabAudioRenderHarness PRIVATE
    EngineLab::Audio EngineLab::Simulation EngineLab::Foundation EngineLab::Physics
    EngineLab::Ecu EngineLab::Events EngineLab::Exhaust EngineLab::Catalog)
target_compile_definitions(EngineLabAudioRenderHarness PRIVATE ENGINELAB_CATALOG_ROOT="${CMAKE_SOURCE_DIR}")
if(MSVC)
    target_link_options(EngineLabAudioRenderHarness PRIVATE /STACK:8388608)
endif()
enginelab_set_warnings(EngineLabAudioRenderHarness)

add_executable(EngineLabAbClipRenderer AbClipRenderer.cpp)
target_link_libraries(EngineLabAbClipRenderer PRIVATE
    EngineLab::Audio EngineLab::Simulation EngineLab::Foundation EngineLab::Physics
    EngineLab::Ecu EngineLab::Events EngineLab::Exhaust EngineLab::Catalog)
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
