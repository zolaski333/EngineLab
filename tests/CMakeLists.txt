add_executable(EngineLabCoreTests CoreTests.cpp)
target_link_libraries(EngineLabCoreTests PRIVATE
    EngineLab::Foundation EngineLab::Events EngineLab::Physics EngineLab::Ecu
    EngineLab::Exhaust EngineLab::Simulation EngineLab::Serialization EngineLab::Runtime)
target_link_libraries(EngineLabCoreTests PRIVATE EngineLab::Audio)
enginelab_set_warnings(EngineLabCoreTests)
add_test(NAME EngineLab.Core COMMAND EngineLabCoreTests)
if(MSVC AND ENGINELAB_ENABLE_SANITIZERS)
    target_link_options(EngineLabCoreTests PRIVATE /STACK:8388608)
    get_filename_component(ENGINELAB_MSVC_RUNTIME_DIR "${CMAKE_CXX_COMPILER}" DIRECTORY)
    set_tests_properties(EngineLab.Core PROPERTIES
        ENVIRONMENT_MODIFICATION "PATH=path_list_prepend:${ENGINELAB_MSVC_RUNTIME_DIR}")
endif()
