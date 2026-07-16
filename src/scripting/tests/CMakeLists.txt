add_executable(EngineLabScriptingTests EngineScriptTests.cpp)
target_link_libraries(EngineLabScriptingTests PRIVATE
    EngineLab::Scripting EngineLab::Serialization)
enginelab_set_warnings(EngineLabScriptingTests)
add_test(NAME EngineLab.Scripting COMMAND EngineLabScriptingTests)

