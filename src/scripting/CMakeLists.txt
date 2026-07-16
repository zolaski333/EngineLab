find_package(Threads REQUIRED)
add_library(EngineLabScripting STATIC
    src/EngineScriptCompiler.cpp
    src/EngineScriptHotReloader.cpp)
add_library(EngineLab::Scripting ALIAS EngineLabScripting)
target_include_directories(EngineLabScripting PUBLIC include)
target_link_libraries(EngineLabScripting PUBLIC
    EngineLab::Foundation EngineLab::Serialization Threads::Threads)
enginelab_set_warnings(EngineLabScripting)
