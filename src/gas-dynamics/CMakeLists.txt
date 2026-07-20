add_library(EngineLabGasDynamics STATIC
    src/FiniteVolumeDuct.cpp)
add_library(EngineLab::GasDynamics ALIAS EngineLabGasDynamics)
target_include_directories(EngineLabGasDynamics PUBLIC include)
target_link_libraries(EngineLabGasDynamics PUBLIC EngineLab::Foundation)
enginelab_set_warnings(EngineLabGasDynamics)

