add_library(EngineLabGasDynamics STATIC
    src/FiniteVolumeDuct.cpp
    src/ExhaustNetworkLayout.cpp
    src/ExhaustGasNetwork.cpp)
add_library(EngineLab::GasDynamics ALIAS EngineLabGasDynamics)
target_include_directories(EngineLabGasDynamics PUBLIC include)
target_link_libraries(EngineLabGasDynamics PUBLIC EngineLab::Foundation EngineLab::Exhaust)
enginelab_set_warnings(EngineLabGasDynamics)
