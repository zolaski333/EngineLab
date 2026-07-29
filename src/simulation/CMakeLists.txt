add_library(EngineLabSimulation STATIC
    src/DynoAbsorberController.cpp
    src/EngineSimulator.cpp)
add_library(EngineLab::Simulation ALIAS EngineLabSimulation)
target_include_directories(EngineLabSimulation PUBLIC include)
target_link_libraries(EngineLabSimulation PUBLIC
    EngineLab::Foundation EngineLab::Events EngineLab::Physics EngineLab::Ecu
    EngineLab::Exhaust EngineLab::GasDynamics)
enginelab_set_warnings(EngineLabSimulation)
