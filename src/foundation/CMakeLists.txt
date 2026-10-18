add_library(EngineLabFoundation STATIC src/EngineTypes.cpp src/CylinderResize.cpp src/IntakeResize.cpp
    src/TurboSizing.cpp src/Units.cpp src/EngineBuilder.cpp src/CrankBalance.cpp)
add_library(EngineLab::Foundation ALIAS EngineLabFoundation)
target_include_directories(EngineLabFoundation PUBLIC include)
target_compile_features(EngineLabFoundation PUBLIC cxx_std_20)
enginelab_set_warnings(EngineLabFoundation)
