add_library(EngineLabCalibration STATIC
    src/CalibrationJson.cpp
    src/CalibrationFileHotReloader.cpp
    src/CalibrationStore.cpp
    src/CalibrationTypes.cpp
    src/EcuCalibration.cpp)
add_library(EngineLab::Calibration ALIAS EngineLabCalibration)
target_include_directories(EngineLabCalibration PUBLIC include)
target_link_libraries(EngineLabCalibration PUBLIC EngineLab::Foundation nlohmann_json::nlohmann_json)
enginelab_set_warnings(EngineLabCalibration)
