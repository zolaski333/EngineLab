add_executable(EngineLabComparisonHarness ComparisonHarness.cpp)
target_link_libraries(EngineLabComparisonHarness PRIVATE
    EngineLab::Simulation EngineLab::Foundation EngineLab::Physics
    EngineLab::Ecu EngineLab::Events EngineLab::Exhaust EngineLab::Catalog)
enginelab_set_warnings(EngineLabComparisonHarness)
