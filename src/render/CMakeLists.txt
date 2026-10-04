add_library(EngineLabRender STATIC
    src/RenderSnapshot.cpp src/Mesh.cpp src/EngineModel3D.cpp src/CrankClock.cpp)
add_library(EngineLab::Render ALIAS EngineLabRender)
target_include_directories(EngineLabRender PUBLIC include)
target_link_libraries(EngineLabRender PUBLIC EngineLab::Foundation EngineLab::Physics)
enginelab_set_warnings(EngineLabRender)
