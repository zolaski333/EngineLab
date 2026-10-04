add_library(EngineLabRender STATIC
    src/RenderSnapshot.cpp src/Mesh.cpp src/EngineModel3D.cpp src/DuctLayout3D.cpp src/CrankClock.cpp
    src/GasFieldView.cpp)
add_library(EngineLab::Render ALIAS EngineLabRender)
target_include_directories(EngineLabRender PUBLIC include)
target_link_libraries(EngineLabRender PUBLIC EngineLab::Foundation EngineLab::Physics PRIVATE EngineLab::Exhaust)
enginelab_set_warnings(EngineLabRender)
