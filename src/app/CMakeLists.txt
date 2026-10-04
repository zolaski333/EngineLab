juce_add_gui_app(EngineLabApp
    PRODUCT_NAME "EngineLab"
    COMPANY_NAME "EngineLab"
    BUNDLE_ID "com.enginelab.desktop"
    VERSION "${PROJECT_VERSION}")
juce_generate_juce_header(EngineLabApp)
# Inter and JetBrains Mono (SIL OFL 1.1) are compiled into the executable so the
# interface looks the same on every machine.
juce_add_binary_data(EngineLabFonts
    HEADER_NAME EngineLabFonts.h
    NAMESPACE EngineLabFonts
    SOURCES
        resources/fonts/Inter-Regular.ttf resources/fonts/Inter-SemiBold.ttf
        resources/fonts/Inter-Bold.ttf
        resources/fonts/JetBrainsMono-Medium.ttf resources/fonts/JetBrainsMono-Bold.ttf)
target_sources(EngineLabApp PRIVATE
    src/Main.cpp src/MainComponent.cpp src/ActionMap.cpp src/AudioWorkshopWindow.cpp src/EcuTunerWindow.cpp
    src/ExhaustDesignerWindow.cpp
    src/Theme.cpp src/Widgets.cpp src/DashboardModel.cpp src/TopBar.cpp src/ControlPanel.cpp
    src/EngineViewport.cpp src/ReadoutStrip.cpp src/DynoPanel.cpp src/SidePanel.cpp src/StatusBar.cpp
    include/enginelab/app/MainComponent.hpp include/enginelab/app/ActionMap.hpp
    include/enginelab/app/AudioWorkshopWindow.hpp
    include/enginelab/app/EcuTunerWindow.hpp
    include/enginelab/app/ExhaustDesignerWindow.hpp
    include/enginelab/app/Theme.hpp include/enginelab/app/Widgets.hpp
    include/enginelab/app/DashboardModel.hpp include/enginelab/app/TopBar.hpp
    include/enginelab/app/ControlPanel.hpp include/enginelab/app/EngineViewport.hpp
    include/enginelab/app/ReadoutStrip.hpp include/enginelab/app/DynoPanel.hpp
    include/enginelab/app/SidePanel.hpp include/enginelab/app/StatusBar.hpp)
target_include_directories(EngineLabApp PRIVATE include)
target_link_libraries(EngineLabApp PRIVATE
    EngineLab::Audio EngineLab::Runtime EngineLab::Diagnostics EngineLab::Serialization EngineLab::Catalog
    EngineLab::Calibration EngineLab::Render EngineLab::Scripting EngineLabFonts
    juce::juce_audio_utils juce::juce_gui_extra
    PUBLIC juce::juce_recommended_config_flags juce::juce_recommended_lto_flags)
target_compile_definitions(EngineLabApp PRIVATE
    JUCE_WEB_BROWSER=0
    JUCE_USE_CURL=0
    ENGINELAB_CATALOG_ROOT="${CMAKE_SOURCE_DIR}")
enginelab_set_warnings(EngineLabApp)
