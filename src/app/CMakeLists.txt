juce_add_gui_app(EngineLabApp
    PRODUCT_NAME "EngineLab"
    COMPANY_NAME "EngineLab"
    BUNDLE_ID "com.enginelab.desktop")
juce_generate_juce_header(EngineLabApp)
target_sources(EngineLabApp PRIVATE
    src/Main.cpp src/MainComponent.cpp src/ActionMap.cpp src/EcuTunerWindow.cpp
    src/ExhaustDesignerWindow.cpp
    include/enginelab/app/MainComponent.hpp include/enginelab/app/ActionMap.hpp
    include/enginelab/app/EcuTunerWindow.hpp
    include/enginelab/app/ExhaustDesignerWindow.hpp)
target_include_directories(EngineLabApp PRIVATE include)
target_link_libraries(EngineLabApp PRIVATE
    EngineLab::Audio EngineLab::Runtime EngineLab::Diagnostics EngineLab::Serialization EngineLab::Catalog
    EngineLab::Calibration EngineLab::Render EngineLab::Scripting
    juce::juce_audio_utils juce::juce_gui_extra
    PUBLIC juce::juce_recommended_config_flags juce::juce_recommended_lto_flags)
target_compile_definitions(EngineLabApp PRIVATE
    JUCE_WEB_BROWSER=0
    JUCE_USE_CURL=0
    ENGINELAB_CATALOG_ROOT="${CMAKE_SOURCE_DIR}")
enginelab_set_warnings(EngineLabApp)
