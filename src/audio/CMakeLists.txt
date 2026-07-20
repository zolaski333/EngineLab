add_library(EngineLabAudio STATIC
    src/PipeRadiationModel.cpp
    src/RealtimeEngineAudio.cpp
    src/RealtimeConvolutionBank.cpp)
add_library(EngineLab::Audio ALIAS EngineLabAudio)
target_include_directories(EngineLabAudio PUBLIC include)
target_link_libraries(EngineLabAudio PUBLIC EngineLab::Runtime juce::juce_audio_basics juce::juce_audio_devices juce::juce_dsp)
target_compile_definitions(EngineLabAudio PUBLIC JUCE_WEB_BROWSER=0 JUCE_USE_CURL=0)
enginelab_set_warnings(EngineLabAudio)
