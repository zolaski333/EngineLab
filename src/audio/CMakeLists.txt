add_library(EngineLabAudio STATIC
    src/AcousticExhaustNetwork.cpp
    src/AcousticIntakeNetwork.cpp
    src/ExhaustJetNoise.cpp
    src/ForcedInductionAcoustics.cpp
    src/FreeFieldObserver.cpp
    src/ImpulseResponseLoader.cpp
    src/OfflineAudioExporter.cpp
    src/PipeRadiationModel.cpp
    src/RealtimeEngineAudio.cpp
    src/RealtimeConvolutionBank.cpp
    src/StructuralModalRadiator.cpp)
add_library(EngineLab::Audio ALIAS EngineLabAudio)
target_include_directories(EngineLabAudio PUBLIC include)
target_link_libraries(EngineLabAudio PUBLIC EngineLab::Runtime
    juce::juce_audio_basics juce::juce_audio_devices juce::juce_audio_formats
    juce::juce_dsp
    PRIVATE nlohmann_json::nlohmann_json)
target_compile_definitions(EngineLabAudio PUBLIC JUCE_WEB_BROWSER=0 JUCE_USE_CURL=0)
enginelab_set_warnings(EngineLabAudio)
