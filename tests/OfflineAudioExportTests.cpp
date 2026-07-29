#include <enginelab/audio/OfflineAudioExporter.hpp>
#include <enginelab/catalog/EngineCatalog.hpp>

#include <nlohmann/json.hpp>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string_view>
#include <vector>

namespace {

void require(bool condition, std::string_view message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(1);
    }
}

class TemporaryDirectory final {
public:
    TemporaryDirectory() {
        const auto stamp =
            std::chrono::high_resolution_clock::now()
                .time_since_epoch().count();
        path_ = std::filesystem::temp_directory_path()
            / ("enginelab-offline-audio-"
               + std::to_string(stamp));
        std::filesystem::create_directories(path_);
    }

    ~TemporaryDirectory() {
        std::error_code ignored;
        std::filesystem::remove_all(path_, ignored);
    }

    [[nodiscard]] const std::filesystem::path& path() const noexcept {
        return path_;
    }

private:
    std::filesystem::path path_;
};

[[nodiscard]] std::uint16_t read16(
    const std::vector<unsigned char>& header,
    std::size_t offset) {
    return static_cast<std::uint16_t>(
        static_cast<std::uint16_t>(header[offset])
        | static_cast<std::uint16_t>(
              header[offset + 1] << 8U));
}

[[nodiscard]] std::uint32_t read32(
    const std::vector<unsigned char>& header,
    std::size_t offset) {
    return static_cast<std::uint32_t>(header[offset])
        | (static_cast<std::uint32_t>(header[offset + 1]) << 8U)
        | (static_cast<std::uint32_t>(header[offset + 2]) << 16U)
        | (static_cast<std::uint32_t>(header[offset + 3]) << 24U);
}

void verifyWave(
    const std::filesystem::path& file,
    std::uint16_t expectedFormat,
    std::uint16_t expectedBits,
    std::uint32_t expectedSampleRate,
    std::uint64_t expectedFrames) {
    std::ifstream input(file, std::ios::binary);
    require(static_cast<bool>(input), "WAV file exists");
    std::vector<unsigned char> header(44);
    input.read(
        reinterpret_cast<char*>(header.data()),
        static_cast<std::streamsize>(header.size()));
    require(input.gcount() == 44, "WAV has a complete canonical header");
    require(std::string(
                reinterpret_cast<const char*>(header.data()), 4)
            == "RIFF",
        "WAV RIFF signature");
    require(std::string(
                reinterpret_cast<const char*>(header.data() + 8), 4)
            == "WAVE",
        "WAV WAVE signature");
    require(read16(header, 20) == expectedFormat, "WAV format code");
    require(read16(header, 22) == 2, "WAV stereo channel count");
    require(
        read32(header, 24) == expectedSampleRate,
        "WAV sample rate");
    require(read16(header, 34) == expectedBits, "WAV bit depth");
    const auto expectedDataBytes = expectedFrames * 2ULL
        * static_cast<std::uint64_t>(expectedBits / 8U);
    require(
        read32(header, 40) == expectedDataBytes,
        "WAV data-byte count");
    require(
        std::filesystem::file_size(file)
            == expectedDataBytes + 44ULL,
        "WAV file size matches header");
}

[[nodiscard]] enginelab::OfflineAudioScenario shortScenario() {
    return {
        "format-smoke",
        {
            { "starter", 0.10, true, true, false,
              0.20, 0.20, 0.0, 0.0, 0.0, 0.0 },
        }
    };
}

} // namespace

int main() {
    TemporaryDirectory temporary;
    const auto catalog =
        enginelab::loadEngineCatalog(ENGINELAB_CATALOG_ROOT);
    require(!catalog.entries.empty(), "catalog loads");
    const enginelab::EngineConfig* engine = nullptr;
    for (const auto& entry : catalog.entries) {
        if (entry.config.name.find("K20") != std::string::npos) {
            engine = &entry.config;
            break;
        }
    }
    require(engine != nullptr, "K20 fixture is present in catalog");

    const auto scenarioFile = temporary.path() / "roundtrip.json";
    auto scenario = shortScenario();
    std::string scenarioError;
    require(
        enginelab::saveOfflineAudioScenario(
            scenarioFile, scenario, scenarioError),
        "scenario JSON writes");
    enginelab::OfflineAudioScenario decoded;
    require(
        enginelab::loadOfflineAudioScenario(
            scenarioFile, decoded, scenarioError),
        "scenario JSON reads");
    require(
        decoded.name == scenario.name
            && decoded.stages.size() == 1
            && decoded.stages.front().starterEngaged,
        "scenario JSON round-trips");

    enginelab::OfflineAudioExportRequest pcmRequest;
    pcmRequest.engine = *engine;
    pcmRequest.scenario = scenario;
    pcmRequest.outputDirectory = temporary.path() / "pcm24";
    pcmRequest.assetRoot = ENGINELAB_CATALOG_ROOT;
    pcmRequest.sampleRateHz = 96'000;
    pcmRequest.format = enginelab::OfflineWaveFormat::pcm24;
    pcmRequest.writeStems = true;
    pcmRequest.loadAuthoredImpulseResponses = false;
    const auto pcm = enginelab::exportOfflineAudio(pcmRequest);
    require(pcm.success, pcm.error);
    require(pcm.files.size() == 9, "master, six stems and metadata written");
    require(
        pcm.renderedFrames == 9'600,
        "96 kHz render has exact scenario frame count");
    require(
        pcm.compiledExhaustTopologyActive,
        "offline path compiled the physical exhaust graph");
    require(
        pcm.delayTruncationCount == 0
            && pcm.droppedFiringEvents == 0
            && pcm.droppedPressureSamples == 0,
        "offline render has no truncation or dropped telemetry");
    verifyWave(
        pcmRequest.outputDirectory / "master.wav",
        1, 24, 96'000, pcm.renderedFrames);
    for (const auto* stem : {
             "combustion", "exhaust_dry", "exhaust_ir",
             "intake", "forced_induction", "mechanical" }) {
        verifyWave(
            pcmRequest.outputDirectory
                / (std::string("stem_") + stem + ".wav"),
            1, 24, 96'000, pcm.renderedFrames);
    }

    {
        std::ifstream manifestFile(
            pcmRequest.outputDirectory / "render-manifest.json");
        const auto manifest = nlohmann::json::parse(manifestFile);
        require(
            manifest.at("render_path")
                == "EngineSimulator -> publishAudioFrame -> RealtimeEngineAudio",
            "manifest identifies the shipping render path");
        require(
            manifest.at("sample_rate_hz") == 96'000
                && manifest.at("bits_per_sample") == 24
                && manifest.at("wave_format_code") == 1,
            "manifest records PCM24 format truthfully");
        require(
            manifest.at("stems_written") == true
                && manifest.at("files").size() == 7,
            "manifest inventories master and stems");
    }

    enginelab::OfflineAudioExportRequest floatRequest = pcmRequest;
    floatRequest.outputDirectory = temporary.path() / "float32";
    floatRequest.sampleRateHz = 192'000;
    floatRequest.format = enginelab::OfflineWaveFormat::float32;
    floatRequest.writeStems = false;
    const auto floating =
        enginelab::exportOfflineAudio(floatRequest);
    require(floating.success, floating.error);
    require(floating.files.size() == 3, "float export writes master and metadata");
    require(
        floating.renderedFrames == 19'200,
        "192 kHz render has exact scenario frame count");
    verifyWave(
        floatRequest.outputDirectory / "master.wav",
        3, 32, 192'000, floating.renderedFrames);

    auto invalidRequest = floatRequest;
    invalidRequest.outputDirectory = temporary.path() / "invalid";
    invalidRequest.sampleRateHz = 44'100;
    const auto invalid =
        enginelab::exportOfflineAudio(invalidRequest);
    require(
        !invalid.success
            && invalid.error.find("48000") != std::string::npos,
        "unsupported sample rate is rejected explicitly");

    auto cancelledRequest = floatRequest;
    cancelledRequest.outputDirectory = temporary.path() / "cancelled";
    const auto cancelled = enginelab::exportOfflineAudio(
        cancelledRequest,
        [](double, std::string_view) { return false; });
    require(
        cancelled.cancelled && !cancelled.success,
        "progress callback can cancel before rendering");
    require(
        !std::filesystem::exists(
            cancelledRequest.outputDirectory / "master.wav"),
        "cancelled export leaves no final WAV");

    std::cout
        << "Offline audio export: PCM24 stems, float32 192 kHz, "
           "JSON scenario, manifest and cancellation PASS\n";
    return 0;
}
