#include <enginelab/audio/OfflineAudioExporter.hpp>
#include <enginelab/catalog/EngineCatalog.hpp>

#include <nlohmann/json.hpp>

#include <array>
#include <bit>
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

[[nodiscard]] std::vector<double> readPcm24(
    const std::filesystem::path& file) {
    std::ifstream input(file, std::ios::binary);
    input.seekg(44);
    std::vector<double> samples;
    std::array<unsigned char, 3> bytes {};
    while (input.read(reinterpret_cast<char*>(bytes.data()), 3)) {
        auto value = static_cast<std::int32_t>(bytes[0])
            | (static_cast<std::int32_t>(bytes[1]) << 8)
            | (static_cast<std::int32_t>(bytes[2]) << 16);
        if ((value & 0x00800000) != 0) value |= ~0x00ffffff;
        samples.push_back(static_cast<double>(value) / 8'388'608.0);
    }
    return samples;
}

[[nodiscard]] std::vector<double> readFloat32(
    const std::filesystem::path& file) {
    std::ifstream input(file, std::ios::binary);
    input.seekg(44);
    std::vector<double> samples;
    std::array<unsigned char, 4> bytes {};
    while (input.read(reinterpret_cast<char*>(bytes.data()), 4)) {
        const auto bits = static_cast<std::uint32_t>(bytes[0])
            | (static_cast<std::uint32_t>(bytes[1]) << 8U)
            | (static_cast<std::uint32_t>(bytes[2]) << 16U)
            | (static_cast<std::uint32_t>(bytes[3]) << 24U);
        samples.push_back(static_cast<double>(std::bit_cast<float>(bits)));
    }
    return samples;
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

[[nodiscard]] enginelab::OfflineAudioScenario audioPhysicsScenario() {
    return {
        "audio-physics-ab",
        {
            { "starter", 0.8, true, true, false,
              0.35, 0.35, 0.0, 0.0, 0.0, 0.0 },
            { "free-run", 2.7, true, false, false,
              0.45, 1.0, 0.0, 0.0, 0.0, 0.0 },
            { "limiter", 1.4, true, false, false,
              1.0, 1.0, 0.0, 0.0, 0.0, 0.0 },
            { "overrun", 1.5, true, false, false,
              0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },
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
    require(pcm.files.size() == 14,
        "master, diagnostic WAVs, order map and metadata written");
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
             "intake", "forced_induction", "mechanical",
             "exhaust_pressure_wave", "exhaust_jet" }) {
        verifyWave(
            pcmRequest.outputDirectory
                / (std::string("stem_") + stem + ".wav"),
            1, 24, 96'000, pcm.renderedFrames);
    }
    verifyWave(pcmRequest.outputDirectory / "premaster.wav",
        1, 24, 96'000, pcm.renderedFrames);
    verifyWave(pcmRequest.outputDirectory / "master_processing_delta.wav",
        1, 24, 96'000, pcm.renderedFrames);

    const auto premaster = readPcm24(
        pcmRequest.outputDirectory / "premaster.wav");
    const auto master = readPcm24(
        pcmRequest.outputDirectory / "master.wav");
    const auto delta = readPcm24(
        pcmRequest.outputDirectory / "master_processing_delta.wav");
    std::vector<std::vector<double>> stems;
    for (const auto* stem : {
             "combustion", "exhaust_dry", "exhaust_ir",
             "intake", "forced_induction", "mechanical" }) {
        stems.push_back(readPcm24(pcmRequest.outputDirectory
            / (std::string("stem_") + stem + ".wav")));
    }
    require(premaster.size() == master.size() && master.size() == delta.size(),
        "diagnostic WAVs have matching sample counts");
    double quantisedStemError = 0.0;
    double quantisedMasterError = 0.0;
    for (std::size_t sample = 0; sample < master.size(); ++sample) {
        double sum = 0.0;
        for (const auto& stem : stems) sum += stem[sample];
        quantisedStemError = std::max(quantisedStemError,
            std::abs(sum - premaster[sample]));
        quantisedMasterError = std::max(quantisedMasterError,
            std::abs(premaster[sample] + delta[sample] - master[sample]));
    }
    require(quantisedStemError <= 1.0e-6
            && quantisedMasterError <= 3.0e-7,
        "PCM24 stems and processing delta reconstruct within quantisation error");
    std::cout << "Exhaust diagnostic decomposition: float_error="
              << pcm.exhaustDiagnosticDecompositionMaxError
              << ", PCM24_stem_error=" << quantisedStemError << '\n';
    {
        std::ifstream orderMap(
            pcmRequest.outputDirectory / "engine-order-map.csv");
        std::string header;
        std::string firstRow;
        std::getline(orderMap, header);
        std::getline(orderMap, firstRow);
        require(header == "time_s,rpm,order,frequency_hz,level_dbfs"
                && !firstRow.empty(),
            "engine order map contains a machine-readable header and data");
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
            manifest.at("schema_version") == 3
                && manifest.at("stems_written") == true
                && manifest.at("files").size() == 12,
            "manifest inventories master, stems and order map");
        require(
            manifest.at("conditions").at("monitor_mode")
                    == "physical_reference"
                && manifest.at("conditions").contains("observer")
                && manifest.at("exhaust_topology").at("node_count")
                    .get<std::size_t>() > 0
                && !manifest.at("exhaust_topology").at("fnv1a64")
                    .get<std::string>().empty()
                && manifest.at("reference_audio").at("supplied") == false
                && manifest.at("reference_audio").at("provenance")
                    == "none_supplied",
            "manifest must pin listening conditions, topology and reference provenance");
        require(
            manifest.contains("audio_physics")
                && manifest.at("audio_physics").at("cycle_variation_samples") == 0
                && manifest.at("audio_physics").at("afterfire_fuel_burned_mg") == 0.0
                && manifest.at("audio_physics").contains("authored_overrun_pulse_hz")
                && manifest.at("audio_physics").contains("authored_overrun_pulse_duty")
                && manifest.at("audio_physics").at("authored_overrun_pulse_timing_variation") == 0.0,
            "manifest exposes an exact bypass for the default deterministic engine");
        require(
            manifest.at("path_diagnostics")
                    .at("saturation_processed_samples") == 0
                && manifest.at("path_diagnostics")
                    .at("dropped_exhaust_acoustic_samples") == 0
                && manifest.at("path_diagnostics")
                    .at("dropped_reaction_events") == 0,
            "physical-reference export must prove neutral saturation and lossless acoustic transport");
        require(
            manifest.at("stem_reconstruction")
                    .at("stem_sum_to_premaster_max_abs_error") == 0.0
                && manifest.at("stem_reconstruction")
                    .at("premaster_plus_delta_to_master_max_abs_error")
                    .get<double>() <= 1.0e-7,
            "manifest proves float-domain diagnostic reconstruction");
        require(
            manifest.at("exhaust_dry_decomposition")
                    .at("sum_to_exhaust_dry_max_abs_error")
                    .get<double>() <= 1.0e-6,
            "manifest proves pressure-wave plus jet reconstructs dry exhaust");
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

    const enginelab::EngineConfig* audioLab = nullptr;
    for (const auto& entry : catalog.entries) {
        if (entry.config.name == "Audio Physics Lab 689 Twin") {
            audioLab = &entry.config;
            break;
        }
    }
    require(audioLab != nullptr, "audio physics lab fixture is present");
    auto bypassEngine = *audioLab;
    bypassEngine.combustionCalibration.cycleVariationCoefficientOfVariation = 0.0;
    bypassEngine.exhaustAfterfire.strategy =
        enginelab::ExhaustAfterfireStrategy::cleanDfco;
    bypassEngine.exhaustAfterfire.enabled = false;
    bypassEngine.exhaustAfterfire.overrunFuelFraction = 0.0;
    bypassEngine.ignition.limiterKeepsFuel = false;
    bypassEngine.exhaust.mufflerPackingFlowResistivityPaSPerM2 = 0.0;
    bypassEngine.exhaust.mufflerPackingThicknessMm = 0.0;
    bypassEngine.exhaust.mufflerPerforatedOpenAreaRatio = 0.0;
    for (auto& path : bypassEngine.exhaustPaths) {
        path.geometry.mufflerPackingFlowResistivityPaSPerM2 = 0.0;
        path.geometry.mufflerPackingThicknessMm = 0.0;
        path.geometry.mufflerPerforatedOpenAreaRatio = 0.0;
        if (path.network) {
            for (auto& component : path.network->components) {
                component.packingFlowResistivityPaSPerM2 = 0.0;
                component.packingThicknessMm = 0.0;
                component.perforatedOpenAreaRatio = 0.0;
            }
        }
    }
    enginelab::OfflineAudioExportRequest bypassRequest;
    bypassRequest.engine = std::move(bypassEngine);
    bypassRequest.scenario = audioPhysicsScenario();
    bypassRequest.outputDirectory = temporary.path() / "physics-bypass";
    bypassRequest.assetRoot = ENGINELAB_CATALOG_ROOT;
    bypassRequest.sampleRateHz = 48'000;
    bypassRequest.format = enginelab::OfflineWaveFormat::float32;
    bypassRequest.writeStems = false;
    bypassRequest.loadAuthoredImpulseResponses = false;
    const auto bypass = enginelab::exportOfflineAudio(bypassRequest);
    require(bypass.success, bypass.error);

    auto demoRequest = bypassRequest;
    demoRequest.engine = *audioLab;
    demoRequest.outputDirectory = temporary.path() / "physics-demo";
    const auto demo = enginelab::exportOfflineAudio(demoRequest);
    require(demo.success, demo.error);
    require(
        demo.cycleVariationSamples > 0
            && demo.cycleMultiplierMinimum < 0.99
            && demo.cycleMultiplierMaximum > 1.01,
        "demo render measures non-neutral physical cycle variation");
    std::cout << "Audio physics raw proof: afterfire="
              << demo.afterfirePeakHeatReleaseKw << " kW / "
              << demo.afterfireFuelBurnedMg << " mg, overrun_frames="
              << demo.overrunAfterfireActiveFrames << '\n';
    require(demo.overrunAfterfireActiveFrames > 0,
        "demo render must enter the ECU-conditioned overrun strategy");
    require(demo.afterfirePeakHeatReleaseKw > 0.01,
        "demo render must measure exhaust chemical heat release");
    require(demo.afterfireFuelBurnedMg > 0.01,
        "demo render must consume real exhaust fuel inventory");
    require(demo.porousMufflerCount == 1 && bypass.porousMufflerCount == 0,
            "demo/bypass pair isolates one authored porous silencer");
    const auto bypassSamples = readFloat32(
        bypassRequest.outputDirectory / "master.wav");
    const auto demoSamples = readFloat32(
        demoRequest.outputDirectory / "master.wav");
    require(bypassSamples.size() == demoSamples.size() && !demoSamples.empty(),
            "A/B WAV files have matching non-empty sample counts");
    long double deltaSquareSum = 0.0;
    double deltaPeak = 0.0;
    for (std::size_t sample = 0; sample < demoSamples.size(); ++sample) {
        const auto sampleDelta = demoSamples[sample] - bypassSamples[sample];
        deltaSquareSum += static_cast<long double>(sampleDelta) * sampleDelta;
        deltaPeak = std::max(deltaPeak, std::abs(sampleDelta));
    }
    const auto deltaRms = std::sqrt(static_cast<double>(
        deltaSquareSum / static_cast<long double>(demoSamples.size())));
    require(deltaRms > 1.0e-5 && deltaPeak > 1.0e-4,
            "physical demo produces a measurable non-identical WAV");
    std::cout << "Audio physics A/B: delta RMS=" << deltaRms
              << ", peak=" << deltaPeak
              << ", cycle=" << demo.cycleMultiplierMinimum << ".."
              << demo.cycleMultiplierMaximum
              << ", afterfire=" << demo.afterfirePeakHeatReleaseKw
              << " kW / " << demo.afterfireFuelBurnedMg << " mg"
              << ", overrun_frames=" << demo.overrunAfterfireActiveFrames
              << '\n';

    std::cout
        << "Offline audio export: PCM24 stems, float32 192 kHz, "
           "JSON scenario, manifest, physical A/B and cancellation PASS\n";
    return 0;
}
