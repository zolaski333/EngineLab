// A loop pack for a game's sample-based engine sound: the points are held on
// and off load, each loop spans whole engine cycles and runs seamlessly from
// its last sample into its first, and one gain serves the whole pack.
#include <enginelab/audio/LoopPackExporter.hpp>
#include <enginelab/catalog/EngineCatalog.hpp>

#include <juce_audio_formats/juce_audio_formats.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
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
        const auto stamp = std::chrono::high_resolution_clock::now()
            .time_since_epoch().count();
        path_ = std::filesystem::temp_directory_path()
            / ("enginelab-loop-pack-" + std::to_string(stamp));
        std::filesystem::create_directories(path_);
    }
    ~TemporaryDirectory() {
        std::error_code ignored;
        std::filesystem::remove_all(path_, ignored);
    }
    [[nodiscard]] const std::filesystem::path& path() const noexcept { return path_; }

private:
    std::filesystem::path path_;
};

struct DecodedLoop final {
    std::vector<float> samples;
    double sampleRate { 0.0 };
    unsigned int channels { 0 };
    unsigned int bits { 0 };
    juce::StringPairArray metadata;
};

[[nodiscard]] DecodedLoop decode(juce::AudioFormat& format,
                                 const std::filesystem::path& file) {
    DecodedLoop decoded;
    const auto encoded = file.u8string();
    auto stream = juce::File(juce::String::fromUTF8(
        reinterpret_cast<const char*>(encoded.data()),
        static_cast<int>(encoded.size()))).createInputStream();
    require(stream != nullptr, "a loop file opens");
    std::unique_ptr<juce::AudioFormatReader> reader(
        format.createReaderFor(stream.release(), true));
    require(reader != nullptr, "a loop file decodes");
    decoded.sampleRate = reader->sampleRate;
    decoded.channels = reader->numChannels;
    decoded.bits = reader->bitsPerSample;
    decoded.metadata = reader->metadataValues;
    juce::AudioBuffer<float> buffer(
        static_cast<int>(reader->numChannels),
        static_cast<int>(reader->lengthInSamples));
    reader->read(&buffer, 0, static_cast<int>(reader->lengthInSamples), 0, true, true);
    decoded.samples.assign(buffer.getReadPointer(0),
        buffer.getReadPointer(0) + buffer.getNumSamples());
    return decoded;
}

} // namespace

int main() {
    const auto catalog = enginelab::loadEngineCatalog(ENGINELAB_CATALOG_ROOT);
    const enginelab::EngineConfig* engine = nullptr;
    for (const auto& entry : catalog.entries)
        if (entry.config.name.find("K20") != std::string::npos) {
            engine = &entry.config;
            break;
        }
    require(engine != nullptr, "K20 fixture is present in catalog");

    // The points: the idle, then every step from a step above it, and the top.
    const auto points = enginelab::loopPackPoints(*engine, 1'000.0, 0.0, 3'000.0);
    require(points.size() == 3 && points.front() == engine->idleRpm
            && points[1] == 2'000.0 && points[2] == 3'000.0,
        "points are the idle, 2000 and 3000 rpm");
    const auto defaultPoints = enginelab::loopPackPoints(*engine, 500.0);
    require(defaultPoints.back() <= 0.95 * std::min(engine->redlineRpm,
                                                    engine->ignition.revLimitRpm) + 1.0e-9
            && defaultPoints.back() > defaultPoints[defaultPoints.size() - 2],
        "the default top point stays under the redline and the rev limit");
    require(enginelab::equivalentFourStrokeCylinders(*engine) == 4,
        "a four-cylinder four-stroke declares four cylinders");
    {
        auto twoStroke = *engine;
        twoStroke.cycle = enginelab::EngineCycle::twoStroke;
        auto rotary = *engine;
        rotary.cycle = enginelab::EngineCycle::rotary;
        rotary.rotary.rotorCount = 2;
        require(enginelab::equivalentFourStrokeCylinders(twoStroke) == 8
                && enginelab::equivalentFourStrokeCylinders(rotary) == 4,
            "a two-stroke declares twice its cylinders, a rotary two per rotor");
    }

    TemporaryDirectory temporary;
    enginelab::LoopPackRequest request;
    request.engine = *engine;
    request.outputDirectory = temporary.path() / "pack";
    request.assetRoot = ENGINELAB_CATALOG_ROOT;
    request.stepRpm = 1'000.0;
    request.highestRpm = 3'000.0;
    request.loopSeconds = 0.5;
    request.writeOgg = true;
    const auto result = enginelab::exportLoopPack(request);
    require(result.success, "the pack renders: " + result.error);
    require(result.loops.size() == 6, "three points, on and off load");
    require(result.oggWritten, "Ogg Vorbis copies are written");
    for (const auto& warning : result.warnings)
        std::cout << "warning: " << warning << '\n';

    std::ifstream summaryStream(request.outputDirectory / "loop-pack.json");
    require(static_cast<bool>(summaryStream), "loop-pack.json is written");
    const auto summary = nlohmann::json::parse(summaryStream);
    require(summary.at("equivalent_four_stroke_cylinders").get<int>() == 4
            && summary.at("cycle_degrees").get<double>() == 720.0
            && summary.at("loops").size() == 6,
        "the summary gives the cylinders to declare and every loop");

    juce::WavAudioFormat wav;
    juce::OggVorbisAudioFormat ogg;
    constexpr double sampleRate = 48'000.0;
    double packPeak = 0.0;
    double quietestPeak = 1.0;
    double onLoadExhaustRms = 0.0;
    double offLoadExhaustRms = 0.0;
    for (const auto& loop : result.loops) {
        const auto label = std::to_string(static_cast<int>(loop.requestedRpm))
            + (loop.onLoad ? " on load" : " off load");
        std::cout << label << ": " << loop.meanRpm << " rpm ("
                  << loop.minimumRpm << ".." << loop.maximumRpm << "), "
                  << loop.cycles << " cycles, " << loop.frames << " frames, "
                  << (loop.settled ? "settled" : "NOT settled") << " after "
                  << loop.settleSeconds << " s, MAP "
                  << loop.meanManifoldPressureKpa << " kPa\n";

        // Held where it was asked: the absorber on load, the motor off load.
        if (loop.requestedRpm > engine->idleRpm * 1.05)
            require(std::abs(loop.meanRpm - loop.requestedRpm)
                    <= 0.03 * loop.requestedRpm,
                label + ": held at the requested speed");
        require(loop.settled, label + ": the point settled");

        // Whole cycles: the crank record and the speed both say so.
        require(loop.cycles >= 4, label + ": several cycles");
        require(std::abs(loop.crankDegrees / 720.0 - loop.cycles) < 0.01,
            label + ": the cut falls on a whole cycle");
        const auto cyclesFromSpeed = static_cast<double>(loop.frames) / sampleRate
            * loop.meanRpm / 120.0;
        require(std::abs(cyclesFromSpeed - loop.cycles) < 0.02 * loop.cycles,
            label + ": its length is its cycles at its speed");

        for (const auto& file : { loop.engineFile, loop.exhaustFile }) {
            const auto decoded = decode(wav, file);
            require(decoded.channels == 1 && decoded.bits == 24
                    && decoded.sampleRate == sampleRate,
                label + ": a mono 24-bit 48 kHz WAV");
            require(decoded.samples.size() == loop.frames,
                label + ": the WAV holds the loop");
            require(decoded.metadata.getValue("NumSampleLoops", "0") == "1"
                    && decoded.metadata.getValue("Loop0Start", "") == "0"
                    && decoded.metadata.getValue("Loop0End", "")
                        == juce::String(static_cast<juce::int64>(loop.frames - 1)),
                label + ": the smpl chunk loops the whole file");

            const auto& samples = decoded.samples;
            double peak = 0.0;
            double largestStep = 0.0;
            double mean = 0.0;
            for (std::size_t index = 0; index < samples.size(); ++index) {
                peak = std::max(peak, static_cast<double>(std::abs(samples[index])));
                mean += samples[index];
                if (index + 1 < samples.size())
                    largestStep = std::max(largestStep, static_cast<double>(
                        std::abs(samples[index + 1] - samples[index])));
            }
            mean /= static_cast<double>(samples.size());
            const auto seamStep = static_cast<double>(
                std::abs(samples.front() - samples.back()));
            std::cout << "  " << file.filename().string() << ": peak " << peak
                      << ", seam step " << seamStep << " (largest inside "
                      << largestStep << "), mean " << mean << '\n';
            if (peak > 0.0) {
                require(seamStep <= largestStep,
                    label + ": the last sample runs into the first as any two do");
                require(std::abs(mean) <= 1.0e-3 * peak, label + ": no DC offset");
            }
            packPeak = std::max(packPeak, peak);
            quietestPeak = std::min(quietestPeak, peak);

            auto oggFile = file;
            oggFile.replace_extension(".ogg");
            const auto vorbis = decode(ogg, oggFile);
            require(vorbis.channels == 1 && vorbis.samples.size() == loop.frames,
                label + ": the Ogg Vorbis copy holds the same loop");
        }
        if (loop.requestedRpm == 3'000.0)
            (loop.onLoad ? onLoadExhaustRms : offLoadExhaustRms) = loop.exhaustRms;
    }

    // One gain: the loudest file peaks at the pack's ceiling, the others below.
    require(std::abs(packPeak - std::pow(10.0, -1.0 / 20.0)) < 1.0e-4,
        "the loudest file peaks at -1 dBFS");
    require(quietestPeak < 0.7 * packPeak,
        "the files are not normalised one by one");
    require(onLoadExhaustRms > 2.0 * offLoadExhaustRms,
        "full throttle at 3000 rpm is louder than the overrun");

    // A second pack into the same folder is refused, and leaves it whole.
    const auto again = enginelab::exportLoopPack(request);
    require(!again.success && again.error.find("already exists") != std::string::npos,
        "an existing pack is not overwritten without asking");
    require(std::filesystem::exists(result.loops.front().engineFile),
        "the refused export leaves the first pack in place");

    std::cout << "EngineLab.LoopPackExport PASS: gain "
              << result.packGainDb << " dB\n";
    return 0;
}
