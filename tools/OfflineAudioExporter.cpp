#include <enginelab/audio/LoopPackExporter.hpp>
#include <enginelab/audio/OfflineAudioExporter.hpp>
#include <enginelab/catalog/EngineCatalog.hpp>

#include <algorithm>
#include <cctype>
#include <charconv>
#include <cmath>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace {

[[nodiscard]] std::string lowercase(std::string value) {
    std::transform(
        value.begin(), value.end(), value.begin(),
        [](unsigned char character) {
            return static_cast<char>(std::tolower(character));
        });
    return value;
}

void printUsage() {
    std::cout
        << "EngineLabOfflineAudioExporter\n"
        << "  --engine <unique catalogue-name substring>\n"
        << "  --output <new-or-empty directory>\n"
        << "  [--catalog-root <EngineLab root>]\n"
        << "  [--sample-rate 48000|96000|192000]\n"
        << "  [--format pcm24|float32]\n"
        << "  [--scenario <schema-v1 JSON>]\n"
        << "  [--stems | --no-stems]\n"
        << "  [--volume <0..2>] [--convolution <0..1>]\n"
        << "  [--combustion <0..2>] [--exhaust <0..2>]\n"
        << "  [--intake <0..2>] [--mechanical <0..2>]\n"
        << "  [--overwrite]\n"
        << "  [--exhaust-cell-mm <mm>]  (a measurement mesh, not the application's)\n"
        << "  [--exhaust-tap <component>:<fraction>]  (the audio network's own pressure there, exhaust_tap_pa.f32)\n"
        << "  [--outlet-convective-loss on|off]  (diagnostic: the outlets' (1-M)/(1+M) reflection factor; absent keeps the default)\n"
        << "  [--loop-pack [--loop-step <rpm>] [--loop-seconds <s>] [--ogg]]\n"
        << "      seamless loops for a game instead of the showcase (48 kHz)\n"
        << "  --list-engines\n";
}

[[nodiscard]] bool parseDouble(
    const char* text, double& destination) {
    try {
        std::size_t consumed {};
        const auto parsed = std::stod(text, &consumed);
        if (consumed != std::string(text).size()
            || !std::isfinite(parsed))
            return false;
        destination = parsed;
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

} // namespace

int main(int argc, char** argv) {
    std::cout << std::unitbuf;
    std::filesystem::path catalogRoot =
        ENGINELAB_CATALOG_ROOT;
    std::filesystem::path outputDirectory;
    std::filesystem::path scenarioFile;
    std::string engineFilter;
    std::uint32_t sampleRate = 96'000;
    auto format = enginelab::OfflineWaveFormat::pcm24;
    bool writeStems = true;
    std::optional<bool> outletConvectiveLoss;
    bool overwrite = false;
    bool listEngines = false;
    bool loopPack = false;
    bool loopOgg = false;
    bool sampleRateGiven = false;
    double loopStep = 500.0;
    double loopSeconds = 2.0;
    std::optional<double> exhaustCellMm;
    std::optional<enginelab::OfflineAudioExportRequest::ExhaustPressureTap> exhaustTap;
    enginelab::OfflineAudioMix mix;
    struct MixOverrides final {
        std::optional<double> volume;
        std::optional<double> convolution;
        std::optional<double> combustion;
        std::optional<double> exhaust;
        std::optional<double> intake;
        std::optional<double> mechanical;
    } mixOverrides;

    for (int argument = 1; argument < argc; ++argument) {
        const std::string option = argv[argument];
        const auto requireValue = [&]() -> const char* {
            if (argument + 1 >= argc) return nullptr;
            return argv[++argument];
        };
        if (option == "--help" || option == "-h") {
            printUsage();
            return 0;
        }
        if (option == "--list-engines") {
            listEngines = true;
            continue;
        }
        if (option == "--stems") {
            writeStems = true;
            continue;
        }
        if (option == "--no-stems") {
            writeStems = false;
            continue;
        }
        if (option == "--outlet-convective-loss") {
            const auto* value = requireValue();
            const std::string text = value != nullptr ? value : "";
            if (text != "on" && text != "off") {
                std::cerr << "--outlet-convective-loss takes on or off.\n";
                return 1;
            }
            outletConvectiveLoss = text == "on";
            continue;
        }
        if (option == "--overwrite") {
            overwrite = true;
            continue;
        }
        if (option == "--loop-pack") {
            loopPack = true;
            continue;
        }
        if (option == "--ogg") {
            loopOgg = true;
            continue;
        }

        const auto* value = requireValue();
        if (value == nullptr) {
            std::cerr << "Missing value for " << option << '\n';
            return 1;
        }
        if (option == "--engine") {
            engineFilter = value;
        } else if (option == "--output") {
            outputDirectory = value;
        } else if (option == "--catalog-root") {
            catalogRoot = value;
        } else if (option == "--scenario") {
            scenarioFile = value;
        } else if (option == "--sample-rate") {
            try {
                std::size_t consumed {};
                const auto parsed = std::stoul(value, &consumed);
                if (consumed != std::string(value).size()
                    || parsed
                        > std::numeric_limits<std::uint32_t>::max())
                    throw std::invalid_argument("sample rate");
                sampleRate = static_cast<std::uint32_t>(parsed);
                sampleRateGiven = true;
            } catch (const std::exception&) {
                std::cerr << "Invalid sample rate: " << value << '\n';
                return 1;
            }
        } else if (option == "--format") {
            const auto selected = lowercase(value);
            if (selected == "pcm24")
                format = enginelab::OfflineWaveFormat::pcm24;
            else if (selected == "float32")
                format = enginelab::OfflineWaveFormat::float32;
            else {
                std::cerr << "Unknown format: " << value << '\n';
                return 1;
            }
        } else if (option == "--loop-step") {
            if (!parseDouble(value, loopStep) || loopStep < 50.0) {
                std::cerr << "The loop step must be at least 50 rpm.\n";
                return 1;
            }
        } else if (option == "--loop-seconds") {
            if (!parseDouble(value, loopSeconds) || loopSeconds < 0.25) {
                std::cerr << "A loop must last at least 0.25 s.\n";
                return 1;
            }
        } else if (option == "--volume") {
            double parsed {};
            if (!parseDouble(value, parsed)
                || parsed < 0.0 || parsed > 2.0) {
                std::cerr << "Volume must be in [0, 2].\n";
                return 1;
            }
            mixOverrides.volume = parsed;
        } else if (option == "--convolution") {
            double parsed {};
            if (!parseDouble(value, parsed)
                || parsed < 0.0 || parsed > 1.0) {
                std::cerr << "Convolution must be in [0, 1].\n";
                return 1;
            }
            mixOverrides.convolution = parsed;
        } else if (option == "--combustion") {
            double parsed {};
            if (!parseDouble(value, parsed)
                || parsed < 0.0 || parsed > 2.0) {
                std::cerr << "Combustion gain must be in [0, 2].\n";
                return 1;
            }
            mixOverrides.combustion = parsed;
        } else if (option == "--exhaust") {
            double parsed {};
            if (!parseDouble(value, parsed)
                || parsed < 0.0 || parsed > 2.0) {
                std::cerr << "Exhaust gain must be in [0, 2].\n";
                return 1;
            }
            mixOverrides.exhaust = parsed;
        } else if (option == "--intake") {
            double parsed {};
            if (!parseDouble(value, parsed)
                || parsed < 0.0 || parsed > 2.0) {
                std::cerr << "Intake gain must be in [0, 2].\n";
                return 1;
            }
            mixOverrides.intake = parsed;
        } else if (option == "--exhaust-cell-mm") {
            double parsed {};
            if (!parseDouble(value, parsed) || !(parsed > 0.0)) {
                std::cerr << "Exhaust cell length must be positive.\n";
                return 1;
            }
            exhaustCellMm = parsed;
        } else if (option == "--exhaust-tap") {
            const std::string text(value);
            const auto colon = text.find(':');
            double fraction {};
            // The id is checked whole: std::stoul threw on "bad" and ended
            // the tool without a word (audit RT-05).
            std::uint32_t component {};
            const auto* const idEnd = text.data() + (colon == std::string::npos ? 0 : colon);
            const auto parsedId = std::from_chars(text.data(), idEnd, component);
            if (colon == std::string::npos || colon == 0 || parsedId.ec != std::errc {} || parsedId.ptr != idEnd
                || !parseDouble(text.substr(colon + 1).c_str(), fraction)
                || fraction < 0.0 || fraction > 1.0) {
                std::cerr << "Exhaust tap is <component id>:<fraction 0..1>.\n";
                return 1;
            }
            exhaustTap = enginelab::OfflineAudioExportRequest::ExhaustPressureTap { component, fraction };
        } else if (option == "--mechanical") {
            double parsed {};
            if (!parseDouble(value, parsed)
                || parsed < 0.0 || parsed > 2.0) {
                std::cerr << "Mechanical gain must be in [0, 2].\n";
                return 1;
            }
            mixOverrides.mechanical = parsed;
        } else {
            std::cerr << "Unknown option: " << option << '\n';
            return 1;
        }
    }

    const auto catalog = enginelab::loadEngineCatalog(catalogRoot);
    if (catalog.entries.empty()) {
        std::cerr << "No engines found below "
                  << catalogRoot.string() << '\n';
        for (const auto& error : catalog.errors)
            std::cerr << "  " << error << '\n';
        return 1;
    }
    if (listEngines) {
        for (const auto& entry : catalog.entries)
            std::cout << entry.config.name << '\n';
        return 0;
    }
    if (engineFilter.empty() || outputDirectory.empty()) {
        printUsage();
        return 1;
    }

    const auto wanted = lowercase(engineFilter);
    std::vector<const enginelab::EngineCatalogEntry*> matches;
    for (const auto& entry : catalog.entries) {
        if (lowercase(entry.config.name).find(wanted)
            != std::string::npos)
            matches.push_back(&entry);
    }
    if (matches.size() != 1) {
        std::cerr << "Engine filter `" << engineFilter << "` matched "
                  << matches.size() << " entries.\n";
        for (const auto* match : matches)
            std::cerr << "  " << match->config.name << '\n';
        return 1;
    }

    // Catalogue voicing is the offline default too; explicit command-line
    // switches remain the final layer for controlled A/B renders.
    mix = matches.front()->config.audioVoicing;
    if (mixOverrides.volume) mix.volume = *mixOverrides.volume;
    if (mixOverrides.convolution) mix.convolution = *mixOverrides.convolution;
    if (mixOverrides.combustion) mix.combustionGain = *mixOverrides.combustion;
    if (mixOverrides.exhaust) mix.exhaustGain = *mixOverrides.exhaust;
    if (mixOverrides.intake) mix.intakeGain = *mixOverrides.intake;
    if (mixOverrides.mechanical) mix.mechanicalGain = *mixOverrides.mechanical;

    if (loopPack) {
        enginelab::LoopPackRequest request;
        request.engine = matches.front()->config;
        request.mix = mix;
        request.outputDirectory = outputDirectory;
        request.assetRoot = catalogRoot;
        request.sampleRateHz = sampleRateGiven ? sampleRate : 48'000U;
        request.stepRpm = loopStep;
        request.loopSeconds = loopSeconds;
        request.writeOgg = loopOgg;
        request.overwriteExistingFiles = overwrite;
        std::cout << "Engine: " << request.engine.name << '\n'
                  << "Loop pack: every " << loopStep << " rpm, "
                  << loopSeconds << " s loops\n";
        std::string lastStage;
        const auto result = enginelab::exportLoopPack(request,
            [&lastStage](double fraction, std::string_view stage) {
                if (stage != lastStage) {
                    lastStage = stage;
                    std::cout << "  " << std::setw(3)
                              << static_cast<int>(std::round(fraction * 100.0))
                              << "%  " << stage << '\n';
                }
                return true;
            });
        if (!result.success) {
            std::cerr << (result.cancelled ? "Export cancelled."
                                          : "Export failed: " + result.error)
                      << '\n';
            return result.cancelled ? 2 : 1;
        }
        std::cout << std::fixed << std::setprecision(1);
        for (const auto& loop : result.loops)
            std::cout << (loop.onLoad ? "  on  " : "  off ") << std::setw(7)
                      << loop.requestedRpm << " rpm: held " << loop.meanRpm
                      << " (" << loop.minimumRpm << ".." << loop.maximumRpm
                      << "), " << loop.cycles << " cycles, MAP "
                      << loop.meanManifoldPressureKpa << " kPa, settled "
                      << (loop.settled ? "yes" : "NO") << " after "
                      << loop.settleSeconds << " s\n";
        std::cout << "Pack gain " << result.packGainDb
                  << " dB; cylinders to declare "
                  << result.equivalentFourStrokeCylinders << '\n';
        for (const auto& warning : result.warnings)
            std::cout << "WARNING: " << warning << '\n';
        return 0;
    }

    enginelab::OfflineAudioScenario scenario =
        enginelab::makeDefaultOfflineAudioScenario(
            matches.front()->config);
    if (!scenarioFile.empty()) {
        std::string error;
        if (!enginelab::loadOfflineAudioScenario(
                scenarioFile, scenario, error)) {
            std::cerr << error << '\n';
            return 1;
        }
    }

    enginelab::OfflineAudioExportRequest request;
    request.engine = matches.front()->config;
    request.scenario = std::move(scenario);
    request.mix = mix;
    request.outputDirectory = outputDirectory;
    request.assetRoot = catalogRoot;
    request.sampleRateHz = sampleRate;
    request.format = format;
    request.writeStems = writeStems;
    request.overwriteExistingFiles = overwrite;
    if (exhaustCellMm) request.exhaustCellLengthM = *exhaustCellMm * 1.0e-3;
    request.exhaustPressureTap = exhaustTap;
    request.outletConvectiveLoss = outletConvectiveLoss;

    std::cout << "Engine: " << request.engine.name << '\n'
              << "Scenario: " << request.scenario.name << '\n'
              << "Output: " << outputDirectory.string() << '\n'
              << "Format: " << sampleRate << " Hz / "
              << enginelab::offlineWaveFormatName(format)
              << (writeStems ? " / master + stems\n" : " / master\n");

    int lastBucket = -1;
    std::string lastStage;
    const auto result = enginelab::exportOfflineAudio(
        request,
        [&](double fraction, std::string_view stage) {
            const auto bucket =
                std::clamp(static_cast<int>(fraction * 10.0), 0, 10);
            if (bucket != lastBucket || stage != lastStage) {
                lastBucket = bucket;
                lastStage = stage;
                std::cout << "  " << std::setw(3)
                          << static_cast<int>(std::round(fraction * 100.0))
                          << "%  " << stage << '\n';
            }
            return true;
        });
    if (!result.success) {
        std::cerr << (result.cancelled ? "Export cancelled."
                                      : "Export failed: " + result.error)
                  << '\n';
        return result.cancelled ? 2 : 1;
    }

    std::cout << std::fixed << std::setprecision(6)
              << "Rendered " << result.renderedFrames << " frames ("
              << result.durationSeconds << " s), peak "
              << result.masterPeak << ", RMS "
              << result.masterRms << '\n'
              << "Physical exhaust: "
              << (result.physicalExhaustActive ? "active" : "inactive")
              << "; compiled exhaust graph: "
              << (result.compiledExhaustTopologyActive ? "yes" : "no")
              << "; delay truncations: "
              << result.delayTruncationCount
              << "; invalid boundaries: "
              << result.invalidBoundarySampleCount
              << "; dropped telemetry: "
              << result.droppedFiringEvents << '/'
              << result.droppedPressureSamples << '\n'
              << "Audio physics: cycle multiplier "
              << result.cycleMultiplierMinimum << ".."
              << result.cycleMultiplierMaximum << " ("
              << result.cycleVariationSamples << " samples); afterfire peak "
              << result.afterfirePeakHeatReleaseKw << " kW, fuel burned "
              << result.afterfireFuelBurnedMg << " mg; porous mufflers "
              << result.porousMufflerCount << '\n';
    for (const auto& warning : result.warnings)
        std::cout << "WARNING: " << warning << '\n';
    for (const auto& file : result.files)
        std::cout << "Wrote " << file.string() << '\n';
    return 0;
}
