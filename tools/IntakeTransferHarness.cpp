// The intake's acoustic transfer, frequency by frequency.
//
// Drives one runner of a catalogue engine's AcousticIntakeNetwork with a
// sinusoidal mass flow and reports the radiated level at each frequency for
// its intake as authored, with the filter element alone (a pod: no airbox),
// and bare (no airbox, no element; the inlet duct kept). The differences
// are the airbox's and the element's insertion losses, to be read against a
// plane-wave model of the same geometry.
//
//   EngineLabIntakeTransferHarness [--engine <name>] [--amplitude <kg/s>]

#include <enginelab/audio/AcousticIntakeNetwork.hpp>
#include <enginelab/catalog/EngineCatalog.hpp>

#include <array>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <numbers>
#include <string>
#include <string_view>
#include <vector>

namespace {

double radiatedDb(enginelab::EngineConfig config, double frequencyHz, double amplitudeKgPerSecond) {
    enginelab::normaliseEngineConfig(config);
    enginelab::AcousticIntakeNetwork intake(config);
    constexpr double sampleRate = 48'000.0;
    if (!intake.valid() || !intake.prepare(sampleRate)) return std::nan("");
    std::vector<enginelab::AcousticIntakeNetwork::PathBoundary> paths(intake.pathCount(),
        { 0.0032F, 1.15F, 350.0F });
    intake.beginBlock(paths, 1.0);
    std::vector<enginelab::AcousticIntakeNetwork::CylinderBoundary> cylinders(config.cylinders.size());
    for (auto& cylinder : cylinders) {
        cylinder.conductanceAreaM2 = 0.00045F;
        cylinder.densityKgPerM3 = 1.15F;
        cylinder.soundSpeedMps = 350.0F;
        cylinder.physical = true;
    }
    auto energy = 0.0;
    constexpr std::size_t settle = 24'000, measured = 24'000;
    for (std::size_t sample = 0; sample < settle + measured; ++sample) {
        cylinders[0].massFlowKgPerSecond = static_cast<float>(0.040 + amplitudeKgPerSecond
            * std::sin(2.0 * std::numbers::pi * frequencyHz * static_cast<double>(sample) / sampleRate));
        const auto output = intake.process(cylinders, 1.0F);
        if (sample < settle) continue;
        for (std::size_t path = 0; path < intake.pathCount(); ++path)
            energy += static_cast<double>(output[path].leftPa) * output[path].leftPa;
    }
    return 10.0 * std::log10(energy / static_cast<double>(measured) + 1.0e-30);
}

} // namespace

int main(int argc, char** argv) {
    std::string engine = "K20";
    auto amplitude = 0.010;
    for (int index = 1; index < argc; ++index) {
        const std::string_view argument { argv[index] };
        if (argument == "--engine" && index + 1 < argc) engine = argv[++index];
        else if (argument == "--amplitude" && index + 1 < argc) amplitude = std::stod(argv[++index]);
    }
    const auto catalog = enginelab::loadEngineCatalog(ENGINELAB_CATALOG_ROOT);
    const enginelab::EngineConfig* found = nullptr;
    for (const auto& entry : catalog.entries)
        if (entry.config.name.find(engine) != std::string::npos) { found = &entry.config; break; }
    if (found == nullptr) {
        std::cerr << "no engine matches " << engine << '\n';
        return EXIT_FAILURE;
    }
    const auto variant = [&](bool airbox, bool element) {
        auto config = *found;
        const auto apply = [&](enginelab::IntakeConfig& geometry) {
            if (!airbox) geometry.airboxVolumeLitres = 0.0;
            geometry.podFilter = !airbox && element;
        };
        apply(config.intake);
        for (auto& path : config.intakePaths) apply(path.geometry);
        return config;
    };
    const auto& geometry = found->intakePaths.empty() ? found->intake : found->intakePaths.front().geometry;
    std::cout << found->name << ": airbox " << geometry.airboxVolumeLitres << " L, inlet duct "
              << geometry.inletDuctLengthMm << " x " << geometry.inletDuctDiameterMm << " mm, bellmouth "
              << geometry.bellmouthDiameterMm << " mm, plenum " << geometry.plenumVolumeLitres << " L\n";
    std::cout << std::setw(8) << "Hz" << std::setw(11) << "authored" << std::setw(11) << "element"
              << std::setw(11) << "bare" << std::setw(12) << "box_IL" << std::setw(12) << "element_IL" << '\n';
    for (const auto frequency : { 30.0, 50.0, 70.0, 100.0, 140.0, 200.0, 280.0, 400.0, 560.0, 800.0, 1'100.0,
             1'600.0, 2'200.0, 3'200.0, 4'500.0 }) {
        const auto authored = radiatedDb(*found, frequency, amplitude);
        const auto element = radiatedDb(variant(false, true), frequency, amplitude);
        const auto bare = radiatedDb(variant(false, false), frequency, amplitude);
        std::cout << std::fixed << std::setprecision(0) << std::setw(8) << frequency << std::setprecision(1)
                  << std::setw(11) << authored << std::setw(11) << element << std::setw(11) << bare
                  << std::setw(12) << element - authored << std::setw(12) << bare - element << '\n';
    }
    return EXIT_SUCCESS;
}
