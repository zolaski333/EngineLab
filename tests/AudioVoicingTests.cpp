#include <enginelab/catalog/EngineCatalog.hpp>

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <algorithm>

namespace {

void require(bool condition, const std::string& message) {
    if (!condition) throw std::runtime_error(message);
}

void write(const std::filesystem::path& file, const std::string& text) {
    std::filesystem::create_directories(file.parent_path());
    std::ofstream output(file);
    output << text;
    if (!output) throw std::runtime_error("could not create voicing fixture");
}

} // namespace

int main() {
    const auto nonce = std::chrono::steady_clock::now().time_since_epoch().count();
    const auto root = std::filesystem::temp_directory_path()
        / ("enginelab-voicing-" + std::to_string(nonce));
    try {
        write(root / "voicing" / "default.yaml",
            "schema_version: 1\nvoicing:\n  volume: 0.8\n  exhaust_gain: 1.1\n");
        write(root / "voicing" / "families" / "motorcycle.yaml",
            "schema_version: 1\nvoicing:\n  exhaust_gain: 1.25\n  outlet_jet_gain: 0.7\n");
        write(root / "voicing" / "engines" / "cp2_engine.yaml",
            "schema_version: 1\nvoicing:\n  stereo_width: 1.4\n"
            "  saturation_drive: 0.6\n  saturation_placement: pre_shelf\n");

        const auto layered = enginelab::loadAudioVoicing(
            root, "Motorcycle", "CP2.engine");
        require(static_cast<bool>(layered) && layered.sources.size() == 3,
            "default, family and engine voicing layers were not all resolved");
        require(std::abs(layered.voicing.volume - 0.8) < 1.0e-12
                && std::abs(layered.voicing.exhaustGain - 1.25) < 1.0e-12
                && std::abs(layered.voicing.outletJetGain - 0.7) < 1.0e-12
                && std::abs(layered.voicing.stereoWidth - 1.4) < 1.0e-12
                && std::abs(layered.voicing.saturationDrive - 0.6) < 1.0e-12
                && layered.voicing.saturationPlacement
                    == enginelab::AudioSaturationPlacement::preShelf,
            "later voicing layers did not override only their authored fields");
        require(std::abs(layered.voicing.intakeGain - 0.85) < 1.0e-12,
            "unspecified fields must retain the shipping default");

        write(root / "voicing" / "engines" / "broken.yaml",
            "schema_version: 1\nvoicing:\n  mystery_gain: 4\n");
        const auto invalid = enginelab::loadAudioVoicing(
            root, "motorcycle", "broken");
        require(!invalid && invalid.sources.empty(),
            "an invalid override must reject the snapshot atomically");
        require(std::abs(invalid.voicing.volume - 1.0) < 1.0e-12
                && invalid.error.find("unknown key") != std::string::npos,
            "invalid voicing must fall back to safe compiled defaults with a diagnostic");

        const auto catalogue = enginelab::loadEngineCatalog(
            ENGINELAB_CATALOG_ROOT);
        require(catalogue.errors.empty() && catalogue.entries.size() == 16,
            "shipping catalogue and all authored voicing files must load");
        const auto authoredCount = std::count_if(
            catalogue.entries.begin(), catalogue.entries.end(),
            [](const auto& entry) {
                const auto& mix = entry.config.audioVoicing;
                return std::abs(mix.highFrequencyGain - 1.0) > 1.0e-12
                    || std::abs(mix.lowFrequencyGain - 1.0) > 1.0e-12
                    || std::abs(mix.exhaustGain - 1.0) > 1.0e-12
                    || std::abs(mix.stereoWidth - 1.0) > 1.0e-12
                    || std::abs(mix.saturationDrive) > 1.0e-12;
            });
        require(static_cast<std::size_t>(authoredCount)
                == catalogue.entries.size(),
            "every shipping engine must resolve an explicit audible monitor profile");
        const auto ls3 = std::find_if(
            catalogue.entries.begin(), catalogue.entries.end(),
            [](const auto& entry) {
                return entry.config.name == "LS3-like 6.2 Crossplane V8";
            });
        require(ls3 != catalogue.entries.end()
                && std::abs(ls3->config.audioVoicing.lowFrequencyGain - 1.20) < 1.0e-12
                && std::abs(ls3->config.audioVoicing.stereoWidth - 1.28) < 1.0e-12
                && std::abs(ls3->config.audioVoicing.saturationDrive - 0.34) < 1.0e-12,
            "LS3 profile must resolve by the normalised engine filename");

        std::filesystem::remove_all(root);
        std::cout << "PASS: layered strict audio voicing and 16 catalogue profiles\n";
        return 0;
    } catch (const std::exception& error) {
        std::error_code ignored;
        std::filesystem::remove_all(root, ignored);
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
