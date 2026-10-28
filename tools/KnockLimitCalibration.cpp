// Measures a catalogue engine's knock-limited spark at full load, as the
// engine designer does when it builds a design (`calibrateKnockLimit`), and
// prints it as the engine file's `ignition.knock_limit_curve`. With --verify
// it measures again with that limit applied: the knock control's retard left
// behind the map, which a map at the limit keeps near zero.

#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/runtime/KnockLimitCalibration.hpp>

#include <filesystem>
#include <iomanip>
#include <iostream>
#include <string>

namespace {

void printCalibration(const enginelab::KnockLimitCalibration& calibration) {
    std::cout << "rpm,limit_advance_deg,map_kpa,knock_retard_deg,ca50_atdc\n" << std::fixed;
    for (std::size_t index = 0; index < calibration.curve.size(); ++index) {
        const auto& sample = calibration.curve[index];
        std::cout << std::setprecision(0) << sample.rpm << ',' << std::setprecision(2) << sample.advanceDegrees
                  << ',' << std::setprecision(1) << sample.manifoldPressureKpa << ','
                  << std::setprecision(2) << calibration.retardDegrees[index] << ','
                  << std::setprecision(1) << calibration.ca50Degrees[index] << '\n';
    }
}

} // namespace

int main(int argc, char** argv) {
    std::filesystem::path catalogRoot = std::filesystem::current_path();
    std::string filter;
    auto verify = false;
    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        if (argument == "--catalog-root" && index + 1 < argc) catalogRoot = argv[++index];
        else if (argument == "--filter" && index + 1 < argc) filter = argv[++index];
        else if (argument == "--verify") verify = true;
        else {
            std::cout << "usage: EngineLabKnockLimitCalibration [--catalog-root DIR] --filter NAME [--verify]\n";
            return argument == "--help" ? 0 : 2;
        }
    }
    auto catalog = enginelab::loadEngineCatalog(catalogRoot);
    const auto selected = enginelab::selectSingleEngineCatalogEntry(catalog.entries, filter);
    if (filter.empty() || !selected) {
        std::cerr << "FAIL: give --filter with one engine's selector\n";
        for (const auto* match : selected.matches)
            std::cerr << "  " << match->config.audioVoicingKey << "  " << match->config.name << '\n';
        return 1;
    }
    auto config = selected.entry->config;
    std::cout << "engine: " << config.name << '\n';
    const auto calibration = enginelab::calibrateKnockLimit(config);
    if (!calibration.error.empty()) {
        std::cerr << "FAIL: " << calibration.error << '\n';
        return 1;
    }
    std::cout << "measured (map alone, knock control on):\n";
    printCalibration(calibration);
    std::cout << "knock_limit_curve:\n" << std::fixed;
    for (const auto& sample : calibration.curve)
        std::cout << "  - { rpm: " << std::setprecision(0) << sample.rpm << ", advance_deg: "
                  << std::setprecision(1) << sample.advanceDegrees << ", map_kpa: " << std::setprecision(0)
                  << sample.manifoldPressureKpa << " }\n";
    if (verify) {
        config.ignition.knockLimitCurve = calibration.curve;
        const auto check = enginelab::measureFullLoadSpark(config);
        if (!check.error.empty()) {
            std::cerr << "FAIL: " << check.error << '\n';
            return 1;
        }
        std::cout << "verified (the map held under that limit):\n";
        printCalibration(check);
    }
    return 0;
}
