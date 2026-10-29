// Every catalogue engine survives a save and a reload: what the encoder writes,
// the decoder reads back. A field written but not read (or read under another
// name) shows up as a second encoding that differs from the first.
//
// A catalogue engine only covers the fields it sets away from their defaults,
// so a struct the whole catalogue leaves alone is checked on an engine built
// here with every one of its fields moved.

#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/serialization/JsonEngineSerializer.hpp>
#include <enginelab/serialization/YamlEngineSerializer.hpp>

#include <cstddef>
#include <cstdlib>
#include <iostream>
#include <string>
#include <string_view>

namespace {
using namespace enginelab;

int failures = 0;

void require(bool condition, std::string_view message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        ++failures;
        return;
    }
    std::cout << "ok: " << message << '\n';
}

/** The first line on which two encodings differ, for the failure message. */
[[nodiscard]] std::string firstDifference(const std::string& first, const std::string& second) {
    std::size_t line = 1;
    std::size_t start = 0;
    for (std::size_t index = 0; index < first.size() && index < second.size(); ++index) {
        if (first[index] != second[index]) {
            const auto end = first.find('\n', index);
            return "line " + std::to_string(line) + ": " + first.substr(start, end - start);
        }
        if (first[index] == '\n') {
            ++line;
            start = index + 1;
        }
    }
    return "one encoding is a prefix of the other";
}

void checkStable(const IEngineSerializer& serializer, std::string_view format, const EngineConfig& config) {
    const auto encoded = serializer.encode(config);
    const auto decoded = serializer.decode(encoded);
    if (!decoded) {
        require(false, std::string(format) + " decodes " + config.name + ": " + decoded.error);
        return;
    }
    const auto reEncoded = serializer.encode(*decoded.config);
    require(reEncoded == encoded, std::string(format) + " round trip of " + config.name + " is stable"
        + (reEncoded == encoded ? std::string {} : " (" + firstDifference(encoded, reEncoded) + ")"));
}

/**
 * Every carburettor field moved off its default, at both sites that carry one.
 * A catalogue engine cannot catch a dropped reader here: its carburettor is
 * all defaults, so an ignored field re-encodes to the same default and the two
 * encodings agree. These values are deliberately ugly so that no other field's
 * default could stand in for one of them.
 */
[[nodiscard]] CarburettorConfig distinctCarburettor(double offset) {
    CarburettorConfig carburettor;
    carburettor.enabled = true;
    carburettor.venturiDiameterMm = 23.75 + offset;
    carburettor.mainJetFlowNumber = 263.5 + offset;
    carburettor.pilotJetFlowNumber = 37.25 + offset;
    carburettor.pilotCircuitFactor = 0.1725 + offset * 0.001;
    carburettor.needleClipPosition = 2.5 + offset * 0.01;
    carburettor.airScrewTurns = 1.875 + offset * 0.01;
    carburettor.idleScrewTurns = 2.625 + offset * 0.01;
    carburettor.jetLiftMm = 6.125 + offset * 0.01;
    carburettor.jetDischargeCoefficient = 0.765 + offset * 0.001;
    carburettor.wallFilmFraction = 0.3125 + offset * 0.001;
    carburettor.wallFilmTimeConstantSeconds = 0.1375 + offset * 0.001;
    return carburettor;
}

} // namespace

int main() {
    const auto catalogue = loadEngineCatalog(ENGINELAB_CATALOG_ROOT);
    require(catalogue.entries.size() >= 10, "the catalogue loads");
    const JsonEngineSerializer json;
    const YamlEngineSerializer yaml;
    for (const auto& entry : catalogue.entries) {
        checkStable(json, "JSON", entry.config);
        checkStable(yaml, "YAML", entry.config);
    }
    // On an engine with one intake path the global intake and the path mirror
    // each other (normaliseEngineConfig), so they cannot carry two different
    // carburettors and the global reader is never on its own. Two or more paths
    // are not mirrored, which gives every reader and writer a value only it
    // can have carried.
    const EngineConfig* multiPath = nullptr;
    for (const auto& entry : catalogue.entries) {
        if (entry.config.intakePaths.size() >= 2) {
            multiPath = &entry.config;
            break;
        }
    }
    require(multiPath != nullptr, "the catalogue has an engine with two intake paths");
    if (multiPath != nullptr) {
        auto carburetted = *multiPath;
        carburetted.name += " (carburetted)";
        carburetted.intake.carburettor = distinctCarburettor(0.0);
        for (std::size_t index = 0; index < carburetted.intakePaths.size(); ++index)
            carburetted.intakePaths[index].geometry.carburettor =
                distinctCarburettor(static_cast<double>(index) + 1.0);
        checkStable(json, "JSON", carburetted);
        checkStable(yaml, "YAML", carburetted);
    }
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
