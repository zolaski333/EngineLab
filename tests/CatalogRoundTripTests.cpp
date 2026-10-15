// Every catalogue engine survives a save and a reload: what the encoder writes,
// the decoder reads back. A field written but not read (or read under another
// name) shows up as a second encoding that differs from the first.

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
    return failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
