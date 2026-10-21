/** Designs engines the way the designer window does, and writes them where the
 *  sweep harness can measure them.
 *
 * The owner's two-stroke complaint is about the engines they *design*, not the
 * hand-authored catalogue ones, and every two-stroke instrument so far
 * measured the catalogue. `designCatalogueEngine` is the designer's whole
 * derivation, so a bench over it is the only way to sweep what the owner
 * actually builds. Each spec becomes one
 * `<out>/engines/<nn>_<key>.engine.yaml`, numbered in the order given, so
 * `--catalog-root <out>` is then a catalogue of designed engines.
 *
 * A spec file holds one engine per line, `key=value` separated by semicolons,
 * `#` comments and blank lines ignored:
 *
 *     key=rd_like; cycle=2t; use=sport_motorcycle; cylinders=2; displacement=0.347
 *     key=small_single; cycle=2t; use=road_motorcycle; cylinders=1; bore=54; stroke=54
 *
 * Keys: key, name, cycle (2t/4t), fuel (gasoline/diesel), use, layout,
 * cylinders, bank_count, bank_angle, bore, stroke, displacement, induction
 * (natural/turbo/super), boost, turbo_count. Anything else is a DesignValue
 * override by key, so `expansion_chamber_tuned_length_mm=900` or
 * `exhaust_port_open_deg=88` work exactly as the designer's own fields do, and
 * a typo surfaces as the designer's own "unknown key" error.
 */

#include <enginelab/catalog/EngineDesign.hpp>
#include <enginelab/serialization/YamlEngineSerializer.hpp>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

namespace {

[[nodiscard]] std::string trimmed(std::string_view text) {
    const auto first = text.find_first_not_of(" \t\r\n");
    if (first == std::string_view::npos) return {};
    const auto last = text.find_last_not_of(" \t\r\n");
    return std::string(text.substr(first, last - first + 1));
}

[[nodiscard]] bool parseNumber(const std::string& text, double& value) {
    try {
        std::size_t consumed = 0;
        const auto parsed = std::stod(text, &consumed);
        if (consumed != text.size()) return false;
        value = parsed;
        return true;
    } catch (const std::exception&) {
        return false;
    }
}

[[nodiscard]] std::optional<enginelab::EngineLayout> parseLayout(const std::string& text) {
    if (text == "inline") return enginelab::EngineLayout::inlineLayout;
    if (text == "v") return enginelab::EngineLayout::vLayout;
    if (text == "flat" || text == "boxer") return enginelab::EngineLayout::flat;
    if (text == "radial") return enginelab::EngineLayout::radial;
    if (text == "custom") return enginelab::EngineLayout::custom;
    return std::nullopt;
}

struct SpecLine final {
    enginelab::EngineSpec spec;
    std::string key;
    std::string error;
};

[[nodiscard]] SpecLine parseSpecLine(const std::string& line) {
    SpecLine result;
    std::istringstream stream(line);
    std::string field;
    while (std::getline(stream, field, ';')) {
        const auto text = trimmed(field);
        if (text.empty()) continue;
        const auto equals = text.find('=');
        if (equals == std::string::npos) {
            result.error = "field without an '=': " + text;
            return result;
        }
        const auto name = trimmed(std::string_view(text).substr(0, equals));
        const auto value = trimmed(std::string_view(text).substr(equals + 1));
        auto& spec = result.spec;
        if (name == "key") { result.key = value; continue; }
        if (name == "name") { spec.name = value; continue; }
        if (name == "cycle") {
            if (value == "2t" || value == "two_stroke") spec.cycle = enginelab::EngineCycle::twoStroke;
            else if (value == "4t" || value == "four_stroke") spec.cycle = enginelab::EngineCycle::fourStroke;
            else result.error = "the cycle is 2t or 4t";
            continue;
        }
        if (name == "fuel") {
            if (value == "diesel") spec.fuel = enginelab::FuelType::diesel;
            else if (value == "gasoline") spec.fuel = enginelab::FuelType::gasoline;
            else result.error = "the fuel is gasoline or diesel";
            continue;
        }
        if (name == "use") {
            if (const auto use = enginelab::parseEngineUse(value)) spec.use = *use;
            else result.error = "unknown use: " + value;
            continue;
        }
        if (name == "layout") {
            if (const auto layout = parseLayout(value)) spec.layout = *layout;
            else result.error = "unknown layout: " + value;
            continue;
        }
        if (name == "induction") {
            if (value == "natural") spec.induction = enginelab::Induction::natural;
            else if (value == "turbo") spec.induction = enginelab::Induction::turbocharged;
            else if (value == "super") spec.induction = enginelab::Induction::supercharged;
            else result.error = "the induction is natural, turbo or super";
            continue;
        }
        double number = 0.0;
        if (!parseNumber(value, number)) {
            result.error = name + " is not a number: " + value;
            return result;
        }
        if (name == "cylinders") spec.cylinderCount = static_cast<std::uint32_t>(number);
        else if (name == "bank_count") spec.bankCount = static_cast<std::uint32_t>(number);
        else if (name == "bank_angle") spec.bankAngleDegrees = number;
        else if (name == "bore") spec.boreMm = number;
        else if (name == "stroke") spec.strokeMm = number;
        else if (name == "displacement") spec.displacementLitres = number;
        else if (name == "boost") spec.boostPressureRatio = number;
        else if (name == "turbo_count") spec.turbochargerCount = static_cast<std::uint32_t>(number);
        else spec.overrides[name] = number;
    }
    if (result.key.empty() && !result.spec.name.empty()) result.key = result.spec.name;
    if (result.spec.name.empty()) result.spec.name = result.key;
    if (result.key.empty() && result.error.empty()) result.error = "every spec needs a key";
    return result;
}

void writeValuesRow(std::ostream& out, const std::string& key, const enginelab::DesignValue& value) {
    out << '"' << key << "\",\"" << value.key << "\",\"" << value.label << "\",\"" << value.unit
        << "\"," << std::setprecision(10) << value.value << ',' << (value.overridden ? 1 : 0) << ','
        << (value.settable ? 1 : 0) << ",\"" << value.rule << "\"\n";
}

} // namespace

int main(int argc, char** argv) {
    std::filesystem::path specFile;
    std::filesystem::path catalogRoot = std::filesystem::current_path();
    std::filesystem::path outputRoot;
    std::filesystem::path valuesFile;
    auto quiet = false;
    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        if (argument == "--spec" && index + 1 < argc) specFile = argv[++index];
        else if (argument == "--catalog-root" && index + 1 < argc) catalogRoot = argv[++index];
        else if (argument == "--output" && index + 1 < argc) outputRoot = argv[++index];
        else if (argument == "--values" && index + 1 < argc) valuesFile = argv[++index];
        else if (argument == "--quiet") quiet = true;
        else {
            std::cerr << "usage: EngineLabDesignBench --spec file [--catalog-root dir]"
                         " [--output dir] [--values file.csv] [--quiet]\n";
            return 2;
        }
    }
    if (specFile.empty()) {
        std::cerr << "--spec is required\n";
        return 2;
    }
    std::ifstream specStream(specFile);
    if (!specStream) {
        std::cerr << "cannot read " << specFile << '\n';
        return 2;
    }

    std::vector<SpecLine> specs;
    std::string line;
    while (std::getline(specStream, line)) {
        const auto text = trimmed(line);
        if (text.empty() || text.front() == '#') continue;
        specs.push_back(parseSpecLine(text));
    }
    if (specs.empty()) {
        std::cerr << "no specs in " << specFile << '\n';
        return 2;
    }

    if (!outputRoot.empty()) {
        std::error_code code;
        std::filesystem::create_directories(outputRoot / "engines", code);
        if (code) {
            std::cerr << "cannot create " << (outputRoot / "engines") << ": " << code.message() << '\n';
            return 2;
        }
    }

    std::ofstream values;
    if (!valuesFile.empty()) {
        values.open(valuesFile, std::ios::binary);
        if (!values) {
            std::cerr << "cannot write " << valuesFile << '\n';
            return 2;
        }
        values << "engine_key,value_key,label,unit,value,overridden,settable,rule\n";
    }

    const enginelab::YamlEngineSerializer serializer;
    auto failures = std::size_t { 0 };
    auto number = 0;
    for (const auto& entry : specs) {
        ++number;
        if (!entry.error.empty()) {
            std::cerr << entry.key << ": " << entry.error << '\n';
            ++failures;
            continue;
        }
        const auto designed = enginelab::designCatalogueEngine(entry.spec, catalogRoot);
        if (!designed.config) {
            std::cerr << entry.key << ": " << designed.error << '\n';
            ++failures;
            continue;
        }
        if (values)
            for (const auto& value : designed.values) writeValuesRow(values, entry.key, value);
        if (!quiet) {
            std::cout << "=== " << entry.key << " (" << designed.config->name << ")\n";
            for (const auto& value : designed.values) {
                if (!value.settable) continue;
                std::cout << "    " << std::setw(34) << std::left << value.key << ' ' << std::setw(12)
                          << std::right << value.value << ' ' << std::setw(8) << std::left << value.unit
                          << (value.overridden ? " (set)" : "") << '\n';
            }
        }
        if (outputRoot.empty()) continue;
        std::ostringstream name;
        name << std::setw(2) << std::setfill('0') << number << '_' << entry.key << ".engine.yaml";
        const auto path = outputRoot / "engines" / name.str();
        std::ofstream out(path, std::ios::binary);
        out << serializer.encode(*designed.config);
        if (!out) {
            std::cerr << entry.key << ": cannot write " << path << '\n';
            ++failures;
        }
    }
    std::cout << "designed " << specs.size() - failures << '/' << specs.size();
    if (!outputRoot.empty()) std::cout << " into " << (outputRoot / "engines").string();
    std::cout << '\n';
    return failures == 0 ? 0 : 1;
}
