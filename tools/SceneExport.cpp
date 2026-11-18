// Writes the 3-D scene of catalogue engines as JSON, for inspecting the
// laid-out geometry outside the app (a three.js page, a script).
//
//   EngineLabSceneExport --out <directory> [--filter <substring>] [--crank <degrees>]
//                        [--catalog-root <directory>] [--saved <folder>] [--designed all]
//                        [--two-stroke-intake <crankcase_reed|cylinder_port>]
//
// One <index>.json per engine: every part (material, layers, posed transform,
// positions in mm, triangle indices), every duct (kind, ids, centreline), the
// port anchors, and the route check (RouteCheck) issues; plus index.json
// listing the files. Prints the issue counts per engine, then the exhaust's
// plausibility (checkPlausibility): mm beside, below, above, ahead of and
// behind the engine past the allowances, coiled pipes, astray outlets, the mirror
// asymmetry ("-" where the ports are not mirror images) and pipes drawn
// folded. `--saved` exports the saved engines of a folder (the app's, e.g.
// %APPDATA%/EngineLab/engines) instead of the catalogue; `--designed all`, the
// designer's engines over its layouts and exhaust systems, with and without a
// turbo (EngineModel3DTests' envelope check). `--two-stroke-intake`
// redraws every two-stroke with that intake, so both layouts can be looked at.

#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/catalog/EngineDesign.hpp>
#include <enginelab/catalog/SavedEngines.hpp>
#include <enginelab/foundation/TwoStrokePorts.hpp>
#include <enginelab/render/EngineModel3D.hpp>
#include <enginelab/render/RouteCheck.hpp>

#include <array>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace {
using namespace enginelab;
using namespace enginelab::render;

/** A JSON string's contents. A control character (a newline in an accepted
 *  engine name) went out raw and the file was not JSON (audit RT-07). */
std::string escape(std::string_view text) {
    std::string out;
    for (const auto c : text) {
        const auto byte = static_cast<unsigned char>(c);
        if (c == '"' || c == '\\') {
            out += '\\';
            out += c;
        } else if (byte < 0x20U) {
            char code[8];
            std::snprintf(code, sizeof(code), "\\u%04x", static_cast<unsigned>(byte));
            out += code;
        } else {
            out += c;
        }
    }
    return out;
}

void writeNumber(std::ostream& out, float value) {
    char buffer[32];
    std::snprintf(buffer, sizeof(buffer), "%.2f", static_cast<double>(value));
    out << buffer;
}

void writeVec(std::ostream& out, Vec3 v) {
    out << '[';
    writeNumber(out, v.x);
    out << ',';
    writeNumber(out, v.y);
    out << ',';
    writeNumber(out, v.z);
    out << ']';
}

/** `written` is false when the file could not be written (audit RT-06). */
RouteReport writeEngine(const EngineConfig& config, double crankDegrees, const std::filesystem::path& file,
                        PlausibilityReport& plausibility, bool& written) {
    const EngineModel3D model(config);
    const auto report = checkRoutes(model);
    plausibility = checkPlausibility(model);
    std::vector<SceneInstance> instances;
    ScenePoseInput input;
    input.crankAngleDegrees = crankDegrees;
    model.pose(input, instances);

    std::ofstream out(file, std::ios::binary);
    out << "{\"name\":\"" << escape(config.name) << "\",\"parts\":[";
    for (std::size_t p = 0; p < model.parts().size(); ++p) {
        const auto& part = model.parts()[p];
        if (p > 0) out << ',';
        out << "{\"material\":" << static_cast<int>(part.material) << ",\"layers\":" << static_cast<int>(part.layers)
            << ",\"role\":" << static_cast<int>(model.identify(static_cast<std::uint16_t>(p)).role) << ",\"transform\":[";
        const auto& m = instances[p].transform.m;
        for (std::size_t i = 0; i < m.size(); ++i) {
            if (i > 0) out << ',';
            out << m[i];
        }
        out << "],\"positions\":[";
        for (std::size_t v = 0; v < part.mesh.vertices.size(); ++v) {
            const auto& position = part.mesh.vertices[v].position;
            if (v > 0) out << ',';
            writeNumber(out, position.x);
            out << ',';
            writeNumber(out, position.y);
            out << ',';
            writeNumber(out, position.z);
        }
        out << "],\"indices\":[";
        for (std::size_t i = 0; i < part.mesh.indices.size(); ++i) {
            if (i > 0) out << ',';
            out << part.mesh.indices[i];
        }
        out << "]}";
    }
    out << "],\"ducts\":[";
    for (std::size_t d = 0; d < model.ducts().size(); ++d) {
        const auto& duct = model.ducts()[d];
        if (d > 0) out << ',';
        out << "{\"kind\":" << static_cast<int>(duct.kind) << ",\"path\":" << duct.pathId
            << ",\"element\":" << duct.elementId << ",\"type\":" << static_cast<int>(duct.componentType)
            << ",\"part\":" << duct.part << ",\"authored\":";
        writeNumber(out, duct.authoredLengthMm);
        out << ",\"folded\":";
        writeNumber(out, duct.foldedMm);
        out << ",\"centreline\":[";
        for (std::size_t i = 0; i < duct.centreline.size(); ++i) {
            if (i > 0) out << ',';
            writeVec(out, duct.centreline[i]);
        }
        out << "],\"radii\":[";
        for (std::size_t i = 0; i < duct.radii.size(); ++i) {
            if (i > 0) out << ',';
            writeNumber(out, duct.radii[i]);
        }
        out << "]}";
    }
    out << "],\"issues\":[";
    for (std::size_t i = 0; i < report.issues.size(); ++i) {
        const auto& issue = report.issues[i];
        if (i > 0) out << ',';
        out << "{\"kind\":" << static_cast<int>(issue.kind) << ",\"duct\":" << issue.duct << ",\"other\":" << issue.other
            << ",\"value\":";
        writeNumber(out, issue.value);
        out << ",\"where\":";
        writeVec(out, issue.where);
        out << '}';
    }
    out << "],\"solids\":[";
    for (std::size_t i = 0; i < model.solids().size(); ++i) {
        const auto& solid = model.solids()[i];
        if (i > 0) out << ',';
        out << "{\"cylinder\":" << (solid.shape == EngineSolid::Shape::cylinder ? 1 : 0) << ",\"centre\":";
        writeVec(out, solid.centre);
        out << ",\"axes\":[";
        for (int a = 0; a < 3; ++a) {
            if (a > 0) out << ',';
            writeVec(out, solid.axes[a]);
        }
        out << "],\"half\":";
        writeVec(out, solid.half);
        out << '}';
    }
    out << "],\"rear\":";
    writeVec(out, model.rear());
    out << ",\"exhaustPorts\":[";
    for (std::size_t i = 0; i < model.exhaustPorts().size(); ++i) {
        const auto& port = model.exhaustPorts()[i];
        if (i > 0) out << ',';
        out << "{\"cylinder\":" << port.cylinderId << ",\"position\":";
        writeVec(out, port.position);
        out << ",\"direction\":";
        writeVec(out, port.direction);
        out << '}';
    }
    out << "]}\n";
    out.close();
    written = static_cast<bool>(out);
    return report;
}
} // namespace

/** The designer's engines the envelope check draws: its layouts, each with
 *  the designer's exhaust systems, and a turbo on the one pipe. */
std::vector<EngineConfig> designedEngines(const std::filesystem::path& catalogRoot) {
    struct Shape final {
        const char* name;
        EngineLayout layout;
        std::uint32_t cylinders;
        double bankAngle;
        std::vector<double> banks;
        EngineUse use;
        double bore;
        double stroke;
    };
    const std::vector<Shape> shapes {
        { "I4", EngineLayout::inlineLayout, 4, 0.0, {}, EngineUse::roadCar, 86.0, 86.0 },
        { "I6", EngineLayout::inlineLayout, 6, 0.0, {}, EngineUse::sportCar, 86.0, 86.0 },
        { "V8", EngineLayout::vLayout, 8, 90.0, {}, EngineUse::sportCar, 100.0, 92.0 },
        { "V12", EngineLayout::vLayout, 12, 60.0, {}, EngineUse::sportCar, 89.0, 80.0 },
        { "flat-6", EngineLayout::flat, 6, 0.0, {}, EngineUse::sportCar, 102.0, 76.0 },
        { "flat-12", EngineLayout::flat, 12, 0.0, {}, EngineUse::sportCar, 80.0, 74.0 },
        { "X8", EngineLayout::custom, 8, 0.0, { -135.0, -45.0, 45.0, 135.0 }, EngineUse::sportCar, 86.0, 86.0 },
        { "X12", EngineLayout::custom, 12, 0.0, { -135.0, -45.0, 45.0, 135.0 }, EngineUse::sportCar, 86.0, 86.0 },
        { "X16", EngineLayout::custom, 16, 0.0, { -135.0, -45.0, 45.0, 135.0 }, EngineUse::sportCar, 86.0, 86.0 },
        { "W8", EngineLayout::custom, 8, 0.0, { -52.5, -37.5, 37.5, 52.5 }, EngineUse::sportCar, 84.0, 90.0 },
        { "W12", EngineLayout::custom, 12, 0.0, { -60.0, 0.0, 60.0 }, EngineUse::sportCar, 84.0, 90.0 },
        { "V4 motorcycle", EngineLayout::vLayout, 4, 65.0, {}, EngineUse::sportMotorcycle, 81.0, 53.5 },
    };
    const std::array<std::pair<ExhaustSystem, const char*>, 5> systems { {
        { ExhaustSystem::automatic, "automatic" },
        { ExhaustSystem::single, "single" },
        { ExhaustSystem::dual, "dual" },
        { ExhaustSystem::dualHPipe, "H-pipe" },
        { ExhaustSystem::dualXPipe, "X-pipe" },
    } };
    std::vector<EngineConfig> configs;
    for (const auto& shape : shapes)
        for (const auto& [system, systemName] : systems)
            for (const auto induction : { Induction::natural, Induction::turbocharged }) {
                if (induction == Induction::turbocharged && system != ExhaustSystem::single
                    && system != ExhaustSystem::automatic)
                    continue;
                EngineSpec spec;
                spec.name = std::string("Designed ") + shape.name;
                spec.use = shape.use;
                spec.layout = shape.layout;
                spec.cylinderCount = shape.cylinders;
                spec.bankAngleDegrees = shape.bankAngle;
                spec.bankAnglesDegrees = shape.banks;
                spec.boreMm = shape.bore;
                spec.strokeMm = shape.stroke;
                spec.induction = induction;
                spec.exhaustSystem = system;
                auto designed = designCatalogueEngine(spec, catalogRoot);
                if (!designed.config) continue;
                designed.config->name = std::string(shape.name) + " " + systemName
                    + (induction == Induction::turbocharged ? " turbo" : "");
                configs.push_back(std::move(*designed.config));
            }
    return configs;
}

int main(int argc, char** argv) {
    std::filesystem::path outDirectory;
    std::string filter;
    double crank = 0.0;
    std::filesystem::path catalogRoot = ENGINELAB_CATALOG_ROOT;
    std::filesystem::path savedFolder;
    bool designed = false;
    std::optional<TwoStrokeIntake> twoStrokeIntake;
    for (int i = 1; i + 1 < argc; i += 2) {
        const std::string_view key = argv[i];
        if (key == "--out") outDirectory = argv[i + 1];
        else if (key == "--filter") filter = argv[i + 1];
        else if (key == "--crank") crank = std::stod(argv[i + 1]);
        else if (key == "--catalog-root") catalogRoot = argv[i + 1];
        else if (key == "--saved") savedFolder = argv[i + 1];
        else if (key == "--designed") designed = std::string_view(argv[i + 1]) == "all";
        else if (key == "--two-stroke-intake") {
            twoStrokeIntake = parseTwoStrokeIntake(argv[i + 1]);
            if (!twoStrokeIntake) {
                std::cerr << "--two-stroke-intake is crankcase_reed or cylinder_port\n";
                return 2;
            }
        }
    }
    if (outDirectory.empty()) {
        std::cerr << "usage: EngineLabSceneExport --out <directory> [--filter <substring>] [--crank <degrees>]"
                     " [--catalog-root <directory>] [--saved <folder>] [--designed all]"
                     " [--two-stroke-intake <crankcase_reed|cylinder_port>]\n";
        return 2;
    }
    const auto catalogue = loadEngineCatalog(catalogRoot);
    if (!catalogue.errors.empty()) {
        std::cerr << "catalogue errors\n";
        for (const auto& error : catalogue.errors) std::cerr << "  " << error << '\n';
        return 1;
    }
    std::vector<EngineConfig> configs;
    if (designed) {
        configs = designedEngines(catalogRoot);
    } else if (savedFolder.empty()) {
        for (const auto& entry : catalogue.entries) configs.push_back(entry.config);
    } else {
        const auto saved = loadSavedEngines(savedFolder, catalogRoot);
        for (const auto& error : saved.errors) std::cerr << "  " << error << '\n';
        for (const auto& entry : saved.engines) configs.push_back(entry.config);
    }
    std::filesystem::create_directories(outDirectory);
    std::ofstream index(outDirectory / "index.json", std::ios::binary);
    index << '[';
    int written = 0;
    std::array<std::size_t, 5> totals {};
    std::array<float, 5> worst {};
    std::array<std::size_t, 3> counts {};
    float worstAsymmetry = 0.0F;
    std::size_t implausible = 0;
    for (std::size_t e = 0; e < configs.size(); ++e) {
        auto config = configs[e];
        if (twoStrokeIntake && config.cycle == EngineCycle::twoStroke) config.twoStroke.intake = *twoStrokeIntake;
        if (!filter.empty() && config.name.find(filter) == std::string::npos) continue;
        const auto file = std::to_string(e + 1U) + ".json";
        auto sceneWritten = false;
        PlausibilityReport plausibility;
        const auto report = writeEngine(config, crank, outDirectory / file, plausibility, sceneWritten);
        if (!sceneWritten) {
            std::cerr << "cannot write " << (outDirectory / file).string() << '\n';
            return 1;
        }
        if (written++ > 0) index << ',';
        index << "{\"name\":\"" << escape(config.name) << "\",\"file\":\"" << file << "\"}";
        char line[256];
        std::snprintf(line, sizeof(line), "%-8s %-46s clash %3zu  self %3zu  engine %3zu  bend %3zu  stretch %3zu\n",
                      file.c_str(), config.name.c_str(), report.count(RouteIssueKind::ductClash),
                      report.count(RouteIssueKind::selfClash), report.count(RouteIssueKind::engineClash),
                      report.count(RouteIssueKind::tightBend), report.count(RouteIssueKind::stretched));
        std::cout << line;
        for (std::size_t k = 0; k < totals.size(); ++k) totals[k] += report.count(static_cast<RouteIssueKind>(k));
        const auto& p = plausibility;
        char asymmetry[32] = "-";
        if (p.mirrored) std::snprintf(asymmetry, sizeof(asymmetry), "%.0f", static_cast<double>(p.asymmetryMm));
        std::snprintf(line, sizeof(line),
                      "         plausibility: beside %5.0f  below %5.0f  above %5.0f  ahead %5.0f  behind %5.0f"
                      "  coiled %2zu  astray %2zu  asymmetry %5s  folded %2zu%s\n",
                      static_cast<double>(p.besideMm), static_cast<double>(p.belowMm),
                      static_cast<double>(p.aboveMm), static_cast<double>(p.aheadMm),
                      static_cast<double>(p.behindMm), p.coiled, p.astray, asymmetry, p.folded,
                      p.plausible() ? "" : "  IMPLAUSIBLE");
        std::cout << line;
        worst = { std::max(worst[0], p.besideMm), std::max(worst[1], p.belowMm), std::max(worst[2], p.aboveMm),
                  std::max(worst[3], p.aheadMm), std::max(worst[4], p.behindMm) };
        counts[0] += p.coiled;
        counts[1] += p.astray;
        counts[2] += p.folded;
        if (p.mirrored) worstAsymmetry = std::max(worstAsymmetry, p.asymmetryMm);
        if (!p.plausible()) ++implausible;
    }
    index << "]\n";
    index.close();
    // An index that was never written was reported as a success (audit RT-06).
    if (!index) {
        std::cerr << "cannot write " << (outDirectory / "index.json").string() << '\n';
        return 1;
    }
    std::cout << "total: clash " << totals[0] << ", self " << totals[1] << ", engine " << totals[2] << ", bend "
              << totals[3] << ", stretch " << totals[4] << '\n';
    std::cout << "plausibility: " << implausible << " of " << written << " implausible; worst beside " << worst[0]
              << ", below " << worst[1] << ", above " << worst[2] << ", ahead " << worst[3] << ", behind "
              << worst[4] << " mm; coiled "
              << counts[0] << ", astray " << counts[1] << ", folded " << counts[2] << "; worst asymmetry "
              << worstAsymmetry << " mm\n";
    return 0;
}
