// Writes the 3-D scene of catalogue engines as JSON, for inspecting the
// laid-out geometry outside the app (a three.js page, a script).
//
//   EngineLabSceneExport --out <directory> [--filter <substring>] [--crank <degrees>]
//
// One <index>.json per engine: every part (material, layers, posed transform,
// positions in mm, triangle indices), every duct (kind, ids, centreline), the
// port anchors; plus index.json listing the files.

#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/render/EngineModel3D.hpp>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <string_view>
#include <vector>

namespace {
using namespace enginelab;
using namespace enginelab::render;

std::string escape(std::string_view text) {
    std::string out;
    for (const auto c : text) {
        if (c == '"' || c == '\\') out += '\\';
        out += c;
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

void writeEngine(const EngineConfig& config, double crankDegrees, const std::filesystem::path& file) {
    const EngineModel3D model(config);
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
        out << ",\"centreline\":[";
        for (std::size_t i = 0; i < duct.centreline.size(); ++i) {
            if (i > 0) out << ',';
            writeVec(out, duct.centreline[i]);
        }
        out << "]}";
    }
    out << "],\"exhaustPorts\":[";
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
}
} // namespace

int main(int argc, char** argv) {
    std::filesystem::path outDirectory;
    std::string filter;
    double crank = 0.0;
    for (int i = 1; i + 1 < argc; i += 2) {
        const std::string_view key = argv[i];
        if (key == "--out") outDirectory = argv[i + 1];
        else if (key == "--filter") filter = argv[i + 1];
        else if (key == "--crank") crank = std::stod(argv[i + 1]);
    }
    if (outDirectory.empty()) {
        std::cerr << "usage: EngineLabSceneExport --out <directory> [--filter <substring>] [--crank <degrees>]\n";
        return 2;
    }
    const auto catalogue = loadEngineCatalog(ENGINELAB_CATALOG_ROOT);
    if (!catalogue.errors.empty()) {
        std::cerr << "catalogue errors\n";
        return 1;
    }
    std::filesystem::create_directories(outDirectory);
    std::ofstream index(outDirectory / "index.json", std::ios::binary);
    index << '[';
    int written = 0;
    for (std::size_t e = 0; e < catalogue.entries.size(); ++e) {
        const auto& config = catalogue.entries[e].config;
        if (!filter.empty() && config.name.find(filter) == std::string::npos) continue;
        const auto file = std::to_string(e + 1U) + ".json";
        writeEngine(config, crank, outDirectory / file);
        if (written++ > 0) index << ',';
        index << "{\"name\":\"" << escape(config.name) << "\",\"file\":\"" << file << "\"}";
        std::cout << file << "  " << config.name << '\n';
    }
    index << "]\n";
    return 0;
}
