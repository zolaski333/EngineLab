// Why an authored exhaust costs what it costs.
//
// A user reported that applying ANY network from the exhaust designer -- any
// diameter, any topology, any complexity -- drops the realtime factor to about
// 0.5, while the shipped catalogue runs at 1.0. That is too systematic to be a
// property of the geometries they chose, so the question this instrument
// answers is deliberately narrow:
//
//   for the SAME physical geometry, does expressing it as a designer component
//   network cost more solver time than the shipped legacy path?
//
// If it does, the cost is in the representation and not in what the user drew,
// which is exactly the failure their report describes.
//
// The geometry number that decides it is the minimum of every duct dx and every
// junction V/sum(A_port). The explicit time step divides that scale by c+|u|,
// so ONE small component sets the substep rate for every duct. A ratio of two
// limiting lengths is the static solver-cost ratio at the same gas state.

#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/gasdynamics/ExhaustNetworkLayout.hpp>

#include <algorithm>
#include <array>
#include <filesystem>
#include <utility>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

namespace {

using enginelab::EngineConfig;
using enginelab::ExhaustComponentConfig;
using enginelab::ExhaustComponentType;
using enginelab::ExhaustGraph;
using enginelab::ExhaustNetworkConfig;
using enginelab::gasdynamics::ExhaustNetworkLayout;

struct Cost final {
    std::size_t ducts { 0 };
    std::size_t junctions { 0 };
    std::size_t cells { 0 };
    double limitingCflLengthM { 0.0 };
    bool valid { false };
};

[[nodiscard]] Cost measure(const EngineConfig& config) {
    Cost cost;
    const auto graph = ExhaustGraph::makeForEngine(config);
    const auto layout = ExhaustNetworkLayout::compile(
        graph, enginelab::gasdynamics::realtimeExhaustFeedbackDiscretisation());
    cost.valid = layout.valid();
    cost.ducts = layout.ducts().size();
    cost.junctions = layout.junctions().size();
    for (const auto& duct : layout.ducts()) cost.cells += duct.cellCount;
    cost.limitingCflLengthM = layout.minimumCflLengthM();
    return cost;
}

/** Lengths a user gives the four elements of an ordinary system, in mm. Zero
 *  means "left blank", which is the case that matters: ExhaustNetworkLayout
 *  floors a missing length with minimumResolvedLengthM, and EngineSimulator
 *  never overrides its 5 mm default. */
struct DesignerLengths final {
    double collectorMm { 120.0 };
    double mufflerMm { 450.0 };
    double outletMm { 400.0 };
};

/** The network the designer produces for a plain header/collector/muffler
 *  exhaust, built from the engine's OWN legacy geometry so the physical pipe
 *  work is identical to what the engine already runs. Any difference measured
 *  afterwards is therefore a cost of the representation, not of the design. */
[[nodiscard]] EngineConfig asDesignerNetwork(const EngineConfig& source,
                                             const DesignerLengths& lengths) {
    auto config = source;
    const auto& geometry = source.exhaust;
    config.exhaustPaths.clear();
    enginelab::ExhaustPathConfig path;
    path.id = 1;
    path.geometry = geometry;
    for (const auto& cylinder : config.cylinders) path.cylinderIds.push_back(cylinder.id);

    ExhaustNetworkConfig network;
    std::uint32_t nextId = 1;
    const auto addComponent = [&](ExhaustComponentType type, double lengthMm,
                                  double diameterMm) {
        ExhaustComponentConfig component;
        component.id = nextId++;
        component.type = type;
        component.lengthMm = lengthMm;
        component.diameterMm = diameterMm;
        network.components.push_back(component);
        return component.id;
    };

    // One primary per cylinder, merged at a collector, then the silencer body
    // and the tailpipe: the canonical thing a user draws.
    std::vector<std::uint32_t> primaries;
    primaries.reserve(config.cylinders.size());
    for (std::size_t index = 0; index < config.cylinders.size(); ++index)
        primaries.push_back(addComponent(ExhaustComponentType::pipe,
            geometry.primaryLengthMm, geometry.primaryDiameterMm));
    const auto collector = addComponent(ExhaustComponentType::merge,
        lengths.collectorMm, geometry.collectorDiameterMm);
    const auto mufflerDiameter = geometry.mufflerChamberDiameterMm > 1.0
        ? geometry.mufflerChamberDiameterMm : geometry.collectorDiameterMm;
    const auto muffler = addComponent(ExhaustComponentType::muffler,
        lengths.mufflerMm, mufflerDiameter);
    const auto outlet = addComponent(ExhaustComponentType::outlet,
        lengths.outletMm, geometry.outletDiameterMm);

    for (const auto primary : primaries)
        network.connections.push_back(
            enginelab::ExhaustComponentConnectionConfig { primary, collector });
    for (std::size_t index = 0; index < primaries.size(); ++index)
        network.cylinderConnections.push_back(
            enginelab::ExhaustCylinderConnectionConfig {
                config.cylinders[index].id, primaries[index] });
    network.connections.push_back(
        enginelab::ExhaustComponentConnectionConfig { collector, muffler });
    network.connections.push_back(
        enginelab::ExhaustComponentConnectionConfig { muffler, outlet });
    path.network = std::move(network);
    config.exhaustPaths.push_back(std::move(path));
    return config;
}

void report(const std::string& label, const Cost& cost) {
    std::cout << "  " << std::left << std::setw(22) << label << std::right
              << " ducts " << std::setw(3) << cost.ducts
              << "  junctions " << std::setw(3) << cost.junctions
              << "  cells " << std::setw(4) << cost.cells
              << "  longueur_CFL " << std::setw(8) << std::fixed
              << std::setprecision(2) << cost.limitingCflLengthM * 1'000.0 << " mm"
              << (cost.valid ? "" : "  [LAYOUT INVALIDE]") << '\n';
}

} // namespace

int main(int argc, char** argv) {
    std::filesystem::path catalogRoot = ENGINELAB_CATALOG_ROOT;
    std::string filter;
    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        if (argument == "--catalog-root" && index + 1 < argc) catalogRoot = argv[++index];
        else if (argument == "--filter" && index + 1 < argc) filter = argv[++index];
        else {
            std::cout << "usage: " << argv[0]
                      << " [--catalog-root path] [--filter name-fragment]\n";
            return argument == "--help" ? 0 : 2;
        }
    }

    const auto catalog = enginelab::loadEngineCatalog(catalogRoot);
    if (catalog.entries.empty()) {
        std::cerr << "catalogue vide sous " << catalogRoot << '\n';
        return 1;
    }

    std::cout << "Cout solveur d'echappement: reseau livre contre le MEME "
                 "reseau exprime en composants.\n"
                 "La longueur CFL inclut dx des conduits et V/somme(A) des "
                 "jonctions.\n\n";

    auto worstRatio = 0.0;
    std::string worstEngine;
    for (const auto& entry : catalog.entries) {
        if (!filter.empty()
            && entry.config.name.find(filter) == std::string::npos) continue;
        const auto shipped = measure(entry.config);
        std::cout << "== " << entry.config.name << '\n';
        report("livre (legacy)", shipped);
        const std::array<std::pair<const char*, DesignerLengths>, 3> variants { {
            { "concepteur, cote", DesignerLengths { 120.0, 450.0, 400.0 } },
            { "concepteur, court", DesignerLengths { 60.0, 200.0, 150.0 } },
            { "concepteur, sortie vide", DesignerLengths { 120.0, 450.0, 0.0 } },
        } };
        for (const auto& [label, lengths] : variants) {
            const auto designed = measure(asDesignerNetwork(entry.config, lengths));
            report(label, designed);
            if (!(shipped.limitingCflLengthM > 0.0)
                || !(designed.limitingCflLengthM > 0.0))
                continue;
            const auto ratio = shipped.limitingCflLengthM
                / designed.limitingCflLengthM;
            std::cout << "        -> cout solveur x" << std::fixed
                      << std::setprecision(2) << ratio << '\n';
            if (ratio > worstRatio) {
                worstRatio = ratio;
                worstEngine = std::string(entry.config.name) + " / " + label;
            }
        }
        std::cout << '\n';
    }
    if (!worstEngine.empty())
        std::cout << "pire cas: " << worstEngine << " x" << std::fixed
                  << std::setprecision(2) << worstRatio << '\n';
    return 0;
}
