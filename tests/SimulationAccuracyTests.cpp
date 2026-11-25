// The accuracy profiles (SimulationAccuracy.hpp): realtime is production,
// each finer profile is one level of a single refinement ladder, and a run
// gets the mesh it asks for rather than a silently clamped one.
#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/physics/SimplifiedGasolinePhysics.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>
#include <enginelab/simulation/SimulationAccuracy.hpp>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string>
#include <string_view>
#include <utility>

namespace {
using namespace enginelab;

void require(bool condition, std::string_view message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(EXIT_FAILURE);
    }
    std::cout << "ok: " << message << '\n';
}

[[nodiscard]] EngineConfig catalogueEngine(std::string_view key) {
    static const auto catalogue = loadEngineCatalog(ENGINELAB_CATALOG_ROOT);
    const auto selected = selectSingleEngineCatalogEntry(catalogue.entries, std::string(key));
    require(static_cast<bool>(selected), "the catalogue has the engine asked");
    auto config = selected.entry->config;
    normaliseEngineConfig(config);
    return config;
}

struct Rig final {
    explicit Rig(EngineConfig engine, EngineSimulatorOptions options = {})
        : config(std::move(engine)), exhaust(ExhaustGraph::makeForEngine(config)),
          simulator(std::make_unique<EngineSimulator>(config, ecu, physics, events, exhaust,
                                                      std::move(options))) {}
    EngineConfig config;
    SimpleEcuModel ecu;
    SimplifiedGasolinePhysics physics;
    FourStrokeEventGenerator events;
    ExhaustGraph exhaust;
    std::unique_ptr<EngineSimulator> simulator;
};

[[nodiscard]] std::unique_ptr<Rig> rigWithMesh(const EngineConfig& config, double cellM) {
    EngineSimulatorOptions options;
    options.exhaustTargetCellLengthM = cellM;
    return std::make_unique<Rig>(config, options);
}

void aFinerMeshIsTheMeshRun() {
    // Until 2026-11-25 the simulator held every request at 25 mm: a 12.5 mm
    // convergence run was the 25 mm run, bit for bit, and said nothing.
    const auto config = catalogueEngine("RD350LC");
    const auto coarse = rigWithMesh(config, 0.025);
    const auto fine = rigWithMesh(config, 0.0125);
    const auto& coarseLayout = coarse->simulator->exhaustLayout();
    const auto& fineLayout = fine->simulator->exhaustLayout();
    std::cout << "RD350LC exhaust: " << coarseLayout.totalCellCount() << " cells at 25 mm, "
              << fineLayout.totalCellCount() << " at 12.5 mm (shortest "
              << fineLayout.minimumCellLengthM() * 1'000.0 << " mm)\n";
    require(fineLayout.totalCellCount() > coarseLayout.totalCellCount(),
        "a 12.5 mm request cuts more cells than a 25 mm one");
    require(std::abs(fineLayout.effectiveTargetCellLengthM() - 0.0125) < 1.0e-12,
        "and the layout reads the 12.5 mm it was cut to");
    require(fineLayout.minimumCellLengthM() < 0.0125 + 1.0e-9,
        "no duct of it is coarser than asked");
    const auto floor = rigWithMesh(config, 0.001);
    require(std::abs(floor->simulator->exhaustLayout().effectiveTargetCellLengthM()
                     - EngineSimulator::finestExhaustCellLengthM) < 1.0e-12,
        "a request under the finest cell is held there, and reads so");
}

void theRealtimeProfileIsProduction() {
    for (const auto* key : { "RD350LC", "2JZ" }) {
        auto config = catalogueEngine(key);
        const auto before = config;
        EngineSimulatorOptions options;
        const auto resolution = applyAccuracyProfile(AccuracyProfile::realtime, config, options);
        require(!options.exhaustTargetCellLengthM && !options.maximumLowSpeedExhaustCouplingSeconds
                && !options.exhaustCouplingEverySubstep && !options.exhaustMaximumCourantNumber
                && !options.exhaustMaximumCellsPerDuct && !options.exhaustMaximumTotalCells
                && !options.intakeTargetCellLengthM && !options.intakeCouplingIntervalSeconds
                && !options.intakeMaximumCourantNumber && !options.intakeMaximumCellsPerRunner,
            "the realtime profile sets no option");
        require(config.solver.maximumCrankDegreesPerStep == before.solver.maximumCrankDegreesPerStep
                && config.solver.maximumMechanicalFrequencyHz
                    == before.solver.maximumMechanicalFrequencyHz,
            "nor the solver");
        require(resolution.exhaustCellLengthM == EngineSimulator::productionExhaustCellLengthM(config)
                && resolution.crankDegreesPerStep == config.solver.maximumCrankDegreesPerStep
                && resolution.exhaustCouplingSeconds == productionExhaustCouplingSeconds
                && !resolution.exhaustCouplingEverySubstep,
            "and reads production's values");
    }
}

void theProfilesRefineOneLadder() {
    for (const auto profile : { AccuracyProfile::realtime, AccuracyProfile::balanced,
                                AccuracyProfile::engineering, AccuracyProfile::highPrecision })
        require(accuracyProfileFromName(accuracyProfileName(profile)) == profile,
            "a profile's name reads back as the profile");
    require(!accuracyProfileFromName("fast"), "an unknown name is no profile");
    for (const auto* key : { "RD350LC", "2JZ" }) {
        const auto config = catalogueEngine(key);
        auto previous = accuracyResolution(AccuracyProfile::realtime, config);
        for (const auto profile : { AccuracyProfile::balanced, AccuracyProfile::engineering,
                                    AccuracyProfile::highPrecision }) {
            const auto next = accuracyResolution(profile, config);
            std::cout << key << ' ' << accuracyProfileName(profile) << ": exhaust "
                      << next.exhaustCellLengthM * 1'000.0 << " mm, coupling "
                      << (next.exhaustCouplingEverySubstep ? std::string("every substep")
                          : std::to_string(next.exhaustCouplingSeconds * 1.0e6) + " us")
                      << ", crank " << next.crankDegreesPerStep << " deg"
                      << (next.crankStepHeldBySolverCeiling ? " (held by the 100 kHz ceiling)" : "")
                      << ", intake " << next.intakeCellLengthM * 1'000.0 << " mm, Courant "
                      << next.exhaustCourantNumber << '\n';
            require(next.exhaustCellLengthM < previous.exhaustCellLengthM
                    && next.intakeCellLengthM < previous.intakeCellLengthM
                    && next.crankDegreesPerStep <= previous.crankDegreesPerStep
                    && (next.exhaustCouplingEverySubstep
                        || next.exhaustCouplingSeconds < previous.exhaustCouplingSeconds)
                    && next.intakeCouplingSeconds < previous.intakeCouplingSeconds
                    && next.exhaustCourantNumber <= previous.exhaustCourantNumber,
                "each profile is finer than the one before it in every resolution");
            previous = next;
        }
    }
}

void eachProfileGetsTheMeshItAsks() {
    for (const auto* key : { "RD350LC", "2JZ", "LS3" }) {
        for (const auto profile : { AccuracyProfile::balanced, AccuracyProfile::engineering,
                                    AccuracyProfile::highPrecision }) {
            auto config = catalogueEngine(key);
            EngineSimulatorOptions options;
            const auto resolution = applyAccuracyProfile(profile, config, options);
            const auto rig = std::make_unique<Rig>(config, options);
            const auto& layout = rig->simulator->exhaustLayout();
            std::cout << key << ' ' << accuracyProfileName(profile) << ": "
                      << layout.totalCellCount() << " exhaust cells at "
                      << layout.effectiveTargetCellLengthM() * 1'000.0 << " mm\n";
            require(std::abs(layout.effectiveTargetCellLengthM() - resolution.exhaustCellLengthM)
                        < 1.0e-12 && !layout.ductCellCapReached(),
                "the profile's exhaust mesh is the one it asked, no ceiling coarsening it");
        }
    }
}

void theFinestProfileRunsAndCouplesEverySubstep() {
    auto config = catalogueEngine("RD350LC");
    EngineSimulatorOptions options;
    const auto resolution = applyAccuracyProfile(AccuracyProfile::highPrecision, config, options);
    const auto rig = std::make_unique<Rig>(config, options);
    EngineState state;
    for (int frame = 0; frame < 48; ++frame)
        state = rig->simulator->step(1.0 / 240.0, { true, true, 0.3, 0.0 }).state;
    std::cout << "RD350LC high precision while cranking: " << state.rpm << " rpm, crank step "
              << state.crankDegreesPerSolverStep << " deg (asked " << resolution.crankDegreesPerStep
              << "), coupling " << state.exhaustCouplingFrequencyHz << " Hz, substeps "
              << state.solverFrequencyHz << " Hz\n";
    require(std::isfinite(state.rpm) && state.rpm > 0.0, "the finest profile cranks the engine");
    require(state.crankDegreesPerSolverStep <= resolution.crankDegreesPerStep + 1.0e-9,
        "at its crank step or finer");
    require(state.exhaustCouplingFrequencyHz > 0.9 * state.solverFrequencyHz,
        "and couples its exhaust on every substep");
}

void theExhaustLedgerCloses() {
    // The ledger reads the network; its error is what the network does not
    // conserve between what the cylinders put in and the outlets took out.
    for (const auto profile : { AccuracyProfile::realtime, AccuracyProfile::engineering }) {
        auto config = catalogueEngine("RD350LC");
        EngineSimulatorOptions options;
        (void)applyAccuracyProfile(profile, config, options);
        options.exhaustMassLedger = true;
        const auto rig = std::make_unique<Rig>(config, options);
        EngineState state;
        for (int frame = 0; frame < 2 * 240; ++frame) {
            const auto time = frame / 240.0;
            state = rig->simulator->step(1.0 / 240.0, { true, time < 1.5, 0.55, 0.0 }).state;
        }
        const auto ratio = state.exhaustLedgerErrorKg / state.exhaustLedgerThroughputKg;
        std::cout << "RD350LC " << accuracyProfileName(profile) << ", 2 s from a start: "
                  << state.exhaustLedgerThroughputKg * 1.0e3 << " g crossed the exhaust, ledger error "
                  << ratio << ", refused exchanges " << state.exhaustRejectedExchangeKg << " kg\n";
        require(state.exhaustLedgerThroughputKg > 0.0, "the ledger sees the exhaust flow");
        require(ratio < 1.0e-9, "and the network conserves the mass that crosses it");
    }
}

} // namespace

int main() {
    aFinerMeshIsTheMeshRun();
    theRealtimeProfileIsProduction();
    theProfilesRefineOneLadder();
    eachProfileGetsTheMeshItAsks();
    theFinestProfileRunsAndCouplesEverySubstep();
    theExhaustLedgerCloses();
    std::cout << "simulation accuracy tests passed\n";
    return EXIT_SUCCESS;
}
