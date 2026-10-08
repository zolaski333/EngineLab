// A finite reservoir supplied to the second half of a conservative intake
// advance must contain the inventory left by the first half and intervening
// exhaust transfer, at the same averaged volume. Observe the actual boundaries on two
// catalogue engines; no expected torque, VE or sound is calibrated here.
#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/physics/SimplifiedGasolinePhysics.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>
#include <numbers>
#include <string>
#include <string_view>

namespace {
constexpr double stepSeconds = 1.0 / 240.0;
constexpr double extraInertiaKgM2 = 2.0;
constexpr double relativeInventoryTolerance = 1.0e-9;
constexpr double absoluteMassToleranceKg = 1.0e-12;
constexpr double absoluteEnergyToleranceJ = 1.0e-8;

void require(bool condition, std::string_view message) {
    if (condition) return;
    std::cerr << "FAILED: " << message << '\n';
    std::exit(EXIT_FAILURE);
}

[[nodiscard]] enginelab::EngineConfig loadFixture(
    const enginelab::EngineCatalogLoadResult& catalog, std::string_view filename) {
    const auto entry = std::find_if(catalog.entries.begin(), catalog.entries.end(),
        [filename](const enginelab::EngineCatalogEntry& candidate) {
            return candidate.sourcePath.filename().string() == filename;
        });
    require(entry != catalog.entries.end(), "the exact catalogue fixture must exist");
    return entry->config;
}

struct Result final {
    enginelab::IntakeSplitReservoirDiagnostics diagnostics;
    std::size_t framesWithObservations { 0 };
    std::size_t framesWithInterveningExhaust { 0 };
    std::uint64_t cylinderCommitFailures { 0 };
    std::uint64_t plenumCommitFailures { 0 };
    double minimumRpm { 0.0 };
    double maximumRpm { 0.0 };
    bool continuous { false };
};

[[nodiscard]] Result measure(const enginelab::EngineConfig& config, double targetRpm) {
    enginelab::SimpleEcuModel ecu;
    enginelab::SimplifiedGasolinePhysics physics;
    enginelab::FourStrokeEventGenerator events;
    auto exhaust = enginelab::ExhaustGraph::makeForEngine(config);
    // Keep the historical multirate reduction as a nonvacuous numerical
    // control. Production joint manifolds no longer average this history.
    enginelab::EngineSimulatorOptions options;
    options.intakeJointManifold = false;
    options.intakeCouplingIntervalSeconds = 400.0e-6;
    auto simulator = std::make_unique<enginelab::EngineSimulator>(
        config, ecu, physics, events, exhaust, options);
    simulator->setIntakeSplitReservoirDiagnosticsEnabled(true);
    auto previousExternalTorqueNm = 0.0;
    const auto inertia = enginelab::effectiveRotatingInertiaKgM2(config)
        + extraInertiaKgM2;
    Result result;
    result.minimumRpm = targetRpm;
    for (std::size_t step = 0; step < 300; ++step) {
        const auto& before = simulator->state();
        const auto targetOmega = targetRpm * 2.0 * std::numbers::pi / 60.0;
        const auto disturbanceTorqueNm = before.netTorqueNm - previousExternalTorqueNm;
        const auto motorTorqueNm = std::clamp(
            inertia * (targetOmega - before.angularVelocityRadPerSecond) / stepSeconds
                - disturbanceTorqueNm,
            -5'000.0, 5'000.0);
        enginelab::EngineControls controls;
        controls.ignitionEnabled = false;
        controls.starterEngaged = false;
        controls.throttle = 1.0;
        controls.externalTorqueNm = motorTorqueNm;
        controls.externalRotatingInertiaKgM2 = extraInertiaKgM2;
        const auto frame = simulator->step(stepSeconds, controls);
        previousExternalTorqueNm = motorTorqueNm;
        if (step < 180) continue;
        require(std::isfinite(frame.state.rpm), "the motored fixture must remain finite");
        result.minimumRpm = std::min(result.minimumRpm, frame.state.rpm);
        result.maximumRpm = std::max(result.maximumRpm, frame.state.rpm);
        const auto& observed = simulator->intakeSplitReservoirDiagnostics();
        if (observed.observationCount != 0) ++result.framesWithObservations;
        if (observed.interveningExhaustObservationCount != 0)
            ++result.framesWithInterveningExhaust;
        auto& total = result.diagnostics;
        total.observationCount += observed.observationCount;
        total.observedCylinderMask |= observed.observedCylinderMask;
        total.absoluteFirstHalfTransferredMassKg += observed.absoluteFirstHalfTransferredMassKg;
        total.interveningExhaustObservationCount += observed.interveningExhaustObservationCount;
        total.interveningExhaustCylinderMask |= observed.interveningExhaustCylinderMask;
        total.absoluteInterveningExhaustTransferredMassKg +=
            observed.absoluteInterveningExhaustTransferredMassKg;
        total.maximumSpeciesInventoryMismatchKg = std::max(
            total.maximumSpeciesInventoryMismatchKg, observed.maximumSpeciesInventoryMismatchKg);
        total.maximumEnergyInventoryMismatchJ = std::max(
            total.maximumEnergyInventoryMismatchJ, observed.maximumEnergyInventoryMismatchJ);
        total.maximumObservedSpeciesInventoryKg = std::max(
            total.maximumObservedSpeciesInventoryKg, observed.maximumObservedSpeciesInventoryKg);
        total.maximumObservedEnergyInventoryJ = std::max(
            total.maximumObservedEnergyInventoryJ, observed.maximumObservedEnergyInventoryJ);
        total.maximumVolumeMismatchM3 = std::max(
            total.maximumVolumeMismatchM3, observed.maximumVolumeMismatchM3);
        result.cylinderCommitFailures += frame.state.intakeCylinderTransferFailures;
        result.plenumCommitFailures += frame.state.intakePlenumTransferFailures;
    }
    const auto& observed = result.diagnostics;
    const auto massToleranceKg = absoluteMassToleranceKg
        + relativeInventoryTolerance * observed.maximumObservedSpeciesInventoryKg;
    const auto energyToleranceJ = absoluteEnergyToleranceJ
        + relativeInventoryTolerance * observed.maximumObservedEnergyInventoryJ;
    result.continuous = observed.maximumSpeciesInventoryMismatchKg <= massToleranceKg
        && observed.maximumEnergyInventoryMismatchJ <= energyToleranceJ
        && observed.maximumVolumeMismatchM3 <= 1.0e-18;
    std::cout << "engine=" << config.name << " target_rpm=" << targetRpm
              << " rpm_band=[" << result.minimumRpm << ',' << result.maximumRpm << ']'
              << " observed_frames=" << result.framesWithObservations
              << " observations=" << observed.observationCount
              << " observed_cylinders=" << std::popcount(observed.observedCylinderMask)
              << " absolute_first_half_transfer_kg=" << observed.absoluteFirstHalfTransferredMassKg
              << " intervening_exhaust_frames=" << result.framesWithInterveningExhaust
              << " intervening_exhaust_observations=" << observed.interveningExhaustObservationCount
              << " intervening_exhaust_cylinders="
              << std::popcount(observed.interveningExhaustCylinderMask)
              << " absolute_intervening_exhaust_transfer_kg="
              << observed.absoluteInterveningExhaustTransferredMassKg
              << " max_species_mismatch_kg=" << observed.maximumSpeciesInventoryMismatchKg
              << " mass_tolerance_kg=" << massToleranceKg
              << " max_energy_mismatch_J=" << observed.maximumEnergyInventoryMismatchJ
              << " energy_tolerance_J=" << energyToleranceJ
              << " max_volume_mismatch_m3=" << observed.maximumVolumeMismatchM3
              << " cylinder_commit_failures=" << result.cylinderCommitFailures
              << " plenum_commit_failures=" << result.plenumCommitFailures
              << " continuous=" << result.continuous << '\n';
    require(result.minimumRpm >= targetRpm * 0.98 && result.maximumRpm <= targetRpm * 1.02,
        "the motor must hold the intended physical operating point");
    require(result.framesWithObservations >= 30
                && observed.observationCount >= config.cylinders.size() * 30,
        "the continuity check must observe many real intake split advances");
    require(static_cast<std::size_t>(std::popcount(observed.observedCylinderMask))
                == config.cylinders.size() && config.cylinders.size() >= 2,
        "all cylinders of both distinct multi-cylinder fixtures must be observed");
    require(observed.absoluteFirstHalfTransferredMassKg > 1.0e-6,
        "the first intake half must actually transfer nonzero physical inventory");
    require(result.framesWithInterveningExhaust >= 10
                && observed.interveningExhaustObservationCount >= config.cylinders.size() * 5,
        "the continuity check must observe many intervening accepted exhaust exchanges");
    require(static_cast<std::size_t>(std::popcount(observed.interveningExhaustCylinderMask))
                == config.cylinders.size(),
        "intervening exhaust exchanges must be observed for every cylinder");
    require(observed.absoluteInterveningExhaustTransferredMassKg > 1.0e-8,
        "the intervening exhaust operator must actually change the finite inventory");
    return result;
}
} // namespace

int main(int argc, char** argv) {
    auto diagnosticOnly = false;
    for (int index = 1; index < argc; ++index) {
        require(std::string_view(argv[index]) == "--diagnostic-only",
            "the only supported option is --diagnostic-only");
        diagnosticOnly = true;
    }
    const auto catalog = enginelab::loadEngineCatalog(
        std::filesystem::path(ENGINELAB_CATALOG_ROOT));
    require(catalog.errors.empty(), "the catalogue must load without errors");
    std::cout << std::setprecision(12);
    const auto cp2 = measure(loadFixture(catalog, "11_yamaha_cp2_mt07_like.engine.yaml"), 5'000.0);
    const auto k20 = measure(loadFixture(catalog, "01_honda_k20a_like.engine.yaml"), 4'000.0);
    if (!diagnosticOnly) {
        require(cp2.continuous && k20.continuous,
            "the second intake half must retain both preceding conservative exchanges");
        require(cp2.cylinderCommitFailures == 0 && cp2.plenumCommitFailures == 0
                    && k20.cylinderCommitFailures == 0 && k20.plenumCommitFailures == 0,
            "the measured conservative exchanges must be accepted by their real reservoirs");
    }
    return EXIT_SUCCESS;
}
