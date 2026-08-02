/**
 * Delivered cyclic-variability instrument.
 *
 * The authored quantity is a burn-RATE dispersion; what an engineer, a listener
 * and the literature all talk about is COV(IMEP), the dispersion of the work a
 * cylinder actually produces. The two are not equal and their ratio is not
 * guessable: near MBT phasing, IMEP sits close to a maximum with respect to
 * burn rate, so it is markedly less dispersed than the rate that drives it.
 * That is exactly why the closure's coefficients have to be set from a measured
 * mapping instead of assumed.
 *
 * This walks the catalogue and reports, per engine and per condition, the
 * per-cylinder COV of `CylinderState::indicatedWorkJoulesPerCycle` -- which is
 * COV(IMEP), since displacement is constant -- plus the effective burn-rate
 * dispersion the closure produced. Cylinder-to-cylinder scatter is excluded by
 * computing each cylinder's COV about its OWN mean and then averaging: a
 * genuinely uneven engine must not be reported as a cyclically variable one.
 *
 * Two conditions, because a single one cannot show the dependence that is the
 * whole point of the closure. BOTH hold the speed with the absorber:
 *
 *   * `light` - part throttle at a low held speed. Low manifold pressure means
 *               a large trapped residual, so this is the diluted, rough end --
 *               the same charge condition an idle has.
 *   * `wot`   - wide open at a mid-range hold. Clean, weakly diluted charge,
 *               so this is the smooth end.
 *
 * A FREE idle was tried first and is not a valid instrument here. Its
 * COV(IMEP) is dominated by the governor hunting, not by combustion: the same
 * run reported 90 % on the radial and 50 % on the EJ25, which is a control-loop
 * oscillation being read as cyclic variability. Holding the speed removes the
 * loop and leaves the charge, which is what the closure is about. Idle
 * STABILITY is a separate question with its own instrument
 * (`EngineLab.IdleStabilityRegression`) and must not be conflated with this one.
 *
 * Reference bands come from published SI-engine cyclic-variability work, never
 * from this simulator: roughly 1-3 % COV(IMEP) for a warm engine at high load,
 * rising to the mid single digits at an ordinary production idle, with the 5-10 %
 * region being where a driver starts to feel it. Compression ignition is
 * markedly steadier because injection, not a flame kernel, controls ignition.
 *
 * `--enforce` turns the bands into a gate. Without it the tool only reports,
 * which is what calibration needs.
 */

#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/physics/SimplifiedGasolinePhysics.hpp>
#include <enginelab/simulation/DynoAbsorberController.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <string>
#include <string_view>
#include <vector>

namespace {
constexpr double dt = 1.0 / 240.0;

struct CylinderSeries final {
    double lastWork { 0.0 };
    double sum { 0.0 };
    double sumSquares { 0.0 };
    std::size_t count { 0 };

    void observe(double work) noexcept {
        // The published value only changes at a cycle boundary, so a change is
        // the cycle event. Sampling every frame instead would count the same
        // cycle many times and collapse the variance toward zero.
        if (std::abs(work - lastWork) < 1.0e-12) return;
        lastWork = work;
        if (!(work > 0.0) || !std::isfinite(work)) return;
        sum += work;
        sumSquares += work * work;
        ++count;
    }
    [[nodiscard]] double coefficientOfVariation() const noexcept {
        if (count < 8) return std::numeric_limits<double>::quiet_NaN();
        const auto n = static_cast<double>(count);
        const auto mean = sum / n;
        if (!(mean > 0.0)) return std::numeric_limits<double>::quiet_NaN();
        const auto variance = std::max(0.0,
            (sumSquares - n * mean * mean) / (n - 1.0));
        return std::sqrt(variance) / mean;
    }
};

struct ConditionResult final {
    double covImep { std::numeric_limits<double>::quiet_NaN() };
    double burnRateDispersion { 0.0 };
    double burnedFraction { 0.0 };
    double rpm { 0.0 };
    /** Coefficient of variation of the HELD speed over the sampling window.
     *  Both conditions are speed-held, so this is how a reader tells a genuine
     *  combustion dispersion from the absorber's own control loop swinging the
     *  operating point. A COV(IMEP) that moves with this one is an artefact. */
    double rpmDispersion { 0.0 };
    std::size_t cycles { 0 };
};

[[nodiscard]] ConditionResult measure(enginelab::EngineConfig config,
                                      bool light, double authoredCov) {
    if (authoredCov >= 0.0)
        config.combustionCalibration.cycleVariationCoefficientOfVariation =
            authoredCov;
    enginelab::SimpleEcuModel ecu;
    enginelab::SimplifiedGasolinePhysics physics;
    enginelab::FourStrokeEventGenerator events;
    auto exhaust = enginelab::ExhaustGraph::makeForEngine(config);
    enginelab::EngineSimulator simulator(config, ecu, physics, events, exhaust);

    // Start on the normal path first; the condition is imposed afterwards.
    for (int step = 0; step < static_cast<int>(3.0 / dt); ++step) {
        const auto time = static_cast<double>(step) * dt;
        (void)simulator.step(dt, { true, time < 1.6, 0.55, 0.0 });
    }

    const auto ceilingRpm =
        std::min(config.redlineRpm, config.ignition.revLimitRpm);
    // The light point sits just above idle so the throttle plate is nearly
    // shut and the trapped residual is large; the full-load point sits in the
    // mid range where the engine is genuinely breathing.
    const auto holdRpm = light
        ? std::clamp(config.idleRpm * 1.35, 700.0, 0.30 * ceilingRpm)
        : std::min(0.62 * ceilingRpm, 4'500.0);
    enginelab::DynoAbsorberController absorber(config);
    absorber.reset(simulator.state().rpm, simulator.state().torqueNm);

    // The hold and the wall film must settle before anything is counted; a
    // transient counted as dispersion would be an artefact of the approach.
    const auto settleSeconds = 7.0;
    const auto sampleSeconds = light ? 12.0 : 6.0;
    const auto settleSteps = static_cast<int>(settleSeconds / dt);
    const auto totalSteps =
        settleSteps + static_cast<int>(sampleSeconds / dt);

    std::vector<CylinderSeries> series(config.cylinders.size());
    ConditionResult result;
    double dispersionSum = 0.0;
    double burnedSum = 0.0;
    double rpmSum = 0.0;
    double rpmSquaredSum = 0.0;
    std::size_t frames = 0;
    for (int step = 0; step < totalSteps; ++step) {
        enginelab::EngineControls controls;
        controls.ignitionEnabled = true;
        controls.starterEngaged = false;
        controls.throttle = light ? 0.10 : 1.0;
        const auto output = absorber.advance(dt, holdRpm, simulator.state());
        controls.dynamometerTorqueNm = output.brakeTorqueNm;
        const auto frame = simulator.step(dt, controls);
        if (step < settleSteps) continue;
        for (std::size_t index = 0;
             index < frame.state.cylinderStateCount && index < series.size();
             ++index) {
            series[index].observe(
                frame.state.cylinderStates[index].indicatedWorkJoulesPerCycle);
            dispersionSum += std::abs(
                frame.state.cylinderStates[index].combustionCycleMultiplier
                - 1.0);
            burnedSum +=
                frame.state.cylinderStates[index].residualGasFractionAtSpark;
        }
        rpmSum += frame.state.rpm;
        rpmSquaredSum += frame.state.rpm * frame.state.rpm;
        ++frames;
    }
    if (frames == 0) return result;
    const auto cylinderCount = static_cast<double>(series.size());
    const auto frameCount = static_cast<double>(frames);
    result.rpm = rpmSum / frameCount;
    if (frames > 1 && result.rpm > 0.0) {
        const auto variance = std::max(0.0,
            (rpmSquaredSum - frameCount * result.rpm * result.rpm)
                / (frameCount - 1.0));
        result.rpmDispersion = std::sqrt(variance) / result.rpm;
    }
    result.burnRateDispersion =
        dispersionSum / (static_cast<double>(frames) * cylinderCount);
    result.burnedFraction =
        burnedSum / (static_cast<double>(frames) * cylinderCount);

    double covSum = 0.0;
    std::size_t covCount = 0;
    for (const auto& cylinder : series) {
        const auto cov = cylinder.coefficientOfVariation();
        if (!std::isfinite(cov)) continue;
        covSum += cov;
        ++covCount;
        result.cycles = std::max(result.cycles, cylinder.count);
    }
    if (covCount > 0) result.covImep = covSum / static_cast<double>(covCount);
    return result;
}
} // namespace

int main(int argc, char** argv) {
    auto enforce = false;
    std::string filter;
    std::string catalogRoot = ENGINELAB_CATALOG_ROOT;
    // Negative means "use whatever the engine file authored". An explicit
    // value overrides `cycle_variation_cov`, so an A/B against the authored
    // dispersion is one binary in one session rather than two builds compared
    // across time -- the protocol this project requires.
    auto authoredCov = -1.0;
    for (int index = 1; index < argc; ++index) {
        const std::string_view argument { argv[index] };
        if (argument == "--enforce") enforce = true;
        else if (argument == "--filter" && index + 1 < argc)
            filter = argv[++index];
        else if (argument == "--catalog-root" && index + 1 < argc)
            catalogRoot = argv[++index];
        else if (argument == "--cov" && index + 1 < argc)
            authoredCov = std::atof(argv[++index]);
    }

    const auto catalog = enginelab::loadEngineCatalog(catalogRoot);
    if (!catalog.errors.empty()) {
        for (const auto& error : catalog.errors)
            std::cerr << "catalog: " << error << '\n';
        return EXIT_FAILURE;
    }

    std::cout << std::left << std::setw(34) << "engine"
              << std::right << std::setw(5) << "cyl"
              << std::setw(10) << "light_rpm"
              << std::setw(11) << "light_COV%"
              << std::setw(10) << "light_xb"
              << std::setw(9) << "wot_rpm"
              << std::setw(10) << "wot_COV%"
              << std::setw(9) << "wot_xb"
              << std::setw(9) << "verdict" << '\n';

    auto failures = 0;
    auto measured = 0;
    for (const auto& entry : catalog.entries) {
        if (!filter.empty()
            && entry.config.name.find(filter) == std::string::npos)
            continue;
        const auto idleResult = measure(entry.config, true, authoredCov);
        const auto wotResult = measure(entry.config, false, authoredCov);
        ++measured;

        const auto diesel = entry.config.fuel == enginelab::FuelType::diesel;
        // Literature bands, not simulator output. A production SI engine is a
        // few per cent at high load and mid single digits at idle; compression
        // ignition is steadier. The upper idle bound is deliberately loose
        // because a genuinely rough idle is a legitimate result -- what is not
        // legitimate is a perfectly repeating cycle, which is the lower bound.
        const auto idleFloor = diesel ? 0.002 : 0.008;
        const auto idleCeiling = diesel ? 0.060 : 0.120;
        const auto wotFloor = diesel ? 0.001 : 0.003;
        const auto wotCeiling = diesel ? 0.030 : 0.045;
        const auto idleOk = std::isfinite(idleResult.covImep)
            && idleResult.covImep >= idleFloor
            && idleResult.covImep <= idleCeiling;
        const auto wotOk = std::isfinite(wotResult.covImep)
            && wotResult.covImep >= wotFloor
            && wotResult.covImep <= wotCeiling;
        // The dependence itself is the claim: the same engine must be rougher
        // on a diluted idle charge than on a clean full-load one. A closure
        // that produced one flat number would satisfy both bands and still be
        // the constant the catalogue was right to refuse.
        const auto orderedOk = !std::isfinite(idleResult.covImep)
            || !std::isfinite(wotResult.covImep)
            || diesel
            || idleResult.covImep > wotResult.covImep;
        const auto ok = idleOk && wotOk && orderedOk;
        if (!ok) ++failures;

        std::cout << std::left << std::setw(34)
                  << entry.config.name.substr(0, 33)
                  << std::right << std::setw(5) << entry.config.cylinders.size()
                  << std::fixed << std::setprecision(0)
                  << std::setw(10) << idleResult.rpm
                  << std::setprecision(2)
                  << std::setw(11) << idleResult.covImep * 100.0
                  << std::setprecision(3)
                  << std::setw(10) << idleResult.burnedFraction
                  << std::setprecision(2)
                  << std::setw(11) << idleResult.rpmDispersion * 100.0
                  << std::setprecision(0)
                  << std::setw(9) << wotResult.rpm
                  << std::setprecision(2)
                  << std::setw(10) << wotResult.covImep * 100.0
                  << std::setprecision(3)
                  << std::setw(9) << wotResult.burnedFraction
                  << std::setprecision(2)
                  << std::setw(9) << wotResult.rpmDispersion * 100.0
                  << std::setw(9) << (ok ? "OK" : "OUT") << '\n';
    }

    std::cout << "measured " << measured << " engines, " << failures
              << " outside the literature bands\n";
    if (enforce && (failures > 0 || measured == 0)) return EXIT_FAILURE;
    return EXIT_SUCCESS;
}
