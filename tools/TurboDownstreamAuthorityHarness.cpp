/**
 * Does downstream exhaust geometry reach a turbocharged engine at all?
 *
 * The user-visible complaint this exists to close: on a turbo engine, enlarging
 * the exhaust changed nothing useful and could only lose power. Reading the
 * model showed why. The turbine was expanded to AMBIENT
 *
 *     expansionRatio = p_turbine_inlet / p_ambient
 *
 * so the pressure downstream of the turbine -- the one thing a downpipe and a
 * silencer actually set -- was absent from the shaft power by construction. The
 * only remaining path from exhaust geometry to performance was the inlet term,
 * and that one is NEGATIVE: larger primaries expand the blowdown pulse into
 * more volume and lower the peak the turbine is charged with. So the model could
 * only ever answer "bigger exhaust, less boost", which is backwards.
 *
 * This sweeps the downstream conductance of a catalogue turbo at one held
 * full-load point and reports what it reaches. The gate is deliberately about
 * SIGN and MONOTONICITY, not about a magnitude: an absolute boost figure for an
 * arbitrary downpipe would have to be calibrated against a dyno this project
 * does not have, whereas the direction is not in doubt anywhere in the
 * turbocharging literature -- lowering turbine outlet pressure raises the
 * expansion ratio, and a bigger downpipe is the standard way of doing it.
 *
 * A naturally aspirated engine is measured with the same sweep as a control. It
 * has no turbine, so its response comes only from pumping work and must stay
 * far smaller than the turbo's; if the two move together, the instrument is
 * measuring the sweep and not the turbine.
 */

#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/physics/SimplifiedGasolinePhysics.hpp>
#include <enginelab/simulation/DynoAbsorberController.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <string>
#include <string_view>
#include <vector>

namespace {
constexpr double dt = 1.0 / 240.0;

struct Point final {
    double outletDiameterMm { 0.0 };
    double rpm { 0.0 };
    double boostRatio { 0.0 };
    double turbineOutletKpa { 0.0 };
    double backPressureKpa { 0.0 };
    double manifoldKpa { 0.0 };
    double wastegateOpening { 0.0 };
    double torqueNm { 0.0 };
};

[[nodiscard]] Point measure(enginelab::EngineConfig config,
                            double outletDiameterMm, double holdRpm,
                            bool openWastegateLoop) {
    // Raising the boost target out of the compressor's reach parks the
    // wastegate shut, so every watt the turbine gains has to appear as boost.
    // Without this the sweep measures a REGULATOR, not a turbine: a wastegated
    // engine on target answers extra turbine power by opening the gate further
    // and holding the same manifold pressure, which is exactly what a wastegate
    // is for and exactly what the first version of this harness mistook for the
    // model still being deaf to its exhaust.
    if (openWastegateLoop) {
        config.forcedInduction.wastegatePressureRatio =
            config.forcedInduction.pressureRatio + 1.5;
    }
    // Only the DOWNSTREAM side moves. Primaries and collector are left alone on
    // purpose: changing them would also move the blowdown pulse the turbine is
    // charged with, and the question here is specifically whether what happens
    // AFTER the turbine reaches the engine.
    config.exhaust.outletDiameterMm = outletDiameterMm;
    if (config.exhaust.mufflerChamberDiameterMm > 1.0)
        config.exhaust.mufflerChamberDiameterMm = std::max(
            config.exhaust.mufflerChamberDiameterMm,
            outletDiameterMm * 1.5);
    enginelab::normaliseEngineConfig(config);

    enginelab::SimpleEcuModel ecu;
    enginelab::SimplifiedGasolinePhysics physics;
    enginelab::FourStrokeEventGenerator events;
    auto exhaust = enginelab::ExhaustGraph::makeForEngine(config);
    enginelab::EngineSimulator simulator(config, ecu, physics, events, exhaust);
    for (int step = 0; step < static_cast<int>(3.0 / dt); ++step) {
        const auto time = static_cast<double>(step) * dt;
        (void)simulator.step(dt, { true, time < 1.6, 0.55, 0.0 });
    }

    enginelab::DynoAbsorberController absorber(config);
    absorber.reset(simulator.state().rpm, simulator.state().torqueNm);
    const auto settleSteps = static_cast<int>(9.0 / dt);
    const auto sampleSteps = static_cast<int>(3.0 / dt);
    Point point { outletDiameterMm };
    for (int step = 0; step < settleSteps + sampleSteps; ++step) {
        const auto output = absorber.advance(dt, holdRpm, simulator.state());
        enginelab::EngineControls controls;
        controls.ignitionEnabled = true;
        controls.throttle = 1.0;
        controls.dynamometerTorqueNm = output.brakeTorqueNm;
        const auto frame = simulator.step(dt, controls);
        if (step < settleSteps) continue;
        point.rpm += frame.state.rpm;
        point.boostRatio += frame.state.boostPressureRatio;
        point.turbineOutletKpa += frame.state.turbineOutletPressureKpa;
        point.backPressureKpa += frame.state.exhaustBackPressureKpa;
        point.manifoldKpa += frame.state.manifoldPressureKpa;
        point.wastegateOpening += frame.state.wastegateOpening;
        point.torqueNm += frame.state.cycleAveragedTorqueNm;
    }
    const auto divisor = static_cast<double>(sampleSteps);
    point.rpm /= divisor;
    point.boostRatio /= divisor;
    point.turbineOutletKpa /= divisor;
    point.backPressureKpa /= divisor;
    point.manifoldKpa /= divisor;
    point.wastegateOpening /= divisor;
    point.torqueNm /= divisor;
    return point;
}

void report(const std::string& name, const std::vector<Point>& points) {
    std::cout << "\n" << name << '\n';
    std::cout << std::right << std::setw(11) << "outlet_mm"
              << std::setw(9) << "rpm"
              << std::setw(9) << "boost"
              << std::setw(13) << "turb_out_kPa"
              << std::setw(12) << "backP_kPa"
              << std::setw(11) << "MAP_kPa"
              << std::setw(8) << "wg"
              << std::setw(12) << "torque_Nm" << '\n';
    for (const auto& point : points) {
        std::cout << std::fixed << std::setprecision(1)
                  << std::setw(11) << point.outletDiameterMm
                  << std::setprecision(0) << std::setw(9) << point.rpm
                  << std::setprecision(3) << std::setw(9) << point.boostRatio
                  << std::setprecision(1)
                  << std::setw(13) << point.turbineOutletKpa
                  << std::setw(12) << point.backPressureKpa
                  << std::setw(11) << point.manifoldKpa
                  << std::setprecision(3) << std::setw(8)
                  << point.wastegateOpening
                  << std::setprecision(2) << std::setw(12) << point.torqueNm
                  << '\n';
    }
}
} // namespace

int main(int argc, char** argv) {
    auto enforce = false;
    std::string catalogRoot = ENGINELAB_CATALOG_ROOT;
    for (int index = 1; index < argc; ++index) {
        const std::string_view argument { argv[index] };
        if (argument == "--enforce") enforce = true;
        else if (argument == "--catalog-root" && index + 1 < argc)
            catalogRoot = argv[++index];
    }
    const auto catalog = enginelab::loadEngineCatalog(catalogRoot);
    if (!catalog.errors.empty()) {
        for (const auto& error : catalog.errors)
            std::cerr << "catalog: " << error << '\n';
        return EXIT_FAILURE;
    }

    const auto find = [&](bool turbo) {
        return std::find_if(catalog.entries.begin(), catalog.entries.end(),
            [turbo](const auto& entry) {
                const auto isTurbo = entry.config.forcedInduction.enabled
                    && entry.config.forcedInduction.type
                        == enginelab::ForcedInductionType::turbocharger
                    && entry.config.fuel == enginelab::FuelType::gasoline;
                return turbo ? isTurbo
                    : (!entry.config.forcedInduction.enabled
                        && entry.config.cylinders.size() >= 4
                        && entry.config.fuel == enginelab::FuelType::gasoline);
            });
    };
    const auto turbo = find(true);
    const auto control = find(false);
    if (turbo == catalog.entries.end() || control == catalog.entries.end()) {
        std::cerr << "FAIL: need one gasoline turbo and one NA control\n";
        return EXIT_FAILURE;
    }

    constexpr std::array outletsMm { 45.0, 60.0, 76.0, 95.0 };
    const auto sweep = [&](const enginelab::EngineConfig& config,
                           bool openWastegateLoop) {
        const auto holdRpm = std::min(0.60 * std::min(config.redlineRpm,
            config.ignition.revLimitRpm), 4'500.0);
        std::vector<Point> points;
        points.reserve(outletsMm.size());
        for (const auto outlet : outletsMm)
            points.push_back(
                measure(config, outlet, holdRpm, openWastegateLoop));
        return points;
    };
    const auto regulated = sweep(turbo->config, false);
    const auto unregulated = sweep(turbo->config, true);
    const auto controlPoints = sweep(control->config, false);
    report(turbo->config.name + "  (turbo, stock wastegate)", regulated);
    report(turbo->config.name + "  (turbo, boost target out of reach)",
           unregulated);
    report(control->config.name + "  (naturally aspirated control)",
           controlPoints);

    const auto relativeSpan = [](const std::vector<Point>& points) {
        auto minimum = points.front().torqueNm;
        auto maximum = points.front().torqueNm;
        for (const auto& point : points) {
            minimum = std::min(minimum, point.torqueNm);
            maximum = std::max(maximum, point.torqueNm);
        }
        return maximum > 0.0 ? (maximum - minimum) / maximum : 0.0;
    };

    // 1. The mechanism itself: the turbine outlet must fall as the downstream
    //    system opens up. Before this change it was identically ambient, so
    //    this quantity could not move at all.
    auto outletFalls = true;
    for (std::size_t index = 1; index < regulated.size(); ++index)
        outletFalls = outletFalls
            && regulated[index].turbineOutletKpa
                <= regulated[index - 1].turbineOutletKpa + 0.05;

    // 2. Back pressure must follow it down. This is the quantity the user sees
    //    in the diagnostics panel and the one the complaint was about.
    const auto backPressureFalls = regulated.back().backPressureKpa
        < regulated.front().backPressureKpa - 1.0;

    // 3. The mechanism must have real magnitude, and it must be SPECIFIC to the
    //    turbine: a naturally aspirated engine has no turbine outlet, so its
    //    reading must stay exactly ambient across the same sweep. That is a
    //    control the sweep cannot accidentally satisfy.
    const auto outletSpanKpa = regulated.front().turbineOutletKpa
        - regulated.back().turbineOutletKpa;
    const auto outletSpanIsReal = outletSpanKpa > 10.0;
    auto controlOutletIsAmbient = true;
    for (const auto& point : controlPoints)
        controlOutletIsAmbient = controlOutletIsAmbient
            && std::abs(point.turbineOutletKpa
                - controlPoints.front().turbineOutletKpa) < 1.0e-6;

    // 4. Torque must improve end to end on the stock engine. This is what a
    //    user actually feels, and before the change the only sensitivity to
    //    exhaust geometry available to a turbo had the wrong sign.
    const auto torqueRises = regulated.back().torqueNm
        > regulated.front().torqueNm * 1.005;

    // 5. The stock wastegate must HOLD its boost target across the sweep. A
    //    regulator answers extra turbine power by opening further, not by
    //    overboosting; a model that raised boost here would have no regulator.
    const auto regulatedBoostHolds = std::abs(regulated.back().boostRatio
        - regulated.front().boostRatio) < 0.02;

    // Deliberately NOT gated: boost with the wastegate parked shut.
    //
    // Measured, it does not move -- and the reason is a SECOND ceiling, further
    // downstream, which this change does not address and must not be papered
    // over. The compressor target is `1 + (PR-1) * speedRatio^2` with the shaft
    // clamped at 1.16x design and the ratio at 1.12, so on the 2JZ it saturates
    // at 1 + 0.92 * 1.12^2 = 2.154 exactly, which is the number measured at
    // every downstream area. Past that clamp the compressor cannot convert
    // additional shaft power into pressure at all. Asserting a boost rise here
    // would fail for a reason that has nothing to do with the turbine, and
    // relaxing the clamp to make it pass would be tuning a model to satisfy a
    // test. It is recorded in the docs as the next open item instead.
    const auto turboSpan = relativeSpan(regulated);
    const auto controlSpan = relativeSpan(controlPoints);

    std::cout << "\nturbine outlet falls with downstream area: "
              << (outletFalls ? "PASS" : "FAIL")
              << "\nturbine outlet span " << std::fixed << std::setprecision(1)
              << outletSpanKpa << " kPa: "
              << (outletSpanIsReal ? "PASS" : "FAIL")
              << "\nNA control has no turbine outlet response: "
              << (controlOutletIsAmbient ? "PASS" : "FAIL")
              << "\nback pressure falls with downstream area: "
              << (backPressureFalls ? "PASS" : "FAIL")
              << "\nstock torque rises with downstream area: "
              << (torqueRises ? "PASS" : "FAIL")
              << "\nstock wastegate holds its boost target: "
              << (regulatedBoostHolds ? "PASS" : "FAIL")
              << "\n(ungated) torque span turbo " << std::setprecision(2)
              << turboSpan * 100.0 << " % vs NA control "
              << controlSpan * 100.0 << " %\n";

    const auto passed = outletFalls && outletSpanIsReal
        && controlOutletIsAmbient && backPressureFalls && torqueRises
        && regulatedBoostHolds;
    if (enforce && !passed) return EXIT_FAILURE;
    return EXIT_SUCCESS;
}
