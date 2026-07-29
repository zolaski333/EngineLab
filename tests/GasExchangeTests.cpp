// Gas-exchange instrument: the pumping loop across the rev range, measured at
// wide-open throttle under a dyno absorber.
//
// Pumping mean effective pressure is the one number that says whether an engine
// can breathe, and it is the one number a cycle-averaged IMEP cannot show. An
// engine that fails to evacuate its cylinder still reports a healthy IMEP: it
// pays for the failure twice, once in the work the piston spends shoving gas
// out against its own exhaust, and again in the residual it re-inducts on the
// next intake stroke. Both charges land in BMEP, neither is visible in IMEP,
// and the symptom that reaches the dyno curve -- peak torque arriving far too
// early and power never climbing to the rated speed -- looks like a dozen other
// things.
//
// This instrument exists because that is exactly what happened here. The
// catalogue's torque peaks sat 1400-4000 rpm below the manufacturers' across
// six engines including three with public curves, and five separate hypotheses
// (exhaust primary length, cam phasing, solver resolution, valve diameter,
// valve lift) were each measured and refuted before the pumping loop was
// instrumented and read -1.5 to -2.1 bar at rated speed where literature says
// -0.4 to -0.6. Four more hypotheses were refuted here (misfire, residual
// dilution, injector saturation, mixture at spark) before the apparent
// "combustion cliff" at the top of the range turned out to be this instrument's
// own absorber running the engine into its rev limiter. See
// docs/physics-audit.md.
//
// Reference numbers come from engine literature, never from this simulator's
// output, so the gate cannot become a re-calibration onto current behaviour:
//
//   1. PMEP is NEGATIVE at wide-open throttle. A naturally aspirated engine
//      cannot have the piston gain work over the gas-exchange strokes; only
//      boost can do that, and this fixture has none. Always on -- it is a
//      statement about the sign of physics, not a tuning target.
//   2. |PMEP| grows with speed. Gas-exchange losses are dynamic-pressure
//      losses, so they scale with the square of flow velocity. Always on.
//   3. |PMEP| <= 0.35 bar in the low/mid range (at ~40 % of the rev limit).
//      Heywood ch. 13: at WOT the gas-exchange loop is small and dominated by
//      valve and manifold flow losses; production naturally aspirated engines
//      measure 0.2-0.4 bar there.
//   4. |PMEP| <= 0.75 bar approaching the rev limit (at ~85 %). Rated-speed
//      WOT pumping for a well-developed engine runs 0.4-0.6 bar; 0.75 is a
//      deliberately generous ceiling that still catches a multiple.
//
// Criteria 3 and 4 are REPORTED by default and enforced under
// --enforce-gas-exchange, the same staging IntakeTuningTests used: they were
// frozen from literature BEFORE the physics could meet them, so that the day it
// does, promotion is a one-word change to the ctest registration and not a
// negotiation over thresholds. The simulator does NOT meet them today: measured
// clean, it is about 1.5x criterion 3 and 2.0x criterion 4. (An earlier note here
// said "three times"; that figure came from points contaminated by the rev
// limiter and is withdrawn.) Do not relax a criterion to make the promotion
// pass -- they encode real-engine behaviour, not a target this simulator has
// ever hit.
//
// Instrument notes carried over from the other WOT instruments in this repo:
//  * WOT under a dyno absorber is the only way to measure this. A part-throttle
//    "hold" is a different experiment entirely -- the ECU answers the absorber's
//    load by reopening the plate -- and throttling loss would then dominate the
//    pumping loop and hide the gas-exchange term this is about.
//  * The net/gross consistency check below is not decoration. The two figures
//    reach EngineState by different routes (an engine-level torque integration
//    and a per-cylinder p-dV integration), so their agreement is a live check
//    that the loop split has not drifted away from the work driving the crank.

#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/foundation/EngineTypes.hpp>
#include <enginelab/physics/SimplifiedGasolinePhysics.hpp>
#include <enginelab/simulation/DynoAbsorberController.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace {
void require(bool condition, const char* message) {
    if (condition) return;
    std::cerr << "FAILED: " << message << '\n';
    std::exit(EXIT_FAILURE);
}

// Literature-anchored windows (documented above).
constexpr double midRangePumpingCeilingBar = 0.35;
constexpr double highRpmPumpingCeilingBar = 0.75;
constexpr double midRangeFractionOfRevLimit = 0.40;
constexpr double highRpmFractionOfRevLimit = 0.85;
constexpr double sanityPumpingCeilingBar = 3.0;

constexpr double sweepStepRpm = 500.0;

struct Point final {
    double targetRpm { 0.0 };
    double actualRpm { 0.0 };
    double pmepBar { 0.0 };
    /** The exhaust-stroke half of pmep; the intake half is pmep minus this. */
    double exhaustStrokeMepBar { 0.0 };
    double netImepBar { 0.0 };
    double grossImepBar { 0.0 };
    double veFraction { 0.0 };
    double deliveredVeFraction { 0.0 };
    double misfireRate { 0.0 };
    double lambda { 1.0 };
    double combustionEfficiency { 0.0 };
    double residualFraction { 0.0 };
    double injectorCapacity { 0.0 };
    double fuelDelivery { 0.0 };
    double cylinderAfr { 0.0 };
    double rpmMinimum { 0.0 };
    double rpmMaximum { 0.0 };
    double heldRpmMinimum { 0.0 };
    double heldRpmMaximum { 0.0 };
    double phiAtSpark { 0.0 };
    double mapKpa { 0.0 };
    double exhaustKpa { 0.0 };
    bool finite { true };
};

// Hold `targetRpm` under a wide-open-throttle absorber and average the steady
// state. Identical controller to the dyno-sweep harness and to
// IntakeTuningTests, so the three instruments describe the same operating line.
Point holdPoint(
    enginelab::EngineSimulator& simulator,
    const enginelab::EngineConfig& config, double targetRpm,
    double settleSeconds, double sampleSeconds) {
    constexpr double dt = 1.0 / 240.0;
    const auto settleSteps = static_cast<int>(settleSeconds / dt);
    const auto sampleSteps = static_cast<int>(sampleSeconds / dt);
    enginelab::DynoAbsorberController absorber(config);
    absorber.reset(
        simulator.state().rpm, simulator.state().torqueNm);
    Point result;
    result.targetRpm = targetRpm;
    auto samples = 0.0;
    auto rpmAcc = 0.0, pmepAcc = 0.0, netAcc = 0.0, grossAcc = 0.0, exhStrokeMepAcc = 0.0;
    auto veAcc = 0.0, mapAcc = 0.0, exhaustAcc = 0.0, deliveredVeAcc = 0.0;
    auto misfireAcc = 0.0, lambdaAcc = 0.0, combustionAcc = 0.0, residualAcc = 0.0;
    auto capacityAcc = 0.0, deliveryAcc = 0.0, afrAcc = 0.0;
    auto rpmLow = 1.0e30, rpmHigh = -1.0e30;
    auto heldRpmLow = 1.0e30, heldRpmHigh = -1.0e30;
    auto phiAcc = 0.0;
    for (int step = 0; step < settleSteps + sampleSteps; ++step) {
        const auto absorberOutput = absorber.advance(
            dt, targetRpm, simulator.state());
        enginelab::EngineControls controls;
        controls.ignitionEnabled = true;
        controls.starterEngaged = simulator.state().rpm < 550.0;
        controls.throttle = 1.0;
        controls.dynamometerTorqueNm =
            absorberOutput.brakeTorqueNm;
        const auto frame = simulator.step(dt, controls);
        if (step < settleSteps) continue;
        if (!std::isfinite(frame.state.rpm)
            || !std::isfinite(frame.state.pumpingMeanEffectivePressureBar)
            || !std::isfinite(frame.state.grossIndicatedMeanEffectivePressureBar)
            || !std::isfinite(frame.state.indicatedMeanEffectivePressureBar))
            result.finite = false;
        samples += 1.0;
        rpmAcc += frame.state.rpm;
        rpmLow = std::min(rpmLow, frame.state.rpm);
        rpmHigh = std::max(rpmHigh, frame.state.rpm);
        heldRpmLow = std::min(
            heldRpmLow, absorberOutput.filteredRpm);
        heldRpmHigh = std::max(
            heldRpmHigh, absorberOutput.filteredRpm);
        pmepAcc += frame.state.pumpingMeanEffectivePressureBar;
        exhStrokeMepAcc += frame.state.exhaustStrokeMeanEffectivePressureBar;
        netAcc += frame.state.indicatedMeanEffectivePressureBar;
        grossAcc += frame.state.grossIndicatedMeanEffectivePressureBar;
        veAcc += frame.state.volumetricEfficiency;
        deliveredVeAcc += frame.state.deliveredVolumetricEfficiency;
        misfireAcc += frame.state.misfireRate;
        lambdaAcc += frame.state.lambda;
        auto cylinderSum = 0.0;
        auto residualSum = 0.0;
        auto capacitySum = 0.0, deliverySum = 0.0, afrSum = 0.0, phiSum = 0.0;
        for (std::size_t c = 0; c < frame.state.cylinderStateCount; ++c) {
            cylinderSum += frame.state.cylinderStates[c].combustionEfficiency;
            residualSum += frame.state.cylinderStates[c].residualGasFractionAtSpark;
            capacitySum += frame.state.cylinderStates[c].injectorCapacityRatio;
            deliverySum += frame.state.cylinderStates[c].fuelDeliveryRatio;
            afrSum += frame.state.cylinderStates[c].airFuelRatio;
            phiSum += frame.state.cylinderStates[c].equivalenceRatioAtSpark;
        }
        const auto cylinders = static_cast<double>(
            std::max<std::size_t>(1, frame.state.cylinderStateCount));
        combustionAcc += cylinderSum / cylinders;
        residualAcc += residualSum / cylinders;
        capacityAcc += capacitySum / cylinders;
        deliveryAcc += deliverySum / cylinders;
        afrAcc += afrSum / cylinders;
        phiAcc += phiSum / cylinders;
        mapAcc += frame.state.manifoldPressureKpa;
        exhaustAcc += frame.state.exhaustPressureKpa;
    }
    const auto d = std::max(1.0, samples);
    result.actualRpm = rpmAcc / d;
    result.pmepBar = pmepAcc / d;
    result.exhaustStrokeMepBar = exhStrokeMepAcc / d;
    result.netImepBar = netAcc / d;
    result.grossImepBar = grossAcc / d;
    result.veFraction = veAcc / d;
    result.deliveredVeFraction = deliveredVeAcc / d;
    result.misfireRate = misfireAcc / d;
    result.lambda = lambdaAcc / d;
    result.combustionEfficiency = combustionAcc / d;
    result.residualFraction = residualAcc / d;
    result.injectorCapacity = capacityAcc / d;
    result.fuelDelivery = deliveryAcc / d;
    result.cylinderAfr = afrAcc / d;
    result.rpmMinimum = rpmLow;
    result.rpmMaximum = rpmHigh;
    result.heldRpmMinimum = heldRpmLow;
    result.heldRpmMaximum = heldRpmHigh;
    result.phiAtSpark = phiAcc / d;
    result.mapKpa = mapAcc / d;
    result.exhaustKpa = exhaustAcc / d;
    return result;
}

// The swept point nearest a given fraction of the rev limit.
const Point& pointNear(const std::vector<Point>& points, double goalRpm) {
    std::size_t best = 0;
    for (std::size_t index = 0; index < points.size(); ++index)
        if (std::abs(points[index].targetRpm - goalRpm)
            < std::abs(points[best].targetRpm - goalRpm))
            best = index;
    return points[best];
}
}  // namespace

int main(int argc, char** argv) {
    auto enforce = false;
    // `--oracle-coupling` advances the exhaust network on every mechanical
    // substep instead of the production 16-samples-per-firing-period interval.
    //
    // This is the one experiment that separates a physical pumping loss from a
    // numerical one. The production coupling TIME-AVERAGES the cylinder
    // boundary state over its interval and then computes one valve flux from
    // that average; because the flux is a concave function of the pressure
    // difference, averaging the state first and the flux second under-predicts
    // the transfer wherever the state moves inside an interval -- which is
    // exactly the blowdown. If the exhaust-stroke half of the pumping loop
    // shrinks when the oracle is switched on, the loss is a coupling artefact
    // and no amount of valve area or duct geometry will fix it.
    auto oracleCoupling = false;
    std::optional<double> exhaustCellLengthM;
    std::optional<double> pointRpm;
    std::optional<double> outletDiameterMm;
    std::optional<double> outletDischargeCoefficient;
    std::optional<double> mufflerRestriction;
    std::optional<double> collectorDiameterMm;
    std::optional<double> collectorVolumeLitres;
    auto independentPaths = false;
    auto idealExhaustReservoir = false;
    std::optional<double> intakeValveAreaMultiplier;
    std::optional<double> exhaustValveAreaMultiplier;
    std::optional<double> exhaustMaximumHeadAreaFraction;
    auto useWellMixedExhaustJunctions = false;
    for (int index = 1; index < argc; ++index) {
        const auto argument = std::string_view(argv[index]);
        if (argument == "--enforce-gas-exchange") enforce = true;
        else if (argument == "--oracle-coupling") oracleCoupling = true;
        else if (argument == "--exhaust-cell-mm" && index + 1 < argc) {
            exhaustCellLengthM = std::stod(argv[++index]) * 0.001;
        } else if (argument == "--point-rpm" && index + 1 < argc) {
            pointRpm = std::stod(argv[++index]);
        } else if (argument == "--outlet-mm" && index + 1 < argc) {
            outletDiameterMm = std::stod(argv[++index]);
        } else if (argument == "--outlet-cd" && index + 1 < argc) {
            outletDischargeCoefficient = std::stod(argv[++index]);
        } else if (argument == "--muffler-restriction" && index + 1 < argc) {
            mufflerRestriction = std::stod(argv[++index]);
        } else if (argument == "--collector-mm" && index + 1 < argc) {
            collectorDiameterMm = std::stod(argv[++index]);
        } else if (argument == "--collector-litres" && index + 1 < argc) {
            collectorVolumeLitres = std::stod(argv[++index]);
        } else if (argument == "--independent-paths") {
            independentPaths = true;
        } else if (argument == "--ideal-exhaust-reservoir") {
            idealExhaustReservoir = true;
        } else if (argument == "--intake-valve-area-x" && index + 1 < argc) {
            intakeValveAreaMultiplier = std::stod(argv[++index]);
        } else if (argument == "--exhaust-valve-area-x" && index + 1 < argc) {
            exhaustValveAreaMultiplier = std::stod(argv[++index]);
        } else if (argument == "--exhaust-head-area-fraction" && index + 1 < argc) {
            exhaustMaximumHeadAreaFraction = std::stod(argv[++index]);
        } else if (argument == "--well-mixed-junctions") {
            useWellMixedExhaustJunctions = true;
        } else {
            std::cerr << "usage: EngineLabGasExchangeTests"
                         " [--enforce-gas-exchange] [--oracle-coupling]"
                         " [--exhaust-cell-mm millimetres] [--point-rpm rpm]"
                         " [--outlet-mm millimetres] [--outlet-cd coefficient]"
                         " [--muffler-restriction coefficient]"
                         " [--collector-mm millimetres]"
                         " [--collector-litres litres] [--independent-paths]"
                         " [--ideal-exhaust-reservoir]"
                         " [--intake-valve-area-x multiplier]"
                         " [--exhaust-valve-area-x multiplier]"
                         " [--exhaust-head-area-fraction fraction]"
                         " [--well-mixed-junctions]\n";
            return EXIT_FAILURE;
        }
    }

    auto config = enginelab::makeDefaultInlineFour();
    if (outletDiameterMm) config.exhaust.outletDiameterMm = *outletDiameterMm;
    if (outletDischargeCoefficient)
        config.exhaust.outletDischargeCoefficient = *outletDischargeCoefficient;
    if (mufflerRestriction) config.exhaust.mufflerRestriction = *mufflerRestriction;
    if (collectorDiameterMm) config.exhaust.collectorDiameterMm = *collectorDiameterMm;
    if (collectorVolumeLitres)
        config.exhaust.collectorVolumeLitres = *collectorVolumeLitres;
    if (independentPaths) {
        config.exhaustPaths.clear();
        for (const auto& cylinder : config.cylinders) {
            enginelab::ExhaustPathConfig path;
            path.id = cylinder.id;
            path.cylinderIds = { cylinder.id };
            path.geometry = config.exhaust;
            path.inheritsGlobalGeometry = false;
            config.exhaustPaths.push_back(std::move(path));
        }
    }
    enginelab::normaliseEngineConfig(config);
    // 0.95 of the limiter, not the limiter itself. `min(redline, revLimit)` puts
    // the top swept target exactly ON the rev limiter, and the ECU's limiter is
    // latched with hysteresis: once rpm touches it, spark is cut until the speed
    // falls 180 rpm below, and `flameEvents_ = {}` in the simulator zeroes the
    // published combustion efficiency for every cut cycle. The 7000 rpm point of
    // this fixture measured combustion efficiency 0.167 and net IMEP 4.8 bar that
    // way; raising the limiter to 9000 rpm and holding the same 7000 rpm point
    // gave 0.911 and 13.0 bar. A manufacturer quotes rated power below the
    // limiter, so a WOT curve must be swept below it too.
    const auto maxRpm = 0.95 * std::min(config.redlineRpm, config.ignition.revLimitRpm);

    enginelab::SimpleEcuModel ecu;
    enginelab::SimplifiedGasolinePhysics physics;
    enginelab::FourStrokeEventGenerator events;
    auto exhaust = enginelab::ExhaustGraph::makeForEngine(config);
    enginelab::EngineSimulatorOptions simulatorOptions;
    simulatorOptions.exhaustTargetCellLengthM = exhaustCellLengthM;
    simulatorOptions.resetExhaustToAmbientEachCoupling = idealExhaustReservoir;
    simulatorOptions.intakeValveAreaMultiplier = intakeValveAreaMultiplier;
    simulatorOptions.exhaustValveAreaMultiplier = exhaustValveAreaMultiplier;
    simulatorOptions.exhaustMaximumHeadAreaFraction =
        exhaustMaximumHeadAreaFraction;
    simulatorOptions.evolveExhaustJunctionAxialMomentum =
        !useWellMixedExhaustJunctions;
    enginelab::EngineSimulator simulator(
        config, ecu, physics, events, exhaust, simulatorOptions);
    simulator.setExhaustCouplingEverySubstep(oracleCoupling);

    constexpr double dt = 1.0 / 240.0;
    for (int step = 0; step < static_cast<int>(2.0 / dt); ++step) {
        const auto time = static_cast<double>(step) * dt;
        (void)simulator.step(dt, { true, time < 1.5, 0.55, 0.0 });
    }

    std::vector<Point> points;
    if (pointRpm) {
        points.push_back(holdPoint(
            simulator, config,
            std::clamp(*pointRpm, 1'000.0, maxRpm),
            5.0, 1.0));
    } else {
        const auto startRpm = std::max(2'000.0,
            std::round(config.idleRpm * 1.5 / sweepStepRpm) * sweepStepRpm);
        auto first = true;
        for (double target = startRpm; target <= maxRpm + 1.0; target += sweepStepRpm) {
            points.push_back(holdPoint(
                simulator, config, target,
                first ? 5.0 : 2.0, 1.0));
            first = false;
        }
    }
    require(pointRpm || points.size() >= 4,
            "the sweep must cover enough points to describe a trend");

    std::cout << std::fixed << std::setprecision(3)
              << "--- Gas-exchange sweep (inline4, WOT dyno absorber) ---\n";
    for (const auto& point : points) {
        std::cout << "  " << std::setw(5) << static_cast<int>(point.targetRpm) << " rpm -> "
                  << "rpm=" << point.actualRpm << " pmep=" << point.pmepBar
                  << "bar(exh=" << point.exhaustStrokeMepBar
                  << " int=" << (point.pmepBar - point.exhaustStrokeMepBar)
                  << ") net=" << point.netImepBar << " gross=" << point.grossImepBar
                  << " ve=" << point.veFraction
                  << " deliveredVe=" << point.deliveredVeFraction
                  << " trap=" << point.veFraction
                        / std::max(1.0e-6, point.deliveredVeFraction)
                  << " misfire=" << point.misfireRate
                  << " lambda=" << point.lambda
                  << " combEff=" << point.combustionEfficiency
                  << " residual=" << point.residualFraction
                  << " injCap=" << point.injectorCapacity
                  << " fuelDel=" << point.fuelDelivery
                  << " cylAfr=" << point.cylinderAfr
                  << " phiSpark=" << point.phiAtSpark
                  << " rawRpmBand=[" << point.rpmMinimum << ","
                  << point.rpmMaximum << "]"
                  << " heldRpmBand=[" << point.heldRpmMinimum << ","
                  << point.heldRpmMaximum << "]"
                  << " map=" << point.mapKpa
                  << " exh=" << point.exhaustKpa << "kPa\n";
    }

    // --- Always on: measurement validity, and the sign of the physics.
    for (const auto& point : points) {
        require(point.finite, "gas-exchange telemetry must stay finite at every point");
        // 2 %, not the 15 % these instruments used to allow. 15 % cannot tell a
        // held point from a runaway: the engine sat 458 rpm above a 6500 rpm
        // target, inside the rev limiter, and every gate passed. A gas-exchange
        // measurement is meaningless if the speed it was taken at is not the
        // speed it claims.
        require(std::abs(point.actualRpm - point.targetRpm) < 0.02 * point.targetRpm,
                "the absorber must actually hold each swept operating point");
        // The mean alone is not enough either: a window average cannot see an
        // oscillating point. Gate the dyno's filtered shaft speed while still
        // reporting raw crank-speed ripple from individual firing events.
        require(point.heldRpmMaximum - point.heldRpmMinimum
                    <= 0.04 * point.targetRpm,
                "the absorber must settle each complete sampling window");
        require(point.mapKpa > 80.0,
                "MAP must sit near ambient at WOT -- otherwise this is not a WOT measurement");
        require(point.pmepBar < 0.0,
                "a naturally aspirated engine cannot gain work over its gas-exchange strokes");
        require(std::abs(point.pmepBar) < sanityPumpingCeilingBar,
                "pumping MEP must stay physically bounded");
        // Trapping efficiency is bounded above by one for a reason no
        // calibration can change: an engine cannot keep more fresh air than it
        // drew past the valve. A ratio over one means the two figures are no
        // longer on the same oxygen basis -- a leak in the accounting, not a
        // breathing result. Two per cent of slack covers the sub-step phase
        // difference between the IVC latch and the per-substep delivery sum.
        require(point.deliveredVeFraction > 0.0,
                "a firing engine at WOT must deliver fresh air past the intake valve");
        if (point.misfireRate < 0.05)
            require(point.veFraction <= 1.02 * point.deliveredVeFraction,
                    "trapped fresh air cannot exceed delivered fresh air");
        // The two routes to the same quantity must agree: net == gross + pmep.
        // Net IMEP latches at the engine-level cycle boundary, while the split
        // sums each cylinder's last completed loop. A strict `gross > net`
        // comparison is therefore redundant with negative PMEP and can invert
        // by a few millibar when those boundaries straddle this sample.
        require(std::abs(point.netImepBar - (point.grossImepBar + point.pmepBar))
                    < 0.02 * std::max(1.0, std::abs(point.netImepBar)),
                "the indicated loop split must reconcile with net indicated work");
    }
    if (!pointRpm) {
        const auto& low = points.front();
        const auto& high = points.back();
        require(std::abs(high.pmepBar) > std::abs(low.pmepBar),
                "gas-exchange loss must grow with engine speed");
    }

    // --- Literature criteria: reported always, enforced under the flag.
    const auto& mid = pointNear(points, pointRpm
        ? *pointRpm : midRangeFractionOfRevLimit * maxRpm);
    const auto& top = pointNear(points, pointRpm
        ? *pointRpm : highRpmFractionOfRevLimit * maxRpm);
    const auto crit3 = std::abs(mid.pmepBar) <= midRangePumpingCeilingBar;
    const auto crit4 = std::abs(top.pmepBar) <= highRpmPumpingCeilingBar;
    const auto within = [](bool ok) { return ok ? "OK" : "outside target"; };
    if (pointRpm) {
        std::cout << "  diagnostic |pmep| at " << top.targetRpm << " rpm = "
                  << std::abs(top.pmepBar) << " bar\n";
    } else {
        std::cout << "  crit3 |pmep| at " << mid.targetRpm << " rpm = " << std::abs(mid.pmepBar)
                  << " <= " << midRangePumpingCeilingBar << " bar (" << within(crit3) << ")\n"
                  << "  crit4 |pmep| at " << top.targetRpm << " rpm = " << std::abs(top.pmepBar)
                  << " <= " << highRpmPumpingCeilingBar << " bar (" << within(crit4) << ")\n";
    }
    if (enforce) {
        require(!pointRpm && crit3,
                "mid-range WOT pumping loss must sit in the literature band");
        require(!pointRpm && crit4,
                "approaching the rev limit, WOT pumping loss must stay in the literature band");
    } else {
        std::cout << "  (pumping criteria reported only; --enforce-gas-exchange gates them --"
                     " promote when the gas-exchange physics meets them)\n";
    }

    std::cout << "Gas exchange tests passed\n";
    return EXIT_SUCCESS;
}
