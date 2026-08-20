// Offline wide-open-throttle dyno sweep.
//
// PhysicsPerfHarness measures three load points chosen for the *perf* budget;
// this tool sweeps the whole rev range so a torque/power curve can be compared
// against a manufacturer number. It holds each rpm with the same absorber the
// perf harness uses (a PI controller on state.load at throttle = 1.0) and
// reports work/angle/time from each completed brake-cycle event exactly once.
// EngineState's cycle averages are intentionally not sampled here: they are
// latched for display and would repeat a cycle at the 240 Hz outer-frame rate.
//
// It warm-starts each point from the previous one (ascending rpm), so the
// settle time can be shorter than a cold measurePoint. Absolutes still come
// from here, not from the --trace instrument, which changes substep structure.
#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/foundation/EngineTypes.hpp>
#include <enginelab/physics/SimplifiedGasolinePhysics.hpp>
#include <enginelab/simulation/DynoAbsorberController.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numbers>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

namespace {
constexpr std::array<std::string_view, 32> dynoColumnNames {
    "engine", "cylinders", "target_rpm", "actual_rpm", "torque_nm",
    "power_kw", "power_ps", "ve", "lambda", "imep_bar", "peak_cyl_bar",
    "egt_c", "map_kpa", "air_mg", "fuel_g_s", "pmep_bar",
    "gross_imep_bar", "exh_kpa", "delivered_ve",
    "exhaust_stroke_mep_bar", "intake_stroke_mep_bar",
    "rpm_min", "rpm_max", "rpm_stddev", "rpm_drift",
    "held_rpm_min", "held_rpm_max", "held_rpm_stddev", "held_rpm_drift",
    "brake_nm_mean", "brake_nm_min", "brake_nm_max",
};

struct Sample final {
    double actualRpm {};
    double torqueNm {};
    double powerKw {};
    double ve {};
    /** Heywood's delivery definition, beside the trapping one above. */
    double deliveredVe {};
    double lambda {};
    double imepBar {};
    double pmepBar {};
    /** The exhaust-stroke half of pmep; the intake half is pmep minus this. */
    double exhaustStrokeMepBar {};
    double grossImepBar {};
    double peakCylBar {};
    double egtC {};
    double mapKpa {};
    double exhaustKpa {};
    double airMgPerCycle {};
    double fuelGramsPerSecond {};
    double minimumRpm {};
    double maximumRpm {};
    double rpmStandardDeviation {};
    /** Second-half mean minus first-half mean over the sample window. */
    double rpmDrift {};
    double minimumHeldRpm {};
    double maximumHeldRpm {};
    double heldRpmStandardDeviation {};
    /** Filtered second-half mean minus first-half mean. */
    double heldRpmDrift {};
    double meanBrakeTorqueNm {};
    double minimumBrakeTorqueNm {};
    double maximumBrakeTorqueNm {};
};

[[nodiscard]] std::array<double, dynoColumnNames.size() - 1U> numericColumns(
    const Sample& sample, std::size_t cylinderCount, double targetRpm) noexcept {
    const auto powerPs = sample.powerKw / 0.735499;
    return {
        static_cast<double>(cylinderCount), targetRpm, sample.actualRpm,
        sample.torqueNm, sample.powerKw, powerPs, sample.ve, sample.lambda,
        sample.imepBar, sample.peakCylBar, sample.egtC, sample.mapKpa,
        sample.airMgPerCycle, sample.fuelGramsPerSecond, sample.pmepBar,
        sample.grossImepBar, sample.exhaustKpa, sample.deliveredVe,
        sample.exhaustStrokeMepBar,
        sample.pmepBar - sample.exhaustStrokeMepBar,
        sample.minimumRpm, sample.maximumRpm,
        sample.rpmStandardDeviation, sample.rpmDrift,
        sample.minimumHeldRpm, sample.maximumHeldRpm,
        sample.heldRpmStandardDeviation, sample.heldRpmDrift,
        sample.meanBrakeTorqueNm, sample.minimumBrakeTorqueNm,
        sample.maximumBrakeTorqueNm,
    };
}

void writeHeader(std::ostream& output) {
    for (std::size_t index = 0; index < dynoColumnNames.size(); ++index) {
        if (index != 0U) output << ',';
        output << dynoColumnNames[index];
    }
    output << '\n';
}

void writeRow(std::ostream& output, const enginelab::EngineConfig& config,
              double targetRpm, const Sample& sample) {
    const auto values = numericColumns(sample, config.cylinders.size(), targetRpm);
    static_assert(dynoColumnNames.size() == std::tuple_size_v<decltype(values)> + 1U);
    output << std::quoted(config.name);
    for (const auto value : values) output << ',' << value;
    output << '\n' << std::flush;
}

bool containsCaseInsensitive(const std::string& text, const std::string& filter) {
    const auto lower = [](unsigned char value) { return static_cast<char>(std::tolower(value)); };
    std::string a(text.size(), '\0');
    std::string b(filter.size(), '\0');
    std::transform(text.begin(), text.end(), a.begin(), lower);
    std::transform(filter.begin(), filter.end(), b.begin(), lower);
    return a.find(b) != std::string::npos;
}

enum class ReferenceMetric {
    torqueNm,
    powerKw,
};

struct ReferencePoint final {
    std::string engineFilter;
    ReferenceMetric metric { ReferenceMetric::torqueNm };
    double targetRpm {};
    double expected {};
    double relativeTolerance {};
    std::string source;
    bool matched {};
};

std::vector<ReferencePoint> readReferencePoints(
    const std::filesystem::path& path, bool& valid) {
    std::ifstream input(path);
    valid = input.good();
    std::vector<ReferencePoint> result;
    std::string line;
    if (!std::getline(input, line)) return result;
    while (std::getline(input, line)) {
        if (line.empty() || line.front() == '#') continue;
        std::array<std::string, 6> fields;
        std::istringstream row(line);
        for (auto& field : fields)
            if (!std::getline(row, field, ',')) {
                valid = false;
                break;
            }
        if (!valid) break;
        ReferencePoint point;
        point.engineFilter = fields[0];
        if (fields[1] == "torque_nm") point.metric = ReferenceMetric::torqueNm;
        else if (fields[1] == "power_kw") point.metric = ReferenceMetric::powerKw;
        else {
            valid = false;
            break;
        }
        try {
            point.targetRpm = std::stod(fields[2]);
            point.expected = std::stod(fields[3]);
            point.relativeTolerance = std::stod(fields[4]);
        } catch (const std::exception&) {
            valid = false;
            break;
        }
        point.source = fields[5];
        if (point.engineFilter.empty() || !(point.targetRpm > 0.0)
            || !(point.expected > 0.0) || !(point.relativeTolerance > 0.0)
            || point.relativeTolerance > 0.25 || point.source.empty()) {
            valid = false;
            break;
        }
        result.push_back(std::move(point));
    }
    return result;
}

// Hold `targetRpm` under a wide-open-throttle absorber and average the steady
// state. Identical controller to PhysicsPerfHarness::measurePoint.
Sample holdPoint(enginelab::EngineSimulator& simulator,
                 const enginelab::EngineConfig& config,
                 double targetRpm, double settleSeconds,
                 double sampleSeconds) {
    constexpr double dt = 1.0 / 240.0;
    const auto settleSteps = static_cast<int>(settleSeconds / dt);
    const auto sampleSteps = static_cast<int>(sampleSeconds / dt);
    enginelab::DynoAbsorberController absorber(config);
    absorber.reset(
        simulator.state().rpm, simulator.state().torqueNm);
    auto n = 0.0;
    Sample acc {};
    auto peak = 0.0;
    auto cylN = 0.0;
    auto veAcc = 0.0, lambdaAcc = 0.0, imepAcc = 0.0, egtAcc = 0.0, mapAcc = 0.0;
    auto airAcc = 0.0, fuelAcc = 0.0, rpmAcc = 0.0;
    auto brakeWorkJoules = 0.0;
    auto brakeCrankRadians = 0.0;
    auto brakeCycleDurationSeconds = 0.0;
    std::size_t brakeCycleCount = 0;
    std::size_t droppedBrakeCycleCount = 0;
    auto pmepAcc = 0.0, grossImepAcc = 0.0, exhaustAcc = 0.0, deliveredVeAcc = 0.0;
    auto exhStrokeMepAcc = 0.0;
    auto rpmSquareAcc = 0.0;
    auto minimumRpm = std::numeric_limits<double>::infinity();
    auto maximumRpm = 0.0;
    auto brakeTorqueAcc = 0.0;
    auto minimumBrakeTorque = std::numeric_limits<double>::infinity();
    auto maximumBrakeTorque = 0.0;
    auto firstHalfRpmAcc = 0.0;
    auto secondHalfRpmAcc = 0.0;
    auto heldRpmAcc = 0.0;
    auto heldRpmSquareAcc = 0.0;
    auto minimumHeldRpm = std::numeric_limits<double>::infinity();
    auto maximumHeldRpm = 0.0;
    auto firstHalfHeldRpmAcc = 0.0;
    auto secondHalfHeldRpmAcc = 0.0;
    auto firstHalfCount = 0.0;
    auto secondHalfCount = 0.0;
    auto sampleStartTimeSeconds = std::numeric_limits<double>::infinity();
    for (int step = 0; step < settleSteps + sampleSteps; ++step) {
        const auto absorberOutput = absorber.advance(
            dt, targetRpm, simulator.state());
        enginelab::EngineControls controls;
        controls.ignitionEnabled = true;
        controls.starterEngaged = simulator.state().rpm < 550.0;
        controls.throttle = 1.0;
        controls.dynamometerTorqueNm =
            absorberOutput.brakeTorqueNm;
        if (step == settleSteps)
            sampleStartTimeSeconds = simulator.state().simulationTimeSeconds;
        const auto frame = simulator.step(dt, controls);
        if (step >= settleSteps) {
            const auto sampleIndex = step - settleSteps;
            n += 1.0;
            rpmAcc += frame.state.rpm;
            rpmSquareAcc += frame.state.rpm * frame.state.rpm;
            minimumRpm = std::min(minimumRpm, frame.state.rpm);
            maximumRpm = std::max(maximumRpm, frame.state.rpm);
            heldRpmAcc += absorberOutput.filteredRpm;
            heldRpmSquareAcc += absorberOutput.filteredRpm
                * absorberOutput.filteredRpm;
            minimumHeldRpm = std::min(
                minimumHeldRpm, absorberOutput.filteredRpm);
            maximumHeldRpm = std::max(
                maximumHeldRpm, absorberOutput.filteredRpm);
            brakeTorqueAcc += controls.dynamometerTorqueNm;
            minimumBrakeTorque = std::min(
                minimumBrakeTorque, controls.dynamometerTorqueNm);
            maximumBrakeTorque = std::max(
                maximumBrakeTorque, controls.dynamometerTorqueNm);
            if (sampleIndex < sampleSteps / 2) {
                firstHalfRpmAcc += frame.state.rpm;
                firstHalfHeldRpmAcc += absorberOutput.filteredRpm;
                firstHalfCount += 1.0;
            } else {
                secondHalfRpmAcc += frame.state.rpm;
                secondHalfHeldRpmAcc += absorberOutput.filteredRpm;
                secondHalfCount += 1.0;
            }
            droppedBrakeCycleCount +=
                frame.droppedCompletedBrakeCycleSampleCount;
            for (std::size_t cycleIndex = 0;
                 cycleIndex < frame.completedBrakeCycleSampleCount;
                 ++cycleIndex) {
                const auto& cycle =
                    frame.completedBrakeCycleSamples[cycleIndex];
                // A cycle which began in the settling interval is not a full
                // member of the acquisition window, even if it ends in its
                // first frame. The unfinished final cycle is naturally absent.
                if (!cycle.numericallyValid
                    || cycle.startTimeSeconds + 1.0e-12
                        < sampleStartTimeSeconds)
                    continue;
                brakeWorkJoules += cycle.brakeWorkJoules;
                brakeCrankRadians += cycle.integratedCrankRadians;
                brakeCycleDurationSeconds += cycle.durationSeconds;
                ++brakeCycleCount;
            }
            veAcc += frame.state.volumetricEfficiency;
            deliveredVeAcc += frame.state.deliveredVolumetricEfficiency;
            lambdaAcc += frame.state.lambda;
            imepAcc += frame.state.indicatedMeanEffectivePressureBar;
            pmepAcc += frame.state.pumpingMeanEffectivePressureBar;
            exhStrokeMepAcc += frame.state.exhaustStrokeMeanEffectivePressureBar;
            grossImepAcc += frame.state.grossIndicatedMeanEffectivePressureBar;
            egtAcc += frame.state.exhaustTemperatureC;
            mapAcc += frame.state.manifoldPressureKpa;
            exhaustAcc += frame.state.exhaustPressureKpa;
            airAcc += frame.state.airMassMgPerCycle;
            fuelAcc += frame.state.fuelFlowGramsPerSecond;
            for (std::size_t c = 0; c < frame.state.cylinderStateCount; ++c) {
                peak = std::max(peak, frame.state.cylinderStates[c].pressureEstimateBar);
                cylN += 1.0;
            }
        }
    }
    const auto d = std::max(1.0, n);
    const auto validBrakeCycles = droppedBrakeCycleCount == 0
        && brakeCycleCount > 0
        && brakeCrankRadians > 0.0
        && brakeCycleDurationSeconds > 0.0;
    if (validBrakeCycles) {
        acc.actualRpm = brakeCrankRadians / brakeCycleDurationSeconds
            * 60.0 / (2.0 * std::numbers::pi);
        acc.torqueNm = brakeWorkJoules / brakeCrankRadians;
        acc.powerKw = brakeWorkJoules
            / brakeCycleDurationSeconds / 1'000.0;
    } else {
        acc.actualRpm = rpmAcc / d;
        acc.torqueNm = std::numeric_limits<double>::quiet_NaN();
        acc.powerKw = std::numeric_limits<double>::quiet_NaN();
    }
    acc.ve = veAcc / d;
    acc.deliveredVe = deliveredVeAcc / d;
    acc.lambda = lambdaAcc / d;
    acc.imepBar = imepAcc / d;
    acc.pmepBar = pmepAcc / d;
    acc.exhaustStrokeMepBar = exhStrokeMepAcc / d;
    acc.grossImepBar = grossImepAcc / d;
    acc.peakCylBar = peak;
    acc.egtC = egtAcc / d;
    acc.mapKpa = mapAcc / d;
    acc.exhaustKpa = exhaustAcc / d;
    acc.airMgPerCycle = airAcc / d;
    acc.fuelGramsPerSecond = fuelAcc / d;
    acc.minimumRpm = std::isfinite(minimumRpm)
        ? minimumRpm : acc.actualRpm;
    acc.maximumRpm = maximumRpm;
    acc.rpmStandardDeviation = std::sqrt(std::max(
        0.0, rpmSquareAcc / d - acc.actualRpm * acc.actualRpm));
    acc.rpmDrift = secondHalfRpmAcc / std::max(1.0, secondHalfCount)
        - firstHalfRpmAcc / std::max(1.0, firstHalfCount);
    const auto meanHeldRpm = heldRpmAcc / d;
    acc.minimumHeldRpm = std::isfinite(minimumHeldRpm)
        ? minimumHeldRpm : meanHeldRpm;
    acc.maximumHeldRpm = maximumHeldRpm;
    acc.heldRpmStandardDeviation = std::sqrt(std::max(
        0.0, heldRpmSquareAcc / d - meanHeldRpm * meanHeldRpm));
    acc.heldRpmDrift =
        secondHalfHeldRpmAcc / std::max(1.0, secondHalfCount)
        - firstHalfHeldRpmAcc / std::max(1.0, firstHalfCount);
    acc.meanBrakeTorqueNm = brakeTorqueAcc / d;
    acc.minimumBrakeTorqueNm = std::isfinite(minimumBrakeTorque)
        ? minimumBrakeTorque : acc.meanBrakeTorqueNm;
    acc.maximumBrakeTorqueNm = maximumBrakeTorque;
    return acc;
}

void sweepEngine(const enginelab::EngineConfig& baseConfig, double stepRpm,
                 bool useWellMixedExhaustJunctions) {
    auto config = baseConfig;
    enginelab::normaliseEngineConfig(config);
    enginelab::SimpleEcuModel ecu;
    enginelab::SimplifiedGasolinePhysics physics;
    enginelab::FourStrokeEventGenerator events;
    auto exhaust = enginelab::ExhaustGraph::makeForEngine(config);
    enginelab::EngineSimulatorOptions simulatorOptions;
    simulatorOptions.evolveExhaustJunctionAxialMomentum =
        !useWellMixedExhaustJunctions;
    enginelab::EngineSimulator simulator(
        config, ecu, physics, events, exhaust, simulatorOptions);

    constexpr double dt = 1.0 / 240.0;
    // Cold start and idle briefly before the first hold point.
    for (int step = 0; step < static_cast<int>(2.0 / dt); ++step) {
        const auto time = static_cast<double>(step) * dt;
        (void)simulator.step(dt, { true, time < 1.5, 0.55, 0.0 });
    }

    // 0.95 of the limiter, not the limiter itself: the ECU's rev limiter is
    // latched with hysteresis, so a target placed ON it is measured with spark
    // being cut, and the last point of every curve this tool ever printed was
    // that point. Measured on the default inline four: the top point read
    // combustion efficiency 0.167 and net IMEP 4.8 bar against 0.911 and 13.0
    // bar for the same speed with the limiter moved out of the way. A
    // manufacturer quotes rated power below the limiter. See
    // tests/GasExchangeTests.cpp and docs/physics-audit.md.
    const auto maxRpm = 0.95 * std::min(config.redlineRpm, config.ignition.revLimitRpm);
    const auto startRpm = std::max(2000.0, std::round(config.idleRpm * 1.5 / stepRpm) * stepRpm);
    auto first = true;
    for (double target = startRpm; target <= maxRpm + 1.0; target += stepRpm) {
        // Longer settle for the first point (cold ramp), shorter warm-started.
        const auto sample = holdPoint(
            simulator, config, target, first ? 5.0 : 2.0, 1.0);
        first = false;
        writeRow(std::cout, config, target, sample);
    }
}

bool validateReferencePoints(const std::filesystem::path& catalogRoot,
                             const std::filesystem::path& referencePath,
                             const std::string& engineFilter,
                             bool useWellMixedExhaustJunctions) {
    bool referenceValid = false;
    auto points = readReferencePoints(referencePath, referenceValid);
    if (!referenceValid || points.empty()) {
        std::cerr << "invalid or empty manufacturer reference file: "
                  << referencePath.string() << '\n';
        return false;
    }
    const auto catalog = enginelab::loadEngineCatalog(catalogRoot);
    if (!catalog.errors.empty() || catalog.entries.empty()) {
        for (const auto& error : catalog.errors)
            std::cerr << "catalog error: " << error << '\n';
        return false;
    }

    auto passed = true;
    std::cout << "engine,metric,target_rpm,actual_rpm,raw_rpm_span_percent,"
                 "held_rpm_span_percent,held_rpm_drift_percent,"
                 "held_rpm_stddev,measured,reference,"
                 "error_percent,tolerance_percent,hold_result,source,result\n";
    for (const auto& entry : catalog.entries) {
        if (!engineFilter.empty()
            && !containsCaseInsensitive(entry.config.name, engineFilter))
            continue;
        std::vector<std::size_t> selected;
        for (std::size_t index = 0; index < points.size(); ++index) {
            if (containsCaseInsensitive(entry.config.name, points[index].engineFilter))
                selected.push_back(index);
        }
        if (selected.empty()) continue;
        std::sort(selected.begin(), selected.end(),
            [&points](std::size_t left, std::size_t right) {
                return points[left].targetRpm < points[right].targetRpm;
            });

        auto config = entry.config;
        enginelab::normaliseEngineConfig(config);
        enginelab::SimpleEcuModel ecu;
        enginelab::SimplifiedGasolinePhysics physics;
        enginelab::FourStrokeEventGenerator events;
        auto exhaust = enginelab::ExhaustGraph::makeForEngine(config);
        enginelab::EngineSimulatorOptions simulatorOptions;
        simulatorOptions.evolveExhaustJunctionAxialMomentum =
            !useWellMixedExhaustJunctions;
        enginelab::EngineSimulator simulator(
            config, ecu, physics, events, exhaust, simulatorOptions);
        constexpr double dt = 1.0 / 240.0;
        for (int step = 0; step < static_cast<int>(2.0 / dt); ++step) {
            const auto time = static_cast<double>(step) * dt;
            (void)simulator.step(dt, { true, time < 1.5, 0.55, 0.0 });
        }

        auto first = true;
        for (const auto index : selected) {
            auto& reference = points[index];
            reference.matched = true;
            const auto maximumRatedRpm =
                std::min(config.redlineRpm, config.ignition.revLimitRpm);
            if (reference.targetRpm >= maximumRatedRpm) {
                std::cerr << "reference point lies on/above the limiter: "
                          << config.name << " at " << reference.targetRpm << " rpm\n";
                passed = false;
                continue;
            }
            const auto sample = holdPoint(
                simulator, config, reference.targetRpm,
                first ? 5.0 : 2.0, 1.0);
            first = false;
            const auto measured = reference.metric == ReferenceMetric::torqueNm
                ? sample.torqueNm : sample.powerKw;
            const auto relativeError =
                (measured - reference.expected) / reference.expected;
            const auto speedHeld = std::abs(
                sample.actualRpm - reference.targetRpm)
                <= reference.targetRpm * 0.02;
            // A mean near target is not a held point. The old normalized-load
            // PI produced means within 2% while CP3 swept 29.3% of target and
            // CP4 swept 16.6%. Gate the filtered dyno shaft speed across the
            // complete window; raw crank-speed ripple remains reported.
            const auto holdStable = speedHeld
                && sample.maximumHeldRpm - sample.minimumHeldRpm
                    <= reference.targetRpm * 0.04
                && std::abs(sample.heldRpmDrift)
                    <= reference.targetRpm * 0.01
                && sample.heldRpmStandardDeviation
                    <= reference.targetRpm * 0.015;
            const auto pointPassed = holdStable && std::isfinite(measured)
                && std::abs(relativeError) <= reference.relativeTolerance;
            passed = passed && pointPassed;
            std::cout << std::quoted(config.name) << ','
                      << (reference.metric == ReferenceMetric::torqueNm
                            ? "torque_nm" : "power_kw")
                      << ',' << reference.targetRpm << ',' << sample.actualRpm
                      << ',' << (sample.maximumRpm - sample.minimumRpm)
                            / reference.targetRpm * 100.0
                      << ',' << (sample.maximumHeldRpm - sample.minimumHeldRpm)
                            / reference.targetRpm * 100.0
                      << ',' << sample.heldRpmDrift
                            / reference.targetRpm * 100.0
                      << ',' << sample.heldRpmStandardDeviation
                      << ',' << measured << ',' << reference.expected
                      << ',' << relativeError * 100.0
                      << ',' << reference.relativeTolerance * 100.0
                      << ',' << (holdStable ? "PASS" : "UNSETTLED")
                      << ',' << std::quoted(reference.source)
                      << ',' << (pointPassed ? "PASS" : "FAIL") << '\n';
        }
    }
    for (const auto& point : points) {
        if (!engineFilter.empty()
            && !containsCaseInsensitive(point.engineFilter, engineFilter)
            && !containsCaseInsensitive(engineFilter, point.engineFilter))
            continue;
        if (point.matched) continue;
        std::cerr << "no catalogue engine matched manufacturer reference: "
                  << point.engineFilter << '\n';
        passed = false;
    }
    return passed;
}
}  // namespace

int main(int argc, char** argv) {
    std::filesystem::path catalogRoot = std::filesystem::current_path();
    std::filesystem::path referenceFile;
    std::string filter;
    auto stepRpm = 500.0;
    auto schemaOnly = false;
    auto useWellMixedExhaustJunctions = false;
    for (int index = 1; index < argc; ++index) {
        const std::string argument = argv[index];
        if (argument == "--catalog-root" && index + 1 < argc) catalogRoot = argv[++index];
        else if (argument == "--filter" && index + 1 < argc) filter = argv[++index];
        else if (argument == "--step" && index + 1 < argc) stepRpm = std::stod(argv[++index]);
        else if (argument == "--reference-file" && index + 1 < argc)
            referenceFile = argv[++index];
        else if (argument == "--schema-only") schemaOnly = true;
        else if (argument == "--well-mixed-junctions")
            useWellMixedExhaustJunctions = true;
        else {
            std::cerr << "usage: EngineLabDynoSweepHarness [--catalog-root dir]"
                         " [--filter name-fragment] [--step rpm]"
                         " [--reference-file csv] [--schema-only]"
                         " [--well-mixed-junctions]\n";
            return EXIT_FAILURE;
        }
    }

    std::cout << std::fixed << std::setprecision(3);
    if (schemaOnly) {
        writeHeader(std::cout);
        return EXIT_SUCCESS;
    }
    if (!referenceFile.empty())
        return validateReferencePoints(
            catalogRoot, referenceFile, filter, useWellMixedExhaustJunctions)
            ? EXIT_SUCCESS : EXIT_FAILURE;
    writeHeader(std::cout);

    const auto catalog = enginelab::loadEngineCatalog(catalogRoot);
    for (const auto& error : catalog.errors)
        std::cerr << "catalog error: " << error << '\n';
    if (catalog.entries.empty()) {
        std::cerr << "no engines loaded\n";
        return EXIT_FAILURE;
    }
    std::vector<enginelab::EngineConfig> engines;
    if (filter.empty()) {
        engines.reserve(catalog.entries.size());
        for (const auto& entry : catalog.entries)
            engines.push_back(entry.config);
    } else {
        const auto selected = enginelab::selectSingleEngineCatalogEntry(
            catalog.entries, filter);
        if (!selected) {
            std::cerr << (selected.status
                    == enginelab::EngineCatalogSelectionStatus::ambiguous
                    ? "ambiguous engine selector; use an exact catalogue key:\n"
                    : "no engine matched selector\n");
            for (const auto* match : selected.matches)
                std::cerr << "  " << match->config.audioVoicingKey
                          << "  " << match->config.name << '\n';
            return EXIT_FAILURE;
        }
        engines.push_back(selected.entry->config);
    }

    for (const auto& engine : engines)
        sweepEngine(engine, stepRpm, useWellMixedExhaustJunctions);
    return EXIT_SUCCESS;
}
