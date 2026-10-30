#include <enginelab/runtime/KnockLimitCalibration.hpp>

#include <enginelab/calibration/EcuCalibrationKeys.hpp>
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/physics/SimplifiedGasolinePhysics.hpp>
#include <enginelab/simulation/DynoAbsorberController.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <exception>
#include <limits>
#include <memory>

namespace enginelab {

namespace {

constexpr double frameSeconds = 1.0 / 240.0;
constexpr int pointCount = 6;
// The first point climbs from idle to near the limiter and spools any turbo;
// the others start from the point above. 3 s gives the control's 1 deg/s
// recovery room for the few degrees a lower speed may give back.
constexpr double firstSettleSeconds = 5.0;
constexpr double settleSeconds = 3.0;
constexpr double sampleSeconds = 2.0;
constexpr int maximumExtraPasses = 2;
constexpr double saturatedRetardShare = 0.75;

struct Rig final {
    explicit Rig(const EngineConfig& engine)
        : config(engine), exhaust(ExhaustGraph::makeForEngine(config)),
          simulator(config, ecu, physics, events, exhaust, EngineSimulatorOptions {}) {}
    EngineConfig config;
    SimpleEcuModel ecu;
    SimplifiedGasolinePhysics physics;
    FourStrokeEventGenerator events;
    ExhaustGraph exhaust;
    EngineSimulator simulator;
};

} // namespace

KnockLimitCalibration calibrateKnockLimit(const EngineConfig& engine, const std::atomic<bool>* cancel) {
    auto config = engine;
    config.ignition.knockLimitCurve.clear();
    return measureFullLoadSpark(config, cancel);
}

KnockLimitCalibration measureFullLoadSpark(const EngineConfig& engine, const std::atomic<bool>* cancel) {
    KnockLimitCalibration result;
    if (engine.fuel == FuelType::diesel) return result;
    const auto cancelled = [cancel] { return cancel != nullptr && cancel->load(); };
    try {
        auto config = engine;
        normaliseEngineConfig(config);
        // About 7.4 MB: on the heap, never on the stack.
        auto rig = std::make_unique<Rig>(config);
        auto& simulator = rig->simulator;
        // Cold start and idle briefly, as the dyno sweep does.
        for (int step = 0; step < static_cast<int>(2.0 / frameSeconds); ++step) {
            const auto time = static_cast<double>(step) * frameSeconds;
            (void)simulator.step(frameSeconds, { true, time < 1.5, 0.55, 0.0 });
        }
        // The sweep's ceiling, under the latched limiter's hysteresis.
        const auto topRpm = std::min(0.95 * std::min(config.redlineRpm, config.ignition.revLimitRpm),
            config.ignition.revLimitRpm - 300.0);
        const auto bottomRpm = std::min(topRpm, std::max(1'500.0, 1.5 * config.idleRpm));
        DynoAbsorberController absorber(config);
        absorber.reset(simulator.state().rpm, simulator.state().torqueNm);
        for (int point = 0; point < pointCount; ++point) {
            const auto targetRpm = topRpm - (topRpm - bottomRpm) * point / (pointCount - 1);
          // A control near the end of its authority has not found the limit:
          // the map is moved by what it pulled and the point measured again.
          auto trimDegrees = 0.0;
          for (int pass = 0;; ++pass) {
            rig->ecu.setIgnitionTrimDegrees(trimDegrees);
            const auto settle = point == 0 && pass == 0 ? firstSettleSeconds : settleSeconds;
            const auto settleSteps = static_cast<int>(settle / frameSeconds);
            const auto sampleSteps = static_cast<int>(sampleSeconds / frameSeconds);
            auto rpmSum = 0.0, advanceSum = 0.0, pressureSum = 0.0, retardSum = 0.0;
            // Cylinder 1's heat release and piston travel by crank degree,
            // sampled within each frame (a frame is 60 deg at 2,400 rpm),
            // over the crank's domain: 720 deg, a rotary face's 1,080.
            const auto domain = static_cast<int>(crankDomainDegreesOf(config.cycle));
            const auto stroke = static_cast<int>(strokeDegreesOf(config.cycle));
            std::array<double, 1'080> heatRelease {};
            std::array<double, 1'080> pistonTravel {};
            std::array<int, 1'080> pistonSamples {};
            for (int step = 0; step < settleSteps + sampleSteps; ++step) {
                if (step % 240 == 0 && cancelled()) {
                    result = {};
                    result.error = "cancelled";
                    return result;
                }
                const auto absorberOutput = absorber.advance(frameSeconds, targetRpm, simulator.state());
                EngineControls controls;
                controls.ignitionEnabled = true;
                controls.starterEngaged = simulator.state().rpm < dynoStarterCutoutRpm(config);
                controls.throttle = 1.0;
                controls.dynamometerTorqueNm = absorberOutput.brakeTorqueNm;
                if (step < settleSteps) {
                    (void)simulator.step(frameSeconds, controls);
                    continue;
                }
                const auto slices = std::clamp(static_cast<int>(
                    std::ceil(std::max(100.0, simulator.state().rpm) * 6.0 * frameSeconds / 0.5)), 1, 200);
                SimulationFrame frame;
                for (int slice = 0; slice < slices; ++slice) {
                    frame = simulator.step(frameSeconds / slices, controls);
                    if (frame.state.cylinderStateCount == 0) continue;
                    const auto& cylinder = frame.state.cylinderStates[0];
                    if (!std::isfinite(cylinder.cyclePhaseDegrees)) continue;
                    const auto bin = static_cast<std::size_t>(std::clamp(
                        std::fmod(std::fmod(cylinder.cyclePhaseDegrees, domain) + domain, domain), 0.0, domain - 1.0));
                    heatRelease[bin] += std::isfinite(cylinder.combustionPulse)
                        ? std::max(0.0, cylinder.combustionPulse) : 0.0;
                    pistonTravel[bin] += cylinder.pistonTravelMm;
                    ++pistonSamples[bin];
                }
                rpmSum += frame.state.rpm;
                advanceSum += frame.state.ignitionAdvanceDegrees;
                pressureSum += frame.state.manifoldPressureKpa;
                retardSum += rig->ecu.knockRetardShare() * calibration::ecuLimits::defaultKnockRetardGainDegrees;
            }
            // Firing TDC: the least piston travel in the stroke before the
            // heat release's centre, as CombustionPhasingTests finds it.
            auto total = 0.0;
            for (const auto value : heatRelease) total += value;
            auto ca50 = std::numeric_limits<double>::quiet_NaN();
            if (total > 0.0) {
                std::size_t peak = 0;
                for (std::size_t bin = 0; bin < static_cast<std::size_t>(domain); ++bin)
                    if (heatRelease[bin] > heatRelease[peak]) peak = bin;
                auto tdc = peak;
                auto leastTravel = std::numeric_limits<double>::infinity();
                for (int offset = -stroke; offset <= 0; ++offset) {
                    const auto bin = static_cast<std::size_t>((static_cast<int>(peak) + offset + domain) % domain);
                    if (pistonSamples[bin] == 0) continue;
                    const auto travel = pistonTravel[bin] / pistonSamples[bin];
                    if (travel < leastTravel) { leastTravel = travel; tdc = bin; }
                }
                auto cumulative = 0.0;
                for (int offset = -stroke; offset < domain - stroke; ++offset) {
                    cumulative += heatRelease[(tdc + static_cast<std::size_t>(offset + domain))
                        % static_cast<std::size_t>(domain)];
                    if (cumulative >= 0.5 * total) { ca50 = offset; break; }
                }
            }
            const auto n = static_cast<double>(std::max(1, sampleSteps));
            if (!std::isfinite(advanceSum) || rpmSum / n < 0.5 * targetRpm) {
                result = {};
                result.error = "the engine did not hold " + std::to_string(static_cast<int>(targetRpm))
                    + " rpm at full load";
                return result;
            }
            const auto advance = advanceSum / n;
            if (pass < maximumExtraPasses
                    && retardSum / n > saturatedRetardShare * calibration::ecuLimits::defaultKnockRetardGainDegrees
                    && advance > calibration::ecuLimits::minimumIgnitionAdvanceDegrees + 1.0) {
                trimDegrees -= retardSum / n;
                continue;
            }
            // Ascending, as the ECU interpolates it; the retard is the
            // control's whole gap behind the map, the trim included.
            result.curve.insert(result.curve.begin(), KnockLimitSample { targetRpm, advance, pressureSum / n });
            result.retardDegrees.insert(result.retardDegrees.begin(), retardSum / n - trimDegrees);
            result.ca50Degrees.insert(result.ca50Degrees.begin(), ca50);
            break;
          }
        }
        rig->ecu.setIgnitionTrimDegrees(0.0);
    } catch (const std::exception& error) {
        result = {};
        result.error = error.what();
    }
    return result;
}

} // namespace enginelab
