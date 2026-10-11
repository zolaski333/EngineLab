// Observe every accepted mechanical step. The policy oracle deliberately does
// not include or call the actuator/window helper under test.
#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/physics/SimplifiedGasolinePhysics.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>
#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <numbers>
#include <optional>
#include <string_view>
#include <type_traits>
#include <utility>

int runValveProfileActuatorContracts();

namespace {
using namespace enginelab;

void require(bool condition, std::string_view message) {
    if (condition) return;
    std::cerr << "FAILED: " << message << '\n';
    std::exit(EXIT_FAILURE);
}

void hashWord(std::uint64_t& hash, std::uint64_t value) {
    for (unsigned int shift = 0; shift < 64; shift += 8) {
        hash ^= (value >> shift) & 0xffU;
        hash *= 1099511628211ULL;
    }
}
#include "ValveProfileStateHash.inc"

template<class T> bool identical(T first, T second) {
    if constexpr (sizeof(T) == 8)
        return std::bit_cast<std::uint64_t>(first) == std::bit_cast<std::uint64_t>(second);
    else
        return std::bit_cast<std::uint32_t>(first) == std::bit_cast<std::uint32_t>(second);
}

EngineConfig catalogue(std::string_view key) {
    const auto loaded = loadEngineCatalog(ENGINELAB_CATALOG_ROOT);
    const auto selected = selectSingleEngineCatalogEntry(loaded.entries, key);
    require(loaded.errors.empty() && static_cast<bool>(selected), "exact catalogue key must load");
    return selected.entry->config;
}

const CamshaftConfig& selectedCam(const EngineConfig& config, const CylinderConfig& cylinder) {
    // The externally visible authored bank precedence, not the Simulator helper.
    for (const auto& bank : config.banks) {
        if (bank.id == cylinder.bankId || std::find(bank.cylinderIds.begin(),
            bank.cylinderIds.end(), cylinder.id) != bank.cylinderIds.end()) return bank.camshafts;
    }
    return config.camshafts;
}

double phaseAt(double shaftAngle, const CylinderConfig& cylinder) {
    return std::fmod(shaftAngle - cylinder.crankOffsetDegrees + 1'440.0, 720.0);
}

struct CycleRange final {
    std::uint64_t number {};
    double minimum { std::numeric_limits<double>::infinity() };
    double maximum {};
};

// Only the engine clock is observed. At a wrap, solve the displacement integral
// by monotone bisection in long double, independently of the helper's closed
// form. Tests use one short forward step, so at most one crossing is possible.
class PhysicalCycleOracle final {
public:
    void advance(const EngineState& begin, const EngineState& end, double seconds) {
        if (!(end.rpm > 0.0)) { reset(); return; }
        if (!active_) { active_ = true; aligned_ = begin.crankAngleDegrees == 0.0; }
        const auto v0 = static_cast<long double>(begin.rpm) * 6.0L;
        const auto v1 = static_cast<long double>(end.rpm) * 6.0L;
        const auto dt = static_cast<long double>(seconds);
        const auto travel = (v0 + v1) * 0.5L * dt;
        require(travel >= 0.0L && travel < 180.0L, "oracle observes one short forward shaft arc");
        if (end.crankAngleDegrees >= begin.crankAngleDegrees) {
            add(begin.rpm, end.rpm);
            return;
        }
        const auto distance = 720.0L - static_cast<long double>(begin.crankAngleDegrees);
        auto low = 0.0L;
        auto high = dt;
        for (unsigned int iteration = 0; iteration < 70; ++iteration) {
            const auto time = (low + high) * 0.5L;
            const auto moved = v0 * time + (v1 - v0) * time * time / (2.0L * dt);
            if (moved < distance) low = time;
            else high = time;
        }
        const auto crossingRpm = static_cast<double>((v0 + (v1 - v0)
            * ((low + high) * 0.5L / dt)) / 6.0L);
        add(begin.rpm, crossingRpm);
        if (aligned_) { current_.number = ++number_; complete_ = current_; }
        aligned_ = true;
        current_ = {};
        add(crossingRpm, end.rpm);
    }
    void reset() { current_ = {}; complete_.reset(); number_ = 0; active_ = aligned_ = false; }
    const std::optional<CycleRange>& complete() const { return complete_; }
private:
    void add(double first, double second) {
        current_.minimum = std::min({current_.minimum, first, second});
        current_.maximum = std::max({current_.maximum, first, second});
    }
    CycleRange current_;
    std::optional<CycleRange> complete_;
    std::uint64_t number_ {};
    bool active_ {}, aligned_ {};
};

struct RequestOracle final {
    bool high {};
    std::uint64_t consumed {};
    void update(const CamshaftConfig& cam, double throttle, const std::optional<CycleRange>& range) {
        if (!cam.variableProfileEnabled || !range) { high = false; consumed = 0; return; }
        if (throttle < cam.switchThrottle) { high = false; consumed = range->number; return; }
        if (range->number == consumed) return;
        consumed = range->number;
        if (range->minimum >= cam.switchRpm) high = true;
        else if (range->maximum < cam.switchRpm) high = false;
    }
};

ValveTrainState observedVvt(const CylinderState& cylinder) {
    return {cylinder.intakeValveAdvanceDegrees, cylinder.exhaustValveAdvanceDegrees,
            cylinder.valveLiftMultiplier};
}

bool clearSupportArc(const CamshaftConfig& cam, const ValveTrainState& vvt,
                     double start, double travel) {
    if (!(travel >= 0.0) || !(travel < 180.0)) return false;
    const auto finish = start + travel;
    const auto outside = [&](double center, double duration) {
        const auto half = std::max(1.0, duration * 0.5);
        const auto firstCopy = static_cast<int>(std::floor((start - center) / 720.0)) - 1;
        for (int copy = firstCopy; copy < firstCopy + 4; ++copy) {
            const auto middle = center + static_cast<double>(copy) * 720.0;
            if (start <= middle + half && finish >= middle - half) return false;
        }
        return true;
    };
    return outside(360.0 + cam.intakeCenterlineDegrees - vvt.intakeAdvanceDegrees,
                   cam.intakeDurationDegrees)
        && outside(360.0 + cam.intakeCenterlineDegrees - vvt.intakeAdvanceDegrees,
                   cam.highIntakeDurationDegrees)
        && outside(360.0 - cam.exhaustCenterlineDegrees - vvt.exhaustAdvanceDegrees,
                   cam.exhaustDurationDegrees)
        && outside(360.0 - cam.exhaustCenterlineDegrees - vvt.exhaustAdvanceDegrees,
                   cam.highExhaustDurationDegrees);
}

bool nominalEventDistanceIsClear(const CamshaftConfig& cam, const ValveTrainState& current,
                                double previousPhase, double currentPhase) {
    // An event-distance oracle, separate from swept support intersection and
    // from every actuator helper. Match the solver's inclusive crossing rule.
    const auto travel = std::fmod(currentPhase - previousPhase + 720.0, 720.0);
    const auto crossed = [&](double target) {
        const auto distance = std::fmod(target - previousPhase + 720.0, 720.0);
        return travel > 1.0e-9 && distance <= travel;
    };
    const auto exhaustCenter = std::fmod(360.0 - cam.exhaustCenterlineDegrees
        - current.exhaustAdvanceDegrees + 720.0, 720.0);
    for (const auto high : {false, true}) {
        const auto intakeDuration = high ? cam.highIntakeDurationDegrees : cam.intakeDurationDegrees;
        const auto exhaustDuration = high ? cam.highExhaustDurationDegrees : cam.exhaustDurationDegrees;
        const auto ivc = std::fmod(360.0 + cam.intakeCenterlineDegrees
            - current.intakeAdvanceDegrees + intakeDuration * 0.5 + 720.0, 720.0);
        const auto evo = std::fmod(exhaustCenter - exhaustDuration * 0.5 + 720.0, 720.0);
        if (crossed(ivc) || crossed(evo)) return false;
    }
    return true;
}

bool fourLiftsClosed(const CamshaftConfig& cam, const ValveTrainState& actual, double phase) {
    for (const auto high : {false, true}) {
        auto copy = actual;
        const auto candidate = ValveTrainModel::evaluate(cam, high, copy, phase, 0.0, 0.0, 0.0);
        if (candidate.intakeLiftMm != 0.0 || candidate.exhaustLiftMm != 0.0) return false;
    }
    return true;
}

class MotorRig final {
public:
    explicit MotorRig(EngineConfig config)
        : config_(std::move(config)), exhaust_(ExhaustGraph::makeForEngine(config_)),
          simulator_(std::make_unique<EngineSimulator>(config_, ecu_, physics_, events_, exhaust_)) {
        config_ = simulator_->config();
        dt_ = 0.5 / config_.solver.maximumMechanicalFrequencyHz;
        simulator_->setPressureSamplingEnabled(true);
    }
    SimulationFrame tick(double targetRpm, double pedal, bool forceStop = false) {
        const auto begin = simulator_->state();
        EngineControls controls;
        controls.ignitionEnabled = false;
        controls.throttle = pedal;
        controls.externalRotatingInertiaKgM2 = motorInertia;
        const auto disturbance = begin.netTorqueNm - previousMotorTorque_;
        controls.externalTorqueNm = std::clamp((effectiveRotatingInertiaKgM2(config_) + motorInertia)
            * (targetRpm * 2.0 * std::numbers::pi / 60.0
               - begin.angularVelocityRadPerSecond) / dt_ - disturbance, -5'000.0, 5'000.0);
        if (forceStop) controls.externalTorqueNm = -5'000.0;
        const auto frame = simulator_->step(dt_, controls);
        previousMotorTorque_ = controls.externalTorqueNm;
        const auto& end = frame.state;
        require(end.solverSubsteps == 1 && !end.solverResolutionLimited,
                "a contract call observes exactly one accepted mechanical step");
        inspectEvaluation(begin, end);
        hashField(hash_, end);
        hashWord(hash_, simulator_->cylinderGeometryRevision());
        for (const auto count : {frame.firingEventCount, frame.droppedFiringEventCount,
            frame.completedBrakeCycleSampleCount, frame.droppedCompletedBrakeCycleSampleCount,
            frame.cylinderPressureSampleCount, frame.droppedCylinderPressureSampleCount,
            frame.exhaustAcousticSampleCount, frame.droppedExhaustAcousticSampleCount}) hashWord(hash_, count);
        for (std::size_t index = 0; index < frame.firingEventCount; ++index) hashField(hash_, frame.firingEvents[index]);
        for (std::size_t index = 0; index < frame.completedBrakeCycleSampleCount; ++index)
            hashField(hash_, frame.completedBrakeCycleSamples[index]);
        CylinderPressureSample pressure;
        while (simulator_->tryPopCylinderPressureSample(pressure)) {
            hashField(hash_, pressure);
            ++pressureSamples_;
            for (std::size_t index = 0; index < pressure.cylinderCount; ++index) {
                const auto& cam = selectedCam(config_, config_.cylinders[index]);
                const auto high = simulator_->appliedHighValveProfile(index);
                const auto maximumLift = std::max(0.1, high ? cam.highExhaustLiftMm : cam.exhaustLiftMm);
                const auto expected = static_cast<float>(std::clamp(
                    end.cylinderStates[index].exhaustValveLiftMm / maximumLift, 0.0, 1.0));
                require(identical(expected, pressure.exhaustValveOpening[index]),
                        "pressure stream normalizes by the actual applied profile");
            }
        }
        ExhaustAcousticSample acoustic;
        while (simulator_->tryPopExhaustAcousticSample(acoustic)) hashField(hash_, acoustic);
        physicalCycles_.advance(begin, end, dt_);
        hasQualifiedCycleSinceReset_ = hasQualifiedCycleSinceReset_ || physicalCycles_.complete().has_value();
        return frame;
    }
    void hold(double targetRpm, double pedal, double seconds) {
        const auto count = static_cast<std::size_t>(std::ceil(seconds / dt_));
        for (std::size_t step = 0; step < count; ++step) (void)tick(targetRpm, pedal);
    }
    void stop(double seconds) {
        const auto count = static_cast<std::size_t>(std::ceil(seconds / dt_));
        for (std::size_t step = 0; step < count; ++step) {
            (void)tick(0.0, 0.0, true);
            if (simulator_->state().rpm == 0.0) break;
        }
        require(simulator_->state().rpm == 0.0 && !physicalCycles_.complete(),
                "signed external braking reaches a stopped unqualified shaft");
    }
    void reset() {
        simulator_->reset();
        physicalCycles_.reset(); requests_.fill({}); observations_.fill({});
        previousMotorTorque_ = 0.0; hash_ = 1469598103934665603ULL;
        up_ = down_ = pressureSamples_ = differingLifts_ = 0;
        hasQualifiedCycleSinceReset_ = false;
        for (std::size_t index = 0; index < config_.cylinders.size(); ++index)
            require(!simulator_->appliedHighValveProfile(index), "reset clears applied high for every cylinder");
    }
    bool allVariableHigh() const {
        auto found = false;
        for (std::size_t index = 0; index < config_.cylinders.size(); ++index) {
            if (!selectedCam(config_, config_.cylinders[index]).variableProfileEnabled) continue;
            found = true;
            if (!simulator_->appliedHighValveProfile(index)) return false;
        }
        return found;
    }
    bool allLow() const {
        for (std::size_t index = 0; index < config_.cylinders.size(); ++index)
            if (simulator_->appliedHighValveProfile(index)) return false;
        return true;
    }
    EngineSimulator& simulator() { return *simulator_; }
    const EngineConfig& config() const { return config_; }
    double dt() const { return dt_; }
    std::uint64_t digest() const { return hash_; }
    std::size_t up() const { return up_; }
    std::size_t down() const { return down_; }
    std::size_t pressureSamples() const { return pressureSamples_; }
    std::size_t differingLifts() const { return differingLifts_; }
private:
    struct Observation final {
        bool valid {};
        bool high {};
        double phase {};
        double travelToNext {};
        ValveTrainState vvt;
    };
    void inspectEvaluation(const EngineState& begin, const EngineState& end) {
        for (std::size_t index = 0; index < config_.cylinders.size(); ++index) {
            const auto& cam = selectedCam(config_, config_.cylinders[index]);
            const auto phase = phaseAt(begin.crankAngleDegrees, config_.cylinders[index]);
            const auto actual = observedVvt(end.cylinderStates[index]);
            const auto high = simulator_->appliedHighValveProfile(index);
            auto& previous = observations_[index];
            requests_[index].update(cam, end.throttle, physicalCycles_.complete());
            if (!cam.variableProfileEnabled) require(!high, "a selected nonvariable bank always uses its low lobe");
            if (high != previous.high) {
                require(cam.variableProfileEnabled && high == requests_[index].high,
                        "a discrete transition obeys the independent complete physical cycle demand");
                require(previous.valid && fourLiftsClosed(cam, previous.vvt, previous.phase)
                    && fourLiftsClosed(cam, actual, phase), "a transition has actual previous and current closed valves");
                require(clearSupportArc(cam, actual, previous.phase, previous.travelToNext),
                        "a transition crosses no low/high nominal support under current VVT");
                require(nominalEventDistanceIsClear(cam, actual, previous.phase, phase),
                        "a transition creates no inclusive IVC/EVO event at distance0 or distance=travel");
                if (high) ++up_; else ++down_;
            }
            auto lowState = actual;
            auto highState = actual;
            const auto lowLift = ValveTrainModel::evaluate(cam, false, lowState, phase, begin.rpm, end.load, 0.0);
            const auto highLift = ValveTrainModel::evaluate(cam, true, highState, phase, begin.rpm, end.load, 0.0);
            differingLifts_ += (!identical(lowLift.intakeLiftMm, highLift.intakeLiftMm)
                || !identical(lowLift.exhaustLiftMm, highLift.exhaustLiftMm)) ? 1U : 0U;
            const auto& expected = high ? highLift : lowLift;
            require(identical(expected.intakeLiftMm, end.cylinderStates[index].intakeValveLiftMm)
                && identical(expected.exhaustLiftMm, end.cylinderStates[index].exhaustValveLiftMm),
                "published lifts match the actual applied lobe at the evaluation start phase");
            // No qualified high request yet: directly check existing lift output,
            // rather than trusting only the newly introduced getter.
            if (cam.variableProfileEnabled && !hasQualifiedCycleSinceReset_)
                require(identical(lowLift.intakeLiftMm, end.cylinderStates[index].intakeValveLiftMm)
                    && identical(lowLift.exhaustLiftMm, end.cylinderStates[index].exhaustValveLiftMm),
                    "before the first complete physical cycle the output is the low lobe");
            auto onceState = previous.valid ? previous.vvt : ValveTrainState {};
            (void)ValveTrainModel::evaluate(cam, previous.high, onceState, phase, begin.rpm, end.load, dt_);
            require(identical(onceState.intakeAdvanceDegrees, actual.intakeAdvanceDegrees)
                && identical(onceState.exhaustAdvanceDegrees, actual.exhaustAdvanceDegrees)
                && identical(onceState.liftMultiplier, actual.liftMultiplier), "VVT advances once per accepted step");
            previous = {true, high, phase, (begin.rpm + end.rpm) * 0.5 * 6.0 * dt_, actual};
        }
    }
    static constexpr double motorInertia = 0.40;
    EngineConfig config_;
    SimpleEcuModel ecu_;
    SimplifiedGasolinePhysics physics_;
    FourStrokeEventGenerator events_;
    ExhaustGraph exhaust_;
    std::unique_ptr<EngineSimulator> simulator_;
    double dt_ {};
    double previousMotorTorque_ {};
    PhysicalCycleOracle physicalCycles_;
    std::array<RequestOracle, 32> requests_ {};
    std::array<Observation, 32> observations_ {};
    std::uint64_t hash_ {1469598103934665603ULL};
    std::size_t up_ {}, down_ {}, pressureSamples_ {}, differingLifts_ {};
    bool hasQualifiedCycleSinceReset_ {};
};

void engineTransitionsAndReset() {
    const auto config = catalogue("01_honda_k20a_like");
    MotorRig rig(config);
    const auto switchRpm = selectedCam(rig.config(), rig.config().cylinders.front()).switchRpm;
    require(selectedCam(rig.config(), rig.config().cylinders.front()).variableProfileEnabled,
            "the real K20-like catalogue fixture must select a variable cam");
    rig.hold(switchRpm * 0.90, 1.0, 0.15);
    require(rig.allLow(), "a physically complete low-speed range stays low");
    rig.hold(switchRpm * 1.10, 1.0, 0.12);
    require(rig.allVariableHigh() && rig.up() >= config.cylinders.size(),
            "fully high physical cycles eventually actuate every variable cylinder");
    rig.hold(switchRpm * 0.90, 1.0, 0.10);
    require(rig.allLow() && rig.down() >= config.cylinders.size(),
            "fully low physical cycles eventually return every variable cylinder");
    rig.hold(switchRpm * 1.10, 1.0, 0.10);
    require(rig.allVariableHigh(), "high mode is reached again before throttle cancellation");
    rig.hold(switchRpm * 1.10, 0.0, 0.16);
    require(rig.allLow(), "physical throttle gate cancels high and actuates safely");
    rig.hold(switchRpm * 1.10, 1.0, 0.14);
    require(rig.allVariableHigh(), "throttle recovery must wait for fresh physical cycle qualification");
    require(rig.pressureSamples() > 50 && rig.differingLifts() > 50,
            "lift and pressure assertions observe nontrivial valve-open states");
    rig.stop(0.25); // A bound; stop on the observed physical zero-speed state.
    rig.hold(switchRpm * 0.90, 1.0, 0.10);
    require(rig.allLow(), "stopped restart at low speed does not retain a stale high demand");
    rig.hold(switchRpm * 1.10, 1.0, 0.10);
    require(rig.allVariableHigh(), "restarted high mode needs new full physical cycles");
    rig.reset();
    MotorRig fresh(config);
    const auto replaySteps = static_cast<std::size_t>(std::ceil(0.035 / rig.dt()));
    for (std::size_t step = 0; step < replaySteps; ++step) {
        (void)rig.tick(switchRpm * 1.10, 1.0);
        (void)fresh.tick(switchRpm * 1.10, 1.0);
        require(rig.digest() == fresh.digest(), "reset replay is physically identical to a fresh Simulator");
    }
    std::cout << "PASS engine-transitions-reset observations=" << replaySteps
        << " digest=" << rig.digest() << '\n';
}

void bankPrecedenceAndInstanceIsolation() {
    auto hidden = makeDefaultInlineFour();
    require(!hidden.banks.empty(), "bank precedence fixture has explicit banks");
    hidden.camshafts.variableProfileEnabled = true;
    for (auto& bank : hidden.banks) bank.camshafts.variableProfileEnabled = false;
    auto plain = hidden;
    plain.camshafts.variableProfileEnabled = false;
    MotorRig hiddenRig(hidden);
    MotorRig plainRig(plain);
    const auto steps = static_cast<std::size_t>(std::ceil(0.045 / hiddenRig.dt()));
    for (std::size_t step = 0; step < steps; ++step) {
        (void)hiddenRig.tick(6'600.0, 1.0);
        (void)plainRig.tick(6'600.0, 1.0);
        require(hiddenRig.digest() == plainRig.digest(), "masked global variable cam has exact nonvariable physical identity");
    }
    require(hiddenRig.allLow() && plainRig.allLow(), "selected nonvariable cams bypass the controller");
    auto bankVariable = plain;
    for (auto& bank : bankVariable.banks) {
        bank.camshafts.variableProfileEnabled = true;
        bank.camshafts.switchRpm = 2'000.0;
    }
    MotorRig bankRig(bankVariable);
    bankRig.hold(3'000.0, 1.0, 0.18);
    require(bankRig.allVariableHigh(), "variable bank cams work despite a nonvariable global cam");
    require(hiddenRig.allLow() && plainRig.allLow(), "two other live Simulator instances retain their independent states");
    require(!bankRig.simulator().appliedHighValveProfile(bankVariable.cylinders.size()),
            "out-of-range concrete getter returns low");
    std::cout << "PASS bank-precedence-instance-isolation nonvariable_digest=" << plainRig.digest() << '\n';
}
}

int main() {
    std::cout << std::setprecision(14);
    if (runValveProfileActuatorContracts() != EXIT_SUCCESS) return EXIT_FAILURE;
    engineTransitionsAndReset();
    bankPrecedenceAndInstanceIsolation();
    return EXIT_SUCCESS;
}
