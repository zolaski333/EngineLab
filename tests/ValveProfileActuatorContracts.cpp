#include <enginelab/simulation/ValveProfileActuator.hpp>
#include <bit>
#include <cstdlib>
#include <iostream>
#include <numbers>
#include <string_view>
#include <utility>

using namespace enginelab;
using namespace enginelab::cam_controller;
namespace {
int checks {};
int contracts {};
bool check(bool value, const char* expression, int line) {
    ++checks;
    if (!value) std::cerr << "FAIL line=" << line << " expression=" << expression << '\n';
    return value;
}
#define REQUIRE(expression) do { if (!check((expression), #expression, __LINE__)) return false; } while (false)
bool near(double first, double second) {
    return std::abs(first - second) <= 1.0e-11 * std::max({1.0, std::abs(first), std::abs(second)});
}
bool sameState(const ValveTrainState& first, const ValveTrainState& second) {
    return std::bit_cast<std::uint64_t>(first.intakeAdvanceDegrees) == std::bit_cast<std::uint64_t>(second.intakeAdvanceDegrees)
        && std::bit_cast<std::uint64_t>(first.exhaustAdvanceDegrees) == std::bit_cast<std::uint64_t>(second.exhaustAdvanceDegrees)
        && std::bit_cast<std::uint64_t>(first.liftMultiplier) == std::bit_cast<std::uint64_t>(second.liftMultiplier);
}
std::optional<ShaftCycleMeasurement> cycle(double minimum, double maximum,
                                          std::uint64_t generation = 1, std::uint64_t epoch = 0) {
    return ShaftCycleMeasurement { epoch, generation, 0.02, minimum, maximum, 6000.0 };
}
CamshaftConfig variableCam() { CamshaftConfig cam; cam.variableProfileEnabled = true; return cam; }
bool shaftConstantAndPartial() {
    ShaftCycleWindow exact;
    REQUIRE(exact.advance(0.0, 0.0, 720.0, 0.02, 6000.0, 6000.0).completedCycles == 1);
    REQUIRE(exact.completed().has_value());
    REQUIRE(near(exact.completed()->meanRpm, 6000.0));
    REQUIRE(exact.completed()->minimumRpm == 6000.0 && exact.completed()->maximumRpm == 6000.0);
    ShaftCycleWindow partial;
    REQUIRE(partial.advance(200.0, 0.0, 520.0, 520.0 / 36000.0, 6000.0, 6000.0).accepted);
    REQUIRE(!partial.completed());
    REQUIRE(partial.advance(0.0, 0.0, 720.0, 0.02, 6000.0, 6000.0).completedCycles == 1);
    REQUIRE(partial.completed()->generation == 1 && near(partial.completed()->durationSeconds, 0.02));
    return true;
}
bool shaftAcceleratingAndDecelerating() {
    for (const bool accelerating : { false, true }) {
        const auto first = accelerating ? 1000.0 : 10000.0;
        const auto last = accelerating ? 10000.0 : 1000.0;
        constexpr auto duration = 0.03;
        const auto velocity = first * 6.0;
        const auto acceleration = (last - first) * 6.0 / duration;
        // Independent analytical positive root in conventional quadratic form.
        const auto expectedTime = (-velocity + std::sqrt(velocity * velocity + 2.0 * acceleration * 720.0)) / acceleration;
        const auto expectedRpm = first + (last - first) * expectedTime / duration;
        ShaftCycleWindow window;
        REQUIRE(window.advance(0.0, 270.0, 990.0, duration, first, last).accepted);
        REQUIRE(window.completed().has_value());
        REQUIRE(near(window.completed()->durationSeconds, expectedTime));
        REQUIRE(!near(window.completed()->durationSeconds, duration * 720.0 / 990.0));
        REQUIRE(near(window.completed()->minimumRpm, std::min(first, expectedRpm)));
        REQUIRE(near(window.completed()->maximumRpm, std::max(first, expectedRpm)));
        const auto tailDuration = 450.0 / (last * 6.0);
        REQUIRE(window.advance(270.0, 0.0, 450.0, tailDuration, last, last).completedCycles == 1);
        REQUIRE(near(window.completed()->durationSeconds, duration - expectedTime + tailDuration));
        REQUIRE(near(window.completed()->minimumRpm, std::min(last, expectedRpm)));
        REQUIRE(near(window.completed()->maximumRpm, std::max(last, expectedRpm)));
    }
    return true;
}
bool shaftMultipleWrapAndRoundoff() {
    ShaftCycleWindow window;
    REQUIRE(window.advance(0.0, 0.0, 2160.0, 0.06, 6000.0, 6000.0).completedCycles == 3);
    REQUIRE(window.completed()->generation == 3 && near(window.completed()->durationSeconds, 0.02));
    ShaftCycleWindow partial;
    REQUIRE(partial.advance(100.0, 100.0, 2160.0, 0.06, 6000.0, 6000.0).completedCycles == 2);
    REQUIRE(partial.completed()->generation == 2);
    ShaftCycleWindow rounded;
    REQUIRE(rounded.advance(0.0, -0.0, std::nextafter(720.0, 0.0), 0.02, 6000.0, 6000.0).accepted);
    REQUIRE(rounded.completed()->generation == 1 && near(rounded.completed()->meanRpm, 6000.0));
    REQUIRE(rounded.advance(-0.0, 0.0, 720.0, 0.02, 6000.0, 6000.0).completedCycles == 1);
    return true;
}
bool shaftStopRestartAndInvalid() {
    ShaftCycleWindow window;
    REQUIRE(window.advance(0.0, 0.0, 720.0, 0.02, 6000.0, 6000.0).accepted);
    REQUIRE(window.advance(0.0, 0.0, 0.0, 0.005, 0.0, 0.0).accepted);
    REQUIRE(!window.completed());
    REQUIRE(window.advance(0.0, 360.0, 360.0, 0.02, 0.0, 6000.0).accepted);
    REQUIRE(!window.completed());
    REQUIRE(window.advance(360.0, 0.0, 360.0, 0.01, 6000.0, 6000.0).accepted);
    REQUIRE(window.completed()->minimumRpm == 0.0 && near(window.completed()->meanRpm, 4000.0));
    REQUIRE(!window.advance(120.0, 120.0, 0.0, 0.005, 0.0, 0.0).accepted);
    REQUIRE(!window.completed());
    REQUIRE(!window.advance(0.0, 0.0, -1.0, 0.02, 6000.0, 6000.0).accepted);
    REQUIRE(!window.advance(0.0, 0.0, 720.0, 0.02, -6000.0, -6000.0).accepted);
    REQUIRE(!window.advance(0.0, 0.0, 720.0, std::numeric_limits<double>::quiet_NaN(), 6000.0, 6000.0).accepted);
    REQUIRE(!window.advance(0.0, 1.0, 720.0, 0.02, 6000.0, 6000.0).accepted);
    REQUIRE(window.advance(0.0, 0.0, 720.0, 0.02, 12000.0, 0.0).accepted);
    REQUIRE(!window.completed());
    window.reset();
    REQUIRE(!window.lastTravelDegrees() && !window.completed());
    return true;
}
bool rangeThresholdAndThrottle() {
    auto cam = variableCam();
    ValveProfileActuator actuator;
    actuator.updateDemand(cam, 1.0, cycle(5800.0, 5900.0), 1);
    REQUIRE(actuator.requestedHigh());
    actuator.updateDemand(cam, 1.0, cycle(5700.0, 5800.0, 2), 1);
    REQUIRE(actuator.requestedHigh());
    actuator.updateDemand(cam, 1.0, cycle(5700.0, 5799.0, 3), 1);
    REQUIRE(!actuator.requestedHigh());
    actuator.updateDemand(cam, 1.0, cycle(5900.0, 6000.0, 4), 1);
    REQUIRE(actuator.requestedHigh());
    actuator.updateDemand(cam, cam.switchThrottle - 0.01, cycle(5900.0, 6000.0, 4), 1);
    REQUIRE(!actuator.requestedHigh());
    actuator.updateDemand(cam, cam.switchThrottle, cycle(5900.0, 6000.0, 4), 1);
    REQUIRE(!actuator.requestedHigh()); // Recovery waits for a new physical cycle.
    actuator.updateDemand(cam, cam.switchThrottle, cycle(5900.0, 6000.0, 5), 1);
    REQUIRE(actuator.requestedHigh());
    ValveProfileActuator initiallyMixed;
    initiallyMixed.updateDemand(cam, 1.0, cycle(5700.0, 5801.0), 1);
    REQUIRE(!initiallyMixed.requestedHigh());
    return true;
}
bool missingDemandAndPendingCancellation() {
    auto cam = variableCam();
    ValveProfileActuator actuator;
    actuator.updateDemand(cam, 1.0, cycle(5900.0, 6000.0), 1);
    REQUIRE(!actuator.finish(cam, {}, 0.0, std::nullopt, 1) && actuator.pending());
    actuator.updateDemand(cam, 1.0, std::nullopt, 1);
    REQUIRE(!actuator.requestedHigh() && !actuator.pending() && !actuator.qualifiedDemand());
    REQUIRE(!actuator.finish(cam, {}, 1.0, 1.0, 1));
    actuator.reset(true);
    actuator.updateDemand(cam, 1.0, std::nullopt, 1);
    REQUIRE(actuator.finish(cam, {}, 10.0, std::nullopt, 1));
    actuator.updateDemand(cam, 1.0, std::nullopt, 1);
    REQUIRE(!actuator.finish(cam, {}, 11.0, 1.0, 1)); // Low after two safe observations.
    actuator.reset();
    actuator.updateDemand(cam, 1.0, cycle(5900.0, 6000.0), 1);
    REQUIRE(!actuator.finish(cam, {}, 0.0, std::nullopt, 1));
    actuator.updateDemand(cam, cam.switchThrottle - 0.01, cycle(5900.0, 6000.0), 1);
    REQUIRE(!actuator.finish(cam, {}, 1.0, 1.0, 1));
    actuator.updateDemand(cam, 1.0, ShaftCycleMeasurement {}, 1);
    REQUIRE(!actuator.requestedHigh());
    return true;
}
bool openValveAndSafeWrap() {
    auto cam = variableCam();
    ValveProfileActuator actuator;
    actuator.updateDemand(cam, 1.0, cycle(5900.0, 6000.0), 1);
    REQUIRE(!actuator.finish(cam, {}, 470.0, std::nullopt, 1));
    REQUIRE(!actuator.pending());
    REQUIRE(!actuator.finish(cam, {}, 471.0, 1.0, 1));
    REQUIRE(!actuator.finish(cam, {}, 612.0, 141.0, 1));
    REQUIRE(actuator.pending());
    REQUIRE(actuator.finish(cam, {}, 613.0, 1.0, 1));
    actuator.reset();
    actuator.updateDemand(cam, 1.0, cycle(5900.0, 6000.0), 1);
    REQUIRE(!actuator.finish(cam, {}, 719.0, std::nullopt, 1));
    REQUIRE(actuator.finish(cam, {}, 1.0, 2.0, 1));
    return true;
}
bool vvtActualPreviousAndShiftedIvc() {
    auto cam = variableCam();
    ValveTrainState advanced {30.0, 0.0, 1.0};
    REQUIRE(!commonBaseCircle(cam, {}, 606.0, 1).common);
    REQUIRE(commonBaseCircle(cam, advanced, 606.0, 1).common);
    ValveProfileActuator actuator;
    actuator.updateDemand(cam, 1.0, cycle(5900.0, 6000.0), 1);
    REQUIRE(!actuator.finish(cam, advanced, 605.0, std::nullopt, 1));
    REQUIRE(actuator.finish(cam, advanced, 606.0, 1.0, 1)); // Already applied after safe arc.
    actuator.reset();
    actuator.updateDemand(cam, 1.0, cycle(5700.0, 5790.0), 1);
    REQUIRE(!actuator.finish(cam, advanced, 605.0, std::nullopt, 1));
    actuator.updateDemand(cam, 1.0, cycle(5900.0, 6000.0, 2), 1);
    REQUIRE(!actuator.finish(cam, advanced, 606.0, 1.0, 1));
    REQUIRE(actuator.pending());
    REQUIRE(commonBaseCircle(cam, {}, 612.0, 1).common);
    REQUIRE(!commonBaseCircleArc(cam, {}, 606.0, 6.0));
    REQUIRE(!actuator.finish(cam, {}, 612.0, 6.0, 1)); // Current high IVC 611 was crossed.
    REQUIRE(actuator.finish(cam, {}, 613.0, 1.0, 1));
    return true;
}
bool customLobesAndWholeArc() {
    auto cam = variableCam();
    cam.intakeDurationDegrees = cam.highIntakeDurationDegrees = 1.0;
    cam.exhaustDurationDegrees = cam.highExhaustDurationDegrees = 1.0;
    cam.intakeLiftProfile = cam.highIntakeLiftProfile = {{-360.0, 1.0}, {360.0, 1.0}};
    const auto outside = commonBaseCircle(cam, {}, 468.0, 1);
    REQUIRE(outside.common && outside.liftEvaluated);
    REQUIRE(std::all_of(outside.actualLiftMm.begin(), outside.actualLiftMm.end(), [](double value) { return value == 0.0; }));
    REQUIRE(!commonBaseCircle(cam, {}, 469.25, 1).common); // Actual half support is1, not0.5.
    auto state = ValveTrainState {};
    REQUIRE(ValveTrainModel::evaluate(cam, true, state, 469.25, 0.0, 0.0, 0.0).intakeLiftMm > 0.0);
    REQUIRE(ValveTrainModel::evaluate(cam, true, state, 468.0, 0.0, 0.0, 0.0).intakeLiftMm == 0.0);
    ValveProfileActuator actuator;
    actuator.updateDemand(cam, 1.0, cycle(5900.0, 6000.0), 1);
    REQUIRE(!actuator.finish(cam, {}, 468.0, std::nullopt, 1));
    REQUIRE(!actuator.finish(cam, {}, 472.0, 4.0, 1)); // Entire short lobe lies between closed endpoints.
    REQUIRE(actuator.finish(cam, {}, 473.0, 1.0, 1));
    cam.intakeDurationDegrees = cam.highIntakeDurationDegrees = 200.0;
    cam.intakeLiftProfile = cam.highIntakeLiftProfile = {{-100.0, 1.0}, {-10.0, 0.0}, {10.0, 0.0}, {100.0, 1.0}};
    REQUIRE(ValveTrainModel::evaluate(cam, true, state, 470.0, 0.0, 0.0, 0.0).intakeLiftMm == 0.0);
    REQUIRE(!commonBaseCircle(cam, {}, 470.0, 1).common); // Internal zero does not move nominal IVC.
    return true;
}
bool nominalIvcEndpointMustNotRelatch() {
    auto cam = variableCam();
    ValveProfileActuator actuator;
    actuator.updateDemand(cam, 1.0, cycle(5900.0, 6000.0), 1);
    REQUIRE(commonBaseCircle(cam, {}, 611.0, 1).common);
    REQUIRE(!actuator.finish(cam, {}, 611.0, std::nullopt, 1));
    REQUIRE(actuator.pending());
    // Simulator counts distance==0: switching on611->612 would cross high IVC
    //611 again even though low IVC594 was already passed. Both lifts are zero.
    REQUIRE(!commonBaseCircleArc(cam, {}, 611.0, 1.0));
    REQUIRE(!actuator.finish(cam, {}, 612.0, 1.0, 1));
    REQUIRE(actuator.pending() && !actuator.appliedHigh());
    REQUIRE(actuator.finish(cam, {}, 613.0, 1.0, 1));
    return true;
}

bool nominalEvoEndpointMustNotOpenEarly() {
    auto cam = variableCam();
    ValveProfileActuator actuator;
    actuator.updateDemand(cam, 1.0, cycle(5900.0, 6000.0), 1);
    REQUIRE(commonBaseCircle(cam, {}, 109.0, 1).common);
    REQUIRE(commonBaseCircle(cam, {}, 110.0, 1).common);
    REQUIRE(!actuator.finish(cam, {}, 109.0, std::nullopt, 1));
    REQUIRE(actuator.pending());
    // Simulator counts distance==travel: zero lift exactly at high EVO110
    // does not authorize a new profile event on the arc109->110.
    REQUIRE(!commonBaseCircleArc(cam, {}, 109.0, 1.0));
    REQUIRE(!actuator.finish(cam, {}, 110.0, 1.0, 1));
    auto vvt = ValveTrainState {};
    REQUIRE(ValveTrainModel::evaluate(cam, true, vvt, 111.0, 0.0, 0.0, 0.0).exhaustLiftMm > 0.0);
    REQUIRE(!actuator.finish(cam, {}, 111.0, 1.0, 1));
    REQUIRE(!actuator.pending() && !actuator.appliedHigh());
    return true;
}

bool onceOnlyVvtAndNonvariable() {
    auto cam = variableCam();
    cam.continuousControl.enabled = true;
    cam.continuousControl.samples = {{0.0, 0.0, 30.0, -15.0, 1.2}};
    ValveTrainState state;
    ValveProfileActuator actuator;
    actuator.updateDemand(cam, 1.0, cycle(5900.0, 6000.0), 1);
    (void) ValveTrainModel::evaluate(cam, actuator.appliedHigh(), state, 0.0, 6000.0, 1.0, 0.001);
    auto actual = state;
    REQUIRE(!actuator.finish(cam, state, 0.0, std::nullopt, 1));
    REQUIRE(sameState(actual, state));
    (void) ValveTrainModel::evaluate(cam, actuator.appliedHigh(), state, 1.0, 6000.0, 1.0, 0.001);
    actual = state;
    REQUIRE(actuator.finish(cam, state, 1.0, 1.0, 1));
    (void) ValveTrainModel::evaluate(cam, actuator.appliedHigh(), state, 1.0, 6000.0, 1.0, 0.0);
    REQUIRE(sameState(actual, state));
    cam.variableProfileEnabled = false;
    auto plain = ValveTrainState {};
    auto controlled = plain;
    for (int index = 0; index < 720; ++index) {
        const auto phase = static_cast<double>(index);
        const auto baseline = ValveTrainModel::evaluate(cam, false, plain, phase, 6000.0, 1.0, 0.0001);
        actuator.updateDemand(cam, 1.0, cycle(5900.0, 6000.0), 1);
        const auto selected = actuator.finish(cam, controlled, phase, 1.0, 1);
        const auto observed = ValveTrainModel::evaluate(cam, selected, controlled, phase, 6000.0, 1.0, 0.0001);
        REQUIRE(!selected && sameState(plain, controlled));
        REQUIRE(std::bit_cast<std::uint64_t>(baseline.intakeLiftMm) == std::bit_cast<std::uint64_t>(observed.intakeLiftMm));
        REQUIRE(std::bit_cast<std::uint64_t>(baseline.exhaustLiftMm) == std::bit_cast<std::uint64_t>(observed.exhaustLiftMm));
    }
    return true;
}
bool resetRevisionInvalidAndLongJump() {
    auto cam = variableCam();
    ValveProfileActuator actuator;
    actuator.updateDemand(cam, 1.0, cycle(5900.0, 6000.0), 1);
    REQUIRE(!actuator.finish(cam, {}, 0.0, std::nullopt, 1));
    actuator.updateDemand(cam, 1.0, cycle(5900.0, 6000.0), 2);
    REQUIRE(!actuator.finish(cam, {}, 1.0, 1.0, 2));
    REQUIRE(actuator.finish(cam, {}, 2.0, 1.0, 2));
    actuator.reset();
    REQUIRE(!actuator.pending() && !actuator.appliedHigh() && !actuator.requestedHigh());
    actuator.updateDemand(cam, 1.0, cycle(5900.0, 6000.0), 2);
    REQUIRE(!actuator.finish(cam, {}, 0.0, std::nullopt, 2));
    REQUIRE(!actuator.finish(cam, {}, 0.0, 720.0, 2));
    auto invalid = ValveTrainState {};
    invalid.intakeAdvanceDegrees = std::numeric_limits<double>::quiet_NaN();
    REQUIRE(!actuator.finish(cam, invalid, 1.0, 1.0, 2));
    REQUIRE(!actuator.pending() && !actuator.requestedHigh());
    return true;
}
}
int runValveProfileActuatorContracts() {
    for (const auto& contract : {
        std::pair<std::string_view, bool(*)()> {"shaft-constant-and-initial-partial", shaftConstantAndPartial},
        {"shaft-exact-accelerating-and-decelerating-crossing", shaftAcceleratingAndDecelerating},
        {"shaft-multiple-wrap-and-signed-zero-roundoff", shaftMultipleWrapAndRoundoff},
        {"shaft-stop-start-reset-and-invalid-input", shaftStopRestartAndInvalid},
        {"cycle-range-equality-mixed-hold-and-throttle-recovery", rangeThresholdAndThrottle},
        {"missing-demand-and-pending-cancellation", missingDemandAndPendingCancellation},
        {"open-valve-rejection-and-safe-phase-wrap", openValveAndSafeWrap},
        {"actual-previous-vvt-and-moving-ivc-target", vvtActualPreviousAndShiftedIvc},
        {"custom-hard-gate-internal-zero-and-entire-short-lobe", customLobesAndWholeArc},
        {"nominal-ivc-inclusive-endpoint-no-relatch", nominalIvcEndpointMustNotRelatch},
        {"nominal-evo-inclusive-endpoint-no-early-opening", nominalEvoEndpointMustNotOpenEarly},
        {"single-vvt-advance-and-nonvariable-bit-control", onceOnlyVvtAndNonvariable},
        {"reset-config-revision-invalid-vvt-and-full-turn", resetRevisionInvalidAndLongJump}}) {
        if (!contract.second()) return EXIT_FAILURE;
        ++contracts;
        std::cout << "PASS " << contract.first << '\n';
    }
    std::cout << "contracts=" << contracts << " checks=" << checks << " failed=0\n";
    return EXIT_SUCCESS;
}
