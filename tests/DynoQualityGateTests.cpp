#include <enginelab/runtime/DynoQualityGate.hpp>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string_view>

namespace {

void require(bool condition, std::string_view message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(EXIT_FAILURE);
    }
}

enginelab::DynoQualityGateInput validInput() {
    enginelab::DynoQualityGateInput input;
    input.mode = enginelab::DynoAcquisitionMode::steppedCalibration;
    input.targetRpm = 3'000.0;
    input.rampRateRpmPerSecond = 500.0;
    input.measuredCycleTorqueNm = 100.0;
    input.prepared = true;
    input.protocolReady = true;
    input.cycleContinuous = true;
    return input;
}

enginelab::EngineState validState() {
    enginelab::EngineState state;
    state.rpm = 3'000.0;
    return state;
}

enginelab::DynoAbsorberOutput validAbsorber() {
    enginelab::DynoAbsorberOutput output;
    output.brakeTorqueNm = 100.0;
    output.filteredRpm = 3'000.0;
    output.filteredAccelerationRpmPerSecond = 0.0;
    output.contacted = true;
    output.contactFraction = 1.0;
    output.unclampedBrakeTorqueNm = 100.0;
    return output;
}

void requireOnly(enginelab::DynoQualityReason actual,
                 enginelab::DynoQualityReason expected,
                 std::string_view message) {
    require(actual == expected, message);
}

} // namespace

int main() {
    using enginelab::DynoAcquisitionMode;
    using enginelab::DynoQualityGate;
    using enginelab::DynoQualityReason;
    using enginelab::hasDynoQualityReason;

    const DynoQualityGate gate;
    auto input = validInput();
    auto state = validState();
    auto absorber = validAbsorber();
    require(gate.evaluate(input, state, absorber).accepted(),
            "a prepared, tracked, continuous sample must pass");

    input.prepared = false;
    requireOnly(gate.evaluate(input, state, absorber).reasons,
                DynoQualityReason::notPrepared,
                "preparation must have one explicit reason");
    input = validInput();
    input.protocolReady = false;
    requireOnly(gate.evaluate(input, state, absorber).reasons,
                DynoQualityReason::protocolNotReady,
                "protocol readiness must have one explicit reason");
    input = validInput();
    input.recoveryActive = true;
    requireOnly(gate.evaluate(input, state, absorber).reasons,
                DynoQualityReason::recoveryActive,
                "recovery must have one explicit reason");
    input = validInput();
    input.cycleContinuous = false;
    requireOnly(gate.evaluate(input, state, absorber).reasons,
                DynoQualityReason::discontinuousCycle,
                "cycle continuity must have one explicit reason");

    input = validInput();
    input.measuredCycleTorqueNm = 1.4;
    requireOnly(gate.evaluate(input, state, absorber).reasons,
                DynoQualityReason::insufficientEngineTorque,
                "the minimum torque threshold is strict");
    input.measuredCycleTorqueNm = std::nextafter(1.4,
        std::numeric_limits<double>::infinity());
    require(gate.evaluate(input, state, absorber).accepted(),
            "torque just above the strict threshold must pass");

    input = validInput();
    absorber = validAbsorber();
    absorber.contactFraction = std::nextafter(0.95, 0.0);
    require(gate.evaluate(input, state, absorber).accepted(),
            "sub-picometric contact rounding must not chatter the gate");
    absorber.contactFraction = 0.949;
    requireOnly(gate.evaluate(input, state, absorber).reasons,
                DynoQualityReason::noBrakeContact,
                "materially partial contact must be rejected");

    absorber = validAbsorber();
    absorber.filteredRpm = 3'060.0;
    absorber.filteredAccelerationRpmPerSecond = 120.0;
    require(gate.evaluate(input, state, absorber).accepted(),
            "steady limits are inclusive");
    absorber.filteredRpm = 3'060.01;
    absorber.filteredAccelerationRpmPerSecond = 120.01;
    auto result = gate.evaluate(input, state, absorber);
    require(hasDynoQualityReason(result.reasons,
                DynoQualityReason::speedTrackingError)
            && hasDynoQualityReason(result.reasons,
                DynoQualityReason::accelerationOutOfBounds),
            "speed and acceleration reasons must accumulate");

    input = validInput();
    input.mode = DynoAcquisitionMode::continuousRamp;
    input.rampRateRpmPerSecond = 500.0;
    absorber = validAbsorber();
    absorber.filteredRpm = 3'150.0;
    absorber.filteredAccelerationRpmPerSecond = 1'500.0;
    result = gate.evaluate(input, state, absorber);
    require(result.accepted()
            && result.maximumAllowedSpeedErrorRpm == 150.0
            && result.maximumAllowedAccelerationRpmPerSecond == 1'500.0,
            "ramp limits must scale with the commanded rate");

    absorber = validAbsorber();
    absorber.saturatedLow = true;
    require(gate.evaluate(input, state, absorber).accepted(),
            "low clamp alone is not an absorber-capacity fault");
    absorber.saturatedHigh = true;
    requireOnly(gate.evaluate(input, state, absorber).reasons,
                DynoQualityReason::absorberCapacityLimited,
                "high clamp must expose absorber capacity exhaustion");

    absorber = validAbsorber();
    state = validState();
    state.ecuHardRevLimiterActive = true;
    requireOnly(gate.evaluate(input, state, absorber).reasons,
                DynoQualityReason::revLimiterActive,
                "any active rev limiter must reject acquisition");

    state = validState();
    input = validInput();
    absorber = validAbsorber();
    input.targetRpm = std::numeric_limits<double>::quiet_NaN();
    input.prepared = false;
    input.cycleContinuous = false;
    result = gate.evaluate(input, state, absorber);
    require(hasDynoQualityReason(result.reasons,
                DynoQualityReason::nonFinite)
            && hasDynoQualityReason(result.reasons,
                DynoQualityReason::notPrepared)
            && hasDynoQualityReason(result.reasons,
                DynoQualityReason::discontinuousCycle),
            "non-finite input must not hide independent protocol reasons");

    std::cout << "PASS: deterministic dyno quality attribution\n";
    return 0;
}
