#include <enginelab/simulation/EngineSimulator.hpp>
#include <algorithm>
#include <cmath>
#include <numbers>
#include <stdexcept>

namespace enginelab {
namespace {
[[nodiscard]] double smooth(double current, double target, double dt, double rate) noexcept {
    return current + (target - current) * (1.0 - std::exp(-dt * rate));
}
}

EngineSimulator::EngineSimulator(EngineConfig config, IEcuModel& ecu, IPhysicsModel& physics,
                                 IFiringEventGenerator& events, IExhaustModel& exhaust)
    : config_(std::move(config)), ecu_(ecu), physics_(physics), eventGenerator_(events), exhaust_(exhaust) {
    if (const auto error = validateEngineConfig(config_)) throw std::invalid_argument(*error);
    reset();
}

SimulationFrame EngineSimulator::step(double dtSeconds, const EngineControls& controls) noexcept {
    const auto dt = std::isfinite(dtSeconds) ? std::clamp(dtSeconds, 0.0, 0.05) : 0.0;
    if (dt <= 0.0) {
        return { state_ };
    }
    auto safeControls = controls;
    safeControls.throttle = std::isfinite(controls.throttle) ? std::clamp(controls.throttle, 0.0, 1.0) : 0.0;
    safeControls.load = std::isfinite(controls.load) ? std::clamp(controls.load, 0.0, 1.0) : 0.0;
    const auto stepStartTime = state_.simulationTimeSeconds;
    const auto previousAngle = state_.crankAngleDegrees;

    constexpr double maxSubStepDt = 1.0 / 240.0; // 240 Hz sub-step limit
    const auto subStepCount = std::max(std::size_t { 1 }, static_cast<std::size_t>(std::ceil(dt / maxSubStepDt)));
    const auto subDt = dt / static_cast<double>(subStepCount);
    double totalTravelled = 0.0;

    for (std::size_t subStep = 0; subStep < subStepCount; ++subStep) {
        const auto subPreviousRpm = state_.rpm;
        const auto subPreviousAngle = state_.crankAngleDegrees;

        state_.load = smooth(state_.load, safeControls.load, subDt, 12.0);
        const auto ecuCommand = ecu_.evaluate(config_, state_, safeControls);
        state_.throttle = smooth(state_.throttle, ecuCommand.effectiveThrottle, subDt, 10.0);
        const auto stationary = state_.rpm < 20.0 && !safeControls.starterEngaged;
        if (stationary) {
            state_.manifoldPressureKpa = smooth(state_.manifoldPressureKpa, config_.ambientPressureKpa, subDt, 13.0);
        } else {
            constexpr double airGasConstant = 287.05;
            const auto temperatureK = std::max(240.0, config_.ambientTemperatureC + 273.15);
            const auto ambientPressurePa = config_.ambientPressureKpa * 1'000.0;
            const auto manifoldPressurePa = state_.manifoldPressureKpa * 1'000.0;

            const auto throttleRadiusM = config_.throttleDiameterMm * 0.0005;
            const auto throttlePlateAreaM2 = std::numbers::pi * throttleRadiusM * throttleRadiusM;
            const auto effectiveAreaM2 = throttlePlateAreaM2 * (0.006 + 0.994 * std::pow(state_.throttle, 1.75));

            const auto pressureRatio = std::clamp(manifoldPressurePa / ambientPressurePa, 0.0, 1.0);
            constexpr double gamma = 1.4;
            constexpr double criticalRatio = 0.52828;
            double psi = 0.0;
            if (pressureRatio <= criticalRatio) {
                psi = 0.4842; // Choked flow
            } else {
                constexpr double c1 = 2.0 * gamma / (gamma - 1.0);
                const auto r1 = std::pow(pressureRatio, 2.0 / gamma);
                const auto r2 = std::pow(pressureRatio, (gamma + 1.0) / gamma);
                psi = std::sqrt(std::max(0.0, c1 * (r1 - r2)));
            }
            const auto intakeMassFlowKgPerSecond = effectiveAreaM2 * (ambientPressurePa / std::sqrt(airGasConstant * temperatureK)) * psi;

            const auto manifoldDensity = manifoldPressurePa / (airGasConstant * temperatureK);
            const auto displacementM3 = engineDisplacementLitres(config_) * 0.001;
            const auto priorVe = std::clamp(state_.volumetricEfficiency, 0.35, 1.15);
            const auto engineMassFlowKgPerSecond = displacementM3 * (state_.rpm / 120.0) * priorVe * manifoldDensity;
            const auto plenumVolumeM3 = config_.plenumVolumeLitres * 0.001;
            const auto pressureRatePaPerSecond = (intakeMassFlowKgPerSecond - engineMassFlowKgPerSecond)
                * airGasConstant * temperatureK / plenumVolumeM3;
            state_.manifoldPressureKpa = std::clamp(state_.manifoldPressureKpa
                + pressureRatePaPerSecond * subDt / 1'000.0, 10.0, config_.ambientPressureKpa * 1.01);
        }

        const auto backPressure = exhaust_.backPressureKpa(state_);
        const auto combustion = physics_.evaluateCombustion(config_, state_, safeControls, ecuCommand, backPressure);
        const auto cranking = safeControls.starterEngaged && state_.rpm < 620.0 && state_.damage < 1.0;
        const auto displacement = engineDisplacementLitres(config_);
        double pistonSpeedSum = 0.0;
        double reciprocatingTorque = 0.0;
        double peakPistonAcceleration = 0.0;
        double combustionPulseSum = 0.0;

        for (std::size_t cylinderIndex = 0; cylinderIndex < config_.cylinders.size(); ++cylinderIndex) {
            const auto& cylinder = config_.cylinders[cylinderIndex];
            const auto theta = (state_.crankAngleDegrees - cylinder.crankOffsetDegrees) * std::numbers::pi / 180.0;
            const auto cyclePhase = std::fmod(state_.crankAngleDegrees - cylinder.crankOffsetDegrees + 1'440.0, 720.0);
            const auto wrappedCycle = cyclePhase < previousCylinderPhases_[cylinderIndex];
            const auto reachedIgnitionWindow = !wrappedCycle && previousCylinderPhases_[cylinderIndex] < 600.0
                && cyclePhase >= 600.0;
            if (!combustion.combustionEnabled) {
                cylinderMisfires_[cylinderIndex] = false;
                cylinderMisfirePrepared_[cylinderIndex] = false;
            } else if (reachedIgnitionWindow || (wrappedCycle && !cylinderMisfirePrepared_[cylinderIndex])) {
                randomState_ ^= randomState_ << 13U;
                randomState_ ^= randomState_ >> 17U;
                randomState_ ^= randomState_ << 5U;
                const auto randomUnit = static_cast<double>(randomState_) / static_cast<double>(0xffffffffU);
                cylinderMisfires_[cylinderIndex] = randomUnit < combustion.misfireProbability;
                cylinderMisfirePrepared_[cylinderIndex] = true;
            }
            if (wrappedCycle) cylinderMisfirePrepared_[cylinderIndex] = false;
            previousCylinderPhases_[cylinderIndex] = cyclePhase;
            const auto pulse = combustion.combustionEnabled && !cylinderMisfires_[cylinderIndex]
                ? enginelab::combustionPulse(cyclePhase, ecuCommand.ignitionAdvanceDegrees,
                                             cylinder.ignitionOffsetDegrees) : 0.0;
            combustionPulseSum += pulse * std::clamp(1.0 + cylinder.efficiencyOffset, 0.8, 1.2);
            const auto radiusM = cylinder.strokeMm * 0.0005;
            const auto rodM = cylinder.connectingRodMm * 0.001;
            const auto massKg = cylinder.pistonMassGrams * 0.001;
            const auto ratio = radiusM / rodM;

            // Exact kinematic acceleration formula
            const auto cosTheta = std::cos(theta);
            const auto sinTheta = std::sin(theta);
            const auto sin2 = sinTheta * sinTheta;
            const auto ratio2 = ratio * ratio;
            const auto denom = 1.0 - ratio2 * sin2;
            const auto exactTerm = (ratio * std::cos(2.0 * theta) + ratio2 * ratio * sin2 * sin2) / (denom * std::sqrt(denom));
            const auto acceleration = radiusM * state_.angularVelocityRadPerSecond * state_.angularVelocityRadPerSecond
                * (cosTheta + exactTerm);

            const auto inertiaForce = massKg * acceleration;
            reciprocatingTorque -= inertiaForce * radiusM * std::sin(theta);
            peakPistonAcceleration = std::max(peakPistonAcceleration, std::abs(acceleration));
            pistonSpeedSum += 2.0 * cylinder.strokeMm * 0.001 * state_.rpm / 60.0;
        }
        const auto meanPistonSpeed = pistonSpeedSum / static_cast<double>(config_.cylinders.size());
        const auto pumpingTorque = state_.rpm > 30.0
            ? std::max(0.0, config_.ambientPressureKpa - state_.manifoldPressureKpa) * displacement * 0.055 : 0.0;
        const auto mechanicalFriction = state_.rpm > 1.0
            ? displacement * (2.2 + config_.frictionCoefficient * 18.0 + state_.rpm * 0.00075
                              + state_.rpm * state_.rpm * 0.000000045 + meanPistonSpeed * 0.12) : 0.0;
        const auto seizureTorque = state_.damage >= 1.0 ? displacement * 250.0
            + state_.angularVelocityRadPerSecond * 0.22 : 0.0;
        const auto frictionTorque = mechanicalFriction + pumpingTorque + seizureTorque;
        const auto loadTorque = state_.rpm > 40.0 && !safeControls.starterEngaged
            ? state_.load * displacement * 105.0 : 0.0;
        const auto starterPeakTorque = 24.0 + displacement * 17.0;
        const auto starterTorque = cranking ? starterPeakTorque * std::clamp(1.0 - state_.rpm / 760.0, 0.18, 1.0) : 0.0;
        const auto normalizedCombustionPulse = std::clamp(combustionPulseSum * 8.0
            / static_cast<double>(config_.cylinders.size()), 0.0, 2.8);
        const auto indicatedTorque = combustion.indicatedTorqueNm * normalizedCombustionPulse;
        const auto brakeTorque = indicatedTorque - frictionTorque;
        const auto boundedReciprocatingTorque = std::clamp(reciprocatingTorque,
            -std::max(20.0, indicatedTorque * 0.5), std::max(20.0, indicatedTorque * 0.5));
        const auto netTorque = brakeTorque + starterTorque - loadTorque + boundedReciprocatingTorque;

        auto omega = state_.angularVelocityRadPerSecond;
        omega = std::max(0.0, omega + netTorque / config_.rotatingInertiaKgM2 * subDt);
        state_.angularVelocityRadPerSecond = omega;
        state_.rpm = omega * 60.0 / (2.0 * std::numbers::pi);
        if (state_.rpm < 0.5 && !cranking && indicatedTorque <= 0.0) {
            state_.rpm = 0.0;
            state_.angularVelocityRadPerSecond = 0.0;
        }
        const auto subTravelled = (subPreviousRpm + state_.rpm) * 0.5 * 6.0 * subDt;
        totalTravelled += subTravelled;
        state_.crankAngleDegrees = std::fmod(subPreviousAngle + subTravelled, eventGenerator_.cycleDegrees());
        state_.simulationTimeSeconds += subDt;

        state_.indicatedTorqueNm = indicatedTorque;
        state_.frictionTorqueNm = frictionTorque;
        state_.reciprocatingTorqueNm = boundedReciprocatingTorque;
        state_.meanPistonSpeedMps = meanPistonSpeed;
        state_.loadTorqueNm = loadTorque;
        state_.starterTorqueNm = starterTorque;
        state_.netTorqueNm = netTorque;
        state_.torqueNm = brakeTorque;
        state_.cycleAveragedTorqueNm = smooth(state_.cycleAveragedTorqueNm, brakeTorque, subDt, 8.0);
        state_.knockLevel = smooth(state_.knockLevel, combustion.knockLevel, subDt, 10.0);
        state_.misfireRate = smooth(state_.misfireRate, combustion.misfireProbability, subDt, 6.0);
        state_.airFuelRatio = combustion.actualAirFuelRatio;
        state_.targetAirFuelRatio = ecuCommand.targetAirFuelRatio;
        state_.ignitionAdvanceDegrees = ecuCommand.ignitionAdvanceDegrees;
        state_.volumetricEfficiency = state_.rpm > 20.0 ? combustion.volumetricEfficiency : 0.0;
        state_.airMassMgPerCycle = state_.rpm > 20.0 ? combustion.airMassMgPerCycle : 0.0;
        state_.injectedFuelMgPerCycle = ecuCommand.fuelEnabled ? combustion.fuelMassMgPerCycle : 0.0;
        state_.fuelFlowGramsPerSecond = state_.injectedFuelMgPerCycle * (state_.rpm / 120.0) / 1'000.0;
        state_.airFlowGramsPerSecond = state_.airMassMgPerCycle * (state_.rpm / 120.0) / 1'000.0;
        state_.lambda = state_.airFuelRatio / 14.7;
        state_.exhaustPressureKpa = backPressure;
        state_.powerKw = state_.torqueNm * state_.angularVelocityRadPerSecond / 1'000.0;
        state_.cycleAveragedPowerKw = state_.cycleAveragedTorqueNm * state_.angularVelocityRadPerSecond / 1'000.0;
        state_.brakeSpecificFuelConsumptionGPerKwh = state_.cycleAveragedPowerKw > 1.0
            ? state_.fuelFlowGramsPerSecond * 3'600.0 / state_.cycleAveragedPowerKw : 0.0;
        state_.mechanicalEfficiency = indicatedTorque > 1.0 ? std::clamp(brakeTorque / indicatedTorque, 0.0, 1.0) : 0.0;
        state_.oilPressureKpa = state_.rpm > 0.0
            ? std::clamp(80.0 + state_.rpm * 0.055 - std::max(0.0, state_.oilTemperatureC - 105.0) * 2.0, 0.0, 520.0) : 0.0;
        state_.peakPistonAccelerationG = peakPistonAcceleration / 9.80665;

        const auto thermostatOpening = std::clamp((state_.coolantTemperatureC - 82.0) / 12.0, 0.08, 1.0);
        const auto coolingAirflow = config_.coolingEfficiency * (0.18 + thermostatOpening
            * (0.42 + state_.rpm / std::max(1.0, config_.redlineRpm) * 0.8));
        const auto coolantTarget = config_.ambientTemperatureC + combustion.heatPowerKw * 0.42 / coolingAirflow;
        const auto oilTarget = config_.ambientTemperatureC + combustion.heatPowerKw * 0.48 / std::max(0.5, coolingAirflow * 0.85);
        const auto exhaustTarget = config_.ambientTemperatureC + combustion.heatOutput * 760.0;
        state_.coolantTemperatureC = smooth(state_.coolantTemperatureC, coolantTarget, subDt, 0.030);
        state_.oilTemperatureC = smooth(state_.oilTemperatureC, oilTarget, subDt, 0.019);
        state_.exhaustTemperatureC = smooth(state_.exhaustTemperatureC, exhaustTarget, subDt, 0.18);
        const auto overrev = std::max(0.0, state_.rpm / config_.redlineRpm - 1.0);
        const auto oilStarvation = state_.rpm > 800.0 && state_.oilPressureKpa < 90.0 ? 1.0 : 0.0;
        const auto excessivePistonAcceleration = std::max(0.0, state_.peakPistonAccelerationG - 5'500.0) / 5'500.0;
        const auto damageRate = state_.knockLevel * state_.throttle * 0.0012 + overrev * overrev * 0.022
                              + std::max(0.0, state_.coolantTemperatureC - 115.0) * 0.000025 + oilStarvation * 0.002
                              + excessivePistonAcceleration * excessivePistonAcceleration * 0.002;
        state_.damage = std::clamp(state_.damage + damageRate * subDt, 0.0, 1.0);
        state_.wear = std::clamp(state_.wear + (state_.rpm / config_.redlineRpm * 0.000002 + damageRate * 0.12) * subDt, 0.0, 1.0);
        state_.runningState = determineRunningState(safeControls);
        state_.cylinderStateCount = std::min(config_.cylinders.size(), state_.cylinderStates.size());
        for (std::size_t index = 0; index < state_.cylinderStateCount; ++index) {
            const auto& cylinder = config_.cylinders[index];
            const auto phase = std::fmod(state_.crankAngleDegrees - cylinder.crankOffsetDegrees + 1'440.0, 720.0);
            const auto active = combustion.combustionEnabled && !cylinderMisfires_[index];
            const auto pulse = active ? enginelab::combustionPulse(phase, ecuCommand.ignitionAdvanceDegrees,
                                                                   cylinder.ignitionOffsetDegrees) : 0.0;
            state_.cylinderStates[index] = { cylinder.id, phase,
                1.0 + combustion.pressureEstimateBar * pulse,
                pulse, combustion.misfireProbability,
                config_.ambientTemperatureC + (state_.exhaustTemperatureC - config_.ambientTemperatureC)
                    * std::clamp(1.0 + cylinder.efficiencyOffset, 0.8, 1.2),
                active, combustion.combustionEnabled && cylinderMisfires_[index] };
        }
    }

    SimulationFrame frame;
    frame.state = state_;
    const auto finalEcuCommand = ecu_.evaluate(config_, state_, safeControls);
    const auto finalBackPressure = exhaust_.backPressureKpa(state_);
    const auto finalCombustion = physics_.evaluateCombustion(config_, state_, safeControls, finalEcuCommand, finalBackPressure);
    if (finalEcuCommand.fuelEnabled && finalEcuCommand.sparkEnabled && totalTravelled > 0.0) {
        frame.firingEventCount = eventGenerator_.generate(config_, state_, finalEcuCommand, finalCombustion,
            stepStartTime, previousAngle, totalTravelled, dt, frame.firingEvents);
        frame.droppedFiringEventCount = eventGenerator_.droppedEventCountLastGenerate();
        for (std::size_t index = 0; index < frame.firingEventCount; ++index) exhaust_.process(frame.firingEvents[index]);
    }
    return frame;
}

void EngineSimulator::reset() noexcept {
    ecu_.reset();
    eventGenerator_.reset();
    state_ = {};
    state_.coolantTemperatureC = config_.ambientTemperatureC;
    state_.oilTemperatureC = config_.ambientTemperatureC;
    state_.exhaustTemperatureC = config_.ambientTemperatureC;
    state_.manifoldPressureKpa = config_.ambientPressureKpa;
    state_.exhaustPressureKpa = config_.ambientPressureKpa;
    randomState_ = 0x6d2b79f5U;
    cylinderMisfires_.fill(false);
    cylinderMisfirePrepared_.fill(false);
    for (std::size_t index = 0; index < config_.cylinders.size(); ++index) {
        previousCylinderPhases_[index] = std::fmod(state_.crankAngleDegrees - config_.cylinders[index].crankOffsetDegrees + 1'440.0, 720.0);
    }
}

RunningState EngineSimulator::determineRunningState(const EngineControls& controls) const noexcept {
    if (state_.damage >= 1.0) return RunningState::destroyed;
    if (state_.damage > 0.5) return RunningState::damaged;
    if (state_.coolantTemperatureC > 118.0) return RunningState::overheating;
    if (state_.knockLevel > 0.45) return RunningState::knocking;
    if (controls.starterEngaged && state_.rpm < 620.0) return RunningState::cranking;
    if (state_.rpm < 250.0) return RunningState::stopped;
    if (state_.rpm < config_.idleRpm * 1.2) return state_.misfireRate > 0.18 ? RunningState::unstable : RunningState::idling;
    return RunningState::running;
}
} // namespace enginelab
