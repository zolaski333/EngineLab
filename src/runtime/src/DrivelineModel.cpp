#include <enginelab/runtime/DrivelineModel.hpp>
#include <algorithm>
#include <cmath>
#include <numbers>

namespace enginelab {

DrivelineModel::DrivelineModel(const EngineConfig& config) : config_(config) {
    clutchTemperatureC_ = config_.ambientTemperatureC;
}

void DrivelineModel::requestGear(int gear) noexcept {
    const auto maximum = static_cast<int>(config_.transmission.gearRatios.size()) - 1;
    requestedGear_ = std::clamp(gear, -2, maximum);
}

void DrivelineModel::shiftUp() noexcept { requestGear(requestedGear_ + 1); }
void DrivelineModel::shiftDown() noexcept { requestGear(requestedGear_ - 1); }

double DrivelineModel::selectedRatio() const noexcept {
    if (engagedGear_ == -2) return -config_.transmission.reverseRatio * config_.transmission.finalDriveRatio;
    if (engagedGear_ < 0 || engagedGear_ >= static_cast<int>(config_.transmission.gearRatios.size())) return 0.0;
    return config_.transmission.gearRatios[static_cast<std::size_t>(engagedGear_)]
        * config_.transmission.finalDriveRatio;
}

DrivelineOutput DrivelineModel::advance(double dt, const EngineState& engineState,
                                        double requestedLoad, double clutchPedal,
                                        double brakePressure) noexcept {
    DrivelineOutput output;
    dt = std::clamp(dt, 0.0, 0.05);
    clutchPedal = std::clamp(clutchPedal, 0.0, 1.0);
    brakePressure = std::clamp(brakePressure, 0.0, 1.0);
    const auto& transmission = config_.transmission;
    const auto& vehicle = config_.vehicle;
    const auto maximumGear = static_cast<int>(transmission.gearRatios.size()) - 1;

    if (transmission.automaticShifting && !shiftInProgress_ && engagedGear_ >= 0) {
        if (engineState.rpm >= transmission.automaticUpshiftRpm && engagedGear_ < maximumGear
            && engineState.throttle > 0.12) requestGear(engagedGear_ + 1);
        else if (engineState.rpm <= transmission.automaticDownshiftRpm && engagedGear_ > 0)
            requestGear(engagedGear_ - 1);
    }
    if (!shiftInProgress_ && requestedGear_ != engagedGear_) {
        shiftInProgress_ = true;
        shiftTargetGear_ = requestedGear_;
        shiftElapsedSeconds_ = 0.0;
    }
    double shiftProgress = 0.0;
    double effectiveClutch = clutchPedal;
    if (shiftInProgress_) {
        shiftElapsedSeconds_ += dt;
        shiftProgress = std::clamp(shiftElapsedSeconds_ / transmission.shiftDurationSeconds, 0.0, 1.0);
        if (shiftProgress < 0.35) effectiveClutch *= 1.0 - shiftProgress / 0.35;
        else if (shiftProgress < 0.60) effectiveClutch = 0.0;
        else effectiveClutch *= (shiftProgress - 0.60) / 0.40;
        if (shiftProgress >= 0.48) engagedGear_ = shiftTargetGear_;
        if (shiftProgress >= 1.0) { engagedGear_ = shiftTargetGear_; shiftInProgress_ = false; shiftProgress = 0.0; }
        output.torqueCutMultiplier = 1.0 - transmission.shiftTorqueCutFraction
            * std::sin(std::numbers::pi * std::clamp(shiftProgress, 0.0, 1.0));
    }

    const auto totalRatio = selectedRatio();
    const auto engineOmega = engineState.angularVelocityRadPerSecond;
    const auto wheelInertia = std::max(0.01, transmission.drivenWheelInertiaKgM2
        + transmission.differentialInertiaKgM2
        + transmission.gearboxInputInertiaKgM2 * totalRatio * totalRatio);
    const auto normalForce = vehicle.massKg * 9.80665 * vehicle.drivenAxleWeightFraction;
    const auto tractionLimit = vehicle.tireFrictionCoefficient * normalForce;
    const auto tireStiffnessNPerMps = normalForce * 7.5;
    const auto brakeForceCapacity = vehicle.maximumBrakeForceN * brakePressure;
    // The tire relaxation time is much shorter than the public simulation step.
    // Integrating that stiff coupling at 1 ms keeps the result stable at every time scale
    // without weakening the physical tire stiffness or hiding instability behind clamps.
    const auto mechanicalStepCount = std::max(1, static_cast<int>(std::ceil(dt / 0.001)));
    const auto mechanicalDt = mechanicalStepCount > 0 ? dt / mechanicalStepCount : 0.0;
    double clutchTorqueIntegral = 0.0;
    double wheelTorqueIntegral = 0.0;
    double clutchLossEnergy = 0.0;
    double wheelInputWork = 0.0;
    double roadWork = 0.0;
    double brakeWork = 0.0;
    double tireSlipWork = 0.0;
    double lastSlipRpm = 0.0;
    double lastTireForce = 0.0;
    bool tractionLimited = false;
    for (int step = 0; step < mechanicalStepCount; ++step) {
        const auto previousWheelOmega = wheelAngularVelocityRadPerSecond_;
        const auto previousVehicleSpeed = vehicleSpeedMps_;
        const auto gearboxOmega = previousWheelOmega * totalRatio;
        const auto slipOmega = engineOmega - gearboxOmega;
        lastSlipRpm = slipOmega * 60.0 / (2.0 * std::numbers::pi);
        const auto fade = std::clamp((transmission.clutchFailureTemperatureC - clutchTemperatureC_)
            / (transmission.clutchFailureTemperatureC - transmission.clutchFadeStartTemperatureC), 0.0, 1.0);
        const auto capacity = transmission.maxClutchTorqueNm * effectiveClutch * fade;
        double clutchTorque = 0.0;
        if (totalRatio != 0.0 && effectiveClutch > 0.0)
            clutchTorque = std::clamp(lastSlipRpm * transmission.clutchSlipStiffnessNmPerRpm,
                                      -capacity, capacity);
        const auto wheelTorque = clutchTorque * totalRatio * transmission.drivelineEfficiency;

        const auto tireSurfaceSpeed = previousWheelOmega * vehicle.tireRadiusM;
        const auto slipVelocity = tireSurfaceSpeed - previousVehicleSpeed;
        const auto unconstrainedTireForce = slipVelocity * tireStiffnessNPerMps;
        lastTireForce = std::clamp(unconstrainedTireForce, -tractionLimit, tractionLimit);
        tractionLimited = tractionLimited || std::abs(unconstrainedTireForce) > tractionLimit + 1.0e-6;
        const auto motionSign = std::abs(previousWheelOmega) > 0.01
            ? std::copysign(1.0, previousWheelOmega)
            : (std::abs(previousVehicleSpeed) > 0.01 ? std::copysign(1.0, previousVehicleSpeed) : 0.0);
        const auto brakeTorque = brakeForceCapacity * vehicle.tireRadiusM * motionSign;
        const auto wheelAcceleration = (wheelTorque - lastTireForce * vehicle.tireRadiusM - brakeTorque)
            / wheelInertia;
        wheelAngularVelocityRadPerSecond_ += wheelAcceleration * mechanicalDt;
        if (brakePressure > 0.0 && previousWheelOmega * wheelAngularVelocityRadPerSecond_ < 0.0)
            wheelAngularVelocityRadPerSecond_ = 0.0;

        const auto speedSquared = previousVehicleSpeed * previousVehicleSpeed;
        const auto speedSign = std::abs(previousVehicleSpeed) > 0.01
            ? std::copysign(1.0, previousVehicleSpeed) : 0.0;
        const auto aeroForce = 0.5 * 1.225 * vehicle.dragCoefficient * vehicle.frontalAreaM2
            * speedSquared * speedSign;
        const auto rollingForce = vehicle.massKg * 9.80665 * vehicle.rollingResistanceCoefficient * speedSign;
        vehicleSpeedMps_ += (lastTireForce - aeroForce - rollingForce) / vehicle.massKg * mechanicalDt;
        if (brakePressure > 0.0 && previousVehicleSpeed * vehicleSpeedMps_ < 0.0) vehicleSpeedMps_ = 0.0;
        vehicleDistanceM_ += std::abs(0.5 * (previousVehicleSpeed + vehicleSpeedMps_)) * mechanicalDt;

        const auto meanWheelOmega = 0.5 * (previousWheelOmega + wheelAngularVelocityRadPerSecond_);
        const auto meanVehicleSpeed = 0.5 * (previousVehicleSpeed + vehicleSpeedMps_);
        const auto clutchLossW = std::abs(clutchTorque * slipOmega);
        const auto coolingW = std::max(0.0, clutchTemperatureC_ - config_.ambientTemperatureC)
            * transmission.clutchCoolingWPerC;
        clutchTemperatureC_ += (clutchLossW - coolingW)
            / transmission.clutchThermalCapacityJPerC * mechanicalDt;
        clutchTemperatureC_ = std::max(config_.ambientTemperatureC, clutchTemperatureC_);
        clutchTorqueIntegral += clutchTorque * mechanicalDt;
        wheelTorqueIntegral += wheelTorque * mechanicalDt;
        clutchLossEnergy += clutchLossW * mechanicalDt;
        wheelInputWork += wheelTorque * meanWheelOmega * mechanicalDt;
        roadWork += (std::abs(aeroForce) + std::abs(rollingForce)) * std::abs(meanVehicleSpeed) * mechanicalDt;
        brakeWork += brakeForceCapacity * vehicle.tireRadiusM * std::abs(meanWheelOmega) * mechanicalDt;
        tireSlipWork += std::abs(lastTireForce * slipVelocity) * mechanicalDt;
    }
    clutchDissipatedEnergyJoules_ += clutchLossEnergy;
    const auto inverseDt = dt > 0.0 ? 1.0 / dt : 0.0;
    output.clutchTorqueNm = clutchTorqueIntegral * inverseDt;
    output.engineReactionTorqueNm = -output.clutchTorqueNm;
    output.wheelTorqueNm = wheelTorqueIntegral * inverseDt;
    output.clutchSlipRpm = lastSlipRpm;
    output.tireForceN = lastTireForce;
    output.tractionLimited = tractionLimited;
    const auto storedEnergy = 0.5 * wheelInertia * wheelAngularVelocityRadPerSecond_
        * wheelAngularVelocityRadPerSecond_ + 0.5 * vehicle.massKg * vehicleSpeedMps_ * vehicleSpeedMps_;
    const auto energyResidual = energyInitialised_
        ? storedEnergy - previousStoredEnergyJoules_ - wheelInputWork + roadWork + brakeWork + tireSlipWork : 0.0;
    previousStoredEnergyJoules_ = storedEnergy;
    energyInitialised_ = true;

    output.requestedLoad = std::clamp(requestedLoad, 0.0, 1.0);
    output.engagedGear = engagedGear_;
    output.clutchPressure = effectiveClutch;
    output.vehicleSpeedMps = vehicleSpeedMps_;
    output.vehicleDistanceM = vehicleDistanceM_;
    output.shiftProgress = shiftProgress;
    output.shiftInProgress = shiftInProgress_;
    output.brakePressure = brakePressure;
    output.brakeForceN = brakeForceCapacity;
    output.clutchTemperatureC = clutchTemperatureC_;
    output.clutchDissipatedEnergyJoules = clutchDissipatedEnergyJoules_;
    output.clutchPowerLossKw = clutchLossEnergy * inverseDt * 0.001;
    output.storedEnergyJoules = storedEnergy;
    output.energyResidualJoules = energyResidual;
    return output;
}

} // namespace enginelab
