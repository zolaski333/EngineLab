#include <enginelab/runtime/DrivelineModel.hpp>
#include <algorithm>
#include <cmath>
#include <numbers>

namespace enginelab {

DrivelineModel::DrivelineModel(const EngineConfig& config) : config_(config) {
    clutchTemperatureC_ = config_.ambientTemperatureC;
    engineInertiaKgM2_ = std::max(0.001, effectiveRotatingInertiaKgM2(config_));
}

void DrivelineModel::requestGear(int gear) noexcept {
    const auto maximum = static_cast<int>(config_.transmission.gearRatios.size()) - 1;
    const auto selected = std::clamp(gear, -2, maximum);
    // A real selector cannot engage a ratio that would instantly reverse a
    // moving driveline. Neutral remains available so the vehicle can coast to
    // the low-speed interlock before reverse/forward is requested again.
    constexpr double directionInterlockMps = 0.5;
    if ((selected == -2 && vehicleSpeedMps_ > directionInterlockMps)
        || (selected >= 0 && vehicleSpeedMps_ < -directionInterlockMps))
        return;
    requestedGear_ = selected;
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
    dt = std::isfinite(dt) ? std::clamp(dt, 0.0, 0.05) : 0.0;
    requestedLoad = std::isfinite(requestedLoad) ? std::clamp(requestedLoad, 0.0, 1.0) : 0.0;
    clutchPedal = std::isfinite(clutchPedal) ? std::clamp(clutchPedal, 0.0, 1.0) : 0.0;
    brakePressure = std::isfinite(brakePressure) ? std::clamp(brakePressure, 0.0, 1.0) : 0.0;
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
        shiftFromGear_ = engagedGear_;
        shiftElapsedSeconds_ = 0.0;
        resyncReferenceSlipRpm_ = 0.0;
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
        const auto forwardUpshift = shiftFromGear_ >= 0
            && shiftTargetGear_ > shiftFromGear_;
        const auto lockBandRpm = std::max(
            1.0, transmission.clutchLockSpeedRpm);
        // The new ratio becomes active just before halfway through the timer.
        // On the following frame lastClutchSlipRpm_ is therefore the first
        // measured slip in that ratio and is a stable reference for the
        // synchronisation phase.
        if (forwardUpshift && shiftProgress >= 0.52
            && resyncReferenceSlipRpm_ <= lockBandRpm
            && std::abs(lastClutchSlipRpm_) > lockBandRpm) {
            resyncReferenceSlipRpm_ =
                std::abs(lastClutchSlipRpm_);
        }

        const auto cutDepth = std::clamp(
            transmission.shiftTorqueCutFraction, 0.0, 1.0);
        const auto cutFloor = 1.0 - cutDepth;
        if (forwardUpshift && shiftProgress > 0.50
            && resyncReferenceSlipRpm_ > lockBandRpm) {
            // Do not restore engine torque merely because a wall-clock shift
            // timer is ending. Release it with the actual clutch slip instead:
            // this prevents rising combustion torque and clutch synchronising
            // torque from fighting across the same few frames, the physical
            // source of the boosted-engine "gear crack".
            const auto remainingSlip = std::clamp(
                (std::abs(lastClutchSlipRpm_) - lockBandRpm)
                    / std::max(1.0,
                        resyncReferenceSlipRpm_ - lockBandRpm),
                0.0, 1.0);
            const auto release = 1.0 - remainingSlip;
            const auto smoothRelease =
                release * release * (3.0 - 2.0 * release);
            output.torqueCutMultiplier =
                cutFloor + cutDepth * smoothRelease;
        } else {
            output.torqueCutMultiplier = 1.0 - cutDepth
                * std::sin(std::numbers::pi
                    * std::clamp(shiftProgress, 0.0, 1.0));
        }
        if (shiftProgress >= 1.0) {
            engagedGear_ = shiftTargetGear_;
            shiftInProgress_ = false;
            shiftProgress = 0.0;
        }
    } else if (resyncReferenceSlipRpm_
        > std::max(1.0, transmission.clutchLockSpeedRpm)) {
        // A high-power upshift can still be synchronising after the selector
        // timer has completed. Keep the same slip-following torque envelope
        // until the dry clutch enters its lock band, then return exactly to
        // the driver's requested torque.
        const auto lockBandRpm = std::max(
            1.0, transmission.clutchLockSpeedRpm);
        const auto currentSlip = std::abs(lastClutchSlipRpm_);
        if (currentSlip <= lockBandRpm) {
            resyncReferenceSlipRpm_ = 0.0;
            output.torqueCutMultiplier = 1.0;
        } else {
            const auto remainingSlip = std::clamp(
                (currentSlip - lockBandRpm)
                    / std::max(1.0,
                        resyncReferenceSlipRpm_ - lockBandRpm),
                0.0, 1.0);
            const auto release = 1.0 - remainingSlip;
            const auto smoothRelease =
                release * release * (3.0 - 2.0 * release);
            const auto cutDepth = std::clamp(
                transmission.shiftTorqueCutFraction, 0.0, 1.0);
            output.torqueCutMultiplier =
                1.0 - cutDepth + cutDepth * smoothRelease;
        }
    }

    const auto totalRatio = selectedRatio();
    const auto engineOmega = engineState.angularVelocityRadPerSecond;
    // The crank's own torque this tick (combustion brake torque plus any starter),
    // used by the locked-clutch constraint. It is a one-tick estimate of a quantity
    // that changes slowly next to the tick, and the friction capacity bounds it.
    // The driveline advances once per public 240 Hz frame while chamber torque
    // is resolved at many crank-angle substeps. Feeding the last instantaneous
    // pressure torque into a one-frame-ahead rigid-clutch constraint aliases a
    // firing pulse into an equal-and-opposite reaction on the next frame. Use
    // the completed-cycle work for the mean constraint load; torsional ripple
    // remains in EngineSimulator, but cannot be phase-inverted by the slower
    // vehicle coupling.
    const auto engineDriveTorque =
        (engineState.simulationTimeSeconds > 0.0 && engineState.rpm > 220.0
            ? engineState.cycleAveragedTorqueNm
            : engineState.torqueNm)
        + engineState.starterTorqueNm;
    const auto wheelInertia = std::max(0.01, transmission.drivenWheelInertiaKgM2
        + transmission.differentialInertiaKgM2
        + transmission.gearboxInputInertiaKgM2 * totalRatio * totalRatio);
    constexpr double gravityMps2 = 9.80665;
    const auto totalNormalForce = vehicle.massKg * gravityMps2;
    const auto drivenAxleNormalForce = [&](double accelerationMps2) noexcept {
        if (vehicle.drivenAxleLayout == DrivenAxleLayout::all)
            return totalNormalForce;
        const auto staticNormalForce =
            totalNormalForce * vehicle.drivenAxleWeightFraction;
        // Quasi-static pitch equilibrium: m*a*h/L moves load rearward under
        // positive acceleration. It therefore helps a rear driven axle and
        // unloads a front driven axle; reverse acceleration swaps the effect.
        const auto longitudinalTransfer = vehicle.massKg * accelerationMps2
            * vehicle.centerOfGravityHeightM / vehicle.wheelbaseM;
        const auto dynamicNormalForce =
            vehicle.drivenAxleLayout == DrivenAxleLayout::rear
            ? staticNormalForce + longitudinalTransfer
            : staticNormalForce - longitudinalTransfer;
        return std::clamp(dynamicNormalForce, 0.0, totalNormalForce);
    };
    const auto brakeForceCapacity = vehicle.maximumBrakeForceN * brakePressure;
    // The tire relaxation time is much shorter than the public simulation step.
    // Integrating that stiff coupling at 1 ms keeps the result stable at every time scale
    // without weakening the physical tire stiffness or hiding instability behind clamps.
    const auto mechanicalStepCount = std::max(1, static_cast<int>(std::ceil(dt / 0.001)));
    const auto mechanicalDt = mechanicalStepCount > 0 ? dt / mechanicalStepCount : 0.0;
    double clutchTorqueIntegral = 0.0;
    double engineCouplingTorqueIntegral = 0.0;
    double reflectedInertiaTimeIntegral = 0.0;
    double wheelTorqueIntegral = 0.0;
    double clutchLossEnergy = 0.0;
    double wheelInputWork = 0.0;
    double roadWork = 0.0;
    double brakeWork = 0.0;
    double roadLoadWork = 0.0;
    double tireSlipWork = 0.0;
    double drivenAxleNormalImpulse = 0.0;
    double longitudinalAccelerationImpulse = 0.0;
    std::size_t clutchStickingSteps = 0;
    double lastSlipRpm = 0.0;
    double lastTireForce = 0.0;
    double lastRoadLoadForce = 0.0;
    // The engine itself is integrated by EngineSimulator after this call, from
    // the frame-mean clutch reaction returned below. The mechanical loop still
    // needs a local prediction of that same response: keeping engineOmega
    // frozen while the wheel advances through four or five 1 ms substeps makes
    // a locked clutch overshoot synchronism, reverse its full kinetic-friction
    // torque on the next substep, and repeat. At high ratio this appeared as
    // +/- clutch capacity and 200-300 rpm crank jumps every public 240 Hz tick.
    //
    // This predictor does not become a second crank state and is never
    // published. It integrates the same estimated drive torque and clutch
    // reaction solely for the next local slip evaluation; EngineSimulator
    // remains authoritative and receives the time-averaged reaction exactly as
    // before.
    auto predictedEngineOmega = engineOmega;
    // Counted, not latched. This used to be a sticky OR across the mechanical
    // sub-steps, so a single clipped sub-step out of five lit the indicator for
    // the whole frame -- and with a slip-velocity spring this stiff, one clipped
    // sub-step happens on every gearshift and every kerb-strength transient. The
    // panel then showed a permanent "tyre limited" on an engine cruising far
    // below the limit. Report saturation only when the tyre actually spent most
    // of the frame on its friction limit. Display-only: no physics reads this.
    std::size_t tractionLimitedSteps = 0;
    for (int step = 0; step < mechanicalStepCount; ++step) {
        const auto previousWheelOmega = wheelAngularVelocityRadPerSecond_;
        const auto previousVehicleSpeed = vehicleSpeedMps_;
        const auto gearboxOmega = previousWheelOmega * totalRatio;
        const auto slipOmega = predictedEngineOmega - gearboxOmega;
        lastSlipRpm = slipOmega * 60.0 / (2.0 * std::numbers::pi);
        const auto fade = std::clamp((transmission.clutchFailureTemperatureC - clutchTemperatureC_)
            / (transmission.clutchFailureTemperatureC - transmission.clutchFadeStartTemperatureC), 0.0, 1.0);
        const auto capacity = transmission.maxClutchTorqueNm * effectiveClutch * fade;

        const auto speedSquared = previousVehicleSpeed * previousVehicleSpeed;
        const auto speedSign = std::abs(previousVehicleSpeed) > 0.01
            ? std::copysign(1.0, previousVehicleSpeed) : 0.0;
        const auto aeroForce = 0.5 * 1.225 * vehicle.dragCoefficient
            * vehicle.frontalAreaM2 * speedSquared * speedSign;
        const auto rollingForce = vehicle.massKg * 9.80665
            * vehicle.rollingResistanceCoefficient * speedSign;
        const auto motionSign = std::abs(previousWheelOmega) > 0.01
            ? std::copysign(1.0, previousWheelOmega)
            : (std::abs(previousVehicleSpeed) > 0.01
                ? std::copysign(1.0, previousVehicleSpeed) : 0.0);
        const auto brakeTorque = brakeForceCapacity
            * vehicle.tireRadiusM * motionSign;

        // Driven-tyre longitudinal force and wheel-brake torque are resolved
        // before the clutch, because the lock solver needs the road load it
        // must react. With the optional grip model OFF, rolling contact is a
        // rigid no-wheelspin constraint: vehicle translation is reflected as
        // m*r^2 at the wheel instead of approximated by an unbounded stiff
        // slip spring. This is both the documented rolling-road behaviour and
        // the stable interpretation of "no adhesion constraint".
        const auto tireSurfaceSpeed = previousWheelOmega * vehicle.tireRadiusM;
        const auto slipVelocity = tireSurfaceSpeed - previousVehicleSpeed;
        // The preceding 1 ms mechanical sub-step supplies acceleration to the
        // quasi-static pitch equilibrium. This is the same causal update used
        // by the tyre relaxation state and avoids an algebraic traction loop.
        const auto normalForce =
            drivenAxleNormalForce(longitudinalAccelerationMps2_);
        auto wheelInertiaForConstraint = wheelInertia;
        auto wheelLoadTorque = 0.0;
        auto roadLoadForce = 0.0;
        if (vehicle.tyreGripLimitEnabled) {
            const auto tractionLimit =
                vehicle.tireFrictionCoefficient * normalForce;
            const auto tireStiffnessNPerMps = normalForce * 7.5;
            const auto unconstrainedTireForce =
                slipVelocity * tireStiffnessNPerMps;
            lastTireForce = std::clamp(
                unconstrainedTireForce, -tractionLimit, tractionLimit);
            if (std::abs(unconstrainedTireForce) > tractionLimit + 1.0e-6)
                ++tractionLimitedSteps;
            wheelLoadTorque = lastTireForce * vehicle.tireRadiusM
                + brakeTorque;
        } else {
            constexpr double roadLoadDirectionThresholdMps = 1.0e-6;
            const auto vehicleIsMoving = std::abs(previousVehicleSpeed)
                > roadLoadDirectionThresholdMps;
            const auto loadDirection = vehicleIsMoving
                ? std::copysign(1.0, previousVehicleSpeed)
                : (std::abs(totalRatio) > 1.0e-9
                    ? std::copysign(1.0, totalRatio) : 0.0);
            roadLoadForce = requestedLoad * vehicle.maximumBrakeForceN
                * loadDirection;
            lastRoadLoadForce = roadLoadForce;
            wheelInertiaForConstraint += vehicle.massKg
                * vehicle.tireRadiusM * vehicle.tireRadiusM;
            wheelLoadTorque = (aeroForce + rollingForce + roadLoadForce)
                * vehicle.tireRadiusM + brakeTorque;
        }

        // Dry-friction clutch with a Karnopp stick/slip law.
        //  * |slip| outside the lock window -> kinetic friction: it transmits its full
        //    capacity opposing the slip, independent of slip magnitude. This launches
        //    the car and is what dissipates energy and heats the disc.
        //  * inside the lock window -> stick: crank and gearbox input are one rigid
        //    body. We solve their shared acceleration from both inertias, the engine's
        //    own torque and the reflected road load, then read off the coupling torque
        //    that holds synchronism. Being the constraint solution rather than a stiff
        //    slope, this reaction cannot overshoot within a tick, so an engaged clutch
        //    holds the engine's torque at a few rpm of residual slip instead of
        //    slipping without end. The friction capacity still bounds it: when the
        //    demanded stick torque exceeds capacity the clutch breaks away into slip.
        double clutchTorque = 0.0;
        auto clutchSticking = false;
        if (totalRatio != 0.0 && effectiveClutch > 0.0) {
            const auto lockBandRpm = std::max(1.0, transmission.clutchLockSpeedRpm);
            if (std::abs(lastSlipRpm) > lockBandRpm) {
                clutchTorque = std::copysign(capacity, slipOmega);
            } else {
                const auto coupledInertia = wheelInertiaForConstraint
                    + engineInertiaKgM2_ * totalRatio * totalRatio * transmission.drivelineEfficiency;
                const auto stickTorque = coupledInertia > 0.0
                    ? (wheelInertiaForConstraint * engineDriveTorque
                       + engineInertiaKgM2_ * totalRatio * wheelLoadTorque) / coupledInertia
                    : 0.0;
                clutchTorque = std::clamp(stickTorque, -capacity, capacity);
                clutchSticking = std::abs(stickTorque) <= capacity + 1.0e-9;
            }
        }
        auto engineCouplingTorque = -clutchTorque;
        auto reflectedInertiaKgM2 = 0.0;
        if (clutchSticking && std::abs(totalRatio) > 1.0e-9) {
            ++clutchStickingSteps;
            // Rigid-shaft equivalent referred to the crank. The clutch torque
            // itself contains the torque needed to accelerate wheel/gearbox
            // inertia; applying it as an external load one 240 Hz frame later
            // phase-inverts combustion ripple. Carry that inertia explicitly
            // and reflect only the road-side load torque instead.
            const auto ratioEfficiency = totalRatio
                * transmission.drivelineEfficiency;
            reflectedInertiaKgM2 = wheelInertiaForConstraint
                / (totalRatio * ratioEfficiency);
            engineCouplingTorque = -wheelLoadTorque / ratioEfficiency;
        }
        const auto wheelTorque = clutchTorque * totalRatio * transmission.drivelineEfficiency;
        const auto wheelAcceleration = (wheelTorque - wheelLoadTorque)
            / wheelInertiaForConstraint;
        wheelAngularVelocityRadPerSecond_ += wheelAcceleration * mechanicalDt;
        if (totalRatio != 0.0 && effectiveClutch > 0.0) {
            predictedEngineOmega = std::max(0.0,
                predictedEngineOmega
                    + (engineDriveTorque + engineCouplingTorque)
                        / (engineInertiaKgM2_ + reflectedInertiaKgM2)
                        * mechanicalDt);
        }
        if (brakePressure > 0.0 && previousWheelOmega * wheelAngularVelocityRadPerSecond_ < 0.0)
            wheelAngularVelocityRadPerSecond_ = 0.0;

        if (vehicle.tyreGripLimitEnabled) {
            // The manual vehicle load is an external longitudinal retarder. Its
            // reaction reaches the crank only through tyre, gearbox and clutch.
            const auto unretardedForce =
                lastTireForce - aeroForce - rollingForce;
            constexpr double roadLoadDirectionThresholdMps = 1.0e-6;
            const auto vehicleIsMoving = std::abs(previousVehicleSpeed)
                > roadLoadDirectionThresholdMps;
            const auto loadDirection = vehicleIsMoving
                ? std::copysign(1.0, previousVehicleSpeed)
                : (std::abs(unretardedForce) > 1.0e-6
                    ? std::copysign(1.0, unretardedForce) : 0.0);
            const auto roadLoadCapacity =
                requestedLoad * vehicle.maximumBrakeForceN;
            const auto roadLoadMagnitude = vehicleIsMoving
                ? roadLoadCapacity
                : std::min(roadLoadCapacity, std::abs(unretardedForce));
            roadLoadForce = roadLoadMagnitude * loadDirection;
            lastRoadLoadForce = roadLoadForce;
            longitudinalAccelerationMps2_ =
                (lastTireForce - aeroForce - rollingForce - roadLoadForce)
                / vehicle.massKg;
            vehicleSpeedMps_ +=
                longitudinalAccelerationMps2_ * mechanicalDt;
            if (loadDirection != 0.0
                    && vehicleSpeedMps_ * loadDirection < 0.0)
                vehicleSpeedMps_ = 0.0;
        } else {
            vehicleSpeedMps_ = wheelAngularVelocityRadPerSecond_
                * vehicle.tireRadiusM;
            longitudinalAccelerationMps2_ =
                (vehicleSpeedMps_ - previousVehicleSpeed) / mechanicalDt;
            // Contact force telemetry includes the force accelerating vehicle
            // mass plus the external longitudinal resistances.
            lastTireForce = vehicle.massKg * longitudinalAccelerationMps2_
                + aeroForce + rollingForce + roadLoadForce;
        }
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
        engineCouplingTorqueIntegral +=
            engineCouplingTorque * mechanicalDt;
        reflectedInertiaTimeIntegral +=
            reflectedInertiaKgM2 * mechanicalDt;
        wheelTorqueIntegral += wheelTorque * mechanicalDt;
        clutchLossEnergy += clutchLossW * mechanicalDt;
        wheelInputWork += wheelTorque * meanWheelOmega * mechanicalDt;
        roadWork += (std::abs(aeroForce) + std::abs(rollingForce)) * std::abs(meanVehicleSpeed) * mechanicalDt;
        roadLoadWork += std::abs(roadLoadForce * meanVehicleSpeed) * mechanicalDt;
        brakeWork += brakeForceCapacity * vehicle.tireRadiusM * std::abs(meanWheelOmega) * mechanicalDt;
        if (vehicle.tyreGripLimitEnabled)
            tireSlipWork += std::abs(lastTireForce * slipVelocity)
                * mechanicalDt;
        drivenAxleNormalImpulse += normalForce * mechanicalDt;
        longitudinalAccelerationImpulse +=
            longitudinalAccelerationMps2_ * mechanicalDt;
    }
    clutchDissipatedEnergyJoules_ += clutchLossEnergy;
    const auto inverseDt = dt > 0.0 ? 1.0 / dt : 0.0;
    output.clutchTorqueNm = clutchTorqueIntegral * inverseDt;
    output.engineReactionTorqueNm = -output.clutchTorqueNm;
    output.engineCouplingTorqueNm =
        engineCouplingTorqueIntegral * inverseDt;
    output.reflectedRotatingInertiaKgM2 =
        reflectedInertiaTimeIntegral * inverseDt;
    output.wheelTorqueNm = wheelTorqueIntegral * inverseDt;
    output.clutchSlipRpm = lastSlipRpm;
    output.clutchStickFraction = mechanicalStepCount > 0
        ? static_cast<double>(clutchStickingSteps)
            / static_cast<double>(mechanicalStepCount)
        : 0.0;
    output.tireForceN = lastTireForce;
    output.drivenAxleNormalForceN =
        drivenAxleNormalImpulse * inverseDt;
    output.longitudinalAccelerationMps2 =
        longitudinalAccelerationImpulse * inverseDt;
    output.tractionLimited = mechanicalStepCount > 0
        && tractionLimitedSteps * 2 > static_cast<std::size_t>(mechanicalStepCount);
    const auto storedEnergy = 0.5 * wheelInertia * wheelAngularVelocityRadPerSecond_
        * wheelAngularVelocityRadPerSecond_ + 0.5 * vehicle.massKg * vehicleSpeedMps_ * vehicleSpeedMps_;
    const auto energyResidual = energyInitialised_
        ? storedEnergy - previousStoredEnergyJoules_ - wheelInputWork + roadWork + roadLoadWork
            + brakeWork + tireSlipWork : 0.0;
    previousStoredEnergyJoules_ = storedEnergy;
    energyInitialised_ = true;

    output.requestedLoad = requestedLoad;
    output.engagedGear = engagedGear_;
    output.clutchPressure = effectiveClutch;
    output.vehicleSpeedMps = vehicleSpeedMps_;
    output.vehicleDistanceM = vehicleDistanceM_;
    output.shiftProgress = shiftProgress;
    output.shiftInProgress = shiftInProgress_;
    output.brakePressure = brakePressure;
    output.brakeForceN = brakeForceCapacity;
    output.roadLoadForceN = std::abs(lastRoadLoadForce);
    output.clutchTemperatureC = clutchTemperatureC_;
    output.clutchDissipatedEnergyJoules = clutchDissipatedEnergyJoules_;
    output.clutchPowerLossKw = clutchLossEnergy * inverseDt * 0.001;
    output.storedEnergyJoules = storedEnergy;
    output.energyResidualJoules = energyResidual;
    lastClutchSlipRpm_ = lastSlipRpm;
    return output;
}

} // namespace enginelab
