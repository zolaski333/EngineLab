#include <enginelab/foundation/EngineTypes.hpp>
#include <enginelab/physics/ConservativeGasSystem.hpp>
#include <enginelab/physics/MechanicalKinematics.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {
void require(bool condition, const char* message) {
    if (condition) return;
    std::cerr << "FAILED: " << message << '\n';
    std::exit(EXIT_FAILURE);
}

double threeCellEnergy(const enginelab::GasCell& first,
                       const enginelab::GasCell& middle,
                       const enginelab::GasCell& last) {
    return first.internalEnergyJoules() + middle.internalEnergyJoules()
        + last.internalEnergyJoules() + first.bulkKineticEnergyJoules()
        + middle.bulkKineticEnergyJoules() + last.bulkKineticEnergyJoules();
}
}

int main() {
    {
        auto explicitTopology = enginelab::makeDefaultInlineFour();
        explicitTopology.crankshafts.front().inheritsLegacyInertia = false;
        explicitTopology.crankshafts.front().momentOfInertiaKgM2 = 0.73;
        explicitTopology.rotatingInertiaKgM2 = 0.11;
        explicitTopology.intakePaths.front().inheritsGlobalGeometry = false;
        explicitTopology.intakePaths.front().geometry.runnerDiameterMm = 51.0;
        explicitTopology.intake.runnerDiameterMm = 33.0;
        enginelab::normaliseEngineConfig(explicitTopology);
        require(std::abs(explicitTopology.crankshafts.front().momentOfInertiaKgM2 - 0.73) < 1.0e-12,
                "normalisation must retain explicit crankshaft inertia");
        require(std::abs(explicitTopology.intakePaths.front().geometry.runnerDiameterMm - 51.0) < 1.0e-12,
                "normalisation must retain explicit intake-path geometry");

        auto missingJournal = explicitTopology;
        missingJournal.cylinders.front().crankJournalId = 0;
        require(enginelab::validateEngineConfig(missingJournal).has_value(),
                "an explicit mechanical topology must reject cylinders without a journal reference");

        auto mismatchedBank = explicitTopology;
        mismatchedBank.cylinders.front().bankId = 99;
        require(enginelab::validateEngineConfig(mismatchedBank).has_value(),
                "cylinder bank IDs must agree with bank membership");
    }

    {
        const auto v8 = enginelab::makeDefaultV8();
        require(!enginelab::validateEngineConfig(v8), "the corrected V8 preset must validate");
        const auto reference = enginelab::buildEngineKinematicsReference(v8);
        for (std::size_t index = 0; index < v8.cylinders.size(); ++index) {
            const auto atConfiguredTdc = enginelab::evaluateCylinderKinematics(v8, reference, index,
                v8.cylinders[index].crankOffsetDegrees, 0.0);
            require(atConfiguredTdc.pistonTravelMm < 1.0e-5,
                    "V8 crank journals must align configured firing phase and mechanical TDC");
        }
        const auto flatSix = enginelab::makeDefaultFlatSix();
        require(!enginelab::validateEngineConfig(flatSix), "the corrected flat-six preset must validate");
        require(std::abs(flatSix.intakePaths.front().geometry.plenumVolumeLitres - 4.2) < 1.0e-12
                && std::abs(flatSix.exhaustPaths.front().geometry.primaryLengthMm - 720.0) < 1.0e-12,
                "flat-six canonical and per-path geometry must agree");
    }

    {
        enginelab::GasCell cell;
        cell.initialise(101.325, 1.0, 300.0);
        cell.setBulkVelocityMps(80.0, 0.0);
        const auto forward = cell.dynamicPressureKpa(1.0, 0.0);
        const auto reverse = cell.dynamicPressureKpa(-1.0, 0.0);
        require(forward > 0.0 && reverse < 0.0
                && std::abs(forward + reverse) < std::abs(forward) * 1.0e-12,
                "dynamic pressure must preserve the projected velocity sign");
    }

    {
        enginelab::GasCell upstream;
        enginelab::GasCell downstream;
        upstream.initialise(101.325, 1.0, 300.0);
        downstream.initialise(101.325, 1.0, 300.0);
        upstream.setBulkVelocityMps(80.0, 0.0);
        downstream.setBulkVelocityMps(80.0, 0.0);
        const auto transfer = enginelab::ConservativeGasSystem::flow({
            &upstream, &downstream, 8.0e-4, 0.72, 0.01,
            1.0, 0.0, 8.0e-4, 8.0e-4 });
        require(std::abs(transfer.transferredMassKg) < 1.0e-15,
                "equal-pressure co-flowing cells must not create a fictitious dynamic-pressure gradient");
    }

    {
        enginelab::GasCell smoothPipe;
        smoothPipe.initialise(130.0, 0.75, 500.0);
        smoothPipe.setGeometry(1.2e-3, 1.0, 0.0);
        smoothPipe.setBulkVelocityMps(110.0, 0.0);
        auto roughPipe = smoothPipe;
        const auto totalEnergyBefore = smoothPipe.internalEnergyJoules()
            + smoothPipe.bulkKineticEnergyJoules();
        smoothPipe.applyFlowResistance(0.8, 0.04, 1.5e-6, 0.0, 0.01);
        roughPipe.applyFlowResistance(0.8, 0.04, 4.5e-5, 0.0, 0.01);
        const auto totalEnergyAfter = smoothPipe.internalEnergyJoules()
            + smoothPipe.bulkKineticEnergyJoules();
        require(smoothPipe.bulkVelocityMps() < 110.0
                && roughPipe.bulkVelocityMps() < smoothPipe.bulkVelocityMps(),
                "Darcy-Weisbach losses must oppose flow and respond to wall roughness");
        require(std::abs(totalEnergyAfter - totalEnergyBefore)
                    < std::max(1.0, totalEnergyBefore) * 1.0e-12,
                "resolved pipe friction must convert kinetic energy to heat conservatively");
    }

    {
        enginelab::GasCell cell;
        cell.initialise(101.325, 1.0, 300.0);
        constexpr double targetTemperatureK = 12'000.0;
        const auto heatCapacity = cell.totalMoles() * cell.molarHeatCapacityCvEffective();
        cell.addHeatJoules(heatCapacity * (targetTemperatureK - cell.temperatureK()));
        const auto temperature = cell.temperatureK();
        const auto pressureFromState = cell.totalMoles() * enginelab::GasCell::universalGasConstant
            * temperature / cell.volumeM3() / 1'000.0;
        require(std::abs(temperature - targetTemperatureK) < targetTemperatureK * 1.0e-12,
                "high-temperature sensible energy must not disappear behind a display cap");
        require(std::abs(cell.internalEnergyJoules() - heatCapacity * temperature)
                    < cell.internalEnergyJoules() * 1.0e-12
                && std::abs(cell.pressureKpa() - pressureFromState) < pressureFromState * 1.0e-12,
                "temperature, pressure and conserved internal energy must remain thermodynamically consistent");
    }

    {
        enginelab::GasCell cell;
        cell.initialise(101.325, 0.62, 340.0);
        const auto gamma = cell.heatCapacityRatioEffective();
        const auto invariantBefore = cell.pressureKpa()
            * std::pow(cell.volumeM3(), gamma);
        cell.setVolumeAdiabatic(cell.volumeLitres() * 0.37);
        const auto invariantAfter = cell.pressureKpa()
            * std::pow(cell.volumeM3(), gamma);
        require(std::abs(invariantAfter - invariantBefore)
                    < std::max(1.0, std::abs(invariantBefore)) * 1.0e-11,
                "finite adiabatic volume changes must preserve P*V^gamma");

        cell.addHeatJoules(-cell.internalEnergyJoules());
        require(cell.temperatureK() == 0.0 && cell.pressureKpa() == 0.0,
                "a zero-energy gas cell must not create a hidden temperature or pressure floor");
    }

    {
        enginelab::GasCell lowPressure;
        enginelab::GasCell highPressure;
        lowPressure.initialise(90.0, 0.25, 300.0);
        highPressure.initialise(150.0, 0.25, 300.0);
        lowPressure.setGeometry(2.0e-4, 1.0, 0.0);
        highPressure.setGeometry(2.0e-4, 1.0, 0.0);
        const auto reverse = enginelab::ConservativeGasSystem::flow(
            lowPressure, highPressure, 1.0e-4, 0.75, 2.0e-4);
        require(reverse.transferredMassKg < 0.0
                && lowPressure.momentumXKgMps() < 0.0
                && highPressure.momentumXKgMps() < 0.0,
                "reverse flow must accelerate both adjacent gas volumes from high to low pressure");
    }

    {
        enginelab::GasCell staticUpstream;
        enginelab::GasCell staticDownstream;
        staticUpstream.initialise(110.0, 0.40, 300.0);
        staticDownstream.initialise(100.0, 0.40, 300.0);
        staticUpstream.setGeometry(2.0e-4, 1.0, 0.0);
        staticDownstream.setGeometry(2.0e-4, 1.0, 0.0);
        staticUpstream.setBulkVelocityMps(-220.0, 0.0);
        auto dynamicUpstream = staticUpstream;
        auto dynamicDownstream = staticDownstream;
        const auto staticResult = enginelab::ConservativeGasSystem::flow(
            staticUpstream, staticDownstream, 8.0e-5, 0.72, 1.0e-4);
        const auto dynamicResult = enginelab::ConservativeGasSystem::flow({
            &dynamicUpstream, &dynamicDownstream, 8.0e-5, 0.72, 1.0e-4,
            1.0, 0.0, 2.0e-4, 2.0e-4 });
        require(staticResult.transferredMassKg > 0.0
                && dynamicResult.transferredMassKg < 0.0,
                "static flow must follow static pressure while the full overload follows effective pressure");
    }

    {
        enginelab::GasCell intakePlenum;
        intakePlenum.initialise(85.0, 1.0, 300.0);
        intakePlenum.setGeometry(2.0e-4, 0.0, 1.0);
        const auto intake = enginelab::ConservativeGasSystem::flowFromBoundary(
            intakePlenum, 120.0, 300.0, 1.0e-4, 0.75, 2.0e-4,
            0.0, -1.0); // target -> upstream boundary
        require(intake.transferredMassKg < 0.0 && intakePlenum.momentumYKgMps() > 0.0,
            "an upstream boundary must inject intake momentum toward the runners");

        enginelab::GasCell exhaustCollector;
        exhaustCollector.initialise(145.0, 1.0, 650.0);
        exhaustCollector.setGeometry(2.0e-4, 1.0, 0.0);
        const auto exhaust = enginelab::ConservativeGasSystem::flowFromBoundary(
            exhaustCollector, 101.325, 300.0, 1.0e-4, 0.75, 2.0e-4,
            1.0, 0.0); // target -> downstream boundary
        require(exhaust.transferredMassKg > 0.0 && exhaustCollector.momentumXKgMps() > 0.0,
            "a downstream boundary must accelerate collector gas toward the outlet");
    }

    {
        enginelab::GasCell intake;
        enginelab::GasCell cylinder;
        enginelab::GasCell exhaust;
        intake.initialise(145.0, 0.40, 320.0);
        cylinder.initialise(108.0, 0.060, 500.0);
        exhaust.initialise(78.0, 0.45, 580.0);
        const auto molesBefore = intake.totalMoles() + cylinder.totalMoles() + exhaust.totalMoles();
        const auto energyBefore = threeCellEnergy(intake, cylinder, exhaust);
        const auto flow = enginelab::ConservativeGasSystem::flowSimultaneous(
            { &intake, &cylinder, 1.2e-4, 0.72, 1.0e-4, 0.0, 1.0, 2.0e-4, 8.0e-4 },
            { &cylinder, &exhaust, 1.0e-4, 0.68, 1.0e-4, 0.0, -1.0, 8.0e-4, 2.0e-4 });
        const auto molesAfter = intake.totalMoles() + cylinder.totalMoles() + exhaust.totalMoles();
        const auto energyAfter = threeCellEnergy(intake, cylinder, exhaust);
        require(flow.first.transferredMassKg > 0.0 && flow.second.transferredMassKg > 0.0,
                "simultaneous overlap must resolve both pressure gradients");
        require(std::abs(molesAfter - molesBefore) < 1.0e-12
                && std::abs(energyAfter - energyBefore) < std::max(1.0, energyBefore) * 1.0e-10,
                "simultaneous overlap must conserve inventory and total energy");

        enginelab::GasCell mirroredRight;
        enginelab::GasCell mirroredCylinder;
        enginelab::GasCell mirroredLeft;
        mirroredRight.initialise(78.0, 0.45, 580.0);
        mirroredCylinder.initialise(108.0, 0.060, 500.0);
        mirroredLeft.initialise(145.0, 0.40, 320.0);
        const auto mirroredFlow = enginelab::ConservativeGasSystem::flowSimultaneous(
            { &mirroredRight, &mirroredCylinder, 1.0e-4, 0.68, 1.0e-4,
              0.0, 1.0, 2.0e-4, 8.0e-4 },
            { &mirroredCylinder, &mirroredLeft, 1.2e-4, 0.72, 1.0e-4,
              0.0, -1.0, 8.0e-4, 2.0e-4 });
        const auto sameState = [](const enginelab::GasCell& first,
                                  const enginelab::GasCell& second) {
            return std::abs(first.totalMoles() - second.totalMoles()) < 1.0e-12
                && std::abs(first.internalEnergyJoules() - second.internalEnergyJoules()) < 1.0e-9
                && std::abs(first.momentumXKgMps() - second.momentumXKgMps()) < 1.0e-12
                && std::abs(first.momentumYKgMps() - second.momentumYKgMps()) < 1.0e-12;
        };
        require(mirroredFlow.first.transferredMassKg < 0.0
                && mirroredFlow.second.transferredMassKg < 0.0
                && sameState(intake, mirroredLeft)
                && sameState(cylinder, mirroredCylinder)
                && sameState(exhaust, mirroredRight),
                "simultaneous flow must be invariant when the two restrictions are permuted");
    }

    {
        const auto radial = enginelab::makeDefaultRadialFive();
        const auto reference = enginelab::buildEngineKinematicsReference(radial);
        for (std::size_t index = 1; index < radial.cylinders.size(); ++index) {
            const auto top = enginelab::evaluateCylinderKinematics(radial, reference, index,
                reference.topDeadCentreAngleDegrees[index], 0.0);
            const auto bottom = enginelab::evaluateCylinderKinematics(radial, reference, index,
                reference.bottomDeadCentreAngleDegrees[index], 0.0);
            require(top.pistonTravelMm < 1.0e-5,
                    "articulated rods must use their resolved rather than nominal TDC");
            require(bottom.chamberVolumeLitres > top.chamberVolumeLitres,
                    "articulated chamber volume must grow away from true TDC");
        }
    }

    std::cout << "Physics regression tests passed\n";
    return EXIT_SUCCESS;
}
