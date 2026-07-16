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
                && highPressure.momentumXKgMps() > 0.0,
                "reverse simple flow must inject jet momentum from the second cell toward the first");
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
        require(exhaust.transferredMassKg > 0.0 && exhaustCollector.momentumXKgMps() < 0.0,
            "a downstream boundary must retain the outlet's geometric direction and recoil");
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
