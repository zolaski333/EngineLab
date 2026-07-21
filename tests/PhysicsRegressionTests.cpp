#include <enginelab/foundation/EngineTypes.hpp>
#include <enginelab/physics/ConservativeGasSystem.hpp>
#include <enginelab/physics/MechanicalKinematics.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <utility>

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
        enginelab::GasCell cell;
        cell.initialise(180.0, 0.55, 620.0);
        const auto before = cell;
        const enginelab::GasInventoryDelta valid {
            { -before.mixture().oxygenMoles * 0.08,
              -before.mixture().inertMoles * 0.08, 0.0, 0.0 },
            -before.internalEnergyJoules() * 0.08,
            0.0,
            0.0,
        };
        require(cell.tryApplyInventoryDelta(valid),
                "external conservative inventory transfer must apply atomically");
        require(std::abs(cell.mixture().oxygenMoles
                    - before.mixture().oxygenMoles * 0.92) < 1.0e-14
                && std::abs(cell.internalEnergyJoules()
                    - before.internalEnergyJoules() * 0.92) < 1.0e-10,
                "accepted inventory transfer must preserve the requested species and energy delta");
        const auto accepted = cell;
        auto invalid = valid;
        invalid.mixture.oxygenMoles = -accepted.mixture().oxygenMoles * 2.0;
        require(!cell.tryApplyInventoryDelta(invalid)
                && cell.mixture().oxygenMoles == accepted.mixture().oxygenMoles
                && cell.internalEnergyJoules() == accepted.internalEnergyJoules(),
                "an overdrawn external transfer must leave the complete cell unchanged");
    }

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

    {
        // Articulated crankpin (knuckle-pin) trace over a full cycle. The
        // articulation is fixed to the master rod at radius R and angular
        // offset A, so its distance from the crank centre must sweep the whole
        // [throw-R, throw+R] band as the master rod swings, trace a smooth
        // once-per-revolution closed loop, and drive a near-nominal stroke.
        // TDC-only checks above cannot catch a corrupted articulation rotation.
        const auto radial = enginelab::makeDefaultRadialFive();
        const auto reference = enginelab::buildEngineKinematicsReference(radial);
        constexpr double masterThrowMm = 63.5;   // crankJournals[0].throwMm
        constexpr double articulationRadiusMm = 34.0; // cylinders[1..].articulatedJournalRadiusMm
        constexpr double nominalStrokeMm = 127.0;
        const double innerBand = masterThrowMm - articulationRadiusMm; // 29.5
        const double outerBand = masterThrowMm + articulationRadiusMm; // 97.5
        for (std::size_t idx = 1; idx < radial.cylinders.size(); ++idx) {
            double prevX = 0.0, prevY = 0.0;
            double maxPinStep = 0.0;
            double minPiston = 1.0e9, maxPiston = -1.0e9;
            double minPinR = 1.0e9, maxPinR = -1.0e9;
            double maxPeriodicityErr = 0.0;
            for (int a = 0; a <= 720; ++a) {
                const auto k = enginelab::evaluateCylinderKinematics(radial, reference, idx,
                    static_cast<double>(a), 1.0);
                require(std::isfinite(k.crankPinXMm) && std::isfinite(k.crankPinYMm)
                        && std::isfinite(k.pistonPositionMm),
                        "articulated kinematics must stay finite over the whole cycle");
                const auto pinR = std::hypot(k.crankPinXMm, k.crankPinYMm);
                minPinR = std::min(minPinR, pinR); maxPinR = std::max(maxPinR, pinR);
                minPiston = std::min(minPiston, k.pistonPositionMm);
                maxPiston = std::max(maxPiston, k.pistonPositionMm);
                if (a > 0)
                    maxPinStep = std::max(maxPinStep,
                        std::hypot(k.crankPinXMm - prevX, k.crankPinYMm - prevY));
                if (a >= 360) {
                    const auto base = enginelab::evaluateCylinderKinematics(radial, reference,
                        idx, static_cast<double>(a - 360), 1.0);
                    maxPeriodicityErr = std::max(maxPeriodicityErr,
                        std::hypot(k.crankPinXMm - base.crankPinXMm,
                                   k.crankPinYMm - base.crankPinYMm));
                }
                prevX = k.crankPinXMm; prevY = k.crankPinYMm;
            }
            require(minPinR > innerBand - 0.5 && maxPinR < outerBand + 0.5,
                    "articulated crankpin must stay within [throw-R, throw+R] of the crank centre");
            require(minPinR < innerBand + 3.0 && maxPinR > outerBand - 3.0,
                    "articulated crankpin must actually sweep the articulation band (not degenerate)");
            require(maxPinStep < 3.0,
                    "articulated crankpin trace must be continuous (no jump at 1 deg resolution)");
            require(maxPeriodicityErr < 1.0e-6,
                    "articulated crankpin must be 360 deg periodic on a single crank throw");
            const auto stroke = maxPiston - minPiston;
            require(stroke > nominalStrokeMm * 0.9 && stroke < nominalStrokeMm * 1.12,
                    "articulated piston stroke must stay near the nominal crank stroke");
        }
    }

    {
        // Four branches independently see the same small, high-pressure shared
        // cell. Their combined candidate outflow deliberately exceeds its
        // inventory, exercising the N-way overdraw limiter as well as the
        // nonlinear kinetic-energy correction used by the threaded simulator.
        const auto runJacobiTransaction = [] {
            enginelab::GasCell shared;
            shared.initialise(400.0, 0.08, 500.0);
            shared.setGeometry(1.0e-3, 1.0, 0.0);
            std::array<enginelab::GasCell, 4> branches;
            for (auto& branch : branches) {
                branch.initialise(20.0, 1.0, 280.0);
                branch.setGeometry(1.0e-3, 1.0, 0.0);
            }
            const auto frozen = shared;
            auto molesBefore = shared.totalMoles();
            auto energyBefore = shared.internalEnergyJoules() + shared.bulkKineticEnergyJoules();
            auto momentumXBefore = shared.momentumXKgMps();
            for (const auto& branch : branches) {
                molesBefore += branch.totalMoles();
                energyBefore += branch.internalEnergyJoules() + branch.bulkKineticEnergyJoules();
                momentumXBefore += branch.momentumXKgMps();
            }

            std::array<enginelab::JacobiGasFlowBranch, 4> transactions;
            auto candidateSharedOxygenDelta = 0.0;
            auto candidateMomentumXDelta = 0.0;
            for (std::size_t index = 0; index < branches.size(); ++index) {
                const auto branchBefore = branches[index];
                auto sharedCandidate = frozen;
                (void)enginelab::ConservativeGasSystem::flow({
                    &sharedCandidate, &branches[index], 5.0e-3, 0.9, 0.01,
                    1.0, 0.0, 1.0e-3, 1.0e-3 });
                auto& transaction = transactions[index];
                transaction.counterpart = &branches[index];
                transaction.sharedDelta = enginelab::ConservativeGasSystem::inventoryDelta(
                    sharedCandidate, frozen);
                transaction.counterpartDelta = enginelab::ConservativeGasSystem::inventoryDelta(
                    branches[index], branchBefore);
                transaction.sharedTotalEnergyDeltaJ =
                    sharedCandidate.internalEnergyJoules() + sharedCandidate.bulkKineticEnergyJoules()
                    - frozen.internalEnergyJoules() - frozen.bulkKineticEnergyJoules();
                candidateSharedOxygenDelta += transaction.sharedDelta.mixture.oxygenMoles;
                candidateMomentumXDelta += transaction.sharedDelta.momentumXKgMps
                    + transaction.counterpartDelta.momentumXKgMps;
            }
            enginelab::ConservativeGasSystem::commitJacobiFlows(shared, transactions);

            auto molesAfter = shared.totalMoles();
            auto energyAfter = shared.internalEnergyJoules() + shared.bulkKineticEnergyJoules();
            auto momentumXAfter = shared.momentumXKgMps();
            for (const auto& branch : branches) {
                molesAfter += branch.totalMoles();
                energyAfter += branch.internalEnergyJoules() + branch.bulkKineticEnergyJoules();
                momentumXAfter += branch.momentumXKgMps();
            }
            const auto appliedScale = (shared.mixture().oxygenMoles - frozen.mixture().oxygenMoles)
                / candidateSharedOxygenDelta;
            require(appliedScale > 0.0 && appliedScale < 1.0,
                    "N-way Jacobi transaction must constrain an aggregate shared-volume overdraw");
            require(shared.mixture().oxygenMoles >= 0.0 && shared.mixture().inertMoles >= 0.0
                    && shared.mixture().fuelMoles >= 0.0 && shared.mixture().burnedMoles >= 0.0
                    && shared.internalEnergyJoules() >= 0.0,
                    "N-way Jacobi commit must leave every shared inventory non-negative");
            require(std::abs(molesAfter - molesBefore) < std::max(1.0, molesBefore) * 1.0e-12,
                    "N-way Jacobi overdraw limiting must conserve total moles");
            require(std::abs(energyAfter - energyBefore) < std::max(1.0, energyBefore) * 1.0e-10,
                    "N-way Jacobi commit must conserve sensible plus bulk kinetic energy");
            require(std::abs(momentumXAfter
                        - (momentumXBefore + appliedScale * candidateMomentumXDelta)) < 1.0e-10,
                    "N-way Jacobi commit must apply every branch momentum delta at the common scale");

            std::array<double, 17> result {
                shared.mixture().oxygenMoles, shared.mixture().inertMoles,
                shared.internalEnergyJoules(), shared.momentumXKgMps(), shared.momentumYKgMps()
            };
            auto output = std::size_t { 5 };
            for (const auto& branch : branches) {
                result[output++] = branch.totalMoles();
                result[output++] = branch.internalEnergyJoules();
                result[output++] = branch.momentumXKgMps();
            }
            return result;
        };
        const auto first = runJacobiTransaction();
        const auto second = runJacobiTransaction();
        require(first == second,
                "fixed-order N-way Jacobi commits must be bit-identical between runs");
    }

    {
        // Steady through-flow must not spin a duct cell past its own continuity
        // velocity.
        //
        // A 0-D duct such as an intake runner passes several of its own masses
        // through itself during one valve event. Any per-transfer momentum term
        // that adds to the cell instead of relaxing it toward the throat jet
        // therefore accumulates in proportion to how much mass has flowed
        // through, and the bulk velocity leaves the physical value behind. The
        // reference here is continuity, v = mdot / (rho * A) -- an identity, not
        // a calibration -- so this gate cannot be re-tuned onto the simulator's
        // own behaviour. It caught a runner carrying 172 m/s where continuity
        // gave 54, which starved every naturally aspirated engine in the
        // catalogue (docs/physics-audit.md).
        constexpr auto runnerAreaM2 = 1.963e-3;   // 50 mm bore duct
        constexpr auto runnerVolumeLitres = 0.55;
        constexpr auto valveAreaM2 = 7.75e-4;
        constexpr auto pistonAreaM2 = 8.37e-3;
        constexpr auto dtSeconds = 5.0e-5;
        const auto steadyThroughFlow = [&](int steps) {
            enginelab::GasCell plenum, runner, sink;
            runner.setGeometry(runnerAreaM2, 0.0, 1.0);
            plenum.initialise(200.0, 6.0, 320.0);
            runner.initialise(200.0, runnerVolumeLitres, 320.0);
            sink.initialise(80.0, 0.78, 320.0);
            auto throughMassKg = 0.0;
            for (int step = 0; step < steps; ++step) {
                // Both ends are held at fixed state so the probe measures the
                // duct's own response to a sustained flow rather than a
                // blow-down transient.
                plenum.reset(200.0, 320.0);
                sink.reset(80.0, 320.0);
                (void)enginelab::ConservativeGasSystem::flow({ &plenum, &runner,
                    runnerAreaM2, 0.78, dtSeconds, 0.0, 1.0, 0.0, runnerAreaM2 });
                const auto valve = enginelab::ConservativeGasSystem::flow({ &runner, &sink,
                    valveAreaM2, 0.70, dtSeconds, 0.0, 1.0, runnerAreaM2, pistonAreaM2 });
                throughMassKg += std::abs(valve.transferredMassKg);
            }
            const auto density = runner.massKg() / runner.volumeM3();
            const auto continuityVelocity = throughMassKg
                / (static_cast<double>(steps) * dtSeconds) / (density * runnerAreaM2);
            return std::pair { std::abs(runner.bulkVelocityMps()), continuityVelocity };
        };

        const auto [shortVelocity, shortContinuity] = steadyThroughFlow(200);
        const auto [longVelocity, longContinuity] = steadyThroughFlow(1'600);
        require(shortContinuity > 1.0 && longContinuity > 1.0,
                "steady through-flow probe must actually move gas");
        require(longVelocity <= longContinuity * 1.5,
                "a duct cell in steady through-flow must not exceed its continuity velocity");
        require(shortVelocity <= shortContinuity * 1.5,
                "duct bulk velocity must respect continuity from the first cell-mass on");
        // The sharpest form of the same statement, and the one free of any
        // chosen margin: passing eight times as much mass through the duct must
        // not leave it spinning faster.
        require(longVelocity <= shortVelocity * 1.2,
                "duct bulk velocity must not accumulate with the mass passed through it");
    }

    std::cout << "Physics regression tests passed\n";
    return EXIT_SUCCESS;
}
