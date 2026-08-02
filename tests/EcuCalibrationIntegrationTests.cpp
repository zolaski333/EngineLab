#include <enginelab/calibration/EcuCalibration.hpp>
#include <enginelab/calibration/EcuCalibrationKeys.hpp>
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/foundation/EngineTypes.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <variant>

namespace {

void require(bool condition, const char* message) {
    if (condition) return;
    std::cerr << "ECU calibration integration failure: " << message << '\n';
    std::exit(EXIT_FAILURE);
}

void requireNear(double actual, double expected, double tolerance, const char* message) {
    require(std::isfinite(actual) && std::abs(actual - expected) <= tolerance, message);
}

} // namespace

int main() {
    auto config = enginelab::makeDefaultInlineFour();
    enginelab::SimpleEcuModel ecu;
    ecu.initialiseCalibration(config);
    auto draft = enginelab::calibration::makeDraft(*ecu.calibrationStore()->snapshot());

    auto afr = *draft.find(enginelab::calibration::keys::targetAirFuelRatio);
    auto& afrTable = std::get<enginelab::calibration::CalibrationTable2D>(afr);
    std::fill(afrTable.values.begin(), afrTable.values.end(), 12.5);
    draft.set(std::move(afr));
    auto advance = *draft.find(enginelab::calibration::keys::ignitionAdvance);
    auto& advanceTable = std::get<enginelab::calibration::CalibrationTable2D>(advance);
    std::fill(advanceTable.values.begin(), advanceTable.values.end(), 5.0);
    draft.set(std::move(advance));
    auto limiter = *draft.find(enginelab::calibration::keys::revLimit);
    std::get<enginelab::calibration::ScalarCalibration>(limiter).value = 2'500.0;
    draft.set(std::move(limiter));
    require(ecu.calibrationStore()->publish(draft).published,
            "edited tables should publish atomically");
    ecu.beginFrame();

    enginelab::EngineState state;
    state.rpm = 2'000.0;
    state.manifoldPressureKpa = config.ambientPressureKpa * 0.63;
    state.coolantTemperatureC = 90.0;
    enginelab::EngineControls controls;
    controls.ignitionEnabled = true;
    controls.throttle = 0.35;
    (void)ecu.evaluate(config, state, controls); // settle transient enrichment
    const auto command = ecu.evaluate(config, state, controls);
    requireNear(command.targetAirFuelRatio, 12.5, 1.0e-12,
                "ECU should consume the live AFR table");
    requireNear(command.ignitionAdvanceDegrees, 5.0, 1.0e-12,
                "ECU should consume the live ignition table");
    require(command.sparkEnabled, "spark should remain enabled below the calibrated limiter");

    state.rpm = 2'600.0;
    state.simulationTimeSeconds = 1.0;
    const auto limited = ecu.evaluate(config, state, controls);
    require(!limited.sparkEnabled, "calibrated rev limiter should apply without rebuilding the ECU");
    require(!limited.fuelEnabled,
            "the backwards-compatible hard limiter should cut fuel by default");
    require(!limited.wetSparkCutActive,
            "the backwards-compatible hard limiter must not request wet injection");

    auto wetLimiterConfig = config;
    wetLimiterConfig.ignition.limiterKeepsFuel = true;
    enginelab::SimpleEcuModel wetLimiterEcu;
    wetLimiterEcu.initialiseCalibration(wetLimiterConfig);
    auto wetLimiterDraft = enginelab::calibration::makeDraft(
        *wetLimiterEcu.calibrationStore()->snapshot());
    auto wetLimiter = *wetLimiterDraft.find(enginelab::calibration::keys::revLimit);
    std::get<enginelab::calibration::ScalarCalibration>(wetLimiter).value = 2'500.0;
    wetLimiterDraft.set(std::move(wetLimiter));
    require(wetLimiterEcu.calibrationStore()->publish(wetLimiterDraft).published,
            "wet limiter calibration should publish");
    wetLimiterEcu.beginFrame();
    const auto wetLimited = wetLimiterEcu.evaluate(wetLimiterConfig, state, controls);
    require(wetLimited.fuelEnabled,
            "an opted-in wet limiter should retain injection at the hard limit");
    require(!wetLimited.sparkEnabled,
            "an opted-in wet limiter should still suppress spark at the hard limit");
    require(wetLimited.wetSparkCutActive,
            "an opted-in wet limiter must explicitly authorize spark-cut injection");

    auto dieselConfig = enginelab::makeDefaultInlineFour();
    dieselConfig.fuel = enginelab::FuelType::diesel;
    dieselConfig.injection.mode = enginelab::InjectionMode::direct;
    dieselConfig.fuelProperties.stoichiometricAirFuelRatio = 14.65;
    dieselConfig.injection.fullLoadFuelLimit = {
        { 1'000.0, 42.0 }, { 2'000.0, 52.5 }, { 4'000.0, 47.0 }
    };
    enginelab::SimpleEcuModel dieselEcu;
    dieselEcu.initialiseCalibration(dieselConfig);
    auto dieselDraft = enginelab::calibration::makeDraft(
        *dieselEcu.calibrationStore()->snapshot());
    auto* dieselQuantityEntry = dieselDraft.find(
        enginelab::calibration::keys::dieselFuelQuantityMgPerCycle);
    require(dieselQuantityEntry != nullptr,
            "diesel defaults should expose the injected-quantity torque curve");
    auto dieselQuantity = *dieselQuantityEntry;
    auto& dieselQuantityCurve = std::get<
        enginelab::calibration::CalibrationCurve1D>(dieselQuantity);
    std::fill(dieselQuantityCurve.values.begin(),
              dieselQuantityCurve.values.end(), 31.0);
    dieselDraft.set(std::move(dieselQuantity));
    require(dieselEcu.calibrationStore()->publish(dieselDraft).published,
            "edited diesel quantity curve should publish atomically");
    dieselEcu.beginFrame();
    state.rpm = 2'000.0;
    state.load = 1.0;
    state.simulationTimeSeconds = 0.0;
    controls.throttle = 1.0;
    const auto dieselCommand = dieselEcu.evaluate(
        dieselConfig, state, controls);
    requireNear(dieselCommand.dieselFuelQuantityLimitMgPerCycle, 31.0,
                1.0e-12,
                "diesel ECU should consume the live quantity curve");
    require(dieselCommand.targetAirFuelRatio
                >= dieselConfig.fuelProperties.stoichiometricAirFuelRatio * 1.16,
            "diesel ECU must enforce its rich-side smoke floor");

    std::cout << "EngineLab ECU calibration integration tests passed\n";
    return EXIT_SUCCESS;
}
