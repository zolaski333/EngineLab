#include <enginelab/calibration/EcuCalibration.hpp>
#include <enginelab/calibration/EcuCalibrationKeys.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <string>

namespace enginelab::calibration {
namespace {

[[nodiscard]] CalibrationMetadata metadata(std::string id, std::string displayName,
                                           std::string description, CalibrationUnit unit,
                                           double minimum, double maximum, int precision = 2) {
    return { std::move(id), std::move(displayName), std::move(description), unit,
             { minimum, maximum }, precision, true };
}

[[nodiscard]] CalibrationAxis rpmAxis(const EngineConfig& config) {
    const auto limit = std::clamp(config.ignition.revLimitRpm, 1'200.0,
                                  ecuLimits::maximumRevLimitRpm);
    std::vector<double> values { 0.0, 800.0, 1'500.0, 2'500.0, 3'500.0,
                                 5'000.0, 6'500.0, limit };
    std::sort(values.begin(), values.end());
    values.erase(std::unique(values.begin(), values.end(), [](double lhs, double rhs) {
        return std::abs(lhs - rhs) < 1.0;
    }), values.end());
    return { { "rpm", "Régime moteur", AxisQuantity::engineSpeed,
               CalibrationUnit::revolutionsPerMinute }, std::move(values) };
}

[[nodiscard]] CalibrationAxis loadAxis() {
    return { { "load", "Charge normalisée", AxisQuantity::normalizedLoad,
               CalibrationUnit::ratio },
             { 0.0, 0.25, 0.5, 0.75, 1.0, 1.25, 1.5, 2.0, 3.0, 4.0 } };
}

[[nodiscard]] double timingAt(const EngineConfig& config, double rpm) noexcept {
    const auto& curve = config.ignition.timingCurve;
    if (curve.empty()) return 18.0;
    if (curve.size() == 1 || rpm <= curve.front().rpm) return curve.front().advanceDegrees;
    for (std::size_t index = 1; index < curve.size(); ++index) {
        if (rpm <= curve[index].rpm) {
            const auto span = std::max(1.0, curve[index].rpm - curve[index - 1].rpm);
            return std::lerp(curve[index - 1].advanceDegrees, curve[index].advanceDegrees,
                             std::clamp((rpm - curve[index - 1].rpm) / span, 0.0, 1.0));
        }
    }
    return curve.back().advanceDegrees;
}

} // namespace

CalibrationDraft makeDefaultEcuCalibration(const EngineConfig& config) {
    CalibrationDraft draft;
    draft.name = config.name + " - calibration ECU";
    draft.description = "Tables ECU modifiables à chaud générées depuis la configuration moteur.";

    const auto speed = rpmAxis(config);
    const auto load = loadAxis();
    std::vector<double> afrValues;
    std::vector<double> ignitionValues;
    afrValues.reserve(speed.breakpoints.size() * load.breakpoints.size());
    ignitionValues.reserve(afrValues.capacity());
    for (std::size_t loadIndex = 0; loadIndex < load.breakpoints.size(); ++loadIndex) {
        const auto normalizedLoad = load.breakpoints[loadIndex];
        const auto afrLoadCorrection = normalizedLoad <= 1.0
            ? 0.45 - 1.40 * normalizedLoad
            : -0.95 - 0.65 * (normalizedLoad - 1.0);
        const auto timingLoadCorrection = normalizedLoad <= 1.0
            ? 7.0 - 9.0 * normalizedLoad
            : -2.0 - 5.0 * (normalizedLoad - 1.0);
        for (const auto rpm : speed.breakpoints) {
            const auto highSpeedEnrichment = rpm > config.redlineRpm * 0.78 ? 0.35 : 0.0;
            if (config.fuel == FuelType::diesel) {
                // This map is a smoke-limit FLOOR, not a stoichiometric target.
                // Driver demand already meters a quality-governed diesel leaner
                // at part load; the map only caps the richest admissible charge.
                // Keep the shipped engine on the EN 590 lambda-1.16 boundary so
                // the separate injected-quantity curve remains its torque map.
                afrValues.push_back(std::clamp(
                    config.fuelProperties.stoichiometricAirFuelRatio * 1.16,
                    ecuLimits::minimumAirFuelRatio,
                    ecuLimits::maximumDieselSmokeLimitAirFuelRatio));
            } else {
                afrValues.push_back(std::clamp(
                    ecuLimits::referenceAirFuelRatio + afrLoadCorrection
                        - highSpeedEnrichment,
                    ecuLimits::minimumAirFuelRatio,
                    ecuLimits::maximumGasolineAirFuelRatio));
            }
            ignitionValues.push_back(std::clamp(timingAt(config, rpm)
                                                + timingLoadCorrection,
                                                ecuLimits::minimumIgnitionAdvanceDegrees,
                                                ecuLimits::maximumIgnitionAdvanceDegrees));
        }
    }

    const auto diesel = config.fuel == FuelType::diesel;
    const auto minimumAfr = diesel
        ? config.fuelProperties.stoichiometricAirFuelRatio * 1.16
        : ecuLimits::minimumAirFuelRatio;
    const auto maximumAfr = diesel
        ? ecuLimits::maximumDieselSmokeLimitAirFuelRatio
        : ecuLimits::maximumGasolineAirFuelRatio;
    draft.set(CalibrationTable2D {
        metadata(std::string(keys::targetAirFuelRatio),
                 diesel ? "Limite fumée AFR" : "AFR cible",
                 diesel
                    ? "AFR minimum admissible. Augmenter appauvrit et réduit la fumée; le fonctionnement normal peut être plus pauvre."
                    : "Richesse commandée par régime et charge.",
                 CalibrationUnit::airFuelRatio, minimumAfr, maximumAfr),
        speed, load, std::move(afrValues)
    });
    if (diesel && !config.injection.fullLoadFuelLimit.empty()) {
        CalibrationAxis quantitySpeed {
            { "rpm", "Régime moteur", AxisQuantity::engineSpeed,
              CalibrationUnit::revolutionsPerMinute }, {}
        };
        std::vector<double> quantities;
        quantitySpeed.breakpoints.reserve(
            config.injection.fullLoadFuelLimit.size());
        quantities.reserve(config.injection.fullLoadFuelLimit.size());
        for (const auto& sample : config.injection.fullLoadFuelLimit) {
            quantitySpeed.breakpoints.push_back(sample.rpm);
            quantities.push_back(sample.milligramsPerCycle);
        }
        draft.set(CalibrationCurve1D {
            metadata(std::string(keys::dieselFuelQuantityMgPerCycle),
                     "Quantité gazole pleine charge",
                     "Plafond injecté par cylindre et par cycle. C'est la carte de couple Diesel; la limite fumée reste prioritaire.",
                     CalibrationUnit::milligram,
                     ecuLimits::minimumDieselFuelQuantityMgPerCycle,
                     ecuLimits::maximumDieselFuelQuantityMgPerCycle, 1),
            std::move(quantitySpeed), std::move(quantities)
        });
    }
    draft.set(CalibrationTable2D {
        metadata(std::string(keys::ignitionAdvance), "Avance allumage", "Avance absolue vilebrequin.",
                 CalibrationUnit::degreeCrankshaft, ecuLimits::minimumIgnitionAdvanceDegrees,
                 ecuLimits::maximumIgnitionAdvanceDegrees),
        speed, load, std::move(ignitionValues)
    });
    draft.set(ScalarCalibration {
        metadata(std::string(keys::revLimit), "Limiteur", "Seuil du limiteur de régime.",
                 CalibrationUnit::revolutionsPerMinute, ecuLimits::minimumRevLimitRpm,
                 ecuLimits::maximumRevLimitRpm, 0),
        std::clamp(config.ignition.revLimitRpm, ecuLimits::minimumRevLimitRpm,
                   ecuLimits::maximumRevLimitRpm)
    });
    return draft;
}

CalibrationDraft makeDraft(const CalibrationSnapshot& snapshot) {
    CalibrationDraft draft;
    draft.name = snapshot.name();
    draft.description = snapshot.description();
    for (const auto& [id, entry] : snapshot.entries()) {
        (void)id;
        draft.set(entry);
    }
    return draft;
}

} // namespace enginelab::calibration
