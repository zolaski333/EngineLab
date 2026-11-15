#include <enginelab/calibration/EcuCalibration.hpp>
#include <enginelab/calibration/EcuCalibrationKeys.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <string>
#include <variant>

namespace enginelab::calibration {
namespace {

constexpr double seaLevelPressureKpa = 101.325;

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
    return { { "rpm", "Engine speed", AxisQuantity::engineSpeed,
               CalibrationUnit::revolutionsPerMinute }, std::move(values) };
}

[[nodiscard]] CalibrationAxis loadAxis() {
    return { { "load", "Normalised load", AxisQuantity::normalizedLoad,
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

/** VTEC's settings, seeded from the engine's first VTEC camshaft so the
 *  engine runs as its cams were authored: on, at their own switch speed and
 *  throttle, no hysteresis. */
[[nodiscard]] std::array<ScalarCalibration, 4> vtecSettings(const CamshaftConfig& cam) {
    // A throttle such as 0.55 stores 55 exactly and reads back as the same
    // double (55 / 100); a finer one rounds to a tenth of a percent.
    const auto throttlePercent = std::clamp(std::round(cam.switchThrottle * 1'000.0) / 10.0,
                                            0.0, ecuLimits::maximumVtecThrottlePercent);
    return { {
        { metadata(std::string(keys::vtecEnabled), "VTEC",
                   "The ECU's VTEC switch: 1 lets the high cam engage, 0 holds the low cam (a high cam drops back "
                   "at its next base circle, with every valve shut).",
                   CalibrationUnit::dimensionless, 0.0, 1.0, 0),
          1.0 },
        { metadata(std::string(keys::vtecEngage), "VTEC engagement",
                   "The high cam engages once a whole shaft cycle runs above this speed, on the next base circle. "
                   "Above the rev limiter it never engages; under idle it is always on, the throttle permitting.",
                   CalibrationUnit::revolutionsPerMinute, 0.0, ecuLimits::maximumRevLimitRpm, 0),
          std::clamp(cam.switchRpm, 0.0, ecuLimits::maximumRevLimitRpm) },
        { metadata(std::string(keys::vtecHysteresis), "VTEC hysteresis",
                   "The high cam drops back once a whole shaft cycle runs below the engagement speed less this: a "
                   "speed held around the switch then no longer hunts between cams. Zero: it drops back below the "
                   "engagement speed itself.",
                   CalibrationUnit::revolutionsPerMinute, 0.0, ecuLimits::maximumVtecHysteresisRpm, 0),
          0.0 },
        { metadata(std::string(keys::vtecMinimumThrottle), "VTEC minimum throttle",
                   "Under this throttle the high cam never runs, whatever the speed. Zero: any throttle.",
                   CalibrationUnit::percent, 0.0, ecuLimits::maximumVtecThrottlePercent, 0),
          throttlePercent },
    } };
}

/** Anti-lag's settings, off: a turbocharged petrol engine's tables carry
    them so the tuner can arm it. */
[[nodiscard]] std::array<ScalarCalibration, 12> antiLagSettings(const EngineConfig& config) {
    return { {
        { metadata(std::string(keys::antiLagEnabled), "Anti-lag",
                   "Anti-lag (a rally car's bang-bang): 1 arms it, 0 (the default) turns it off. Armed, with your "
                   "foot off the throttle above its speed, it holds the throttle open, fires the spark late and "
                   "cuts the spark of a share of the firings with their fuel kept: those charges burn in the "
                   "exhaust manifold, before the turbine, and keep the turbo spinning. It heats the exhaust hard.",
                   CalibrationUnit::dimensionless, 0.0, 1.0, 0),
          0.0 },
        { metadata(std::string(keys::antiLagMinimumRpm), "Anti-lag minimum speed",
                   "Anti-lag works only above this engine speed.",
                   CalibrationUnit::revolutionsPerMinute, 0.0, ecuLimits::maximumRevLimitRpm, 0),
          std::clamp(0.45 * config.redlineRpm, 0.0, ecuLimits::maximumRevLimitRpm) },
        { metadata(std::string(keys::antiLagAir), "Anti-lag air",
                   "The throttle opening anti-lag holds with your foot off: the air the late and the unlit "
                   "charges need. Its feedback opens up to Anti-lag aggressiveness of it.",
                   CalibrationUnit::percent, 0.0, 100.0, 0),
          40.0 },
        { metadata(std::string(keys::antiLagRetard), "Anti-lag spark",
                   "How late the spark fires, degrees after top dead centre: the flame is still burning when the "
                   "exhaust valve opens.",
                   CalibrationUnit::degreeCrankshaft, 0.0, ecuLimits::maximumAntiLagSparkAtdcDegrees, 1),
          25.0 },
        { metadata(std::string(keys::antiLagLambda), "Anti-lag mixture",
                   "The mixture, as a lambda: under 1 is rich.",
                   CalibrationUnit::dimensionless, ecuLimits::minimumAntiLagLambda, ecuLimits::maximumAntiLagLambda, 2),
          0.90 },
        { metadata(std::string(keys::antiLagCut), "Anti-lag cut",
                   "The share of firings whose spark is cut with the fuel kept: their charge, air and fuel, "
                   "lights in the hot manifold.",
                   CalibrationUnit::percent, 0.0, 100.0, 0),
          33.0 },
        { metadata(std::string(keys::antiLagMode), "Anti-lag mode",
                   "0 rally: it works on every lift above its speed. 1 launch: only while launch control holds "
                   "the engine on the line, foot down, to build boost before the start.",
                   CalibrationUnit::dimensionless, 0.0, 1.0, 0),
          0.0 },
        { metadata(std::string(keys::antiLagBrakingKept), "Anti-lag engine braking kept",
                   "The share of the engine's own braking it keeps, by feedback on the engine's torque: 100 % brakes "
                   "as without anti-lag (and holds little boost), 0 % holds the engine's speed. It never makes the "
                   "engine gain speed.",
                   CalibrationUnit::percent, 0.0, 100.0, 0),
          50.0 },
        { metadata(std::string(keys::antiLagAggressiveness), "Anti-lag aggressiveness",
                   "How much of its air (Anti-lag air) it may open to keep the turbo spinning.",
                   CalibrationUnit::percent, 0.0, 100.0, 0),
          70.0 },
        { metadata(std::string(keys::antiLagMaximumRpm), "Anti-lag maximum speed",
                   "Above this engine speed it lets go. Zero: no limit but the rev limiter.",
                   CalibrationUnit::revolutionsPerMinute, 0.0, ecuLimits::maximumRevLimitRpm, 0),
          std::clamp(0.90 * config.redlineRpm, 0.0, ecuLimits::maximumRevLimitRpm) },
        { metadata(std::string(keys::antiLagMaximumSeconds), "Anti-lag maximum time",
                   "It lets go after working this long on one lift, until the throttle is pressed again. Zero: no "
                   "limit.",
                   CalibrationUnit::second, 0.0, ecuLimits::maximumAntiLagSeconds, 1),
          4.0 },
        { metadata(std::string(keys::antiLagExhaustLimit), "Anti-lag exhaust temperature limit",
                   "Above this exhaust temperature at the turbine inlet it lets go, and works again 50 C below "
                   "it. Zero: no limit. 1,050 C is what a turbine's stainless housing and nickel-alloy wheel "
                   "are rated to.",
                   CalibrationUnit::degreeCelsius, 0.0, ecuLimits::maximumAntiLagExhaustLimitC, 0),
          // Garrett's G-Series rating (stainless housing, Mar-M wheel). The
          // 2JZ's turbine inlet reads ~1,020 C after a full-boost pull, so
          // 1,000 C held anti-lag off on the lift it is for.
          1'050.0 },
    } };
}

[[nodiscard]] bool antiLagCapable(const EngineConfig& config) noexcept {
    return config.fuel == FuelType::gasoline && config.forcedInduction.enabled
        && config.forcedInduction.type == ForcedInductionType::turbocharger;
}

} // namespace

CalibrationDraft makeDefaultEcuCalibration(const EngineConfig& config) {
    CalibrationDraft draft;
    draft.name = config.name + " - ECU calibration";
    draft.description = "Live-editable ECU tables generated from the engine configuration.";

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
        // Above atmospheric load the map holds the full-load advance. It used
        // to pull a further 5 deg per bar of boost, an unsourced rule written
        // before the end-gas knock model and its control existed: those retard
        // a charge that knocks, so the map retarded every boosted engine a
        // second time (2JZ at 1.9 bar: CA50 21 deg ATDC with no knock at all,
        // +3.3 % torque once removed). An engine's measured limit belongs in
        // `ignition.knock_limit_curve`, which still clamps this table.
        const auto timingLoadCorrection = normalizedLoad <= 1.0
            ? 7.0 - 9.0 * normalizedLoad
            : -2.0;
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
                    ecuLimits::minimumDefaultMapAirFuelRatio,
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
                 diesel ? "AFR smoke limit" : "Target AFR",
                 diesel
                    ? "Minimum allowed AFR. Raising it leans the mixture and reduces smoke; normal running may be leaner."
                    : "Commanded mixture by engine speed and load, in petrol AFR (lambda x 14.7): "
                      "another fuel is metered at the same lambda.",
                 CalibrationUnit::airFuelRatio, minimumAfr, maximumAfr),
        speed, load, std::move(afrValues)
    });
    if (diesel && !config.injection.fullLoadFuelLimit.empty()) {
        CalibrationAxis quantitySpeed {
            { "rpm", "Engine speed", AxisQuantity::engineSpeed,
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
                     "Full-load diesel quantity",
                     "Injected ceiling per cylinder per cycle. This is the diesel torque map; the smoke limit still takes priority.",
                     CalibrationUnit::milligram,
                     ecuLimits::minimumDieselFuelQuantityMgPerCycle,
                     ecuLimits::maximumDieselFuelQuantityMgPerCycle, 1),
            std::move(quantitySpeed), std::move(quantities)
        });
    }
    draft.set(CalibrationTable2D {
        metadata(std::string(keys::ignitionAdvance), "Spark advance", "Absolute crankshaft spark advance.",
                 CalibrationUnit::degreeCrankshaft, ecuLimits::minimumIgnitionAdvanceDegrees,
                 ecuLimits::maximumIgnitionAdvanceDegrees),
        speed, load, std::move(ignitionValues)
    });
    draft.set(ScalarCalibration {
        metadata(std::string(keys::revLimit), "Rev limiter", "Rev limiter threshold.",
                 CalibrationUnit::revolutionsPerMinute, ecuLimits::minimumRevLimitRpm,
                 ecuLimits::maximumRevLimitRpm, 0),
        std::clamp(config.ignition.revLimitRpm, ecuLimits::minimumRevLimitRpm,
                   ecuLimits::maximumRevLimitRpm)
    });
    if (idleCarburettor(config) == nullptr) {
        draft.set(ScalarCalibration {
            metadata(std::string(keys::idleTarget), "Idle target",
                     "The speed the idle governor holds with the throttle shut, warm and unloaded. It opens and closes the idle air (the fuel on a diesel) to hold it; an engine whose closed-throttle leak alone outruns the target, or whose idle air is all open, idles where its air takes it.",
                     CalibrationUnit::revolutionsPerMinute, ecuLimits::minimumIdleTargetRpm,
                     ecuLimits::maximumIdleTargetRpm, 0),
            std::clamp(config.idleRpm, ecuLimits::minimumIdleTargetRpm, ecuLimits::maximumIdleTargetRpm)
        });
    }
    draft.set(ScalarCalibration {
        metadata(std::string(keys::launchLimit), "Launch control",
                 "Rev limit while the vehicle stands in gear with the throttle past half (two-step): the clutch takes up from there. Released above 5 km/h. Zero turns it off.",
                 CalibrationUnit::revolutionsPerMinute, 0.0, ecuLimits::maximumRevLimitRpm, 0),
        0.0
    });
    if (config.forcedInduction.enabled
            && config.forcedInduction.type == ForcedInductionType::turbocharger) {
        // The configuration's wastegate ratio is a sea-level ratio, so the
        // same gauge boost: flat, the controller holds what it always held.
        const auto gaugeKpa = std::clamp(
            (config.forcedInduction.wastegatePressureRatio - 1.0) * seaLevelPressureKpa,
            ecuLimits::minimumBoostTargetGaugeKpa, ecuLimits::maximumBoostTargetGaugeKpa);
        draft.set(CalibrationCurve1D {
            metadata(std::string(keys::boostTargetGauge), "Boost target",
                     "Gauge boost the wastegate controller holds, by engine speed. Below the speed where the turbo can make it, it is a ceiling the turbo does not reach.",
                     CalibrationUnit::kilopascal, ecuLimits::minimumBoostTargetGaugeKpa,
                     ecuLimits::maximumBoostTargetGaugeKpa, 1),
            speed, std::vector<double>(speed.breakpoints.size(), gaugeKpa)
        });
        draft.set(ScalarCalibration {
            metadata(std::string(keys::overboostCut), "Overboost cut",
                     "Cuts the fuel while the manifold pressure runs this far past the boost target (a wastegate too small, a boost run away), and gives it back at half of it. Zero, the default, turns it off: the boost is then free to run past its target. 40 kPa is a production ECU's kind of margin.",
                     CalibrationUnit::kilopascal, 0.0, ecuLimits::maximumOverboostCutMarginKpa, 0),
            ecuLimits::defaultOverboostCutMarginKpa
        });
    }
    if (!diesel) {
        draft.set(ScalarCalibration {
            metadata(std::string(keys::knockRetardGain), "Knock retard limit",
                     "The most spark the knock control may pull. It retards fast while the engine knocks and gives the advance back at about a degree per second, so it settles at the knock limit; past this much retard the knock is no longer held. Zero turns knock control off: the knock then runs free and severe knock damages the engine.",
                     CalibrationUnit::degreeCrankshaft, ecuLimits::minimumKnockRetardGainDegrees,
                     ecuLimits::maximumKnockRetardGainDegrees, 1),
            ecuLimits::defaultKnockRetardGainDegrees
        });
    }
    if (const auto* cam = firstVtecCamshaft(config))
        for (auto& setting : vtecSettings(*cam)) draft.set(std::move(setting));
    if (antiLagCapable(config))
        for (auto& setting : antiLagSettings(config)) draft.set(std::move(setting));
    return draft;
}

void addMissingDefaultSettings(CalibrationDraft& draft, const EngineConfig& config) {
    (void)syncVtecSettings(draft, config);
    // A table saved under a narrower AFR limit keeps the limit it was saved
    // with (the file carries it): widen it to today's, values untouched.
    if (const auto* afr = draft.find(keys::targetAirFuelRatio)) {
        const auto widest = config.fuel == FuelType::diesel
            ? ecuLimits::maximumDieselSmokeLimitAirFuelRatio : ecuLimits::maximumGasolineAirFuelRatio;
        const auto& stored = metadataOf(*afr).hardLimits.maximum;
        if (stored && *stored < widest) {
            auto widened = *afr;
            std::visit([widest](auto& entry) { entry.metadata.hardLimits.maximum = widest; }, widened);
            draft.set(std::move(widened));
        }
    }
    if (antiLagCapable(config))
        for (auto& setting : antiLagSettings(config))
            if (draft.find(setting.metadata.id) == nullptr) draft.set(std::move(setting));
    if (draft.find(keys::overboostCut) != nullptr) return;
    const auto defaults = makeDefaultEcuCalibration(config);
    if (const auto* cut = defaults.find(keys::overboostCut)) draft.set(*cut);
}

bool syncVtecSettings(CalibrationDraft& draft, const EngineConfig& config) {
    auto changed = false;
    if (const auto* cam = firstVtecCamshaft(config)) {
        for (auto& setting : vtecSettings(*cam)) {
            if (draft.find(setting.metadata.id) != nullptr) continue;
            draft.set(std::move(setting));
            changed = true;
        }
        return changed;
    }
    for (const auto key : { keys::vtecEnabled, keys::vtecEngage, keys::vtecHysteresis, keys::vtecMinimumThrottle })
        changed = draft.erase(key) || changed;
    return changed;
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

bool shiftBoostTargetTable(CalibrationStore& store, double gaugeStepKpa) {
    auto draft = makeDraft(*store.snapshot());
    const auto* found = draft.find(keys::boostTargetGauge);
    if (found == nullptr) return false;
    auto entry = *found;
    auto* curve = std::get_if<CalibrationCurve1D>(&entry);
    if (curve == nullptr) return false;
    for (auto& value : curve->values)
        value = std::clamp(value + gaugeStepKpa, ecuLimits::minimumBoostTargetGaugeKpa,
                           ecuLimits::maximumBoostTargetGaugeKpa);
    draft.set(std::move(entry));
    return store.publish(draft, { std::nullopt, "wastegate-edit" }).published;
}

} // namespace enginelab::calibration
