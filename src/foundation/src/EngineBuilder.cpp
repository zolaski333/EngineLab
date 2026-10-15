#include <enginelab/foundation/EngineBuilder.hpp>
#include <enginelab/foundation/CylinderResize.hpp>
#include <enginelab/foundation/IntakeResize.hpp>
#include <enginelab/foundation/TurboSizing.hpp>

#include <algorithm>
#include <cmath>
#include <exception>
#include <numbers>
#include <utility>
#include <vector>

namespace enginelab {
namespace {

/** The pressure ratio the engine breathes at full load. */
[[nodiscard]] double boostRatio(const EngineConfig& config) noexcept {
    const auto& forced = config.forcedInduction;
    if (!forced.enabled) return 1.0;
    return std::max(1.0, forced.type == ForcedInductionType::turbocharger ? forced.wastegatePressureRatio
                                                                          : forced.pressureRatio);
}

/** Resizes every intake path's `part` by `scale`. */
[[nodiscard]] std::string scaleIntakeParts(EngineConfig& config, IntakePart part, double scale) {
    std::vector<std::uint32_t> ids;
    for (const auto& path : config.intakePaths) ids.push_back(path.id);
    for (const auto id : ids) {
        const auto size = intakePartSize(config, id, part);
        if (!size || size->first <= 0.0) continue;
        if (auto error = resizeIntakePart(config, id, part, 0, size->first * scale); !error.empty()) return error;
    }
    return {};
}

/** Gives an engine authored by chamber geometry the head chamber that makes
 *  `ratio`; one authored by ratio takes it as is. */
[[nodiscard]] std::string setCompressionRatio(EngineConfig& config, double ratio) {
    if (!(ratio > 1.0)) return "The compression ratio must be above 1";
    for (auto& cylinder : config.cylinders) {
        cylinder.compressionRatio = ratio;
        if (cylinder.headChamberVolumeCc <= 0.0) continue;
        const auto areaMm2 = std::numbers::pi * cylinder.boreMm * cylinder.boreMm * 0.25;
        const auto sweptCc = areaMm2 * cylinder.strokeMm / 1'000.0;
        const auto deckClearanceMm = cylinder.deckHeightMm - cylinder.strokeMm * 0.5 - cylinder.connectingRodMm
            - cylinder.compressionHeightMm;
        const auto chamberCc = sweptCc / (ratio - 1.0) + cylinder.pistonCrownVolumeCc
            - areaMm2 * (cylinder.headGasketThicknessMm + deckClearanceMm) / 1'000.0;
        if (chamberCc <= 0.0)
            return "The deck and gasket leave no room for that compression ratio";
        cylinder.headChamberVolumeCc = chamberCc;
    }
    return {};
}

} // namespace

BuiltEngine buildEngine(const EngineConfig& base, const EngineRecipe& recipe) {
    BuiltEngine built;
    if (recipe.name.empty()) {
        built.error = "The engine needs a name";
        return built;
    }
    try {
        auto config = base;
        normaliseEngineConfig(config);
        const auto baseDisplacement = engineDisplacementLitres(config);
        const auto baseCylinderDisplacement = baseDisplacement / static_cast<double>(std::max<std::size_t>(1,
            config.cylinders.size()));
        const auto baseRedline = config.redlineRpm;
        const auto baseBoost = boostRatio(config);
        config.name = recipe.name;

        if (recipe.boreMm > 0.0 || recipe.strokeMm > 0.0) {
            const auto size = cylinderSize(config);
            if (!size) {
                built.error = "The base engine's cylinders differ: its bore and stroke cannot change together";
                return built;
            }
            const auto bore = recipe.boreMm > 0.0 ? recipe.boreMm : size->boreMm;
            const auto stroke = recipe.strokeMm > 0.0 ? recipe.strokeMm : size->strokeMm;
            if (auto error = resizeCylinders(config, bore, stroke); !error.empty()) {
                built.error = std::move(error);
                return built;
            }
            const auto boreScale = bore / size->boreMm;
            for (auto& cylinder : config.cylinders) {
                cylinder.pistonMassGrams *= boreScale * boreScale * boreScale;
                cylinder.connectingRodMassGrams *= boreScale * boreScale;
            }
        }
        if (recipe.compressionRatio > 0.0) {
            if (auto error = setCompressionRatio(config, recipe.compressionRatio); !error.empty()) {
                built.error = std::move(error);
                return built;
            }
        }

        if (recipe.redlineRpm > 0.0) {
            const auto speedScale = recipe.redlineRpm / baseRedline;
            config.ignition.revLimitRpm = recipe.redlineRpm + (config.ignition.revLimitRpm - baseRedline);
            config.redlineRpm = recipe.redlineRpm;
            for (auto& sample : config.ignition.timingCurve) sample.rpm *= speedScale;
            for (auto& sample : config.injection.fullLoadFuelLimit) sample.rpm *= speedScale;
        }

        switch (recipe.aspiration) {
        case Aspiration::keep: break;
        case Aspiration::natural: config = withoutForcedInduction(std::move(config)); break;
        case Aspiration::turbocharged:
            config = withTurbochargerCount(withTurbocharger(std::move(config)), recipe.turbochargerCount);
            break;
        }

        const auto displacement = engineDisplacementLitres(config);
        const auto cylinderDisplacement = displacement / static_cast<double>(std::max<std::size_t>(1,
            config.cylinders.size()));
        const auto boost = boostRatio(config);
        const auto airScale = displacement * config.redlineRpm * boost / (baseDisplacement * baseRedline * baseBoost);
        const auto torqueScale = cylinderDisplacement * boost / (baseCylinderDisplacement * baseBoost);
        const auto displacementScale = displacement / baseDisplacement;
        config.injection.injectorFlowMgPerSecond *= airScale;
        for (auto& sample : config.injection.fullLoadFuelLimit) sample.milligramsPerCycle *= torqueScale;
        config.transmission.maxClutchTorqueNm *= displacement * boost / (baseDisplacement * baseBoost);
        if (auto error = scaleIntakeParts(config, IntakePart::throttle, std::sqrt(airScale)); !error.empty()) {
            built.error = std::move(error);
            return built;
        }
        if (auto error = scaleIntakeParts(config, IntakePart::plenum, displacementScale); !error.empty()) {
            built.error = std::move(error);
            return built;
        }

        normaliseEngineConfig(config);
        if (auto error = validateEngineConfig(config)) {
            built.error = std::move(*error);
            return built;
        }
        built.config = std::move(config);
    } catch (const std::exception& error) {
        built.error = error.what();
    }
    return built;
}

} // namespace enginelab
