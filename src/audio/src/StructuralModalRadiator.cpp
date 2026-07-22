#include <enginelab/audio/StructuralModalRadiator.hpp>

#include <algorithm>
#include <cmath>
#include <numbers>

namespace enginelab {
namespace {

constexpr double aluminiumYoungsModulusPa = 70.0e9;
constexpr double aluminiumDensityKgPerM3 = 2'700.0;
constexpr double aluminiumPoissonRatio = 0.33;
constexpr double airDensityKgPerM3 = 1.204;
constexpr double airSoundSpeedMps = 343.0;

[[nodiscard]] std::size_t bankCount(const EngineConfig& config) noexcept {
    if (!config.banks.empty()) return config.banks.size();
    return config.layout == EngineLayout::vLayout || config.layout == EngineLayout::flat
        ? 2U : 1U;
}

[[nodiscard]] double modeRadiationEfficiency(double frequencyHz,
                                             double radiatingAreaM2) noexcept {
    const auto equivalentRadiusM = std::sqrt(
        std::max(radiatingAreaM2, 1.0e-8) / std::numbers::pi);
    const auto waveNumberPerM = 2.0 * std::numbers::pi * frequencyHz
        / airSoundSpeedMps;
    const auto ka = waveNumberPerM * equivalentRadiusM;
    // Low-ka baffled-piston radiation rises as (ka)^2 and tends to unity once
    // the panel is acoustically large. This is an impedance law, not a voicing
    // curve; its result remains visible through ModeInfo.
    return std::clamp(ka * ka / (1.0 + ka * ka), 0.0, 1.0);
}

} // namespace

StructuralModalRadiator::StructuralModalRadiator(const EngineConfig& config) {
    if (config.cylinders.empty()) return;

    double meanBoreM = 0.0;
    double meanStrokeM = 0.0;
    double meanRodM = 0.0;
    for (const auto& cylinder : config.cylinders) {
        meanBoreM += cylinder.boreMm * 0.001;
        meanStrokeM += cylinder.strokeMm * 0.001;
        meanRodM += cylinder.connectingRodMm * 0.001;
    }
    const auto cylinderCount = config.cylinders.size();
    const auto inverseCount = 1.0 / static_cast<double>(cylinderCount);
    meanBoreM *= inverseCount;
    meanStrokeM *= inverseCount;
    meanRodM *= inverseCount;

    const auto banks = bankCount(config);
    const auto cylindersPerBank = static_cast<double>(
        (cylinderCount + banks - 1U) / banks);
    // Family estimate: bore pitch includes a water-jacket/web allowance; deck
    // height follows the actual crank/rod geometry. No engine block dimensions
    // exist in schema v1, so the provenance must remain estimatedFamily.
    const auto blockLengthM = std::max(0.18,
        meanBoreM * (1.18 * cylindersPerBank + 0.85));
    const auto blockWidthM = std::max(0.12, meanBoreM * (
        config.layout == EngineLayout::inlineLayout ? 1.75
        : config.layout == EngineLayout::radial ? 3.2 : 2.75));
    const auto blockHeightM = std::max(0.16,
        meanRodM + 0.65 * meanStrokeM + 0.55 * meanBoreM);
    const auto shellThicknessM = std::clamp(meanBoreM * 0.075, 0.0045, 0.010);
    const auto innerWidthM = std::max(0.01, blockWidthM - 2.0 * shellThicknessM);
    const auto innerHeightM = std::max(0.01, blockHeightM - 2.0 * shellThicknessM);
    const auto shellAreaM2 = blockWidthM * blockHeightM
        - innerWidthM * innerHeightM;
    const auto shellSecondMomentVerticalM4 = (
        blockWidthM * std::pow(blockHeightM, 3.0)
        - innerWidthM * std::pow(innerHeightM, 3.0)) / 12.0;
    const auto shellSecondMomentLateralM4 = (
        blockHeightM * std::pow(blockWidthM, 3.0)
        - innerHeightM * std::pow(innerWidthM, 3.0)) / 12.0;
    const auto blockMassKg = std::max(2.0,
        aluminiumDensityKgPerM3 * shellAreaM2 * blockLengthM);
    const auto blockRadiatingAreaM2 = 2.0 * blockLengthM
        * (blockWidthM + blockHeightM);

    const auto appendBeamModes = [&](Drive drive, double secondMomentM4,
                                     std::size_t count, double damping) {
        const auto flexuralScale = std::sqrt(aluminiumYoungsModulusPa
            * secondMomentM4 / (aluminiumDensityKgPerM3 * shellAreaM2));
        for (std::size_t order = 1; order <= count; ++order) {
            const auto beta = static_cast<double>(order) * std::numbers::pi;
            const auto frequencyHz = beta * beta / (2.0 * std::numbers::pi
                * blockLengthM * blockLengthM) * flexuralScale;
            Mode compiled;
            compiled.info = { frequencyHz, damping, blockMassKg * 0.5,
                blockRadiatingAreaM2,
                modeRadiationEfficiency(frequencyHz, blockRadiatingAreaM2) };
            compiled.drive = drive;
            for (std::size_t cylinder = 0; cylinder < cylinderCount; ++cylinder) {
                const auto local = (static_cast<double>(cylinder %
                    static_cast<std::size_t>(cylindersPerBank)) + 0.5)
                    / cylindersPerBank;
                compiled.participation[cylinder] = static_cast<float>(
                    std::sin(static_cast<double>(order) * std::numbers::pi * local));
            }
            modes_.push_back(compiled);
        }
    };
    appendBeamModes(Drive::bearingAxial, shellSecondMomentVerticalM4, 3U, 0.035);
    appendBeamModes(Drive::bearingLateral, shellSecondMomentLateralM4, 3U, 0.040);

    // Each bank head is represented as a simply supported aluminium plate.
    const auto headLengthM = blockLengthM;
    const auto headWidthM = std::max(meanBoreM * 1.35,
        blockWidthM / static_cast<double>(banks));
    const auto headThicknessM = std::clamp(meanBoreM * 0.18, 0.012, 0.024);
    const auto plateRigidity = aluminiumYoungsModulusPa * std::pow(headThicknessM, 3.0)
        / (12.0 * (1.0 - aluminiumPoissonRatio * aluminiumPoissonRatio));
    const auto plateScale = std::sqrt(plateRigidity
        / (aluminiumDensityKgPerM3 * headThicknessM));
    const auto headAreaM2 = headLengthM * headWidthM * static_cast<double>(banks);
    const auto headMassKg = aluminiumDensityKgPerM3 * headAreaM2 * headThicknessM;
    for (std::size_t longitudinal = 1; longitudinal <= 3; ++longitudinal) {
        for (std::size_t transverse = 1; transverse <= 2; ++transverse) {
            const auto frequencyHz = std::numbers::pi * 0.5 * plateScale * (
                std::pow(static_cast<double>(longitudinal) / headLengthM, 2.0)
                + std::pow(static_cast<double>(transverse) / headWidthM, 2.0));
            Mode compiled;
            compiled.info = { frequencyHz, 0.045, std::max(0.5, headMassKg * 0.25),
                headAreaM2, modeRadiationEfficiency(frequencyHz, headAreaM2) };
            compiled.drive = Drive::headGas;
            for (std::size_t cylinder = 0; cylinder < cylinderCount; ++cylinder) {
                const auto local = (static_cast<double>(cylinder %
                    static_cast<std::size_t>(cylindersPerBank)) + 0.5)
                    / cylindersPerBank;
                compiled.participation[cylinder] = static_cast<float>(
                    std::sin(static_cast<double>(longitudinal)
                        * std::numbers::pi * local));
            }
            modes_.push_back(compiled);
        }
    }

    // Two torsional shell modes. The equivalent torque radius converts Nm to a
    // generalized tangential force while retaining dimensional consistency.
    const auto shearModulusPa = aluminiumYoungsModulusPa
        / (2.0 * (1.0 + aluminiumPoissonRatio));
    const auto torsionalWaveSpeedMps = std::sqrt(
        shearModulusPa / aluminiumDensityKgPerM3);
    for (std::size_t order = 1; order <= 2; ++order) {
        const auto frequencyHz = static_cast<double>(order)
            * torsionalWaveSpeedMps / (2.0 * blockLengthM);
        Mode compiled;
        compiled.info = { frequencyHz, 0.030, blockMassKg * 0.45,
            blockRadiatingAreaM2,
            modeRadiationEfficiency(frequencyHz, blockRadiatingAreaM2) };
        compiled.drive = Drive::torsion;
        compiled.torqueRadiusM = std::max(0.025, 0.5 * blockWidthM);
        for (std::size_t cylinder = 0; cylinder < cylinderCount; ++cylinder) {
            const auto local = (static_cast<double>(cylinder %
                static_cast<std::size_t>(cylindersPerBank)) + 0.5)
                / cylindersPerBank;
            compiled.participation[cylinder] = static_cast<float>(
                std::sin(static_cast<double>(order) * std::numbers::pi * local));
        }
        modes_.push_back(compiled);
    }
}

bool StructuralModalRadiator::prepare(double sampleRateHz,
                                      double observerDistanceM) noexcept {
    if (!(sampleRateHz > 1'000.0) || !std::isfinite(sampleRateHz)
        || !(observerDistanceM > 0.0) || !std::isfinite(observerDistanceM)
        || modes_.empty())
        return false;
    sampleRateHz_ = sampleRateHz;
    observerDistanceM_ = observerDistanceM;
    const auto dt = 1.0 / sampleRateHz_;
    for (auto& mode : modes_) {
        if (!(mode.info.frequencyHz > 0.0)
            || mode.info.frequencyHz >= 0.45 * sampleRateHz_
            || !(mode.info.modalMassKg > 0.0)) {
            mode.decay = 0.0;
            continue;
        }
        mode.angularFrequencyRadPerSecond = 2.0 * std::numbers::pi
            * mode.info.frequencyHz;
        const auto damping = std::clamp(mode.info.dampingRatio, 0.0, 0.99);
        mode.dampedFrequencyRadPerSecond = mode.angularFrequencyRadPerSecond
            * std::sqrt(1.0 - damping * damping);
        mode.decay = std::exp(-damping
            * mode.angularFrequencyRadPerSecond * dt);
        mode.cosine = std::cos(mode.dampedFrequencyRadPerSecond * dt);
        mode.sine = std::sin(mode.dampedFrequencyRadPerSecond * dt);
    }
    reset();
    return true;
}

void StructuralModalRadiator::reset() noexcept {
    for (auto& mode : modes_) {
        mode.displacementM = 0.0;
        mode.velocityMps = 0.0;
    }
}

float StructuralModalRadiator::process(
    const StructuralExcitationSample& excitation) noexcept {
    const auto count = std::min<std::size_t>(
        excitation.cylinderCount, excitation.gasForceN.size());
    auto pressurePa = 0.0;
    for (auto& mode : modes_) {
        if (!(mode.decay > 0.0)) continue;
        auto generalizedForceN = 0.0;
        for (std::size_t cylinder = 0; cylinder < count; ++cylinder) {
            const auto participation = static_cast<double>(
                mode.participation[cylinder]);
            switch (mode.drive) {
            case Drive::headGas:
                generalizedForceN += participation
                    * excitation.gasForceN[cylinder];
                break;
            case Drive::bearingAxial:
                generalizedForceN += participation
                    * excitation.bearingReactionForceN[cylinder];
                break;
            case Drive::bearingLateral:
                generalizedForceN += participation
                    * excitation.sideThrustForceN[cylinder];
                break;
            case Drive::torsion:
                generalizedForceN += participation
                    * excitation.crankReactionTorqueNm[cylinder]
                    / mode.torqueRadiusM;
                break;
            }
        }
        const auto forcingAccelerationMps2 = generalizedForceN
            / mode.info.modalMassKg;
        const auto omega = mode.angularFrequencyRadPerSecond;
        const auto damping = std::clamp(mode.info.dampingRatio, 0.0, 0.99);
        const auto equilibriumM = forcingAccelerationMps2 / (omega * omega);
        const auto relativeM = mode.displacementM - equilibriumM;
        const auto dampedOmega = mode.dampedFrequencyRadPerSecond;
        const auto nextDisplacementM = equilibriumM + mode.decay * (
            relativeM * mode.cosine
            + (mode.velocityMps + damping * omega * relativeM)
                / dampedOmega * mode.sine);
        const auto nextVelocityMps = mode.decay * (
            mode.velocityMps * mode.cosine
            - (damping * omega * mode.velocityMps + omega * omega * relativeM)
                / dampedOmega * mode.sine);
        mode.displacementM = nextDisplacementM;
        mode.velocityMps = nextVelocityMps;

        // Energy-consistent far-field conversion for a radiating panel. The
        // configured modal surface velocity produces acoustic power
        // 0.5*rho*c*sigma*S*v_peak^2, spread over a sphere at the observer.
        const auto pressurePerVelocity = airDensityKgPerM3 * airSoundSpeedMps
            * std::sqrt(mode.info.radiationEfficiency
                * mode.info.radiatingAreaM2
                / (4.0 * std::numbers::pi
                    * observerDistanceM_ * observerDistanceM_));
        pressurePa += pressurePerVelocity * nextVelocityMps;
    }
    if (!std::isfinite(pressurePa)) {
        reset();
        return 0.0F;
    }
    return static_cast<float>(pressurePa);
}

StructuralModalRadiator::ModeInfo StructuralModalRadiator::mode(
    std::size_t index) const noexcept {
    return index < modes_.size() ? modes_[index].info : ModeInfo {};
}

} // namespace enginelab
