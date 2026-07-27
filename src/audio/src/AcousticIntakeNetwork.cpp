#include <enginelab/audio/AcousticIntakeNetwork.hpp>

#include <enginelab/audio/DuctWallLoss.hpp>
#include <enginelab/audio/PipeRadiationModel.hpp>
#include <enginelab/audio/ValvePortTermination.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <vector>

namespace enginelab {
namespace {

[[nodiscard]] std::size_t nextPowerOfTwo(std::size_t value) noexcept {
    auto result = std::size_t { 1 };
    while (result < value && result <= std::numeric_limits<std::size_t>::max() / 2U)
        result <<= 1U;
    return result;
}

[[nodiscard]] std::size_t intakePathIndexFor(
    const EngineConfig& config, const CylinderConfig& cylinder) noexcept {
    const auto found = std::find_if(config.intakePaths.begin(), config.intakePaths.end(),
        [&cylinder](const IntakePathConfig& path) {
            return std::find(path.cylinderIds.begin(), path.cylinderIds.end(), cylinder.id)
                != path.cylinderIds.end();
        });
    return found != config.intakePaths.end()
        ? static_cast<std::size_t>(std::distance(config.intakePaths.begin(), found)) : 0U;
}

[[nodiscard]] const IntakeConfig& intakeGeometryAt(
    const EngineConfig& config, std::size_t path) noexcept {
    return path < config.intakePaths.size()
        ? config.intakePaths[path].geometry : config.intake;
}

[[nodiscard]] double circularAreaM2(double diameterMm) noexcept {
    const auto radiusM = std::max(1.0, diameterMm) * 0.0005;
    return std::numbers::pi * radiusM * radiusM;
}

} // namespace

struct AcousticIntakeNetwork::Impl final {
    struct Duct final {
        double lengthM {};
        double radiusM {};
        double areaM2 {};
        double inletAreaM2 {};
        double outletAreaM2 {};
        std::vector<float> forward;
        std::vector<float> reverse;
        std::size_t write {};
        std::size_t mask {};
        float delaySamples { 1.0F };
        float delayTargetSamples { 1.0F };
        DuctWallLoss::Coefficients wallLoss {};
        DuctWallLoss::State forwardLoss {};
        DuctWallLoss::State reverseLoss {};
    };

    struct Runner final {
        std::size_t cylinderIndex {};
        std::size_t pathIndex {};
        Duct duct;
        float incidentAtValve {};
        float incidentAtPlenum {};
        float outgoingAtValve {};
        float outgoingAtPlenum {};
        float meanMassFlowKgPerSecond {};
        bool meanInitialised { false };
        ValvePortTermination::State reflectionState {};
    };

    struct Compliance final {
        double volumeM3 {};
        float admittanceM3PerPaSecond {};
        float incident {};
    };

    struct Path final {
        IntakeConfig geometry;
        std::vector<std::size_t> runners;
        Compliance plenum;
        Compliance airbox;
        float throttleIncidentAtPlenum {};
        float throttleIncidentAtAirbox {};
        float throttleAdmittanceM3PerPaSecond {};
        bool hasInletDuct { false };
        Duct inletDuct;
        float inletIncidentAtMouth {};
        float inletIncidentAtAirbox {};
        float inletOutgoingAtMouth {};
        float inletOutgoingAtAirbox {};
        float radiationIncidentAtAirbox {};
        UnflangedPipeRadiation radiation;
        FreeFieldObserver observer;
        PathBoundary medium;
    };

    std::vector<Runner> runners;
    std::vector<Path> paths;
    double sampleRateHz { 48'000.0 };
    double observerDistanceM { 1.0 };
    AcousticObserverConfig observerConfig;
    float meanFlowCoefficient { 0.0F };
    Diagnostics diagnostics;
    bool configured { false };
    bool prepared { false };

    explicit Impl(const EngineConfig& config)
        : observerConfig(config.acousticObserver) {
        if (config.cylinders.empty()) return;
        const auto count = std::clamp<std::size_t>(
            config.intakePaths.empty() ? 1U : config.intakePaths.size(), 1U,
            maximumPaths);
        paths.resize(count);
        for (std::size_t path = 0; path < count; ++path) {
            auto& compiled = paths[path];
            compiled.geometry = intakeGeometryAt(config, path);
            compiled.plenum.volumeM3 = std::max(1.0e-6,
                compiled.geometry.plenumVolumeLitres * 0.001);
            compiled.airbox.volumeM3 = std::max(0.0,
                compiled.geometry.airboxVolumeLitres * 0.001);
            const auto inletDiameterMm = compiled.geometry.inletDuctDiameterMm > 1.0
                ? compiled.geometry.inletDuctDiameterMm
                : compiled.geometry.throttleDiameterMm;
            compiled.hasInletDuct = compiled.geometry.inletDuctLengthMm > 1.0;
            if (compiled.hasInletDuct) {
                compiled.inletDuct.lengthM = compiled.geometry.inletDuctLengthMm * 0.001;
                compiled.inletDuct.areaM2 = circularAreaM2(inletDiameterMm);
                compiled.inletDuct.radiusM = std::sqrt(
                    compiled.inletDuct.areaM2 / std::numbers::pi);
            }
        }
        runners.reserve(config.cylinders.size());
        for (std::size_t cylinder = 0; cylinder < config.cylinders.size(); ++cylinder) {
            const auto path = std::min(intakePathIndexFor(
                config, config.cylinders[cylinder]), count - 1U);
            const auto& geometry = paths[path].geometry;
            const auto lengthMm = config.cylinders[cylinder].intakeRunnerLengthMm > 1.0
                ? config.cylinders[cylinder].intakeRunnerLengthMm
                : geometry.runnerLengthMm;
            const auto diameterMm = config.cylinders[cylinder].intakeRunnerDiameterMm > 1.0
                ? config.cylinders[cylinder].intakeRunnerDiameterMm
                : geometry.runnerDiameterMm;
            const auto plenumDiameterMm = geometry.runnerPlenumDiameterMm > 1.0
                ? geometry.runnerPlenumDiameterMm : diameterMm;
            Runner runner;
            runner.cylinderIndex = cylinder;
            runner.pathIndex = path;
            runner.duct.lengthM = std::max(0.001, lengthMm * 0.001);
            runner.duct.inletAreaM2 = circularAreaM2(diameterMm);
            runner.duct.outletAreaM2 = circularAreaM2(plenumDiameterMm);
            runner.duct.areaM2 = (runner.duct.inletAreaM2
                + std::sqrt(runner.duct.inletAreaM2
                    * runner.duct.outletAreaM2)
                + runner.duct.outletAreaM2) / 3.0;
            runner.duct.radiusM = std::sqrt(runner.duct.areaM2 / std::numbers::pi);
            paths[path].runners.push_back(runners.size());
            runners.push_back(std::move(runner));
        }
        configured = !runners.empty() && std::all_of(paths.begin(), paths.end(),
            [](const auto& path) { return !path.runners.empty(); });
    }

    [[nodiscard]] float readDelayed(const Duct& duct,
                                    const std::vector<float>& line) const noexcept {
        const auto delay0 = static_cast<std::size_t>(duct.delaySamples);
        const auto fraction = duct.delaySamples - static_cast<float>(delay0);
        const auto read0 = (duct.write + line.size() - delay0) & duct.mask;
        const auto read1 = (duct.write + line.size() - delay0 - 1U) & duct.mask;
        return std::lerp(line[read0], line[read1], fraction);
    }

    void prepareDuct(Duct& duct) {
        constexpr double minimumSoundSpeedMps = 300.0;
        constexpr double maximumTimeScaleStretch = 4.0;
        const auto maximumSamples = static_cast<std::size_t>(std::ceil(
            duct.lengthM / minimumSoundSpeedMps * sampleRateHz
                * maximumTimeScaleStretch)) + 4U;
        const auto size = std::max<std::size_t>(64U,
            nextPowerOfTwo(maximumSamples));
        duct.forward.assign(size, 0.0F);
        duct.reverse.assign(size, 0.0F);
        duct.mask = size - 1U;
    }

};

AcousticIntakeNetwork::AcousticIntakeNetwork(const EngineConfig& config)
    : impl_(std::make_unique<Impl>(config)) {}

AcousticIntakeNetwork::~AcousticIntakeNetwork() = default;

bool AcousticIntakeNetwork::prepare(double sampleRateHz,
                                    double observerDistanceM) {
    if (!impl_->configured || !(sampleRateHz > 1'000.0)
        || !std::isfinite(sampleRateHz) || !(observerDistanceM > 0.0)
        || !std::isfinite(observerDistanceM))
        return false;
    impl_->sampleRateHz = sampleRateHz;
    impl_->observerDistanceM = observerDistanceM;
    // Five hertz separates the conserved mean flow from the acoustic
    // perturbation while retaining every audible engine order.
    impl_->meanFlowCoefficient = static_cast<float>(1.0 - std::exp(
        -2.0 * std::numbers::pi * 5.0 / sampleRateHz));
    for (auto& runner : impl_->runners) impl_->prepareDuct(runner.duct);
    for (auto& path : impl_->paths) {
        if (path.hasInletDuct) impl_->prepareDuct(path.inletDuct);
        const auto inletDiameterMm = path.geometry.bellmouthDiameterMm > 1.0
            ? path.geometry.bellmouthDiameterMm
            : path.geometry.inletDuctDiameterMm > 1.0
                ? path.geometry.inletDuctDiameterMm
                : path.geometry.throttleDiameterMm;
        const auto radiusM = inletDiameterMm * 0.0005;
        if (!path.radiation.prepare(sampleRateHz, radiusM, 1.0)
            || !path.observer.prepare(sampleRateHz, radiusM, {},
                { 0.0, 1.0, 0.0 }, AcousticTerminationType::unflanged,
                impl_->observerConfig))
            return false;
        // A sharp unflanged mouth sheds vortices once acoustic particle
        // velocity is no longer infinitesimal. The quasi-steady termination
        // resistance is Znl/Zc = 2*Cd/(3*pi) * |u|/c with Cd=2 for a
        // thin-wall edge (Peters et al.; Atig et al.). Without this passive
        // loss, a linear Helmholtz mode can grow beyond its own validity range.
        constexpr double unflangedVortexLoss = 4.0
            / (3.0 * std::numbers::pi);
        if (!path.radiation.setNonlinearLossCoefficient(
                unflangedVortexLoss))
            return false;
    }
    impl_->prepared = true;
    reset();
    std::array<PathBoundary, maximumPaths> defaults {};
    beginBlock(std::span<const PathBoundary>(
        defaults.data(), impl_->paths.size()), 1.0);
    return true;
}

void AcousticIntakeNetwork::reset() noexcept {
    if (!impl_) return;
    impl_->diagnostics = {};
    const auto resetDuct = [](Impl::Duct& duct) {
        std::fill(duct.forward.begin(), duct.forward.end(), 0.0F);
        std::fill(duct.reverse.begin(), duct.reverse.end(), 0.0F);
        duct.write = 0;
        duct.delaySamples = 1.0F;
        duct.delayTargetSamples = 1.0F;
        duct.forwardLoss.reset();
        duct.reverseLoss.reset();
    };
    for (auto& runner : impl_->runners) {
        resetDuct(runner.duct);
        runner.incidentAtValve = 0.0F;
        runner.incidentAtPlenum = 0.0F;
        runner.outgoingAtValve = 0.0F;
        runner.outgoingAtPlenum = 0.0F;
        runner.meanMassFlowKgPerSecond = 0.0F;
        runner.meanInitialised = false;
        runner.reflectionState.reset();
    }
    for (auto& path : impl_->paths) {
        if (path.hasInletDuct) resetDuct(path.inletDuct);
        path.plenum.incident = 0.0F;
        path.airbox.incident = 0.0F;
        path.throttleIncidentAtPlenum = 0.0F;
        path.throttleIncidentAtAirbox = 0.0F;
        path.inletIncidentAtMouth = 0.0F;
        path.inletIncidentAtAirbox = 0.0F;
        path.inletOutgoingAtMouth = 0.0F;
        path.inletOutgoingAtAirbox = 0.0F;
        path.radiationIncidentAtAirbox = 0.0F;
        path.radiation.reset();
        path.observer.reset();
    }
}

void AcousticIntakeNetwork::beginBlock(
    std::span<const PathBoundary> boundaries,
    double acousticTimeScale) noexcept {
    if (!impl_->prepared) return;
    const auto timeScale = std::clamp(
        std::isfinite(acousticTimeScale) ? acousticTimeScale : 1.0, 0.25, 4.0);
    for (std::size_t pathIndex = 0; pathIndex < impl_->paths.size(); ++pathIndex) {
        auto& path = impl_->paths[pathIndex];
        if (pathIndex < boundaries.size()
            && std::isfinite(boundaries[pathIndex].densityKgPerM3)
            && boundaries[pathIndex].densityKgPerM3 > 0.0F
            && std::isfinite(boundaries[pathIndex].soundSpeedMps)
            && boundaries[pathIndex].soundSpeedMps > 0.0F)
            path.medium = boundaries[pathIndex];
        const auto rho = static_cast<double>(path.medium.densityKgPerM3);
        const auto c = static_cast<double>(path.medium.soundSpeedMps);
        path.plenum.admittanceM3PerPaSecond = static_cast<float>(
            2.0 * path.plenum.volumeM3 / (rho * c * c) * impl_->sampleRateHz);
        path.airbox.admittanceM3PerPaSecond = path.airbox.volumeM3 > 0.0
            ? static_cast<float>(2.0 * path.airbox.volumeM3
                / (rho * c * c) * impl_->sampleRateHz) : 0.0F;
        const auto throttleArea = std::isfinite(path.medium.throttleConductanceAreaM2)
            ? std::max(0.0F, path.medium.throttleConductanceAreaM2) : 0.0F;
        path.throttleAdmittanceM3PerPaSecond = throttleArea
            / (path.medium.densityKgPerM3 * path.medium.soundSpeedMps);
        (void) path.radiation.setMedium(rho, c);

        const auto fitDuct = [&](Impl::Duct& duct) {
            const auto traversalSeconds = duct.lengthM / c / timeScale;
            const auto limit = static_cast<float>(duct.forward.size() - 2U);
            duct.delayTargetSamples = std::clamp(static_cast<float>(
                traversalSeconds * impl_->sampleRateHz), 1.0F, limit);
            if (duct.delaySamples <= 1.0F)
                duct.delaySamples = duct.delayTargetSamples;
            duct.wallLoss = DuctWallLoss::fit(
                traversalSeconds, duct.radiusM, rho, c, impl_->sampleRateHz);
        };
        for (const auto runnerIndex : path.runners)
            fitDuct(impl_->runners[runnerIndex].duct);
        if (path.hasInletDuct) fitDuct(path.inletDuct);
    }
}

std::array<StereoPressure, AcousticIntakeNetwork::maximumPaths>
AcousticIntakeNetwork::process(
    std::span<const CylinderBoundary> cylinders,
    float delayRampCoefficient) noexcept {
    std::array<StereoPressure, maximumPaths> result {};
    if (!impl_->prepared) return result;
    const auto ramp = std::clamp(delayRampCoefficient, 0.0F, 1.0F);
    for (auto& runner : impl_->runners) {
        auto& duct = runner.duct;
        duct.delaySamples += ramp * (duct.delayTargetSamples - duct.delaySamples);
        runner.incidentAtValve = DuctWallLoss::process(
            duct.wallLoss, duct.reverseLoss, impl_->readDelayed(duct, duct.reverse));
        runner.incidentAtPlenum = DuctWallLoss::process(
            duct.wallLoss, duct.forwardLoss, impl_->readDelayed(duct, duct.forward));
        impl_->diagnostics.runnerPressurePa = std::max(
            impl_->diagnostics.runnerPressurePa,
            std::max(std::abs(runner.incidentAtValve),
                std::abs(runner.incidentAtPlenum)));
    }
    for (auto& path : impl_->paths) {
        if (!path.hasInletDuct) continue;
        auto& duct = path.inletDuct;
        duct.delaySamples += ramp * (duct.delayTargetSamples - duct.delaySamples);
        path.inletIncidentAtMouth = DuctWallLoss::process(
            duct.wallLoss, duct.reverseLoss, impl_->readDelayed(duct, duct.reverse));
        path.inletIncidentAtAirbox = DuctWallLoss::process(
            duct.wallLoss, duct.forwardLoss, impl_->readDelayed(duct, duct.forward));
    }

    for (std::size_t pathIndex = 0; pathIndex < impl_->paths.size(); ++pathIndex) {
        auto& path = impl_->paths[pathIndex];
        const auto rho = path.medium.densityKgPerM3;
        const auto c = path.medium.soundSpeedMps;
        auto weighted = static_cast<double>(path.throttleAdmittanceM3PerPaSecond)
            * path.throttleIncidentAtPlenum
            + static_cast<double>(path.plenum.admittanceM3PerPaSecond)
                * path.plenum.incident;
        auto admittance = static_cast<double>(path.throttleAdmittanceM3PerPaSecond)
            + path.plenum.admittanceM3PerPaSecond;
        for (const auto runnerIndex : path.runners) {
            const auto& runner = impl_->runners[runnerIndex];
            const auto runnerAdmittance = static_cast<float>(
                runner.duct.outletAreaM2 / (rho * c));
            weighted += runnerAdmittance * runner.incidentAtPlenum;
            admittance += runnerAdmittance;
        }
        const auto plenumPressure = admittance > 1.0e-15
            ? static_cast<float>(2.0 * weighted / admittance) : 0.0F;
        impl_->diagnostics.plenumPressurePa = std::max(
            impl_->diagnostics.plenumPressurePa, std::abs(plenumPressure));
        for (const auto runnerIndex : path.runners) {
            auto& runner = impl_->runners[runnerIndex];
            runner.outgoingAtPlenum = plenumPressure - runner.incidentAtPlenum;
        }
        const auto throttleOutgoingFromPlenum = plenumPressure
            - path.throttleIncidentAtPlenum;
        path.plenum.incident = plenumPressure - path.plenum.incident;

        const auto inletAreaM2 = path.hasInletDuct
            ? path.inletDuct.areaM2
            : circularAreaM2(path.geometry.bellmouthDiameterMm > 1.0
                ? path.geometry.bellmouthDiameterMm
                : path.geometry.throttleDiameterMm);
        const auto inletAdmittance = static_cast<float>(inletAreaM2 / (rho * c));
        const auto inletIncident = path.hasInletDuct
            ? path.inletIncidentAtAirbox : path.radiationIncidentAtAirbox;
        weighted = static_cast<double>(path.throttleAdmittanceM3PerPaSecond)
                * path.throttleIncidentAtAirbox
            + static_cast<double>(inletAdmittance) * inletIncident
            + static_cast<double>(path.airbox.admittanceM3PerPaSecond)
                * path.airbox.incident;
        admittance = path.throttleAdmittanceM3PerPaSecond + inletAdmittance
            + path.airbox.admittanceM3PerPaSecond;
        const auto airboxPressure = admittance > 1.0e-15
            ? static_cast<float>(2.0 * weighted / admittance) : 0.0F;
        impl_->diagnostics.airboxPressurePa = std::max(
            impl_->diagnostics.airboxPressurePa, std::abs(airboxPressure));
        const auto throttleOutgoingFromAirbox = airboxPressure
            - path.throttleIncidentAtAirbox;
        if (path.hasInletDuct)
            path.inletOutgoingAtAirbox = airboxPressure - path.inletIncidentAtAirbox;
        else {
            const auto towardMouth = airboxPressure - path.radiationIncidentAtAirbox;
            const auto radiation = path.radiation.process(towardMouth);
            impl_->diagnostics.mouthPressurePa = std::max(
                impl_->diagnostics.mouthPressurePa, std::abs(towardMouth));
            impl_->diagnostics.radiatedPressurePa = std::max(
                impl_->diagnostics.radiatedPressurePa,
                static_cast<float>(std::abs(radiation.farFieldPressurePa)));
            path.radiationIncidentAtAirbox = static_cast<float>(
                radiation.reflectedPressurePa);
            const auto observed = path.observer.process(
                static_cast<float>(radiation.farFieldPressurePa));
            result[pathIndex].leftPa += observed.leftPa;
            result[pathIndex].rightPa += observed.rightPa;
        }
        if (path.airbox.admittanceM3PerPaSecond > 0.0F)
            path.airbox.incident = airboxPressure - path.airbox.incident;
        else
            path.airbox.incident = 0.0F;
        // Cross the two zero-length throttle characteristics. This is the
        // causal one-sample storage of the variable-admittance two-port.
        path.throttleIncidentAtAirbox = throttleOutgoingFromPlenum;
        path.throttleIncidentAtPlenum = throttleOutgoingFromAirbox;
    }

    for (auto& runner : impl_->runners) {
        const auto cylinderIndex = runner.cylinderIndex;
        const auto boundary = cylinderIndex < cylinders.size()
            ? cylinders[cylinderIndex] : CylinderBoundary {};
        auto sourcePressurePa = 0.0F;
        if (boundary.physical && std::isfinite(boundary.massFlowKgPerSecond)
            && boundary.densityKgPerM3 > 0.0F && boundary.soundSpeedMps > 0.0F) {
            // Positive solver intake flow leaves the runner. From the acoustic
            // line's outward convention it is therefore a negative source.
            const auto signedFlow = -boundary.massFlowKgPerSecond;
            if (!runner.meanInitialised) {
                runner.meanMassFlowKgPerSecond = signedFlow;
                runner.meanInitialised = true;
            } else {
                runner.meanMassFlowKgPerSecond += impl_->meanFlowCoefficient
                    * (signedFlow - runner.meanMassFlowKgPerSecond);
            }
            const auto perturbationKgPerSecond = signedFlow
                - runner.meanMassFlowKgPerSecond;
            const auto impedance = boundary.densityKgPerM3
                * boundary.soundSpeedMps
                / static_cast<float>(runner.duct.inletAreaM2);
            // The valve-flow telemetry is a Norton source located between the
            // cylinder control volume and the acoustic runner.  Its volume
            // velocity launches two characteristic partners; only the runner-
            // travelling half belongs in this one-way acoustic network.  Using
            // Zc*U here injected the complete two-sided source into the runner
            // and then reflected it at the valve a second time.  Besides being
            // non-passive, that doubled every intake pulse before the plenum
            // could distribute its energy and drove the monitor limiter on the
            // catalogue engines.  This is the same characteristic split used
            // by ValveFlowAcousticSource on the exhaust boundary.
            sourcePressurePa = 0.5F * impedance * perturbationKgPerSecond
                / boundary.densityKgPerM3;
            impl_->diagnostics.sourcePressurePa = std::max(
                impl_->diagnostics.sourcePressurePa, std::abs(sourcePressurePa));
            const auto acousticVolumeVelocity = impedance > 0.0F
                ? (runner.incidentAtValve - runner.reflectionState.previousOutput)
                    / impedance : 0.0F;
            const auto coefficients = ValvePortTermination::compute(
                boundary.conductanceAreaM2, signedFlow,
                acousticVolumeVelocity, boundary.densityKgPerM3,
                impedance, impl_->sampleRateHz);
            runner.outgoingAtValve = sourcePressurePa
                + ValvePortTermination::process(coefficients,
                    runner.reflectionState, runner.incidentAtValve);
        } else {
            runner.outgoingAtValve = runner.incidentAtValve;
        }
    }

    for (std::size_t pathIndex = 0; pathIndex < impl_->paths.size(); ++pathIndex) {
        auto& path = impl_->paths[pathIndex];
        if (path.hasInletDuct) {
            const auto radiation = path.radiation.process(path.inletIncidentAtMouth);
            impl_->diagnostics.mouthPressurePa = std::max(
                impl_->diagnostics.mouthPressurePa,
                std::abs(path.inletIncidentAtMouth));
            impl_->diagnostics.radiatedPressurePa = std::max(
                impl_->diagnostics.radiatedPressurePa,
                static_cast<float>(std::abs(radiation.farFieldPressurePa)));
            path.inletOutgoingAtMouth = static_cast<float>(
                radiation.reflectedPressurePa);
            const auto observed = path.observer.process(
                static_cast<float>(radiation.farFieldPressurePa));
            result[pathIndex].leftPa += observed.leftPa;
            result[pathIndex].rightPa += observed.rightPa;
            auto& duct = path.inletDuct;
            duct.forward[duct.write] = std::isfinite(path.inletOutgoingAtMouth)
                ? path.inletOutgoingAtMouth : 0.0F;
            duct.reverse[duct.write] = std::isfinite(path.inletOutgoingAtAirbox)
                ? path.inletOutgoingAtAirbox : 0.0F;
            duct.write = (duct.write + 1U) & duct.mask;
        }
    }
    for (auto& runner : impl_->runners) {
        auto& duct = runner.duct;
        duct.forward[duct.write] = std::isfinite(runner.outgoingAtValve)
            ? runner.outgoingAtValve : 0.0F;
        duct.reverse[duct.write] = std::isfinite(runner.outgoingAtPlenum)
            ? runner.outgoingAtPlenum : 0.0F;
        duct.write = (duct.write + 1U) & duct.mask;
    }
    return result;
}

bool AcousticIntakeNetwork::valid() const noexcept {
    return impl_ && impl_->configured;
}

std::size_t AcousticIntakeNetwork::runnerCount() const noexcept {
    return impl_ ? impl_->runners.size() : 0U;
}

std::size_t AcousticIntakeNetwork::pathCount() const noexcept {
    return impl_ ? impl_->paths.size() : 0U;
}

AcousticIntakeNetwork::Diagnostics
AcousticIntakeNetwork::diagnostics() const noexcept {
    return impl_ ? impl_->diagnostics : Diagnostics {};
}

} // namespace enginelab
