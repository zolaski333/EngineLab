#include <enginelab/audio/AcousticExhaustNetwork.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/foundation/EngineTypes.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstddef>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <numbers>
#include <span>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace {

constexpr double sampleRateHz = 48'000.0;
constexpr float gasDensityKgPerM3 = 0.55F;
constexpr float soundSpeedMps = 535.0F;
constexpr std::size_t renderSampleCount = 32'768;
constexpr std::size_t sourceSampleCount = 33;
constexpr std::array<double, 12> probeFrequenciesHz {
    125.0, 250.0, 400.0, 630.0, 800.0, 1'000.0,
    1'250.0, 1'600.0, 2'500.0, 4'000.0, 6'300.0, 8'000.0
};

enum class FixtureTopology : std::uint8_t {
    straight,
    longStraight,
    expansionChamber,
    smoothTaper,
    parallelTaperBudget,
    sideBranch250Hz,
    sideBranch250HzWithCavity,
    sideBranch500HzOverride,
    catalystBypass,
    catalystMonolith,
    perforatedCoreSmallCan,
    perforatedCoreLargeCan,
    asymmetricSplit,
    compactSplitSmallVolume,
    compactSplitLargeVolume,
};

[[noreturn]] void fail(std::string message) {
    throw std::runtime_error(std::move(message));
}

void require(bool condition, std::string message) {
    if (!condition) fail(std::move(message));
}

[[nodiscard]] enginelab::ExhaustComponentConfig component(
    std::uint32_t id, enginelab::ExhaustComponentType type,
    double lengthMm, double diameterMm) {
    enginelab::ExhaustComponentConfig result;
    result.id = id;
    result.type = type;
    result.lengthMm = lengthMm;
    result.diameterMm = diameterMm;
    return result;
}

/** Build deliberately small authored DAGs around one and the same cylinder.
 *
 * There is no running engine, catalogue voicing, random jet source or reference
 * recording in this fixture. Geometry is the only independent variable.
 */
[[nodiscard]] enginelab::EngineConfig makeFixture(FixtureTopology topology) {
    auto config = enginelab::makeDefaultInlineFour();
    config.name = "Deterministic exhaust transfer fixture";
    config.cylinders.resize(1);
    const auto cylinderId = config.cylinders.front().id;
    config.exhaustPaths.resize(1);
    auto& path = config.exhaustPaths.front();
    path.cylinderIds = { cylinderId };
    path.inheritsGlobalGeometry = false;
    path.audioVolume = 1.0;

    // Freeze the observation scene as well as the source. This keeps a changed
    // transfer attributable to the authored DAG, never to a catalogue camera.
    config.acousticObserver.leftMicrophoneM = { -0.08, 1.0, 0.05 };
    config.acousticObserver.rightMicrophoneM = { 0.08, 1.0, 0.05 };
    config.acousticObserver.listeningDistanceM = 1.0;
    config.acousticObserver.soundSpeedMps = 343.0;

    enginelab::ExhaustNetworkConfig network;
    switch (topology) {
    case FixtureTopology::straight: {
        network.components.push_back(component(
            100, enginelab::ExhaustComponentType::pipe, 750.0, 42.0));
        auto outlet = component(
            200, enginelab::ExhaustComponentType::outlet, 750.0, 42.0);
        outlet.acousticPositionM = { 0.0, 0.0, 0.0 };
        network.components.push_back(outlet);
        network.cylinderConnections.push_back({ cylinderId, 100 });
        network.connections.push_back({ 100, 200 });
        break;
    }
    case FixtureTopology::longStraight: {
        // Exactly 600 mm longer than `straight`; all other dimensions match.
        network.components.push_back(component(
            100, enginelab::ExhaustComponentType::pipe, 1'350.0, 42.0));
        auto outlet = component(
            200, enginelab::ExhaustComponentType::outlet, 750.0, 42.0);
        outlet.acousticPositionM = { 0.0, 0.0, 0.0 };
        network.components.push_back(outlet);
        network.cylinderConnections.push_back({ cylinderId, 100 });
        network.connections.push_back({ 100, 200 });
        break;
    }
    case FixtureTopology::expansionChamber: {
        // Same total centreline length as `straight`; only the two area steps
        // and their finite 500 mm separation are new.
        network.components.push_back(component(
            100, enginelab::ExhaustComponentType::pipe, 500.0, 42.0));
        network.components.push_back(component(
            200, enginelab::ExhaustComponentType::pipe, 500.0, 105.0));
        auto outlet = component(
            300, enginelab::ExhaustComponentType::outlet, 500.0, 42.0);
        outlet.acousticPositionM = { 0.0, 0.0, 0.0 };
        network.components.push_back(outlet);
        network.cylinderConnections.push_back({ cylinderId, 100 });
        network.connections.push_back({ 100, 200 });
        network.connections.push_back({ 200, 300 });
        break;
    }
    case FixtureTopology::smoothTaper: {
        auto taper = component(
            100, enginelab::ExhaustComponentType::pipe, 750.0, 42.0);
        taper.outletDiameterMm = 84.0;
        network.components.push_back(taper);
        auto outlet = component(
            200, enginelab::ExhaustComponentType::outlet, 750.0, 84.0);
        outlet.acousticPositionM = { 0.0, 0.0, 0.0 };
        network.components.push_back(outlet);
        network.cylinderConnections.push_back({ cylinderId, 100 });
        network.connections.push_back({ 100, 200 });
        break;
    }
    case FixtureTopology::parallelTaperBudget: {
        network.components.push_back(component(
            100, enginelab::ExhaustComponentType::pipe, 500.0, 52.0));
        network.components.push_back(component(
            200, enginelab::ExhaustComponentType::splitter, 0.0, 52.0));
        network.cylinderConnections.push_back({ cylinderId, 100 });
        network.connections.push_back({ 100, 200 });
        for (std::uint32_t index = 0; index < 12U; ++index) {
            auto outlet = component(300U + index,
                enginelab::ExhaustComponentType::outlet, 150.0, 52.0);
            outlet.outletDiameterMm = 140.0;
            outlet.acousticPositionM = {
                -0.55 + 0.10 * static_cast<double>(index), 0.0, 0.0 };
            network.components.push_back(outlet);
            network.connections.push_back({ 200, 300U + index });
        }
        break;
    }
    case FixtureTopology::sideBranch250Hz:
    case FixtureTopology::sideBranch250HzWithCavity:
    case FixtureTopology::sideBranch500HzOverride: {
        network.components.push_back(component(
            100, enginelab::ExhaustComponentType::pipe, 750.0, 42.0));
        auto outlet = component(
            200, enginelab::ExhaustComponentType::outlet, 750.0, 42.0);
        outlet.acousticPositionM = { 0.0, 0.0, 0.0 };
        network.components.push_back(outlet);
        auto resonator = component(
            300, enginelab::ExhaustComponentType::resonator, 535.0, 28.0);
        resonator.volumeLitres =
            topology == FixtureTopology::sideBranch250HzWithCavity
            ? 0.35 : 0.0;
        resonator.resonanceHz =
            topology == FixtureTopology::sideBranch500HzOverride
            ? 500.0 : 0.0;
        network.components.push_back(resonator);
        network.cylinderConnections.push_back({ cylinderId, 100 });
        network.connections.push_back({ 100, 200 });
        network.connections.push_back({ 100, 300 });
        break;
    }
    case FixtureTopology::catalystBypass:
    case FixtureTopology::catalystMonolith: {
        network.components.push_back(component(
            100, enginelab::ExhaustComponentType::pipe, 600.0, 56.0));
        auto catalyst = component(
            200, enginelab::ExhaustComponentType::catalyst, 300.0, 56.0);
        catalyst.restriction = 0.18;
        if (topology == FixtureTopology::catalystMonolith) {
            catalyst.catalystCellDensityCpsi = 400.0;
            catalyst.catalystOpenAreaRatio = 0.80;
            catalyst.catalystSubstrateVolumetricHeatCapacityJPerM3K =
                2'000'000.0;
        }
        network.components.push_back(catalyst);
        auto outlet = component(
            300, enginelab::ExhaustComponentType::outlet, 600.0, 56.0);
        outlet.acousticPositionM = { 0.0, 0.0, 0.0 };
        network.components.push_back(outlet);
        network.cylinderConnections.push_back({ cylinderId, 100 });
        network.connections.push_back({ 100, 200 });
        network.connections.push_back({ 200, 300 });
        break;
    }
    case FixtureTopology::perforatedCoreSmallCan:
    case FixtureTopology::perforatedCoreLargeCan: {
        network.components.push_back(component(
            100, enginelab::ExhaustComponentType::pipe, 500.0, 62.0));
        auto muffler = component(
            200, enginelab::ExhaustComponentType::muffler, 500.0, 62.0);
        muffler.volumeLitres =
            topology == FixtureTopology::perforatedCoreSmallCan ? 4.0 : 8.0;
        muffler.packingFlowResistivityPaSPerM2 = 24'000.0;
        muffler.packingThicknessMm = 35.0;
        muffler.perforatedOpenAreaRatio = 0.28;
        network.components.push_back(muffler);
        auto outlet = component(
            300, enginelab::ExhaustComponentType::outlet, 500.0, 62.0);
        outlet.acousticPositionM = { 0.0, 0.0, 0.0 };
        network.components.push_back(outlet);
        network.cylinderConnections.push_back({ cylinderId, 100 });
        network.connections.push_back({ 100, 200 });
        network.connections.push_back({ 200, 300 });
        break;
    }
    case FixtureTopology::asymmetricSplit:
    case FixtureTopology::compactSplitSmallVolume:
    case FixtureTopology::compactSplitLargeVolume: {
        network.components.push_back(component(
            100, enginelab::ExhaustComponentType::pipe, 500.0, 42.0));
        auto splitter = component(
            200, enginelab::ExhaustComponentType::splitter, 0.0, 42.0);
        if (topology == FixtureTopology::compactSplitSmallVolume)
            splitter.volumeLitres = 0.25;
        else if (topology == FixtureTopology::compactSplitLargeVolume)
            splitter.volumeLitres = 2.0;
        network.components.push_back(splitter);
        auto leftOutlet = component(
            300, enginelab::ExhaustComponentType::outlet, 1'000.0, 42.0);
        leftOutlet.acousticPositionM = { -0.20, 0.0, 0.0 };
        auto rightOutlet = component(
            301, enginelab::ExhaustComponentType::outlet, 1'250.0, 42.0);
        rightOutlet.acousticPositionM = { 0.20, 0.0, 0.0 };
        network.components.push_back(leftOutlet);
        network.components.push_back(rightOutlet);
        network.cylinderConnections.push_back({ cylinderId, 100 });
        network.connections.push_back({ 100, 200 });
        network.connections.push_back({ 200, 300 });
        network.connections.push_back({ 200, 301 });
        break;
    }
    }
    path.network = std::move(network);
    enginelab::normaliseEngineConfig(config);
    return config;
}

[[nodiscard]] std::array<float, renderSampleCount> fixedExcitation() {
    std::array<float, renderSampleCount> result {};
    // A short, exactly repeatable half-sine has finite bandwidth without the
    // Nyquist-only emphasis of a one-sample impulse. Its peak is one pascal, so
    // the nonlinear mouth and duct laws remain in their small-signal region.
    for (std::size_t sample = 0; sample < sourceSampleCount; ++sample) {
        result[sample] = static_cast<float>(std::sin(
            std::numbers::pi * static_cast<double>(sample + 1U)
                / static_cast<double>(sourceSampleCount + 1U)));
    }
    return result;
}

struct TransferResult final {
    std::vector<float> left;
    std::vector<float> right;
    std::size_t firstArrivalSample { renderSampleCount };
    double sourceSquareSum {};
    double outputSquareSum {};
    double lateSquareSum {};
    double outputPeakPa {};
    bool finite { true };
    std::size_t acousticDuctCount {};
    std::array<double, probeFrequenciesHz.size()> shapeDb {};
};

[[nodiscard]] double stereoSquareSum(
    std::span<const float> left, std::span<const float> right,
    std::size_t begin, std::size_t end) {
    begin = std::min(begin, std::min(left.size(), right.size()));
    end = std::min(end, std::min(left.size(), right.size()));
    auto result = 0.0;
    for (auto sample = begin; sample < end; ++sample) {
        result += static_cast<double>(left[sample]) * left[sample]
            + static_cast<double>(right[sample]) * right[sample];
    }
    return result;
}

[[nodiscard]] std::array<double, probeFrequenciesHz.size()> transferShapeDb(
    std::span<const float> left, std::span<const float> right) {
    std::array<double, probeFrequenciesHz.size()> magnitudes {};
    auto maximumMagnitude = 0.0;
    for (std::size_t probe = 0; probe < probeFrequenciesHz.size(); ++probe) {
        std::complex<double> leftSpectrum {};
        std::complex<double> rightSpectrum {};
        const auto angularStep = -2.0 * std::numbers::pi
            * probeFrequenciesHz[probe] / sampleRateHz;
        for (std::size_t sample = 0; sample < left.size(); ++sample) {
            const auto phase = angularStep * static_cast<double>(sample);
            const auto rotation = std::exp(std::complex<double> { 0.0, phase });
            leftSpectrum += static_cast<double>(left[sample]) * rotation;
            rightSpectrum += static_cast<double>(right[sample]) * rotation;
        }
        magnitudes[probe] = std::sqrt(
            std::norm(leftSpectrum) + std::norm(rightSpectrum));
        maximumMagnitude = std::max(maximumMagnitude, magnitudes[probe]);
    }
    require(maximumMagnitude > 0.0,
        "the fixed excitation produced an empty transfer function");

    std::array<double, probeFrequenciesHz.size()> result {};
    auto meanDb = 0.0;
    const auto floor = maximumMagnitude * 1.0e-9;
    for (std::size_t probe = 0; probe < result.size(); ++probe) {
        result[probe] = 20.0 * std::log10(std::max(magnitudes[probe], floor));
        meanDb += result[probe];
    }
    meanDb /= static_cast<double>(result.size());
    for (auto& value : result) value -= meanDb;
    return result;
}

[[nodiscard]] TransferResult render(
    FixtureTopology topology,
    std::span<const float, renderSampleCount> excitation) {
    auto config = makeFixture(topology);
    const auto graph = enginelab::ExhaustGraph::makeForEngine(config);
    require(graph.diagnostics().empty(),
        "the transfer fixture must compile without graph fallbacks");
    const std::array<std::uint32_t, 1> cylinderIds {
        config.cylinders.front().id };
    enginelab::AcousticExhaustNetwork network(graph, cylinderIds);
    require(network.valid() && network.prepare(sampleRateHz),
        "the transfer fixture must compile and prepare for audio");
    network.setOutletJetNoiseEnabled(false);
    require(!network.outletJetNoiseEnabled(),
        "the fixed-source oracle must not contain a random outlet source");

    const std::array<enginelab::AcousticExhaustNetwork::Medium, 1> medium {{
        { gasDensityKgPerM3, soundSpeedMps }
    }};
    // A fixed positive mean flow leaves by the outlet; it enters through an
    // open exhaust valve, whose orifice is the termination's loss
    // (ValvePortTermination). The outlet's pressure reflection keeps
    // Levine-Schwinger's |R| (Munt with a Kutta condition, journal
    // 2026-11-16). Jet noise remains explicitly disabled above.
    const std::array<float, 1> meanFlowKgPerSecond {{ 0.060F }};
    network.beginBlock(medium, 1.0, meanFlowKgPerSecond);

    TransferResult result;
    result.acousticDuctCount = network.ductCount();
    result.left.resize(renderSampleCount);
    result.right.resize(renderSampleCount);
    std::array<float, 1> source {};
    // The valve that passes that flow: a 30 mm head at 8 mm lift, Cd 0.6
    // (CdA 4.5 cm2), into the 42 mm primary.
    constexpr float primaryAreaM2 = static_cast<float>(
        std::numbers::pi * 0.021 * 0.021);
    std::array<enginelab::AcousticExhaustNetwork::CylinderBoundary, 1>
        cylinderBoundary {{ { 4.5e-4F, meanFlowKgPerSecond[0], gasDensityKgPerM3,
            gasDensityKgPerM3 * soundSpeedMps / primaryAreaM2, true } }};
    for (std::size_t sample = 0; sample < renderSampleCount; ++sample) {
        source[0] = excitation[sample];
        result.sourceSquareSum += static_cast<double>(source[0]) * source[0];
        const auto output = network.process(source, cylinderBoundary, 1.0F);
        result.left[sample] = output[0].leftPa;
        result.right[sample] = output[0].rightPa;
        result.finite = result.finite
            && std::isfinite(result.left[sample])
            && std::isfinite(result.right[sample]);
        result.outputPeakPa = std::max({ result.outputPeakPa,
            std::abs(static_cast<double>(result.left[sample])),
            std::abs(static_cast<double>(result.right[sample])) });
        if (result.firstArrivalSample == renderSampleCount
            && (result.left[sample] != 0.0F || result.right[sample] != 0.0F))
            result.firstArrivalSample = sample;
    }
    result.outputSquareSum = stereoSquareSum(
        result.left, result.right, 0, renderSampleCount);
    result.lateSquareSum = stereoSquareSum(
        result.left, result.right, renderSampleCount - 4'096,
        renderSampleCount);
    result.shapeDb = transferShapeDb(result.left, result.right);
    return result;
}

[[nodiscard]] double shapeDistanceDb(
    const TransferResult& left, const TransferResult& right) {
    auto squareSum = 0.0;
    for (std::size_t probe = 0; probe < probeFrequenciesHz.size(); ++probe) {
        const auto delta = left.shapeDb[probe] - right.shapeDb[probe];
        squareSum += delta * delta;
    }
    return std::sqrt(squareSum
        / static_cast<double>(probeFrequenciesHz.size()));
}

void requireBoundedPassiveResponse(
    std::string_view name, const TransferResult& result) {
    require(result.finite, std::string(name) + " produced a non-finite sample");
    require(result.firstArrivalSample < renderSampleCount,
        std::string(name) + " did not transport the fixed source to an outlet");
    require(result.outputSquareSum > std::numeric_limits<double>::min(),
        std::string(name) + " produced no measurable radiated energy");

    // This is intentionally an energy-bounded stability contract, not a claim
    // that Pa^2 at a microphone is the acoustic joule flux at the cylinder
    // boundary: those surfaces have different impedances and areas. A passive
    // network may resonate, but a one-pascal finite excitation must neither
    // grow without bound nor leave a non-decaying tail. The generous absolute
    // ceiling catches positive-feedback/scattering-sign regressions without
    // calibrating the test onto today's insertion loss.
    require(result.outputSquareSum < 16.0 * result.sourceSquareSum,
        std::string(name) + " exceeded the bounded response energy ceiling");
    require(result.outputPeakPa < 4.0,
        std::string(name) + " amplified a one-pascal source beyond the passive ceiling");
    require(result.lateSquareSum < result.outputSquareSum * 0.01,
        std::string(name) + " did not dissipate its impulse-response tail");
}

void transferOracleRegression() {
    const auto excitation = fixedExcitation();
    const auto straight = render(FixtureTopology::straight, excitation);
    const auto straightRepeat = render(FixtureTopology::straight, excitation);
    const auto longStraight = render(FixtureTopology::longStraight, excitation);
    const auto chamber = render(FixtureTopology::expansionChamber, excitation);
    const auto taper = render(FixtureTopology::smoothTaper, excitation);
    const auto parallelTapers = render(
        FixtureTopology::parallelTaperBudget, excitation);
    const auto resonator250 = render(
        FixtureTopology::sideBranch250Hz, excitation);
    const auto resonator250WithCavity = render(
        FixtureTopology::sideBranch250HzWithCavity, excitation);
    const auto resonator500 = render(
        FixtureTopology::sideBranch500HzOverride, excitation);
    const auto catalystBypass = render(
        FixtureTopology::catalystBypass, excitation);
    const auto catalystMonolith = render(
        FixtureTopology::catalystMonolith, excitation);
    const auto perforatedCoreSmall = render(
        FixtureTopology::perforatedCoreSmallCan, excitation);
    const auto perforatedCoreLarge = render(
        FixtureTopology::perforatedCoreLargeCan, excitation);
    const auto split = render(FixtureTopology::asymmetricSplit, excitation);
    const auto compactSmall = render(
        FixtureTopology::compactSplitSmallVolume, excitation);
    const auto compactLarge = render(
        FixtureTopology::compactSplitLargeVolume, excitation);

    // The straight-through graph is the deterministic bypass/control. Equal
    // topology, medium, observer and samples must remain bit-exact across fresh
    // allocations; a hidden RNG or uninitialised state fails here immediately.
    require(straight.left == straightRepeat.left
            && straight.right == straightRepeat.right,
        "the straight-through bypass must render sample-exactly on repeat");

    requireBoundedPassiveResponse("straight", straight);
    requireBoundedPassiveResponse("long straight", longStraight);
    requireBoundedPassiveResponse("expansion chamber", chamber);
    requireBoundedPassiveResponse("smooth taper", taper);
    requireBoundedPassiveResponse("250 Hz side branch", resonator250);
    requireBoundedPassiveResponse(
        "250 Hz side branch with cavity", resonator250WithCavity);
    requireBoundedPassiveResponse("500 Hz side branch", resonator500);
    requireBoundedPassiveResponse("catalyst bypass", catalystBypass);
    requireBoundedPassiveResponse("catalyst monolith", catalystMonolith);
    requireBoundedPassiveResponse(
        "perforated core, 4 litre can", perforatedCoreSmall);
    requireBoundedPassiveResponse(
        "perforated core, 8 litre can", perforatedCoreLarge);
    requireBoundedPassiveResponse("asymmetric split", split);
    requireBoundedPassiveResponse("compact split, 0.25 litre", compactSmall);
    requireBoundedPassiveResponse("compact split, 2.0 litres", compactLarge);

    constexpr double addedLengthM = 0.600;
    const auto expectedAddedSamples = addedLengthM
        / static_cast<double>(soundSpeedMps) * sampleRateHz;
    const auto measuredAddedSamples = static_cast<double>(
        longStraight.firstArrivalSample) - static_cast<double>(straight.firstArrivalSample);
    require(std::abs(measuredAddedSamples - expectedAddedSamples) < 3.0,
        "a 600 mm geometry change must add its physical propagation delay");
    require(taper.acousticDuctCount > straight.acousticDuctCount,
        "a finite taper must contain distributed acoustic sections, not only endpoint areas");
    const auto taperExtraSections = taper.acousticDuctCount
        - straight.acousticDuctCount;
    // Cascaded linear fractional delays keep the summed group delay, but each
    // section contributes one earlier non-zero interpolation tap. The support
    // onset may therefore move by at most N-1 samples without shortening the
    // physical centreline delay.
    require(std::abs(static_cast<double>(taper.firstArrivalSample)
            - static_cast<double>(straight.firstArrivalSample))
            <= static_cast<double>(taperExtraSections),
        "taper sectioning must preserve the authored centreline delay (straight="
            + std::to_string(straight.firstArrivalSample) + ", taper="
            + std::to_string(taper.firstArrivalSample) + ")");
    require(taper.acousticDuctCount <= straight.acousticDuctCount + 3U,
        "the bounded taper approximation allocated more than four sections");
    // One core duct plus twelve equivalent outlets. The graph-wide budget must
    // allocate one complete refinement round (two sections per outlet), never
    // refine only the first branches and destroy bank symmetry.
    require(parallelTapers.acousticDuctCount == 25U,
        "parallel tapers must share the bounded refinement budget symmetrically");
    require(resonator250.acousticDuctCount == straight.acousticDuctCount + 1U
            && resonator250WithCavity.acousticDuctCount
                == resonator250.acousticDuctCount
            && resonator500.acousticDuctCount == resonator250.acousticDuctCount,
        "a terminal resonator must add one bounded acoustic branch and no mean-flow route");
    require(catalystBypass.acousticDuctCount == catalystMonolith.acousticDuctCount,
        "a homogenised catalyst must not add acoustic delay lines per substrate cell");
    require(perforatedCoreSmall.acousticDuctCount
            == perforatedCoreLarge.acousticDuctCount,
        "outer-can volume must not change the bounded acoustic work");

    const auto chamberShapeDistanceDb = shapeDistanceDb(straight, chamber);
    const auto taperShapeDistanceDb = shapeDistanceDb(straight, taper);
    const auto resonatorTuningShapeDistanceDb = shapeDistanceDb(
        resonator250, resonator500);
    const auto resonatorCavityShapeDistanceDb = shapeDistanceDb(
        resonator250, resonator250WithCavity);
    const auto catalystShapeDistanceDb = shapeDistanceDb(
        catalystBypass, catalystMonolith);
    const auto perforatedCanShapeDistanceDb = shapeDistanceDb(
        perforatedCoreSmall, perforatedCoreLarge);
    const auto splitShapeDistanceDb = shapeDistanceDb(straight, split);
    const auto junctionVolumeShapeDistanceDb = shapeDistanceDb(
        compactSmall, compactLarge);
    // These low thresholds reject a graph that has become decorative while
    // leaving the actual transfer shape unconstrained for future better models.
    require(chamberShapeDistanceDb > 0.25,
        "the expansion ratio did not change the fixed-source transfer shape");
    require(taperShapeDistanceDb > 0.25,
        "the distributed taper did not change the fixed-source transfer shape");
    require(resonatorTuningShapeDistanceDb > 0.25,
        "resonanceHz did not retune the passive side-branch transfer");
    require(resonatorCavityShapeDistanceDb > 0.25,
        "terminal cavity volume did not change the passive side-branch transfer");
    require(catalystShapeDistanceDb > 0.25,
        "authored catalyst cell geometry did not change the passive transfer");
    require(perforatedCanShapeDistanceDb > 0.25,
        "changing only annular can volume did not change the perforated-core transfer");
    require(splitShapeDistanceDb > 0.25,
        "the asymmetric split did not change the fixed-source transfer shape");
    require(junctionVolumeShapeDistanceDb > 0.25,
        "changing only the compact junction volume did not change its acoustic transfer");

    std::cout << std::fixed << std::setprecision(3)
              << "Exhaust transfer oracle\n"
              << "  straight onset=" << straight.firstArrivalSample
              << " energy_ratio=" << std::scientific
              << straight.outputSquareSum / straight.sourceSquareSum
              << " late_share="
              << straight.lateSquareSum / straight.outputSquareSum
              << std::fixed << '\n'
              << "  long onset=" << longStraight.firstArrivalSample
              << " added_samples=" << measuredAddedSamples
              << " expected=" << expectedAddedSamples << '\n'
              << "  chamber shape_delta=" << chamberShapeDistanceDb << " dB\n"
              << "  taper shape_delta=" << taperShapeDistanceDb
              << " dB sections=" << taper.acousticDuctCount - 1U << '\n'
              << "  resonator tuning shape_delta="
              << resonatorTuningShapeDistanceDb << " dB\n"
              << "  resonator cavity shape_delta="
              << resonatorCavityShapeDistanceDb << " dB\n"
              << "  catalyst monolith shape_delta="
              << catalystShapeDistanceDb << " dB\n"
              << "  perforated can volume shape_delta="
              << perforatedCanShapeDistanceDb << " dB\n"
              << "  split shape_delta=" << splitShapeDistanceDb << " dB\n"
              << "  junction volume shape_delta="
              << junctionVolumeShapeDistanceDb << " dB\n";
}

/** A cone must carry no more power than it is given.
 *
 * A 27 mm pipe, a 100 mm cone to 66 mm, a 66 mm pipe and its open end. Far
 * below the first resonance what lies downstream of the cone's mouth is a
 * lumped inertance, rho times the sum of L/S plus the end correction, and the
 * volume flow through the end radiates as a monopole: the transfer from the
 * mouth to a microphone at r is rho0 / (4 pi r rho sum(L/S)), whatever drives
 * it (a plane-wave transfer matrix agrees within 0.15 dB at 40 Hz). Each
 * section of a cone once scattered with its inlet area at one end and its
 * outlet area at the other, so a forward wave gained power across it: +3.3 dB
 * here, +9 dB on the ACME single's radiated transfer (journal 2026-11-15).
 */
void taperPassivityRegression() {
    auto config = makeFixture(FixtureTopology::straight);
    // Microphones exactly 1 m down the outlet's axis, as authored.
    config.acousticObserver.leftMicrophoneM = { 0.0, 1.0, 0.0 };
    config.acousticObserver.rightMicrophoneM = { 0.0, 1.0, 0.0 };
    config.acousticObserver.listeningDistanceM = 0.0;
    enginelab::ExhaustNetworkConfig network;
    const auto cylinderId = config.cylinders.front().id;
    network.components.push_back(component(
        100, enginelab::ExhaustComponentType::pipe, 200.0, 27.0));
    auto cone = component(101, enginelab::ExhaustComponentType::pipe, 100.0, 27.0);
    cone.outletDiameterMm = 66.0;
    network.components.push_back(cone);
    network.components.push_back(component(
        102, enginelab::ExhaustComponentType::pipe, 300.0, 66.0));
    auto outlet = component(200, enginelab::ExhaustComponentType::outlet, 50.0, 66.0);
    outlet.acousticPositionM = { 0.0, 0.0, 0.0 };
    network.components.push_back(outlet);
    network.cylinderConnections.push_back({ cylinderId, 100 });
    network.connections.push_back({ 100, 101 });
    network.connections.push_back({ 101, 102 });
    network.connections.push_back({ 102, 200 });
    config.exhaustPaths.front().network = network;
    const auto graph = enginelab::ExhaustGraph::makeForEngine(config);
    require(graph.diagnostics().empty(), "the cone fixture must compile without fallbacks");
    const std::array<std::uint32_t, 1> cylinderIds { cylinderId };
    enginelab::AcousticExhaustNetwork acoustic(graph, cylinderIds);
    require(acoustic.valid() && acoustic.prepare(sampleRateHz),
        "the cone fixture must compile and prepare for audio");
    acoustic.setOutletJetNoiseEnabled(false);
    constexpr double ambientDensityKgPerM3 = 1.2;
    acoustic.setOutletFarFieldDensity(ambientDensityKgPerM3);
    auto tapped = false;
    for (const auto& node : graph.nodes()) {
        if (node.sourceComponentId != 100U) continue;
        tapped = acoustic.setPressureTap(node.id, 1.0F);
        if (tapped) break;
    }
    require(tapped, "the cone fixture's first pipe must carry a pressure tap at its end");
    const std::array<enginelab::AcousticExhaustNetwork::Medium, 1> medium {{
        { gasDensityKgPerM3, soundSpeedMps } }};
    acoustic.beginBlock(medium, 1.0);

    constexpr std::size_t samples = 96'000;
    std::vector<double> tap(samples);
    std::vector<double> microphone(samples);
    std::array<float, 1> source {};
    std::array<enginelab::AcousticExhaustNetwork::CylinderBoundary, 1> boundary {};
    for (std::size_t sample = 0; sample < samples; ++sample) {
        source[0] = sample == 0 ? 1'000.0F : 0.0F;
        const auto output = acoustic.process(source, boundary, 1.0F);
        tap[sample] = acoustic.lastTapPressurePa();
        microphone[sample] = output[0].leftPa;
    }
    const auto magnitudeAt = [](const std::vector<double>& signal, double frequencyHz) {
        std::complex<double> sum {};
        const auto step = std::polar(1.0, -2.0 * std::numbers::pi * frequencyHz / sampleRateHz);
        std::complex<double> rotation { 1.0, 0.0 };
        for (const auto value : signal) {
            sum += value * rotation;
            rotation *= step;
        }
        return std::abs(sum);
    };
    constexpr double frequencyHz = 40.0;
    const auto measuredDb = 20.0 * std::log10(
        magnitudeAt(microphone, frequencyHz) / magnitudeAt(tap, frequencyHz));
    const auto area = [](double diameterM) {
        return std::numbers::pi * diameterM * diameterM / 4.0;
    };
    const auto inertancePerDensity = 0.100 / (std::numbers::pi / 4.0 * 0.027 * 0.066)
        + (0.300 + 0.050 + 0.6133 * 0.033) / area(0.066);
    // The fixture's observer sits at its 1 m listening distance.
    const auto expectedDb = 20.0 * std::log10(ambientDensityKgPerM3
        / (4.0 * std::numbers::pi * 1.0 * gasDensityKgPerM3 * inertancePerDensity));
    std::cout << "Taper passivity: transfer at 40 Hz " << measuredDb
              << " dB, lumped inertance " << expectedDb << " dB\n";
    require(std::abs(measuredDb - expectedDb) < 0.5,
        "a cone in the audio network must transmit what its inertance passes, not gain power ("
            + std::to_string(measuredDb) + " dB against " + std::to_string(expectedDb) + " dB)");
}

/** A tap at the end of a cone reads the junction pressure the next pipe's
 *  start reads. The cone compiles to several sections of one node, and the
 *  fraction went to the first alone (audit A2-06). */
void taperTapPositionRegression() {
    auto config = makeFixture(FixtureTopology::straight);
    enginelab::ExhaustNetworkConfig network;
    const auto cylinderId = config.cylinders.front().id;
    network.components.push_back(component(
        100, enginelab::ExhaustComponentType::pipe, 200.0, 27.0));
    auto cone = component(101, enginelab::ExhaustComponentType::pipe, 400.0, 27.0);
    cone.outletDiameterMm = 80.0;
    network.components.push_back(cone);
    network.components.push_back(component(
        102, enginelab::ExhaustComponentType::pipe, 300.0, 80.0));
    auto outlet = component(200, enginelab::ExhaustComponentType::outlet, 50.0, 80.0);
    network.components.push_back(outlet);
    network.cylinderConnections.push_back({ cylinderId, 100 });
    network.connections.push_back({ 100, 101 });
    network.connections.push_back({ 101, 102 });
    network.connections.push_back({ 102, 200 });
    config.exhaustPaths.front().network = network;
    const auto graph = enginelab::ExhaustGraph::makeForEngine(config);
    require(graph.diagnostics().empty(), "the cone tap fixture must compile without fallbacks");
    const auto record = [&](std::uint32_t componentId, float fraction) {
        const std::array<std::uint32_t, 1> cylinderIds { cylinderId };
        enginelab::AcousticExhaustNetwork acoustic(graph, cylinderIds);
        require(acoustic.valid() && acoustic.prepare(sampleRateHz),
            "the cone tap fixture must compile and prepare for audio");
        acoustic.setOutletJetNoiseEnabled(false);
        auto tapped = false;
        for (const auto& node : graph.nodes()) {
            if (node.sourceComponentId != componentId) continue;
            tapped = acoustic.setPressureTap(node.id, fraction);
            if (tapped) break;
        }
        require(tapped, "the cone tap fixture carries its taps");
        const std::array<enginelab::AcousticExhaustNetwork::Medium, 1> medium {{
            { gasDensityKgPerM3, soundSpeedMps } }};
        acoustic.beginBlock(medium, 1.0);
        std::vector<double> tap(4'800);
        std::array<float, 1> source {};
        std::array<enginelab::AcousticExhaustNetwork::CylinderBoundary, 1> boundary {};
        for (std::size_t sample = 0; sample < tap.size(); ++sample) {
            source[0] = sample == 0 ? 1'000.0F : 0.0F;
            (void)acoustic.process(source, boundary, 1.0F);
            tap[sample] = acoustic.lastTapPressurePa();
        }
        return tap;
    };
    const auto coneEnd = record(101U, 1.0F);
    const auto pipeStart = record(102U, 0.0F);
    double energy = 0.0;
    double difference = 0.0;
    for (std::size_t sample = 0; sample < coneEnd.size(); ++sample) {
        energy += pipeStart[sample] * pipeStart[sample];
        difference += (coneEnd[sample] - pipeStart[sample]) * (coneEnd[sample] - pipeStart[sample]);
    }
    const auto relative = std::sqrt(difference / std::max(energy, 1.0e-30));
    std::cout << "Taper tap: cone end against next pipe start, relative difference " << relative << '\n';
    require(energy > 0.0 && relative < 1.0e-3,
        "a tap at a cone's end reads where the next pipe starts (" + std::to_string(relative) + ")");
}

} // namespace

namespace enginelab::tests {

void exhaustTransferRegression() {
    transferOracleRegression();
    taperPassivityRegression();
    taperTapPositionRegression();
}

} // namespace enginelab::tests
