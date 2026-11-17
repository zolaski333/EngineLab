// The outlets' far field below a crossover taken from the gas solver's mouth
// flow (AcousticExhaustNetwork::setOutletSolverLowBand). Off, or on with no
// solver flow named, the network is bit for bit the same; on, a silent
// network radiates the solver's low-band volume acceleration as a monopole
// and rejects its high band; a steady solver outflow, its mean, makes no
// thump when it first arrives.

#include <enginelab/audio/AcousticExhaustNetwork.hpp>
#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iostream>
#include <numbers>
#include <string>
#include <vector>

namespace {

void require(bool condition, const std::string& message) {
    if (condition) return;
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
}

constexpr double rate = 48'000.0;

enginelab::EngineConfig stockEngine() {
    const auto catalog = enginelab::loadEngineCatalog(ENGINELAB_CATALOG_ROOT);
    for (const auto& entry : catalog.entries)
        if (entry.config.name.find("K20") != std::string::npos) return entry.config;
    require(false, "the K20A is in the catalogue");
    return {};
}

/** Solver flow each sample: what every outlet is told, or nothing. */
enum class Feed { none, sine, steady };

struct Run final {
    std::vector<float> left;
};

Run render(const enginelab::EngineConfig& config, double crossoverHz, bool pulses,
           Feed feed, double frequencyHz, double amplitudeM3PerS, double seconds) {
    const auto graph = enginelab::ExhaustGraph::makeForEngine(config);
    std::vector<std::uint32_t> ids;
    for (const auto& cylinder : config.cylinders) ids.push_back(cylinder.id);
    enginelab::AcousticExhaustNetwork network(graph, ids);
    require(network.valid() && network.prepare(rate), "the audio network compiles");
    network.setOutletJetNoiseEnabled(false);
    network.setOutletFarFieldDensity(1.2);
    if (crossoverHz > 0.0) network.setOutletSolverLowBand(crossoverHz);
    std::vector<enginelab::AcousticExhaustNetwork::Medium> media(
        config.exhaustPaths.size(), { 0.55F, 560.0F });
    std::vector<float> sources(ids.size(), 0.0F);
    std::vector<enginelab::AcousticExhaustNetwork::CylinderBoundary> boundaries(ids.size());
    std::vector<std::uint32_t> nodes;
    std::vector<std::uint8_t> paths;
    for (std::size_t outlet = 0; outlet < network.outletCount(); ++outlet) {
        nodes.push_back(network.outletNodeId(outlet));
        paths.push_back(static_cast<std::uint8_t>(network.outletPathIndex(outlet)));
    }
    std::vector<float> flows(nodes.size(), 0.0F);
    Run run;
    const auto samples = static_cast<std::size_t>(seconds * rate);
    run.left.reserve(samples);
    for (std::size_t sample = 0; sample < samples; ++sample) {
        if (sample % 240 == 0) network.beginBlock(media, 1.0);
        const auto time = static_cast<double>(sample) / rate;
        const auto phase = std::fmod(time * 50.0, 1.0);
        for (std::size_t cylinder = 0; cylinder < sources.size(); ++cylinder) {
            const auto local = std::fmod(phase + static_cast<double>(cylinder)
                / static_cast<double>(sources.size()), 1.0);
            sources[cylinder] = pulses && local < 0.02
                ? static_cast<float>(2'000.0 * std::sin(std::numbers::pi * local / 0.02)) : 0.0F;
        }
        if (feed != Feed::none) {
            const auto value = feed == Feed::steady ? amplitudeM3PerS
                : amplitudeM3PerS * std::sin(2.0 * std::numbers::pi * frequencyHz * time);
            std::fill(flows.begin(), flows.end(), static_cast<float>(value));
            network.setOutletSolverVolumeFlow(flows, nodes, paths);
        }
        const auto output = network.process(sources, boundaries, 0.01F);
        float sum = 0.0F;
        for (std::size_t path = 0; path < config.exhaustPaths.size(); ++path)
            sum += output[path].leftPa;
        run.left.push_back(sum);
    }
    return run;
}

double rmsOverLastHalf(const Run& run) {
    double sum = 0.0;
    const auto first = run.left.size() / 2;
    for (auto index = first; index < run.left.size(); ++index)
        sum += static_cast<double>(run.left[index]) * run.left[index];
    return std::sqrt(sum / static_cast<double>(run.left.size() - first));
}

} // namespace

int main() {
    const auto config = stockEngine();

    // Off, and on with no solver flow named: the network as it was.
    const auto off = render(config, 0.0, true, Feed::none, 0.0, 0.0, 0.5);
    const auto unnamed = render(config, 200.0, true, Feed::none, 0.0, 0.0, 0.5);
    require(off.left == unnamed.left, "with no solver flow the far field is the network's, bit for bit");
    std::cout << "no solver flow: the network's own far field, bit for bit\n";

    // A silent network fed the same volume acceleration at 60 Hz and 2 kHz:
    // the low band passes, the high band is the network's (here silent).
    constexpr double acceleration = 0.5; // m3/s2, amplitude of dQ/dt
    const auto low = render(config, 200.0, false, Feed::sine, 60.0,
        acceleration / (2.0 * std::numbers::pi * 60.0), 1.0);
    const auto high = render(config, 200.0, false, Feed::sine, 2'000.0,
        acceleration / (2.0 * std::numbers::pi * 2'000.0), 1.0);
    const auto lowRms = rmsOverLastHalf(low);
    const auto highRms = rmsOverLastHalf(high);
    require(lowRms > 0.0, "the solver's 60 Hz flow is radiated");
    require(highRms < 0.03 * lowRms, "the solver's 2 kHz flow is rejected (second order at 200 Hz)");
    std::cout << "60 Hz passes, 2 kHz at " << 20.0 * std::log10(highRms / lowRms) << " dB\n";

    // A 60 Hz flow twice as fast radiates twice the pressure: a monopole,
    // rho dQ/dt / (4 pi r).
    const auto doubled = render(config, 200.0, false, Feed::sine, 60.0,
        2.0 * acceleration / (2.0 * std::numbers::pi * 60.0), 1.0);
    require(std::abs(rmsOverLastHalf(doubled) / lowRms - 2.0) < 1.0e-3,
        "the far field is linear in the solver's flow");

    // A steady outflow from the first sample: its mean is no sound.
    const auto steady = render(config, 200.0, false, Feed::steady, 0.0, 0.05, 0.5);
    float peak = 0.0F;
    for (const auto value : steady.left) peak = std::max(peak, std::abs(value));
    require(peak < 1.0e-6F, "a steady solver outflow makes no thump");
    std::cout << "a steady outflow is silent (peak " << peak << " Pa)\n";

    std::cout << "EngineLab.OutletLowBand PASS\n";
    return 0;
}
