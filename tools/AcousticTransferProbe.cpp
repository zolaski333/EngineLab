// The audio exhaust network's transfer from one place in a pipe to the
// microphones: what Winterbone & Pearson (1999, Figs 6.12-6.14) measure as
// the difference between an in-pipe transducer and a microphone.
//
// The network alone is built from an engine's exhaust graph, its gas held at
// one state, and an impulse is put into the cylinder port. Downstream of the
// tap the network is linear and passive, so the ratio of the left
// microphone's spectrum to the tap's is the downstream transfer whatever the
// source; it is printed every --step-hz as dB (Pa at the microphone per Pa in
// the pipe). Jet and flow noise are off.
#include <enginelab/audio/AcousticExhaustNetwork.hpp>
#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/foundation/EngineTypes.hpp>
#include <enginelab/gasdynamics/ExhaustNetworkLayout.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <memory>
#include <numbers>
#include <string>
#include <vector>

namespace {

struct Settings final {
    std::filesystem::path catalogRoot { ENGINELAB_CATALOG_ROOT };
    std::string filter;
    std::uint32_t componentId { 0 };
    double fraction { 0.5 };
    double densityKgPerM3 { 0.55 };
    double soundSpeedMps { 520.0 };
    double meanFlowKgPerSecond { 0.0 };
    // Production radiates into ambient air (RealtimeEngineAudio), 22 C.
    double farFieldDensityKgPerM3 { 1.196 };
    double sampleRateHz { 48'000.0 };
    double seconds { 4.0 };
    double maxHz { 2'000.0 };
    double stepHz { 5.0 };
};

[[noreturn]] void usage() {
    std::cerr << "usage: EngineLabAcousticTransferProbe --filter exact-key --component id"
                 " [--fraction 0..1] [--catalog-root dir] [--density kg/m3]"
                 " [--sound-speed m/s] [--mean-flow kg/s] [--far-field-density kg/m3] [--seconds s]"
                 " [--max-hz hz] [--step-hz hz]\n";
    std::exit(EXIT_FAILURE);
}

Settings parse(int argc, char** argv) {
    Settings s;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        const auto next = [&]() -> std::string {
            if (i + 1 >= argc) usage();
            return argv[++i];
        };
        if (arg == "--catalog-root") s.catalogRoot = next();
        else if (arg == "--filter") s.filter = next();
        else if (arg == "--component") s.componentId = static_cast<std::uint32_t>(std::stoul(next()));
        else if (arg == "--fraction") s.fraction = std::stod(next());
        else if (arg == "--density") s.densityKgPerM3 = std::stod(next());
        else if (arg == "--sound-speed") s.soundSpeedMps = std::stod(next());
        else if (arg == "--mean-flow") s.meanFlowKgPerSecond = std::stod(next());
        else if (arg == "--far-field-density") s.farFieldDensityKgPerM3 = std::stod(next());
        else if (arg == "--seconds") s.seconds = std::stod(next());
        else if (arg == "--max-hz") s.maxHz = std::stod(next());
        else if (arg == "--step-hz") s.stepHz = std::stod(next());
        else usage();
    }
    if (s.filter.empty() || s.componentId == 0 || !(s.seconds > 0.0)
        || !(s.stepHz > 0.0) || !(s.maxHz > 0.0))
        usage();
    return s;
}

std::complex<double> spectrumAt(const std::vector<double>& signal,
                                double frequencyHz, double sampleRateHz) {
    std::complex<double> sum {};
    const auto step = std::polar(1.0, -2.0 * std::numbers::pi * frequencyHz / sampleRateHz);
    std::complex<double> rotation { 1.0, 0.0 };
    for (const auto value : signal) {
        sum += value * rotation;
        rotation *= step;
    }
    return sum;
}

} // namespace

int main(int argc, char** argv) {
    try {
        const auto settings = parse(argc, argv);
        auto catalog = enginelab::loadEngineCatalog(settings.catalogRoot);
        for (const auto& error : catalog.errors) std::cerr << "catalog error: " << error << '\n';
        const auto selection = enginelab::selectSingleEngineCatalogEntry(catalog.entries, settings.filter);
        if (!selection) {
            std::cerr << "filter must name exactly one engine\n";
            return EXIT_FAILURE;
        }
        auto config = selection.entry->config;
        enginelab::normaliseEngineConfig(config);
        const auto graph = enginelab::ExhaustGraph::makeForEngine(config);
        for (const auto& diagnostic : graph.diagnostics()) std::cerr << "graph issue " << static_cast<int>(diagnostic.issue) << " on " << diagnostic.relatedId << '\n';

        // The compiled ducts the network is built from, for the record.
        const auto layout = enginelab::gasdynamics::ExhaustNetworkLayout::compile(graph);
        for (const auto& duct : layout.ducts())
            std::cout << "# duct node " << duct.nodeId << " component " << duct.sourceComponentId
                      << " length_mm " << duct.lengthM * 1.0e3
                      << " inlet_d_mm " << 2.0e3 * std::sqrt(duct.inletFlowAreaM2 / std::numbers::pi)
                      << " outlet_d_mm " << 2.0e3 * std::sqrt(duct.outletFlowAreaM2 / std::numbers::pi) << '\n';
        std::vector<std::uint32_t> cylinderIds;
        for (const auto& cylinder : config.cylinders) cylinderIds.push_back(cylinder.id);
        auto network = std::make_unique<enginelab::AcousticExhaustNetwork>(graph, cylinderIds);
        if (!network->valid() || !network->prepare(settings.sampleRateHz)) {
            std::cerr << "the audio network does not compile\n";
            return EXIT_FAILURE;
        }
        network->setOutletJetNoiseEnabled(false);
        network->setOutletFarFieldDensity(settings.farFieldDensityKgPerM3);
        auto tapped = false;
        for (const auto& node : graph.nodes()) {
            if (node.sourceComponentId != settings.componentId) continue;
            tapped = network->setPressureTap(node.id, static_cast<float>(settings.fraction));
            if (tapped) break;
        }
        if (!tapped) {
            std::cerr << "no audio duct carries component " << settings.componentId << '\n';
            return EXIT_FAILURE;
        }

        const std::array<enginelab::AcousticExhaustNetwork::Medium, 1> medium {{
            { static_cast<float>(settings.densityKgPerM3), static_cast<float>(settings.soundSpeedMps) } }};
        const std::array<float, 1> meanFlow {{ static_cast<float>(settings.meanFlowKgPerSecond) }};
        network->beginBlock(medium, 1.0, meanFlow);

        const auto samples = static_cast<std::size_t>(settings.seconds * settings.sampleRateHz);
        std::vector<double> tap(samples), microphone(samples);
        std::vector<float> source(cylinderIds.size(), 0.0F);
        std::vector<enginelab::AcousticExhaustNetwork::CylinderBoundary> boundaries(cylinderIds.size());
        for (std::size_t sample = 0; sample < samples; ++sample) {
            std::fill(source.begin(), source.end(), 0.0F);
            if (sample == 0 && !source.empty()) source.front() = 1'000.0F;
            const auto output = network->process(source, boundaries, 1.0F);
            tap[sample] = network->lastTapPressurePa();
            microphone[sample] = output[0].leftPa;
        }
        const auto tail = std::max(std::abs(microphone.back()), std::abs(tap.back()));

        std::cout << "# component " << settings.componentId << " fraction " << settings.fraction
                  << " density " << settings.densityKgPerM3 << " sound_speed " << settings.soundSpeedMps
                  << " mean_flow " << settings.meanFlowKgPerSecond << " far_field_density " << settings.farFieldDensityKgPerM3 << " last_sample_pa " << tail << '\n';
        std::cout << "frequency_hz,transfer_db,tap_db,microphone_db\n" << std::fixed << std::setprecision(2);
        for (auto frequency = settings.stepHz; frequency <= settings.maxHz + 1.0e-9; frequency += settings.stepHz) {
            const auto tapSpectrum = std::abs(spectrumAt(tap, frequency, settings.sampleRateHz));
            const auto micSpectrum = std::abs(spectrumAt(microphone, frequency, settings.sampleRateHz));
            const auto floor = 1.0e-30;
            std::cout << frequency << ','
                      << 20.0 * std::log10(std::max(micSpectrum, floor) / std::max(tapSpectrum, floor)) << ','
                      << 20.0 * std::log10(std::max(tapSpectrum, floor)) << ','
                      << 20.0 * std::log10(std::max(micSpectrum, floor)) << '\n';
        }
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "error: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
