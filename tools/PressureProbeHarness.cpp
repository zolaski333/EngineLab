// In-pipe pressure at one place of an exhaust, against crank angle, in
// pascals: what a pressure transducer in the pipe records.
//
// It holds a speed under the same wide-open absorber as DynoSweepHarness (or
// a part throttle), probes one sample of one authored exhaust component
// through the simulator's gas probe (EngineSimulator::probeGasField), and
// averages that place's pressure over whole cycles, bin by bin (2 deg on a
// four-stroke). It prints the averaged cycle (with cylinder 1's pressure
// beside it), then its harmonics as sound
// pressure levels (dB re 20 uPa, RMS): the form in which published in-pipe
// measurements are read (Winterbone & Pearson 1999, Figs 6.12-6.14).
//
// The radiated side is EngineLabOfflineAudioExporter's (a governed stage, no
// convolution, float32, stems): its samples are pascals over the monitor's
// full scale (AcousticMonitorCalibration).
#include <enginelab/calibration/CalibrationStore.hpp>
#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/foundation/EngineTypes.hpp>
#include <enginelab/events/CylinderPressureSample.hpp>
#include <enginelab/events/ExhaustAcousticSample.hpp>
#include <enginelab/foundation/GasFieldSnapshot.hpp>
#include <enginelab/physics/SimplifiedGasolinePhysics.hpp>
#include <enginelab/simulation/DynoAbsorberController.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <numbers>
#include <optional>
#include <string>

namespace {

struct Settings final {
    std::filesystem::path catalogRoot { ENGINELAB_CATALOG_ROOT };
    std::string filter;
    double rpm { 3'000.0 };
    double throttle { 1.0 };
    std::uint32_t componentId { 0 };
    double positionMm { 0.0 };
    double exhaustCellMm { 0.0 };
    double settleSeconds { 4.0 };
    double sampleSeconds { 2.0 };
    int harmonics { 40 };
    double observerM { 0.5 };
    // Diagnostics: the exhaust coupled on every mechanical substep (the
    // simulator's oracle) and the mechanical step's crank angle.
    bool couplingEverySubstep { false };
    double crankDegreesPerStep { 0.0 };
};

[[noreturn]] void usage() {
    std::cerr << "usage: EngineLabPressureProbeHarness --filter exact-key --component id"
                 " --position-mm mm [--catalog-root dir] [--rpm rpm] [--throttle 0..1]"
                 " [--exhaust-cell-mm mm] [--settle s] [--sample s] [--harmonics n] [--observer-m m]"
                 " [--coupling-every-substep] [--crank-degrees deg]\n";
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
        else if (arg == "--rpm") s.rpm = std::stod(next());
        else if (arg == "--throttle") s.throttle = std::clamp(std::stod(next()), 0.0, 1.0);
        else if (arg == "--component") s.componentId = static_cast<std::uint32_t>(std::stoul(next()));
        else if (arg == "--position-mm") s.positionMm = std::stod(next());
        else if (arg == "--exhaust-cell-mm") s.exhaustCellMm = std::stod(next());
        else if (arg == "--settle") s.settleSeconds = std::stod(next());
        else if (arg == "--sample") s.sampleSeconds = std::stod(next());
        else if (arg == "--harmonics") s.harmonics = std::clamp(std::stoi(next()), 1, 170);
        else if (arg == "--observer-m") s.observerM = std::stod(next());
        else if (arg == "--coupling-every-substep") s.couplingEverySubstep = true;
        else if (arg == "--crank-degrees") s.crankDegreesPerStep = std::stod(next());
        else usage();
    }
    if (s.filter.empty() || s.componentId == 0 || !(s.rpm > 0.0) || !(s.settleSeconds > 0.0)
        || !(s.sampleSeconds > 0.0) || s.exhaustCellMm < 0.0 || !(s.observerM > 0.0))
        usage();
    return s;
}

// The authored length of a component, for placing the probe along it.
std::optional<double> componentLengthMm(const enginelab::EngineConfig& config, std::uint32_t id) {
    for (const auto& path : config.exhaustPaths)
        if (path.network) for (const auto& component : path.network->components)
            if (component.id == id) return component.lengthMm;
    return std::nullopt;
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
        // As DynoSweepHarness --crank-degrees: a finer step raises the
        // solver's frequency ceiling with it.
        if (settings.crankDegreesPerStep > 0.0) {
            config.solver.maximumCrankDegreesPerStep = settings.crankDegreesPerStep;
            config.solver.maximumMechanicalFrequencyHz = std::clamp(
                std::max(config.redlineRpm, config.ignition.revLimitRpm) * 6.0
                    / settings.crankDegreesPerStep * 1.01,
                config.solver.maximumMechanicalFrequencyHz, 100'000.0);
        }
        const auto lengthMm = componentLengthMm(config, settings.componentId);
        if (!lengthMm || !(*lengthMm > 0.0)) {
            std::cerr << "no exhaust component " << settings.componentId << " with a length\n";
            return EXIT_FAILURE;
        }

        auto calibrations = std::make_shared<enginelab::calibration::CalibrationStore>();
        enginelab::SimpleEcuModel ecu(calibrations);
        enginelab::SimplifiedGasolinePhysics physics;
        enginelab::FourStrokeEventGenerator events;
        auto exhaust = enginelab::ExhaustGraph::makeForEngine(config);
        enginelab::EngineSimulatorOptions options;
        if (settings.exhaustCellMm > 0.0) options.exhaustTargetCellLengthM = settings.exhaustCellMm * 1.0e-3;
        // EngineRuntime-sized: never on the stack.
        auto simulator = std::make_unique<enginelab::EngineSimulator>(
            config, ecu, physics, events, exhaust, options);
        simulator->setExhaustCouplingEverySubstep(settings.couplingEverySubstep);

        constexpr double dt = 1.0 / 240.0;
        for (int step = 0; step < static_cast<int>(2.0 / dt); ++step) {
            const auto time = static_cast<double>(step) * dt;
            (void)simulator->step(dt, { true, time < 1.5, 0.55, 0.0 });
        }

        // The probed element: the authored component's duct.
        std::int32_t elementIndex = -1;
        std::uint8_t sample = 0;
        simulator->captureGasFieldNow();
        const auto& elements = simulator->gasField().elements;
        for (std::size_t e = 0; e < elements.size(); ++e) {
            const auto& element = elements[e];
            if (element.kind != enginelab::GasFieldElementKind::exhaustDuct
                || element.componentId != settings.componentId || element.sampleCount == 0) continue;
            elementIndex = static_cast<std::int32_t>(e);
            // Sample s sits at (s + 0.5) / count of the element's length.
            const auto count = static_cast<double>(element.sampleCount);
            const auto nearest = std::round(settings.positionMm / *lengthMm * count - 0.5);
            sample = static_cast<std::uint8_t>(std::clamp(nearest, 0.0, count - 1.0));
            std::cerr << "probe: element " << e << " of component " << settings.componentId
                      << ", sample " << static_cast<int>(sample) << " of " << static_cast<int>(element.sampleCount)
                      << " at " << (static_cast<double>(sample) + 0.5) / count * *lengthMm << " mm\n";
            break;
        }
        if (elementIndex < 0) {
            std::cerr << "component " << settings.componentId << " is not an exhaust duct of the gas field\n";
            return EXIT_FAILURE;
        }

        enginelab::DynoAbsorberController absorber(config);
        absorber.reset(simulator->state().rpm, simulator->state().torqueNm);
        const auto controlsAt = [&](double targetRpm) {
            const auto output = absorber.advance(dt, targetRpm, simulator->state());
            enginelab::EngineControls controls;
            controls.ignitionEnabled = true;
            controls.starterEngaged = simulator->state().rpm < enginelab::dynoStarterCutoutRpm(config);
            controls.throttle = settings.throttle;
            controls.dynamometerTorqueNm = output.brakeTorqueNm;
            return controls;
        };
        for (int step = 0; step < static_cast<int>(settings.settleSeconds / dt); ++step)
            (void)simulator->step(dt, controlsAt(settings.rpm));

        simulator->probeGasField(elementIndex, sample);
        constexpr auto bins = enginelab::GasProbeTrace::bins;
        std::array<double, bins> sum {};
        std::array<double, bins> count {};
        // The finite-volume outlets' volume flow (mass flow over the outlet
        // gas density, every outlet summed), binned by crank angle: what the
        // mean-flow solver alone would radiate as a monopole, with no audio
        // network in between. Samples carry time; the crank angle is
        // interpolated across the frame (the held speed is steady).
        simulator->setPressureSamplingEnabled(true);
        simulator->setExhaustAcousticEveryCoupling(true);
        std::array<double, bins> flowSum {};
        std::array<double, bins> flowCount {};
        // The outlets' mass flow alone, binned the same way, and the outlet
        // gas density averaged over the run: mass flow over that MEAN density
        // is the volume flow a uniform-medium acoustic model can carry. The
        // difference from mass flow over the instantaneous density is the
        // exit's temperature (entropy) modulation of its volume flux.
        std::array<double, bins> massFlowSum {};
        auto outletDensitySum = 0.0, outletDensityCount = 0.0;
        // Cylinder 1's pressure, binned the same way: the state a measured
        // case gives at the exhaust opening (Jones & Brown's pc(0)).
        std::array<double, bins> cylinderSum {};
        std::array<double, bins> cylinderCount {};
        // Cylinder 1's exhaust-port flow, binned the same way: its volume
        // flow at the port's instantaneous density and its mass flow, to
        // compare what enters the exhaust with what leaves the outlets.
        std::array<double, bins> portVolumeFlowSum {};
        std::array<double, bins> portMassFlowSum {};
        auto portDensitySum = 0.0, portDensityCount = 0.0;
        const auto cycleOf = [&] { return simulator->gasProbe().cycleDegrees > 0.0
            ? simulator->gasProbe().cycleDegrees : 720.0; };
        const auto forward = [](double degrees, double cycle) {
            return std::fmod(std::fmod(degrees, cycle) + cycle, cycle); };
        std::int32_t previous = -1;
        auto rpmSum = 0.0, torqueSum = 0.0, frames = 0.0, couplingHzSum = 0.0;
        std::size_t acousticSamples = 0, droppedAcousticSamples = 0, staleAcousticSamples = 0;
        for (int step = 0; step < static_cast<int>(settings.sampleSeconds / dt); ++step) {
            const auto controls = controlsAt(settings.rpm);
            const auto startSeconds = simulator->state().simulationTimeSeconds;
            const auto startDegrees = simulator->state().crankAngleDegrees;
            const auto frame = simulator->step(dt, controls);
            rpmSum += frame.state.rpm;
            couplingHzSum += frame.state.exhaustCouplingFrequencyHz;
            torqueSum += controls.dynamometerTorqueNm;
            frames += 1.0;
            acousticSamples += frame.exhaustAcousticSampleCount;
            droppedAcousticSamples += frame.droppedExhaustAcousticSampleCount;
            {
                const auto cycle = cycleOf();
                const auto spanSeconds = frame.state.simulationTimeSeconds - startSeconds;
                const auto spanDegrees = forward(frame.state.crankAngleDegrees - startDegrees, cycle);
                enginelab::CylinderPressureSample cylinder;
                while (simulator->tryPopCylinderPressureSample(cylinder)) {
                    if (!(spanSeconds > 0.0) || cylinder.cylinderCount == 0) continue;
                    const auto fraction = std::clamp((cylinder.timeSeconds - startSeconds) / spanSeconds, 0.0, 1.0);
                    const auto degrees = forward(startDegrees + fraction * spanDegrees, cycle);
                    const auto b = std::min<std::size_t>(bins - 1, static_cast<std::size_t>(degrees / cycle * bins));
                    cylinderSum[b] += cylinder.pressureBar[0];
                    cylinderCount[b] += 1.0;
                    const auto portDensity = static_cast<double>(cylinder.exhaustPortDensityKgPerM3[0]);
                    const auto portMassFlow = static_cast<double>(cylinder.exhaustAcousticMassFlowKgPerSecond[0]);
                    if (portDensity > 0.0) {
                        portVolumeFlowSum[b] += portMassFlow / portDensity;
                        portDensitySum += portDensity;
                        portDensityCount += 1.0;
                    }
                    portMassFlowSum[b] += portMassFlow;
                }
                enginelab::ExhaustAcousticSample acoustic;
                while (simulator->tryPopExhaustAcousticSample(acoustic)) {
                    if (!(spanSeconds > 0.0)) continue;
                    if (acoustic.timeSeconds < startSeconds - 1.0e-9
                        || acoustic.timeSeconds > startSeconds + spanSeconds + 1.0e-9)
                        ++staleAcousticSamples;
                    auto volumeFlow = 0.0, massFlow = 0.0;
                    for (std::size_t o = 0; o < acoustic.outletCount; ++o)
                        if (acoustic.outletDensityKgPerM3[o] > 0.0F) {
                            volumeFlow += acoustic.outletMassFlowKgPerSecond[o] / acoustic.outletDensityKgPerM3[o];
                            massFlow += acoustic.outletMassFlowKgPerSecond[o];
                            outletDensitySum += acoustic.outletDensityKgPerM3[o];
                            outletDensityCount += 1.0;
                        }
                    const auto fraction = std::clamp((acoustic.timeSeconds - startSeconds) / spanSeconds, 0.0, 1.0);
                    const auto degrees = forward(startDegrees + fraction * spanDegrees, cycle);
                    const auto b = std::min<std::size_t>(bins - 1, static_cast<std::size_t>(degrees / cycle * bins));
                    flowSum[b] += volumeFlow;
                    massFlowSum[b] += massFlow;
                    flowCount[b] += 1.0;
                }
            }
            const auto& probe = simulator->gasProbe();
            if (probe.latestBin < 0) continue;
            // The bins this frame wrote: after the previous frame's last bin,
            // through this one's. The first frame only places the start.
            if (previous >= 0) {
                for (auto b = previous; b != probe.latestBin;) {
                    b = (b + 1) % static_cast<std::int32_t>(bins);
                    sum[static_cast<std::size_t>(b)] += probe.pressurePa[static_cast<std::size_t>(b)];
                    count[static_cast<std::size_t>(b)] += 1.0;
                }
            }
            previous = probe.latestBin;
        }
        simulator->stopGasProbe();

        std::array<double, bins> mean {};
        auto minimumCount = std::numeric_limits<double>::infinity();
        for (std::size_t b = 0; b < bins; ++b) {
            minimumCount = std::min(minimumCount, count[b]);
            mean[b] = count[b] > 0.0 ? sum[b] / count[b] : 0.0;
        }
        if (!(minimumCount > 0.0)) {
            std::cerr << "a crank bin was never sampled\n";
            return EXIT_FAILURE;
        }
        const auto heldRpm = rpmSum / frames;
        const auto cycleDegrees = simulator->gasProbe().cycleDegrees;
        std::cout << std::fixed << std::setprecision(3);
        std::cout << "# engine " << config.name << "\n# held_rpm " << heldRpm
                  << "\n# brake_torque_nm " << torqueSum / frames
                  << "\n# power_kw " << torqueSum / frames * heldRpm * std::numbers::pi / 30.0 / 1'000.0
                  << "\n# cycles_per_bin_min " << minimumCount
                  << "\n# exhaust_coupling_hz " << couplingHzSum / frames << '\n';
        std::cout << "crank_deg,pressure_bar,cylinder1_bar\n";
        for (std::size_t b = 0; b < bins; ++b)
            std::cout << (static_cast<double>(b) + 0.5) * cycleDegrees / static_cast<double>(bins) << ','
                      << mean[b] * 1.0e-5 << ','
                      << (cylinderCount[b] > 0.0 ? cylinderSum[b] / cylinderCount[b] : 0.0) << '\n';

        // Harmonics of the cycle frequency: a bin-averaged cycle is periodic
        // by construction, so its discrete Fourier series is exact.
        const auto cycleHz = heldRpm / 60.0 * 360.0 / cycleDegrees;
        std::cout << "harmonic,frequency_hz,spl_db\n";
        for (int k = 1; k <= settings.harmonics; ++k) {
            auto re = 0.0, im = 0.0;
            for (std::size_t b = 0; b < bins; ++b) {
                const auto phase = 2.0 * std::numbers::pi * static_cast<double>(k)
                    * (static_cast<double>(b) + 0.5) / static_cast<double>(bins);
                re += mean[b] * std::cos(phase);
                im -= mean[b] * std::sin(phase);
            }
            const auto amplitudePa = 2.0 * std::hypot(re, im) / static_cast<double>(bins);
            const auto rmsPa = amplitudePa / std::numbers::sqrt2;
            std::cout << k << ',' << static_cast<double>(k) * cycleHz << ','
                      << 20.0 * std::log10(std::max(rmsPa, 1.0e-12) / 20.0e-6) << '\n';
        }

        // The outlets' monopole at `--observer-m` in the ambient air:
        // p = rho0 dQ/dt / (4 pi r), harmonic by harmonic. Bins the coupling
        // clock skipped are interpolated from their neighbours.
        const auto ambientDensity = config.ambientPressureKpa * 1'000.0
            / (287.05 * (config.ambientTemperatureC + 273.15));
        const auto meanOutletDensity = outletDensityCount > 0.0
            ? outletDensitySum / outletDensityCount : 0.0;
        // Bins the coupling clock skipped are interpolated from their
        // neighbours; false when too few bins were sampled.
        const auto fillBinsBy = [&](const std::array<double, bins>& total,
                                    const std::array<double, bins>& counts,
                                    std::array<double, bins>& out) {
            std::size_t filled = 0;
            for (std::size_t b = 0; b < bins; ++b)
                if (counts[b] > 0.0) { out[b] = total[b] / counts[b]; ++filled; }
            if (filled <= bins / 4) return std::size_t { 0 };
            for (std::size_t b = 0; b < bins; ++b) {
                if (counts[b] > 0.0) continue;
                std::size_t before = b, after = b, back = 0, ahead = 0;
                do { before = (before + bins - 1) % bins; ++back; } while (counts[before] <= 0.0);
                do { after = (after + 1) % bins; ++ahead; } while (counts[after] <= 0.0);
                const auto weight = static_cast<double>(back) / static_cast<double>(back + ahead);
                out[b] = out[before] + weight * (out[after] - out[before]);
            }
            return filled;
        };
        const auto fillBins = [&](const std::array<double, bins>& total,
                                  std::array<double, bins>& out) {
            return fillBinsBy(total, flowCount, out);
        };
        // p = rho0 dQ/dt / (4 pi r), harmonic by harmonic.
        const auto printMonopole = [&](const std::array<double, bins>& flow) {
            for (int k = 1; k <= settings.harmonics; ++k) {
                auto re = 0.0, im = 0.0;
                for (std::size_t b = 0; b < bins; ++b) {
                    const auto phase = 2.0 * std::numbers::pi * static_cast<double>(k)
                        * (static_cast<double>(b) + 0.5) / static_cast<double>(bins);
                    re += flow[b] * std::cos(phase);
                    im -= flow[b] * std::sin(phase);
                }
                const auto flowAmplitude = 2.0 * std::hypot(re, im) / static_cast<double>(bins);
                const auto omega = 2.0 * std::numbers::pi * static_cast<double>(k) * cycleHz;
                const auto rmsPa = ambientDensity * omega * flowAmplitude
                    / (4.0 * std::numbers::pi * settings.observerM) / std::numbers::sqrt2;
                std::cout << k << ',' << static_cast<double>(k) * cycleHz << ','
                          << 20.0 * std::log10(std::max(rmsPa, 1.0e-12) / 20.0e-6) << '\n';
            }
        };
        // The outlets' monopole at `--observer-m` in the ambient air.
        std::array<double, bins> flow {};
        const auto filled = fillBins(flowSum, flow);
        std::cout << "# outlet_samples " << acousticSamples << " dropped " << droppedAcousticSamples
                  << " outside_frame " << staleAcousticSamples << " bins_sampled " << filled << '\n';
        if (filled > 0) {
            std::cout << "# outlet_bins_sampled " << filled << " of " << bins
                      << "\n# mean_outlet_volume_flow_m3_s " << std::accumulate(flow.begin(), flow.end(), 0.0) / bins
                      << "\nharmonic,frequency_hz,outlet_monopole_spl_db_at_" << settings.observerM << "_m\n";
            printMonopole(flow);
        }
        // The same monopole with the outlet gas held at its mean density.
        std::array<double, bins> massFlow {};
        if (fillBins(massFlowSum, massFlow) > 0 && meanOutletDensity > 0.0) {
            for (auto& value : massFlow) value /= meanOutletDensity;
            std::cout << "# mean_outlet_density_kg_m3 " << meanOutletDensity
                      << "\nharmonic,frequency_hz,outlet_monopole_mean_density_spl_db_at_"
                      << settings.observerM << "_m\n";
            printMonopole(massFlow);
        }
        // Cylinder 1's port flow as the same monopole: what enters the
        // exhaust, at the port's instantaneous and at its mean density.
        std::array<double, bins> portFlow {};
        if (fillBinsBy(portVolumeFlowSum, cylinderCount, portFlow) > 0) {
            std::cout << "# mean_port_volume_flow_m3_s "
                      << std::accumulate(portFlow.begin(), portFlow.end(), 0.0) / bins
                      << "\nharmonic,frequency_hz,port_monopole_spl_db_at_"
                      << settings.observerM << "_m\n";
            printMonopole(portFlow);
        }
        std::array<double, bins> portMass {};
        if (fillBinsBy(portMassFlowSum, cylinderCount, portMass) > 0 && portDensityCount > 0.0) {
            const auto meanPortDensity = portDensitySum / portDensityCount;
            for (auto& value : portMass) value /= meanPortDensity;
            std::cout << "# mean_port_density_kg_m3 " << meanPortDensity
                      << "\nharmonic,frequency_hz,port_monopole_mean_density_spl_db_at_"
                      << settings.observerM << "_m\n";
            printMonopole(portMass);
        }
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "error: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
