// A two-stroke's inlet tract (TwoStrokeConfig::inletTractLengthMm): until
// 2026-11-25 the carburettor was an orifice straight into the crankcase and no
// length upstream of the reed reached the physics (the intake's
// `inlet_duct_length_mm` feeds the sound only). The tract is now a 1-D duct
// between the bellmouth and the reed, with the slide a restriction inside it.
//
// What is checked here: with the reed shut, the tract rings as a pipe closed at
// one end and open at the other, at c / 4L (the 1-D solver's open end has no
// end correction: a real bellmouth's adds about 0.6 radius, Levine & Schwinger
// 1948, which is reported, not asserted); its length moves the engine's air
// flow, which no geometry upstream of the reed could before; a tract on a
// perfect check valve is refused; an engine without one is unchanged.
#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/foundation/ReedValve.hpp>
#include <enginelab/physics/SimplifiedGasolinePhysics.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <numbers>
#include <string>
#include <string_view>
#include <vector>

namespace {
using namespace enginelab;

void require(bool condition, std::string_view message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(EXIT_FAILURE);
    }
}

[[nodiscard]] ReedValveConfig illustrativePetals() {
    // The same illustrative petals as EngineLab.ReedValve: no maker's data.
    ReedValveConfig reed;
    reed.petalCount = 2;
    reed.freeLengthMm = 30.0;
    reed.widthMm = 25.0;
    reed.thicknessMm = 0.30;
    reed.youngsModulusGpa = 20.0;
    reed.densityKgPerM3 = 1'900.0;
    reed.dampingRatio = 0.05;
    reed.stopLiftMm = 8.0;
    reed.dischargeCoefficient = 0.6;
    return reed;
}

[[nodiscard]] EngineConfig rd350WithPetals() {
    static const auto catalogue = loadEngineCatalog(ENGINELAB_CATALOG_ROOT);
    const auto selected = selectSingleEngineCatalogEntry(catalogue.entries, std::string("RD350LC"));
    require(static_cast<bool>(selected), "the catalogue has the RD350LC");
    auto config = selected.entry->config;
    normaliseEngineConfig(config);
    config.twoStroke.reed = illustrativePetals();
    return config;
}

[[nodiscard]] std::unique_ptr<EngineSimulator> simulatorFor(const EngineConfig& config, SimpleEcuModel& ecu,
                                                            SimplifiedGasolinePhysics& physics,
                                                            FourStrokeEventGenerator& events, ExhaustGraph& exhaust) {
    return std::make_unique<EngineSimulator>(config, ecu, physics, events, exhaust);
}

struct Ringing final {
    double frequencyHz { 0.0 };
    double closedSeconds { 0.0 };
    double amplitudeKpa { 0.0 };
    double rpm { 0.0 };
};

// The dominant frequency of the reed-end pressure while the reed is shut, on
// an engine the starter turns over with the ignition off.
[[nodiscard]] Ringing ringingWhileTheReedIsShut(const EngineConfig& config) {
    SimpleEcuModel ecu;
    SimplifiedGasolinePhysics physics;
    FourStrokeEventGenerator events;
    auto exhaust = ExhaustGraph::makeForEngine(config);
    auto simulator = simulatorFor(config, ecu, physics, events, exhaust);
    const EngineControls cranking { false, true, 1.0, 0.0 };
    for (int frame = 0; frame < 240; ++frame) static_cast<void>(simulator->step(1.0 / 240.0, cranking));
    constexpr double dt = 1.0 / 48'000.0;
    std::vector<double> pressure;
    std::vector<double> lift;
    Ringing result;
    for (int sample = 0; sample < 48'000 / 2; ++sample) {
        const auto state = simulator->step(dt, cranking).state;
        pressure.push_back(state.inletTractReedPressureKpa);
        lift.push_back(state.reedTipLiftMm);
        result.rpm += state.rpm;
    }
    result.rpm /= static_cast<double>(pressure.size());
    // The longest shut stretch, from 2 ms after the petal seats (its closing
    // transient) to its next lift.
    std::size_t bestBegin = 0;
    std::size_t bestEnd = 0;
    for (std::size_t index = 1; index < lift.size(); ++index) {
        if (!(lift[index] == 0.0 && lift[index - 1] > 0.0)) continue;
        auto end = index;
        while (end < lift.size() && lift[end] == 0.0) ++end;
        if (end - index > bestEnd - bestBegin) {
            bestBegin = index;
            bestEnd = end;
        }
    }
    const auto begin = bestBegin + static_cast<std::size_t>(0.002 / dt);
    if (bestEnd <= begin + 200) return result;
    const auto count = bestEnd - begin;
    result.closedSeconds = static_cast<double>(count) * dt;
    // Remove a straight line (the slow drift back to ambient), window, and
    // scan the spectrum at 1 Hz.
    double meanT = 0.0;
    double meanP = 0.0;
    for (std::size_t index = 0; index < count; ++index) {
        meanT += static_cast<double>(index);
        meanP += pressure[begin + index];
    }
    meanT /= static_cast<double>(count);
    meanP /= static_cast<double>(count);
    double covariance = 0.0;
    double variance = 0.0;
    for (std::size_t index = 0; index < count; ++index) {
        const auto t = static_cast<double>(index) - meanT;
        covariance += t * (pressure[begin + index] - meanP);
        variance += t * t;
    }
    const auto slope = covariance / variance;
    std::vector<double> signal(count);
    for (std::size_t index = 0; index < count; ++index) {
        const auto t = static_cast<double>(index) - meanT;
        const auto hann = 0.5 - 0.5 * std::cos(2.0 * std::numbers::pi * static_cast<double>(index)
            / static_cast<double>(count - 1));
        signal[index] = (pressure[begin + index] - meanP - slope * t) * hann;
        result.amplitudeKpa = std::max(result.amplitudeKpa, std::abs(pressure[begin + index] - meanP - slope * t));
    }
    double bestPower = -1.0;
    for (double frequency = 150.0; frequency <= 3'000.0; frequency += 1.0) {
        double re = 0.0;
        double im = 0.0;
        for (std::size_t index = 0; index < count; ++index) {
            const auto phase = 2.0 * std::numbers::pi * frequency * static_cast<double>(index) * dt;
            re += signal[index] * std::cos(phase);
            im += signal[index] * std::sin(phase);
        }
        if (re * re + im * im > bestPower) {
            bestPower = re * re + im * im;
            result.frequencyHz = frequency;
        }
    }
    return result;
}

void aShutReedLeavesTheTractRingingAsAQuarterWavePipe() {
    auto config = rd350WithPetals();
    // A carburettor bore wider than the tract opens the slide to the duct's
    // whole face: a plain pipe, so the comparison is with a pipe's theory.
    config.intake.throttleDiameterMm = 40.0;
    config.throttleDiameterMm = 40.0;
    for (auto& path : config.intakePaths) path.geometry.throttleDiameterMm = 40.0;
    for (const auto lengthMm : { 200.0, 400.0 }) {
        config.twoStroke.inletTractLengthMm = lengthMm;
        config.twoStroke.inletTractDiameterMm = 26.0;
        config.twoStroke.inletTractSlideFromReedMm = 0.0;
        require(!validateEngineConfig(config).has_value(), "the RD350LC with petals and a tract validates");
        const auto ringing = ringingWhileTheReedIsShut(config);
        constexpr double soundSpeedMps = 343.2; // dry air at 20 C
        const auto pipeHz = soundSpeedMps / (4.0 * lengthMm * 0.001);
        const auto bellmouthHz = soundSpeedMps / (4.0 * (lengthMm + 0.6133 * 13.0) * 0.001);
        std::cout << "tract " << lengthMm << " mm, cranking at " << ringing.rpm << " rpm: reed shut "
                  << ringing.closedSeconds * 1'000.0 << " ms, ringing " << ringing.frequencyHz << " Hz ("
                  << ringing.amplitudeKpa << " kPa) against c/4L " << pipeHz
                  << " Hz; with a bellmouth's end correction " << bellmouthHz << " Hz\n";
        require(ringing.closedSeconds > 0.010, "the starter leaves the reed shut for at least 10 ms at a time");
        require(ringing.amplitudeKpa > 0.05, "and the tract rings while it is shut");
        require(std::abs(ringing.frequencyHz / pipeHz - 1.0) < 0.05,
            "at a quarter-wave pipe's frequency, c/4L, within 5 %");
    }
}

struct Run final {
    double airGps { 0.0 };
    double rpm { 0.0 };
    double minimumReedSideKpa { 1.0e9 };
    double maximumReedSideKpa { 0.0 };
};

[[nodiscard]] Run wideOpen(const EngineConfig& config) {
    SimpleEcuModel ecu;
    SimplifiedGasolinePhysics physics;
    FourStrokeEventGenerator events;
    auto exhaust = ExhaustGraph::makeForEngine(config);
    auto simulator = simulatorFor(config, ecu, physics, events, exhaust);
    Run run;
    int samples = 0;
    constexpr int frames = 3 * 240;
    for (int frame = 0; frame < frames; ++frame) {
        const auto time = frame / 240.0;
        const auto state = simulator->step(1.0 / 240.0, { true, time < 1.5, 1.0, 0.0 }).state;
        if (frame >= frames - 120) {
            run.airGps += state.airFlowGramsPerSecond;
            run.rpm += state.rpm;
            run.minimumReedSideKpa = std::min(run.minimumReedSideKpa, state.inletTractReedPressureKpa);
            run.maximumReedSideKpa = std::max(run.maximumReedSideKpa, state.inletTractReedPressureKpa);
            ++samples;
        }
    }
    run.airGps /= samples;
    run.rpm /= samples;
    return run;
}

void theTractsLengthMovesTheEnginesAir() {
    auto config = rd350WithPetals();
    const auto orifice = wideOpen(config);
    require(orifice.maximumReedSideKpa == 0.0, "without a tract there is no tract pressure to publish");
    config.twoStroke.inletTractDiameterMm = 26.0;
    config.twoStroke.inletTractLengthMm = 130.0;
    const auto shortTract = wideOpen(config);
    config.twoStroke.inletTractLengthMm = 400.0;
    const auto longTract = wideOpen(config);
    std::cout << "RD350LC with petals, 1.5 s at WOT after the start: orifice carburettor " << orifice.airGps
              << " g/s at " << orifice.rpm << " rpm; 130 mm tract " << shortTract.airGps << " g/s at "
              << shortTract.rpm << " rpm (reed side " << shortTract.minimumReedSideKpa << "-"
              << shortTract.maximumReedSideKpa << " kPa); 400 mm " << longTract.airGps << " g/s at "
              << longTract.rpm << " rpm\n";
    require(shortTract.minimumReedSideKpa > 20.0 && shortTract.maximumReedSideKpa < 250.0,
        "the tract's pressure at the reed is a physical one");
    require(std::abs(longTract.airGps / shortTract.airGps - 1.0) > 0.01,
        "the tract's length moves the engine's air flow, which no length upstream of the reed did");
}

void aTractNeedsAnAreaAtItsEnd() {
    auto config = rd350WithPetals();
    config.twoStroke.inletTractLengthMm = 130.0;
    require(!validateEngineConfig(config).has_value(), "a tract on petals validates");
    config.twoStroke.reed = {};
    require(validateEngineConfig(config).has_value(),
        "a tract on a perfect check valve is refused: the valve gives its duct no area");
    config.twoStroke.intake = TwoStrokeIntake::cylinderPort;
    config.twoStroke.intakePortReed = false;
    require(!validateEngineConfig(config).has_value(), "a tract on an open cylinder port validates");
    const auto piston = wideOpen(config);
    std::cout << "RD350LC on an open cylinder port with a 130 mm tract: " << piston.airGps << " g/s at "
              << piston.rpm << " rpm\n";
    require(piston.airGps > 1.0 && piston.rpm > 1'000.0, "and runs on it");
    config.twoStroke.inletTractLengthMm = 20.0;
    require(validateEngineConfig(config).has_value(), "a tract shorter than 40 mm is refused");
    config.twoStroke.inletTractLengthMm = 130.0;
    config.twoStroke.inletTractSlideFromReedMm = 120.0;
    require(validateEngineConfig(config).has_value(), "a slide within 15 mm of an end is refused");
}

}

int main() {
    aShutReedLeavesTheTractRingingAsAQuarterWavePipe();
    theTractsLengthMovesTheEnginesAir();
    aTractNeedsAnAreaAtItsEnd();
    std::cout << "PASS\n";
    return 0;
}
