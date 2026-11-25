// A two-stroke's ports window by window (TwoStrokeConfig::exhaustWindows,
// transferWindows). Until 2026-11-25 each port group was one sharp-cornered
// rectangle with one constant discharge coefficient: no corner radius, no
// auxiliary or boost window opening at its own angle, no measured map of the
// coefficient against the area and pressure ratios could reach the physics
// (audit findings M-1, M-2).
//
// What is checked here: the rounded window's area is the exact geometry (an
// independent quadrature of its outline, and the closed form when open);
// one sharp window per group is the old rectangle, bit for bit, on the
// running engine; radii, a later boost window and a discharge map each move
// the engine's air as their geometry says; the map interpolates as written
// and reads its direction; a power valve lowers only the windows under it;
// files keep it all; the inconsistent cases are refused.
#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/foundation/TwoStrokePorts.hpp>
#include <enginelab/physics/SimplifiedGasolinePhysics.hpp>
#include <enginelab/serialization/JsonEngineSerializer.hpp>
#include <enginelab/serialization/YamlEngineSerializer.hpp>
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

[[nodiscard]] EngineConfig rd350() {
    static const auto catalogue = loadEngineCatalog(ENGINELAB_CATALOG_ROOT);
    const auto selected = selectSingleEngineCatalogEntry(catalogue.entries, std::string("RD350LC"));
    require(static_cast<bool>(selected), "the catalogue has the RD350LC");
    auto config = selected.entry->config;
    normaliseEngineConfig(config);
    return config;
}

// The window's outline, integrated by the midpoint rule over its depth: the
// width at a depth is the rectangle's less what each rounded corner cuts off.
[[nodiscard]] double quadratureAreaMm2(double width, double height, double top, double bottom, double depth) {
    constexpr int strips = 200'000;
    const auto step = depth / strips;
    double area = 0.0;
    for (int index = 0; index < strips; ++index) {
        const auto y = (index + 0.5) * step;
        auto cut = 0.0;
        if (top > 0.0 && y < top) cut += 2.0 * (top - std::sqrt(top * top - (top - y) * (top - y)));
        const auto fromBottom = height - y;
        if (bottom > 0.0 && fromBottom < bottom)
            cut += 2.0 * (bottom - std::sqrt(bottom * bottom - (bottom - fromBottom) * (bottom - fromBottom)));
        area += (width - cut) * step;
    }
    return area;
}

void aRoundedWindowIsItsExactGeometry() {
    struct Case final { double width, height, top, bottom; };
    for (const auto& window : { Case { 40.0, 25.0, 6.0, 4.0 }, Case { 18.0, 12.0, 9.0, 3.0 },
                                Case { 30.0, 10.0, 5.0, 5.0 }, Case { 50.0, 30.0, 0.0, 8.0 } }) {
        for (const auto fraction : { 0.03, 0.2, 0.5, 0.81, 0.97, 1.0 }) {
            const auto depth = fraction * window.height;
            const auto exact = roundedWindowAreaMm2(window.width, window.height, window.top, window.bottom, depth);
            const auto reference = quadratureAreaMm2(window.width, window.height, window.top, window.bottom, depth);
            require(std::abs(exact / reference - 1.0) < 1.0e-6,
                "the rounded window's area is its outline's integral, within 1e-6");
        }
        const auto full = roundedWindowAreaMm2(window.width, window.height, window.top, window.bottom, window.height);
        const auto closedForm = window.width * window.height
            - (4.0 - std::numbers::pi) * 0.5 * (window.top * window.top + window.bottom * window.bottom);
        require(std::abs(full - closedForm) < 1.0e-9 * closedForm,
            "fully open it is w h - (1 - pi/4)(2 r_top^2 + 2 r_bottom^2)");
    }
    require(roundedWindowAreaMm2(40.0, 25.0, 6.0, 4.0, 0.0) == 0.0, "a covered window has no area");
    require(roundedWindowAreaMm2(40.0, 25.0, 0.0, 0.0, 10.0) == 400.0, "a sharp one is its rectangle");
    // Early in the opening, where the radius matters: 1 mm open under 6 mm
    // corners, each cuts 6 - sqrt(36 - (6 - y)^2) of the chord at depth y
    // (12 mm at the edge, 5.4 mm at 1 mm): the area the old rectangle
    // overstated.
    const auto early = roundedWindowAreaMm2(40.0, 25.0, 6.0, 0.0, 1.0);
    std::cout << "40 x 25 mm window, 6 mm top radii, 1 mm open: " << early << " mm2 against the sharp 40\n";
    require(early < 0.85 * 40.0, "a 6 mm radius takes over 15 % of the first millimetre's area");
}

[[nodiscard]] TwoStrokePortWindow window(double openDegrees, double widthMm, double topRadius = 0.0,
                                         double bottomRadius = 0.0) {
    TwoStrokePortWindow result;
    result.openDegrees = openDegrees;
    result.widthMm = widthMm;
    result.topCornerRadiusMm = topRadius;
    result.bottomCornerRadiusMm = bottomRadius;
    return result;
}

// One sharp window per group, as wide as the rectangle it replaces.
[[nodiscard]] EngineConfig asOneWindowEach(EngineConfig config) {
    const auto& cylinder = config.cylinders.front();
    config.twoStroke.exhaustWindows = { window(config.twoStroke.exhaustPortOpenDegrees,
        exhaustPortWidthMm(config.twoStroke, cylinder)) };
    config.twoStroke.transferWindows = { window(config.twoStroke.transferPortOpenDegrees,
        transferPortWidthMm(config.twoStroke, cylinder)) };
    config.twoStroke.exhaustPortWidthMm = 0.0;
    config.twoStroke.transferPortWidthMm = 0.0;
    return config;
}

struct Held final {
    double airGps { 0.0 };
    double torqueNm { 0.0 };
    double deliveredVolumetricEfficiency { 0.0 };
    double rpm { 0.0 };
    EngineState last {};
};

// Full throttle against a dyno holding `rpm`; means over the last 1 s of 5.
[[nodiscard]] Held heldFullLoad(const EngineConfig& config, double rpm) {
    SimpleEcuModel ecu;
    SimplifiedGasolinePhysics physics;
    FourStrokeEventGenerator events;
    auto exhaust = ExhaustGraph::makeForEngine(config);
    auto simulator = std::make_unique<EngineSimulator>(config, ecu, physics, events, exhaust);
    constexpr double frameSeconds = 1.0 / 240.0;
    constexpr int frames = 5 * 240;
    constexpr int firstMeasured = frames - 240;
    Held held;
    auto speed = 0.0;
    for (int frame = 0; frame < frames; ++frame) {
        const auto t = frame * frameSeconds;
        EngineControls controls;
        controls.ignitionEnabled = true;
        controls.starterEngaged = t < 1.5;
        controls.throttle = t < 2.0 ? 0.0 : 1.0;
        if (t >= 2.0) controls.dynamometerTorqueNm = std::max(0.0, 2.0 * (speed - rpm));
        const auto& result = simulator->step(frameSeconds, controls);
        speed = result.state.rpm;
        held.last = result.state;
        if (frame < firstMeasured) continue;
        held.airGps += result.state.airFlowGramsPerSecond / 240.0;
        held.torqueNm += result.state.torqueNm / 240.0;
        held.deliveredVolumetricEfficiency += result.state.deliveredVolumetricEfficiency / 240.0;
        held.rpm += result.state.rpm / 240.0;
    }
    return held;
}

void oneSharpWindowIsTheOldRectangle() {
    const auto config = rd350();
    const auto windowed = asOneWindowEach(config);
    require(!validateEngineConfig(windowed).has_value(), "one window per group validates");
    const auto& cylinder = config.cylinders.front();
    for (double phase = 0.0; phase < 360.0; phase += 0.37) {
        require(transferPortsAreaMm2(windowed.twoStroke, cylinder, phase)
                == transferPortsAreaMm2(config.twoStroke, cylinder, phase),
            "a sharp transfer window's area is the rectangle's, bit for bit");
        require(exhaustPortsAreaMm2(windowed.twoStroke, cylinder, config.twoStroke.exhaustPortOpenDegrees, phase)
                == exhaustPortsAreaMm2(config.twoStroke, cylinder, config.twoStroke.exhaustPortOpenDegrees, phase),
            "and an exhaust one's");
    }
    const auto before = heldFullLoad(config, 8'000.0);
    const auto after = heldFullLoad(windowed, 8'000.0);
    std::cout << "RD350LC at 8,000 rpm: rectangles " << before.torqueNm << " N.m, " << before.airGps
              << " g/s; one window each " << after.torqueNm << " N.m, " << after.airGps << " g/s\n";
    require(after.torqueNm == before.torqueNm && after.airGps == before.airGps
            && after.last.rpm == before.last.rpm && after.last.crankAngleDegrees == before.last.crankAngleDegrees,
        "and the running engine is the same, bit for bit");
}

void theWindowsGeometryMovesTheAir() {
    const auto config = asOneWindowEach(rd350());
    const auto sharp = heldFullLoad(config, 8'000.0);
    // The same transfer chord split into two mains and a later boost port, as
    // a five-port cylinder is laid out; then their corners rounded.
    auto split = config;
    const auto open = config.twoStroke.transferPortOpenDegrees;
    const auto chord = config.twoStroke.transferWindows.front().widthMm;
    split.twoStroke.transferWindows = { window(open, 0.375 * chord), window(open, 0.375 * chord),
                                        window(open + 4.0, 0.25 * chord) };
    require(!validateEngineConfig(split).has_value(), "a later boost window validates");
    const auto& cylinder = config.cylinders.front();
    require(transferPortsAreaMm2(split.twoStroke, cylinder, open + 2.0)
            < 0.80 * transferPortsAreaMm2(config.twoStroke, cylinder, open + 2.0),
        "before the boost window opens, the transfers have only the mains' area");
    // At bottom dead centre the whole chord is open, the boost window shorter
    // by the crown's travel between the two angles.
    const auto lost = 0.25 * chord * config.twoStroke.portDischargeCoefficient
        * (pistonTravelMm(cylinder.strokeMm, cylinder.connectingRodMm, open + 4.0)
           - pistonTravelMm(cylinder.strokeMm, cylinder.connectingRodMm, open));
    require(std::abs(transferPortsAreaMm2(split.twoStroke, cylinder, 180.0)
                     - (transferPortsAreaMm2(config.twoStroke, cylinder, 180.0) - lost)) < 1.0e-9,
        "and at bottom dead centre the whole chord's, less the boost window's shorter height");
    const auto later = heldFullLoad(split, 8'000.0);
    auto rounded = config;
    rounded.twoStroke.transferWindows.front().topCornerRadiusMm = 5.0;
    rounded.twoStroke.exhaustWindows.front().topCornerRadiusMm = 8.0;
    require(!validateEngineConfig(rounded).has_value(), "rounded corners validate");
    const auto round = heldFullLoad(rounded, 8'000.0);
    std::cout << "RD350LC at 8,000 rpm, delivered VE: sharp rectangles " << sharp.deliveredVolumetricEfficiency
              << " (" << sharp.torqueNm << " N.m); a boost window 4 deg later "
              << later.deliveredVolumetricEfficiency << " (" << later.torqueNm << " N.m); 5 and 8 mm top radii "
              << round.deliveredVolumetricEfficiency << " (" << round.torqueNm << " N.m)\n";
    require(std::abs(later.airGps / sharp.airGps - 1.0) > 0.002,
        "a window opening later moves the air, which no single rectangle could say");
    require(std::abs(round.airGps / sharp.airGps - 1.0) > 0.002, "and so do the corner radii");
}

void aDischargeMapIsReadAsWritten() {
    PortDischargeMap map;
    map.areaRatios = { 0.0, 0.5, 1.0 };
    map.pressureRatios = { 1.0, 2.0 };
    map.forwardCd = { 0.40, 0.50,
                      0.60, 0.70,
                      0.80, 0.90 };
    map.reverseCd = { 0.30, 0.30,
                      0.30, 0.30,
                      0.30, 0.30 };
    const auto near = [](double a, double b) { return std::abs(a - b) < 1.0e-12; };
    require(near(portDischargeMapCd(map, 0.5, { 2.0, true }), 0.70), "a node is its value");
    require(near(portDischargeMapCd(map, 0.25, { 1.5, true }), 0.55), "between nodes it is bilinear");
    require(near(portDischargeMapCd(map, 1.0, { 6.0, true }), 0.90), "and held past its last pressure ratio");
    require(near(portDischargeMapCd(map, 0.75, { 1.0, false }), 0.30), "the reverse map for the other way");
    map.reverseCd.clear();
    require(near(portDischargeMapCd(map, 0.75, { 1.0, false }), 0.70), "and the forward one without it");

    // On the engine: the same window with a map that halves its coefficient
    // early in the opening and keeps it at the engine's constant later.
    const auto config = asOneWindowEach(rd350());
    auto mapped = config;
    auto& transfer = mapped.twoStroke.transferWindows.front();
    const auto constant = config.twoStroke.portDischargeCoefficient;
    transfer.dischargeMap.areaRatios = { 0.0, 0.3, 1.0 };
    transfer.dischargeMap.pressureRatios = { 1.0 };
    transfer.dischargeMap.forwardCd = { 0.5 * constant, constant, constant };
    require(!validateEngineConfig(mapped).has_value(), "a window with a map validates");
    const auto& cylinder = config.cylinders.front();
    const auto phase = config.twoStroke.transferPortOpenDegrees + 1.0;
    require(transferPortsAreaMm2(mapped.twoStroke, cylinder, phase)
            < 0.75 * transferPortsAreaMm2(config.twoStroke, cylinder, phase),
        "early in the opening the map's lower coefficient gives less area");
    require(std::abs(transferPortsAreaMm2(mapped.twoStroke, cylinder, 180.0)
                     - transferPortsAreaMm2(config.twoStroke, cylinder, 180.0)) < 1.0e-9,
        "fully open the same");
    const auto constantRun = heldFullLoad(config, 8'000.0);
    const auto mappedRun = heldFullLoad(mapped, 8'000.0);
    std::cout << "RD350LC at 8,000 rpm: constant Cd " << constantRun.airGps << " g/s, mapped "
              << mappedRun.airGps << " g/s\n";
    require(std::abs(mappedRun.airGps / constantRun.airGps - 1.0) > 0.002, "and the engine's air moves");
}

void aPowerValveLowersOnlyItsWindows() {
    auto config = asOneWindowEach(rd350());
    const auto open = config.twoStroke.exhaustPortOpenDegrees;
    const auto chord = config.twoStroke.exhaustWindows.front().widthMm;
    config.twoStroke.exhaustWindows = { window(open, 0.7 * chord), window(open, 0.15 * chord),
                                        window(open, 0.15 * chord) };
    config.twoStroke.exhaustWindows.front().powerValve = true;
    config.twoStroke.powerValveClosedExhaustPortOpenDegrees = open + 10.0;
    config.twoStroke.powerValveOpeningStartRpm = 6'000.0;
    config.twoStroke.powerValveFullyOpenRpm = 8'000.0;
    require(!validateEngineConfig(config).has_value(), "a valve over the main window validates");
    const auto& cylinder = config.cylinders.front();
    const auto shutRoof = exhaustPortOpenDegreesAt(config.twoStroke, 0.0);
    const auto phase = open + 5.0;
    const auto shut = exhaustPortsAreaMm2(config.twoStroke, cylinder, shutRoof, phase);
    auto auxiliaries = config.twoStroke;
    auxiliaries.exhaustWindows.erase(auxiliaries.exhaustWindows.begin());
    require(shut > 0.0 && std::abs(shut - exhaustPortsAreaMm2(auxiliaries, cylinder, shutRoof, phase)) < 1.0e-12,
        "with the valve shut, only the auxiliary windows are open before its roof");
    config.twoStroke.exhaustWindows.front().powerValve = false;
    require(validateEngineConfig(config).has_value(), "a valve over windows none of which is under it is refused");
}

void filesKeepTheWindows() {
    auto config = asOneWindowEach(rd350());
    config.twoStroke.transferWindows = { window(config.twoStroke.transferPortOpenDegrees, 21.5, 4.25, 1.5),
                                         window(config.twoStroke.transferPortOpenDegrees + 3.0, 14.0, 2.0, 0.0) };
    auto& map = config.twoStroke.transferWindows.back().dischargeMap;
    map.areaRatios = { 0.0, 1.0 };
    map.pressureRatios = { 1.0, 1.5 };
    map.forwardCd = { 0.55, 0.6, 0.75, 0.8 };
    map.reverseCd = { 0.45, 0.5, 0.65, 0.7 };
    config.twoStroke.transferWindows.front().dischargeCoefficient = 0.66;
    require(!validateEngineConfig(config).has_value(), "the windows validate");
    const JsonEngineSerializer json;
    const YamlEngineSerializer yaml;
    for (const auto* serializer : { static_cast<const IEngineSerializer*>(&json),
                                    static_cast<const IEngineSerializer*>(&yaml) }) {
        const auto decoded = serializer->decode(serializer->encode(config));
        require(decoded && decoded.config->twoStroke.transferWindows == config.twoStroke.transferWindows
                && decoded.config->twoStroke.exhaustWindows == config.twoStroke.exhaustWindows,
            "JSON and YAML keep the windows, their radii, coefficients and maps");
        require(serializer->encode(rd350()).find("windows") == std::string::npos,
            "a file without windows writes none");
    }
}

void inconsistentWindowsAreRefused() {
    const auto good = asOneWindowEach(rd350());
    auto config = good;
    config.twoStroke.transferWindows.front().topCornerRadiusMm = 0.6 * config.twoStroke.transferWindows.front().widthMm;
    require(validateEngineConfig(config).has_value(), "a radius over half the width is refused");
    config = good;
    config.twoStroke.transferWindows.front().topCornerRadiusMm = 9.0;
    config.twoStroke.transferWindows.front().bottomCornerRadiusMm = 9.0;
    config.twoStroke.transferWindows.front().widthMm = 60.0;
    require(validateEngineConfig(config).has_value(), "radii taller together than the window are refused");
    config = good;
    config.twoStroke.transferWindows.front().openDegrees += 2.0;
    require(validateEngineConfig(config).has_value(), "a group angle that is not its earliest window's is refused");
    synchronisePortWindowTimings(config.twoStroke);
    require(!validateEngineConfig(config).has_value(), "and accepted once synchronised");
    config = good;
    config.twoStroke.exhaustPortWidthMm = 40.0;
    require(validateEngineConfig(config).has_value(), "a width beside its windows is refused (it would do nothing)");
    config = good;
    config.twoStroke.transferWindows.front().powerValve = true;
    require(validateEngineConfig(config).has_value(), "a power valve over a transfer is refused");
    config = good;
    auto& map = config.twoStroke.exhaustWindows.front().dischargeMap;
    map.areaRatios = { 0.0, 1.0 };
    map.pressureRatios = { 1.0 };
    map.forwardCd = { 0.7 };
    require(validateEngineConfig(config).has_value(), "a map with a coefficient missing is refused");
    map.forwardCd = { 0.7, 1.4 };
    require(validateEngineConfig(config).has_value(), "and one over 1");
    map.forwardCd = { 0.7, 0.8 };
    map.areaRatios = { 1.0, 0.0 };
    require(validateEngineConfig(config).has_value(), "and one whose area ratios descend");
}

// The transfer duct the runner made, as one authored duct.
[[nodiscard]] TwoStrokeTransferDuct runnerAsDuct(const EngineConfig& config) {
    const auto& cylinder = config.cylinders.front();
    const auto& geometry = config.intakePaths.empty() ? config.intake : config.intakePaths.front().geometry;
    const auto length = cylinder.intakeRunnerLengthMm > 0.0 ? cylinder.intakeRunnerLengthMm : geometry.runnerLengthMm;
    const auto port = cylinder.intakeRunnerDiameterMm > 0.0 ? cylinder.intakeRunnerDiameterMm : geometry.runnerDiameterMm;
    const auto crankcase = geometry.runnerPlenumDiameterMm > 0.0 ? geometry.runnerPlenumDiameterMm : port;
    TwoStrokeTransferDuct duct;
    duct.lengthMm = length;
    duct.portAreaMm2 = std::numbers::pi * 0.25 * port * port;
    duct.crankcaseAreaMm2 = std::numbers::pi * 0.25 * crankcase * crankcase;
    return duct;
}

// Fresh air the transfers delivered over what the carburettor metered: a
// lost or double-counted duct exchange moves it.
[[nodiscard]] double deliveredOverMetered(const EngineConfig& config, const Held& held) {
    const auto& cylinder = config.cylinders.front();
    const auto sweptM3 = std::numbers::pi * 0.25 * cylinder.boreMm * cylinder.boreMm * cylinder.strokeMm * 1.0e-9;
    const auto ambientDensity = config.ambientPressureKpa * 1'000.0 / (287.05 * (config.ambientTemperatureC + 273.15));
    const auto deliveredGps = held.deliveredVolumetricEfficiency * ambientDensity * sweptM3 * 1'000.0
        * held.rpm / 60.0 * static_cast<double>(config.cylinders.size());
    return deliveredGps / held.airGps;
}

void eachTransferDuctIsItsOwnPipe() {
    const auto windowed = asOneWindowEach(rd350());
    auto oneDuct = windowed;
    oneDuct.twoStroke.transferDucts = { runnerAsDuct(windowed) };
    require(!validateEngineConfig(oneDuct).has_value(), "the runner as one authored duct validates");
    const auto runner = heldFullLoad(windowed, 8'000.0);
    const auto authored = heldFullLoad(oneDuct, 8'000.0);
    // Two ducts, each with half the chord and half the duct's area: the same
    // total, now two pipes onto the cylinder.
    auto twoDucts = oneDuct;
    const auto open = windowed.twoStroke.transferPortOpenDegrees;
    const auto chord = windowed.twoStroke.transferWindows.front().widthMm;
    auto half = runnerAsDuct(windowed);
    half.portAreaMm2 *= 0.5;
    half.crankcaseAreaMm2 *= 0.5;
    twoDucts.twoStroke.transferDucts = { half, half };
    twoDucts.twoStroke.transferWindows = { window(open, 0.5 * chord), window(open, 0.5 * chord) };
    twoDucts.twoStroke.transferWindows.back().duct = 1;
    require(!validateEngineConfig(twoDucts).has_value(), "two ducts, a window each, validate");
    const auto& cylinder = windowed.cylinders.front();
    for (double phase = open; phase <= 180.0; phase += 7.0)
        require(std::abs(transferDuctAreaMm2(twoDucts.twoStroke, cylinder, 0, phase)
                         + transferDuctAreaMm2(twoDucts.twoStroke, cylinder, 1, phase)
                         - transferPortsAreaMm2(windowed.twoStroke, cylinder, phase)) < 1.0e-9,
            "the two ducts' windows add up to the one window's area");
    const auto split = heldFullLoad(twoDucts, 8'000.0);
    // Then the second duct alone made longer: only a duct of its own can.
    auto longer = twoDucts;
    longer.twoStroke.transferDucts.back().lengthMm *= 2.0;
    const auto lengthened = heldFullLoad(longer, 8'000.0);
    std::cout << "RD350LC at 8,000 rpm, torque / air / delivered over metered: runner " << runner.torqueNm << " / "
              << runner.airGps << " / " << deliveredOverMetered(windowed, runner) << "; one authored duct "
              << authored.torqueNm << " / " << authored.airGps << " / " << deliveredOverMetered(oneDuct, authored)
              << "; two half ducts " << split.torqueNm << " / " << split.airGps << " / "
              << deliveredOverMetered(twoDucts, split) << "; the second twice as long " << lengthened.torqueNm
              << " / " << lengthened.airGps << " / " << deliveredOverMetered(longer, lengthened) << '\n';
    require(std::abs(authored.airGps / runner.airGps - 1.0) < 0.01
            && std::abs(authored.torqueNm / runner.torqueNm - 1.0) < 0.01,
        "the runner authored as one duct runs as the runner, within 1 %");
    require(std::abs(split.airGps / authored.airGps - 1.0) < 0.03,
        "two half ducts side by side carry about what one duct of their total area does");
    require(std::abs(deliveredOverMetered(twoDucts, split) / deliveredOverMetered(oneDuct, authored) - 1.0) < 0.02,
        "and every duct's exchange reaches the cylinder once: delivered over metered within 2 % of one duct's");
    require(std::abs(lengthened.airGps / split.airGps - 1.0) > 0.003,
        "one duct's length moves the air, which a single runner for every window could not say");
    auto orphan = twoDucts;
    orphan.twoStroke.transferWindows.back().duct = 0;
    require(validateEngineConfig(orphan).has_value(), "a duct feeding no window is refused");
    orphan.twoStroke.transferWindows.back().duct = 2;
    require(validateEngineConfig(orphan).has_value(), "a window naming a missing duct is refused");
    auto noWindows = rd350();
    noWindows.twoStroke.transferDucts = { half };
    require(validateEngineConfig(noWindows).has_value(), "ducts without windows are refused");
    const JsonEngineSerializer json;
    const YamlEngineSerializer yaml;
    for (const auto* serializer : { static_cast<const IEngineSerializer*>(&json),
                                    static_cast<const IEngineSerializer*>(&yaml) }) {
        const auto decoded = serializer->decode(serializer->encode(twoDucts));
        require(decoded && decoded.config->twoStroke.transferDucts == twoDucts.twoStroke.transferDucts
                && decoded.config->twoStroke.transferWindows == twoDucts.twoStroke.transferWindows,
            "JSON and YAML keep the ducts and which window each feeds");
    }
}

} // namespace

int main() {
    aRoundedWindowIsItsExactGeometry();
    aDischargeMapIsReadAsWritten();
    aPowerValveLowersOnlyItsWindows();
    filesKeepTheWindows();
    inconsistentWindowsAreRefused();
    oneSharpWindowIsTheOldRectangle();
    theWindowsGeometryMovesTheAir();
    eachTransferDuctIsItsOwnPipe();
    std::cout << "two-stroke port windows: all checks passed\n";
    return EXIT_SUCCESS;
}
