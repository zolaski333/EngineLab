// VTEC fitted, tuned and switched from EngineLab (owner plan 2026-11-11, step
// 3), checked to work whatever happens: on every kind of engine; switched at
// idle, around and above its engagement, at the limiter, in launch control,
// through a gear shift's cut and a fuel cut; with its settings at their
// extremes; its cams fitted and taken off while the engine runs; and through
// save and reload. No sound or curve is measured: only that the engine keeps
// running and stays finite, and that its valves run the cam the ECU asks for.
// That a switch lands on a base circle is EngineLab.ValveProfileTransition's
// oracle, which this file does not repeat.

#include <enginelab/calibration/CalibrationJson.hpp>
#include <enginelab/calibration/EcuCalibration.hpp>
#include <enginelab/calibration/EcuCalibrationKeys.hpp>
#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/physics/SimplifiedGasolinePhysics.hpp>
#include <enginelab/serialization/IEngineSerializer.hpp>
#include <enginelab/serialization/JsonEngineSerializer.hpp>
#include <enginelab/serialization/YamlEngineSerializer.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>

#include <algorithm>
#include <bit>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <variant>

namespace {
using namespace enginelab;

constexpr double frameSeconds = 1.0 / 240.0;

void require(bool condition, std::string_view message) {
    if (condition) return;
    std::cerr << "FAIL: " << message << '\n';
    std::exit(EXIT_FAILURE);
}

EngineConfig catalogue(std::string_view key, const std::filesystem::path& root = ENGINELAB_CATALOG_ROOT) {
    static const auto shipped = loadEngineCatalog(ENGINELAB_CATALOG_ROOT);
    const auto other = root == std::filesystem::path(ENGINELAB_CATALOG_ROOT) ? EngineCatalogLoadResult {}
                                                                              : loadEngineCatalog(root);
    const auto& loaded = other.entries.empty() ? shipped : other;
    const auto selected = selectSingleEngineCatalogEntry(loaded.entries, key);
    require(static_cast<bool>(selected), "the catalogue engine loads");
    return selected.entry->config;
}

/** The torture W16 reads the catalogue's parts: copied beside it. */
EngineConfig tortureW16() {
    const auto root = std::filesystem::temp_directory_path() / "enginelab-vtec-w16";
    std::filesystem::remove_all(root);
    std::filesystem::create_directories(root / "engines");
    std::filesystem::copy(std::filesystem::path(ENGINELAB_CATALOG_ROOT) / "parts", root / "parts",
                          std::filesystem::copy_options::recursive);
    std::filesystem::copy_file(std::filesystem::path(ENGINELAB_CATALOG_ROOT)
                                   / "tools/torture-engines/engines/t7_w16_8l.engine.yaml",
                               root / "engines/t7_w16_8l.engine.yaml");
    auto config = catalogue("t7_w16_8l", root);
    std::filesystem::remove_all(root);
    return config;
}

std::uint32_t fittedCylinders(const EngineConfig& config) {
    std::uint32_t mask = 0U;
    for (std::size_t index = 0; index < config.cylinders.size(); ++index)
        if (camshaftOfCylinder(config, config.cylinders[index]).variableProfileEnabled) mask |= 1U << index;
    return mask;
}

struct Rig final {
    explicit Rig(EngineConfig engine)
        : config(std::move(engine)), exhaust(ExhaustGraph::makeForEngine(config)),
          simulator(std::make_unique<EngineSimulator>(config, ecu, physics, events, exhaust)) {
        config = simulator->config();
        ecu.initialiseCalibration(config);
    }
    EngineConfig config;
    SimpleEcuModel ecu;
    SimplifiedGasolinePhysics physics;
    FourStrokeEventGenerator events;
    ExhaustGraph exhaust;
    std::unique_ptr<EngineSimulator> simulator;
    EngineControls controls;
    EngineState state;
    double time { 0.0 };

    void set(std::string_view key, double value) {
        auto draft = calibration::makeDraft(*ecu.calibrationStore()->snapshot());
        const auto* found = draft.find(key);
        require(found != nullptr, "the tables of an engine with VTEC cams carry the VTEC settings");
        auto entry = *found;
        std::get<calibration::ScalarCalibration>(entry).value = value;
        draft.set(std::move(entry));
        require(ecu.calibrationStore()->publish(draft).published, "a VTEC setting within its range publishes");
    }
    /** The engine runs `seconds`; every frame stays finite. */
    void run(double seconds) {
        const auto frames = static_cast<int>(std::ceil(seconds / frameSeconds));
        for (int frame = 0; frame < frames; ++frame) {
            state = simulator->step(frameSeconds, controls).state;
            time += frameSeconds;
            require(std::isfinite(state.rpm) && std::isfinite(state.torqueNm)
                        && std::isfinite(state.manifoldPressureKpa) && std::isfinite(state.crankAngleDegrees)
                        && std::isfinite(state.vtecEngageRpm),
                    "the engine stays finite");
            require((state.vtecHighCylinderMask & ~lastMask & ~fittedCylinders(config)) == 0U,
                    "only a cylinder with VTEC cams engages its high cam");
            lastMask = state.vtecHighCylinderMask;
        }
    }
    /** Fits or removes the cams live, as the inspector or the tuner does. */
    void changeCams(bool fit) {
        auto target = config;
        if (fit) require(fitVtecCams(target), "VTEC cams fit");
        else removeVtecCams(target);
        require(EngineSimulator::camshaftsReplaceable(config, target), "VTEC cams change live");
        require(simulator->beginCamshaftChange(target, 0.0), "the running engine takes the cams");
        config = simulator->config();
        auto draft = calibration::makeDraft(*ecu.calibrationStore()->snapshot());
        if (calibration::syncVtecSettings(draft, config))
            require(ecu.calibrationStore()->publish(draft).published, "the tables follow the cams");
    }
    [[nodiscard]] bool allHigh() const { return state.vtecHighCylinderMask == fittedCylinders(config); }
    [[nodiscard]] bool allLow() const { return state.vtecHighCylinderMask == 0U; }
    std::uint32_t lastMask { 0U };
};

void calibrationFollowsTheCams() {
    const auto k20 = catalogue("01_honda_k20a_like");
    const auto* cam = firstVtecCamshaft(k20);
    require(cam != nullptr && cam->switchRpm == 5'800.0 && cam->switchThrottle == 0.55,
            "the catalogue K20A carries VTEC cams at 5,800 rpm and 55 %");
    const auto tables = calibration::makeDefaultEcuCalibration(k20);
    const auto scalar = [&](const calibration::CalibrationDraft& draft, std::string_view key) {
        const auto* entry = draft.find(key);
        require(entry != nullptr, "the VTEC setting is in the tables");
        return std::get<calibration::ScalarCalibration>(*entry).value;
    };
    require(scalar(tables, calibration::keys::vtecEnabled) == 1.0
                && scalar(tables, calibration::keys::vtecEngage) == 5'800.0
                && scalar(tables, calibration::keys::vtecHysteresis) == 0.0
                && scalar(tables, calibration::keys::vtecMinimumThrottle) == 55.0,
            "the K20A's tables start where its cams were authored: on, 5,800 rpm, no hysteresis, 55 %");
    // The ECU hands the simulator the cam's own figures to the bit: the
    // catalogue K20A runs as before.
    SimpleEcuModel ecu;
    ecu.initialiseCalibration(k20);
    EngineState state;
    state.rpm = 3'000.0;
    state.coolantTemperatureC = 90.0;
    EngineControls controls;
    controls.ignitionEnabled = true;
    const auto command = ecu.evaluate(k20, state, controls);
    require(command.vtecEnabled && command.vtecEngageRpm == cam->switchRpm && command.vtecHysteresisRpm == 0.0
                && std::bit_cast<std::uint64_t>(command.vtecMinimumThrottle)
                       == std::bit_cast<std::uint64_t>(cam->switchThrottle),
            "the default tables give the cam's own switch, bit for bit");

    auto ls3 = catalogue("03_gm_ls3_like");
    require(!vtecCamsFitted(ls3) && calibration::makeDefaultEcuCalibration(ls3).find(calibration::keys::vtecEnabled)
                == nullptr,
            "an engine without VTEC cams has no VTEC settings");
    auto draft = calibration::makeDefaultEcuCalibration(ls3);
    require(fitVtecCams(ls3) && vtecCamsFitted(ls3) && calibration::syncVtecSettings(draft, ls3)
                && draft.find(calibration::keys::vtecEngage) != nullptr,
            "fitting VTEC cams gives the tables their VTEC settings");
    require(scalar(draft, calibration::keys::vtecEngage)
                == std::round(ls3.ignition.revLimitRpm * 2.0 / 3.0 / 50.0) * 50.0,
            "fitted VTEC engages at two thirds of the rev limit");
    require(!calibration::syncVtecSettings(draft, ls3), "a second sync changes nothing");
    removeVtecCams(ls3);
    require(!vtecCamsFitted(ls3) && calibration::syncVtecSettings(draft, ls3)
                && draft.find(calibration::keys::vtecEnabled) == nullptr
                && draft.find(calibration::keys::vtecMinimumThrottle) == nullptr,
            "removing them takes the settings away");

    // Tables saved before VTEC settings existed take them on load.
    auto older = calibration::makeDefaultEcuCalibration(k20);
    for (const auto key : { calibration::keys::vtecEnabled, calibration::keys::vtecEngage,
                            calibration::keys::vtecHysteresis, calibration::keys::vtecMinimumThrottle })
        (void)older.erase(key);
    calibration::addMissingDefaultSettings(older, k20);
    require(older.find(calibration::keys::vtecHysteresis) != nullptr, "an older table gains the VTEC settings");

    // Every setting takes its whole range, and nothing past it.
    calibration::CalibrationStore store;
    require(store.publish(tables).published, "the K20A's tables publish");
    const auto publishWith = [&](std::string_view key, double value) {
        auto edited = calibration::makeDraft(*store.snapshot());
        auto entry = *edited.find(key);
        std::get<calibration::ScalarCalibration>(entry).value = value;
        edited.set(std::move(entry));
        return store.publish(edited).published;
    };
    require(publishWith(calibration::keys::vtecEngage, 0.0) && publishWith(calibration::keys::vtecEngage, 25'000.0)
                && publishWith(calibration::keys::vtecHysteresis, 25'000.0)
                && publishWith(calibration::keys::vtecMinimumThrottle, 0.0)
                && publishWith(calibration::keys::vtecMinimumThrottle, 100.0)
                && publishWith(calibration::keys::vtecEnabled, 0.0),
            "engagement from 0 to 25,000 rpm, hysteresis up to 25,000, throttle 0 to 100 %, a switch");
    require(!publishWith(calibration::keys::vtecMinimumThrottle, 150.0) && !publishWith(calibration::keys::vtecEngage, -5.0),
            "150 % throttle or a negative speed are refused");

    // Save and reload: the cams in both engine formats, the settings in the
    // ECU file.
    auto edited = k20;
    edited.camshafts.highIntakeDurationDegrees = 300.0;
    for (auto& bank : edited.banks) bank.camshafts.highIntakeDurationDegrees = 300.0;
    const JsonEngineSerializer json;
    const YamlEngineSerializer yaml;
    for (const IEngineSerializer* serializer : { static_cast<const IEngineSerializer*>(&json),
                                                 static_cast<const IEngineSerializer*>(&yaml) }) {
        const auto decoded = serializer->decode(serializer->encode(edited));
        require(static_cast<bool>(decoded) && vtecCamsFitted(*decoded.config)
                    && firstVtecCamshaft(*decoded.config)->highIntakeDurationDegrees == 300.0,
                "an engine's VTEC cams survive save and reload");
        auto off = edited;
        removeVtecCams(off);
        const auto reloaded = serializer->decode(serializer->encode(off));
        require(static_cast<bool>(reloaded) && !vtecCamsFitted(*reloaded.config),
                "removed VTEC cams stay removed");
    }
    require(publishWith(calibration::keys::vtecEngage, 4'321.0), "an edited engagement publishes");
    const auto saved = calibration::CalibrationJson::serialize(*store.snapshot());
    const auto parsed = calibration::CalibrationJson::parse(saved);
    require(parsed.hasValue() && scalar(parsed.value(), calibration::keys::vtecEngage) == 4'321.0
                && scalar(parsed.value(), calibration::keys::vtecEnabled) == 0.0,
            "the VTEC settings survive the ECU file");

    // What fitting lays out: the K20A's cam over the K20A's own low cam; a
    // rotary's exhaust port keeps its timing; a two-stroke has no cam.
    auto bare = k20;
    removeVtecCams(bare);
    require(fitVtecCams(bare), "the cams fit again");
    const auto& refit = *firstVtecCamshaft(bare);
    require(refit.highIntakeDurationDegrees == 292.0 && refit.highExhaustDurationDegrees == 286.0
                && std::abs(refit.highIntakeLiftMm - 13.2) < 1.0e-9 && std::abs(refit.highExhaustLiftMm - 12.4) < 1.0e-9
                && refit.switchRpm == 5'800.0 && refit.switchThrottle == 0.55,
            "fitted to the K20A's low cam, they are the K20A's VTEC cams");
    require(!fitVtecCams(bare), "a camshaft that has them keeps them");
    auto rotary = catalogue("18_mazda_13b_renesis_like");
    require(!vtecCamsFitted(rotary) && fitVtecCams(rotary), "a rotary takes staged ports");
    const auto& ports = *firstVtecCamshaft(rotary);
    require(ports.highExhaustDurationDegrees == ports.exhaustDurationDegrees
                && ports.highIntakeDurationDegrees == ports.intakeDurationDegrees + 24.0
                && !validateEngineConfig(rotary).has_value(),
            "a rotary's staged intake ports open longer, its exhaust port is unchanged");
    auto twoStroke = catalogue("17_yamaha_rd350lc_like");
    require(!fitVtecCams(twoStroke) && !vtecCamsFitted(twoStroke), "a two-stroke has no cam to switch");
    std::cout << "PASS calibration-follows-the-cams\n";
}

/** The whole set of switches on one engine. */
void robustOn(std::string_view name, EngineConfig config, bool full) {
    const auto started = std::chrono::steady_clock::now();
    if (!vtecCamsFitted(config)) require(fitVtecCams(config), "VTEC cams fit");
    require(!validateEngineConfig(config).has_value(), "an engine with VTEC cams validates");
    auto rig = std::make_unique<Rig>(config);
    const auto engage = firstVtecCamshaft(rig->config)->switchRpm;
    const auto idle = std::max(300.0, rig->config.idleRpm);
    auto& controls = rig->controls;
    controls.ignitionEnabled = true;
    controls.starterEngaged = true;
    rig->run(1.5);
    controls.starterEngaged = false;
    rig->run(2.0);
    require(rig->state.rpm > 0.5 * idle, "the engine starts and idles");
    require(rig->state.vtecEnabled && rig->allLow(), "it idles on the low cam");

    // At idle: off and on again.
    rig->set(calibration::keys::vtecEnabled, 0.0);
    rig->run(0.4);
    rig->set(calibration::keys::vtecEnabled, 1.0);
    rig->run(0.4);
    require(rig->state.rpm > 0.5 * idle && rig->allLow(), "switched at idle it idles on");

    // Full throttle to the limiter: the high cam engages, the switch takes
    // it off and gives it back, and launch control holds a speed above it.
    controls.throttle = 1.0;
    rig->run(2.5);
    const auto top = rig->state.rpm;
    const auto gasoline = rig->config.fuel != FuelType::diesel;
    if (top > engage * 1.05) require(rig->allHigh(), "well above its engagement it runs the high cam");
    rig->set(calibration::keys::vtecEnabled, 0.0);
    rig->run(0.5);
    require(rig->allLow() && !rig->state.vtecEnabled, "switched off at the top it drops back");
    rig->set(calibration::keys::vtecEnabled, 1.0);
    rig->run(0.5);
    if (top > engage * 1.05) require(rig->allHigh(), "and switched on it engages again");

    // A gear shift's injection cut and retard, with the switch thrown inside it.
    controls.shiftCylinderCutFraction = 0.5;
    controls.shiftSparkRetardDegrees = 10.0;
    rig->set(calibration::keys::vtecEnabled, 0.0);
    rig->run(0.15);
    rig->set(calibration::keys::vtecEnabled, 1.0);
    rig->run(0.15);
    controls.shiftCylinderCutFraction = 0.0;
    controls.shiftSparkRetardDegrees = 0.0;

    // Off the throttle from the top: the fuel cut, then idle again.
    controls.throttle = 0.0;
    rig->set(calibration::keys::vtecEnabled, 0.0);
    rig->run(0.3);
    rig->set(calibration::keys::vtecEnabled, 1.0);
    rig->run(3.0);
    require(rig->state.rpm > 0.5 * idle && rig->allLow(), "after the fuel cut it idles on the low cam");

    // The cams come off at full throttle, high cam running, and go back on.
    controls.throttle = 1.0;
    rig->run(1.5);
    rig->changeCams(false);
    rig->run(0.5);
    require(rig->allLow() && !vtecCamsFitted(rig->config) && !rig->state.vtecEnabled,
            "cams taken off at speed drop back to the low cam");
    rig->changeCams(true);
    require(rig->simulator->state().vtecHighCylinderMask == 0U, "cams fitted live start on the low cam");
    rig->run(1.0);
    if (rig->state.rpm > engage * 1.05) require(rig->allHigh(), "and engage once whole cycles run above the switch");
    controls.throttle = 0.0;
    rig->run(2.5);
    require(rig->state.rpm > 0.5 * idle, "back to idle");

    if (full) {
        // Launch control: in first, standing, foot down, held above the switch.
        auto draft = calibration::makeDraft(*rig->ecu.calibrationStore()->snapshot());
        auto launch = *draft.find(calibration::keys::launchLimit);
        std::get<calibration::ScalarCalibration>(launch).value = std::min(engage + 800.0, top - 200.0);
        draft.set(std::move(launch));
        require(rig->ecu.calibrationStore()->publish(draft).published, "launch control sets");
        controls.engagedGear = 0;
        controls.vehicleSpeedMps = 0.0;
        controls.throttle = 1.0;
        rig->run(1.5);
        rig->set(calibration::keys::vtecEnabled, 0.0);
        rig->run(0.4);
        require(rig->allLow(), "switched off in launch control it drops back");
        rig->set(calibration::keys::vtecEnabled, 1.0);
        rig->run(0.6);
        controls.engagedGear = -1;
        controls.throttle = 0.0;
        rig->run(2.5);

        // The settings at their extremes.
        rig->set(calibration::keys::vtecEngage, 0.5 * idle);
        rig->set(calibration::keys::vtecMinimumThrottle, 0.0);
        rig->run(1.0);
        require(rig->allHigh(), "engaged under idle and at any throttle, it idles on the high cam");
        require(rig->state.rpm > 0.3 * idle, "and keeps running");
        rig->set(calibration::keys::vtecHysteresis, 25'000.0);
        rig->set(calibration::keys::vtecEngage, engage);
        controls.throttle = 1.0;
        rig->run(1.0);
        controls.throttle = 0.0;
        rig->run(2.5);
        require(rig->allHigh(), "a hysteresis past the engagement never drops it by speed");
        rig->set(calibration::keys::vtecHysteresis, 0.0);
        rig->set(calibration::keys::vtecMinimumThrottle, 55.0);
        rig->run(1.0);
        require(rig->allLow() && rig->state.rpm > 0.5 * idle, "with the settings back it idles on the low cam");
        rig->set(calibration::keys::vtecEngage, 25'000.0);
        controls.throttle = 1.0;
        rig->run(1.5);
        require(rig->allLow(), "engaged above the limiter it never engages");
        rig->set(calibration::keys::vtecEngage, engage);
        rig->set(calibration::keys::vtecMinimumThrottle, 100.0);
        controls.throttle = 0.99;
        rig->run(1.0);
        require(rig->allLow(), "at 100 % minimum throttle, 99 % is not enough");
        controls.throttle = 1.0;
        rig->run(1.0);
        if (rig->state.rpm > engage * 1.05) require(rig->allHigh(), "and the whole pedal is");
        controls.throttle = 0.0;
        rig->set(calibration::keys::vtecMinimumThrottle, 55.0);
        rig->run(2.5);
        require(rig->state.rpm > 0.5 * idle, "it idles at the end");
    }
    const auto seconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - started).count();
    std::cout << "PASS " << name << " engage " << engage << " rpm, top " << top << " rpm" << (gasoline ? "" : " (diesel)")
              << ", " << rig->time << " s simulated in " << seconds << " s\n";
}

/** A cam smaller than the low, and per-bank cams of their own. */
void oddCams() {
    auto smaller = catalogue("01_honda_k20a_like");
    const auto shrink = [](CamshaftConfig& cam) {
        cam.highIntakeDurationDegrees = cam.intakeDurationDegrees - 40.0;
        cam.highExhaustDurationDegrees = cam.exhaustDurationDegrees - 40.0;
        cam.highIntakeLiftMm = cam.intakeLiftMm - 3.0;
        cam.highExhaustLiftMm = cam.exhaustLiftMm - 3.0;
        cam.highIntakeLiftProfile.clear();
        cam.highExhaustLiftProfile.clear();
    };
    shrink(smaller.camshafts);
    for (auto& bank : smaller.banks) shrink(bank.camshafts);
    robustOn("K20A high cam smaller than the low", smaller, false);

    auto v8 = makeDefaultV8();
    require(v8.banks.size() == 2, "the default V8 has two banks");
    for (auto& bank : v8.banks) bank.camshafts = v8.camshafts;
    v8.banks[1].camshafts.intakeDurationDegrees += 12.0;
    require(fitVtecCams(v8), "both banks take VTEC cams");
    require(v8.banks[1].camshafts.highIntakeDurationDegrees == v8.banks[1].camshafts.intakeDurationDegrees + 24.0
                && v8.banks[0].camshafts.highIntakeDurationDegrees == v8.banks[0].camshafts.intakeDurationDegrees + 24.0,
            "each bank's VTEC cam is laid over its own cam");
    robustOn("V8 with a cam per bank", v8, false);
}

} // namespace

int main() {
    calibrationFollowsTheCams();
    robustOn("K20A (inline, VTEC and VVT)", catalogue("01_honda_k20a_like"), true);
    oddCams();
    robustOn("2JZ (turbo)", catalogue("02_toyota_2jz_gte_like"), false);
    robustOn("EJ25 (flat, turbo)", catalogue("04_subaru_ej25_like"), false);
    robustOn("radial five", catalogue("10_radial5_like"), false);
    robustOn("TDI (diesel)", catalogue("14_vw_2_0_tdi_like"), false);
    robustOn("13B (rotary, staged ports)", catalogue("18_mazda_13b_renesis_like"), false);
    robustOn("W16 (torture)", tortureW16(), false);
    return EXIT_SUCCESS;
}
