// Shift styles: how a gear change is made (TransmissionConfig::shiftStyle).
//
// A real engine, ECU and driveline are coupled the way EngineRuntime couples
// them (the driveline advances on the previous frame's engine state; a blip
// opens the throttle past the driver's; a quickshifter cut reaches the ECU),
// and one shift is measured on its result, not on its implementation:
//  - a quickshifter upshift cuts the ignition for its cut length, keeps the
//    drive gap well below a clutch shift's, and lands synchronised;
//  - a dual-clutch upshift has no torque gap at all;
//  - an auto-blipped downshift brings the engine to the new gear's speed
//    before the dogs engage, where a clutch downshift without it slips hard.
// The references are the behaviour of the real devices (an ignition cut of a
// few tens of ms, no gap for a dual clutch, a matched blip), not numbers read
// back from this simulator.

#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/physics/SimplifiedGasolinePhysics.hpp>
#include <enginelab/runtime/DriverCommandArbiter.hpp>
#include <enginelab/runtime/DrivelineModel.hpp>
#include <enginelab/serialization/JsonEngineSerializer.hpp>
#include <enginelab/serialization/YamlEngineSerializer.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <numbers>
#include <string>
#include <string_view>

namespace {
constexpr double stepSeconds = 1.0 / 240.0;

void require(bool condition, std::string_view message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(EXIT_FAILURE);
    }
}

const enginelab::EngineConfig& findEngine(
    const enginelab::EngineCatalogLoadResult& catalog, std::string_view fragment) {
    const auto found = std::find_if(catalog.entries.begin(), catalog.entries.end(),
        [&](const auto& entry) { return entry.config.name.find(fragment) != std::string::npos; });
    require(found != catalog.entries.end(), "expected engine must exist in the catalogue");
    return found->config;
}

double rpmFromOmega(double omega) { return omega * 60.0 / (2.0 * std::numbers::pi); }

/** One engine, ECU and driveline, stepped as EngineRuntime steps them. */
class Rig final {
public:
    explicit Rig(const enginelab::EngineConfig& config)
        : config_(config), exhaust_(enginelab::ExhaustGraph::makeForEngine(config)),
          simulator_(config, ecu_, physics_, events_, exhaust_), driveline_(config) {}

    struct Tick final {
        enginelab::DrivelineOutput drive;
        enginelab::EngineState state;
    };

    Tick step(double throttle, double clutch, bool starter = false, double brake = 0.0) {
        Tick tick;
        const auto vehicle = enginelab::advanceVehicle(driveline_, stepSeconds, simulator_.state(), 0.0,
            enginelab::DriverInputs { throttle, clutch, brake });
        tick.drive = vehicle.drive;
        enginelab::EngineControls controls;
        controls.ignitionEnabled = true;
        controls.starterEngaged = starter;
        enginelab::applyVehicleStep(controls, vehicle);
        tick.state = simulator_.step(stepSeconds, controls).state;
        return tick;
    }

    /** Starts the engine, idles in neutral, then launches in first at full
     * throttle and holds it there until `rpm`. */
    void launchTo(double rpm) {
        driveline_.requestGear(-1);
        for (int index = 0; index < static_cast<int>(8.0 / stepSeconds); ++index) {
            const auto time = index * stepSeconds;
            step(0.0, 0.0, simulator_.state().rpm < config_.idleRpm * 0.85 && time < 3.0);
        }
        driveline_.requestGear(0);
        const auto launchSteps = static_cast<int>(0.9 / stepSeconds);
        for (int index = 0; index < launchSteps; ++index)
            step(1.0, static_cast<double>(index) / launchSteps);
        for (int index = 0; index < static_cast<int>(10.0 / stepSeconds); ++index) {
            const auto tick = step(1.0, 1.0);
            if (tick.state.rpm >= rpm && !tick.drive.shiftInProgress
                    && std::abs(tick.drive.clutchSlipRpm) < 2.0 * config_.transmission.clutchLockSpeedRpm)
                return;
        }
        require(false, config_.name + " must reach the shift speed in first");
    }

    enginelab::DrivelineModel& driveline() { return driveline_; }
    double rpm() const { return simulator_.state().rpm; }
    const enginelab::EngineConfig& config() const { return config_; }
    double wheelOmega(const Tick& tick) const {
        return tick.drive.vehicleSpeedMps / config_.vehicle.tireRadiusM;
    }
    double gearRatio(int gear) const {
        return config_.transmission.gearRatios[static_cast<std::size_t>(gear)]
            * config_.transmission.finalDriveRatio;
    }

private:
    enginelab::EngineConfig config_;
    enginelab::SimpleEcuModel ecu_;
    enginelab::SimplifiedGasolinePhysics physics_;
    enginelab::FourStrokeEventGenerator events_;
    enginelab::ExhaustGraph exhaust_;
    enginelab::EngineSimulator simulator_;
    enginelab::DrivelineModel driveline_;
};

struct UpshiftResult final {
    double cutSeconds { 0.0 };        // frames with the ignition cut
    bool sparkCutSeen { false };      // the ECU actually dropped the spark
    double gapSeconds { 0.0 };        // wheel torque below half the new gear's
    double syncSeconds { -1.0 };      // to the new gear, clutch locked
    double minimumWheelShare { 1.0 }; // lowest wheel torque / new gear's
    double syncErrorShare { 1.0 };    // |engine - new gear speed| / that speed
    double peakWheelShare { 0.0 };    // highest wheel torque / new gear's
    double pushImpulse { 0.0 };       // integral of the excess over it, s
    double advanceBefore { 0.0 };     // ignition advance before the shift
    double lowestAdvance { 0.0 };     // lowest advance until synchronised
};

UpshiftResult measureUpshift(enginelab::EngineConfig config, enginelab::ShiftStyle style,
                             double dualClutchRetardDegrees = -1.0, double shiftAtRedlineShare = 0.70) {
    config.transmission.shiftStyle = style;
    if (dualClutchRetardDegrees >= 0.0)
        config.transmission.dualClutchUpshiftRetardDegrees = dualClutchRetardDegrees;
    auto rig = std::make_unique<Rig>(config);
    rig->launchTo(config.redlineRpm * shiftAtRedlineShare);
    // Wheel torque before the shift, over the last 0.1 s.
    double before = 0.0;
    double advanceBefore = 0.0;
    const auto beforeSteps = static_cast<int>(0.1 / stepSeconds);
    for (int index = 0; index < beforeSteps; ++index) {
        const auto tick = rig->step(1.0, 1.0);
        before += tick.drive.wheelTorqueNm / beforeSteps;
        advanceBefore += tick.state.ignitionAdvanceDegrees / beforeSteps;
    }
    const auto expectedAfter = before * rig->gearRatio(1) / rig->gearRatio(0);
    rig->driveline().shiftUp();
    UpshiftResult result;
    result.advanceBefore = advanceBefore;
    result.lowestAdvance = advanceBefore;
    const auto lockBand = 2.0 * config.transmission.clutchLockSpeedRpm;
    for (int index = 0; index < static_cast<int>(1.0 / stepSeconds); ++index) {
        const auto tick = rig->step(1.0, 1.0);
        if (tick.drive.ignitionCut) {
            result.cutSeconds += stepSeconds;
            result.sparkCutSeen = result.sparkCutSeen || !tick.state.ecuSparkEnabled;
        }
        const auto share = tick.drive.wheelTorqueNm / std::max(1.0, expectedAfter);
        if (result.syncSeconds < 0.0) {
            result.minimumWheelShare = std::min(result.minimumWheelShare, share);
            result.peakWheelShare = std::max(result.peakWheelShare, share);
            result.pushImpulse += std::max(0.0, share - 1.0) * stepSeconds;
            result.lowestAdvance = std::min(result.lowestAdvance, tick.state.ignitionAdvanceDegrees);
            if (share < 0.5) result.gapSeconds += stepSeconds;
        }
        if (result.syncSeconds < 0.0 && tick.drive.engagedGear == 1 && !tick.drive.shiftInProgress
                && std::abs(tick.drive.clutchSlipRpm) < lockBand) {
            result.syncSeconds = (index + 1) * stepSeconds;
            const auto gearRpm = rpmFromOmega(rig->wheelOmega(tick) * rig->gearRatio(1));
            result.syncErrorShare = std::abs(tick.state.rpm - gearRpm) / std::max(1.0, gearRpm);
        }
    }
    std::printf("  %-34s %-11s cut %5.1f ms spark-off %d gap %5.1f ms sync %6.1f ms"
                " wheel %5.2f-%5.2f push %5.1f ms sync err %5.3f advance %5.1f -> %5.1f deg\n",
        config.name.c_str(), enginelab::shiftStyleName(style), result.cutSeconds * 1000.0,
        result.sparkCutSeen ? 1 : 0, result.gapSeconds * 1000.0, result.syncSeconds * 1000.0,
        result.minimumWheelShare, result.peakWheelShare, result.pushImpulse * 1000.0, result.syncErrorShare,
        result.advanceBefore,
        result.lowestAdvance);
    return result;
}

struct DownshiftResult final {
    double engagedRpmShare { 0.0 }; // engine speed / new gear's at engagement
    double peakSlipRpm { 0.0 };     // after the new gear engages
    double peakBlip { 0.0 };
};

/** Third gear at part throttle, closed throttle, then down to second. */
DownshiftResult measureDownshift(enginelab::EngineConfig config, bool blipper) {
    config.transmission.shiftStyle = enginelab::ShiftStyle::sequential;
    config.transmission.autoBlipper = blipper;
    auto rig = std::make_unique<Rig>(config);
    rig->launchTo(config.redlineRpm * 0.55);
    rig->driveline().requestGear(2);
    for (int index = 0; index < static_cast<int>(1.5 / stepSeconds); ++index) rig->step(0.35, 1.0);
    for (int index = 0; index < static_cast<int>(0.3 / stepSeconds); ++index) rig->step(0.0, 1.0);
    rig->driveline().shiftDown();
    DownshiftResult result;
    auto engaged = false;
    for (int index = 0; index < static_cast<int>(0.6 / stepSeconds); ++index) {
        const auto tick = rig->step(0.0, 1.0);
        result.peakBlip = std::max(result.peakBlip, tick.drive.blipThrottle);
        // From the moment second gear carries the drive: halfway through a
        // clutch shift's timer, at the dogs' engagement for a blipped one.
        if (!engaged && tick.drive.engagedGear == 1) {
            engaged = true;
            const auto gearRpm = rpmFromOmega(rig->wheelOmega(tick) * rig->gearRatio(1));
            result.engagedRpmShare = tick.state.rpm / std::max(1.0, gearRpm);
        }
        if (engaged) result.peakSlipRpm = std::max(result.peakSlipRpm, std::abs(tick.drive.clutchSlipRpm));
    }
    require(engaged, config.name + " must complete the downshift");
    std::printf("  %-34s blipper %d engaged at %5.3f of the gear's speed, peak slip %6.0f rpm,"
                " peak blip %4.2f\n",
        config.name.c_str(), blipper ? 1 : 0, result.engagedRpmShare, result.peakSlipRpm,
        result.peakBlip);
    return result;
}

/** Third gear, throttle shut, the driver pulls the clutch and changes down
 * with it held out: the gear must change at once and no blipper may act. */
void checkClutchHeldDownshift(enginelab::EngineConfig config) {
    config.transmission.shiftStyle = enginelab::ShiftStyle::sequential;
    config.transmission.autoBlipper = true;
    auto rig = std::make_unique<Rig>(config);
    rig->launchTo(config.redlineRpm * 0.55);
    rig->driveline().requestGear(2);
    for (int index = 0; index < static_cast<int>(1.5 / stepSeconds); ++index) rig->step(0.35, 1.0);
    Rig::Tick before {};
    // Long enough for the engine to fall below the lower gear's speed, so a
    // blipper would have something to do.
    for (int index = 0; index < static_cast<int>(1.0 / stepSeconds); ++index) before = rig->step(0.0, 0.0);
    std::printf("  before the downshift: gear %d, %.0f rpm, %.1f m/s\n",
        before.drive.engagedGear + 1, before.state.rpm, before.drive.vehicleSpeedMps);
    rig->driveline().shiftDown();
    double peakBlip = 0.0;
    int framesToEngage = -1;
    for (int index = 0; index < static_cast<int>(0.5 / stepSeconds); ++index) {
        const auto tick = rig->step(0.0, 0.0);
        peakBlip = std::max(peakBlip, tick.drive.blipThrottle);
        if (framesToEngage < 0 && tick.drive.engagedGear == 1) framesToEngage = index + 1;
    }
    std::printf("  %-34s clutch held: engaged after %d frame(s), peak blip %4.2f\n",
        config.name.c_str(), framesToEngage, peakBlip);
    require(peakBlip == 0.0, "no blip while the driver holds the clutch out");
    require(framesToEngage == 1, "with the clutch held out the gear changes at once");
}

/** A dual clutch ignores the pedal: in first at idle, foot off everything,
 * it creeps; on the brake it stops; it never stalls. */
void checkDualClutchCreep(enginelab::EngineConfig config) {
    config.transmission.shiftStyle = enginelab::ShiftStyle::dualClutch;
    config.transmission.automaticShifting = false;
    auto rig = std::make_unique<Rig>(config);
    rig->driveline().requestGear(-1);
    for (int index = 0; index < static_cast<int>(6.0 / stepSeconds); ++index)
        rig->step(0.0, 0.0, rig->rpm() < config.idleRpm * 0.85 && index * stepSeconds < 3.0);
    rig->driveline().requestGear(0);
    double lowestRpm = 1.0e9;
    Rig::Tick last {};
    for (int index = 0; index < static_cast<int>(8.0 / stepSeconds); ++index) {
        last = rig->step(0.0, 0.0);
        lowestRpm = std::min(lowestRpm, last.state.rpm);
    }
    const auto creepSpeed = last.drive.vehicleSpeedMps;
    for (int index = 0; index < static_cast<int>(6.0 / stepSeconds); ++index) {
        last = rig->step(0.0, 0.0, false, 1.0);
        lowestRpm = std::min(lowestRpm, last.state.rpm);
    }
    std::printf("  %-34s dual clutch creep %.2f m/s, braked %.2f m/s, lowest %4.0f rpm\n",
        config.name.c_str(), creepSpeed, last.drive.vehicleSpeedMps, lowestRpm);
    require(creepSpeed > 0.5, "a dual clutch creeps at idle with the pedal ignored");
    require(std::abs(last.drive.vehicleSpeedMps) < 0.05, "on the brake a creeping dual clutch stops");
    require(lowestRpm > 0.6 * config.idleRpm, "a dual clutch never stalls the engine");
}

/** Second gear high in the revs, first requested: a dual clutch refuses a
 * downshift that would over-rev the engine; a manual box lets the driver do
 * it (the money shift). Returns the gear engaged after the request. */
int overRevDownshift(enginelab::EngineConfig config, enginelab::ShiftStyle style) {
    config.transmission.shiftStyle = style;
    config.transmission.automaticShifting = false;
    auto rig = std::make_unique<Rig>(config);
    rig->launchTo(config.redlineRpm * 0.55);
    rig->driveline().requestGear(1);
    for (int index = 0; index < static_cast<int>(6.0 / stepSeconds); ++index) {
        const auto tick = rig->step(1.0, 1.0);
        if (tick.drive.engagedGear == 1 && !tick.drive.shiftInProgress
                && tick.state.rpm > 0.75 * config.redlineRpm) break;
    }
    rig->driveline().requestGear(0);
    int engaged = -1;
    for (int index = 0; index < static_cast<int>(1.0 / stepSeconds); ++index)
        engaged = rig->step(0.0, 1.0).drive.engagedGear;
    std::printf("  %-34s %-11s second at 75 %% of redline, first requested: gear %d engaged\n",
        config.name.c_str(), enginelab::shiftStyleName(style), engaged + 1);
    return engaged;
}

void checkSerialisation() {
    auto config = enginelab::makeDefaultInlineFour();
    config.transmission.shiftStyle = enginelab::ShiftStyle::sequential;
    config.transmission.quickshifter = false;
    config.transmission.quickshifterCutSeconds = 0.08;
    config.transmission.autoBlipper = false;
    const enginelab::YamlEngineSerializer yaml;
    const enginelab::JsonEngineSerializer json;
    for (const auto* serializer : { static_cast<const enginelab::IEngineSerializer*>(&yaml),
                                    static_cast<const enginelab::IEngineSerializer*>(&json) }) {
        const auto encoded = serializer->encode(config);
        const auto decoded = serializer->decode(encoded);
        require(decoded && decoded.config->transmission.shiftStyle == enginelab::ShiftStyle::sequential
                && !decoded.config->transmission.quickshifter
                && std::abs(decoded.config->transmission.quickshifterCutSeconds - 0.08) < 1.0e-9
                && !decoded.config->transmission.autoBlipper,
            "an engine file must keep its shift style, quickshifter and blipper");
        auto unknown = encoded;
        const auto at = unknown.find("sequential");
        require(at != std::string::npos, "the shift style must be written by name");
        unknown.replace(at, std::string_view("sequential").size(), "automatic");
        require(!serializer->decode(unknown), "an unknown shift style must be refused");
    }
    require(enginelab::parseShiftStyle("dual_clutch") == enginelab::ShiftStyle::dualClutch,
        "dual_clutch must name the dual-clutch style");
}
} // namespace

int main() {
    try {
        checkSerialisation();
        const auto catalog = enginelab::loadEngineCatalog(std::filesystem::path(ENGINELAB_CATALOG_ROOT));
        require(catalog.errors.empty(), "the catalogue must load");
        const auto& cp2 = findEngine(catalog, "CP2 MT-07");
        const auto& jz = findEngine(catalog, "2JZ");
        require(cp2.transmission.shiftStyle == enginelab::ShiftStyle::sequential
                && cp2.transmission.quickshifter && !cp2.transmission.autoBlipper,
            "the catalogue CP2 shifts up through a quickshifter, down through its clutch");
        require(jz.transmission.shiftStyle == enginelab::ShiftStyle::manual,
            "the catalogue 2JZ keeps its manual gearbox");
        // A motorcycle's wheels, chain and clutch basket, reflected to the
        // road in first gear, are a small share of its mass (a car's
        // defaults made them 90 % of a CP2's).
        for (const auto& entry : catalog.entries) {
            const auto& config = entry.config;
            if (config.vehicle.massKg > 500.0 || config.transmission.gearRatios.empty()) continue;
            const auto& box = config.transmission;
            const auto first = box.gearRatios.front() * box.finalDriveRatio;
            const auto radius = config.vehicle.tireRadiusM;
            const auto equivalentKg = (box.drivenWheelInertiaKgM2 + box.differentialInertiaKgM2
                + box.gearboxInputInertiaKgM2 * first * first) / (radius * radius);
            std::printf("  %-34s rotating inertia in first: %5.1f kg on %5.0f kg\n", config.name.c_str(),
                equivalentKg, config.vehicle.massKg);
            require(equivalentKg < 0.2 * config.vehicle.massKg,
                config.name + ": a motorcycle's rotating inertia must not weigh like a car's");
        }

        std::printf("upshift 1 -> 2 at full throttle, 70 %% of redline\n");
        const auto cp2Manual = measureUpshift(cp2, enginelab::ShiftStyle::manual);
        const auto cp2Quick = measureUpshift(cp2, enginelab::ShiftStyle::sequential);
        const auto jzManual = measureUpshift(jz, enginelab::ShiftStyle::manual);
        const auto jzDual = measureUpshift(jz, enginelab::ShiftStyle::dualClutch);
        std::printf("dual-clutch upshift 1 -> 2 at 85 %% of redline, spark retard 20 and 0 deg\n");
        const auto jzRetard = measureUpshift(jz, enginelab::ShiftStyle::dualClutch, 20.0, 0.85);
        const auto jzNoRetard = measureUpshift(jz, enginelab::ShiftStyle::dualClutch, 0.0, 0.85);

        // Quickshifter: the ignition is cut for the authored length (one
        // frame of rounding), the ECU really drops the spark, and the drive
        // comes back much sooner than through a clutch shift.
        require(std::abs(cp2Quick.cutSeconds - cp2.transmission.quickshifterCutSeconds) <= stepSeconds + 1.0e-9,
            "a quickshifter must cut the ignition for its cut length");
        require(cp2Quick.sparkCutSeen, "the quickshifter cut must reach the ECU's spark");
        require(cp2Manual.cutSeconds == 0.0 && jzDual.cutSeconds == 0.0,
            "only a quickshifter cuts the ignition");
        require(cp2Quick.syncSeconds > 0.0 && cp2Quick.syncErrorShare < 0.03,
            "a quickshifted CP2 must end in second, at its speed");
        require(cp2Quick.gapSeconds < 0.6 * cp2Manual.gapSeconds,
            "a quickshifter must keep the drive gap well below a clutch shift's");

        // Dual clutch: the oncoming clutch carries the drive throughout.
        require(jzDual.syncSeconds > 0.0 && jzDual.syncErrorShare < 0.03,
            "a dual-clutch 2JZ must end in second, at its speed");
        require(jzDual.gapSeconds == 0.0 && jzDual.minimumWheelShare > 0.5,
            "a dual-clutch upshift must have no torque gap");
        require(jzManual.gapSeconds > 0.05, "a clutch shift must have a torque gap (control)");
        // Its torque intervention: the spark is retarded while the clutch
        // pulls the engine down. The 2JZ's clutch is at its capacity through
        // the pull-down either way, so the push's peak stays; the engine,
        // making less torque, reaches the new speed sooner and the push is
        // shorter.
        require(jzRetard.lowestAdvance < jzNoRetard.lowestAdvance - 8.0,
            "a loaded dual-clutch upshift must retard the spark");
        require(jzRetard.pushImpulse < 0.98 * jzNoRetard.pushImpulse,
            "the spark retard must shorten the push the wheel feels");
        require(jzNoRetard.gapSeconds == 0.0 && jzNoRetard.syncErrorShare < 0.03,
            "without it the hand-over still completes with no gap");

        std::printf("downshift 3 -> 2, throttle shut\n");
        const auto blipped = measureDownshift(cp2, true);
        const auto unblipped = measureDownshift(cp2, false);
        require(blipped.peakBlip > 0.2, "the blipper must open the throttle");
        require(blipped.engagedRpmShare > 0.95,
            "a blipped downshift must engage at the new gear's speed");
        require(blipped.peakSlipRpm < 0.3 * unblipped.peakSlipRpm,
            "a blipped downshift must slip far less than an unblipped one");
        require(unblipped.peakBlip == 0.0, "no blip without the blipper");

        std::printf("driver's clutch and the dual clutch's own\n");
        checkClutchHeldDownshift(cp2);
        checkDualClutchCreep(jz);
        require(overRevDownshift(jz, enginelab::ShiftStyle::dualClutch) == 1,
            "a dual clutch must refuse a downshift that over-revs the engine");
        require(overRevDownshift(jz, enginelab::ShiftStyle::manual) == 0,
            "a manual box lets the driver over-rev on a downshift");
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
    std::cout << "Shift style tests passed\n";
    return EXIT_SUCCESS;
}
