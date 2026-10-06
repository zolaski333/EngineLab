#include <enginelab/audio/OfflineAudioExporter.hpp>
#include <enginelab/calibration/CalibrationJson.hpp>
#include <enginelab/calibration/EcuCalibration.hpp>
#include <enginelab/calibration/EcuCalibrationKeys.hpp>
#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/catalog/SavedEngines.hpp>
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/physics/SimplifiedGasolinePhysics.hpp>
#include <enginelab/runtime/EngineRuntime.hpp>
#include <enginelab/serialization/JsonEngineSerializer.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <variant>

namespace {
using namespace enginelab;

void require(bool condition, const std::string& message) {
    if (condition) return;
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
}

/** A temporary folder, removed on exit. */
struct Folder final {
    Folder() : path(std::filesystem::temp_directory_path() / ("enginelab-saved-" + std::to_string(std::rand()))) {
        std::filesystem::remove_all(path);
        std::filesystem::create_directories(path);
    }
    ~Folder() {
        std::error_code error;
        std::filesystem::remove_all(path, error);
    }
    std::filesystem::path path;
};

/** A simulator as the runtime builds it, its ECU reading `store`. */
struct Bench final {
    Bench(const EngineConfig& config, std::shared_ptr<calibration::CalibrationStore> store)
        : ecu(std::move(store)),
          exhaust(ExhaustGraph::makeForEngine(config)),
          simulator(std::make_unique<EngineSimulator>(config, ecu, physics, events, exhaust)) {}
    SimpleEcuModel ecu;
    SimplifiedGasolinePhysics physics;
    FourStrokeEventGenerator events;
    ExhaustGraph exhaust;
    std::unique_ptr<EngineSimulator> simulator;
};

struct Run final {
    double rpm {};
    double crankAngle {};
    double exhaustKpa {};
};

/** Cranks, idles, then opens the throttle: 2.5 s at the runtime's 240 Hz. */
[[nodiscard]] Run run(const EngineConfig& config, std::shared_ptr<calibration::CalibrationStore> store) {
    Bench bench(config, std::move(store));
    for (int frame = 0; frame < 600; ++frame) {
        EngineControls controls;
        controls.ignitionEnabled = true;
        controls.starterEngaged = frame < 240;
        controls.throttle = frame < 360 ? 0.1 : 0.8;
        controls.load = 0.3;
        (void)bench.simulator->step(1.0 / 240.0, controls);
    }
    const auto& state = bench.simulator->state();
    return { state.rpm, state.crankAngleDegrees, state.exhaustPressureKpa };
}

[[nodiscard]] bool same(const Run& a, const Run& b) {
    return a.rpm == b.rpm && a.crankAngle == b.crankAngle && a.exhaustKpa == b.exhaustKpa;
}

[[nodiscard]] EngineConfig canonical(EngineConfig config) {
    normaliseEngineConfig(config);
    if (const auto error = validateEngineConfig(config)) require(false, config.name + ": " + *error);
    return config;
}

[[nodiscard]] std::shared_ptr<calibration::CalibrationStore> defaultStore(const EngineConfig& config) {
    auto store = std::make_shared<calibration::CalibrationStore>();
    require(store->publish(calibration::makeDefaultEcuCalibration(config), { 0, "engine-default" }).published,
            config.name + ": the default calibration publishes");
    return store;
}

[[nodiscard]] std::shared_ptr<calibration::CalibrationStore> storeFrom(const calibration::CalibrationDraft& draft) {
    auto store = std::make_shared<calibration::CalibrationStore>();
    require(store->publish(draft, { 0, "saved" }).published, "a saved calibration publishes");
    return store;
}

/** An engine edited the way the app edits one: smaller bores (the Merlin sits
    at the 20 L limit), longer runners. */
[[nodiscard]] EngineConfig edited(const EngineConfig& original) {
    auto config = original;
    for (auto& cylinder : config.cylinders) cylinder.boreMm -= 1.0;
    for (auto& path : config.intakePaths) path.geometry.runnerLengthMm += 40.0;
    config.intake.runnerLengthMm += 40.0;
    return canonical(config);
}

/** Every catalogue engine, edited, saved and read back, runs bit for bit as
    the edited engine did: nothing the simulation reads is lost on disk. */
void checkCatalogueRoundTrip(const EngineCatalogLoadResult& catalogue) {
    Folder folder;
    const JsonEngineSerializer json;
    for (const auto& entry : catalogue.entries) {
        const auto original = canonical(entry.config);
        const auto config = edited(original);
        const auto store = defaultStore(config);
        const auto file = savedEngineFile(folder.path, config.name);
        require(!saveEngine(file, config, store->snapshot().get()), config.name + ": the engine saves");
        auto read = readSavedEngine(file);
        require(read.engine.has_value(), config.name + ": the saved engine reads back (" + read.error + ")");
        require(read.engine->calibration.has_value(), config.name + ": its calibration is saved beside it");
        const auto reloaded = canonical(read.engine->config);
        require(json.encode(reloaded) == json.encode(config), config.name + ": the configuration reads back unchanged");

        const auto editedRun = run(config, store);
        const auto reloadedRun = run(reloaded, storeFrom(*read.engine->calibration));
        require(!same(editedRun, run(original, defaultStore(original))),
                config.name + ": the edit changes the simulation, so the round trip is tested");
        require(same(editedRun, reloadedRun), config.name + ": the saved engine runs bit for bit as the edited one ("
                                                  + std::to_string(editedRun.rpm) + " / "
                                                  + std::to_string(reloadedRun.rpm) + " rpm)");
    }
    std::cout << "SavedEngine: " << catalogue.entries.size() << " engines saved and read back bit-identical\n";
}

/** Tables edited in the ECU window are part of the saved engine. */
void checkCalibrationSaved(const EngineConfig& source) {
    Folder folder;
    const auto config = canonical(source);
    const auto store = defaultStore(config);
    auto draft = calibration::makeDraft(*store->snapshot());
    const auto* entry = draft.find(calibration::keys::ignitionAdvance);
    require(entry != nullptr, config.name + ": an ignition advance table exists");
    auto advance = *entry;
    const auto retard = [](double& value) {
        value = std::max(value - 6.0, calibration::ecuLimits::minimumIgnitionAdvanceDegrees);
    };
    std::visit([&](auto& item) {
        if constexpr (requires { item.values; }) {
            for (auto& value : item.values) retard(value);
        } else {
            retard(item.value);
        }
    }, advance);
    draft.set(advance);
    const auto tuned = store->publish(draft, { store->snapshot()->revision(), "tuned" });
    for (const auto& issue : tuned.issues) std::cerr << issue.path << ": " << issue.message << '\n';
    require(tuned.published, config.name + ": the retarded calibration publishes");

    // The app's "modified" mark: the runtime's ECU publishes the default
    // tables into an empty store, which is the unmodified engine.
    const EngineBaseline baseline(config, calibration::makeDefaultEcuCalibration(config));
    const auto published = std::make_shared<calibration::CalibrationStore>();
    { const Bench bench(config, published); }
    require(published->snapshot()->revision() != 0, "the ECU publishes its default tables");
    require(!baseline.calibrationDiffers(*published->snapshot()), "the default tables are not a modification");
    require(baseline.calibrationDiffers(*store->snapshot()), "retarded tables are a modification");
    require(!baseline.configDiffers(config), "the loaded configuration is not a modification");
    require(baseline.configDiffers(edited(config)), "a resized engine is a modification");

    const auto file = savedEngineFile(folder.path, "Tuned " + config.name);
    require(!saveEngine(file, config, store->snapshot().get()), config.name + ": the tuned engine saves");
    const auto read = readSavedEngine(file);
    require(read.engine && read.engine->calibration, config.name + ": the tuned engine reads back");
    require(!EngineBaseline(config, *read.engine->calibration).calibrationDiffers(*store->snapshot()),
            "the saved tables read back as the tables that were saved");
    const auto tunedRun = run(config, store);
    require(!same(tunedRun, run(config, defaultStore(config))),
            config.name + ": 6 degrees of retard change the simulation");
    require(same(tunedRun, run(canonical(read.engine->config), storeFrom(*read.engine->calibration))),
            config.name + ": the tuned calibration reads back and runs bit for bit");

    // Saved again without a calibration, the old tables do not linger.
    require(!saveEngine(file, config, nullptr), "the engine saves without a calibration");
    require(!std::filesystem::exists(savedCalibrationFile(file)), "a stale calibration file is removed");
    require(!readSavedEngine(file).engine->calibration, "an engine without a calibration file has none");

    // A damaged calibration fails the read instead of running other tables.
    require(!saveEngine(file, config, store->snapshot().get()), "the tuned engine saves again");
    std::ofstream(savedCalibrationFile(file), std::ios::trunc) << "{ \"schema_version\": 1, \"entries\": [ ";
    const auto damaged = readSavedEngine(file);
    require(!damaged.engine && !damaged.error.empty(), "a damaged calibration fails the read with a reason");
}

/** Master level of a short start and rev through the shipping audio path,
    heard through the engine's own voicing (the physical reference monitor
    ignores it). */
[[nodiscard]] std::pair<double, double> sound(const EngineConfig& config, const std::filesystem::path& output) {
    OfflineAudioExportRequest request;
    request.engine = config;
    request.mix = config.audioVoicing;
    request.mix.monitorMode = AudioMonitorMode::captureVoiced;
    request.outputDirectory = output;
    request.assetRoot = ENGINELAB_CATALOG_ROOT;
    request.writeStems = false;
    request.overwriteExistingFiles = true;
    OfflineAudioStage start;
    start.name = "start";
    start.durationSeconds = 0.8;
    start.starterEngaged = true;
    start.governed = false;
    start.throttleStart = start.throttleEnd = 0.1;
    OfflineAudioStage rev;
    rev.name = "rev";
    rev.durationSeconds = 1.2;
    rev.governed = false;
    rev.throttleStart = 0.1;
    rev.throttleEnd = 0.7;
    rev.loadStart = rev.loadEnd = 0.2;
    request.scenario.stages = { start, rev };
    const auto result = exportOfflineAudio(request);
    require(result.success, config.name + ": the render runs (" + result.error + ")");
    return { result.masterRms, result.masterPeak };
}

/** A saved engine keeps its voice: the voicing it names comes back from the
    catalogue, and the shipping audio path renders it sample for sample. */
void checkSavedSound(const EngineCatalogLoadResult& catalogue) {
    Folder folder;
    for (const auto& entry : catalogue.entries) {
        const auto config = canonical(entry.config);
        require(!saveEngine(savedEngineFile(folder.path / "engines", config.name), config, nullptr),
                config.name + ": the engine saves");
    }
    const auto saved = loadSavedEngines(folder.path / "engines", ENGINELAB_CATALOG_ROOT);
    require(saved.errors.empty() && saved.engines.size() == catalogue.entries.size(),
            "every saved engine loads with its voicing");
    for (const auto& entry : catalogue.entries) {
        const auto config = canonical(entry.config);
        const auto match = std::find_if(saved.engines.begin(), saved.engines.end(),
            [&config](const SavedEngine& engine) { return engine.config.name == config.name; });
        require(match != saved.engines.end(), config.name + ": the saved engine is listed");
        require(match->config.audioVoicingKey == config.audioVoicingKey
                    && match->config.audioVoicingFamily == config.audioVoicingFamily,
                config.name + ": the saved engine names its voicing");
        const auto original = sound(config, folder.path / "original");
        const auto reloaded = sound(canonical(match->config), folder.path / "reloaded");
        require(original == reloaded, config.name + ": the saved engine sounds as the original (rms "
                                          + std::to_string(original.first) + " / "
                                          + std::to_string(reloaded.first) + ")");
    }
    std::cout << "SavedEngine: " << catalogue.entries.size() << " engines render the same audio once saved\n";
}

/** "Save as" names the running engine without restarting it: a dyno run
    started afterwards carries the saved name. */
void checkRunningEngineRenamed(const EngineConfig& source) {
    auto runtime = std::make_unique<EngineRuntime>(canonical(source));
    runtime->renameEngine("My " + source.name);
    require(runtime->engineConfig().name == "My " + source.name, "the running engine takes the new name");
    runtime->setIgnitionEnabled(true);
    runtime->start();
    runtime->startDyno();
    auto run = runtime->currentDynoRun();
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    while (run.engineName.empty() && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
        run = runtime->currentDynoRun();
    }
    runtime->stop();
    require(run.engineName == "My " + source.name, "a dyno run after the rename carries the saved name ("
                                                     + run.engineName + ")");
}

void checkNamesAndFolder(const EngineConfig& source) {
    Folder folder;
    require(savedEngineFile(folder.path, " a/b:c?  ").filename() == "a_b_c_.json", "a name is made safe for a file");
    require(savedEngineFile(folder.path, "x.ecu").filename() == "x.ecu_.json",
            "an engine is never named like a calibration file");
    require(savedEngineFile(folder.path, " . ").filename() == "engine.json", "an empty file name gets a stem");
    require(savedCalibrationFile(folder.path / "My EJ25.json").filename() == "My EJ25.ecu.json",
            "the calibration shares the engine's stem");
    const std::vector<std::string> catalogue { source.name };
    require(savedEngineNameError(source.name, catalogue).has_value(), "a catalogue name is refused");
    require(savedEngineNameError("   ", catalogue).has_value(), "an empty name is refused");
    require(savedEngineNameError(std::string(81, 'x'), catalogue).has_value(), "a long name is refused");
    require(!savedEngineNameError("My " + source.name, catalogue), "another name is accepted");

    require(loadSavedEngines(folder.path / "missing", ENGINELAB_CATALOG_ROOT).engines.empty(),
            "a missing folder holds no engines");
    auto config = canonical(source);
    const auto store = defaultStore(config);
    for (const auto* name : { "B engine", "A engine" }) {
        config.name = name;
        require(!saveEngine(savedEngineFile(folder.path, name), config, store->snapshot().get()), "an engine saves");
    }
    std::ofstream(folder.path / "broken.json") << "{ not json";
    const auto loaded = loadSavedEngines(folder.path, ENGINELAB_CATALOG_ROOT);
    require(loaded.engines.size() == 2 && loaded.engines[0].config.name == "A engine"
                && loaded.engines[1].config.name == "B engine",
            "the folder lists both engines, by file name, and not their calibration files");
    require(loaded.errors.size() == 1, "a broken file is reported, not loaded");
}
} // namespace

int main() {
    const auto catalogue = loadEngineCatalog(ENGINELAB_CATALOG_ROOT);
    require(catalogue.errors.empty() && !catalogue.entries.empty(), "the shipped catalogue loads");
    const auto cp2 = std::find_if(catalogue.entries.begin(), catalogue.entries.end(),
                                  [](const EngineCatalogEntry& entry) { return entry.config.name.find("CP2") != std::string::npos; });
    require(cp2 != catalogue.entries.end(), "the CP2 is in the catalogue");
    checkNamesAndFolder(cp2->config);
    checkCalibrationSaved(cp2->config);
    checkRunningEngineRenamed(cp2->config);
    checkSavedSound(catalogue);
    checkCatalogueRoundTrip(catalogue);
    return 0;
}
