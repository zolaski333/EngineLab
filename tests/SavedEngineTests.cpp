#include <enginelab/audio/OfflineAudioExporter.hpp>
#include <enginelab/calibration/CalibrationJson.hpp>
#include <enginelab/calibration/EcuCalibration.hpp>
#include <enginelab/calibration/EcuCalibrationKeys.hpp>
#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/catalog/EngineDesign.hpp>
#include <enginelab/catalog/SavedEngines.hpp>
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/physics/SimplifiedGasolinePhysics.hpp>
#include <enginelab/runtime/EngineRuntime.hpp>
#include <enginelab/serialization/JsonEngineSerializer.hpp>
#include <enginelab/serialization/YamlEngineSerializer.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>
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

/** An engine edited the way the app edits one: smaller bores (a narrower
    rotor, whose faces follow it), longer runners. */
[[nodiscard]] EngineConfig edited(const EngineConfig& original) {
    auto config = original;
    if (config.cycle == EngineCycle::rotary) {
        config.rotary.rotorWidthMm -= 1.0;
        applyRotaryFaceGeometry(config);
    } else {
        for (auto& cylinder : config.cylinders) cylinder.boreMm -= 1.0;
    }
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
        require(read.engine->calibration.has_value(), config.name + ": its calibration is saved inside it");
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

    // Backfires are switched in the Audio workshop, on the engine running:
    // they are neither a modification nor saved.
    auto backfiring = config;
    backfiring.exhaustAfterfire.strategy = ExhaustAfterfireStrategy::lateSparkOverrun;
    backfiring.exhaustAfterfire.enabled = true;
    backfiring.ignition.limiterKeepsFuel = true;
    require(!baseline.configDiffers(backfiring), "backfires switched on are not a modification");
    {
        const auto backfiringFile = savedEngineFile(folder.path, "Backfiring " + config.name);
        require(!saveEngine(backfiringFile, backfiring, nullptr), "an engine with backfires on saves");
        const auto backfiringRead = readSavedEngine(backfiringFile);
        require(backfiringRead.engine && !backfiringRead.engine->config.exhaustAfterfire.enabled
                    && !backfiringRead.engine->config.ignition.limiterKeepsFuel
                    && backfiringRead.engine->config.exhaustAfterfire.strategy
                        == ExhaustAfterfireStrategy::cleanDfco,
                "a saved engine reads back with backfires off");
    }
    {
        // An edit made from a copy taken before the workshop switched them
        // on keeps the running engine's backfires and its own change.
        auto resized = edited(config);
        keepBackfires(backfiring, resized);
        require(resized.exhaustAfterfire.enabled && resized.ignition.limiterKeepsFuel
                    && resized.exhaustAfterfire.strategy
                        == ExhaustAfterfireStrategy::lateSparkOverrun
                    && baseline.configDiffers(resized),
                "a resized engine keeps the backfires it was running");
        auto quiet = backfiring;
        keepBackfires(config, quiet);
        require(!quiet.exhaustAfterfire.enabled && !quiet.ignition.limiterKeepsFuel,
                "an edit cannot switch backfires on");
    }

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

    // A damaged calibration beside an older save fails the read instead of
    // running other tables.
    require(!saveEngine(file, config, nullptr), "the engine saves without a calibration again");
    std::ofstream(savedCalibrationFile(file), std::ios::trunc) << "{ \"schema_version\": 1, \"entries\": [ ";
    const auto damaged = readSavedEngine(file);
    require(!damaged.engine && !damaged.error.empty(), "a damaged calibration fails the read with a reason");
}

[[nodiscard]] std::string textOf(const std::filesystem::path& file) {
    std::ifstream in(file, std::ios::binary);
    return { std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>() };
}

void writeText(const std::filesystem::path& file, const std::string& text) {
    std::ofstream(file, std::ios::binary | std::ios::trunc) << text;
}

/** One file holds the engine and its ECU tables, JSON or YAML; a save made
    by 0.9.1 or earlier (tables beside it) still loads, and its next save
    folds them in. Before, a YAML export lost the tables altogether. */
void checkOneFile(const EngineConfig& source) {
    Folder folder;
    const JsonEngineSerializer json;
    const YamlEngineSerializer yaml;
    const auto config = canonical(source);
    const auto store = defaultStore(config);
    auto draft = calibration::makeDraft(*store->snapshot());
    const auto* entry = draft.find(calibration::keys::ignitionAdvance);
    require(entry != nullptr, config.name + ": an ignition advance table exists");
    auto advance = *entry;
    const auto retard = [](double& value) {
        value = std::max(value - 3.0, calibration::ecuLimits::minimumIgnitionAdvanceDegrees);
    };
    std::visit([&](auto& item) {
        if constexpr (requires { item.values; }) {
            for (auto& value : item.values) retard(value);
        } else {
            retard(item.value);
        }
    }, advance);
    draft.set(advance);
    require(store->publish(draft, { store->snapshot()->revision(), "tuned" }).published, "the tuned tables publish");
    const auto tuned = store->snapshot();
    const EngineBaseline asSaved(config, calibration::makeDraft(*tuned));
    require(asSaved.calibrationDiffers(*defaultStore(config)->snapshot()), "the tuned tables differ from the default");

    for (const auto* extension : { ".json", ".yaml", ".yml" }) {
        const auto file = folder.path / (std::string("One file") + extension);
        require(droppedFileKind(file) == DroppedFileKind::engine, std::string(extension) + " drops as an engine");
        require(!saveEngine(file, config, tuned.get()), std::string(extension) + ": the engine saves");
        require(!std::filesystem::exists(savedCalibrationFile(file)), std::string(extension) + ": one file, nothing beside it");
        require(textOf(file).find("ecu_calibration") != std::string::npos,
                std::string(extension) + ": the tables are inside it");
        const auto read = readSavedEngine(file);
        require(read.engine && read.engine->calibration,
                std::string(extension) + ": the engine reads back with its tables (" + read.error + ")");
        const auto back = json.encode(canonical(read.engine->config));
        const auto expected = json.encode(config);
        const auto at = std::mismatch(back.begin(), back.end(), expected.begin(), expected.end()).first - back.begin();
        const auto from = static_cast<std::size_t>(std::max<std::ptrdiff_t>(at - 120, 0));
        require(back == expected, std::string(extension) + ": the configuration reads back unchanged (read "
                                      + back.substr(from, 200) + " | saved " + expected.substr(from, 200) + ")");
        require(!EngineBaseline(config, *read.engine->calibration).calibrationDiffers(*tuned),
                std::string(extension) + ": the tables read back as saved");
        // The engine half is what the plain encoder writes: an older
        // EngineLab, which ignores the tables, reads it as before.
        const auto plain = engineFileFormatOf(file) == EngineFileFormat::yaml
            ? yaml.decode(textOf(file)) : json.decode(textOf(file));
        require(static_cast<bool>(plain) && json.encode(canonical(*plain.config)) == json.encode(config),
                std::string(extension) + ": the engine decoder alone reads the file");
    }

    // Tables whose YAML alias points back at itself never end: their
    // conversion ran the stack out and ended the process (audit PG-07).
    // They are a read error now, and so are aliases repeated level after level.
    const auto plainYaml = encodeEngineFile(config, nullptr, EngineFileFormat::yaml);
    require(decodeEngineFile(plainYaml, EngineFileFormat::yaml, "plain.yaml").engine.has_value(),
            "the YAML engine without tables reads");
    const auto looped = decodeEngineFile(plainYaml + "\necu_calibration: &loop [*loop]\n", EngineFileFormat::yaml,
                                         "loop.yaml");
    require(!looped.engine && looped.error.find("too deep") != std::string::npos, "tables in an alias loop are a read error: " + looped.error);
    std::string laughs = "\nlaugh0: &l0 [1, 1, 1, 1, 1, 1, 1, 1, 1, 1]\n";
    for (int level = 1; level <= 7; ++level) {
        laughs += "laugh" + std::to_string(level) + ": &l" + std::to_string(level) + " [";
        for (int item = 0; item < 10; ++item) laughs += (item ? ", *l" : "*l") + std::to_string(level - 1);
        laughs += "]\n";
    }
    const auto laughed = decodeEngineFile(plainYaml + laughs + "ecu_calibration: *l7\n", EngineFileFormat::yaml,
                                          "laughs.yaml");
    require(!laughed.engine && laughed.error.find("too many") != std::string::npos, "tables of a hundred million aliased values are a read error: " + laughed.error);

    // The tables inside win over a stale file beside it.
    const auto single = folder.path / "Single.json";
    require(!saveEngine(single, config, tuned.get()), "the single file saves");
    writeText(savedCalibrationFile(single), calibration::CalibrationJson::serialize(*defaultStore(config)->snapshot()));
    const auto preferred = readSavedEngine(single);
    require(preferred.engine && preferred.engine->calibration
                && !EngineBaseline(config, *preferred.engine->calibration).calibrationDiffers(*tuned),
            "the tables inside the file are the ones read");

    // A save made by 0.9.1: the engine alone, its tables beside it.
    const auto old = folder.path / "Old save.json";
    writeText(old, json.encode(config));
    writeText(savedCalibrationFile(old), calibration::CalibrationJson::serialize(*tuned));
    const auto oldRead = readSavedEngine(old);
    require(oldRead.engine && oldRead.engine->calibration
                && !EngineBaseline(config, *oldRead.engine->calibration).calibrationDiffers(*tuned),
            "an old two-file save loads with its tables (" + oldRead.error + ")");
    const auto folded = storeFrom(*oldRead.engine->calibration);
    require(!saveEngine(old, oldRead.engine->config, folded->snapshot().get()), "the old save saves again");
    require(!std::filesystem::exists(savedCalibrationFile(old)), "its tables are folded into the engine file");
    const auto foldedRead = readSavedEngine(old);
    require(foldedRead.engine && foldedRead.engine->calibration
                && !EngineBaseline(config, *foldedRead.engine->calibration).calibrationDiffers(*tuned),
            "and read back from inside it");
    // A damaged file beside an old save still fails the read.
    writeText(old, json.encode(config));
    writeText(savedCalibrationFile(old), "{ \"schema_version\": 1, \"calibrations\": [ ");
    require(!readSavedEngine(old).engine, "a damaged calibration beside an old save fails the read");

    // An older YAML export carried no tables: it runs on the default ones.
    const auto oldYaml = folder.path / "Old export.yaml";
    writeText(oldYaml, yaml.encode(config));
    const auto oldYamlRead = readSavedEngine(oldYaml);
    require(oldYamlRead.engine && !oldYamlRead.engine->calibration,
            "an old YAML export loads without tables (" + oldYamlRead.error + ")");

    // Damaged tables inside the file fail the read, with a reason.
    const auto damagedJson = folder.path / "Damaged.json";
    auto text = json.encode(config);
    text.insert(text.find('{') + 1, "\n  \"ecu_calibration\": { \"schema_version\": 1, \"calibrations\": 7 },");
    writeText(damagedJson, text);
    const auto damagedJsonRead = readSavedEngine(damagedJson);
    require(!damagedJsonRead.engine && damagedJsonRead.error.find("ECU calibration") != std::string::npos,
            "damaged tables inside a JSON file fail the read (" + damagedJsonRead.error + ")");
    const auto damagedYaml = folder.path / "Damaged.yaml";
    writeText(damagedYaml, yaml.encode(config) + "\necu_calibration:\n  schema_version: 1\n  calibrations: 7\n");
    const auto damagedYamlRead = readSavedEngine(damagedYaml);
    require(!damagedYamlRead.engine && damagedYamlRead.error.find("ECU calibration") != std::string::npos,
            "damaged tables inside a YAML file fail the read (" + damagedYamlRead.error + ")");

    // What a drop on the window opens.
    require(droppedFileKind("maps.ecu.json") == DroppedFileKind::calibration, "a .ecu.json drops as ECU tables");
    require(droppedFileKind("MAPS.ECU.JSON") == DroppedFileKind::calibration, "whatever its case");
    require(droppedFileKind("V8.ELS") == DroppedFileKind::engineScript
                && droppedFileKind("v8.engine") == DroppedFileKind::engineScript,
            ".els and .engine drop as scripts");
    require(droppedFileKind("notes.txt") == DroppedFileKind::none && droppedFileKind("engine") == DroppedFileKind::none,
            "anything else is not taken");
    std::cout << "SavedEngine: one file (JSON, YAML) holds the engine and its tables; old saves load\n";
}

struct RenderedSound final {
    double rms { 0.0 };
    /** master.wav as written: sample for sample. RMS and peak alone accept
        any permutation of the samples (audit TEST-02). */
    std::string masterBytes;
};

/** A short start and rev through the shipping audio path, heard through the
    engine's own voicing (the physical reference monitor ignores it). */
[[nodiscard]] RenderedSound sound(const EngineConfig& config, const std::filesystem::path& output) {
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
    std::ifstream master(output / "master.wav", std::ios::binary);
    RenderedSound rendered { result.masterRms,
        std::string((std::istreambuf_iterator<char>(master)), std::istreambuf_iterator<char>()) };
    require(!rendered.masterBytes.empty(), config.name + ": the render wrote its master");
    return rendered;
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
        require(original.masterBytes == reloaded.masterBytes,
                config.name + ": the saved engine sounds as the original, sample for sample (rms "
                    + std::to_string(original.rms) + " / " + std::to_string(reloaded.rms) + ")");
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

/** Save on a saved engine passes the menu's own entry back to the list it
    lives in. Held by reference, the entry was erased first and the next
    saved engine slid into its place: each Save renamed the running engine to
    the next one and wrote over its file (owner report 2026-11-16: an Audi
    RC8 saved three times became "Diesel V Six Truck", "Diesel V8 Truck" and
    "Eight-Rotor", in file order). */
void checkSaveKeepsTheMenu(const EngineConfig& source) {
    const std::filesystem::path folder = "engines";
    std::vector<SavedEngine> menu;
    auto engine = [&source](const std::string& name, const std::filesystem::path& file) {
        auto config = source;
        config.name = name;
        return SavedEngine { std::move(config), std::nullopt, file };
    };
    menu.push_back(engine("Catalogue engine", {}));
    const std::vector<std::string> names { "Audi RC8", "Diesel V Six Truck", "Diesel V8 Truck", "Eight-Rotor" };
    for (const auto& name : names) menu.push_back(engine(name, savedEngineFile(folder, name)));
    const auto expectMenu = [&](const std::string& when) {
        require(menu.size() == names.size() + 1, when + ": the menu keeps its " + std::to_string(names.size() + 1)
                                                     + " engines (" + std::to_string(menu.size()) + ")");
        require(menu.front().file.empty() && menu.front().config.name == "Catalogue engine",
                when + ": the catalogue engine stays first");
        for (std::size_t index = 0; index < names.size(); ++index) {
            const auto& saved = menu[index + 1];
            require(saved.config.name == names[index] && saved.file == savedEngineFile(folder, names[index]),
                    when + ": entry " + std::to_string(index + 1) + " is still \"" + names[index] + "\", not \""
                        + saved.config.name + "\" in " + saved.file.filename().string());
        }
    };
    for (int save = 1; save <= 3; ++save) {
        const auto index = placeSavedEngine(menu, menu[1]);
        require(index == 1, "Save " + std::to_string(save) + " puts the engine back in its place");
        expectMenu("after Save " + std::to_string(save));
    }
    // "Save as" a new name lands in file order among the saved engines.
    const auto added = placeSavedEngine(menu, engine("Cobra", savedEngineFile(folder, "Cobra")));
    require(added == 2 && menu[2].config.name == "Cobra", "a new saved engine is placed by its file name");
    require(menu.size() == names.size() + 2, "and replaces none");
    // ... and over another engine's name replaces that entry only.
    const auto replaced = placeSavedEngine(menu, engine("Eight-Rotor", savedEngineFile(folder, "Eight-Rotor")));
    require(menu.size() == names.size() + 2 && menu[replaced].config.name == "Eight-Rotor",
            "saving over a saved engine replaces it, once");
}

void checkNamesAndFolder(const EngineConfig& source) {
    Folder folder;
    require(savedEngineFile(folder.path, " a/b:c?  ").filename() == "a_b_c_.json", "a name is made safe for a file");
    require(savedEngineFile(folder.path, "x.ecu").filename() == "x.ecu_.json",
            "an engine is never named like a calibration file");
    require(savedEngineFile(folder.path, " . ").filename() == "engine.json", "an empty file name gets a stem");
    // "My.ECU.json" dropped back on the app was read as a calibration
    // (audit DR-06); "CON.json" could not be written (audit DR-10).
    const auto upperEcu = savedEngineFile(folder.path, "My.ECU");
    require(upperEcu.filename() == "My.ECU_.json" && droppedFileKind(upperEcu) == DroppedFileKind::engine,
            "an engine is never named like a calibration file, whatever the case");
    for (const auto& [name, expected] : std::vector<std::pair<std::string, std::string>> {
             { "CON", "CON_.json" }, { "nul.v8", "nul_.v8.json" }, { "COM1", "COM1_.json" },
             { "lpt9", "lpt9_.json" }, { "Console", "Console.json" }, { "COM10", "COM10.json" },
             { "Aux V8", "Aux V8.json" } })
        require(savedEngineFile(folder.path, name).filename() == std::filesystem::path(expected),
                "\"" + name + "\" is kept in " + expected + ", not "
                    + savedEngineFile(folder.path, name).filename().string());
    {
        auto device = canonical(source);
        device.name = "CON";
        require(!saveEngine(savedEngineFile(folder.path, device.name), device, nullptr)
                    && readSavedEngine(savedEngineFile(folder.path, device.name)).engine.has_value(),
                "an engine named CON saves and reads back");
        std::filesystem::remove(savedEngineFile(folder.path, device.name));
    }
    // Two names, one file (audit DR-01): the designer checks this before it
    // builds, and the menu holds one engine per file.
    require(sameSavedEngineFile(savedEngineFile(folder.path, "A?B"), savedEngineFile(folder.path, "A*B")),
            "\"A?B\" and \"A*B\" share a file");
    require(!sameSavedEngineFile(savedEngineFile(folder.path, "A B"), savedEngineFile(folder.path, "A C")),
            "two names, two files");
#ifdef _WIN32
    require(sameSavedEngineFile(folder.path / "My V8.json", folder.path / "my v8.JSON"),
            "names that differ only in case are one file on Windows");
    {
        auto engines = std::vector<SavedEngine> {};
        auto first = canonical(source);
        first.name = "My V8";
        engines.push_back({ first, std::nullopt, folder.path / "My V8.json" });
        auto second = canonical(source);
        second.name = "my v8";
        (void)placeSavedEngine(engines, { second, std::nullopt, folder.path / "my v8.json" });
        require(engines.size() == 1 && engines.front().config.name == "my v8",
                "the menu keeps one engine for one file, whatever the case");
    }
#endif
    // A save the reader refuses lost the engine (audit DR-02): an engine of
    // 200,000 gears is 2.6 MB indented, 0.8 MB without.
    {
        auto large = canonical(source);
        large.name = "Many gears";
        large.transmission.gearRatios.assign(200'000, 1.0);
        require(!validateEngineConfig(large).has_value(), "an engine of 200,000 gears validates");
        const auto file = savedEngineFile(folder.path, large.name);
        const auto saved = saveEngine(file, large, nullptr);
        require(!saved, "it saves: " + saved.value_or(""));
        const auto read = readSavedEngine(file);
        require(read.engine && read.engine->config.transmission.gearRatios.size() == 200'000,
                "and reads back: " + read.error);
        std::filesystem::remove(file);
    }
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

/** A hand-edited value outside its range (an octane of 150) loads clamped,
 *  with a warning naming it, instead of dropping the engine from the menu;
 *  a value of a list (a bore) is still refused, as is anything non-finite. */
void checkOutOfRangeEdits(const EngineConfig& source) {
    Folder folder;
    auto config = canonical(source);
    config.name = "Hand edited";
    const auto file = savedEngineFile(folder.path, config.name);
    require(!saveEngine(file, config, nullptr), "an engine saves");
    std::string text;
    {
        std::ifstream in(file);
        text.assign(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
    }
    const auto replaceValue = [](std::string document, const std::string& key, const std::string& value) {
        const auto at = document.find("\"" + key + "\": ");
        require(at != std::string::npos, key + " is written");
        const auto start = at + key.size() + 4;
        const auto end = document.find_first_of(",\n}", start);
        return document.replace(start, end - start, value);
    };
    std::ofstream(file) << replaceValue(text, "octane_rating", "150.0");
    const auto read = readSavedEngine(file);
    require(read.engine.has_value(), "an octane of 150 loads: " + read.error);
    require(read.engine->config.octaneRating == 130.0, "clamped into its range (130)");
    require(read.warnings.size() == 1 && read.warnings.front().find("octaneRating = 150") != std::string::npos,
            "and says so");
    const auto loaded = loadSavedEngines(folder.path, ENGINELAB_CATALOG_ROOT);
    require(loaded.engines.size() == 1 && loaded.errors.empty() && loaded.warnings.size() == 1,
            "the folder lists it, with its warning");

    std::ofstream(file) << replaceValue(text, "bore_mm", "-1.0");
    require(!readSavedEngine(file).engine.has_value(), "a negative bore is still refused");

    // A whole-number field holding a fraction or a value past 32 bits was
    // truncated or wrapped into another valid one (audit DR-08).
    std::ofstream(file) << replaceValue(text, "intake_valve_count", "1.9");
    const auto fractional = readSavedEngine(file);
    require(!fractional.engine.has_value() && fractional.error.find("intake_valve_count") != std::string::npos,
            "a fractional valve count is refused by name: " + fractional.error);
    std::ofstream(file) << replaceValue(text, "schema_version", "4294967305");
    const auto wrapped = readSavedEngine(file);
    require(!wrapped.engine.has_value() && wrapped.error.find("schema_version") != std::string::npos,
            "a schema past 32 bits is refused by name: " + wrapped.error);
    std::ofstream(file) << replaceValue(text, "intake_valve_count", "2.0");
    require(readSavedEngine(file).engine.has_value(), "a whole 2.0 still reads");

    // The strict reader keeps refusing what the lenient one corrects.
    auto strict = config;
    strict.octaneRating = 150.0;
    require(validateEngineConfig(strict).has_value(), "validation still refuses an octane of 150");

    // The same edit in a YAML file loads the same way: it was refused, so
    // one engine loaded or vanished by its file format (audit DR-07).
    std::filesystem::remove(file);
    const auto yamlFile = folder.path / "Hand edited.yaml";
    std::ofstream(yamlFile, std::ios::binary) << encodeEngineFile(strict, nullptr, EngineFileFormat::yaml);
    const auto yamlRead = readSavedEngine(yamlFile);
    require(yamlRead.engine.has_value(), "a YAML octane of 150 loads: " + yamlRead.error);
    require(yamlRead.engine->config.octaneRating == 130.0, "the YAML one is clamped into its range (130)");
    require(yamlRead.warnings.size() == 1
                && yamlRead.warnings.front().find("octaneRating = 150") != std::string::npos,
            "and the YAML one says so");
}
} // namespace

/** An engine with an H-pipe reads back, as a saved engine (JSON) and as YAML:
 *  its balance pipe meets each side on port 2, which both readers refused as
 *  "not a crossover port", so a designed H-pipe engine vanished from My
 *  engines at the next start (owner report 2026-10-30). */
void checkHPipeSaved() {
    EngineSpec spec;
    spec.name = "Saved H-pipe V8";
    spec.use = EngineUse::sportCar;
    spec.layout = EngineLayout::vLayout;
    spec.cylinderCount = 8;
    spec.displacementLitres = 5.0;
    spec.exhaustSystem = ExhaustSystem::dualHPipe;
    const auto designed = designCatalogueEngine(spec, ENGINELAB_CATALOG_ROOT);
    require(designed.config.has_value(), "an H-pipe V8 designs: " + designed.error);
    const auto sideBranches = [](const EngineConfig& config) {
        std::size_t count = 0;
        for (const auto& path : config.exhaustPaths)
            if (path.network)
                for (const auto& connection : path.network->connections)
                    count += (connection.fromPort == exhaustSideBranchPort ? 1U : 0U)
                        + (connection.toPort == exhaustSideBranchPort ? 1U : 0U);
        return count;
    };
    require(sideBranches(*designed.config) == 2U, "the balance pipe meets each side as a side branch");
    Folder folder;
    const auto file = savedEngineFile(folder.path, spec.name);
    require(!saveEngine(file, *designed.config, nullptr).has_value(), "the H-pipe engine saves");
    const auto read = readSavedEngine(file);
    require(read.engine.has_value(), "the saved H-pipe engine reads back: " + read.error);
    require(sideBranches(read.engine->config) == 2U, "and keeps its balance pipe's side branches");
    const YamlEngineSerializer yaml;
    const auto decoded = yaml.decode(yaml.encode(*designed.config));
    require(static_cast<bool>(decoded), "the H-pipe engine reads back from YAML: " + decoded.error);
    require(sideBranches(*decoded.config) == 2U, "and keeps its side branches there too");
}

/** Every listed fuel runs an engine of its kind as the inspector applies it
 *  (properties, octane, latent heat), and a saved engine keeps it. */
void checkFuelPresets(const EngineCatalogLoadResult& catalogue) {
    const auto presets = loadFuelPresets(ENGINELAB_CATALOG_ROOT);
    const auto named = [&presets](const std::string& key) {
        return std::any_of(presets.begin(), presets.end(), [&key](const FuelPreset& preset) { return preset.key == key; });
    };
    require(named("ethanol_e85") && named("methanol_m100") && named("road_diesel_en590"),
            "the fuel list has E85, methanol and diesel");
    const auto engineOfKind = [&catalogue](bool diesel) {
        const auto found = std::find_if(catalogue.entries.begin(), catalogue.entries.end(),
            [diesel](const EngineCatalogEntry& entry) { return (entry.config.fuel == FuelType::diesel) == diesel; });
        require(found != catalogue.entries.end(), "the catalogue has a spark and a diesel engine");
        return found->config;
    };
    for (const auto& preset : presets) {
        auto config = canonical(engineOfKind(preset.diesel()));
        config.fuelProperties = preset.properties;
        if (!preset.diesel()) {
            require(preset.octaneRating > 0.0, preset.key + " has an octane");
            config.octaneRating = preset.octaneRating;
        }
        require(preset.latentHeatKjPerKg > 0.0, preset.key + " has a latent heat");
        config.injection.latentHeatKjPerKg = preset.latentHeatKjPerKg;
        const auto invalid = validateEngineConfig(config);
        require(!invalid, preset.key + " makes a valid engine: " + invalid.value_or(""));
        Folder folder;
        config.name = "Fuel " + preset.key;
        const auto file = savedEngineFile(folder.path, config.name);
        require(!saveEngine(file, config, nullptr), preset.key + ": the engine saves");
        const auto read = readSavedEngine(file);
        require(read.engine.has_value() && read.warnings.empty(), preset.key + ": it reads back as saved");
        const auto& back = read.engine->config;
        require(back.fuelProperties.name == preset.properties.name && back.octaneRating == config.octaneRating
                    && back.injection.latentHeatKjPerKg == preset.latentHeatKjPerKg,
                preset.key + ": with its fuel");
    }
}

int main(int argc, char** argv) {
    // `--report <folder>`: what the app would say about a folder of saved
    // engines (a user's, to diagnose a file missing from the menu).
    if (argc == 3 && std::string(argv[1]) == "--report") {
        const auto saved = loadSavedEngines(argv[2], ENGINELAB_CATALOG_ROOT);
        std::cout << saved.engines.size() << " engines loaded\n";
        for (const auto& error : saved.errors) std::cout << "not loaded: " << error << '\n';
        for (const auto& warning : saved.warnings) std::cout << "corrected: " << warning << '\n';
        // The app's menu tells engines apart by name: a second one is not loaded.
        const auto shipped = loadEngineCatalog(ENGINELAB_CATALOG_ROOT);
        std::vector<std::string> names;
        for (const auto& entry : shipped.entries) names.push_back(entry.config.name);
        for (const auto& engine : saved.engines) {
            if (std::find(names.begin(), names.end(), engine.config.name) != names.end())
                std::cout << "not loaded: " << engine.file.filename().string()
                          << ": another engine is already named \"" << engine.config.name << "\"\n";
            names.push_back(engine.config.name);
        }
        return 0;
    }
    // `--write <catalogue key> <file>`: a catalogue engine saved with its
    // generated ECU tables, as the app saves it -- a fixture for the tools'
    // `--saved-engine --saved-ecu` paths.
    if (argc == 4 && std::string(argv[1]) == "--write") {
        const auto shipped = loadEngineCatalog(ENGINELAB_CATALOG_ROOT);
        const auto selected = selectSingleEngineCatalogEntry(shipped.entries, argv[2]);
        if (!selected) {
            std::cerr << "--write: no single engine matches " << argv[2] << '\n';
            return EXIT_FAILURE;
        }
        const auto store = defaultStore(selected.entry->config);
        std::filesystem::create_directories(std::filesystem::path(argv[3]).parent_path());
        if (const auto error = saveEngine(argv[3], selected.entry->config, store->snapshot().get())) {
            std::cerr << "--write: " << *error << '\n';
            return EXIT_FAILURE;
        }
        std::cout << "saved " << selected.entry->config.name << " with its ECU tables\n";
        return 0;
    }
    checkHPipeSaved();
    const auto catalogue = loadEngineCatalog(ENGINELAB_CATALOG_ROOT);
    require(catalogue.errors.empty() && !catalogue.entries.empty(), "the shipped catalogue loads");
    require(catalogue.warnings.empty(), "with no value corrected into its range");
    const auto cp2 = std::find_if(catalogue.entries.begin(), catalogue.entries.end(),
                                  [](const EngineCatalogEntry& entry) { return entry.config.name.find("CP2") != std::string::npos; });
    require(cp2 != catalogue.entries.end(), "the CP2 is in the catalogue");
    checkSaveKeepsTheMenu(cp2->config);
    checkNamesAndFolder(cp2->config);
    checkOutOfRangeEdits(cp2->config);
    checkCalibrationSaved(cp2->config);
    checkOneFile(cp2->config);
    checkRunningEngineRenamed(cp2->config);
    checkSavedSound(catalogue);
    checkCatalogueRoundTrip(catalogue);
    checkFuelPresets(catalogue);
    return 0;
}
