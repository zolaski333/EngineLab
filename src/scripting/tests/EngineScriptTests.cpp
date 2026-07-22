#include <enginelab/scripting/EngineScriptCompiler.hpp>
#include <enginelab/scripting/EngineScriptHotReloader.hpp>
#include <enginelab/serialization/JsonEngineSerializer.hpp>
#include <enginelab/serialization/YamlEngineSerializer.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>

using namespace enginelab;
using namespace enginelab::scripting;
using namespace std::chrono_literals;

namespace {

void require(bool condition, const char* message) {
    if (condition) return;
    std::cerr << "Engine-script test failure: " << message << '\n';
    std::exit(1);
}

void requireNear(double actual, double expected, double tolerance, const char* message) {
    require(std::isfinite(actual) && std::abs(actual - expected) <= tolerance, message);
}

class TemporaryDirectory final {
public:
    TemporaryDirectory() {
        const auto nonce = std::chrono::steady_clock::now().time_since_epoch().count();
        path_ = std::filesystem::temp_directory_path()
            / ("enginelab-script-tests-" + std::to_string(nonce));
        std::filesystem::create_directories(path_);
    }
    ~TemporaryDirectory() {
        std::error_code error;
        std::filesystem::remove_all(path_, error);
    }
    [[nodiscard]] const std::filesystem::path& path() const noexcept { return path_; }
private:
    std::filesystem::path path_;
};

void writeFile(const std::filesystem::path& path, std::string_view contents) {
    std::ofstream output(path, std::ios::binary | std::ios::trunc);
    output.write(contents.data(), static_cast<std::streamsize>(contents.size()));
    output.close();
    require(static_cast<bool>(output), "test fixture file should be written");
}

template <typename Predicate>
bool waitUntil(Predicate predicate, std::chrono::milliseconds timeout = 3s) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (predicate()) return true;
        std::this_thread::sleep_for(10ms);
    }
    return predicate();
}

bool hasDiagnostic(const EngineScriptCompileResult& result, std::string_view code) {
    return std::any_of(result.diagnostics.begin(), result.diagnostics.end(), [code](const auto& diagnostic) {
        return diagnostic.code == code;
    });
}

void testCompleteUnitAwareScript() {
    constexpr std::string_view script = R"(
preset inline_four
engine/name = "Unit-aware I4"
let bore = 8.6 cm
let runner = 0.30 m + 20 mm
set idle_rpm = 900 rpm
set engine.rotating_inertia_kg_m2 = 0.30 kg_m2
set engine.redline_rpm = 7.5 khz * 1 rpm / 1 hz
set ignition.rev_limit_rpm = 7.5 khz * 1 rpm / 1 hz
set intake.runner_length_mm = runner
set intake.runner_diameter_mm = 4.2 cm
set exhaust.primary_length_mm = 0.52 m
set exhaust.collector_volume_l = 2200 cc
set acoustic_observer.left_x_m = -0.2 m
set acoustic_observer.left_y_m = 3 m
set acoustic_observer.right_x_m = 0.2 m
set acoustic_observer.right_y_m = 3 m
set acoustic_observer.sound_speed_mps = 343 m_s
set injection.mode = direct
set injection.rail_pressure_bar = 20 MPa
set injection.wall_film_fraction = 8 percent
set solver.gas_substeps = 3
set forced_induction.enabled = true
set forced_induction.type = turbo
set forced_induction.pressure_ratio = 1.35
set cylinder.all.bore_mm = bore
set cylinder.2.ignition_offset_deg = -2 deg
ignition clear
ignition point 4000 rpm, 31 deg
ignition point 1000 rpm, 14 deg
)";

    // MPa is intentionally unsupported: verify diagnostics first, then compile the corrected input.
    EngineScriptCompiler compiler;
    const auto unsupported = compiler.compileText(script, "unsupported-unit.engine");
    require(!unsupported && !unsupported.diagnostics.empty(), "unknown compound unit should be rejected");

    auto corrected = std::string(script);
    const auto unitOffset = corrected.find("20 MPa");
    require(unitOffset != std::string::npos, "test script replacement should be found");
    corrected.replace(unitOffset, 6, "200 bar");
    // Frequency and engine-speed dimensions cannot be mixed, so make the RPM intent explicit.
    const std::string invalidRpm = "7.5 khz * 1 rpm / 1 hz";
    std::size_t searchFrom = 0;
    while ((searchFrom = corrected.find(invalidRpm, searchFrom)) != std::string::npos) {
        corrected.replace(searchFrom, invalidRpm.size(), "7500 rpm");
        searchFrom += 8;
    }
    const auto result = compiler.compileText(corrected, "complete.engine");
    if (!result) {
        for (const auto& item : result.diagnostics)
            std::cerr << item.code << " at " << item.location.line << ':' << item.location.column
                      << ": " << item.message << '\n';
    }
    require(static_cast<bool>(result), "complete engine script should compile");
    const auto& config = *result.config;
    require(config.name == "Unit-aware I4", "engine/name should update the engine name");
    requireNear(config.idleRpm, 900.0, 1.0e-12, "RPM literal should be assigned");
    requireNear(config.rotatingInertiaKgM2, 0.30, 1.0e-12,
                "legacy inertia setter should survive final topology normalization");
    double crankshaftInertia = 0.0;
    for (const auto& crankshaft : config.crankshafts)
        crankshaftInertia += crankshaft.momentOfInertiaKgM2
            * crankshaft.rotationRatio * crankshaft.rotationRatio;
    requireNear(crankshaftInertia, 0.30, 1.0e-12,
                "inertia setter should update the authoritative crankshaft topology");
    requireNear(config.intake.runnerLengthMm, 320.0, 1.0e-12, "mixed length units should add");
    requireNear(config.intake.runnerDiameterMm, 42.0, 1.0e-12, "centimetres should convert to millimetres");
    requireNear(config.exhaust.collectorVolumeLitres, 2.2, 1.0e-12, "cc should convert to litres");
    requireNear(config.acousticObserver.leftMicrophoneM.x, -0.2, 1.0e-12,
                "observer coordinates must remain SI metres after script unit conversion");
    requireNear(config.acousticObserver.rightMicrophoneM.y, 3.0, 1.0e-12,
                "observer distance must remain SI metres after script unit conversion");
    requireNear(config.acousticObserver.soundSpeedMps, 343.0, 1.0e-12,
                "observer sound speed must be script-authorable in SI");
    requireNear(config.injection.railPressureBar, 200.0, 1.0e-12, "pressure should convert back to bar field units");
    requireNear(config.injection.wallFilmFraction, 0.08, 1.0e-12, "percent should convert to a ratio");
    require(config.solver.gasSubsteps == 3, "integer property should be assigned exactly");
    require(config.forcedInduction.enabled, "boolean setting should be assigned");
    requireNear(config.cylinders[1].ignitionOffsetDegrees, -2.0, 1.0e-12,
                "single-cylinder selector should address IDs, not indices");
    require(std::all_of(config.cylinders.begin(), config.cylinders.end(), [](const auto& cylinder) {
        return std::abs(cylinder.boreMm - 86.0) < 1.0e-12;
    }), "cylinder.all should update all cylinders");
    require(config.ignition.timingCurve.size() == 2
            && config.ignition.timingCurve.front().rpm == 1'000.0,
            "ignition points should replace and sort the timing curve");
    require(std::all_of(config.intakePaths.begin(), config.intakePaths.end(), [](const auto& path) {
        return std::abs(path.geometry.runnerLengthMm - 320.0) < 1.0e-12;
    }), "global intake edits should update physical intake paths");
    require(std::all_of(config.exhaustPaths.begin(), config.exhaustPaths.end(), [](const auto& path) {
        return std::abs(path.geometry.primaryLengthMm - 520.0) < 1.0e-12;
    }), "global exhaust edits should update physical exhaust paths");
}

void testDiagnosticsAndSafetyLimits() {
    EngineScriptCompiler compiler;
    const auto mismatch = compiler.compileText(
        "preset inline_four\nset intake.runner_length_mm = 3 bar\n", "units.engine");
    require(!mismatch && hasDiagnostic(mismatch, "ES216"), "dimension mismatch should reject the script");
    const auto diagnostic = std::find_if(mismatch.diagnostics.begin(), mismatch.diagnostics.end(),
                                         [](const auto& item) { return item.code == "ES216"; });
    require(diagnostic != mismatch.diagnostics.end() && diagnostic->location.line == 2
            && diagnostic->location.column == 5, "diagnostic should preserve line and column");

    const auto division = compiler.compileText(
        "preset inline_four\nlet bad = 1 / 0\nset idle_rpm = bad * 1 rpm\n", "division.engine");
    require(!division && hasDiagnostic(division, "ES236"), "division by zero should be diagnosed");

    const auto missingCylinder = compiler.compileText(
        "preset inline_four\nset cylinder.99.bore_mm = 86 mm\n", "cylinder.engine");
    require(!missingCylinder && hasDiagnostic(missingCylinder, "ES228"),
            "unknown cylinder ID should not silently address an array index");

    EngineScriptCompileOptions limited;
    limited.maximumSourceBytes = 8;
    const auto tooLarge = compiler.compileText("preset inline_four", "large.engine", limited);
    require(!tooLarge && hasDiagnostic(tooLarge, "ES303"), "source-size limit should be enforced");
}

void testBaseFilesIncludesDependenciesAndCycles() {
    TemporaryDirectory directory;
    const auto yamlPath = directory.path() / "base.yaml";
    const auto jsonPath = directory.path() / "base.json";
    const auto includePath = directory.path() / "shared.engine";
    const auto rootPath = directory.path() / "root.engine";
    writeFile(yamlPath, YamlEngineSerializer {}.encode(makeDefaultV6()));
    writeFile(jsonPath, JsonEngineSerializer {}.encode(makeDefaultInlineTwo()));
    writeFile(includePath,
              "let shared_runner = 31 cm\nset intake.runner_length_mm = shared_runner\n");
    writeFile(rootPath,
              "base \"base.yaml\"\ninclude \"shared.engine\"\nengine \"Included V6\"\n");

    EngineScriptCompiler compiler;
    const auto yaml = compiler.compileFile(rootPath);
    require(yaml && yaml.config->cylinders.size() == 6, "YAML base should load before overrides");
    require(yaml.config->name == "Included V6", "include flow should continue into root statements");
    requireNear(yaml.config->intake.runnerLengthMm, 310.0, 1.0e-12,
                "included variables and statements should compile in shared context");
    require(yaml.dependencies.size() == 3, "root, include and base should all be tracked as dependencies");
    require(std::is_sorted(yaml.dependencies.begin(), yaml.dependencies.end()),
            "dependency order should be deterministic");

    const auto json = compiler.compileText("base \"base.json\"\nname \"JSON I2\"\n",
                                           directory.path() / "memory.engine",
                                           { directory.path() });
    require(json && json.config->cylinders.size() == 2 && json.config->name == "JSON I2",
            "JSON base should decode through the common serializer");

    const auto a = directory.path() / "a.engine";
    const auto b = directory.path() / "b.engine";
    writeFile(a, "include \"b.engine\"\n");
    writeFile(b, "include \"a.engine\"\n");
    const auto cycle = compiler.compileFile(a);
    require(!cycle && hasDiagnostic(cycle, "ES301"), "include cycles should be detected");
    require(cycle.dependencies.size() == 2, "cyclic include dependencies should still be tracked");
}

void testBackgroundHotReloadKeepsLastValidConfig() {
    TemporaryDirectory directory;
    const auto root = directory.path() / "root.engine";
    const auto shared = directory.path() / "shared.engine";
    writeFile(shared, "name \"First\"\n");
    writeFile(root, "preset inline_four\ninclude \"shared.engine\"\n");

    EngineScriptHotReloader reloader(root, {}, 20ms);
    reloader.start();
    require(waitUntil([&] { return reloader.poll()->attempt >= 1; }), "initial background compile should complete");
    const auto first = reloader.poll();
    require(first->lastAttemptSucceeded && first->revision == 1 && first->config
            && first->config->name == "First", "initial valid script should publish revision one");
    const auto firstConfig = first->config;

    // Change an included dependency without requestReload(): the polling watcher must detect it.
    std::this_thread::sleep_for(30ms);
    writeFile(shared, "name \"A much longer second name\"\n");
    require(waitUntil([&] { return reloader.poll()->revision >= 2; }),
            "included dependency change should trigger background reload");
    const auto second = reloader.poll();
    require(second->lastAttemptSucceeded && second->config->name == "A much longer second name",
            "dependency reload should publish the new valid configuration");

    writeFile(shared, "set intake.runner_length_mm = 2 bar\n");
    reloader.requestReload();
    const auto failedAttempt = second->attempt + 1;
    require(waitUntil([&] { return reloader.poll()->attempt >= failedAttempt; }),
            "explicit non-blocking reload request should compile on worker");
    const auto failed = reloader.poll();
    require(!failed->lastAttemptSucceeded && failed->revision == second->revision,
            "failed reload should not increment the active revision");
    require(failed->config.get() == second->config.get(),
            "failed reload should retain the exact last valid configuration snapshot");
    require(!failed->diagnostics.empty(), "failed reload diagnostics should remain observable");

    writeFile(shared, "name \"Recovered\"\n");
    reloader.requestReload();
    require(waitUntil([&] { return reloader.poll()->revision == second->revision + 1; }),
            "valid repair should publish the next revision");
    require(reloader.poll()->config->name == "Recovered", "recovered configuration should become active");
    reloader.stop();
    require(!reloader.running(), "stop should join the polling worker");
    require(firstConfig->name == "First", "previous immutable configuration remains safe for existing readers");
}

} // namespace

int main() {
    testCompleteUnitAwareScript();
    testDiagnosticsAndSafetyLimits();
    testBaseFilesIncludesDependenciesAndCycles();
    testBackgroundHotReloadKeepsLastValidConfig();
    std::cout << "EngineLab scripting tests passed\n";
    return 0;
}
