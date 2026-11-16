# Standalone audit reproductions

These are exact documentation copies of the sources/build scripts used for [the audit](../audit.md), collected 2026-11-16. The original runnable copies remain under `out/audit/`. They do not alter production code or the CMake project. To recreate a scratch file, copy its fenced contents to the indicated repository-relative path; run PowerShell build scripts from the repository root. Existing authoritative Windows VS Release libraries/objects and MSVC 14.44 are prerequisites, as documented in the register. Rebuilds against different libraries constitute a new experiment.

All fixture writes are confined to `out/audit/`. Use a fresh output folder for the UI/audio export probe, since it deliberately leaves collision and failure fixtures as evidence. The AudioAb wrapper shown below catches the exception (exit 73); the earlier uncaught run is preserved in the captured output. The scene summary first requires `EngineLabSceneExport --out out/audit/scenes`.

Executed commands:

```powershell
& out/audit/build_physics_probe.ps1
& out/audit/run-data-repro.ps1
& out/audit/run-script-repro.ps1
& out/audit/build-ui-audio-repro.ps1
& out/audit/ui_audio_repro.exe . out/audit/ui-audio-repeat
& out/audit/build-audio-ab-current.ps1
& out/audit/audio_ab_current.exe --root . --output out/audit/audio-ab-repeat
python out/audit/listening-repro.py
& out/build/windows-vs2022/tools/Release/EngineLabSceneExport.exe --out out/audit/scenes
python out/audit/scene-summary.py
```

## out/audit/physics_probe.cpp

```cpp
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/calibration/EcuCalibration.hpp>
#include <enginelab/runtime/DrivelineModel.hpp>
#include <enginelab/catalog/EngineDesign.hpp>
#include <enginelab/serialization/YamlEngineSerializer.hpp>
#include <iostream>
#include <iomanip>
#include <numbers>
#include <variant>
#include <stdexcept>
#include <limits>
#include <cmath>
using namespace enginelab;
void set(SimpleEcuModel& ecu, std::string_view key, double value) {
    auto d = calibration::makeDraft(*ecu.calibrationStore()->snapshot());
    auto p = d.find(key);
    if (!p) throw std::runtime_error(std::string(key));
    auto e = *p;
    std::get<calibration::ScalarCalibration>(e).value=value;
    d.set(std::move(e));
    auto published=ecu.calibrationStore()->publish(d);
    if (!published.published) throw std::runtime_error("publish failed");
    ecu.beginFrame();
}
int main() {
    std::cout << std::setprecision(15);
    auto c=makeDefaultInlineFour();
    c.forcedInduction.enabled=true;
    c.forcedInduction.type=ForcedInductionType::turbocharger;
    SimpleEcuModel ecu;
    ecu.initialiseCalibration(c);
    set(ecu, calibration::keys::antiLagEnabled,1);
    set(ecu, calibration::keys::antiLagMinimumRpm,2000);
    set(ecu, calibration::keys::antiLagMaximumSeconds,0.05);
    set(ecu, calibration::keys::antiLagExhaustLimit,1000);
    EngineState s;
    s.rpm=3000; s.gear=0; s.coolantTemperatureC=90; s.turbineInletTemperatureC=500;
    s.manifoldPressureKpa=101; s.throttle=0; s.angularVelocityRadPerSecond=3000*std::numbers::pi/30;
    EngineControls u; u.ignitionEnabled=true;
    EcuCommand cmd;
    for (int i=0;i<20;++i) { s.simulationTimeSeconds=1+i*.01; cmd=ecu.evaluate(c,s,u); }
    std::cout << "antilag_before_reset active="<<cmd.antiLagActive<<" blockers="<<cmd.antiLagBlockers<<"\n";
    ecu.reset(); s.simulationTimeSeconds=0;
    cmd=ecu.evaluate(c,s,u);
    std::cout << "antilag_after_reset active="<<cmd.antiLagActive<<" blockers="<<cmd.antiLagBlockers<<"\n";
    SimpleEcuModel fresh; fresh.initialiseCalibration(c);
    set(fresh,calibration::keys::antiLagEnabled,1);
    set(fresh,calibration::keys::antiLagMinimumRpm,2000);
    set(fresh,calibration::keys::antiLagMaximumSeconds,0.05);
    set(fresh,calibration::keys::antiLagExhaustLimit,1000);
    cmd=fresh.evaluate(c,s,u);
    std::cout << "antilag_fresh active="<<cmd.antiLagActive<<" blockers="<<cmd.antiLagBlockers<<"\n";

    auto vehicle=makeDefaultInlineFour();
    vehicle.transmission.automaticShifting=false;
    vehicle.transmission.gearRatios={1.0}; vehicle.transmission.finalDriveRatio=1.0;
    vehicle.transmission.shiftDurationSeconds=.01;
    vehicle.transmission.maxClutchTorqueNm=100;
    vehicle.transmission.clutchLockSpeedRpm=.1;
    vehicle.transmission.drivelineEfficiency=.5;
    vehicle.vehicle.tyreGripLimitEnabled=false;
    vehicle.vehicle.rollingResistanceCoefficient=0;
    vehicle.vehicle.dragCoefficient=.05;
    vehicle.vehicle.frontalAreaM2=.01;
    vehicle.vehicle.massKg=1;
    vehicle.vehicle.tireRadiusM=.1;
    DrivelineModel dl(vehicle); dl.requestGear(0);
    s.rpm=1000; s.angularVelocityRadPerSecond=s.rpm*std::numbers::pi/30;
    s.torqueNm=10; s.cycleAveragedTorqueNm=10; s.simulationTimeSeconds=1;
    DrivelineOutput o;
    for(int i=0;i<30000;++i) o=dl.advance(.001,s,0,1,0);
    s.rpm=900; s.angularVelocityRadPerSecond=s.rpm*std::numbers::pi/30;
    s.torqueNm=-10; s.cycleAveragedTorqueNm=-10;
    o=dl.advance(.001,s,0,1,0);
    const auto wheelOmega=o.vehicleSpeedMps/vehicle.vehicle.tireRadiusM;
    std::cout<<"backdrive slip_rpm="<<o.clutchSlipRpm<<" stick_fraction="<<o.clutchStickFraction<<" engine_reaction_nm="<<o.engineReactionTorqueNm<<" engine_coupling_nm="<<o.engineCouplingTorqueNm<<" reflected_inertia="<<o.reflectedRotatingInertiaKgM2<<" wheel_torque_nm="<<o.wheelTorqueNm<<" engine_rpm="<<s.rpm<<" wheel_rpm="<<wheelOmega*30/std::numbers::pi<<" engine_power_w="<<o.engineCouplingTorqueNm*s.angularVelocityRadPerSecond<<" wheel_power_w="<<o.wheelTorqueNm*wheelOmega<<" clutch_loss_w="<<o.clutchPowerLossKw*1000<<"\n";
    vehicle.ambientPressureKpa=50;
    DrivelineModel low(vehicle); low.requestGear(0);
    vehicle.ambientPressureKpa=120;
    DrivelineModel high(vehicle); high.requestGear(0);
    s.torqueNm=100; s.cycleAveragedTorqueNm=100;
    double maxDifference=0;
    for(int i=0;i<10000;++i){ auto a=low.advance(.001,s,0,1,0); auto b=high.advance(.001,s,0,1,0); maxDifference=std::max(maxDifference,std::abs(a.vehicleSpeedMps-b.vehicleSpeedMps)); }
    std::cout<<"aero_50_vs_120_kpa_max_speed_difference="<<maxDifference<<"\n";
    auto malformed=makeDefaultInlineFour();
    malformed.ignition.knockLimitCurve={{3000,-std::numeric_limits<double>::infinity(),101.325}};
    auto error=validateEngineConfig(malformed);
    SimpleEcuModel bad; bad.initialiseCalibration(malformed);
    s.rpm=3000; s.manifoldPressureKpa=40; s.simulationTimeSeconds=1;
    u.throttle=.1;
    auto badCommand=bad.evaluate(malformed,s,u);
    std::cout<<"infinite_knock_curve validation="<<(error?*error:"accepted")<<" advance="<<badCommand.ignitionAdvanceDegrees<<" finite="<<std::isfinite(badCommand.ignitionAdvanceDegrees)<<"\n";
    YamlEngineSerializer yaml;
    auto yamlText=yaml.encode(malformed);
    auto decoded=yaml.decode(yamlText);
    std::cout<<"infinite_knock_curve_yaml accepted="<<static_cast<bool>(decoded)<<" error="<<decoded.error<<"\n";
    malformed.ignition.knockLimitCurve={{5000,-10,101.325},{1000,30,101.325}};
    error=validateEngineConfig(malformed);
    std::cout<<"descending_knock_curve validation="<<(error?*error:"accepted")<<"\n";
    for (auto n : {8u,9u}) {
        EngineSpec spec; spec.name="Audit two-stroke"; spec.cycle=EngineCycle::twoStroke;
        spec.cylinderCount=n; spec.boreMm=54; spec.strokeMm=54; spec.use=EngineUse::roadMotorcycle;
        auto design=designCatalogueEngine(spec,".");
        std::cout<<"two_stroke_design cylinders="<<n<<" accepted="<<static_cast<bool>(design.config)<<" error="<<design.error<<"\n";
    }
}
```

## out/audit/build_physics_probe.ps1

```powershell
$ErrorActionPreference='Stop'
# Run from the repository root. No tracked source/build configuration is edited.
. scripts/vsenv.ps1
$inc=Get-ChildItem src -Directory | ForEach-Object {
    $p=Join-Path $_.FullName include
    if(Test-Path $p){ '/I'+$p }
}
$libs=Get-ChildItem out/build/windows-vs2022/src -Filter '*.lib' -Recurse |
    Where-Object { $_.DirectoryName -match '\\Release$' -and $_.Name -notmatch 'Fonts|App' } |
    ForEach-Object FullName
& cl.exe /nologo /std:c++20 /EHsc /O2 /MD @inc /Foout/audit/ /Feout/audit/physics_probe.exe `
    out/audit/physics_probe.cpp src/ecu/src/SimpleEcuModel.cpp src/calibration/src/EcuCalibration.cpp `
    src/runtime/src/DrivelineModel.cpp src/runtime/src/ShiftSchedule.cpp src/runtime/src/VariatorControl.cpp `
    @libs out/build/windows-vs2022/_deps/yaml_cpp-build/Release/yaml-cpp.lib
if($LASTEXITCODE -ne 0){ throw 'Probe build failed' }
& out/audit/physics_probe.exe | Tee-Object -FilePath out/audit/physics_probe.log
if($LASTEXITCODE -ne 0){ throw 'Probe run failed' }
```

## out/audit/data-repro.cpp

```cpp
#include <enginelab/catalog/SavedEngines.hpp>
#include <enginelab/foundation/EngineTypes.hpp>
#include <enginelab/serialization/JsonEngineSerializer.hpp>
#include <enginelab/serialization/YamlEngineSerializer.hpp>
#include <enginelab/runtime/DataLogger.hpp>
#include <nlohmann/json.hpp>
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <iostream>

using namespace enginelab;
int main() {
    const std::filesystem::path folder = "out/audit/data-fixtures";
    std::filesystem::create_directories(folder);
    auto config = makeBaseEnginePresets().front();
    normaliseEngineConfig(config);
    std::cout << "base_valid=" << !validateEngineConfig(config) << '\n';
    const auto first = savedEngineFile(folder, "A?B");
    const auto second = savedEngineFile(folder, "A*B");
    std::cout << "collision_paths_equal=" << (first == second)
              << " first=" << first.filename().string() << '\n';
    config.name = "A?B";
    std::cout << "first_save_error=" << saveEngine(first, config, nullptr).value_or("none") << '\n';
    config.name = "A*B";
    std::cout << "second_save_error=" << saveEngine(second, config, nullptr).value_or("none") << '\n';
    const auto read = readSavedEngine(first);
    std::cout << "first_file_now_contains=" << (read.engine ? read.engine->config.name : read.error) << '\n';
    const auto upperEcu = savedEngineFile(folder, "My.ECU");
    std::cout << "upper_ecu_name_error=" << savedEngineNameError("My.ECU", {}).value_or("none")
              << " filename=" << upperEcu.filename().string()
              << " classified_as_calibration=" << (droppedFileKind(upperEcu) == DroppedFileKind::calibration) << '\n';
    const auto deviceFile = savedEngineFile(folder, "CON");
    std::cout << "reserved_name_error=" << savedEngineNameError("CON", {}).value_or("none")
              << " save_error=" << saveEngine(deviceFile, config, nullptr).value_or("none") << '\n';
    config.name = "Range-check";
    config.octaneRating = 150.0;
    auto jsonText = JsonEngineSerializer{}.encode(config);
    auto yamlText = YamlEngineSerializer{}.encode(config);
    const auto jsonRead = decodeEngineFile(jsonText, EngineFileFormat::json, "range.json");
    const auto yamlRead = decodeEngineFile(yamlText, EngineFileFormat::yaml, "range.yaml");
    std::cout << "range_json_loaded=" << bool(jsonRead.engine)
              << " octane=" << (jsonRead.engine ? jsonRead.engine->config.octaneRating : -1.0)
              << " warnings=" << jsonRead.warnings.size() << '\n';
    std::cout << "range_yaml_loaded=" << bool(yamlRead.engine) << " error=" << yamlRead.error << '\n';
    config.octaneRating = 98.0;
    auto encoded = nlohmann::json::parse(JsonEngineSerializer{}.encode(config));
    encoded["schema_version"] = 4294967305ULL;
    const auto oversizedSchema = JsonEngineSerializer{}.decode(encoded.dump());
    std::cout << "oversized_schema_loaded=" << bool(oversizedSchema)
              << " loaded_schema=" << (oversizedSchema ? oversizedSchema.config->schemaVersion : 0) << '\n';
    encoded["schema_version"] = 9;
    encoded["engine"]["cylinders"][0]["intake_valve_count"] = 1.9;
    const auto fractionalValveCount = JsonEngineSerializer{}.decode(encoded.dump());
    std::cout << "fractional_valve_count_loaded=" << bool(fractionalValveCount)
              << " count=" << (fractionalValveCount ? fractionalValveCount.config->cylinders[0].intakeValveCount : 0) << '\n';
    config.transmission.gearRatios.assign(200000, 1.0);
    std::cout << "large_engine_valid=" << !validateEngineConfig(config) << '\n';
    const auto compact = nlohmann::json::parse(JsonEngineSerializer{}.encode(config)).dump();
    std::cout << "large_compact_bytes=" << compact.size()
              << " compact_decodes=" << bool(JsonEngineSerializer{}.decode(compact)) << '\n';
    const auto largeFile = folder / "large.json";
    std::cout << "large_save_error=" << saveEngine(largeFile, config, nullptr).value_or("none")
              << " file_bytes=" << std::filesystem::file_size(largeFile) << '\n';
    const auto largeRead = readSavedEngine(largeFile);
    std::cout << "large_saved_file_loaded=" << bool(largeRead.engine) << " error=" << largeRead.error << '\n';
    const auto logFile = folder / "same-second.csv";
    DataLogger logger;
    EngineState state;
    state.rpm = 4321.0;
    const auto firstLogOpened = logger.start(logFile);
    logger.push(state);
    logger.stop();
    const auto secondLogOpened = logger.start(logFile);
    logger.stop();
    std::ifstream logIn(logFile);
    const std::string finalLog((std::istreambuf_iterator<char>(logIn)), std::istreambuf_iterator<char>());
    std::cout << "log_both_opened=" << (firstLogOpened && secondLogOpened)
              << " first_log_row_preserved=" << (finalLog.find("4321") != std::string::npos) << '\n';
    return 0;
}
```

## out/audit/run-data-repro.ps1

```powershell
$ErrorActionPreference = 'Stop'
. scripts/vsenv.ps1
$auditIncludes = Get-ChildItem -LiteralPath src -Directory | ForEach-Object { '/I' + (Join-Path $_.FullName 'include') }
$auditIncludes += '/Iout/build/windows-vs2022/_deps/nlohmann_json-src/include'
$auditIncludes += '/Iout/build/windows-vs2022/_deps/yaml_cpp-src/include'
$auditLibs = @(
    'out/build/windows-vs2022/src/catalog/Release/EngineLabCatalog.lib',
    'out/build/windows-vs2022/src/serialization/Release/EngineLabSerialization.lib',
    'out/build/windows-vs2022/src/calibration/Release/EngineLabCalibration.lib',
    'out/build/windows-vs2022/src/foundation/Release/EngineLabFoundation.lib',
    'out/build/windows-vs2022/src/runtime/Release/EngineLabRuntime.lib',
    'out/build/windows-vs2022/_deps/yaml_cpp-build/Release/yaml-cpp.lib'
)
& cl /nologo /std:c++20 /EHsc /MD /O2 @auditIncludes out/audit/data-repro.cpp /Foout/audit/data-repro.obj /Feout/audit/data-repro.exe /link @auditLibs
if ($LASTEXITCODE -ne 0) { throw 'Compile failed.' }
& out/audit/data-repro.exe
```

## out/audit/script-repro.cpp

```cpp
#include <enginelab/scripting/EngineScriptCompiler.hpp>
#include <enginelab/calibration/CalibrationFileHotReloader.hpp>
#include <enginelab/calibration/CalibrationJson.hpp>
#include <windows.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>

int main(int argc, char** argv) {
    enginelab::scripting::EngineScriptCompiler compiler;
    if (argc > 1 && std::string(argv[1]) == "nesting") {
        const std::size_t count = argc > 2 ? std::stoull(argv[2]) : 10000;
        const std::string script = "preset inline_four\nset idle_rpm = " + std::string(count, '(')
            + "900 rpm" + std::string(count, ')') + "\n";
        std::cout << "script_bytes=" << script.size() << " parentheses=" << count << std::endl;
        const auto compiled = compiler.compileText(script);
        std::cout << "returned=" << bool(compiled) << " diagnostics=" << compiled.diagnostics.size() << std::endl;
        return 0;
    }
    const std::filesystem::path folder = std::filesystem::absolute("out/audit/script-fixtures");
    std::filesystem::create_directories(folder);
    const auto accentedFile = folder / std::filesystem::path(std::u8string(u8"\u00e9.engine"));
    std::ofstream(accentedFile) << "set idle_rpm = 900 rpm\n";
    enginelab::scripting::EngineScriptCompileOptions options;
    options.baseDirectory = folder;
    const auto compiled = compiler.compileText("preset inline_four\ninclude \"\xc3\xa9.engine\"\n", "<memory>", options);
    std::cout << "windows_ansi_code_page=" << GetACP() << '\n';
    std::cout << "accented_include_exists=" << std::filesystem::exists(accentedFile)
              << " compile_success=" << bool(compiled) << '\n';
    for (const auto& diagnostic : compiled.diagnostics)
        std::cout << diagnostic.code << ": " << diagnostic.message << '\n';
    const auto calibrationPath = folder / std::filesystem::path(std::u8string(u8"\u00e9.ecu.json"));
    std::ofstream(calibrationPath) << "{\"schema_version\":1,\"calibrations\":[]}";
    const auto u8Path = calibrationPath.u8string();
    const std::string utf8Path(reinterpret_cast<const char*>(u8Path.data()), u8Path.size());
    enginelab::calibration::CalibrationStore store;
    enginelab::calibration::CalibrationFileHotReloader reloader(store);
    reloader.watch(utf8Path);
    const auto reloaded = reloader.poll();
    std::cout << "accented_calibration_exists=" << std::filesystem::exists(calibrationPath)
              << " watcher_error=" << reloaded.error << '\n';
    return 0;
}
```

## out/audit/run-script-repro.ps1

```powershell
$ErrorActionPreference = 'Stop'
. scripts/vsenv.ps1
$auditIncludes = Get-ChildItem -LiteralPath src -Directory | ForEach-Object { '/I' + (Join-Path $_.FullName 'include') }
$auditIncludes += '/Iout/build/windows-vs2022/_deps/nlohmann_json-src/include'
$auditIncludes += '/Iout/build/windows-vs2022/_deps/yaml_cpp-src/include'
$auditLibs = @(
    'out/build/windows-vs2022/src/scripting/Release/EngineLabScripting.lib',
    'out/build/windows-vs2022/src/serialization/Release/EngineLabSerialization.lib',
    'out/build/windows-vs2022/src/foundation/Release/EngineLabFoundation.lib',
    'out/build/windows-vs2022/src/calibration/Release/EngineLabCalibration.lib',
    'out/build/windows-vs2022/_deps/yaml_cpp-build/Release/yaml-cpp.lib'
)
& cl /nologo /std:c++20 /EHsc /MD /O2 @auditIncludes out/audit/script-repro.cpp /Foout/audit/script-repro.obj /Feout/audit/script-repro.exe /link @auditLibs
if ($LASTEXITCODE -ne 0) { throw 'Compile failed.' }
& out/audit/script-repro.exe
& out/audit/script-repro.exe nesting 100
'small_nesting_exit=' + $LASTEXITCODE
& out/audit/script-repro.exe nesting 10000
'large_nesting_exit=' + $LASTEXITCODE
```

## out/audit/ui_audio_repro.cpp

```cpp
#include <enginelab/app/ActionMap.hpp>
#include <enginelab/audio/OfflineAudioExporter.hpp>
#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/app/EcuTunerWindow.hpp>
#include <enginelab/calibration/EcuCalibration.hpp>
#include <enginelab/calibration/EcuCalibrationKeys.hpp>
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include "../../src/audio/src/OfflineRenderChain.hpp"
#include <fstream>
#include <iostream>
#include <filesystem>

namespace fs = std::filesystem;
static std::string read(const fs::path& p) {
    std::ifstream f(p, std::ios::binary);
    return {std::istreambuf_iterator<char>(f), {}};
}
int main(int argc, char** argv) {
    if (argc != 3) return 2;
    const fs::path root = argv[1], output = argv[2];
    juce::ScopedJuceInitialiser_GUI juceInit;
    enginelab::ActionMap map;
    juce::String error;
    const auto accepted = map.fromJson("{\"pause\":\"shift + Q\"}", error);
    const auto key = juce::KeyPress::createFromDescription("shift + Q");
    std::cout << "shift_conflict accepted=" << accepted
        << " pause=" << map.matches(enginelab::AppAction::pause, key)
        << " throttle_idle=" << map.matches(enginelab::AppAction::throttleIdle, key)
        << " error=" << error << '\n';
    const auto acceptedSave = map.fromJson("{\"pause\":\"ctrl + S\"}", error);
    std::cout << "reserved_save accepted=" << acceptedSave
        << " pause=" << map.matches(enginelab::AppAction::pause,
                juce::KeyPress::createFromDescription("ctrl + S"))
        << " error=" << error << '\n';

    auto catalog = enginelab::loadEngineCatalog(root);
    const auto choice = enginelab::selectSingleEngineCatalogEntry(catalog.entries, "K20");
    if (!choice) { std::cout << "catalog selection failed\n"; return 3; }
    enginelab::OfflineAudioExportRequest request;
    request.engine = choice.entry->config;
    auto tunedStore = std::make_shared<enginelab::calibration::CalibrationStore>();
    auto tunedDraft = enginelab::calibration::makeDefaultEcuCalibration(request.engine);
    auto limit = std::get<enginelab::calibration::ScalarCalibration>(*tunedDraft.find(enginelab::calibration::keys::revLimit));
    limit.value = 3000.;
    tunedDraft.set(limit);
    const auto tunedPublished = tunedStore->publish(tunedDraft);
    std::cout << "live_tuned_limit published=" << tunedPublished.published << " limit="
        << enginelab::activeRevLimitRpm(request.engine, *tunedStore->snapshot()) << '\n';
    auto renderChain = std::make_unique<enginelab::offline::RenderChain>(request.engine, request.mix);
    enginelab::EngineState testState;
    testState.rpm = 4000.;
    testState.manifoldPressureKpa = request.engine.ambientPressureKpa;
    enginelab::EngineControls testControls;
    testControls.ignitionEnabled = true;
    testControls.throttle = 1.;
    enginelab::SimpleEcuModel exportedDefaults;
    exportedDefaults.initialise(request.engine);
    enginelab::SimpleEcuModel liveTuned(tunedStore);
    liveTuned.initialise(request.engine);
    const auto liveCommand = liveTuned.evaluate(request.engine, testState, testControls);
    const auto exportCommand = exportedDefaults.evaluate(request.engine, testState, testControls);
    std::cout << "at_4000 live_limiter=" << liveCommand.hardRevLimiterActive
        << " export_default_limiter=" << exportCommand.hardRevLimiterActive
        << " config_limit=" << request.engine.ignition.revLimitRpm << '\n';
    renderChain.reset();

    const auto turbo = enginelab::selectSingleEngineCatalogEntry(catalog.entries, "2JZ");
    if (turbo) {
        auto tunerStore = std::make_shared<enginelab::calibration::CalibrationStore>();
        const auto tunerPublish = tunerStore->publish(enginelab::calibration::makeDefaultEcuCalibration(turbo.entry->config));
        std::cout << "tuner_store published=" << tunerPublish.published << '\n';
        enginelab::EcuTunerComponent tuner(tunerStore);
        tuner.setVtecFittable(true);
        tuner.setSize(720, 450);
        for (auto* child : tuner.getChildren()) {
            if (auto* button = dynamic_cast<juce::Button*>(child); button && button->isVisible())
                std::cout << "tuner_button '" << button->getButtonText() << "' bounds=" << button->getBounds().toString() << '\n';
        }
        for (auto* child : tuner.getChildren()) {
            if (auto* selector = dynamic_cast<juce::ComboBox*>(child)) {
                for (int i = 0; i < selector->getNumItems(); ++i) {
                    if (selector->getItemText(i).contains("Anti-lag settings")) {
                        selector->setSelectedItemIndex(i, juce::sendNotificationSync);
                        break;
                    }
                }
            }
        }
        int count=0, outside=0, overlapping=0;
        for (auto* child : tuner.getChildren()) {
            if (auto* editor = dynamic_cast<juce::TextEditor*>(child)) {
                ++count;
                if (!tuner.getLocalBounds().contains(editor->getBounds())) ++outside;
                if (editor->getBottom() > tuner.getHeight()-68) ++overlapping;
                std::cout << "antilag_editor '" << editor->getText() << "' bounds=" << editor->getBounds().toString() << '\n';
            }
        }
        std::cout << "antilag_editor_count=" << count << " outside=" << outside << " overlapping_toolbar=" << overlapping << '\n';
        fs::create_directories(output);
        const auto shot = tuner.createComponentSnapshot(tuner.getLocalBounds());
        juce::PNGImageFormat png;
        juce::FileOutputStream pngFile(juce::File(fs::absolute(output / "tuner-minimum.png").string()));
        png.writeImageToStream(shot, pngFile);
    }
    request.scenario = {"audit", {{"starter", .1, true, true, false, .2, .2, 0., 0., 0., 0.}}};
    request.sampleRateHz = 48000;
    request.writeStems = false;
    request.loadAuthoredImpulseResponses = false;
    request.outputDirectory = output / "failed-replacement";
    fs::create_directories(request.outputDirectory / "scenario.json");
    { std::ofstream f(request.outputDirectory / "scenario.json" / "blocker.txt"); f << "blocked"; }
    { std::ofstream f(request.outputDirectory / "master.wav"); f << "EXISTING MASTER SENTINEL"; }
    request.overwriteExistingFiles = true;
    auto failed = enginelab::exportOfflineAudio(request);
    std::cout << "failed_replace success=" << failed.success << " error=" << failed.error << '\n'
        << "old_master_preserved=" << (read(request.outputDirectory / "master.wav") == "EXISTING MASTER SENTINEL")
        << " new_master_bytes=" << fs::file_size(request.outputDirectory / "master.wav") << '\n';

    request.outputDirectory = output / "tap-preflight";
    request.overwriteExistingFiles = false;
    std::uint32_t ductId = 0;
    if (!request.engine.exhaustPaths.empty() && request.engine.exhaustPaths[0].network) {
        for (const auto& c : request.engine.exhaustPaths[0].network->components)
            if (c.type == enginelab::ExhaustComponentType::pipe) { ductId = c.id; break; }
    }
    std::cout << "tap duct=" << ductId << '\n';
    if (ductId != 0) {
        request.exhaustPressureTap = enginelab::OfflineAudioExportRequest::ExhaustPressureTap {ductId, .5};
        fs::create_directories(request.outputDirectory);
        { std::ofstream f(request.outputDirectory / "exhaust_tap_pa.f32"); f << "EXISTING TAP SENTINEL"; }
        const auto tap = enginelab::exportOfflineAudio(request);
        std::cout << "tap_export success=" << tap.success << " error=" << tap.error << '\n'
            << "old_tap_preserved=" << (read(request.outputDirectory / "exhaust_tap_pa.f32") == "EXISTING TAP SENTINEL")
            << " tap_bytes=" << fs::file_size(request.outputDirectory / "exhaust_tap_pa.f32") << '\n';
    }
}
```

## out/audit/build-ui-audio-repro.ps1

```powershell
$ErrorActionPreference = 'Stop'
. "$PSScriptRoot/../../scripts/vsenv.ps1"
$auditRoot = (Resolve-Path "$PSScriptRoot/../..").Path
[xml]$project = Get-Content "$auditRoot/out/build/windows-vs2022/tests/EngineLabOfflineAudioExportTests.vcxproj"
$group = @($project.Project.ItemDefinitionGroup) | Where-Object { $_.Condition -like '*Release|x64*' } | Select-Object -First 1
$includes = $group.ClCompile.AdditionalIncludeDirectories.Split(';') | Where-Object { $_ -notlike '%*' }
$includes += "$auditRoot/src/app/include"
$includes += "$auditRoot/out/build/windows-vs2022/_deps/juce-src/modules"
$argsCompile = @('/nologo','/std:c++20','/EHsc','/MD','/O2','/utf-8','/DNDEBUG','/DJUCE_GLOBAL_MODULE_SETTINGS_INCLUDED=1','/DJUCE_WEB_BROWSER=0','/DJUCE_USE_CURL=0','/DYAML_CPP_STATIC_DEFINE')
foreach ($dir in $includes) { $argsCompile += "/I$dir" }
$argsCompile += @('/c',"$auditRoot/out/audit/ui_audio_repro.cpp","$auditRoot/src/app/src/ActionMap.cpp","/Fo$auditRoot/out/audit/")
& cl.exe @argsCompile
if ($LASTEXITCODE) { exit $LASTEXITCODE }
$deps = $group.Link.AdditionalDependencies.Split(';') | Where-Object { $_ -and $_ -notlike '%*' }
$argsLink = @('/NOLOGO','/SUBSYSTEM:CONSOLE','/STACK:8388608',"/OUT:$auditRoot/out/audit/ui_audio_repro.exe","$auditRoot/out/audit/ui_audio_repro.obj","$auditRoot/out/audit/ActionMap.obj")
$argsLink += @("$auditRoot/out/build/windows-vs2022/src/app/EngineLabApp.dir/Release/EcuTunerWindow.obj","$auditRoot/out/build/windows-vs2022/src/app/EngineLabApp.dir/Release/Theme.obj","$auditRoot/out/build/windows-vs2022/src/app/Release/EngineLabFonts.lib")
foreach ($dep in $deps) {
    if ($dep.StartsWith('..')) { $argsLink += [System.IO.Path]::GetFullPath("$auditRoot/out/build/windows-vs2022/tests/$dep") }
    else { $argsLink += $dep }
}
& link.exe @argsLink
exit $LASTEXITCODE
```

## out/audit/audio_ab_current.cpp

```cpp
// Audit wrapper: compile the owning source without modifying the repository.
#define ENGINELAB_CATALOG_ROOT "."
#define main audit_harness_entry
#include "../../tools/AudioAbHarness.cpp"
#undef main

int main(int argc, char** argv) {
    try {
        return audit_harness_entry(argc, argv);
    } catch (const std::exception& exception) {
        std::cerr << "AUDIT_CAUGHT_EXCEPTION: " << exception.what() << '\n';
        return 73;
    }
}
```

## out/audit/build-audio-ab-current.ps1

```powershell
$ErrorActionPreference = 'Stop'
. "$PSScriptRoot/../../scripts/vsenv.ps1"
$auditRoot = (Resolve-Path "$PSScriptRoot/../..").Path
[xml]$project = Get-Content "$auditRoot/out/build/windows-vs2022/tests/EngineLabOfflineAudioExportTests.vcxproj"
$group = @($project.Project.ItemDefinitionGroup) | Where-Object { $_.Condition -like '*Release|x64*' } | Select-Object -First 1
$includes = $group.ClCompile.AdditionalIncludeDirectories.Split(';') | Where-Object { $_ -notlike '%*' }
$includes += "$auditRoot/out/build/windows-vs2022/_deps/juce-src/modules"
$argsCompile = @('/nologo','/std:c++20','/EHsc','/MD','/O2','/utf-8','/DNDEBUG','/DJUCE_GLOBAL_MODULE_SETTINGS_INCLUDED=1','/DJUCE_WEB_BROWSER=0','/DJUCE_USE_CURL=0','/DYAML_CPP_STATIC_DEFINE')
foreach ($dir in $includes) { $argsCompile += "/I$dir" }
$argsCompile += @('/c',"$auditRoot/out/audit/audio_ab_current.cpp","/Fo$auditRoot/out/audit/audio_ab_current.obj")
& cl.exe @argsCompile
if ($LASTEXITCODE) { exit $LASTEXITCODE }
$deps = $group.Link.AdditionalDependencies.Split(';') | Where-Object { $_ -and $_ -notlike '%*' }
$argsLink = @('/NOLOGO','/SUBSYSTEM:CONSOLE','/STACK:8388608',"/OUT:$auditRoot/out/audit/audio_ab_current.exe","$auditRoot/out/audit/audio_ab_current.obj")
foreach ($dep in $deps) {
    if ($dep.StartsWith('..')) { $argsLink += [System.IO.Path]::GetFullPath("$auditRoot/out/build/windows-vs2022/tests/$dep") }
    else { $argsLink += $dep }
}
& link.exe @argsLink
exit $LASTEXITCODE
```

## out/audit/listening-repro.py

```python
from pathlib import Path
import contextlib
import csv
import importlib.util
import io
import json

root = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('engine_listening', root / 'scripts/listening.py')
listening = importlib.util.module_from_spec(spec)
spec.loader.exec_module(listening)
base = root / 'out/audit/listening-cases'
base.mkdir(parents=True, exist_ok=True)

def run_case(name, pairs, rows):
    pack = base / name
    (pack / 'responses').mkdir(parents=True, exist_ok=True)
    key = {'mode':'variant', 'baseline_engine':'control', 'candidate_engine':'candidate',
           'seed':1, 'target_lufs':-18, 'pairs':pairs}
    (pack / 'listening-key.json').write_text(json.dumps(key), encoding='utf-8')
    with (pack / 'responses/listener_01.csv').open('w', newline='', encoding='utf-8') as handle:
        writer = csv.writer(handle)
        writer.writerow(listening.COLUMNS)
        writer.writerows(rows)
    try:
        with contextlib.redirect_stdout(io.StringIO()), contextlib.redirect_stderr(io.StringIO()):
            result = listening.report(pack, False)
        data = json.loads((pack / 'listening-report.json').read_text(encoding='utf-8'))
        text = (pack / 'listening-report.md').read_text(encoding='utf-8')
        print(json.dumps({'case':name, 'exit':result, 'listeners':len(data['listeners']),
              'judgements':data['judgements'], 'dropped_rows':data['dropped_rows'],
              'per_pair_n':{k:v['n'] for k,v in data['per_pair'].items()},
              'sign_test_p':data['global']['most_realistic']['sign_test_p'],
              'warning': 'worthless' in text}, allow_nan=False))
    except Exception as exc:
        print(json.dumps({'case':name, 'exception':type(exc).__name__, 'message':str(exc)}))

pair = lambda n, matched=True: {'pair':n, 'engine':'test-engine', 'segment':'idle',
       'subject_side':'A', 'control_present':True,
       'control':{'match_quality':'exact-platform', 'window_condition_matched':matched}}
row = lambda n: [n,5,1,5,1,'A','A','']
run_case('missing-pair', [pair(1),pair(2)], [row(1)])
run_case('duplicate-rows', [pair(1)], [row(1)] * 20)
run_case('unmatched-window', [pair(1,False)], [row(1)])
run_case('1024-judgements', [pair(1)], [row(1)] * 1024)
```

## out/audit/scene-summary.py

```python
from pathlib import Path
import json
import math
root = Path(__file__).resolve().parents[2]
folder = root/'out/audit/scenes'
names = ['ductClash','selfClash','engineClash','tightBend','stretched']
for entry in json.loads((folder/'index.json').read_text()):
    data = json.loads((folder/entry['file']).read_text())
    counts = {name:sum(i['kind']==k for i in data['issues']) for k,name in enumerate(names)}
    if not any(counts.values()):
        continue
    print(data['name'], counts)
    for k in (0,1,3,4):
        issues = [i for i in data['issues'] if i['kind']==k]
        if not issues: continue
        issue = (min if k==3 else max)(issues, key=lambda i:i['value'])
        duct = data['ducts'][issue['duct']]
        drawn = sum(math.dist(a,b) for a,b in zip(duct['centreline'],duct['centreline'][1:]))
        print(json.dumps({'issue':names[k], 'duct':issue['duct'], 'kind':duct['kind'],
              'path':duct['path'], 'element':duct['element'], 'other':issue['other'],
              'value':issue['value'], 'authored_mm':duct['authored'],
              'drawn_mm':round(drawn,3), 'where_mm':issue['where']}))
```

