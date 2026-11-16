# Continuation reproduction sources

These are isolated audit witnesses, not proposed fixes. Restore each block to its labelled ignored out/ path if needed; scripts expect the repository root/current authoritative build as stated. Sources and results are tied to the coverage hashes. Production owner files are read/compiled rather than patched.

The full registered baseline ran before all heavy probes; standalone sources compile with the project MSVC environment. Linux/performance commands and limitations are recorded separately when complete.

## out/audit2/physics/component_probe.cpp

````cpp
#include <enginelab/physics/ConservativeGasSystem.hpp>
#include <enginelab/gasdynamics/ExhaustGasNetwork.hpp>
#include <array>
#include <optional>
#include <algorithm>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>
namespace enginelab {
struct AirboxSlug { double massKg {}; double fuelMoles {}; };
#include "throttle_helper.inc"
}
using namespace enginelab;
using namespace enginelab::gasdynamics;

struct RingProbe {
    struct Knot { double timeSeconds {}; bool valid {}; };
    struct { double simulationTimeSeconds {}; } state_;
    static constexpr std::size_t exhaustBoundaryKnotCount=8;
    static constexpr double exhaustBoundaryReconstructionDelaySeconds=625e-6;
    std::array<std::array<Knot,8>,1> exhaustBoundaryKnots_ {};
    std::size_t exhaustBoundaryKnotWrite_ {};
    double sample(bool flushExhaustNetwork) {
        const auto index=std::size_t{0};
#include "ring_helper.inc"
        return blend(from.timeSeconds,to.timeSeconds);
    }
    void push(double t) {
        state_.simulationTimeSeconds=t;
        exhaustBoundaryKnots_[0][exhaustBoundaryKnotWrite_]={t,true};
    }
    void next() { exhaustBoundaryKnotWrite_=(exhaustBoundaryKnotWrite_+1)%8; }
};

ExhaustNetworkLayout layout(std::size_t cells,std::size_t runners=1) {
    std::vector<CompiledExhaustDuct> ducts;
    std::vector<CompiledCylinderPort> ports;
    std::vector<CompiledExhaustOutlet> mouths;
    for(std::size_t i=0;i<runners;++i){
        CompiledExhaustDuct d; d.nodeId=100+static_cast<unsigned>(i);
        d.lengthM=1; d.inletFlowAreaM2=0.004; d.outletFlowAreaM2=0.001;
        d.flowAreaM2=(.004+.002+.001)/3; d.inletConnectionAreaM2=.004;
        d.outletConnectionAreaM2=.001; d.connectionAreaM2=d.flowAreaM2;
        d.hydraulicDiameterM=.05; d.cellCount=cells;
        ducts.push_back(d);
        CompiledCylinderPort p; p.cylinderId=1+static_cast<unsigned>(i);
        p.networkEndpoint={ExhaustEndpointType::ductInlet,i,d.nodeId};
        p.runnerConnectionAreaM2=.004; ports.push_back(p);
        CompiledExhaustOutlet m; m.outletNodeId=200+static_cast<unsigned>(i);
        m.networkEndpoint={ExhaustEndpointType::ductOutlet,i,d.nodeId};
        m.openingAreaM2=.001; mouths.push_back(m);
    }
    return ExhaustNetworkLayout::assemble(ducts,{}, {},ports,mouths);
}
struct Executor {
    std::size_t maxCount{}, calls{};
    static void run(void* raw,std::size_t count,ExhaustGasNetwork::ParallelFor::Body body,void* context) noexcept {
        auto& e=*static_cast<Executor*>(raw); e.maxCount=std::max(e.maxCount,count); ++e.calls;
        for(auto i=count;i>0;--i)body(context,i-1);
    }
};
int main(){
    std::cout<<std::setprecision(17);
    GasCell withFuel,control;
    withFuel.initialise(80,1,300); control=withFuel;
    AirboxSlug slug{0.001,0.0001},empty{};
    const auto before=withFuel.massKg();
    throttleFlowKeepingReversedFuel(withFuel,slug,10,101.325,300,1e-5,.7,1e-4);
    throttleFlowKeepingReversedFuel(control,empty,10,101.325,300,1e-5,.7,1e-4);
    const auto returned=withFuel.mixture().fuelMoles;
    std::cout<<"slug fresh_boundary_inflow_kg="<<control.massKg()-before
             <<" returned_fuel_kg="<<returned*GasCell::gasolineMolarMassKg
             <<" extra_plenum_mass_kg="<<withFuel.massKg()-control.massKg()
             <<" unchanged_oxygen="<<(withFuel.mixture().oxygenMoles==control.mixture().oxygenMoles)<<'\n';
    RingProbe r; double t=0,last=0,maxJump=0,maxError=0;
    for(int i=0;i<80;++i){
        t+=i<20?125e-6:30e-6; r.push(t);
        auto reconstructed=r.sample(true);
        if(i>=40){maxJump=std::max(maxJump,std::abs(reconstructed-last));
            maxError=std::max(maxError,std::abs(reconstructed-(t-625e-6)));}
        last=reconstructed;r.next();
    }
    std::cout<<"ring fine_delay_us="<<(t-last)*1e6<<" target_delay_us=625 max_error_us="<<maxError*1e6<<'\n';
    r.state_.simulationTimeSeconds=t+10e-6; const auto a=r.sample(false);
    r.state_.simulationTimeSeconds=t+20e-6; const auto b=r.sample(false);
    std::cout<<"ring between_knots_output_delta_us="<<(b-a)*1e6<<" input_delta_us=10\n";
    ExhaustGasNetwork coarse,fine;
    if(!coarse.configure(layout(2))||!fine.configure(layout(3)))return 2;
    auto& duct=coarse.ducts().front();
    duct.cells()[1]=*coarse.mixtureModel().conservativeFromPressureTemperature(202650,300);
    const auto prior=coarse.inventory();
    if(!fine.adoptStateFrom(coarse))return 3;
    const auto after=fine.inventory();
    const auto mass=[](const auto& x){double m=0;for(double s:x.speciesMassKg)m+=s;return m;};
    std::cout<<"taper remap_before_mass_kg="<<mass(prior)<<" after_mass_kg="<<mass(after)
             <<" relative_mass_change="<<mass(after)/mass(prior)-1
             <<" relative_energy_change="<<after.totalEnergyJ/prior.totalEnergyJ-1<<'\n';
    ExhaustGasNetwork n; if(!n.configure(layout(3,6)))return 4;
    Executor e; n.setParallelFor({&Executor::run,&e});
    const auto ambient=*n.mixtureModel().conservativeFromPressureTemperature(101325,300);
    const auto step=n.advance(1e-5,{}, {ambient,0});
    std::cout<<"executor completed="<<step.completed<<" dispatches="<<e.calls<<" maximum_dispatched_chunks="<<e.maxCount<<" ducts=6\n";
}
````
## out/audit2/physics/prepare_probes.py

````python
from pathlib import Path
import hashlib, json, re

root = Path(__file__).resolve().parents[3]
dest = root / 'out/audit2/physics'
src = (root / 'src/simulation/src/EngineSimulator.cpp').read_text(encoding='utf-8')
hdr = (root / 'src/simulation/include/enginelab/simulation/EngineSimulator.hpp').read_text(encoding='utf-8')
assert re.search(r'exhaustBoundaryKnotCount\s*=\s*8', hdr)
assert re.search(r'exhaustBoundaryReconstructionDelaySeconds\s*=\s*625\.0e-6', hdr)
start = src.index('void throttleFlowKeepingReversedFuel(')
end = src.index('\n/** Isentropic ideal-gas orifice', start)
(dest / 'throttle_helper.inc').write_text(src[start:end], encoding='utf-8')
start = src.index('                const auto targetTimeSeconds =')
end = src.index('                exhaustRunnerPressureKpa_[index]', start)
block = src[start:end]
(dest / 'ring_helper.inc').write_text(block, encoding='utf-8')
manifest = {'source': 'src/simulation/src/EngineSimulator.cpp',
            'sha256': hashlib.sha256((root / 'src/simulation/src/EngineSimulator.cpp').read_bytes()).hexdigest(),
            'header': 'src/simulation/include/enginelab/simulation/EngineSimulator.hpp',
            'header_sha256': hashlib.sha256((root / 'src/simulation/include/enginelab/simulation/EngineSimulator.hpp').read_bytes()).hexdigest(),
            'extracts': {'throttle_helper.inc': 'complete anonymous throttle helper, verbatim',
                         'ring_helper.inc': 'complete knot selection and blend block, verbatim'}}
(dest / 'probe_provenance.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')
````
## out/audit2/physics/build_probe.ps1

````powershell
$ErrorActionPreference='Stop'
. scripts/vsenv.ps1
$inc=Get-ChildItem src -Directory | ForEach-Object {
    $p=Join-Path $_.FullName include
    if(Test-Path $p){ '/I'+$p }
}
$libs=Get-ChildItem out/build/windows-vs2022/src -Filter '*.lib' -Recurse |
    Where-Object { $_.DirectoryName -match '\\Release$' -and $_.Name -notmatch 'Fonts|App' } |
    ForEach-Object FullName
python out/audit2/physics/prepare_probes.py
if($LASTEXITCODE -ne 0){ throw 'Probe extraction failed' }
& cl.exe /nologo /std:c++20 /EHsc /O2 /MD @inc /Foout/audit2/physics/ /Feout/audit2/physics/component_probe.exe `
    out/audit2/physics/component_probe.cpp src/physics/src/ConservativeGasSystem.cpp `
    src/gas-dynamics/src/ExhaustGasNetwork.cpp src/gas-dynamics/src/FiniteVolumeDuct.cpp `
    src/gas-dynamics/src/ExhaustNetworkLayout.cpp src/physics/src/DuctWallHeatTransferModel.cpp @libs
if($LASTEXITCODE -ne 0){ throw 'Probe build failed' }
& out/audit2/physics/component_probe.exe | Tee-Object out/audit2/physics/component_probe.log
if($LASTEXITCODE -ne 0){ throw 'Probe run failed' }
````
## out/audit2/physics/ecu_probe.cpp

````cpp
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/calibration/EcuCalibration.hpp>
#include <enginelab/calibration/EcuCalibrationKeys.hpp>
#include <iostream>
#include <memory>
#include <variant>

int main() {
    using namespace enginelab;
    auto config = makeDefaultInlineFour();
    config.forcedInduction.enabled = true;
    config.forcedInduction.pressureRatio = 1.7;
    config.forcedInduction.wastegatePressureRatio = 1.5;
    config.exhaustAfterfire.strategy = ExhaustAfterfireStrategy::lateSparkOverrun;
    config.exhaustAfterfire.enabled = true;
    normaliseEngineConfig(config);
    if (const auto error = validateEngineConfig(config)) {
        std::cout << "invalid_fixture=" << *error << '\n';
        return 1;
    }
    auto draft = calibration::makeDefaultEcuCalibration(config);
    auto setting = *draft.find(calibration::keys::overboostCut);
    std::get<calibration::ScalarCalibration>(setting).value = 40.0;
    draft.set(setting);
    auto store = std::make_shared<calibration::CalibrationStore>();
    const auto published = store->publish(draft);
    if (!published.published) return 2;
    SimpleEcuModel ecu(store);
    ecu.initialise(config);
    auto state = std::make_unique<EngineState>();
    state->rpm = 4'000.0;
    state->simulationTimeSeconds = 20.0;
    state->manifoldPressureKpa = 300.0;
    state->coolantTemperatureC = 90.0;
    state->throttle = 0.0;
    EngineControls controls;
    controls.ignitionEnabled = true;
    const auto afterfire = ecu.evaluate(config, *state, controls);
    config.exhaustAfterfire.strategy = ExhaustAfterfireStrategy::cleanDfco;
    config.exhaustAfterfire.enabled = false;
    ecu.reset();
    const auto clean = ecu.evaluate(config, *state, controls);
    std::cout << "overboost_active=" << hasActiveLimit(afterfire.activeLimits, ActiveLimit::overboostCut)
              << " afterfire_active=" << afterfire.overrunAfterfireActive
              << " afterfire_fuel=" << afterfire.fuelEnabled
              << " clean_fuel=" << clean.fuelEnabled << '\n';
    return 0;
}
````
## out/audit2/physics/persistence_probe.cpp

````cpp
#include <enginelab/catalog/SavedEngines.hpp>
#include <enginelab/serialization/YamlEngineSerializer.hpp>
#include <enginelab/scripting/EngineScriptCompiler.hpp>
#include <iostream>
#include <string>
#if defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <Windows.h>
#endif

int main(int argc, char**) {
#if defined(_WIN32)
    SetErrorMode(SEM_FAILCRITICALERRORS | SEM_NOGPFAULTERRORBOX);
#endif
    using namespace enginelab;
    if (argc > 1) {
        auto text = YamlEngineSerializer {}.encode(makeDefaultInlineFour());
        const auto plain = decodeEngineFile(text, EngineFileFormat::yaml, "probe.yaml");
        std::cout << "plain_yaml_valid=" << plain.engine.has_value() << std::endl;
        text += "\necu_calibration: &loop [*loop]\n";
        std::cout << "entering_alias_conversion" << std::endl;
        const auto cycle = decodeEngineFile(text, EngineFileFormat::yaml, "probe.yaml");
        std::cout << "alias_returned=" << cycle.engine.has_value() << " error=" << cycle.error << std::endl;
        return 0;
    }
    scripting::EngineScriptCompiler compiler;
    const auto metres = compiler.compileText("set acoustic_observer.listening_distance_m = 4 m\n");
    std::cout << "four_metres_accepted=" << static_cast<bool>(metres);
    if (!metres.diagnostics.empty()) std::cout << " diagnostic=" << metres.diagnostics.front().message;
    std::cout << '\n';
    const auto millimetres = compiler.compileText("set acoustic_observer.listening_distance_m = 100 mm\n");
    std::cout << "hundred_millimetres_accepted=" << static_cast<bool>(millimetres);
    if (millimetres) std::cout << " stored_metres=" << millimetres.config->acousticObserver.listeningDistanceM;
    std::cout << '\n';
    return 0;
}
````
## out/audit2/physics/build_auxiliary.ps1

````powershell
$ErrorActionPreference='Stop'
. scripts/vsenv.ps1
$inc=Get-ChildItem src -Directory | ForEach-Object {
    $p=Join-Path $_.FullName include
    if(Test-Path $p){ '/I'+$p }
}
$libs=Get-ChildItem out/build/windows-vs2022/src -Filter '*.lib' -Recurse |
    Where-Object { $_.DirectoryName -match '\\Release$' -and $_.Name -notmatch 'Fonts|App' } |
    ForEach-Object FullName
$yamlLib=(Resolve-Path out/build/windows-vs2022/_deps/yaml_cpp-build/Release/yaml-cpp.lib).Path
& cl.exe /nologo /std:c++20 /EHsc /O2 /MD @inc /Foout/audit2/physics/ /Feout/audit2/physics/ecu_probe.exe `
    out/audit2/physics/ecu_probe.cpp src/ecu/src/SimpleEcuModel.cpp @libs
if($LASTEXITCODE -ne 0){ throw 'ECU probe build failed' }
& out/audit2/physics/ecu_probe.exe | Tee-Object out/audit2/physics/ecu_probe.log
if($LASTEXITCODE -ne 0){ throw 'ECU probe run failed' }
& cl.exe /nologo /std:c++20 /EHsc /O2 /MD @inc /Foout/audit2/physics/ /Feout/audit2/physics/persistence_probe.exe `
    out/audit2/physics/persistence_probe.cpp @libs $yamlLib
if($LASTEXITCODE -ne 0){ throw 'Persistence probe build failed' }
& out/audit2/physics/persistence_probe.exe | Tee-Object out/audit2/physics/persistence_probe.log
if($LASTEXITCODE -ne 0){ throw 'Persistence probe run failed' }
@'
import subprocess
from pathlib import Path
p=subprocess.run(['out/audit2/physics/persistence_probe.exe','alias'],capture_output=True,text=True,timeout=10)
result=p.stdout+p.stderr+f'child_exit_code={p.returncode} hex={p.returncode & 0xffffffff:08X}\n'
Path('out/audit2/physics/alias_probe.log').write_text(result)
print(result)
'@ | python -
if($LASTEXITCODE -ne 0){ throw 'Alias child controller failed' }
````
## out/audit2/audio/audio_probe.cpp

````cpp
#include <enginelab/audio/ExhaustJetNoise.hpp>
#include <enginelab/audio/ValvePortTermination.hpp>
#include <enginelab/audio/ValveFlowAcousticSource.hpp>
#include <enginelab/audio/RealtimeEngineAudio.hpp>
#include <enginelab/audio/OfflineAudioExporter.hpp>
#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/runtime/EngineRuntime.hpp>
#include <enginelab/runtime/DataLogger.hpp>
#include "../../../src/audio/src/OfflineRenderChain.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <memory>
#include <juce_events/juce_events.h>

int main(int argc, char** argv) {
    if (argc != 3) return 2;
    namespace fs = std::filesystem;
    using namespace enginelab;
    const fs::path root(argv[1]), output(argv[2]);
    fs::create_directories(output);
    std::cout << std::setprecision(12);
    juce::ScopedJuceInitialiser_GUI juceInit;
    {
        constexpr double rate=48000.,coupling=5000.,source=40000.,frequency=.47*coupling,delay=625e-6;
        const auto low=BoundaryReconstructionFilter::compute(coupling,rate);
        const auto high=ValveFlowAcousticSource::compute(coupling,rate,source);
        BoundaryReconstructionFilter::State lowAligned,lowDelayed;
        ValveFlowAcousticSource::State highState;
        double alignedEnergy=0.,delayedEnergy=0.;
        for(int n=0;n<144000;++n){
            const auto x=std::sin(2.*3.14159265358979323846*frequency*n/rate);
            const auto xd=std::sin(2.*3.14159265358979323846*frequency*(n/rate-delay));
            const auto h=ValveFlowAcousticSource::process(high,highState,x,1.,2.).outgoingPressurePa;
            const auto a=BoundaryReconstructionFilter::process(low,lowAligned,x)+h;
            const auto d=BoundaryReconstructionFilter::process(low,lowDelayed,xd)+h;
            if(n>=48000){alignedEnergy+=a*a;delayedEnergy+=d*d;}
        }
        std::cout << "crossover source_hz=" << source << " coupling_hz=" << coupling
                  << " cutoff_hz=" << frequency << " history_seconds=" << 7./coupling
                  << " low_only_delay_seconds=" << delay
                  << " aligned_rms=" << std::sqrt(alignedEnergy/96000.)
                  << " delayed_rms=" << std::sqrt(delayedEnergy/96000.)
                  << " relative_db=" << 10.*std::log10(delayedEnergy/alignedEnergy) << '\n';
    }
    {
        ValvePortTermination::State state;
        const auto open = ValvePortTermination::compute(1e-4, 0., 0., 1.2, 411600., 48000.);
        const auto closed = ValvePortTermination::compute(0., 0., 0., 1.2, 411600., 48000.);
        const auto first = ValvePortTermination::process(open, state, 1000.F);
        double tailEnergy=0.; float last=0.;
        for (int n=0;n<48000;++n) {last=ValvePortTermination::process(closed,state,0.F); tailEnergy+=last*last;}
        std::cout << "valve open_b0=" << open.b0 << " initial_output=" << first
                  << " closed_zero_input_1s_last=" << last << " tail_energy=" << tailEnergy << '\n';
    }
    {
        ExhaustJetNoise jet;
        if (!jet.prepare(48000.)) return 3;
        const double area=.001,rho=1.2,c=343.2,flow=2*c*rho*area;
        jet.configure(flow,area,rho,c,1.204); jet.snapToTarget();
        double sum=0.;
        for (int n=0;n<144000;++n) {const auto x=jet.process(1.F); if(n>=48000) sum+=x*x;}
        const auto rms=std::sqrt(sum/96000.);
        std::cout << "jet configured_mach=2 target_rms=" << jet.targetPressureRmsPa()
                  << " measured_rms=" << rms << " excess_db=" << 20*std::log10(rms/jet.targetPressureRmsPa()) << '\n';
    }
    const auto catalog=loadEngineCatalog(root);
    const auto k20=selectSingleEngineCatalogEntry(catalog.entries,"K20");
    const auto turbo=selectSingleEngineCatalogEntry(catalog.entries,"2JZ");
    if (!k20 || !turbo) return 4;
    {
        auto runtime=std::make_unique<EngineRuntime>(turbo.entry->config);
        auto queue=std::make_unique<FiringEventQueue>();
        auto audio=std::make_unique<RealtimeEngineAudio>(*queue,runtime->audioState(),nullptr,
            &runtime->exhaustGraph(),&runtime->engineConfig(),nullptr);
        audio->prepare(48000.,256);
        EngineState state;
        state.rpm=3000.; state.forcedInductionShaftSpeedRpm=12000.;
        state.compressorPowerKw=10.; state.turbinePowerKw=10.;
        state.airFlowGramsPerSecond=30.;state.exhaustFlowGramsPerSecond=30.;
        state.boostPressureRatio=1.2; state.manifoldPressureKpa=120.;
        publishAudioFrame(runtime->audioState(),state,{true,false,0.,1.});
        juce::AudioBuffer<float> master(2,256),fi(2,256);
        RealtimeAudioStemBuffers stems;stems.forcedInduction=&fi;
        double sum=0.;
        for(int block=0;block<400;++block){audio->renderWithStems(master,0,256,stems);
            if(block>=200)for(int n=0;n<256;++n)sum+=fi.getSample(0,n)*fi.getSample(0,n);}
        std::cout << "paused_fi published_time_scale=" << runtime->audioState().timeScale.load()
                  << " shaft_rpm=" << runtime->audioState().forcedInductionShaftRpm.load()
                  << " late_stem_rms=" << std::sqrt(sum/(200.*256.)) << '\n';
    }
    {
        OfflineAudioExportRequest request;request.engine=k20.entry->config;
        request.outputDirectory=output/"cancelled-export";request.sampleRateHz=48000;
        request.loadAuthoredImpulseResponses=false;request.writeStems=true;
        OfflineAudioStage stage;stage.durationSeconds=.1;stage.ignitionEnabled=false;
        stage.governed=false;stage.throttleStart=stage.throttleEnd=0.;
        request.scenario={"cancel",{stage}};
        const auto result=exportOfflineAudio(request,[](double fraction,std::string_view){return fraction==0.;});
        const auto partial=request.outputDirectory/"engine-order-map.csv.partial";
        std::cout << "cancelled_export cancelled=" << result.cancelled << " partial_exists=" << fs::exists(partial) << '\n';
        const auto retry=exportOfflineAudio(request);
        std::cout << "cancelled_export_retry success=" << retry.success << " error=" << retry.error << '\n';
    }
    {
        const auto assets=output/fs::path(u8"ir-é");fs::create_directories(assets);
        fs::copy_file(root/"assets/ir/exhaust_default.wav",assets/"kernel.wav");
        auto config=k20.entry->config;config.exhaustPaths.front().impulseResponsePath="kernel.wav";
        auto chain=std::make_unique<offline::RenderChain>(config,OfflineAudioMix{});
        std::vector<std::string> warnings;std::size_t authored=0,loaded=0;
        chain->loadAuthoredImpulseResponses(config,assets,warnings,authored,loaded);
        std::cout << "unicode_ir authored=" << authored << " loaded=" << loaded;
        for(const auto& warning:warnings)std::cout << " warning=" << warning;
        std::cout << '\n';
    }
    {
        const auto file=output/"rewound-datalog.csv";
        DataLogger logger; if(!logger.start(file))return 5;
        EngineState state;
        for(const auto t:{100.,101.,0.,.02,.04}){state.simulationTimeSeconds=t;logger.push(state);}
        logger.stop();
        std::ifstream input(file);std::string line;std::getline(input,line);
        std::cout << "datalog_rewind_times=";
        while(std::getline(input,line))std::cout << line.substr(0,line.find(',')) << ',';
        std::cout << '\n';
    }
}
````
## out/audit2/audio/build-audio-probe.ps1

````powershell
$ErrorActionPreference='Stop'
. "$PSScriptRoot/../../../scripts/vsenv.ps1"
$auditRoot=(Resolve-Path "$PSScriptRoot/../../..").Path
[xml]$project=Get-Content "$auditRoot/out/build/windows-vs2022/tests/EngineLabRealtimeRegressionTests.vcxproj"
$group=@($project.Project.ItemDefinitionGroup)|Where-Object {$_.Condition -like '*Release|x64*'}|Select-Object -First 1
$includes=$group.ClCompile.AdditionalIncludeDirectories.Split(';')|Where-Object {$_ -notlike '%*'}
$includes+="$auditRoot/out/build/windows-vs2022/_deps/juce-src/modules"
$compile=@('/nologo','/std:c++20','/EHsc','/MD','/O2','/utf-8','/DNDEBUG','/DJUCE_GLOBAL_MODULE_SETTINGS_INCLUDED=1','/DJUCE_WEB_BROWSER=0','/DJUCE_USE_CURL=0','/DYAML_CPP_STATIC_DEFINE')
foreach($dir in $includes){$compile+="/I$dir"}
$compile+=@('/c',"$PSScriptRoot/audio_probe.cpp","/Fo$PSScriptRoot/audio_probe.obj")
& cl.exe @compile
if($LASTEXITCODE){exit $LASTEXITCODE}
$deps=$group.Link.AdditionalDependencies.Split(';')|Where-Object {$_ -and $_ -notlike '%*'}
$link=@('/NOLOGO','/SUBSYSTEM:CONSOLE','/STACK:8388608',"/OUT:$PSScriptRoot/audio_probe.exe","$PSScriptRoot/audio_probe.obj")
foreach($dep in $deps){if($dep.StartsWith('..')){$link+=[IO.Path]::GetFullPath("$auditRoot/out/build/windows-vs2022/tests/$dep")}else{$link+=$dep}}
& link.exe @link
exit $LASTEXITCODE
````
## out/audit2/audio/crossover-delay-math.py

````python
"""Frequency response of the exact current LR biquad coefficient formulas.

No engine/render execution. Compare identical complementary source inputs,
with and without the production low-band-only 625 us reconstruction delay.
Upper reconstruction filter is kept in both controls.
"""
import cmath
import json
import math
from pathlib import Path

sample_rate = 48000.0
source_rate = 40000.0
coupling_rate = 5000.0
frequency = 0.47 * coupling_rate
delay = 625e-6
quality_factors = [0.541196100146197, 1.306562964876377]

def lr_response(cutoff, high=False):
    w0 = 2 * math.pi * cutoff / sample_rate
    c, s = math.cos(w0), math.sin(w0)
    z = cmath.exp(-2j * math.pi * frequency / sample_rate)
    h = 1 + 0j
    for q in quality_factors:
        a0 = 1 + s / (2 * q)
        if high:
            b0, b1 = 0.5 * (1 + c) / a0, -(1 + c) / a0
        else:
            b0, b1 = 0.5 * (1 - c) / a0, (1 - c) / a0
        b2 = b0
        a1, a2 = -2 * c / a0, (1 - s / (2 * q)) / a0
        h *= (b0 + b1 * z + b2 * z * z) / (1 + a1 * z + a2 * z * z)
    return h * h

low = lr_response(frequency)
high = lr_response(frequency, high=True)
upper = lr_response(min(0.42 * source_rate, 0.45 * sample_rate))
aligned = low + high * upper
delayed = low * cmath.exp(-2j * math.pi * frequency * delay) + high * upper
result = {
    'kind': 'exact_coefficient_frequency_math_not_engine_measurement',
    'sample_rate_hz': sample_rate,
    'mechanical_source_rate_hz': source_rate,
    'coupling_rate_hz': coupling_rate,
    'crossover_hz': frequency,
    'history_span_seconds_for_eight_knots': 7 / coupling_rate,
    'required_delay_seconds': delay,
    'history_is_adequate': 7 / coupling_rate > delay,
    'low_magnitude_at_crossover': abs(low),
    'high_magnitude_at_crossover': abs(high),
    'ideal_lr_pair_sum_magnitude': abs(low + high),
    'aligned_sum_with_same_upper_filter_magnitude': abs(aligned),
    'production_delay_sum_with_same_upper_filter_magnitude': abs(delayed),
    'relative_db_from_low_branch_delay_alone': 20 * math.log10(abs(delayed / aligned)),
}
Path(__file__).with_name('crossover-delay-math.json').write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
print(json.dumps(result))
````
## out/audit2/audio/tool-text-evidence.py

````python
from pathlib import Path
import json
import math
import re

root=Path(__file__).resolve().parents[3]
text=(root/'tools/DriveScenarioHarness.cpp').read_text()
header_source=text[text.index('csv << "time_s'):text.index('Pedal throttle')]
header=''.join(re.findall(r'"([^"]*)"',header_source)).replace('\\n','')
format_source=text[text.index('std::snprintf(row'):text.index('now, state.rpm, speed * 3.6')]
row_format=''.join(re.findall(r'"([^"]*)"',format_source)).replace('\\n','')
values=[0.,100.]*10
mean=sum(values)/len(values)
cov=math.sqrt(sum((x-mean)**2 for x in values)/(len(values)-1))/mean
evidence={
 'TL2-01': {'input_work_j':values,'retained_work_j':[v for v in values if v>0.], 'reported_cov_from_retained':0., 'complete_sample_cov':cov},
 'TL2-02': {'header_fields':len(header.split(',')), 'row_format_fields':len(row_format.split(',')), 'trailing_unnamed_fields':['lambda','airFuelRatioValid','manifoldPressureKpa','wastegateOpening','injectorDutyCycle']},
 'TL2-04': {'shipped_bank_route_mm':sum([690,620,140,1070,400,80]),'control_bank_route_mm':sum([690,620,400,80]),'deleted_pipe_length_per_bank_mm':140+1070}
}
(root/'out/audit2/audio/tool-text-evidence.json').write_text(json.dumps(evidence,indent=2)+'\n')
print(json.dumps(evidence,indent=2))
````
## out/audit2/app/model_probe.cpp

````cpp
#include <enginelab/foundation/EngineTypes.hpp>
#include <enginelab/foundation/TurboSizing.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/exhaust/LegacyExhaustNetwork.hpp>
#include <enginelab/exhaust/ExhaustPathTopologyEditor.hpp>
#include <enginelab/render/CrankClock.hpp>
#include <enginelab/render/EngineModel3D.hpp>
#include <enginelab/render/GasFieldView.hpp>
#include <iostream>
#include <algorithm>
#include <iomanip>

using namespace enginelab;
using namespace enginelab::render;
void validity(const char* label, const EngineConfig& c) {
    std::cout << label << " validity=" << validateEngineConfig(c).value_or("valid") << '\n';
}
void routes(const char* label, const EngineConfig& c) {
    const auto graph = ExhaustGraph::makeForEngine(c);
    std::cout << label << " nodes=" << graph.nodes().size() << " diagnostics=" << graph.diagnostics().size() << '\n';
    for (const auto& r : graph.routes())
        std::cout << " route cylinder=" << r.cylinderId << " length_mm=" << r.lengthMm << " delay_s=" << r.delaySeconds << '\n';
}
int main() {
    std::cout << std::setprecision(12);
    auto scalar = makeDefaultInlineFour();
    scalar.exhaust.primaryLengthMm = 400;
    scalar.exhaust.midpipeLengthMm = 200;
    scalar.exhaust.collectorVolumeLitres = 1.0;
    scalar.exhaust.mufflerChamberDiameterMm = 0;
    scalar.exhaust.mufflerChamberLengthMm = 0;
    normaliseEngineConfig(scalar);
    for (auto& p : scalar.exhaustPaths) { p.network.reset(); p.geometry = scalar.exhaust; }
    scalar.cylinders.front().exhaustPrimaryLengthMm = 777;
    validity("scalar", scalar);
    auto editable = scalar;
    for (auto& p : editable.exhaustPaths) p.network = makeEditableExhaustNetwork(p);
    validity("editable", editable);
    routes("scalar", scalar); routes("editable", editable);

    const auto graph = ExhaustGraph::makeForEngine(scalar);
    const EngineModel3D scene(scalar);
    GasFieldSnapshot field;
    for (const auto& n : graph.nodes()) {
        if (n.type == ExhaustNodeType::port) continue;
        GasFieldElement e;
        e.kind = n.type == ExhaustNodeType::merge || n.type == ExhaustNodeType::crossover
            ? GasFieldElementKind::exhaustJunction : GasFieldElementKind::exhaustDuct;
        e.pathIndex = n.pathIndex; e.nodeId = n.id; e.componentId = n.sourceComponentId;
        e.nodeType = static_cast<std::uint8_t>(n.type); e.sampleCount = 1;
        e.pressurePa[0] = static_cast<float>(100000 + n.lengthMm);
        field.elements.push_back(e);
    }
    const auto binding = bindGasField(scene, field);
    for (std::size_t d = 0; d < scene.ducts().size(); ++d) {
        const auto& duct = scene.ducts()[d];
        if (duct.kind != DuctKind::exhaustComponent || duct.componentType != ExhaustComponentType::pipe) continue;
        std::cout << "binding component=" << duct.elementId << " authored_mm=" << duct.authoredLengthMm;
        if (binding[d] >= 0) std::cout << " node=" << field.elements[binding[d]].nodeId << " p=" << field.elements[binding[d]].pressurePa[0];
        std::cout << '\n';
    }

    auto multi = withTurbocharger(makeDefaultInlineFour());
    auto templatePath = multi.exhaustPaths.front();
    multi.exhaustPaths.clear();
    for (std::uint32_t p = 0; p < 3; ++p) {
        auto path = templatePath; path.id = p + 1; path.network.reset(); path.inheritsGlobalGeometry = false;
        path.cylinderIds.clear(); path.turbocharger = -1;
        if (p < 2) path.cylinderIds.push_back(multi.cylinders[p].id);
        else for (std::size_t c = 2; c < multi.cylinders.size(); ++c) path.cylinderIds.push_back(multi.cylinders[c].id);
        multi.exhaustPaths.push_back(path);
    }
    multi.forcedInduction.turbochargerCount = 2;
    normaliseEngineConfig(multi);
    validity("multi_before", multi);
    const auto mapping = [](const EngineConfig& c) {
        for (std::size_t p = 0; p < c.exhaustPaths.size(); ++p)
            std::cout << " turbo_path=" << c.exhaustPaths[p].id << " unit="
                << (c.exhaustPaths[p].turbocharger >= 0 ? c.exhaustPaths[p].turbocharger : p % c.forcedInduction.turbochargerCount);
        std::cout << '\n';
    };
    mapping(multi);
    auto removed = ExhaustPathTopologyEditor::removePath(multi, 1, 3);
    std::cout << "remove accepted=" << bool(removed) << " error=" << removed.error << " note=" << removed.note << '\n';
    validity("multi_after", multi); mapping(multi);

    auto gears = makeDefaultInlineFour();
    gears.transmission.gearRatios = { 1.0, 0.1 };
    validity("gears", gears);
    const EngineModel3D gearbox(gears);
    for (std::size_t g = 0; g < gearbox.gearbox().inputTeeth.size(); ++g)
        std::cout << "gear=" << g << " requested=" << gears.transmission.gearRatios[g]
            << " rendered=" << double(gearbox.gearbox().outputTeeth[g]) / gearbox.gearbox().inputTeeth[g] << '\n';

    for (const auto rate : { 0.0, 0.25, 1.0 }) {
        CrankClock clock;
        clock.observe(0, 0, 3000, rate); clock.setPlaybackFactor(1.0 / 250.0);
        (void)clock.advance(0); (void)clock.advance(0.1);
        std::cout << "strobe rate=" << rate << " crank_deg_s=" << clock.displayDegreesPerSecond()
            << " viewport_shaft_formula_deg_s=" << 6.0 * 3000 / 250.0 << '\n';
    }
}
````
## out/audit2/app/build-model-probe.ps1

````powershell
$ErrorActionPreference = 'Stop'
. "$PSScriptRoot/../../../scripts/vsenv.ps1"
$auditRepo = (Resolve-Path "$PSScriptRoot/../../..").Path
[xml]$project = Get-Content "$auditRepo/out/build/windows-vs2022/tests/EngineLabEngineModel3DTests.vcxproj"
$group = @($project.Project.ItemDefinitionGroup) | Where-Object { $_.Condition -like '*Release|x64*' } | Select-Object -First 1
$includes = $group.ClCompile.AdditionalIncludeDirectories.Split(';') | Where-Object { $_ -notlike '%*' }
$compileArgs = @('/nologo','/std:c++20','/EHsc','/MD','/O2','/utf-8','/DNDEBUG','/DYAML_CPP_STATIC_DEFINE')
foreach ($dir in $includes) { $compileArgs += "/I$dir" }
$compileArgs += @('/c',"$PSScriptRoot/model_probe.cpp","/Fo$PSScriptRoot/model_probe.obj")
& cl.exe @compileArgs
if ($LASTEXITCODE) { exit $LASTEXITCODE }
$linkArgs = @('/NOLOGO','/SUBSYSTEM:CONSOLE','/STACK:8388608',"/OUT:$PSScriptRoot/model_probe.exe","$PSScriptRoot/model_probe.obj")
foreach ($dep in $group.Link.AdditionalDependencies.Split(';')) {
    if (!$dep -or $dep -like '%*') { continue }
    if ($dep.StartsWith('..')) { $linkArgs += [System.IO.Path]::GetFullPath("$auditRepo/out/build/windows-vs2022/tests/$dep") }
    else { $linkArgs += $dep }
}
& link.exe @linkArgs
exit $LASTEXITCODE
````
## out/audit2/app/component_probe.cpp

````cpp
#include <enginelab/app/AudioWorkshopWindow.hpp>
#include <enginelab/app/ExhaustDesignerWindow.hpp>
#include <enginelab/app/PassByWindow.hpp>
#include <enginelab/app/SidePanel.hpp>
#include <enginelab/app/TopBar.hpp>
#include <enginelab/app/ReadoutStrip.hpp>
#include <enginelab/app/Theme.hpp>
#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/gasdynamics/ExhaustNetworkLayout.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>
#include <iostream>
#include <vector>
#include <filesystem>

template <class T> void descendants(juce::Component& c, std::vector<T*>& found) {
    for (auto* child : c.getChildren()) {
        if (auto* item = dynamic_cast<T*>(child)) found.push_back(item);
        descendants<T>(*child, found);
    }
}
juce::Button* button(juce::Component& c, const juce::String& label) {
    std::vector<juce::Button*> all; descendants(c, all);
    for (auto* item : all) if (item->getButtonText() == label) return item;
    return nullptr;
}
void click(juce::Component& c, const juce::String& label) {
    auto* b = button(c, label);
    if (!b || !b->onClick) throw std::runtime_error("missing button " + label.toStdString());
    b->onClick();
}
void snapshot(juce::Component& c, const juce::File& output) {
    auto image = c.createComponentSnapshot(c.getLocalBounds(), true);
    auto stream = output.createOutputStream();
    juce::PNGImageFormat png;
    if (!stream || !png.writeImageToStream(image, *stream)) throw std::runtime_error("snapshot failed");
}
double minimumCfl(const enginelab::EngineConfig& config) {
    auto mesh = enginelab::gasdynamics::realtimeExhaustFeedbackDiscretisation();
    mesh.targetCellLengthM = enginelab::EngineSimulator::productionExhaustCellLengthM(config);
    return enginelab::gasdynamics::ExhaustNetworkLayout::compile(
        enginelab::ExhaustGraph::makeForEngine(config), mesh).minimumCflLengthM();
}
int main(int argc, char** argv) {
    if (argc != 3) return 2;
    juce::ScopedJuceInitialiser_GUI init;
    enginelab::ui::LookAndFeel theme;
    juce::LookAndFeel::setDefaultLookAndFeel(&theme);
    const auto root = std::filesystem::path(argv[1]);
    const juce::File output(juce::String::fromUTF8(argv[2]));
    output.createDirectory();
    const auto catalog = enginelab::loadEngineCatalog(root);
    const auto k20 = enginelab::selectSingleEngineCatalogEntry(catalog.entries, "K20");
    if (!k20) return 3;
    auto a = k20.entry->config;
    a.name = "Audit engine A";
    a.combustionCalibration.cycleVariationCoefficientOfVariation = 0.11;
    auto b = a;
    b.name = "Audit engine B";
    b.combustionCalibration.cycleVariationCoefficientOfVariation = 0.03;
    enginelab::AudioPhysicsSettings captured;
    {
        enginelab::AudioWorkshopWindow workshop(a, root, {}, true, true, false, {},
            [&](const auto& settings) { captured = settings; return true; });
        workshop.setEngine(b, root, true, true, false);
        click(*workshop.getContentComponent(), "APPLY LIVE");
        std::cout << "workshop_after_switch expected_cov="
                  << b.combustionCalibration.cycleVariationCoefficientOfVariation
                  << " applied_cov=" << captured.cycleVariationCoefficientOfVariation << '\n';
    }
    {
        std::unique_ptr<enginelab::ExhaustDesignerWindow> designer;
        double before = minimumCfl(a), after = 0.0;
        designer = std::make_unique<enginelab::ExhaustDesignerWindow>(a,
            [&](const auto& applied) { after = minimumCfl(applied); designer->setConfig(applied); return true; });
        auto& content = *designer->getContentComponent();
        click(content, "GENERATE FROM LEGACY");
        std::vector<juce::TextEditor*> editors; descendants(content, editors);
        std::cout << "exhaust_property_editors=" << editors.size() << '\n';
        if (editors.size() != 12) return 4;
        editors[1]->setText("10", false);
        click(content, "VALIDATE AND APPLY");
        std::cout << "exhaust_cost before_cfl=" << before << " after_cfl=" << after
                  << " true_ratio=" << (after > 0 ? before / after : 0) << '\n';
        std::vector<juce::Label*> labels; descendants(content, labels);
        for (auto* label : labels)
            if (label->getText().startsWith("INFO") || label->getText().startsWith("ERROR"))
                std::cout << "exhaust_status " << label->getText() << '\n';
    }
    enginelab::DashboardModel model(a);
    model.redlineRpm = a.redlineRpm;
    model.state.rpm = 3250;
    model.state.cycleAveragedTorqueNm = 180;
    model.state.cycleAveragedPowerKw = 62;
    model.state.manifoldPressureKpa = 98;
    model.state.exhaustTemperatureC = 722;
    model.state.coolantTemperatureC = 90;
    model.state.oilPressureKpa = 400;
    model.state.airFuelRatio = 14.2;
    model.state.airFuelRatioValid = true;
    model.state.targetAirFuelRatio = 14.4;
    {
        enginelab::ui::TelemetryPanel telemetry(model);
        telemetry.setSize(380, telemetry.preferredHeight());
        std::cout << "telemetry_height=" << telemetry.getHeight() << " vtec=" << enginelab::vtecCamsFitted(a) << '\n';
        std::vector<juce::Button*> buttons; descendants(telemetry, buttons);
        for (auto* item : buttons)
            std::cout << "telemetry_button " << item->getButtonText() << " bounds=" << item->getBounds().toString() << '\n';
        snapshot(telemetry, output.getChildFile("telemetry-vtec.png"));
    }
    {
        enginelab::ui::TopBar top;
        top.setEngine(a, 0);
        top.refresh(model);
        for (const auto width : {1280, 1440}) {
            top.setSize(width, 56);
            std::vector<juce::Button*> buttons; descendants(top, buttons);
            std::cout << "topbar_width=" << width << '\n';
            for (auto* item : buttons)
                std::cout << " button " << item->getName() << " text=" << item->getButtonText()
                          << " bounds=" << item->getBounds().toString() << '\n';
            snapshot(top, output.getChildFile("topbar-" + juce::String(width) + ".png"));
        }
        enginelab::ui::ReadoutStrip readout(model);
        readout.setSize(652, 92);
        snapshot(readout, output.getChildFile("readouts-minimum.png"));
    }
    {
        enginelab::PassByWindow passby(a, {});
        auto& content = *passby.content();
        std::cout << "passby_default_window=" << passby.getBounds().toString()
                  << " content=" << content.getBounds().toString() << '\n';
        snapshot(content, output.getChildFile("passby-window-default.png"));
        passby.setSize(760, 560);
        std::cout << "passby_minimum_window=" << passby.getBounds().toString()
                  << " content=" << content.getBounds().toString() << '\n';
        std::vector<juce::Label*> minimumLabels; descendants(content, minimumLabels);
        for (auto* label : minimumLabels)
            if (label->getText() == "Level" || label->getText() == "Ground")
                std::cout << " minimum_label " << label->getText() << " bounds=" << label->getBounds().toString() << '\n';
        snapshot(content, output.getChildFile("passby-window-minimum.png"));
        for (const auto height : {560, 600}) {
            content.setSize(760, height);
            std::vector<juce::Label*> labels; descendants(content, labels);
            std::cout << "passby_height=" << height << '\n';
            for (auto* label : labels)
                if (label->getText() == "Level" || label->getText() == "Ground")
                    std::cout << " label " << label->getText() << " bounds=" << label->getBounds().toString() << '\n';
            snapshot(content, output.getChildFile("passby-" + juce::String(height) + ".png"));
        }
    }
    juce::LookAndFeel::setDefaultLookAndFeel(nullptr);
}
````
## out/audit2/app/build-component-probe.ps1

````powershell
$ErrorActionPreference = 'Stop'
. "$PSScriptRoot/../../../scripts/vsenv.ps1"
$auditRepo = (Resolve-Path "$PSScriptRoot/../../..").Path
$projectBase = "$auditRepo/out/build/windows-vs2022/src/app"
[xml]$project = Get-Content "$projectBase/EngineLabApp.vcxproj"
$group = @($project.Project.ItemDefinitionGroup) | Where-Object { $_.Condition -like '*Release|x64*' } | Select-Object -First 1
$includes = $group.ClCompile.AdditionalIncludeDirectories.Split(';') | Where-Object { $_ -notlike '%*' }
$includes += "$auditRepo/src/app/include"
$includes += "$auditRepo/out/build/windows-vs2022/_deps/juce-src/modules"
$compileArgs = @('/nologo','/std:c++20','/EHsc','/MD','/O2','/utf-8','/DNDEBUG','/DJUCE_GLOBAL_MODULE_SETTINGS_INCLUDED=1','/DJUCE_WEB_BROWSER=0','/DJUCE_USE_CURL=0','/DYAML_CPP_STATIC_DEFINE')
foreach ($dir in $includes) { $compileArgs += "/I$dir" }
$compileArgs += @('/c',"$PSScriptRoot/component_probe.cpp","/Fo$PSScriptRoot/component_probe.obj")
& cl.exe @compileArgs
if ($LASTEXITCODE) { exit $LASTEXITCODE }
$linkArgs = @('/NOLOGO','/LTCG','/SUBSYSTEM:CONSOLE','/STACK:8388608',"/OUT:$PSScriptRoot/component_probe.exe","$PSScriptRoot/component_probe.obj")
foreach ($name in @('AudioWorkshopWindow','ExhaustDesignerWindow','PassByWindow','SidePanel','DynoPanel','DashboardModel','Theme','Widgets','UnitFormat','TopBar','ReadoutStrip','ActionMap')) {
    $linkArgs += "$auditRepo/out/build/windows-vs2022/src/app/EngineLabApp.dir/Release/$name.obj"
}
$linkArgs += "$auditRepo/out/build/windows-vs2022/src/app/Release/EngineLabFonts.lib"
$deps = $group.Link.AdditionalDependencies.Split(';') | Where-Object { $_ -and $_ -notlike '%*' }
foreach ($dep in $deps) {
    if ($dep -eq '-LTCG') { continue }
    if ($dep.EndsWith('.lib') -and $dep.Contains('\')) { $linkArgs += [System.IO.Path]::GetFullPath("$projectBase/$dep") }
    else { $linkArgs += $dep }
}
& link.exe @linkArgs
exit $LASTEXITCODE
````
## out/audit2/app/part_probe.cpp

````cpp
#include <enginelab/app/EngineViewport.hpp>
#include <enginelab/app/Theme.hpp>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <stdexcept>

extern "C" [[noreturn]] void auditVectorDoom(const char* message) noexcept {
    std::fprintf(stderr, "CHECKED_VECTOR_FAILURE: %s\n", message);
    std::fflush(nullptr);
    std::_Exit(86);
}

juce::Button* findButton(juce::Component& component, const juce::String& text,
                         const juce::String& tooltipPrefix) {
    for (auto* child : component.getChildren()) {
        if (auto* item = dynamic_cast<juce::Button*>(child))
            if (item->getButtonText() == text && item->getTooltip().startsWith(tooltipPrefix))
                return item;
    }
    throw std::runtime_error("missing button");
}

int main(int argc, char** argv) {
    if (argc < 2) return 2;
    juce::ScopedJuceInitialiser_GUI init;
    enginelab::ui::LookAndFeel theme;
    juce::LookAndFeel::setDefaultLookAndFeel(&theme);
    enginelab::ui::PartInspector inspector;
    enginelab::ui::PartInspector::Edit edit;
    edit.key = 1;
    edit.heading = "GEARBOX";
    edit.value("Shift time", 100, "ms", 0, 10, 1, 0, 1000);
    edit.value("Torque cut", 0.5, "", 2, 0.1, 0.01, 0, 1);
    edit.value("Final drive", 3, "", 2, 0.1, 0.01, 0.05, 10);
    edit.value("Clutch", 500, "Nm", 0, 10, 1, 1, 10000);
    edit.value("Forward gears", 3, "", 0, 1, 1, 1, 20);
    edit.value("First ratio", 3, "", 2, 0.1, 0.01, 0.05, 10);
    edit.value("Second ratio", 2, "", 2, 0.1, 0.01, 0.05, 10);
    edit.value("Third ratio", 1, "", 2, 0.1, 0.01, 0.05, 10);
    inspector.show("Gearbox", "Standalone actual component", {});
    inspector.setSize(280, 400);
    inspector.setEdit(edit);
    inspector.setSize(280, inspector.idealHeight());
    if (juce::String(argv[1]) == "layout") {
        edit.fields.resize(2);
        edit.choice = enginelab::ui::PartInspector::Edit::Choice{{"Port", "Direct"}, "First choice", 0};
        edit.secondChoice = enginelab::ui::PartInspector::Edit::Choice{{"Filtered", "Open"}, "Second choice", 0};
        edit.note = "This authored note starts on the same reserved row as the Reset and first choice controls.";
        inspector.setEdit(edit);
        inspector.setSize(280, inspector.idealHeight());
        for (auto* child : inspector.getChildren())
            if (auto* item = dynamic_cast<juce::Button*>(child))
                if (item->isVisible())
                    std::cout << "button=" << item->getButtonText() << " bounds="
                              << item->getBounds().toString() << '\n';
        if (argc != 3) return 3;
        const juce::File path(juce::String::fromUTF8(argv[2]));
        auto stream = path.createOutputStream();
        juce::PNGImageFormat png;
        if (!stream || !png.writeImageToStream(inspector.createComponentSnapshot(inspector.getLocalBounds()), *stream))
            return 4;
        juce::LookAndFeel::setDefaultLookAndFeel(nullptr);
        return 0;
    }
    auto* reduceCount = findButton(inspector, "-", "Forward gears");
    auto* lastUp = findButton(inspector, "+", "Third ratio");
    const auto position = juce::Point<float>(static_cast<float>(lastUp->getX() - 39),
                                            static_cast<float>(lastUp->getBounds().getCentreY()));
    int callbacks = 0;
    inspector.onApplyEdit = [&](const std::vector<double>& values, bool) {
        ++callbacks;
        std::cout << "callback=" << callbacks << " value_count=" << values.size()
                  << " requested_forward_gears=" << values[4] << std::endl;
        edit.fields[4].value = values[4];
        edit.fields.resize(7);
        inspector.setEdit(edit);
        std::cout << "callback_replaced_fields=7; returning to original last-field click" << std::endl;
    };
    reduceCount->onClick();
    std::cout << "after_stepper callbacks=" << callbacks << " old_last_value_click="
              << position.toString() << std::endl;
    const auto time = juce::Time::getCurrentTime();
    const juce::MouseEvent event(juce::Desktop::getInstance().getMainMouseSource(), position,
        juce::ModifierKeys(juce::ModifierKeys::leftButtonModifier), 1.0F, 0, 0, 0, 0,
        &inspector, &inspector, time, position, time, 1, false);
    inspector.mouseUp(event);
    std::cout << "unexpected_return_after_reentry callbacks=" << callbacks << std::endl;
    juce::LookAndFeel::setDefaultLookAndFeel(nullptr);
    return 5;
}
````
## out/audit2/app/checked_vector.hpp

````cpp
// Forced include for the standalone probe only. The tracked source is unchanged.
// This hook uses MSVC's supported STL hardening error-report customization.
extern "C" [[noreturn]] void auditVectorDoom(const char*) noexcept;
#define _MSVC_STL_HARDENING_VECTOR 1
#define _MSVC_STL_DOOM_FUNCTION(message) auditVectorDoom(message)
````
## out/audit2/app/build-part-probe.ps1

````powershell
$ErrorActionPreference = 'Stop'
. "$PSScriptRoot/../../../scripts/vsenv.ps1"
$auditRepo = (Resolve-Path "$PSScriptRoot/../../..").Path
$projectBase = "$auditRepo/out/build/windows-vs2022/src/app"
[xml]$project = Get-Content -LiteralPath "$projectBase/EngineLabApp.vcxproj"
$group = @($project.Project.ItemDefinitionGroup) | Where-Object { $_.Condition -like '*Release|x64*' } | Select-Object -First 1
$compileArgs = @('/nologo','/std:c++20','/EHsc','/MD','/O2','/Zi','/utf-8','/DNDEBUG','/DJUCE_GLOBAL_MODULE_SETTINGS_INCLUDED=1','/DJUCE_WEB_BROWSER=0','/DJUCE_USE_CURL=0','/DYAML_CPP_STATIC_DEFINE',"/FI$PSScriptRoot/checked_vector.hpp","/Fd$PSScriptRoot/part-probe.pdb")
foreach ($dir in $group.ClCompile.AdditionalIncludeDirectories.Split(';')) {
    if ($dir -and $dir -notlike '%*') { $compileArgs += "/I$dir" }
}
$compileArgs += "/I$auditRepo/out/build/windows-vs2022/_deps/juce-src/modules"
& cl.exe @compileArgs /c "$auditRepo/src/app/src/EngineViewport.cpp" "/Fo$PSScriptRoot/EngineViewport-checked.obj"
if ($LASTEXITCODE) { exit $LASTEXITCODE }
& cl.exe @compileArgs /c "$PSScriptRoot/part_probe.cpp" "/Fo$PSScriptRoot/part_probe.obj"
if ($LASTEXITCODE) { exit $LASTEXITCODE }
$linkArgs = @('/NOLOGO','/SUBSYSTEM:CONSOLE','/STACK:8388608','/DEBUG:FULL',"/OUT:$PSScriptRoot/part_probe.exe","/PDB:$PSScriptRoot/part_probe.pdb","$PSScriptRoot/part_probe.obj","$PSScriptRoot/EngineViewport-checked.obj")
foreach ($name in @('EngineScene','Theme','Widgets','UnitFormat','FuelEditing','DashboardModel')) {
    $linkArgs += "$projectBase/EngineLabApp.dir/Release/$name.obj"
}
foreach ($dep in $group.Link.AdditionalDependencies.Split(';')) {
    if (!$dep -or $dep -like '%*' -or $dep -eq '-LTCG') { continue }
    if ($dep.EndsWith('.lib') -and $dep.Contains('\')) { $linkArgs += [System.IO.Path]::GetFullPath("$projectBase/$dep") }
    else { $linkArgs += $dep }
}
& link.exe @linkArgs
exit $LASTEXITCODE
````
## out/audit2/app/oracle_witness.py

````python
"""Tiny mathematical witnesses; does not simulate engines or invoke test binaries.

TEST-01 predicates mirror IdleStabilityRegressionTests.cpp:588-595,630-634.
TEST-02 scalar comparison mirrors SavedEngineTests.cpp:398,421-425.
"""
import json
import math
from pathlib import Path

idle_target = 1000.0
recovery = [900.0] + [2000.0] * 11
minimum = min(recovery)
final_mean = sum(recovery[-2:]) / 2
final_sample = recovery[-1]
allowlisted_name = "RD350 mathematical witness"
slow_coast_allowed = minimum < final_mean and "RD350" in allowlisted_name
dip_failure = minimum < 0.78 * idle_target
return_failure = (final_mean < 0.85 * idle_target
                  or (final_mean > 1.15 * idle_target and not slow_coast_allowed))
documented_coasting = final_sample < final_mean

original = [1.0, -1.0, 0.0, 0.0]
changed = [1.0, 0.0, -1.0, 0.0]
def summary(samples):
    return [math.sqrt(sum(x * x for x in samples) / len(samples)),
            max(abs(x) for x in samples)]

result = {
    "scope": "Mathematical counterexamples only; no simulator/render/test execution",
    "TEST-01": {
        "target": idle_target, "minimum": minimum,
        "last_2s_mean": final_mean, "final_sample": final_sample,
        "slowCoastAllowed": slow_coast_allowed,
        "dipFailure": dip_failure, "returnFailure": return_failure,
        "documentedFinalSampleBelowMean": documented_coasting,
        "raisedIdlePredicateSameCondition": True,
    },
    "TEST-02": {
        "original": original, "changed": changed,
        "originalSummary": summary(original), "changedSummary": summary(changed),
        "samplesEqual": original == changed,
        "acceptedBySummaryEquality": summary(original) == summary(changed),
    },
}
Path(__file__).with_suffix(".json").write_text(
    json.dumps(result, indent=2) + "\n", encoding="utf-8")
print(json.dumps(result, indent=2))
````
## out/audit2/root-probes.py

````python
from pathlib import Path
import contextlib,importlib.util,io,json
import numpy as np
from scipy.io import wavfile
root=Path(__file__).resolve().parents[2]
folder=root/'out/audit2/root-fixtures'; folder.mkdir(parents=True,exist_ok=True)
spec=importlib.util.spec_from_file_location('pop_metrics',root/'scripts/pop_metrics.py')
pop=importlib.util.module_from_spec(spec); spec.loader.exec_module(pop)
wav=folder/'silence.wav'
wavfile.write(wav,48000,np.zeros(4800,dtype=np.int16))
result=pop.analyse(wav,0,None,18,-30)
payload=json.dumps([result],indent=1)
(folder/'silence.json').write_text(payload)
try:
 json.loads(payload,parse_constant=lambda v: (_ for _ in ()).throw(ValueError('nonstandard JSON '+v)))
 standard=True; error='none'
except ValueError as exc: standard=False; error=str(exc)
print(json.dumps({'case':'pop-metrics-silence','pop_count':result['pop_count'],'standard_json':standard,'error':error,'nan_tokens':payload.count('NaN')}))

spec=importlib.util.spec_from_file_location('listening',root/'scripts/listening.py')
listening=importlib.util.module_from_spec(spec); spec.loader.exec_module(listening)
# One real listener answers two conditions belonging to the same run.
pack=folder/'label-modes'; (pack/'responses').mkdir(parents=True,exist_ok=True)
key={'mode':'output-stage','engine':'Engine X','target_lufs':-18,'seed':1,'pairs':[{'pair':1,'engine':'Engine X','subject_side':'B','control_present':True,'segment':'idle','control':{'match_quality':'same-engine','window_condition_matched':True}}]}
(pack/'listening-key.json').write_text(json.dumps(key))
(pack/'responses/listener_01.csv').write_text('pair,realism_A,realism_B,preference_A,preference_B,most_realistic,preferred,comment\n1,1,5,1,5,B,B,\n')
with contextlib.redirect_stdout(io.StringIO()): status=listening.report(pack,False)
text=(pack/'listening-report.md').read_text()
print(json.dumps({'case':'self-mode-label','exit':status,'heading':text.splitlines()[0],'claims_two_catalogue_engines':'This test compares two catalogue engines' in text}))
````
## out/audit2/contrast-probe.py

````python
import json
from pathlib import Path

def luminance(hex_colour):
    rgb = [int(hex_colour[i:i+2], 16) / 255 for i in (0, 2, 4)]
    linear = [v / 12.92 if v <= .04045 else ((v + .055) / 1.055) ** 2.4 for v in rgb]
    return sum(a * b for a, b in zip(linear, (.2126, .7152, .0722)))

rows = []
for name, background in [('background', '0b0f0e'), ('panel', '111716'), ('panelRaised', '161e1c')]:
    foreground = '5c6a65'
    a, b = sorted([luminance(foreground), luminance(background)])
    ratio = (b + .05) / (a + .05)
    rows.append(dict(foreground=foreground, background_name=name, background=background,
                     ratio=ratio, reaches_4_5=ratio >= 4.5))
print(json.dumps(rows, indent=2))
````
## out/audit2/root-cli-probes.py

````python
from pathlib import Path
import json, shutil, subprocess

root = Path(__file__).resolve().parents[2]
scratch = root / 'out/audit2/root-fixtures'
scratch.mkdir(parents=True, exist_ok=True)
tools = root / 'out/build/windows-vs2022/tools/Release'

def run(case, executable, args):
    try:
        result = subprocess.run([str(tools / executable), *map(str, args)], cwd=root,
                                capture_output=True, text=True, errors='replace', timeout=30)
        record = dict(case=case, args=list(map(str, args)), exit=result.returncode,
                      stdout=result.stdout, stderr=result.stderr)
    except subprocess.TimeoutExpired:
        record = dict(case=case, timeout=True)
    (scratch / (case + '.json')).write_text(json.dumps(record, indent=2), encoding='utf-8')
    print(json.dumps(record))

run('invalid-tap-token', 'EngineLabOfflineAudioExporter.exe',
    ['--engine', 'K20A', '--output', scratch / 'tap-output', '--exhaust-tap', 'bad:0.5'])
blocked = scratch / 'blocked-scene'
(blocked / 'index.json').mkdir(parents=True, exist_ok=True)
run('scene-index-write-failure', 'EngineLabSceneExport.exe',
    ['--out', blocked, '--filter', 'no-matching-engine-for-audit'])
run('zero-sample-transfer', 'EngineLabAcousticTransferProbe.exe',
    ['--filter', 'K20A', '--component', '100', '--seconds', '.000001', '--max-hz', '10'])
catalog = scratch / 'control-name-catalog'
(catalog / 'engines').mkdir(parents=True, exist_ok=True)
shutil.copytree(root / 'parts', catalog / 'parts', dirs_exist_ok=True)
engine = (root / 'examples/inline-four.yaml').read_text(encoding='utf-8')
engine = engine.replace('  name: EL-20 Inline Four', '  name: "Audit\\nEngine"', 1)
(catalog / 'engines/one.engine.yaml').write_text(engine, encoding='utf-8')
scene = scratch / 'control-name-scene'
run('scene-control-character', 'EngineLabSceneExport.exe', ['--out', scene, '--catalog-root', catalog])
try:
    json.loads((scene / 'index.json').read_text(encoding='utf-8'))
    summary = dict(case='scene-json-standard', accepted=True)
except (ValueError, OSError) as exc:
    summary = dict(case='scene-json-standard', accepted=False, error=str(exc))
(scratch / 'scene-json-standard.json').write_text(json.dumps(summary), encoding='utf-8')
print(json.dumps(summary))
````
## out/audit2/package-inspection.ps1

````powershell
$ErrorActionPreference = 'Stop'
. "$PSScriptRoot/../../scripts/vsenv.ps1"
$auditRoot = (Resolve-Path "$PSScriptRoot/../..").Path
$auditStage = Join-Path $auditRoot 'out/audit2/package-stage'
cmake --install "$auditRoot/out/build/windows-vs2022" --config Release --component EngineLab --prefix $auditStage 2>&1 |
    Tee-Object "$PSScriptRoot/package-install.log"
if ($LASTEXITCODE) { exit $LASTEXITCODE }
$auditFiles = Get-ChildItem -LiteralPath $auditStage -File -Recurse | ForEach-Object {
    [pscustomobject]@{
        path = [IO.Path]::GetRelativePath($auditStage, $_.FullName)
        bytes = $_.Length
        sha256 = (Get-FileHash -LiteralPath $_.FullName -Algorithm SHA256).Hash.ToLowerInvariant()
    }
}
$auditFiles | ConvertTo-Json -Depth 4 | Set-Content "$PSScriptRoot/package-files.json"
$auditNotices = Get-ChildItem -LiteralPath $auditStage -File -Recurse | Where-Object {
    $_.Extension -in '.txt','.md' -or $_.Name -match 'LICENSE|OFL|NOTICE'
}
$auditNoticeText = ($auditNotices | ForEach-Object { Get-Content -LiteralPath $_.FullName -Raw }) -join "`n"
[pscustomobject]@{
    installedFiles = @($auditFiles).Count
    installedBytes = ($auditFiles | Measure-Object -Property bytes -Sum).Sum
    includesNlohmannCopyright = $auditNoticeText.Contains('2013-2025 Niels Lohmann')
    includesYamlCppCopyright = $auditNoticeText.Contains('2008-2015 Jesse Beder')
    noticeFiles = @($auditNotices | ForEach-Object { [IO.Path]::GetRelativePath($auditStage,$_.FullName) })
} | ConvertTo-Json -Depth 4 | Tee-Object "$PSScriptRoot/package-notices.json"
````
## out/audit2/package-root-probe.py

````python
from pathlib import Path
import json,subprocess
root=Path(__file__).resolve().parents[2]
stage=root/'out/audit2/package-stage'
engine=stage/'engines/01_honda_k20a_like.engine.yaml'
original=engine.read_text(encoding='utf-8')
sentinel='Audit package catalogue sentinel'
assert 'name: K20A-like 2.0 I4 VTEC' in original
engine.write_text(original.replace('name: K20A-like 2.0 I4 VTEC','name: '+sentinel,1),encoding='utf-8')
results=[]
try:
    for mode,args in [('default',[]),('explicit',['--catalog-root',str(stage)])]:
        run=subprocess.run([str(stage/'tools/EngineLabOfflineAudioExporter.exe'),'--list-engines',*args],
                           cwd=stage,capture_output=True,text=True,errors='replace',timeout=20)
        results.append(dict(mode=mode,exit=run.returncode,seesPackagedSentinel=sentinel in run.stdout,
                            seesRepositoryName='K20A-like 2.0 I4 VTEC' in run.stdout,
                            stdout=run.stdout,stderr=run.stderr))
finally:
    engine.write_text(original,encoding='utf-8')
(root/'out/audit2/package-root-probe.json').write_text(json.dumps(results,indent=2),encoding='utf-8')
print(json.dumps(results,indent=2))
````
## out/audit2/scene-summary.py

````python
from pathlib import Path
import json,math
root=Path(__file__).resolve().parents[2]
folder=root/'out/audit2/scenes'
names=['ductClash','selfClash','engineClash','tightBend','stretched']
for entry in json.loads((folder/'index.json').read_text(encoding='utf-8')):
    data=json.loads((folder/entry['file']).read_text(encoding='utf-8'))
    counts={name:sum(i['kind']==k for i in data['issues']) for k,name in enumerate(names)}
    if not any(counts.values()):continue
    print(data['name'],counts)
    for k in (0,1,3,4):
        issues=[i for i in data['issues'] if i['kind']==k]
        if not issues:continue
        issue=(min if k==3 else max)(issues,key=lambda i:i['value'])
        duct=data['ducts'][issue['duct']]
        drawn=sum(math.dist(a,b) for a,b in zip(duct['centreline'],duct['centreline'][1:]))
        print(json.dumps(dict(issue=names[k],duct=issue['duct'],kind=duct['kind'],
          path=duct['path'],element=duct['element'],other=issue['other'],
          value=issue['value'],authored_mm=duct['authored'],drawn_mm=round(drawn,3),where_mm=issue['where'])))
````
## out/audit2/refresh-audio-ab.py

````python
from pathlib import Path
import json,subprocess
root=Path(__file__).resolve().parents[2]
args=[str(root/'out/build/windows-vs2022/tools/Release/EngineLabAudioAbHarness.exe'),
      '--root',str(root),'--output',str(root/'out/audit2/audio-ab-refresh')]
try:
    result=subprocess.run(args,cwd=root,capture_output=True,text=True,errors='replace',timeout=30)
    record=dict(args=args,exit=result.returncode,stdout=result.stdout,stderr=result.stderr)
except subprocess.TimeoutExpired as exc:
    record=dict(args=args,timeout=True,stdout=str(exc.stdout),stderr=str(exc.stderr))
(root/'out/audit2/audio-ab-refresh.json').write_text(json.dumps(record,indent=2),encoding='utf-8')
print(json.dumps(record,indent=2))
````
## out/audit2/baseline-analysis.py

````python
from pathlib import Path
import csv,io,json,re,xml.etree.ElementTree as ET
root=Path(__file__).resolve().parents[2]
folder=root/'out/audit2'
suite=ET.parse(folder/'full-ctest.xml').getroot()
full=(folder/'full-LastTest.log').read_text(encoding='utf-8')
results=[]
for case in suite.findall('testcase'):
    failed=case.find('failure')
    results.append(dict(name=case.attrib['name'],seconds=float(case.attrib['time']),
                        result='FAIL' if failed is not None else 'PASS',
                        failure=failed.attrib if failed is not None else None))
    if case.attrib['name'] in ('EngineLab.CatalogReference','EngineLab.DesignedReference'):
        # JUnit truncates successful output to 1 KiB; LastTest preserves it.
        pattern=(r'^\d+/108 Testing: '+re.escape(case.attrib['name'])+
                 r'\n.*?\nOutput:\n-+\n(.*?)\n<end of output>')
        match=re.search(pattern,full,re.M|re.S)
        assert match,case.attrib['name']
        output=match[1]
        stem=case.attrib['name'].split('.')[-1]
        (folder/(stem+'.csv')).write_text(output,encoding='utf-8')
        if stem=='CatalogReference':print(output)
(folder/'baseline-results.json').write_text(json.dumps(results,indent=2),encoding='utf-8')
print(json.dumps(dict(tests=len(results),failed=[r['name'] for r in results if r['result']=='FAIL'])))
````
