# The interface's reference scenes: the same list runs on the build before the
# "Test cell" interface and on every build after it, so their captures and
# costs can be set side by side.
#
#   scripts/ui-scenes.ps1 -Exe <EngineLab.exe> -OutDir <folder> [-Only <scene,...>] [-Perf]
#
# Without -Perf: the capture scenes (each main-window size, each side tab,
# each tool window, a 125 % scale). With -Perf: the cost scenes only (four
# engines on a dyno run, 20 s measured each); run them interleaved with the
# other build (CLAUDE.md, Instruments).
param(
    [Parameter(Mandatory = $true)][string]$Exe,
    [Parameter(Mandatory = $true)][string]$OutDir,
    [string]$Only = '',
    [switch]$Perf
)
$ErrorActionPreference = 'Stop'
$bench = Join-Path $PSScriptRoot 'ui-bench.ps1'
$short = '--warmup 8 --seconds 3'
$captures = [ordered]@{
    'main-idle-1440'     = "--engine K20 --throttle 0 --width 1440 --height 860 $short"
    'main-dyno-1600'     = "--engine 2JZ --dyno --tab dyno --width 1600 --height 900 --warmup 8 --seconds 14"
    'main-dyno-1280'     = "--engine 2JZ --dyno --tab dyno --width 1280 --height 720 --warmup 8 --seconds 14"
    'main-dyno-1366'     = "--engine 2JZ --dyno --tab dyno --width 1366 --height 768 --warmup 8 --seconds 14"
    'main-dyno-1900'     = "--engine 2JZ --dyno --tab dyno --width 1900 --height 980 --warmup 8 --seconds 14"
    'main-scale125-1280' = "--engine 2JZ --dyno --tab dyno --width 1280 --height 720 --scale 1.25 --warmup 8 --seconds 14"
    'tab-telemetry'      = "--engine 2JZ --throttle 0.2 --tab telemetry --width 1440 --height 860 $short"
    'tab-audio'          = "--engine 2JZ --throttle 0.2 --tab audio --width 1440 --height 860 $short"
    'tab-diagnostics'    = "--engine 2JZ --throttle 0.2 --tab diagnostics --width 1440 --height 860 $short"
    'two-stroke-rd350'   = "--engine RD350 --throttle 0.2 --width 1440 --height 860 $short"
    'rotary-13b'         = "--engine 13B --throttle 0.2 --width 1440 --height 860 $short"
    'window-ecu'         = "--engine 2JZ --throttle 0.2 --open ecu --width 1440 --height 860 $short"
    'window-exhaust'     = "--engine 2JZ --throttle 0 --open exhaust --width 1440 --height 860 $short"
    'window-fuel'        = "--engine 2JZ --throttle 0 --open fuel --width 1440 --height 860 $short"
    'window-equipment'   = "--engine 2JZ --throttle 0 --open equipment --width 1440 --height 860 $short"
    'window-passby'      = "--engine 2JZ --throttle 0 --open passby --width 1440 --height 860 $short"
    'window-vehicle'     = "--engine 2JZ --throttle 0 --open vehicle --width 1440 --height 860 $short"
    'window-audio'       = "--engine 2JZ --throttle 0 --open audio --width 1440 --height 860 $short"
    'window-designer'    = "--engine 2JZ --throttle 0 --open designer --width 1440 --height 860 $short"
}
$costs = [ordered]@{
    'perf-2jz'      = '--engine 2JZ --dyno --tab dyno --width 1440 --height 860 --warmup 8 --seconds 20'
    'perf-hayabusa' = '--engine Hayabusa --dyno --tab dyno --width 1440 --height 860 --warmup 8 --seconds 20'
    'perf-13b'      = '--engine 13B --dyno --tab dyno --width 1440 --height 860 --warmup 8 --seconds 20'
    'perf-merlin'   = '--engine Merlin --dyno --tab dyno --width 1440 --height 860 --warmup 8 --seconds 20'
}
$scenes = if ($Perf) { $costs } else { $captures }
$wanted = if ($Only) { $Only.Split(',') } else { @($scenes.Keys) }
foreach ($name in $scenes.Keys) {
    if ($wanted -notcontains $name) { continue }
    $summary = & $bench -Exe $Exe -Name $name -OutDir $OutDir -BenchArgs $scenes[$name]
    "$name $summary"
}
