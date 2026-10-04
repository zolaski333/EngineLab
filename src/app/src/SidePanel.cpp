#include <enginelab/app/SidePanel.hpp>

#include <enginelab/app/Theme.hpp>
#include <algorithm>
#include <array>
#include <cmath>

namespace enginelab::ui {
namespace {
constexpr int sectionHeight = 24;
constexpr int rowHeight = 24;
constexpr int padding = 14;

using Rows = std::vector<std::pair<juce::String, juce::String>>;

void drawRows(juce::Graphics& g, juce::Rectangle<float>& area, const Rows& rows, float height = rowHeight) {
    for (const auto& [label, value] : rows) {
        auto row = area.removeFromTop(height);
        g.setColour(colours::grid);
        g.fillRect(row.removeFromBottom(1.0F));
        g.setColour(colours::muted);
        g.setFont(uiFont(14.0F));
        const auto labelWidth = std::min(row.getWidth() * 0.55F,
            juce::GlyphArrangement::getStringWidth(g.getCurrentFont(), label) + 8.0F);
        g.drawText(label, row.removeFromLeft(labelWidth), juce::Justification::centredLeft, true);
        g.setColour(colours::text);
        g.setFont(monoFont(13.5F, false));
        g.drawFittedText(value, row.toNearestInt(), juce::Justification::centredRight, 1, 0.75F);
    }
}

[[nodiscard]] Rows engineRows(const DashboardModel& model) {
    const auto& state = model.state;
    double peakPressure = 0.0;
    for (std::size_t index = 0; index < state.cylinderStateCount; ++index)
        peakPressure = std::max(peakPressure, state.cylinderStates[index].pressureEstimateBar);
    const auto dot = utf8("  \xc2\xb7  ");
    const auto degree = utf8("\xc2\xb0");
    return {
        { "State", runningStateName(state.runningState) },
        { "Layout", juce::String(layoutName(model.config.layout)) + dot
              + juce::String(static_cast<int>(model.config.cylinders.size())) + " cyl" },
        { "Volumetric efficiency", juce::String(state.volumetricEfficiency * 100.0, 1) + " %" },
        { "Air / fuel flow", juce::String(state.airFlowGramsPerSecond, 1) + " / "
              + juce::String(state.fuelFlowGramsPerSecond, 2) + " g/s" },
        { "Lambda", juce::String(state.lambda, 3) },
        { "Ignition advance", juce::String(state.ignitionAdvanceDegrees, 1) + degree },
        { "Knock level", juce::String(state.knockLevel, 2) },
        { "Peak cylinder pressure", juce::String(peakPressure, 1) + " bar" },
        { "Exhaust runner", fixed(state.exhaustRunnerPressureKpa, 0) + " kPa" },
        { "Oil", fixed(state.oilPressureKpa, 0) + " kPa" + dot
              + fixed(state.oilTemperatureC, 0) + degree + "C" },
        { "Mean piston speed", juce::String(state.meanPistonSpeedMps, 1) + " m/s" },
        { "Peak piston acceleration", groupedInteger(state.peakPistonAccelerationG) + " g" },
    };
}

[[nodiscard]] Rows drivelineRows(const DashboardModel& model) {
    const auto& state = model.state;
    const auto benchText = state.dynoHoldEnabled
        ? juce::String("hold")
        : (state.dynoRampEnabled
            ? "ramp " + fixed(state.dynoRampRpmPerSecond, 0) + " rpm/s"
            : juce::String("steps 250 rpm"));
    return {
        { "Gear", gearName(state.gear) + " / " + juce::String(state.gearCount) },
        { "Clutch", fixed(state.clutchPressure * 100.0, 0) + " %" },
        { "Vehicle speed", juce::String(state.vehicleSpeedMps * 3.6, 1) + " km/h" },
        { "Distance", juce::String(state.vehicleDistanceM / 1'000.0, 3) + " km" },
        { "Wheel torque", fixed(state.wheelTorqueNm, 0) + " Nm" },
        { "Driveline drag", fixed(state.drivelineLoadTorqueNm, 0) + " Nm" },
        { "Clutch temperature", juce::String(state.clutchTemperatureC, 1) + utf8(" \xc2\xb0""C") },
        { "Clutch loss", juce::String(state.clutchPowerLossKw, 2) + " kW" },
        { "Brake", fixed(state.brakePressure * 100.0, 0) + " %" },
        { "Tyre force", fixed(state.tireLongitudinalForceN, 0) + " N" },
        { "Fuel economy", juce::String(state.fuelEconomyLitresPer100Km, 2) + " L/100 km" },
        { "Dyno mode", benchText },
        { "Hold speed", groupedInteger(state.dynoHoldRpm) + " rpm" },
    };
}

struct Trace final {
    const char* name;
    juce::Colour colour;
    double minimum;
    double maximum;
    double (*value)(const EngineState&);
};

constexpr int traceChartHeight = 190;
constexpr int engineRowCount = 12;
constexpr int drivelineRowCount = 13;
constexpr int mixRowHeight = 34;
constexpr int mixRowCount = 9;
} // namespace

TelemetryPanel::TelemetryPanel(const DashboardModel& model) : model_(model) {}

int TelemetryPanel::preferredHeight() const {
    return padding + sectionHeight + traceChartHeight + 30 + 12
        + sectionHeight + engineRowCount * rowHeight + 18
        + sectionHeight + drivelineRowCount * rowHeight + padding;
}

void TelemetryPanel::paint(juce::Graphics& g) {
    auto area = getLocalBounds().toFloat().reduced(padding, padding);
    drawSectionLabel(g, area.removeFromTop(sectionHeight), utf8("Live traces  \xc2\xb7  10 s"));
    const auto chart = area.removeFromTop(traceChartHeight);
    fillCard(g, chart);
    const auto plot = chart.reduced(10.0F, 12.0F);
    g.setColour(colours::grid);
    for (int grid = 1; grid < 5; ++grid)
        g.drawHorizontalLine(static_cast<int>(plot.getY() + plot.getHeight() * static_cast<float>(grid) / 5.0F),
                             plot.getX(), plot.getRight());
    const auto redline = std::max(1.0, model_.config.redlineRpm);
    const std::array<Trace, 5> traces {{
        { "RPM", colours::accent, 0.0, redline, [](const EngineState& s) { return s.rpm; } },
        { "MAP", colours::intake, 20.0, 110.0, [](const EngineState& s) { return s.manifoldPressureKpa; } },
        { "Lambda", colours::ok, 0.55, 1.35, [](const EngineState& s) { return s.lambda; } },
        { "Knock", juce::Colour(0xffffca28), 0.0, 1.0, [](const EngineState& s) { return s.knockLevel; } },
        { "Exhaust", juce::Colour(0xffff7a45), 80.0, 220.0,
          [](const EngineState& s) { return s.exhaustRunnerPressureKpa; } },
    }};
    const auto& telemetry = model_.telemetry;
    if (telemetry.size() > 1) {
        for (const auto& trace : traces) {
            juce::Path path;
            for (std::size_t sample = 0; sample < telemetry.size(); ++sample) {
                const auto raw = trace.value(telemetry.at(sample));
                const auto normalised = std::clamp((raw - trace.minimum) / (trace.maximum - trace.minimum), 0.0, 1.0);
                const auto x = plot.getX() + plot.getWidth() * static_cast<float>(sample)
                    / static_cast<float>(telemetry.size() - 1);
                const auto y = plot.getBottom() - plot.getHeight() * static_cast<float>(normalised);
                if (sample == 0) path.startNewSubPath(x, y);
                else path.lineTo(x, y);
            }
            g.setColour(trace.colour);
            g.strokePath(path, juce::PathStrokeType(trace.colour == colours::accent ? 2.0F : 1.5F,
                                                    juce::PathStrokeType::curved));
        }
    }
    auto legend = area.removeFromTop(30.0F).withTrimmedTop(8.0F);
    g.setFont(uiFont(13.0F));
    for (const auto& trace : traces) {
        const juce::String name(trace.name);
        const auto width = juce::GlyphArrangement::getStringWidth(g.getCurrentFont(), name) + 26.0F;
        auto chip = legend.removeFromLeft(width);
        g.setColour(trace.colour);
        g.fillRoundedRectangle(chip.removeFromLeft(9.0F).withSizeKeepingCentre(9.0F, 9.0F), 2.0F);
        chip.removeFromLeft(6.0F);
        g.setColour(colours::muted);
        g.drawText(name, chip, juce::Justification::centredLeft, false);
    }
    area.removeFromTop(12.0F);
    drawSectionLabel(g, area.removeFromTop(sectionHeight), "Engine");
    drawRows(g, area, engineRows(model_));
    area.removeFromTop(18.0F);
    drawSectionLabel(g, area.removeFromTop(sectionHeight), "Driveline");
    drawRows(g, area, drivelineRows(model_));
}

AudioPanel::AudioPanel(const DashboardModel& model) : model_(model) {
    const std::array<const char*, 5> presets { "Street", "Open", "Turbo", "Long tube", "Moto" };
    for (int index = 0; index < static_cast<int>(presets.size()); ++index)
        exhaustPreset_.addItem(presets[static_cast<std::size_t>(index)], index + 1);
    exhaustPreset_.addItem("Physical graph", 6);
    exhaustPreset_.setWantsKeyboardFocus(false);
    exhaustPreset_.onChange = [this] {
        const auto selected = exhaustPreset_.getSelectedItemIndex();
        if (selected >= 0 && selected < 5 && onExhaustPreset) onExhaustPreset(selected);
    };
    addAndMakeVisible(exhaustPreset_);
    keyCaps_.resize(mixRowCount);
}

void AudioPanel::setKeyCaps(const ActionMap& actions) {
    const std::array<AppAction, mixRowCount> actionsInOrder {
        AppAction::wheelVolume, AppAction::wheelConvolution, AppAction::wheelHighGain,
        AppAction::wheelLowNoise, AppAction::wheelHighNoise, AppAction::wheelCombustion,
        AppAction::wheelExhaust, AppAction::wheelIntake, AppAction::wheelMechanical };
    for (std::size_t index = 0; index < actionsInOrder.size(); ++index)
        keyCaps_[index] = keyCapText(actions.shortcut(actionsInOrder[index]));
    repaint();
}

void AudioPanel::syncExhaustPreset() {
    const auto& audio = model_.audio;
    if (audio.physicalExhaustTopology) {
        exhaustPreset_.setSelectedId(6, juce::dontSendNotification);
        exhaustPreset_.setEnabled(false);
        exhaustPreset_.setTooltip(utf8("The physical exhaust graph owns the sound. Change its geometry "
                                       "in EXHAUST PRO instead of applying a procedural preset."));
    } else {
        exhaustPreset_.setSelectedItemIndex(audio.exhaustPresetIndex, juce::dontSendNotification);
        exhaustPreset_.setEnabled(true);
        exhaustPreset_.setTooltip(utf8("Compatibility preset for a configuration without a physical graph."));
    }
}

int AudioPanel::preferredHeight() const {
    return padding + sectionHeight + 34 + 26 + 16 + sectionHeight + mixRowCount * mixRowHeight + 56 + padding;
}

void AudioPanel::paint(juce::Graphics& g) {
    const auto& audio = model_.audio;
    const auto& mix = audio.mix;
    auto area = getLocalBounds().toFloat().reduced(padding, padding);
    drawSectionLabel(g, area.removeFromTop(sectionHeight), "Exhaust");
    area.removeFromTop(34.0F);
    g.setColour(audio.impulseResponseLoadError ? colours::accent : colours::muted);
    g.setFont(uiFont(14.0F));
    g.drawText(audio.impulseResponseStatus, area.removeFromTop(26.0F), juce::Justification::centredLeft, true);
    area.removeFromTop(16.0F);
    drawSectionLabel(g, area.removeFromTop(sectionHeight), "Mix");

    struct Level final { juce::String label; double fraction; double value; bool available; };
    const std::array<Level, mixRowCount> levels {{
        { "Volume", mix.volume / 2.0, mix.volume, true },
        { audio.impulseResponseAvailable ? "Measured IR send" : "IR send",
          mix.convolution, mix.convolution, audio.impulseResponseAvailable },
        { "High gain", mix.highFrequencyGain / 2.5, mix.highFrequencyGain, true },
        { "Low noise", mix.lowFrequencyNoise / 1.5, mix.lowFrequencyNoise, !audio.physicalIntakeTopology },
        { "High noise", mix.highFrequencyNoise / 1.5, mix.highFrequencyNoise, !audio.physicalExhaustTopology },
        { "Direct combustion", mix.combustionGain / 2.0, mix.combustionGain, !audio.physicalExhaustTopology },
        { "Exhaust", mix.exhaustGain / 2.0, mix.exhaustGain, true },
        { audio.forcedInductionAcousticsActive ? "Intake + turbo" : "Intake",
          mix.intakeGain / 2.0, mix.intakeGain, true },
        { audio.structuralRadiationActive ? "Structure" : "Mechanical",
          mix.mechanicalGain / 2.0, mix.mechanicalGain, true },
    }};
    for (std::size_t index = 0; index < levels.size(); ++index) {
        const auto& level = levels[index];
        auto row = area.removeFromTop(static_cast<float>(mixRowHeight)).reduced(0.0F, 3.0F);
        fillCard(g, row, 7.0F, colours::panel, colours::line);
        auto content = row.reduced(8.0F, 0.0F);
        if (keyCaps_[index].isNotEmpty()) {
            const auto capWidth = std::max(24.0F, keyCapWidth(keyCaps_[index]));
            drawKeyCap(g, content.removeFromLeft(capWidth), keyCaps_[index]);
            content.removeFromLeft(8.0F);
        }
        g.setColour(level.available ? colours::text : colours::faint);
        g.setFont(uiFont(14.0F));
        g.drawFittedText(level.label, content.removeFromLeft(118.0F).toNearestInt(),
                         juce::Justification::centredLeft, 1, 0.8F);
        g.setFont(monoFont(13.0F, false));
        g.setColour(level.available ? colours::muted : colours::faint);
        g.drawText(level.available ? juce::String(level.value, 2) : juce::String("n/a"),
                   content.removeFromRight(40.0F), juce::Justification::centredRight, false);
        content.removeFromRight(8.0F);
        const auto meter = content.withSizeKeepingCentre(content.getWidth(), 6.0F);
        g.setColour(colours::line);
        g.fillRoundedRectangle(meter, 3.0F);
        if (level.available) {
            g.setColour(colours::accent);
            g.fillRoundedRectangle(meter.withWidth(meter.getWidth()
                * static_cast<float>(std::clamp(level.fraction, 0.0, 1.0))), 3.0F);
        }
    }
    area.removeFromTop(10.0F);
    g.setColour(colours::faint);
    g.setFont(uiFont(13.5F));
    g.drawFittedText(utf8("Hold a key and turn the mouse wheel to adjust a level. "
                          "Audio, in the top bar, opens the full workshop."),
                     area.removeFromTop(40.0F).toNearestInt(), juce::Justification::topLeft, 2, 0.9F);
}

void AudioPanel::resized() {
    exhaustPreset_.setBounds(padding, padding + sectionHeight, getWidth() - 2 * padding, 32);
}

DiagnosticsPanel::DiagnosticsPanel(const DashboardModel& model) : model_(model) {}

int DiagnosticsPanel::preferredHeight() const {
    const auto faults = std::max<std::size_t>(1, model_.diagnostics.size());
    return padding + sectionHeight + static_cast<int>(faults) * 50 + 18
        + sectionHeight + static_cast<int>(debugRows().size()) * 22 + padding;
}

void DiagnosticsPanel::paint(juce::Graphics& g) {
    auto area = getLocalBounds().toFloat().reduced(padding, padding);
    drawSectionLabel(g, area.removeFromTop(sectionHeight), "Faults");
    if (model_.diagnostics.empty()) {
        auto card = area.removeFromTop(50.0F).withTrimmedBottom(6.0F);
        fillCard(g, card, 8.0F, colours::okFill, colours::okBorder);
        g.setColour(colours::ok);
        g.setFont(uiFont(14.5F, true));
        g.drawText(utf8("No fault detected"), card.reduced(12.0F, 0.0F), juce::Justification::centredLeft, true);
    }
    for (const auto& diagnostic : model_.diagnostics) {
        auto card = area.removeFromTop(50.0F).withTrimmedBottom(6.0F);
        const auto critical = diagnostic.severity == DiagnosticSeverity::critical;
        const auto information = diagnostic.severity == DiagnosticSeverity::information;
        const auto accent = critical ? colours::critical : (information ? colours::intake : colours::warn);
        fillCard(g, card, 8.0F, critical ? colours::criticalFill : colours::panel, colours::line);
        g.setColour(accent);
        g.fillRoundedRectangle(card.removeFromLeft(4.0F).reduced(0.0F, 6.0F), 2.0F);
        auto body = card.reduced(10.0F, 4.0F);
        g.setFont(monoFont(11.5F, false));
        g.setColour(colours::faint);
        g.drawText(juce::String::fromUTF8(diagnostic.code.c_str()), body.removeFromTop(14.0F),
                   juce::Justification::centredLeft, true);
        g.setColour(colours::text);
        g.setFont(uiFont(14.0F));
        g.drawFittedText(juce::String::fromUTF8(diagnostic.message.c_str()), body.toNearestInt(),
                         juce::Justification::centredLeft, 1, 0.75F);
    }
    area.removeFromTop(18.0F);
    drawSectionLabel(g, area.removeFromTop(sectionHeight), "Physics and real-time counters");
    drawRows(g, area, debugRows(), 22.0F);
}

std::vector<std::pair<juce::String, juce::String>> DiagnosticsPanel::debugRows() const {
    const auto& state = model_.state;
    const auto& health = model_.health;
    const auto cylinderCount = std::max<std::size_t>(1, state.cylinderStateCount);
    double maxPressure = 0.0;
    double intakeLift = 0.0;
    double exhaustLift = 0.0;
    double flameSpeed = 0.0;
    double burnedFraction = 0.0;
    double injectorCapacity = 1.0;
    double intakeAdvance = 0.0;
    double liftMultiplier = 0.0;
    double runnerResonanceHz = 0.0;
    for (std::size_t index = 0; index < cylinderCount; ++index) {
        const auto& cylinder = state.cylinderStates[index];
        maxPressure = std::max(maxPressure, cylinder.pressureEstimateBar);
        intakeLift = std::max(intakeLift, cylinder.intakeValveLiftMm);
        exhaustLift = std::max(exhaustLift, cylinder.exhaustValveLiftMm);
        flameSpeed = std::max(flameSpeed, cylinder.flameSpeedMps);
        burnedFraction = std::max(burnedFraction, cylinder.burnedFraction);
        injectorCapacity = std::min(injectorCapacity, cylinder.injectorCapacityRatio);
        intakeAdvance = std::max(intakeAdvance, cylinder.intakeValveAdvanceDegrees);
        liftMultiplier = std::max(liftMultiplier, cylinder.valveLiftMultiplier);
        runnerResonanceHz = std::max(runnerResonanceHz, cylinder.intakeResonanceFrequencyHz);
    }
    const auto count = [](std::uint64_t value) { return juce::String(static_cast<juce::int64>(value)); };
    return {
        { "Net torque", juce::String(state.netTorqueNm, 2) + " Nm" },
        { "Indicated torque", juce::String(state.indicatedTorqueNm, 2) + " Nm" },
        { "Mean-work torque", juce::String(state.meanWorkTorqueNm, 2) + " Nm" },
        { "Pressure torque", juce::String(state.cylinderPressureTorqueNm, 2) + " Nm" },
        { "Pressure torque active", state.cylinderPressureTorqueBlend > 0.5 ? "yes" : "startup" },
        { "P-dV work", juce::String(state.indicatedWorkJoulesPerCycle, 2) + " J/cycle" },
        { "IMEP", juce::String(state.indicatedMeanEffectivePressureBar, 3) + " bar" },
        { "Indicated power", juce::String(state.indicatedPowerKw, 3) + " kW" },
        { "P-dV mean torque", juce::String(state.pdvTorqueNm, 3) + " Nm" },
        { "Friction torque", juce::String(state.frictionTorqueNm, 2) + " Nm" },
        { "FMEP", juce::String(state.frictionMeanEffectivePressureBar, 2) + " bar" },
        { "Reciprocating torque", juce::String(state.reciprocatingTorqueNm, 2) + " Nm" },
        { "Load torque", juce::String(state.loadTorqueNm, 2) + " Nm" },
        { "Starter torque", juce::String(state.starterTorqueNm, 2) + " Nm" },
        { "Manifold pressure", juce::String(state.manifoldPressureKpa, 3) + " kPa" },
        { "Intake runner", juce::String(state.intakeRunnerPressureKpa, 3) + " kPa" },
        { "Exhaust runner", juce::String(state.exhaustRunnerPressureKpa, 3) + " kPa" },
        { "Exhaust flow", juce::String(state.exhaustFlowGramsPerSecond, 4) + " g/s" },
        { "Air mass", juce::String(state.airMassMgPerCycle, 3) + " mg/cycle" },
        { "Fuel mass", juce::String(state.injectedFuelMgPerCycle, 4) + " mg/cycle" },
        { "Fuel consumed", juce::String(state.fuelConsumedGrams, 3) + " g / "
              + juce::String(state.fuelConsumedLitres, 5) + " L" },
        { "Consumption", juce::String(state.fuelEconomyLitresPer100Km, 2) + " L/100 km" },
        { "Gas in manifold", juce::String(state.manifoldGasMassGrams, 4) + " g" },
        { "Gas in cylinders", juce::String(state.cylinderGasMassGrams, 4) + " g" },
        { "Gas internal energy", juce::String(state.gasInternalEnergyJoules, 1) + " J" },
        { "Boost ratio", juce::String(state.boostPressureRatio, 3) },
        { "Turbo shaft", fixed(state.forcedInductionShaftSpeedRpm, 0) + " rpm / "
              + juce::String(state.forcedInductionShaftSpeedRatio, 3)
              + (state.forcedInductionShaftSpeedRatio > 1.0 ? " >design" : "") },
        { "Turbine / compressor / bearing", juce::String(state.turbinePowerKw, 2) + " / "
              + juce::String(state.compressorPowerKw, 2) + " / "
              + juce::String(state.turboBearingPowerKw, 2) + " kW" },
        { "Turbo shaft net", juce::String(state.turboShaftNetPowerKw, 3) + " kW" },
        { "Solver", fixed(state.solverFrequencyHz, 0) + " Hz / "
              + juce::String(state.solverSubsteps) + " steps" },
        { "Crank step", juce::String(state.crankDegreesPerSolverStep, 3) + utf8("\xc2\xb0") },
        { "Solver resolution limited", state.solverResolutionLimited ? "YES" : "no" },
        { "Max cylinder pressure", juce::String(maxPressure, 3) + " bar" },
        { "Intake lift max", juce::String(intakeLift, 3) + " mm" },
        { "Exhaust lift max", juce::String(exhaustLift, 3) + " mm" },
        { "Flame speed", juce::String(flameSpeed, 3) + " m/s" },
        { "Burned fraction", juce::String(burnedFraction * 100.0, 1) + " %" },
        // The field is duty HEADROOM, so the readout is the duty itself: that
        // is the number an engine builder compares against the 85-90 % sizing
        // limit, and showing its complement invited it to be read backwards.
        { "Injector duty", juce::String((1.0 - injectorCapacity) * 100.0, 1) + " %" },
        { "VVT intake", juce::String(intakeAdvance, 2) + utf8("\xc2\xb0") },
        { "VVL multiplier", juce::String(liftMultiplier, 3) },
        { "Runner resonance", juce::String(runnerResonanceHz, 1) + " Hz" },
        { "Clutch temperature", juce::String(state.clutchTemperatureC, 2) + utf8(" \xc2\xb0""C") },
        { "Clutch energy", juce::String(state.clutchDissipatedEnergyJoules, 1) + " J" },
        { "Tyre limited", state.tractionLimited ? "YES" : "no" },
        { "Damage / wear", juce::String(state.damage, 4) + " / " + juce::String(state.wear, 4) },
        // Each real-time transport failure stays separate. A single aggregate
        // made a silent exhaust indistinguishable from missing combustion,
        // pressure starvation, or a saturated reaction voice pool.
        { "Dropped firings", count(health.droppedFirings) },
        { "Dropped pressure samples", count(health.droppedPressureSamples) },
        { "Dropped exhaust acoustic", count(health.droppedExhaustAcousticSamples) },
        { "Dropped reaction events", count(health.droppedReactionEvents) },
        { "Afterfire pressure limit", count(health.reactionPressureLimitedSamples) },
        { "Dropped dyno cycles", count(health.droppedDynoCycles) },
        { "Dyno gate mask", juce::String(static_cast<juce::int64>(state.dynoQualityReasons)) },
        { "Dyno contact / saturation", juce::String(state.dynoBrakeContactFraction, 3) + " / "
              + (state.dynoAbsorberSaturatedHigh ? "HIGH" : (state.dynoAbsorberSaturatedLow ? "low" : "no")) },
        { "Dyno window speed", juce::String(state.dynoWindowMeanRpm, 1) + " ["
              + fixed(state.dynoWindowMinimumRpm, 0) + ", "
              + fixed(state.dynoWindowMaximumRpm, 0) + "]" },
        { "Dyno window", juce::String(state.dynoWindowDurationSeconds, 3) + " s / "
              + juce::String(state.dynoAcceptedCycleCount)
              + (state.dynoMeasurementReady ? " ready" : " warming") },
        // Simulated seconds delivered per wall second. Below 1.0 the simulation
        // is in slow motion, controls answer late in proportion to cylinder
        // count, and the cylinder-pressure telemetry -- the exhaust chain's
        // only excitation -- is produced slower than the audio thread drains it.
        { "Real-time factor", juce::String(health.realtimeFactor, 3) + utf8(" \xc3\x97") },
        { "Runtime overruns", count(health.timingOverruns) },
        { "Audio late / pending", count(health.lateAudioEvents) + " / " + count(health.droppedPendingAudioEvents) },
        { "Load protection", juce::String(health.loadProtectionActive ? "ACTIVE / " : "idle / ")
              + count(health.loadProtectionActivations) + " activations" },
        // Every output non-linearity stays distinct: the slow safety AGC, the
        // voicing saturation, the 2x soft limiter and the hard clamp.
        { "AGC gain / frames", juce::String(health.minimumLevelGain, 3) + " / " + count(health.levelLimitedSamples) },
        { "Voicing saturation frames", count(health.saturationProcessedSamples) },
        { "Soft-limited samples", count(health.softLimitedSamples) },
        { "Hard-clamped frames", count(health.hardClampedSamples) },
        { "Post-limit peak", juce::String(health.maximumPostLimiterMagnitude, 4) },
        // The afterfire has nine independent preconditions and produces the
        // same silence whichever one is missing: state first, diagnosis after.
        { "Afterfire", state.exhaustAfterfireOverrunActive
              ? "ACTIVE " + juce::String(state.exhaustAfterfireHeatReleaseKw, 2) + " kW"
              : juce::String(afterfireBlockerName(state.exhaustAfterfireBlockers)) },
        { "Afterfire induction", juce::String(state.exhaustAfterfireInductionProgress, 3) + " / "
              + juce::String(state.exhaustAfterfireMinimumInductionDelayMs, 3) + ".."
              + juce::String(state.exhaustAfterfireMaximumInductionDelayMs, 3) + " ms" },
        // The ignition source during overrun is the pipe wall -- the gas is
        // cold once the spark is cut -- and 1.5 mm of steel has a time constant
        // near 55 s. A just-started engine cannot pop, and that is correct.
        { "Exhaust wall / ignition", fixed(state.exhaustWallTemperatureC, 0) + " / "
              + fixed(model_.config.exhaustAfterfire.ignitionTemperatureK - 273.15, 0) + utf8(" \xc2\xb0""C") },
    };
}

SidePanel::SidePanel(const DashboardModel& model, std::function<bool()> wheelModifierActive)
    : dyno(model, wheelModifierActive), audio(model), model_(model), telemetry_(model), diagnostics_(model) {
    tabs_.onChange = [this](int tab) { showTab(tab); };
    addAndMakeVisible(tabs_);
    addAndMakeVisible(dyno);
    for (auto* pane : { &telemetryPane_, &audioPane_, &diagnosticsPane_ }) {
        pane->forwardWheel = wheelModifierActive;
        addChildComponent(*pane);
    }
    telemetryPane_.setViewedComponent(&telemetry_, false);
    audioPane_.setViewedComponent(&audio, false);
    diagnosticsPane_.setViewedComponent(&diagnostics_, false);
}

void SidePanel::showTab(int tab) {
    tab = std::clamp(tab, 0, tabCount - 1);
    tabs_.setSelected(tab);
    dyno.setVisible(tab == dynoTab);
    telemetryPane_.setVisible(tab == telemetryTab);
    audioPane_.setVisible(tab == audioTab);
    diagnosticsPane_.setVisible(tab == diagnosticsTab);
    refresh();
}

void SidePanel::nextTab() { showTab((currentTab() + 1) % tabCount); }

void SidePanel::fitContent(ScrollPane& pane, juce::Component& content, int height) {
    const auto visibleWidth = pane.getMaximumVisibleWidth();
    if (content.getWidth() != visibleWidth || content.getHeight() != height) content.setSize(visibleWidth, height);
}

void SidePanel::refresh() {
    switch (currentTab()) {
    case dynoTab: dyno.refresh(); break;
    case telemetryTab:
        fitContent(telemetryPane_, telemetry_, telemetry_.preferredHeight());
        telemetry_.repaint();
        break;
    case audioTab:
        fitContent(audioPane_, audio, audio.preferredHeight());
        audio.repaint();
        break;
    case diagnosticsTab:
        fitContent(diagnosticsPane_, diagnostics_, diagnostics_.preferredHeight());
        diagnostics_.repaint();
        break;
    default: break;
    }
}

void SidePanel::paint(juce::Graphics& g) {
    g.fillAll(colours::background);
    g.setColour(colours::line);
    g.fillRect(getLocalBounds().removeFromLeft(1));
    g.fillRect(juce::Rectangle<int>(0, tabs_.getBottom(), getWidth(), 1));
}

void SidePanel::resized() {
    auto area = getLocalBounds().withTrimmedLeft(1);
    auto tabRow = area.removeFromTop(48);
    tabs_.setBounds(tabRow.withTrimmedLeft(12).withTrimmedTop(10).withWidth(tabs_.idealWidth()));
    area.removeFromTop(1);
    dyno.setBounds(area);
    for (auto* pane : { &telemetryPane_, &audioPane_, &diagnosticsPane_ }) pane->setBounds(area);
    fitContent(telemetryPane_, telemetry_, telemetry_.preferredHeight());
    fitContent(audioPane_, audio, audio.preferredHeight());
    fitContent(diagnosticsPane_, diagnostics_, diagnostics_.preferredHeight());
}

} // namespace enginelab::ui
