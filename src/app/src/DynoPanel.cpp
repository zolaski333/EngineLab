#include <enginelab/app/DynoPanel.hpp>

#include <enginelab/app/Theme.hpp>
#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

namespace enginelab::ui {
namespace {
[[nodiscard]] double torqueOf(const DynoPoint& point) noexcept {
    return point.correctedTorqueNm > 0.0 ? point.correctedTorqueNm : point.torqueNm;
}

[[nodiscard]] double powerOf(const DynoPoint& point) noexcept {
    return point.correctedPowerKw > 0.0 ? point.correctedPowerKw : point.powerKw;
}

[[nodiscard]] juce::String kiloRpm(double rpm) {
    const auto thousands = rpm / 1'000.0;
    return juce::String(thousands, std::abs(thousands - std::round(thousands)) < 0.05 ? 0 : 1) + "k";
}

[[nodiscard]] juce::Colour runColour(const DashboardModel& model, std::size_t index) {
    const auto* presentation = model.presentationFor(model.archivedRuns[index].id);
    return juce::Colour(presentation != nullptr ? presentation->colour : 0xffffffffU);
}

[[nodiscard]] juce::String runName(const DashboardModel& model, std::size_t index) {
    const auto& run = model.archivedRuns[index];
    const auto* presentation = model.presentationFor(run.id);
    return presentation != nullptr ? presentation->name : juce::String::fromUTF8(run.engineName.c_str());
}

[[nodiscard]] bool runVisible(const DashboardModel& model, const DynoRun& run) {
    const auto* presentation = model.presentationFor(run.id);
    return presentation == nullptr || presentation->visible;
}

struct Peak final { double value { 0.0 }; double rpm { 0.0 }; };

[[nodiscard]] std::pair<Peak, Peak> peaksOf(const DynoRun& run) {
    Peak torque;
    Peak power;
    for (const auto& point : run.points) {
        if (!point.valid) continue;
        if (torqueOf(point) > torque.value) torque = { torqueOf(point), point.rpm };
        if (powerOf(point) > power.value) power = { powerOf(point), point.rpm };
    }
    return { torque, power };
}
} // namespace

DynoChart::DynoChart(const DashboardModel& model) : model_(model) {}

void DynoChart::mouseMove(const juce::MouseEvent& event) {
    mouse_ = event.position;
    repaint();
}

void DynoChart::mouseExit(const juce::MouseEvent&) {
    mouse_.reset();
    repaint();
}

void DynoChart::paint(juce::Graphics& g) {
    const auto& model = model_;
    const auto& state = model.state;
    auto area = getLocalBounds().toFloat();
    fillCard(g, area);
    const auto plot = area.withTrimmedLeft(42.0F).withTrimmedRight(40.0F).withTrimmedTop(28.0F)
        .withTrimmedBottom(26.0F);

    auto maximumVisibleRpm = model.currentRun.points.empty() ? 0.0 : model.config.redlineRpm;
    const auto accumulateRpm = [&maximumVisibleRpm](const DynoRun& run) {
        for (const auto& point : run.points)
            if (point.valid && std::isfinite(point.rpm))
                maximumVisibleRpm = std::max(maximumVisibleRpm, point.rpm);
    };
    for (const auto& run : model.archivedRuns)
        if (runVisible(model, run)) accumulateRpm(run);
    accumulateRpm(model.currentRun);
    const auto maxRpm = maximumVisibleRpm > 0.0
        ? std::max(1'000.0, std::ceil(maximumVisibleRpm * 1.05 / 500.0) * 500.0)
        : std::max(1'000.0, model.config.redlineRpm);

    double maxTorque = 100.0;
    double maxPower = 75.0;
    const auto accumulateMax = [&maxTorque, &maxPower](const DynoRun& run) {
        for (const auto& point : run.points) {
            if (!point.valid) continue;
            maxTorque = std::max(maxTorque, torqueOf(point));
            maxPower = std::max(maxPower, powerOf(point));
        }
    };
    for (const auto& run : model.archivedRuns)
        if (runVisible(model, run)) accumulateMax(run);
    accumulateMax(model.currentRun);

    g.setFont(monoFont(12.0F, false));
    for (int grid = 0; grid <= 5; ++grid) {
        const auto y = plot.getBottom() - plot.getHeight() * static_cast<float>(grid) / 5.0F;
        g.setColour(colours::grid);
        g.drawHorizontalLine(static_cast<int>(y), plot.getX(), plot.getRight());
        g.setColour(colours::faint);
        g.drawText(fixed(maxTorque * static_cast<double>(grid) / 5.0, 0),
                   juce::Rectangle<float>(area.getX() + 4.0F, y - 7.0F, 34.0F, 14.0F),
                   juce::Justification::centredRight);
        g.drawText(fixed(maxPower * static_cast<double>(grid) / 5.0, 0),
                   juce::Rectangle<float>(plot.getRight() + 6.0F, y - 7.0F, 34.0F, 14.0F),
                   juce::Justification::centredLeft);
    }
    for (int grid = 0; grid <= 4; ++grid) {
        const auto x = plot.getX() + plot.getWidth() * static_cast<float>(grid) / 4.0F;
        g.drawText(kiloRpm(maxRpm * static_cast<double>(grid) / 4.0),
                   juce::Rectangle<float>(x - 25.0F, plot.getBottom() + 5.0F, 50.0F, 14.0F),
                   juce::Justification::centred);
    }
    g.setFont(uiFont(13.0F));
    const auto top = juce::Rectangle<float>(area.getX() + 12.0F, area.getY() + 7.0F,
                                            area.getWidth() - 24.0F, 16.0F);
    g.drawText("Nm", top, juce::Justification::centredLeft);
    g.drawText("kW", top, juce::Justification::centredRight);
    g.drawText(utf8("solid: torque  \xc2\xb7  dashed: power"), top, juce::Justification::centred);

    const auto drawRun = [&](const DynoRun& run, juce::Colour colour, bool emphasised) {
        if (run.points.empty()) return;
        juce::Path torquePath;
        juce::Path powerPath;
        // The recorder owns a physically averaged 50-rpm series. Drawing it
        // directly avoids a second, non-causal filter. Invalid bins break the
        // path instead of being bridged or plotted as zero.
        auto pathStarted = false;
        for (const auto& point : run.points) {
            const auto torque = torqueOf(point);
            const auto power = powerOf(point);
            if (!point.valid || !std::isfinite(torque) || !std::isfinite(power)) {
                pathStarted = false;
                continue;
            }
            const auto x = plot.getX() + static_cast<float>(point.rpm / maxRpm) * plot.getWidth();
            const auto torqueY = plot.getBottom() - static_cast<float>(torque / maxTorque) * plot.getHeight();
            const auto powerY = plot.getBottom() - static_cast<float>(power / maxPower) * plot.getHeight();
            if (!pathStarted) {
                torquePath.startNewSubPath(x, torqueY);
                powerPath.startNewSubPath(x, powerY);
                pathStarted = true;
            } else {
                torquePath.lineTo(x, torqueY);
                powerPath.lineTo(x, powerY);
            }
        }
        g.setColour(colour.withAlpha(emphasised ? 1.0F : 0.72F));
        g.strokePath(torquePath, juce::PathStrokeType(emphasised ? 2.6F : 1.6F,
                                                      juce::PathStrokeType::curved));
        g.setColour(colour.brighter(0.6F).withAlpha(emphasised ? 0.88F : 0.55F));
        juce::Path dashedPower;
        const float dashLengths[] { 7.0F, 4.0F };
        juce::PathStrokeType(emphasised ? 1.8F : 1.2F).createDashedStroke(dashedPower, powerPath, dashLengths, 2);
        g.fillPath(dashedPower);
    };
    for (std::size_t index = 0; index < model.archivedRuns.size(); ++index) {
        const auto& run = model.archivedRuns[index];
        if (!runVisible(model, run)) continue;
        drawRun(run, runColour(model, index), static_cast<int>(index) == model.selectedRun);
    }
    drawRun(model.currentRun, juce::Colours::white, true);

    if (state.dynoHoldEnabled) {
        const auto holdX = plot.getX() + static_cast<float>(state.dynoHoldRpm / maxRpm) * plot.getWidth();
        g.setColour(colours::ok.withAlpha(0.75F));
        g.drawVerticalLine(static_cast<int>(holdX), plot.getY(), plot.getBottom());
    }

    if (!mouse_ || !plot.contains(*mouse_)) return;
    const auto mouse = *mouse_;
    const DynoPoint* nearestPoint = nullptr;
    juce::String nearestName;
    auto nearestColour = juce::Colours::white;
    auto nearestDistance = std::numeric_limits<float>::max();
    const auto consider = [&](const DynoRun& run, const juce::String& name, juce::Colour colour) {
        for (const auto& point : run.points) {
            if (!point.valid) continue;
            const auto x = plot.getX() + static_cast<float>(point.rpm / maxRpm) * plot.getWidth();
            const auto distance = std::abs(mouse.x - x);
            if (distance < nearestDistance) {
                nearestDistance = distance;
                nearestPoint = &point;
                nearestName = name;
                nearestColour = colour;
            }
        }
    };
    for (std::size_t index = 0; index < model.archivedRuns.size(); ++index)
        if (runVisible(model, model.archivedRuns[index]))
            consider(model.archivedRuns[index], runName(model, index), runColour(model, index));
    consider(model.currentRun, utf8("Current run"), juce::Colours::white);
    if (nearestPoint == nullptr || nearestDistance > 14.0F) return;
    const auto text = nearestName + "\n"
        + groupedInteger(nearestPoint->rpm) + utf8(" rpm  \xc2\xb7  ")
        + juce::String(torqueOf(*nearestPoint), 1) + utf8(" Nm  \xc2\xb7  ")
        + juce::String(powerOf(*nearestPoint), 1) + " kW\n"
        + (nearestPoint->airFuelRatioValid
            ? "AFR " + juce::String(nearestPoint->airFuelRatio, 2) + " / target "
                + juce::String(nearestPoint->targetAirFuelRatio, 2)
            : utf8("AFR \xe2\x80\x94 invalid reading"));
    const auto tooltipWidth = std::min(250.0F, plot.getWidth());
    const auto tooltip = juce::Rectangle<float>(
        std::clamp(mouse.x + 12.0F, plot.getX(), plot.getRight() - tooltipWidth),
        std::max(plot.getY(), mouse.y - 62.0F), tooltipWidth, 56.0F);
    g.setColour(juce::Colour(0xf00b1110));
    g.fillRoundedRectangle(tooltip, 6.0F);
    g.setColour(nearestColour);
    g.drawRoundedRectangle(tooltip, 6.0F, 1.2F);
    g.setColour(colours::text);
    g.setFont(uiFont(13.0F));
    g.drawFittedText(text, tooltip.reduced(8.0F, 5.0F).toNearestInt(), juce::Justification::centredLeft, 3);
}

RunList::RunList(const DashboardModel& model) : model_(model) {
    setMouseCursor(juce::MouseCursor::PointingHandCursor);
}

int RunList::preferredHeight() const {
    return static_cast<int>(model_.archivedRuns.size()) * rowHeight;
}

void RunList::paint(juce::Graphics& g) {
    const auto count = static_cast<int>(model_.archivedRuns.size());
    for (int row = 0; row < count; ++row) {
        const auto index = static_cast<std::size_t>(count - 1 - row);
        const auto& run = model_.archivedRuns[index];
        const auto selected = static_cast<int>(index) == model_.selectedRun;
        const auto visible = runVisible(model_, run);
        auto cell = juce::Rectangle<float>(0.0F, static_cast<float>(row * rowHeight),
                                           static_cast<float>(getWidth()), static_cast<float>(rowHeight))
                        .reduced(0.0F, 2.0F);
        fillCard(g, cell, 8.0F, selected ? colours::panelRaised : colours::panel,
                 selected ? colours::lineHover : colours::line);
        auto content = cell.reduced(10.0F, 0.0F);
        const auto swatch = content.removeFromLeft(12.0F).withSizeKeepingCentre(12.0F, 3.0F);
        g.setColour(runColour(model_, index).withAlpha(visible ? 1.0F : 0.3F));
        g.fillRoundedRectangle(swatch, 1.5F);
        content.removeFromLeft(8.0F);
        const auto power = run.peakCorrectedPowerKw > 0.0 ? run.peakCorrectedPowerKw : run.peakPowerKw;
        g.setFont(monoFont(13.0F, false));
        g.setColour(colours::muted);
        g.drawText(juce::String(power, 1) + " kW", content.removeFromRight(70.0F),
                   juce::Justification::centredRight, false);
        if (run.status != DynoRunStatus::completed) {
            g.setColour(colours::warn);
            g.setFont(uiFont(12.0F, true));
            g.drawText(juce::String(dynoStatusLabel(run.status)).toLowerCase(), content.removeFromRight(70.0F),
                       juce::Justification::centredRight, false);
        }
        g.setColour(visible ? colours::text : colours::faint);
        g.setFont(uiFont(14.0F, selected));
        g.drawFittedText(runName(model_, index) + (visible ? juce::String {} : juce::String("  (hidden)")),
                         content.toNearestInt(), juce::Justification::centredLeft, 1, 0.85F);
    }
}

void RunList::mouseDown(const juce::MouseEvent& event) {
    const auto count = static_cast<int>(model_.archivedRuns.size());
    const auto row = static_cast<int>(event.position.y) / rowHeight;
    if (row < 0 || row >= count || !onSelect) return;
    onSelect(count - 1 - row);
}

DynoPanel::DynoPanel(const DashboardModel& model, std::function<bool()> wheelModifierActive)
    : model_(model), chart_(model), runs_(model) {
    runs_.onSelect = [this](int index) { if (onSelectRun) onSelectRun(index); };
    runsPane_.setViewedComponent(&runs_, false);
    runsPane_.forwardWheel = std::move(wheelModifierActive);
    runName_.setTextToShowWhenEmpty(utf8("Run name"), colours::faint);
    runName_.setSelectAllWhenFocused(true);
    runName_.setFont(uiFont(14.0F));
    runName_.setIndents(8, 6);
    runName_.onTextChange = [this] {
        if (!updatingName_ && onRenameRun) onRenameRun(runName_.getText().trim());
    };
    runName_.onReturnKey = [this] { unfocusAllComponents(); };
    colourButton_.onClick = [this] { if (onCycleColour) onCycleColour(); };
    visibilityButton_.onClick = [this] { if (onToggleVisibility) onToggleVisibility(); };
    deleteButton_.onClick = [this] { if (onDeleteRun) onDeleteRun(); };
    csvButton_.onClick = [this] { if (onExportCsv) onExportCsv(); };
    colourButton_.setTooltip(utf8("Next colour for the selected run"));
    csvButton_.setTooltip(utf8("Export the selected run (or the current one) as CSV"));
    for (juce::Component* child : { static_cast<juce::Component*>(&chart_),
                                    static_cast<juce::Component*>(&runsPane_),
                                    static_cast<juce::Component*>(&runName_),
                                    static_cast<juce::Component*>(&colourButton_),
                                    static_cast<juce::Component*>(&visibilityButton_),
                                    static_cast<juce::Component*>(&deleteButton_),
                                    static_cast<juce::Component*>(&csvButton_) })
        addAndMakeVisible(*child);
    syncRuns();
}

void DynoPanel::refresh() {
    chart_.repaint();
    repaint(header_.getSmallestIntegerContainer());
    repaint(peaks_.getSmallestIntegerContainer());
}

void DynoPanel::syncRuns() {
    const auto* run = model_.selectedArchivedRun();
    for (juce::Component* control : { static_cast<juce::Component*>(&runName_),
                                      static_cast<juce::Component*>(&colourButton_),
                                      static_cast<juce::Component*>(&visibilityButton_),
                                      static_cast<juce::Component*>(&deleteButton_) })
        control->setEnabled(run != nullptr);
    updatingName_ = true;
    if (run != nullptr) {
        const auto* presentation = model_.presentationFor(run->id);
        if (presentation != nullptr && !runName_.hasKeyboardFocus(false))
            runName_.setText(presentation->name, false);
        visibilityButton_.setButtonText(presentation == nullptr || presentation->visible ? "Hide" : "Show");
    } else {
        runName_.clear();
        visibilityButton_.setButtonText("Hide");
    }
    updatingName_ = false;
    runs_.setSize(runsPane_.getMaximumVisibleWidth(), std::max(1, runs_.preferredHeight()));
    runs_.repaint();
    resized();
    repaint();
}

void DynoPanel::paint(juce::Graphics& g) {
    const auto& state = model_.state;
    g.setColour(colours::text);
    g.setFont(uiFont(17.0F, true));
    g.drawText("Brake dynamometer", header_, juce::Justification::centredLeft, false);
    juce::String status;
    auto statusColour = colours::muted;
    if (state.dynoActive) {
        statusColour = state.dynoPreparing || state.dynoPhase == DynoPhase::recovery ? colours::warn : colours::ok;
        status = state.dynoPhase == DynoPhase::recovery
            ? utf8("Recovering \xe2\x86\x92 ") + groupedInteger(state.dynoTargetRpm) + " rpm"
            : (state.dynoPreparing
                ? utf8("Preparing \xe2\x86\x92 ") + groupedInteger(state.dynoTargetRpm) + " rpm"
                : "Measuring " + groupedInteger(state.dynoTargetRpm) + utf8(" rpm \xc2\xb7 ")
                    + fixed(state.dynoProgress * 100.0, 0) + " %");
    } else if (state.dynoHoldEnabled) {
        statusColour = colours::ok;
        status = "Hold " + groupedInteger(state.dynoHoldRpm) + " rpm";
    } else {
        status = state.dynoRampEnabled
            ? utf8("Ramp \xc2\xb7 ") + fixed(state.dynoRampRpmPerSecond, 0) + " rpm/s"
            : utf8("Steps \xc2\xb7 250 rpm");
    }
    g.setColour(statusColour);
    g.setFont(uiFont(14.0F));
    g.drawText(status, header_, juce::Justification::centredRight, true);

    const auto* selected = model_.selectedArchivedRun();
    const auto& source = selected != nullptr && !model_.dynoRunning ? *selected : model_.currentRun;
    const auto [torque, power] = peaksOf(source);
    auto cards = peaks_;
    const auto cardWidth = (cards.getWidth() - 8.0F) * 0.5F;
    const auto drawPeak = [&g](juce::Rectangle<float> card, const juce::String& label, const Peak& peak,
                               const juce::String& unit, int decimals) {
        fillCard(g, card);
        auto body = card.reduced(11.0F, 8.0F);
        g.setColour(colours::faint);
        g.setFont(sectionFont());
        g.drawText(label, body.removeFromTop(16.0F), juce::Justification::centredLeft, false);
        const auto value = peak.value > 0.0 ? juce::String(peak.value, decimals) : utf8("\xe2\x80\x94");
        const auto valueFont = monoFont(23.0F);
        const auto valueWidth = juce::GlyphArrangement::getStringWidth(valueFont, value);
        g.setColour(colours::text);
        g.setFont(valueFont);
        g.drawText(value, body, juce::Justification::centredLeft, false);
        if (peak.value <= 0.0) return;
        g.setColour(colours::muted);
        g.setFont(uiFont(13.0F));
        g.drawText(unit + " @ " + groupedInteger(peak.rpm), body.withTrimmedLeft(valueWidth + 5.0F).withTrimmedTop(4.0F),
                   juce::Justification::centredLeft, true);
    };
    drawPeak(cards.removeFromLeft(cardWidth), "PEAK TORQUE", torque, "Nm", 0);
    drawPeak(cards.removeFromRight(cardWidth), "PEAK POWER", power, "kW", 1);

    drawSectionLabel(g, runsLabel_, "Saved runs");
    if (model_.archivedRuns.empty()) {
        g.setColour(colours::faint);
        g.setFont(uiFont(14.0F));
        g.drawText(utf8("No saved run yet. A finished dyno run appears here."),
                   runsPane_.getBounds().toFloat(), juce::Justification::centredLeft, true);
    }
}

void DynoPanel::resized() {
    auto area = getLocalBounds().reduced(14);
    header_ = area.removeFromTop(26).toFloat();
    area.removeFromTop(10);
    peaks_ = area.removeFromTop(62).toFloat();
    area.removeFromTop(12);

    auto controls = area.removeFromBottom(30);
    area.removeFromBottom(8);
    const auto rows = std::clamp(static_cast<int>(model_.archivedRuns.size()), 1, 4);
    auto list = area.removeFromBottom(rows * RunList::rowHeight);
    area.removeFromBottom(4);
    auto label = area.removeFromBottom(26);
    area.removeFromBottom(10);
    chart_.setBounds(area);

    csvButton_.setBounds(label.removeFromRight(csvButton_.idealWidth()).withSizeKeepingCentre(
        csvButton_.idealWidth(), 24));
    runsLabel_ = label.toFloat();
    runsPane_.setBounds(list);
    runs_.setSize(runsPane_.getMaximumVisibleWidth(), std::max(1, runs_.preferredHeight()));

    deleteButton_.setBounds(controls.removeFromRight(66));
    controls.removeFromRight(6);
    visibilityButton_.setBounds(controls.removeFromRight(58));
    controls.removeFromRight(6);
    colourButton_.setBounds(controls.removeFromRight(66));
    controls.removeFromRight(6);
    runName_.setBounds(controls);
}

} // namespace enginelab::ui
