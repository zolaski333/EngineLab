#include <enginelab/app/ReadoutStrip.hpp>

#include <enginelab/app/Theme.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <numbers>

namespace enginelab::ui {
namespace {
struct Readout final {
    juce::String label;
    juce::String value;
    juce::String unit;
    juce::String detail;
    juce::Colour colour { colours::text };
};

void drawReadout(juce::Graphics& g, juce::Rectangle<float> cell, const Readout& readout) {
    auto body = cell.reduced(13.0F, 11.0F);
    g.setColour(colours::faint);
    g.setFont(sectionFont());
    g.drawFittedText(readout.label.toUpperCase(), body.removeFromTop(16.0F).toNearestInt(),
                     juce::Justification::centredLeft, 1, 0.8F);
    body.removeFromTop(3.0F);
    auto valueRow = body.removeFromTop(30.0F);
    const auto valueFont = monoFont(26.0F);
    const auto valueWidth = juce::GlyphArrangement::getStringWidth(valueFont, readout.value);
    g.setColour(readout.colour);
    g.setFont(valueFont);
    g.drawText(readout.value, valueRow, juce::Justification::centredLeft, false);
    if (readout.unit.isNotEmpty()) {
        g.setColour(colours::muted);
        g.setFont(uiFont(14.0F));
        g.drawText(readout.unit, valueRow.withTrimmedLeft(valueWidth + 4.0F).withTrimmedTop(6.0F),
                   juce::Justification::centredLeft, true);
    }
    g.setColour(colours::muted);
    g.setFont(uiFont(13.5F));
    g.drawFittedText(readout.detail, body.toNearestInt(), juce::Justification::topLeft, 1, 0.8F);
}
} // namespace

ReadoutStrip::ReadoutStrip(const DashboardModel& model) : model_(model) {
    setOpaque(true);
}

void ReadoutStrip::drawTachometer(juce::Graphics& g, juce::Rectangle<float> area) const {
    const auto& state = model_.state;
    const auto redline = std::max(1.0, model_.config.redlineRpm);
    // Angles are clockwise from twelve o'clock: the arc spans 252 degrees
    // across the top, and the last 1/11 of the scale (above redline) is red.
    constexpr auto start = std::numbers::pi_v<float> * 1.3F;
    constexpr auto end = std::numbers::pi_v<float> * 2.7F;
    // The arc's ends sit cos(0.3 pi) of a radius below the centre: size the
    // radius so the whole stroke fits the cell height.
    const auto gauge = area.removeFromLeft(area.getHeight() * 1.2F).reduced(2.0F, 12.0F);
    const auto strokeWidth = std::max(4.0F, gauge.getHeight() * 0.09F);
    const auto radius = (gauge.getHeight() - strokeWidth)
                        / (1.0F + std::cos(std::numbers::pi_v<float> * 0.3F));
    const auto centre = juce::Point<float>(gauge.getCentreX(), gauge.getY() + strokeWidth * 0.5F + radius);
    const auto arc = [&](float from, float to) {
        juce::Path path;
        path.addCentredArc(centre.x, centre.y, radius, radius, 0.0F, from, to, true);
        return path;
    };
    const juce::PathStrokeType roundStroke(strokeWidth, juce::PathStrokeType::curved,
                                           juce::PathStrokeType::rounded);
    g.setColour(colours::line);
    g.strokePath(arc(start, end), roundStroke);
    const auto redStart = start + (end - start) / 1.1F;
    g.setColour(colours::redline);
    g.strokePath(arc(redStart, end), juce::PathStrokeType(strokeWidth));
    const auto fraction = static_cast<float>(std::clamp(state.rpm / (redline * 1.1), 0.0, 1.0));
    if (fraction > 0.001F) {
        g.setGradientFill(juce::ColourGradient(colours::accentDeep, gauge.getX(), centre.y,
                                               colours::accentLight, gauge.getRight(), centre.y, false));
        g.strokePath(arc(start, start + (end - start) * fraction), roundStroke);
    }

    auto text = area.withTrimmedLeft(10.0F).reduced(0.0F, 11.0F);
    g.setColour(colours::faint);
    g.setFont(sectionFont());
    g.drawText("ENGINE SPEED", text.removeFromTop(16.0F), juce::Justification::centredLeft, true);
    g.setColour(state.rpm > redline ? colours::critical : colours::text);
    g.setFont(monoFont(34.0F));
    g.drawText(groupedInteger(state.rpm), text.removeFromTop(36.0F), juce::Justification::centredLeft, false);
    g.setColour(colours::muted);
    g.setFont(uiFont(13.5F));
    g.drawFittedText(utf8("rpm  \xc2\xb7  redline ") + groupedInteger(redline),
                     text.toNearestInt(), juce::Justification::topLeft, 1, 0.8F);
}

void ReadoutStrip::paint(juce::Graphics& g) {
    const auto& state = model_.state;
    const auto& config = model_.config;
    g.fillAll(colours::line);
    auto area = getLocalBounds().toFloat().withTrimmedTop(1.0F);
    const auto tachWidth = std::clamp(area.getWidth() * 0.28F, 200.0F, 260.0F);
    auto tachCell = area.removeFromLeft(tachWidth);
    g.setColour(colours::background);
    g.fillRect(tachCell);
    drawTachometer(g, tachCell.reduced(14.0F, 0.0F));

    const auto diesel = config.fuel == FuelType::diesel;
    const auto mixtureOff = state.airFuelRatioValid && !diesel
        && std::abs(state.airFuelRatio - state.targetAirFuelRatio) > 1.0;
    const std::array<Readout, 6> readouts {{
        { "Torque", fixed(state.cycleAveragedTorqueNm, 0), "Nm",
          "inst. " + fixed(state.torqueNm, 0) + " Nm" },
        { "Power", fixed(state.cycleAveragedPowerKw, 1), "kW",
          fixed(state.cycleAveragedPowerKw / 0.745700, 1) + " hp" },
        { "Manifold", fixed(state.manifoldPressureKpa, 0), "kPa",
          config.forcedInduction.enabled
              ? "boost " + juce::String(state.boostPressureRatio, 2) + utf8("\xc3\x97")
              : "VE " + fixed(state.volumetricEfficiency * 100.0, 0) + " %" },
        { "Air / fuel", state.airFuelRatioValid ? juce::String(state.airFuelRatio, 1) : utf8("\xe2\x80\x94"), {},
          !state.airFuelRatioValid ? utf8("cut / sync")
              : (diesel ? "smoke limit " : "target ") + juce::String(state.targetAirFuelRatio, 1),
          mixtureOff ? colours::warn : colours::text },
        { "Exhaust gas", fixed(state.exhaustTemperatureC, 0), utf8("\xc2\xb0""C"),
          "runner " + fixed(state.exhaustRunnerPressureKpa, 0) + " kPa",
          state.exhaustTemperatureC > 950.0 ? colours::warn : colours::text },
        { "Coolant", fixed(state.coolantTemperatureC, 0), utf8("\xc2\xb0""C"),
          "oil " + juce::String(state.oilPressureKpa / 100.0, 1) + " bar",
          state.coolantTemperatureC > 110.0 ? colours::warn : colours::text },
    }};
    const auto cellWidth = (area.getWidth() - static_cast<float>(readouts.size())) / static_cast<float>(readouts.size());
    for (const auto& readout : readouts) {
        area.removeFromLeft(1.0F);
        const auto cell = area.removeFromLeft(cellWidth);
        g.setColour(colours::background);
        g.fillRect(cell);
        drawReadout(g, cell, readout);
    }
}

} // namespace enginelab::ui
