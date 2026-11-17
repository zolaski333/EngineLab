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
    g.setColour(colours::label);
    g.setFont(sectionFont());
    g.drawFittedText(readout.label.toUpperCase(), body.removeFromTop(16.0F).toNearestInt(),
                     juce::Justification::centredLeft, 1, 0.8F);
    body.removeFromTop(3.0F);
    auto valueRow = body.removeFromTop(30.0F);
    // The value and its unit shrink together to the cell's width rather than
    // lose a digit (audit APP-10).
    const auto natural = juce::GlyphArrangement::getStringWidth(monoFont(26.0F), readout.value)
        + (readout.unit.isEmpty() ? 0.0F : 4.0F + juce::GlyphArrangement::getStringWidth(uiFont(14.0F), readout.unit));
    const auto scale = natural > valueRow.getWidth() && natural > 0.0F
        ? std::max(0.5F, valueRow.getWidth() / natural) : 1.0F;
    const auto valueFont = monoFont(26.0F * scale);
    const auto valueWidth = juce::GlyphArrangement::getStringWidth(valueFont, readout.value);
    g.setColour(readout.colour);
    g.setFont(valueFont);
    g.drawText(readout.value, valueRow, juce::Justification::centredLeft, false);
    if (readout.unit.isNotEmpty()) {
        g.setColour(colours::muted);
        g.setFont(uiFont(14.0F * scale));
        g.drawText(readout.unit, valueRow.withTrimmedLeft(valueWidth + 4.0F).withTrimmedTop(6.0F * scale),
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
    const auto redline = std::max(1.0, model_.redlineRpm > 0.0 ? model_.redlineRpm : model_.config.redlineRpm);
    // Angles are clockwise from twelve o'clock: the arc spans 252 degrees
    // across the top, and the last 1/11 of the scale (above redline) is red.
    constexpr auto start = std::numbers::pi_v<float> * 1.3F;
    constexpr auto end = std::numbers::pi_v<float> * 2.7F;
    // The arc's ends sit cos(0.3 pi) of a radius below the centre: size the
    // radius so the whole stroke fits the cell height.
    // A narrow strip keeps the speed's digits before the gauge's size.
    const auto gauge = area.removeFromLeft(std::min(area.getHeight() * 1.2F, area.getWidth() * 0.47F))
                           .reduced(2.0F, 12.0F);
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
    auto heading = text.removeFromTop(16.0F);
    // The VTEC light, on an engine with VTEC cams: lit while the valves run
    // the high cam, an outline on the low cam, struck through when the ECU
    // holds it off.
    if (vtecCamsFitted(model_.config)) {
        const auto light = heading.removeFromRight(40.0F).reduced(0.0F, 1.0F);
        const auto engaged = state.vtecHighCylinderMask != 0U;
        if (engaged) {
            g.setColour(colours::warn);
            g.fillRoundedRectangle(light, 3.0F);
        } else {
            g.setColour(state.vtecEnabled ? colours::muted : colours::faint);
            g.drawRoundedRectangle(light, 3.0F, 1.0F);
        }
        g.setColour(engaged ? colours::background : state.vtecEnabled ? colours::muted : colours::faint);
        g.setFont(sectionFont());
        g.drawText("VTEC", light, juce::Justification::centred, false);
        if (!state.vtecEnabled)
            g.drawLine(light.getX() + 3.0F, light.getCentreY(), light.getRight() - 3.0F, light.getCentreY(), 1.0F);
    }
    g.setColour(colours::label);
    g.setFont(sectionFont());
    // Beside the VTEC light, or in a narrow strip, the heading is the unit.
    const auto narrow = juce::GlyphArrangement::getStringWidth(sectionFont(), "ENGINE SPEED") > heading.getWidth();
    g.drawText(narrow ? "RPM" : "ENGINE SPEED", heading, juce::Justification::centredLeft, true);
    g.setColour(state.rpm > redline ? colours::critical : colours::text);
    const auto speed = groupedInteger(state.rpm);
    // Sized for five digits and a separator ("10,500"), so the font does not
    // jump as the speed crosses 1,000 or 10,000.
    const auto speedNatural = juce::GlyphArrangement::getStringWidth(monoFont(34.0F), "00,000");
    const auto speedScale = speedNatural > text.getWidth() ? std::max(0.5F, text.getWidth() / speedNatural) : 1.0F;
    g.setFont(monoFont(34.0F * speedScale));
    g.drawText(speed, text.removeFromTop(36.0F), juce::Justification::centredLeft, false);
    g.setColour(colours::muted);
    g.setFont(uiFont(13.5F));
    g.drawFittedText((narrow ? juce::String {} : utf8("rpm  \xc2\xb7  ")) + "redline " + groupedInteger(redline),
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
    // One petrol AFR off target, in lambda: the same warning on E85 or methanol.
    const auto stoichiometric = config.fuelProperties.stoichiometricAirFuelRatio;
    const auto mixtureOff = state.airFuelRatioValid && !diesel
        && std::abs(state.airFuelRatio - state.targetAirFuelRatio) > stoichiometric / 14.7;
    using units::Quantity;
    const auto& unit = model_.units;
    // The second power figure is the other horsepower-or-kilowatt reading.
    auto alternate = unit;
    alternate.preferences.select(Quantity::power, unit.preferences.of(Quantity::power).id == "kw" ? "hp" : "kw");
    const std::array<Readout, 6> readouts {{
        { "Torque", unit.value(Quantity::torque, state.cycleAveragedTorqueNm, 0), unit.symbol(Quantity::torque),
          "inst. " + unit.text(Quantity::torque, state.torqueNm, 0) },
        { "Power", unit.value(Quantity::power, state.cycleAveragedPowerKw), unit.symbol(Quantity::power),
          alternate.text(Quantity::power, state.cycleAveragedPowerKw) },
        { "Manifold", unit.value(Quantity::pressure, state.manifoldPressureKpa), unit.symbol(Quantity::pressure),
          config.forcedInduction.enabled
              ? "boost " + juce::String(state.boostPressureRatio, 2) + utf8("\xc3\x97")
              : "VE " + fixed(state.volumetricEfficiency * 100.0, 0) + " %" },
        { "Air / fuel", state.airFuelRatioValid ? juce::String(state.airFuelRatio, 1) : utf8("\xe2\x80\x94"), {},
          !state.airFuelRatioValid ? utf8("cut / sync")
              : utf8("\xce\xbb ") + juce::String(state.airFuelRatio / std::max(1.0, stoichiometric), 2)
                  + utf8("  \xc2\xb7  ") + (diesel ? "limit " : "target ")
                  + juce::String(state.targetAirFuelRatio, 1),
          mixtureOff ? colours::warn : colours::text },
        { "Exhaust gas", unit.value(Quantity::temperature, state.exhaustTemperatureC), unit.symbol(Quantity::temperature),
          "runner " + unit.text(Quantity::pressure, state.exhaustRunnerPressureKpa),
          state.exhaustTemperatureC > 950.0 ? colours::warn : colours::text },
        { "Coolant", unit.value(Quantity::temperature, state.coolantTemperatureC), unit.symbol(Quantity::temperature),
          "oil " + unit.pressureDifference(state.oilPressureKpa, unit.preferences.of(Quantity::pressure).id == "kpa" ? 0 : 1),
          state.coolantTemperatureC > 110.0 ? colours::warn : colours::text },
    }};
    // Under 100 px a cell cuts its figures: a narrow window drops the coolant,
    // then the exhaust gas (both stay on the Telemetry tab).
    std::size_t shown = readouts.size();
    while (shown > 4 && (area.getWidth() - static_cast<float>(shown)) / static_cast<float>(shown) < 100.0F) --shown;
    const auto cellWidth = (area.getWidth() - static_cast<float>(shown)) / static_cast<float>(shown);
    for (std::size_t index = 0; index < shown; ++index) {
        const auto& readout = readouts[index];
        area.removeFromLeft(1.0F);
        const auto cell = area.removeFromLeft(cellWidth);
        g.setColour(colours::background);
        g.fillRect(cell);
        drawReadout(g, cell, readout);
    }
}

} // namespace enginelab::ui
