#include <enginelab/app/ReadoutStrip.hpp>

#include <enginelab/app/Theme.hpp>
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace enginelab::ui {
namespace {
/** What a channel's sparkline plots from one telemetry sample (NaN: a gap),
    and the smallest span its scale may show, so noise stays flat. */
struct Trace final {
    double (*sample)(const EngineState&, double stoichiometric) { nullptr };
    double minimumSpan { 1.0 };
};

struct Readout final {
    juce::String label;
    juce::String value;
    juce::String unit;
    juce::String detail;
    juce::Colour colour { colours::text };
    Trace trace {};
};

constexpr auto noSample = std::numeric_limits<double>::quiet_NaN();

/** The last ten seconds of one channel, scaled to its own range, along the
    cell's foot. One point a pixel column at most. */
void drawSparkline(juce::Graphics& g, juce::Rectangle<float> area, const TelemetryRing& ring, const Trace& trace,
                   double stoichiometric) {
    const auto count = ring.size();
    if (trace.sample == nullptr || count < 2 || area.getWidth() < 8.0F) return;
    auto low = std::numeric_limits<double>::max();
    auto high = std::numeric_limits<double>::lowest();
    for (std::size_t index = 0; index < count; ++index) {
        const auto value = trace.sample(ring.at(index), stoichiometric);
        if (!std::isfinite(value)) continue;
        low = std::min(low, value);
        high = std::max(high, value);
    }
    if (low > high) return;
    if (high - low < trace.minimumSpan) {
        const auto middle = 0.5 * (low + high);
        low = middle - 0.5 * trace.minimumSpan;
        high = middle + 0.5 * trace.minimumSpan;
    }
    // The ring's full ten seconds span the width: a short history starts on the right.
    const auto capacity = static_cast<double>(TelemetryRing::capacity - 1);
    const auto first = static_cast<double>(TelemetryRing::capacity - count);
    const auto step = std::max<std::size_t>(1, static_cast<std::size_t>(std::floor(
        static_cast<double>(TelemetryRing::capacity) / static_cast<double>(area.getWidth()))));
    juce::Path path;
    auto drawing = false;
    for (std::size_t index = 0; index < count; index += step) {
        const auto value = trace.sample(ring.at(index), stoichiometric);
        if (!std::isfinite(value)) {
            drawing = false;
            continue;
        }
        const auto x = area.getX() + area.getWidth() * static_cast<float>((first + static_cast<double>(index)) / capacity);
        const auto y = area.getBottom() - area.getHeight() * static_cast<float>((value - low) / (high - low));
        if (drawing) path.lineTo(x, y);
        else path.startNewSubPath(x, y);
        drawing = true;
    }
    g.setColour(colours::label.withAlpha(0.75F));
    g.strokePath(path, juce::PathStrokeType(1.0F));
}

void drawReadout(juce::Graphics& g, juce::Rectangle<float> cell, const Readout& readout, const TelemetryRing& ring,
                 double stoichiometric) {
    auto body = cell.reduced(12.0F, 0.0F).withTrimmedTop(9.0F).withTrimmedBottom(8.0F);
    g.setColour(colours::label);
    g.setFont(sectionFont());
    g.drawFittedText(readout.label, body.removeFromTop(15.0F).toNearestInt(), juce::Justification::centredLeft, 1,
                     0.8F);
    body.removeFromTop(2.0F);
    auto valueRow = body.removeFromTop(28.0F);
    // The value and its unit shrink together to the cell's width rather than
    // lose a digit (audit APP-10).
    const auto natural = juce::GlyphArrangement::getStringWidth(monoFont(24.0F), readout.value)
        + (readout.unit.isEmpty() ? 0.0F : 4.0F + juce::GlyphArrangement::getStringWidth(uiFont(14.0F), readout.unit));
    const auto scale = natural > valueRow.getWidth() && natural > 0.0F
        ? std::max(0.5F, valueRow.getWidth() / natural) : 1.0F;
    const auto valueFont = monoFont(24.0F * scale);
    const auto valueWidth = juce::GlyphArrangement::getStringWidth(valueFont, readout.value);
    g.setColour(readout.colour);
    g.setFont(valueFont);
    g.drawText(readout.value, valueRow, juce::Justification::centredLeft, false);
    if (readout.unit.isNotEmpty()) {
        g.setColour(colours::label);
        g.setFont(uiFont(14.0F * scale));
        g.drawText(readout.unit, valueRow.withTrimmedLeft(valueWidth + 4.0F).withTrimmedTop(5.0F * scale),
                   juce::Justification::centredLeft, true);
    }
    g.setColour(colours::muted);
    g.setFont(uiFont(13.5F));
    g.drawFittedText(readout.detail, body.removeFromTop(16.0F).toNearestInt(), juce::Justification::centredLeft, 1,
                     0.8F);
    body.removeFromTop(3.0F);
    drawSparkline(g, body, ring, readout.trace, stoichiometric);
}

/** The tape's scale step, in rpm: at most eleven numbers along it. */
[[nodiscard]] double scaleStep(double scaleMaximum) {
    for (const auto step : { 500.0, 1000.0, 2000.0, 2500.0, 5000.0 })
        if (scaleMaximum / step <= 10.0) return step;
    return 10000.0;
}
} // namespace

ReadoutStrip::ReadoutStrip(const DashboardModel& model) : model_(model) {
    setOpaque(true);
}

void ReadoutStrip::drawTachometer(juce::Graphics& g, juce::Rectangle<float> area) const {
    const auto& state = model_.state;
    const auto redline = std::max(1.0, model_.redlineRpm > 0.0 ? model_.redlineRpm : model_.config.redlineRpm);
    auto body = area.withTrimmedTop(9.0F).withTrimmedBottom(8.0F);
    auto top = body.removeFromTop(38.0F);

    // Right of the figures: the VTEC light over the redline. The light is lit
    // while the valves run the high cam, an outline on the low cam, struck
    // through when the ECU holds it off.
    const auto redlineText = "redline " + groupedInteger(redline);
    const auto metaFont = monoFont(11.5F, false);
    auto meta = top.removeFromRight(juce::GlyphArrangement::getStringWidth(metaFont, redlineText) + 2.0F);
    if (vtecCamsFitted(model_.config)) {
        const auto light = meta.removeFromTop(17.0F).removeFromRight(40.0F);
        const auto engaged = state.vtecHighCylinderMask != 0U;
        if (engaged) {
            g.setColour(colours::warn);
            g.fillRoundedRectangle(light, cornerRadius);
        } else {
            g.setColour(state.vtecEnabled ? colours::lineHover : colours::line);
            g.drawRoundedRectangle(light.reduced(0.5F), cornerRadius, 1.0F);
        }
        g.setColour(engaged ? colours::ink : state.vtecEnabled ? colours::label : colours::faint);
        g.setFont(sectionFont());
        g.drawText("VTEC", light, juce::Justification::centred, false);
        if (!state.vtecEnabled)
            g.drawLine(light.getX() + 3.0F, light.getCentreY(), light.getRight() - 3.0F, light.getCentreY(), 1.0F);
    }
    g.setColour(colours::label);
    g.setFont(metaFont);
    g.drawText(redlineText, meta.removeFromBottom(16.0F), juce::Justification::centredRight, false);

    // The engine speed, sized for five digits and a separator ("10,500") so
    // the font does not jump as the speed crosses 1,000 or 10,000.
    const auto unitFont = uiFont(14.0F);
    const auto unitWidth = juce::GlyphArrangement::getStringWidth(unitFont, "rpm") + 6.0F;
    const auto room = top.getWidth() - unitWidth - 8.0F;
    const auto natural = juce::GlyphArrangement::getStringWidth(monoFont(34.0F), "00,000");
    const auto speedScale = natural > room ? std::max(0.5F, room / natural) : 1.0F;
    const auto speedFont = monoFont(34.0F * speedScale);
    const auto speed = groupedInteger(state.rpm);
    const auto speedWidth = juce::GlyphArrangement::getStringWidth(speedFont, speed);
    g.setColour(state.rpm > redline ? colours::critical : colours::text);
    g.setFont(speedFont);
    g.drawText(speed, top, juce::Justification::centredLeft, false);
    g.setColour(colours::label);
    g.setFont(unitFont);
    g.drawText("rpm", top.withTrimmedLeft(speedWidth + 6.0F).withTrimmedTop(8.0F * speedScale),
               juce::Justification::centredLeft, false);

    // The tape: forty segments to a tenth over the redline; the last tenth
    // under it is the shift band, past it the red.
    body.removeFromTop(7.0F);
    const auto tape = body.removeFromTop(15.0F);
    constexpr auto segments = 40;
    constexpr auto gap = 2.0F;
    const auto scaleMaximum = std::max(500.0, std::ceil(redline * 1.1 / 500.0) * 500.0);
    const auto segmentRpm = scaleMaximum / segments;
    const auto segmentWidth = (tape.getWidth() - gap * (segments - 1)) / static_cast<float>(segments);
    for (auto index = 0; index < segments; ++index) {
        const auto from = segmentRpm * index;
        const auto lit = state.rpm >= from + 0.5 * segmentRpm;
        const auto red = from >= redline;
        const auto shift = !red && from >= 0.9 * redline;
        g.setColour(lit ? (red ? colours::critical : shift ? colours::accent : colours::text)
                        : (red ? colours::redline : colours::unlit));
        g.fillRect(tape.getX() + static_cast<float>(index) * (segmentWidth + gap), tape.getY(), segmentWidth,
                   tape.getHeight());
    }

    // Its scale, in thousands.
    body.removeFromTop(3.0F);
    const auto scaleRow = body.removeFromTop(13.0F);
    const auto step = scaleStep(scaleMaximum);
    g.setColour(colours::label);
    g.setFont(monoFont(11.0F, false));
    for (auto mark = 0.0; mark <= scaleMaximum + 0.5; mark += step) {
        const auto thousands = mark / 1000.0;
        const auto text = std::abs(thousands - std::round(thousands)) < 1.0e-6 ? juce::String(std::lround(thousands))
                                                                               : juce::String(thousands, 1);
        const auto x = scaleRow.getX() + scaleRow.getWidth() * static_cast<float>(mark / scaleMaximum);
        const auto width = juce::GlyphArrangement::getStringWidth(monoFont(11.0F, false), text) + 2.0F;
        const auto left = std::clamp(x - width * 0.5F, scaleRow.getX(), scaleRow.getRight() - width);
        g.drawText(text, juce::Rectangle<float>(left, scaleRow.getY(), width, scaleRow.getHeight()),
                   juce::Justification::centred, false);
    }
}

void ReadoutStrip::paint(juce::Graphics& g) {
    const auto& state = model_.state;
    const auto& config = model_.config;
    g.fillAll(colours::panel);
    g.setColour(colours::line);
    g.fillRect(getLocalBounds().removeFromTop(1));
    auto area = getLocalBounds().toFloat().withTrimmedTop(1.0F);
    const auto tachWidth = std::clamp(area.getWidth() * 0.30F, 230.0F, 320.0F);
    drawTachometer(g, area.removeFromLeft(tachWidth).reduced(16.0F, 0.0F));

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
          "inst. " + unit.text(Quantity::torque, state.torqueNm, 0), colours::text,
          { [](const EngineState& sample, double) { return sample.cycleAveragedTorqueNm; }, 20.0 } },
        { "Power", unit.value(Quantity::power, state.cycleAveragedPowerKw), unit.symbol(Quantity::power),
          alternate.text(Quantity::power, state.cycleAveragedPowerKw), colours::text,
          { [](const EngineState& sample, double) { return sample.cycleAveragedPowerKw; }, 10.0 } },
        { "Manifold", unit.value(Quantity::pressure, state.manifoldPressureKpa), unit.symbol(Quantity::pressure),
          config.forcedInduction.enabled
              ? "boost " + juce::String(state.boostPressureRatio, 2) + utf8("\xc3\x97")
              : "VE " + fixed(state.volumetricEfficiency * 100.0, 0) + " %",
          colours::text,
          { [](const EngineState& sample, double) { return sample.manifoldPressureKpa; }, 10.0 } },
        { "Air / fuel", state.airFuelRatioValid ? juce::String(state.airFuelRatio, 1) : utf8("\xe2\x80\x94"), {},
          !state.airFuelRatioValid ? utf8("cut / sync")
              : utf8("\xce\xbb ") + juce::String(state.airFuelRatio / std::max(1.0, stoichiometric), 2)
                  + utf8("  \xc2\xb7  ") + (diesel ? "limit " : "target ")
                  + juce::String(state.targetAirFuelRatio, 1),
          mixtureOff ? colours::warn : colours::text,
          { [](const EngineState& sample, double stoichiometricRatio) {
                return sample.airFuelRatioValid ? sample.airFuelRatio / std::max(1.0, stoichiometricRatio) : noSample;
            },
            0.1 } },
        { "Exhaust gas", unit.value(Quantity::temperature, state.exhaustTemperatureC), unit.symbol(Quantity::temperature),
          "runner " + unit.text(Quantity::pressure, state.exhaustRunnerPressureKpa),
          state.exhaustTemperatureC > 950.0 ? colours::warn : colours::text,
          { [](const EngineState& sample, double) { return sample.exhaustTemperatureC; }, 50.0 } },
        { "Coolant", unit.value(Quantity::temperature, state.coolantTemperatureC), unit.symbol(Quantity::temperature),
          "oil " + unit.pressureDifference(state.oilPressureKpa, unit.preferences.of(Quantity::pressure).id == "kpa" ? 0 : 1),
          state.coolantTemperatureC > 110.0 ? colours::warn : colours::text,
          { [](const EngineState& sample, double) { return sample.coolantTemperatureC; }, 5.0 } },
    }};
    // Under 100 px a cell cuts its figures: a narrow window drops the coolant,
    // then the exhaust gas (both stay on the Telemetry tab).
    std::size_t shown = readouts.size();
    while (shown > 4 && (area.getWidth() - static_cast<float>(shown)) / static_cast<float>(shown) < 100.0F) --shown;
    const auto cellWidth = (area.getWidth() - static_cast<float>(shown)) / static_cast<float>(shown);
    for (std::size_t index = 0; index < shown; ++index) {
        g.setColour(colours::line);
        g.fillRect(area.removeFromLeft(1.0F));
        drawReadout(g, area.removeFromLeft(cellWidth), readouts[index], model_.telemetry, stoichiometric);
    }
}

} // namespace enginelab::ui
