#include <enginelab/app/StatusBar.hpp>

#include <enginelab/app/Theme.hpp>
#include <algorithm>

namespace enginelab::ui {

StatusBar::StatusBar(const DashboardModel& model) : model_(model) { setOpaque(true); }

StatusBar::Content StatusBar::compose() const {
    Content content;
    const auto& diagnostics = model_.diagnostics;
    const auto critical = std::any_of(diagnostics.begin(), diagnostics.end(), [](const Diagnostic& item) {
        return item.severity == DiagnosticSeverity::critical;
    });
    if (diagnostics.empty()) {
        content.fault = utf8("\xe2\x97\x8f  No fault detected");
        content.faultColour = colours::ok;
    } else {
        content.fault = utf8("\xe2\x96\xb2  ") + juce::String::fromUTF8(diagnostics.front().message.c_str());
        if (diagnostics.size() > 1)
            content.fault << "  (+" << juce::String(static_cast<int>(diagnostics.size() - 1)) << ")";
        content.faultColour = critical ? colours::critical : colours::warn;
    }
    const auto& health = model_.health;
    // Below 0.95 the simulation runs in slow motion: controls answer late and
    // users report it as an engine fault, so it has to be visible at a glance.
    content.realtimeColour = health.realtimeFactor < 0.95 ? colours::warn : colours::faint;
    content.items.add(utf8("Physics real-time \xc3\x97") + juce::String(health.realtimeFactor, 2));
    content.items.add(health.paused ? juce::String("Simulation paused")
                                    : utf8("Simulation speed \xc3\x97") + juce::String(health.timeScale, 2));
    if (model_.scriptLive) content.items.add("Script live r" + juce::String(model_.scriptRevision));
    if (model_.voicingReloadCount > 0)
        content.items.add("Voicing live r" + juce::String(model_.voicingReloadCount));
    content.items.add(model_.audio.impulseResponseStatus);
    content.counters = "overruns " + juce::String(health.timingOverruns)
        + utf8("  \xc2\xb7  RT drops ") + juce::String(health.droppedFirings)
        + utf8("  \xc2\xb7  late audio ") + juce::String(health.lateAudioEvents)
        + utf8("  \xc2\xb7  audio queue ") + juce::String(health.droppedPendingAudioEvents);
    return content;
}

void StatusBar::refresh() {
    auto content = compose();
    if (content == content_) return;
    content_ = std::move(content);
    repaint();
}

void StatusBar::paint(juce::Graphics& g) {
    g.fillAll(colours::chrome);
    g.setColour(colours::line);
    g.fillRect(getLocalBounds().removeFromTop(1));
    auto area = getLocalBounds().toFloat().reduced(14.0F, 0.0F).withTrimmedTop(1.0F);

    g.setFont(monoFont(13.0F, false));
    const auto countersWidth = juce::GlyphArrangement::getStringWidth(g.getCurrentFont(), content_.counters);
    g.setColour(colours::faint);
    g.drawText(content_.counters, area.removeFromRight(countersWidth + 4.0F), juce::Justification::centredRight, false);
    area.removeFromRight(18.0F);

    g.setFont(uiFont(13.5F));
    const auto faultWidth = std::min(area.getWidth() * 0.45F,
        juce::GlyphArrangement::getStringWidth(g.getCurrentFont(), content_.fault) + 4.0F);
    g.setColour(content_.faultColour);
    g.drawFittedText(content_.fault, area.removeFromLeft(faultWidth).toNearestInt(),
                     juce::Justification::centredLeft, 1, 0.9F);
    for (int index = 0; index < content_.items.size(); ++index) {
        area.removeFromLeft(18.0F);
        const auto& item = content_.items[index];
        const auto width = juce::GlyphArrangement::getStringWidth(g.getCurrentFont(), item) + 4.0F;
        if (width > area.getWidth()) break;
        g.setColour(index == 0 ? content_.realtimeColour : colours::faint);
        g.drawText(item, area.removeFromLeft(width), juce::Justification::centredLeft, false);
    }
}

} // namespace enginelab::ui
