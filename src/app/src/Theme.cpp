#include <enginelab/app/Theme.hpp>

#include <EngineLabFonts.h>

#include <algorithm>
#include <cmath>
#include <initializer_list>

namespace enginelab::ui {
namespace {
struct EmbeddedTypefaces final {
    juce::Typeface::Ptr regular = load(EngineLabFonts::InterRegular_ttf, EngineLabFonts::InterRegular_ttfSize);
    juce::Typeface::Ptr semiBold = load(EngineLabFonts::InterSemiBold_ttf, EngineLabFonts::InterSemiBold_ttfSize);
    juce::Typeface::Ptr bold = load(EngineLabFonts::InterBold_ttf, EngineLabFonts::InterBold_ttfSize);
    juce::Typeface::Ptr monoMedium = load(EngineLabFonts::JetBrainsMonoMedium_ttf, EngineLabFonts::JetBrainsMonoMedium_ttfSize);
    juce::Typeface::Ptr monoBold = load(EngineLabFonts::JetBrainsMonoBold_ttf, EngineLabFonts::JetBrainsMonoBold_ttfSize);

    static juce::Typeface::Ptr load(const char* data, int size) {
        return juce::Typeface::createSystemTypefaceFor(data, static_cast<std::size_t>(size));
    }
};

[[nodiscard]] const EmbeddedTypefaces& typefaces() {
    static const EmbeddedTypefaces instance;
    return instance;
}

[[nodiscard]] juce::Font fontFor(const juce::Typeface::Ptr& typeface, float height) {
    return juce::Font(juce::FontOptions(typeface).withHeight(height));
}
} // namespace

juce::Font uiFont(float height, bool bold) {
    return fontFor(bold ? typefaces().semiBold : typefaces().regular, height);
}

juce::Font monoFont(float height, bool bold) {
    return fontFor(bold ? typefaces().monoBold : typefaces().monoMedium, height);
}

juce::Font sectionFont() {
    return uiFont(13.0F, true).withExtraKerningFactor(0.08F);
}

juce::String utf8(const char* text) { return juce::String::fromUTF8(text); }

juce::String fixed(double value, int decimals) {
    if (!std::isfinite(value)) return utf8("â");
    // Rounds first so a small negative value prints "0.0", not "-0.0".
    const auto scale = std::pow(10.0, std::max(decimals, 0));
    const auto rounded = std::round(value * scale) / scale;
    if (rounded == 0.0) return decimals > 0 ? juce::String(0.0, decimals) : juce::String("0");
    return decimals > 0 ? juce::String(rounded, decimals) : juce::String(static_cast<long long>(rounded));
}

juce::String keyCapText(const juce::String& keyDescription) {
    const auto lower = keyDescription.toLowerCase();
    if (lower == "spacebar") return "Space";
    if (lower == "cursor up") return utf8("\xe2\x86\x91");
    if (lower == "cursor down") return utf8("\xe2\x86\x93");
    if (lower == "cursor left") return utf8("\xe2\x86\x90");
    if (lower == "cursor right") return utf8("\xe2\x86\x92");
    if (lower == "tab") return "Tab";
    if (lower == "return") return "Enter";
    if (lower == "backspace") return "Bksp";
    return keyDescription.length() == 1 ? keyDescription.toUpperCase() : keyDescription;
}

float keyCapWidth(const juce::String& keyCap) {
    juce::GlyphArrangement glyphs;
    glyphs.addLineOfText(monoFont(12.5F), keyCap, 0.0F, 0.0F);
    return std::max(20.0F, glyphs.getBoundingBox(0, -1, true).getWidth() + 12.0F);
}

void drawKeyCap(juce::Graphics& g, juce::Rectangle<float> area, const juce::String& keyCap,
                bool onAccent) {
    const auto cap = area.withSizeKeepingCentre(keyCapWidth(keyCap), std::min(area.getHeight(), 19.0F));
    g.setColour(onAccent ? juce::Colours::black.withAlpha(0.20F) : juce::Colour(0xff0a0e0d));
    g.fillRoundedRectangle(cap, 5.0F);
    g.setColour(onAccent ? juce::Colours::white.withAlpha(0.25F) : colours::lineStrong);
    g.drawRoundedRectangle(cap.reduced(0.5F), 5.0F, 1.0F);
    g.setColour(onAccent ? juce::Colours::white : colours::muted);
    g.setFont(monoFont(12.5F));
    g.drawText(keyCap, cap, juce::Justification::centred, false);
}

void fillCard(juce::Graphics& g, juce::Rectangle<float> area, float radius, juce::Colour fill,
              juce::Colour border) {
    g.setColour(fill);
    g.fillRoundedRectangle(area, radius);
    g.setColour(border);
    g.drawRoundedRectangle(area.reduced(0.5F), radius, 1.0F);
}

void drawSectionLabel(juce::Graphics& g, juce::Rectangle<float> area, const juce::String& text,
                      juce::Justification justification) {
    g.setColour(colours::faint);
    g.setFont(sectionFont());
    g.drawText(text.toUpperCase(), area, justification, true);
}

LookAndFeel::LookAndFeel()
    : juce::LookAndFeel_V4(juce::LookAndFeel_V4::ColourScheme {
          colours::background, colours::panel, colours::panel,
          colours::lineStrong, colours::text, colours::accent,
          juce::Colours::white, colours::selection, colours::text }) {
    setColour(juce::ResizableWindow::backgroundColourId, colours::background);
    setColour(juce::DocumentWindow::backgroundColourId, colours::background);
    setColour(juce::TextButton::buttonColourId, colours::panel);
    setColour(juce::TextButton::buttonOnColourId, colours::okFill);
    setColour(juce::TextButton::textColourOffId, colours::text);
    setColour(juce::TextButton::textColourOnId, colours::text);
    setColour(juce::ComboBox::backgroundColourId, colours::panel);
    setColour(juce::ComboBox::outlineColourId, colours::lineStrong);
    setColour(juce::ComboBox::arrowColourId, colours::muted);
    setColour(juce::ComboBox::textColourId, colours::text);
    setColour(juce::PopupMenu::backgroundColourId, colours::panelRaised);
    setColour(juce::PopupMenu::textColourId, colours::text);
    setColour(juce::PopupMenu::highlightedBackgroundColourId, colours::selection);
    setColour(juce::PopupMenu::highlightedTextColourId, colours::text);
    setColour(juce::Label::textColourId, colours::text);
    setColour(juce::Slider::backgroundColourId, colours::line);
    setColour(juce::Slider::trackColourId, colours::accent);
    setColour(juce::Slider::thumbColourId, colours::text);
    setColour(juce::Slider::textBoxTextColourId, colours::text);
    setColour(juce::Slider::textBoxBackgroundColourId, colours::panel);
    setColour(juce::Slider::textBoxOutlineColourId, colours::lineStrong);
    setColour(juce::TextEditor::backgroundColourId, colours::panel);
    setColour(juce::TextEditor::outlineColourId, colours::lineStrong);
    setColour(juce::TextEditor::focusedOutlineColourId, colours::accent);
    setColour(juce::TextEditor::textColourId, colours::text);
    setColour(juce::TextEditor::highlightColourId, colours::accent.withAlpha(0.35F));
    setColour(juce::CaretComponent::caretColourId, colours::accent);
    setColour(juce::ScrollBar::thumbColourId, colours::lineStrong);
    setColour(juce::ListBox::backgroundColourId, colours::panel);
    setColour(juce::ListBox::outlineColourId, colours::line);
    setColour(juce::TooltipWindow::backgroundColourId, colours::panelRaised);
    setColour(juce::TooltipWindow::textColourId, colours::text);
    setColour(juce::TooltipWindow::outlineColourId, colours::lineStrong);
    setColour(juce::AlertWindow::backgroundColourId, colours::panelRaised);
    setColour(juce::AlertWindow::textColourId, colours::text);
    setColour(juce::AlertWindow::outlineColourId, colours::lineStrong);
}

juce::Font LookAndFeel::getTextButtonFont(juce::TextButton&, int buttonHeight) {
    return uiFont(std::min(15.0F, static_cast<float>(buttonHeight) * 0.5F), true);
}

void LookAndFeel::drawButtonBackground(juce::Graphics& g, juce::Button& button,
                                       const juce::Colour& backgroundColour, bool highlighted,
                                       bool down) {
    const auto bounds = button.getLocalBounds().toFloat().reduced(0.5F);
    const auto on = button.getToggleState();
    auto fill = on ? button.findColour(juce::TextButton::buttonOnColourId) : backgroundColour;
    if (!button.isEnabled()) fill = fill.withMultipliedAlpha(0.5F);
    else if (down) fill = fill.brighter(0.12F);
    else if (highlighted) fill = fill.brighter(0.06F);
    g.setColour(fill);
    g.fillRoundedRectangle(bounds, 7.0F);
    g.setColour(on ? colours::okBorder : (highlighted ? colours::lineHover : colours::lineStrong));
    g.drawRoundedRectangle(bounds, 7.0F, 1.0F);
}

void LookAndFeel::drawComboBox(juce::Graphics& g, int width, int height, bool, int, int, int, int,
                               juce::ComboBox& box) {
    const auto bounds = juce::Rectangle<float>(0.0F, 0.0F, static_cast<float>(width),
                                               static_cast<float>(height)).reduced(0.5F);
    g.setColour(box.findColour(juce::ComboBox::backgroundColourId)
        .withMultipliedAlpha(box.isEnabled() ? 1.0F : 0.5F));
    g.fillRoundedRectangle(bounds, 7.0F);
    g.setColour(box.isMouseOver(true) && box.isEnabled() ? colours::lineHover
                                                         : box.findColour(juce::ComboBox::outlineColourId));
    g.drawRoundedRectangle(bounds, 7.0F, 1.0F);
    const auto arrowZone = juce::Rectangle<float>(static_cast<float>(width) - 24.0F, 0.0F, 16.0F,
                                                  static_cast<float>(height));
    juce::Path chevron;
    chevron.startNewSubPath(arrowZone.getCentreX() - 4.0F, arrowZone.getCentreY() - 2.0F);
    chevron.lineTo(arrowZone.getCentreX(), arrowZone.getCentreY() + 2.0F);
    chevron.lineTo(arrowZone.getCentreX() + 4.0F, arrowZone.getCentreY() - 2.0F);
    g.setColour(box.findColour(juce::ComboBox::arrowColourId)
        .withMultipliedAlpha(box.isEnabled() ? 1.0F : 0.4F));
    g.strokePath(chevron, juce::PathStrokeType(1.6F, juce::PathStrokeType::curved,
                                               juce::PathStrokeType::rounded));
}

juce::Font LookAndFeel::getComboBoxFont(juce::ComboBox& box) {
    return uiFont(std::min(15.0F, static_cast<float>(box.getHeight()) * 0.5F));
}

void LookAndFeel::positionComboBoxText(juce::ComboBox& box, juce::Label& label) {
    label.setBounds(4, 1, box.getWidth() - 30, box.getHeight() - 2);
    label.setFont(getComboBoxFont(box));
}

juce::Font LookAndFeel::getPopupMenuFont() { return uiFont(15.0F); }

void LookAndFeel::drawPopupMenuBackground(juce::Graphics& g, int width, int height) {
    g.fillAll(colours::panelRaised);
    g.setColour(colours::lineStrong);
    g.drawRect(0, 0, width, height);
}

void LookAndFeel::drawPopupMenuItem(juce::Graphics& g, const juce::Rectangle<int>& area,
                                    bool isSeparator, bool isActive, bool isHighlighted,
                                    bool isTicked, bool hasSubMenu, const juce::String& text,
                                    const juce::String& shortcutKeyText, const juce::Drawable*,
                                    const juce::Colour* textColour) {
    if (isSeparator) {
        g.setColour(colours::line);
        g.fillRect(area.reduced(8, 0).withSizeKeepingCentre(area.getWidth() - 16, 1));
        return;
    }
    auto row = area.toFloat().reduced(4.0F, 1.0F);
    if (isHighlighted && isActive) {
        g.setColour(colours::selection);
        g.fillRoundedRectangle(row, 5.0F);
    }
    auto colour = textColour != nullptr ? *textColour : colours::text;
    if (!isActive) colour = colours::faint;
    auto content = row.reduced(8.0F, 0.0F);
    auto tickArea = content.removeFromLeft(16.0F);
    if (isTicked) {
        g.setColour(colours::accent);
        g.fillEllipse(tickArea.withSizeKeepingCentre(6.0F, 6.0F));
    }
    content.removeFromLeft(4.0F);
    if (hasSubMenu) {
        auto arrow = content.removeFromRight(10.0F);
        juce::Path chevron;
        chevron.startNewSubPath(arrow.getX() + 2.0F, arrow.getCentreY() - 4.0F);
        chevron.lineTo(arrow.getX() + 6.0F, arrow.getCentreY());
        chevron.lineTo(arrow.getX() + 2.0F, arrow.getCentreY() + 4.0F);
        g.setColour(colour);
        g.strokePath(chevron, juce::PathStrokeType(1.5F));
    }
    g.setColour(colour);
    g.setFont(getPopupMenuFont());
    g.drawFittedText(text, content.toNearestInt(), juce::Justification::centredLeft, 1);
    if (shortcutKeyText.isNotEmpty()) {
        const auto cap = keyCapText(shortcutKeyText);
        const auto width = keyCapWidth(cap);
        drawKeyCap(g, content.removeFromRight(width), cap);
    }
}

void LookAndFeel::drawLinearSlider(juce::Graphics& g, int x, int y, int width, int height,
                                   float sliderPos, float minSliderPos, float maxSliderPos,
                                   juce::Slider::SliderStyle style, juce::Slider& slider) {
    const auto area = juce::Rectangle<float>(static_cast<float>(x), static_cast<float>(y),
                                             static_cast<float>(width), static_cast<float>(height));
    const auto alpha = slider.isEnabled() ? 1.0F : 0.45F;
    if (style == juce::Slider::LinearBarVertical) {
        // The throttle: a filled column that rises from the bottom.
        const auto frame = slider.getLocalBounds().toFloat().reduced(0.5F);
        g.setColour(colours::panel);
        g.fillRoundedRectangle(frame, 9.0F);
        const auto fillTop = std::clamp(sliderPos, frame.getY(), frame.getBottom());
        auto fill = frame.withTop(fillTop);
        if (fill.getHeight() > 0.5F) {
            juce::Graphics::ScopedSaveState state(g);
            juce::Path clip;
            clip.addRoundedRectangle(frame, 9.0F);
            g.reduceClipRegion(clip);
            g.setGradientFill(juce::ColourGradient(colours::accentLight.withMultipliedAlpha(alpha),
                0.0F, frame.getY(), colours::accentDeep.withMultipliedAlpha(alpha), 0.0F,
                frame.getBottom(), false));
            g.fillRect(fill);
        }
        g.setColour(slider.isMouseOverOrDragging() ? colours::lineHover : colours::lineStrong);
        g.drawRoundedRectangle(frame, 9.0F, 1.0F);
        return;
    }
    if (!slider.isHorizontal()) {
        juce::LookAndFeel_V4::drawLinearSlider(g, x, y, width, height, sliderPos, minSliderPos,
                                               maxSliderPos, style, slider);
        return;
    }
    const auto track = area.withSizeKeepingCentre(area.getWidth(), 4.0F);
    g.setColour(colours::line);
    g.fillRoundedRectangle(track, 2.0F);
    // Bipolar ranges (trims) fill from zero, unipolar ones from the minimum.
    const auto range = slider.getRange();
    const auto origin = range.getStart() < 0.0 && range.getEnd() > 0.0
        ? static_cast<float>(slider.getPositionOfValue(0.0)) : track.getX();
    const auto left = std::min(origin, sliderPos);
    const auto right = std::max(origin, sliderPos);
    g.setColour(colours::accent.withMultipliedAlpha(alpha));
    g.fillRoundedRectangle(track.withX(left).withWidth(std::max(0.0F, right - left)), 2.0F);
    const auto thumb = juce::Rectangle<float>(13.0F, 13.0F).withCentre({ sliderPos, track.getCentreY() });
    g.setColour(colours::text.withMultipliedAlpha(alpha));
    g.fillEllipse(thumb);
    g.setColour(colours::accent.withMultipliedAlpha(alpha));
    g.drawEllipse(thumb.reduced(1.0F), slider.isMouseOverOrDragging() ? 2.5F : 2.0F);
}

juce::Typeface::Ptr LookAndFeel::getTypefaceForFont(const juce::Font& font) {
    const auto& name = font.getTypefaceName();
    if (name == juce::Font::getDefaultSansSerifFontName())
        return font.isBold() ? typefaces().bold : typefaces().regular;
    if (name == juce::Font::getDefaultMonospacedFontName())
        return font.isBold() ? typefaces().monoBold : typefaces().monoMedium;
    return juce::LookAndFeel_V4::getTypefaceForFont(font);
}

int LookAndFeel::getSliderThumbRadius(juce::Slider& slider) {
    return slider.getSliderStyle() == juce::Slider::LinearBarVertical ? 0 : 7;
}

void LookAndFeel::fillTextEditorBackground(juce::Graphics& g, int width, int height,
                                           juce::TextEditor& editor) {
    g.setColour(editor.findColour(juce::TextEditor::backgroundColourId));
    g.fillRoundedRectangle(0.0F, 0.0F, static_cast<float>(width), static_cast<float>(height), 6.0F);
}

void LookAndFeel::drawTextEditorOutline(juce::Graphics& g, int width, int height,
                                        juce::TextEditor& editor) {
    if (!editor.isEnabled()) return;
    const auto focused = editor.hasKeyboardFocus(true) && !editor.isReadOnly();
    g.setColour(editor.findColour(focused ? juce::TextEditor::focusedOutlineColourId
                                          : juce::TextEditor::outlineColourId));
    g.drawRoundedRectangle(0.5F, 0.5F, static_cast<float>(width) - 1.0F,
                           static_cast<float>(height) - 1.0F, 6.0F, focused ? 1.5F : 1.0F);
}

void LookAndFeel::drawScrollbar(juce::Graphics& g, juce::ScrollBar&, int x, int y, int width,
                                int height, bool vertical, int thumbStart, int thumbSize,
                                bool mouseOver, bool mouseDown) {
    if (thumbSize <= 0) return;
    auto thumb = vertical
        ? juce::Rectangle<int>(x, thumbStart, width, thumbSize)
        : juce::Rectangle<int>(thumbStart, y, thumbSize, height);
    g.setColour(mouseOver || mouseDown ? colours::lineHover : colours::lineStrong);
    g.fillRoundedRectangle(thumb.toFloat().reduced(2.5F), 3.0F);
}

juce::Rectangle<int> LookAndFeel::getTooltipBounds(const juce::String& tipText,
                                                   juce::Point<int> screenPos,
                                                   juce::Rectangle<int> parentArea) {
    juce::TextLayout layout;
    juce::AttributedString text;
    text.append(tipText, uiFont(14.0F), colours::text);
    layout.createLayoutWithBalancedLineLengths(text, 360.0F);
    const auto width = static_cast<int>(layout.getWidth()) + 20;
    const auto height = static_cast<int>(layout.getHeight()) + 14;
    return juce::Rectangle<int>(
        screenPos.x > parentArea.getCentreX() ? screenPos.x - (width + 12) : screenPos.x + 24,
        screenPos.y > parentArea.getCentreY() ? screenPos.y - (height + 6) : screenPos.y + 6,
        width, height).constrainedWithin(parentArea);
}

void LookAndFeel::drawTooltip(juce::Graphics& g, const juce::String& text, int width, int height) {
    const auto bounds = juce::Rectangle<float>(static_cast<float>(width), static_cast<float>(height));
    fillCard(g, bounds, 6.0F, colours::panelRaised, colours::lineStrong);
    juce::AttributedString attributed;
    attributed.append(text, uiFont(14.0F), colours::text);
    juce::TextLayout layout;
    layout.createLayoutWithBalancedLineLengths(attributed, 360.0F);
    layout.draw(g, bounds.reduced(10.0F, 7.0F));
}

} // namespace enginelab::ui
