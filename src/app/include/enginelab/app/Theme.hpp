#pragma once

#include <juce_gui_basics/juce_gui_basics.h>

/** Visual identity of the desktop application: colour tokens, fonts and the
    look-and-feel every window shares. Panels draw with these tokens only, so a
    palette change happens here and nowhere else. */
namespace enginelab::ui {

namespace colours {
inline const juce::Colour background { 0xff0b0f0e };
inline const juce::Colour chrome { 0xff0d1211 };      // top bar and status bar
inline const juce::Colour panel { 0xff111716 };
inline const juce::Colour panelRaised { 0xff161e1c };
inline const juce::Colour line { 0xff222c29 };
inline const juce::Colour lineStrong { 0xff2d3a36 };
inline const juce::Colour lineHover { 0xff3e4f4a };
inline const juce::Colour selection { 0xff24302c };
inline const juce::Colour grid { 0xff1d2624 };
inline const juce::Colour text { 0xffe6ece9 };
inline const juce::Colour muted { 0xff8a9893 };
inline const juce::Colour faint { 0xff5c6a65 };
inline const juce::Colour accent { 0xffef6f3c };
inline const juce::Colour accentDeep { 0xffd85b2b };
inline const juce::Colour accentLight { 0xfff6a15d };
inline const juce::Colour intake { 0xff41b6d7 };
inline const juce::Colour ok { 0xff79b89f };
inline const juce::Colour okFill { 0xff12231d };
inline const juce::Colour okBorder { 0xff2f6b55 };
inline const juce::Colour warn { 0xffffd46a };
inline const juce::Colour warnFill { 0xff231d10 };
inline const juce::Colour critical { 0xffff9a8c };
inline const juce::Colour criticalFill { 0xff2a1513 };
inline const juce::Colour redline { 0xff5c2723 };
} // namespace colours

/** Interface text: Inter Regular, or Inter SemiBold when `bold`. Both are
    embedded in the executable (src/app/resources/fonts). */
[[nodiscard]] juce::Font uiFont(float height, bool bold = false);
/** Figures and key caps: JetBrains Mono Bold, or Medium when not `bold`. */
[[nodiscard]] juce::Font monoFont(float height, bool bold = true);
/** Small upper-case section heading with the prototype's letter spacing. */
[[nodiscard]] juce::Font sectionFont();

/** `const char*` literals are Latin-1 for juce::String; every non-ASCII literal
    goes through this. */
[[nodiscard]] juce::String utf8(const char* text);

/** A number with a fixed count of decimals. juce::String(value, 0) prints
    every significant digit instead of rounding, hence this helper. */
[[nodiscard]] juce::String fixed(double value, int decimals);

/** Short key-cap text for a JUCE key description ("spacebar" -> "Space"). */
[[nodiscard]] juce::String keyCapText(const juce::String& keyDescription);
[[nodiscard]] float keyCapWidth(const juce::String& keyCap);
void drawKeyCap(juce::Graphics&, juce::Rectangle<float> area, const juce::String& keyCap,
                bool onAccent = false);

void fillCard(juce::Graphics&, juce::Rectangle<float> area, float radius = 9.0F,
              juce::Colour fill = colours::panel, juce::Colour border = colours::line);
void drawSectionLabel(juce::Graphics&, juce::Rectangle<float> area, const juce::String& text,
                      juce::Justification = juce::Justification::centredLeft);

/** Look-and-feel installed as the application default, so the secondary
    windows (ECU tuner, EXHAUST PRO, AUDIO HQ) and the alert boxes share it. */
class LookAndFeel final : public juce::LookAndFeel_V4 {
public:
    LookAndFeel();

    juce::Font getTextButtonFont(juce::TextButton&, int buttonHeight) override;
    void drawButtonBackground(juce::Graphics&, juce::Button&, const juce::Colour& backgroundColour,
                              bool highlighted, bool down) override;
    void drawComboBox(juce::Graphics&, int width, int height, bool down, int buttonX, int buttonY,
                      int buttonW, int buttonH, juce::ComboBox&) override;
    juce::Font getComboBoxFont(juce::ComboBox&) override;
    void positionComboBoxText(juce::ComboBox&, juce::Label&) override;
    juce::Font getPopupMenuFont() override;
    void drawPopupMenuBackground(juce::Graphics&, int width, int height) override;
    void drawPopupMenuItem(juce::Graphics&, const juce::Rectangle<int>& area, bool isSeparator,
                           bool isActive, bool isHighlighted, bool isTicked, bool hasSubMenu,
                           const juce::String& text, const juce::String& shortcutKeyText,
                           const juce::Drawable* icon, const juce::Colour* textColour) override;
    void drawLinearSlider(juce::Graphics&, int x, int y, int width, int height, float sliderPos,
                          float minSliderPos, float maxSliderPos, juce::Slider::SliderStyle,
                          juce::Slider&) override;
    int getSliderThumbRadius(juce::Slider&) override;
    void fillTextEditorBackground(juce::Graphics&, int width, int height, juce::TextEditor&) override;
    void drawTextEditorOutline(juce::Graphics&, int width, int height, juce::TextEditor&) override;
    void drawScrollbar(juce::Graphics&, juce::ScrollBar&, int x, int y, int width, int height,
                       bool vertical, int thumbStart, int thumbSize, bool mouseOver,
                       bool mouseDown) override;
    int getDefaultScrollbarWidth() override { return 10; }
    /** Maps JUCE's default sans-serif and monospaced fonts to the embedded
        Inter and JetBrains Mono, so windows that build plain FontOptions
        share the theme's typefaces. */
    juce::Typeface::Ptr getTypefaceForFont(const juce::Font&) override;
    juce::Rectangle<int> getTooltipBounds(const juce::String& tipText, juce::Point<int> screenPos,
                                          juce::Rectangle<int> parentArea) override;
    void drawTooltip(juce::Graphics&, const juce::String& text, int width, int height) override;
};

} // namespace enginelab::ui
