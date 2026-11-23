#pragma once

#include <juce_gui_basics/juce_gui_basics.h>

#include <functional>

/** Visual identity of the desktop application ("Test cell", 2026-11-23):
    colour tokens, fonts and the look-and-feel every window shares. Panels draw
    with these tokens only, so a palette change happens here and nowhere else.

    Colour is data. The chrome is graphite and nearly monochrome; colour
    appears only for gas pressure (amber above ambient, violet below), the
    intake charge (cyan), heat and alarms (red), and lamps that are lit.
    Corners are 2 px hairline instruments, never rounded cards. */
namespace enginelab::ui {

/** The corner radius of every frame, button, field and panel. */
inline constexpr float cornerRadius = 2.0F;

namespace colours {
inline const juce::Colour background { 0xff0d0e0f };
inline const juce::Colour chrome { 0xff101113 };      // top bar and status bar
inline const juce::Colour panel { 0xff121315 };
inline const juce::Colour panelRaised { 0xff17181a };
/** Recessed wells: a fader's slot, a plot's ground. */
inline const juce::Colour well { 0xff0a0b0c };
inline const juce::Colour line { 0xff232528 };
inline const juce::Colour lineStrong { 0xff303337 };
inline const juce::Colour lineHover { 0xff44484d };
inline const juce::Colour selection { 0xff24262a };
inline const juce::Colour grid { 0xff1c1e21 };
inline const juce::Colour text { 0xffe9e7e2 };
inline const juce::Colour muted { 0xffb3b0a9 };
/** Decoration only (ticks, disabled glyphs): below 4.5:1 on purpose. */
inline const juce::Colour faint { 0xff6b6964 };
// Section headings and readout labels: small text the user reads, so at least
// 4.5:1 on every panel fill (WCAG 2.1 1.4.3; audit RT-03; checked by
// EngineLab.AppLayout).
inline const juce::Colour label { 0xff939089 };
/** An unlit lamp or segment. */
inline const juce::Colour unlit { 0xff2a2c2f };
/** Text on a lit lamp or an inverted (selected) control. */
inline const juce::Colour ink { 0xff0d0e0f };
/** Data: gas above ambient, boost, the live cursor and operating point. */
inline const juce::Colour accent { 0xfff0a43c };
inline const juce::Colour accentDeep { 0xffc98428 };
/** Data: a pressure peak. */
inline const juce::Colour accentLight { 0xffffd978 };
/** Data: gas below ambient. */
inline const juce::Colour pressureLow { 0xff9a86ff };
/** Data: the intake charge. */
inline const juce::Colour intake { 0xff4fb6dc };
/** Running, a gain. */
inline const juce::Colour ok { 0xff8fc79c };
inline const juce::Colour okFill { 0xff142019 };
inline const juce::Colour okBorder { 0xff3d6b4c };
inline const juce::Colour warn { 0xfff0a43c };
inline const juce::Colour warnFill { 0xff241b0e };
/** Heat, knock, a fault. */
inline const juce::Colour critical { 0xffff6a4a };
inline const juce::Colour criticalFill { 0xff2a1512 };
inline const juce::Colour redline { 0xff4a211b };
} // namespace colours

/** Interface text: Barlow Semi Condensed Regular, or SemiBold when `bold`
    (src/app/resources/fonts, embedded). Characters Barlow lacks (lambda, arrows,
    the ellipsis, triangles, dots) come from Inter, embedded as the fallback. */
[[nodiscard]] juce::Font uiFont(float height, bool bold = false);
/** Barlow Semi Condensed Medium: buttons and tabs. */
[[nodiscard]] juce::Font uiMediumFont(float height);
/** Figures and key caps: IBM Plex Mono Medium, or Regular when not `bold`.
    Characters Plex lacks come from JetBrains Mono, embedded as the fallback. */
[[nodiscard]] juce::Font monoFont(float height, bool bold = true);
/** A section heading: small, semi-bold, in the label colour, sentence case. */
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

void fillCard(juce::Graphics&, juce::Rectangle<float> area, float radius = cornerRadius,
              juce::Colour fill = colours::panel, juce::Colour border = colours::line);
void drawSectionLabel(juce::Graphics&, juce::Rectangle<float> area, const juce::String& text,
                      juce::Justification = juce::Justification::centredLeft);

/** WCAG 2.1 contrast ratio of two opaque colours (1 to 21). */
[[nodiscard]] double contrastRatio(juce::Colour, juce::Colour);
/** The typeface names the text fonts fall back to for characters they lack. */
inline constexpr const char* symbolsSans = "EngineLab Symbols Sans";
inline constexpr const char* symbolsMono = "EngineLab Symbols Mono";

/** The window `component` is drawn in (its top-level component), or null
    while it is in no window on screen. Alert boxes and dialogs are drawn
    inside it when they fit, so a capture of that one window (a screen
    recorder's window capture) shows them; otherwise they are desktop windows,
    as before. */
[[nodiscard]] juce::Component* windowOf(juce::Component* component);
/** Marks `view` as drawn by a native child window (the 3-D view's OpenGL
    surface): on Windows nothing its window draws itself shows over it, so a
    box over it would be hidden. `drawn` false clears the mark. */
void markNativeView(juce::Component& view, bool drawn);
/** An alert box with an OK button, drawn inside the window of `host` when it
    fits there (placeInWindow). */
void showMessage(juce::Component* host, juce::MessageBoxIconType, const juce::String& title,
                 const juce::String& message);
/** Two buttons, drawn inside the window of `host` when they fit; `onResult`
    gets true for the first (`accept`). */
void showOkCancel(juce::Component* host, juce::MessageBoxIconType, const juce::String& title,
                  const juce::String& message, const juce::String& accept, const juce::String& cancel,
                  std::function<void(bool)> onResult);
/** Moves a dialog (an AlertWindow, one built by hand with editors) inside the
    window of `host`, centred, before it enters its modal state -- only when
    it fits whole inside that window and covers no native view
    (markNativeView). Otherwise it stays a desktop window, centred over the
    host's window: a box shrunk to a small window or drawn under the 3-D view
    is unreadable. Returns true when it was moved inside. */
bool placeInWindow(juce::Component& dialog, juce::Component& host);

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
        Barlow Semi Condensed and IBM Plex Mono, so windows that build plain
        FontOptions share the theme's typefaces; maps the fallback names
        (symbolsSans, symbolsMono) to the embedded Inter and JetBrains Mono. */
    juce::Typeface::Ptr getTypefaceForFont(const juce::Font&) override;
    juce::Rectangle<int> getTooltipBounds(const juce::String& tipText, juce::Point<int> screenPos,
                                          juce::Rectangle<int> parentArea) override;
    void drawTooltip(juce::Graphics&, const juce::String& text, int width, int height) override;
};

} // namespace enginelab::ui
