#pragma once

#include <enginelab/app/DashboardModel.hpp>
#include <enginelab/app/EngineScene.hpp>
#include <enginelab/app/Widgets.hpp>
#include <enginelab/foundation/GasFieldSnapshot.hpp>
#include <enginelab/render/CrankClock.hpp>
#include <enginelab/render/GasFieldView.hpp>
#include <enginelab/render/ListenerFrame.hpp>
#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>

namespace enginelab::ui {

/** The 2-D cutaway of the cylinders and valvetrain, kept as the fallback when
    OpenGL 3.2 is not available. Wheel zooms, drag pans, double-click
    recentres. */
class EngineCutawayView final : public juce::Component {
public:
    explicit EngineCutawayView(const DashboardModel&);
    void setLayer(int layer);
    /** True while a wheel-modifier key is held: the wheel then belongs to the
        main component and must not zoom. */
    std::function<bool()> wheelModifierActive;

    void paint(juce::Graphics&) override;
    void mouseWheelMove(const juce::MouseEvent&, const juce::MouseWheelDetails&) override;
    void mouseDown(const juce::MouseEvent&) override;
    void mouseDrag(const juce::MouseEvent&) override;
    void mouseDoubleClick(const juce::MouseEvent&) override;

private:
    void drawEngine(juce::Graphics&, juce::Rectangle<float> area) const;
    const DashboardModel& model_;
    int layer_ { 0 };
    float zoom_ { 1.0F };
    juce::Point<float> pan_ {};
    juce::Point<float> dragStartPan_ {};
};

/** Display options of the engine view, saved next to the key bindings. */
struct ViewSettings final {
    bool renderer3d { true };
    /** Frames per second; 0 = unlimited. */
    int frameRateCap { 60 };
    bool vsync { true };
    bool motionBlur { true };
    bool antiAliasing { true };
    /** 180 mm exhaust gas cells instead of 360 mm: the waves keep their
        shape; the simulation has 13 to 20 % less headroom (LS3 at 60 %
        of redline: 1.05 times real time instead of 1.31). */
    bool fineExhaustWaves { false };
    /** How the gas field colours the ducts in the All layer: not at all
        (the default: nothing flickers), by how strongly each cell pulses
        (steady), or by each pulse as it travels. The Gas flow layer shows
        the strength when this is `hidden`. */
    enum class PressureWaves { hidden, strength, live };
    PressureWaves pressureWaves { PressureWaves::hidden };
    /** The sound is heard from the camera (EngineViewport::cameraMicrophones)
        instead of the engine's authored listener. */
    bool cameraMicrophone { true };

    static juce::File file();
    void load();
    void save() const;
};

/** Phase of every cylinder over the four strokes, bottom left of the view. */
class CyclePanel final : public juce::Component {
public:
    std::function<double(std::size_t cylinder)> phaseOf;
    void setCylinders(std::vector<juce::String> labels);
    [[nodiscard]] int idealHeight() const;
    void paint(juce::Graphics&) override;

private:
    std::vector<juce::String> labels_;
};

/** Colour scale of the pressure waves, bottom centre of the view. */
class WaveLegend final : public juce::Component {
public:
    void setScale(float pascals);
    /** Root-mean-square scale (strength colouring) instead of ± departure. */
    void setStrength(bool strength);
    void paint(juce::Graphics&) override;

private:
    float scalePa_ {};
    bool strength_ {};
};

/** What the clicked part is, with its authored geometry and live gas state. */
class PartInspector final : public juce::Component, private juce::Timer {
public:
    struct Row final {
        juce::String label;
        juce::String value;
    };
    std::function<void()> onClose;
    /** An oscilloscope trace over the engine cycle. */
    struct Scope final {
        /** Pressure against ambient per crank bin (kPa); NaN where empty. */
        std::vector<float> kpa;
        int latestBin { -1 };
        double cycleDegrees { 720.0 };
        juce::String caption;
    };
    /** Two values edited live: an exhaust component's or a runner's length
        and diameter, the cylinders' bore and stroke, a plenum's volume, the
        turbo's boost, the injectors' flow. Steppers, or a click on the value
        to type it (Enter applies, Escape cancels; both hand the keyboard back
        to the engine). */
    struct Edit final {
        /** Which part: the same key keeps the values being edited. */
        std::uint64_t key {};
        juce::String heading { "RESIZE" };
        juce::String firstLabel { "Length" };
        juce::String secondLabel { "Diameter" };
        juce::String firstUnit { " mm" };
        juce::String secondUnit { " mm" };
        /** Digits shown; -1 shows a tenth only when the value has one. */
        int firstDecimals { -1 };
        int secondDecimals { -1 };
        double first {};
        double second {};
        bool firstEditable { true };
        /** A press, and a press with Shift. */
        double firstStep { 10.0 };
        double firstFineStep { 1.0 };
        double secondStep { 1.0 };
        double secondFineStep { 0.1 };
        double minimum { 10.0 };
        /** The second value's lower bound, when not `minimum`. */
        std::optional<double> secondMinimum;
        double firstMaximum { 5'000.0 };
        double secondMaximum { 400.0 };
        /** Offers the choice between the next cycle and a gradual change. */
        bool offersRamp {};
        juce::String note;
    };
    PartInspector();
    /** Called with the edited sizes a moment after the last stepper press,
        and at once by Reset: the engine hears each size live. `gradual` is
        the ramp choice, when offered. */
    std::function<void(double first, double second, bool gradual)> onApplyEdit;
    /** Called when a typed value is applied or cancelled, to hand the
        keyboard back. */
    std::function<void()> onDoneTyping;
    /** `note`, if any, is a short paragraph under the rows. */
    void show(juce::String title, juce::String subtitle, std::vector<Row> rows, juce::String note = {});
    /** Shows a trace under the rows, or none. */
    void setScope(std::optional<Scope> scope);
    /** Offers the size steppers, or none. */
    void setEdit(std::optional<Edit> edit);
    /** Why the last edit was refused, under the steppers; empty clears it. */
    void setEditStatus(juce::String status);
    [[nodiscard]] int idealHeight() const;
    void paint(juce::Graphics&) override;
    void resized() override;
    void mouseUp(const juce::MouseEvent&) override;
    void mouseMove(const juce::MouseEvent&) override;

    /** Stepper presses closer together than this make a single edit. */
    static constexpr int editSettleMs = 250;

private:
    void timerCallback() override;
    void sendEdit();
    [[nodiscard]] juce::String formatValue(double value, bool first, bool withUnit = true) const;
    [[nodiscard]] double lowest(bool first) const;
    [[nodiscard]] juce::Rectangle<int> closeArea() const;
    juce::String title_;
    juce::String subtitle_;
    std::vector<Row> rows_;
    juce::String note_;
    std::optional<Scope> scope_;
    std::optional<Edit> edit_;
    double pendingFirst_ {};
    double pendingSecond_ {};
    /** The size when the component was selected, which Reset goes back to. */
    double originFirst_ {};
    double originSecond_ {};
    juce::String editStatus_;
    ActionButton firstDown_ { "-", ActionButton::Style::compact };
    ActionButton firstUp_ { "+", ActionButton::Style::compact };
    ActionButton secondDown_ { "-", ActionButton::Style::compact };
    ActionButton secondUp_ { "+", ActionButton::Style::compact };
    ActionButton resetEdit_ { "Reset", ActionButton::Style::compact };
    ActionButton rampEdit_ { "Next cycle", ActionButton::Style::compact };
    /** The owner's choice (2026-10-06): a resize glides over the ramp by
        default, which also avoids the lean cycle of an instant change. */
    bool gradual_ { true };
    juce::TextEditor valueEditor_;
    /** The value being typed: 0 the first, 1 the second, -1 none. */
    int typing_ { -1 };
    [[nodiscard]] juce::Rectangle<int> valueArea(bool first) const;
    void beginTyping(bool first);
    void endTyping(bool apply);
    [[nodiscard]] juce::TextLayout noteLayout(const juce::String& text, juce::Colour, float width) const;
    [[nodiscard]] int editHeight() const;
    void stepEdit(bool length, double direction);
    void updateEditButtons();
    void paintScope(juce::Graphics&, juce::Rectangle<float> area) const;
    void paintEdit(juce::Graphics&, juce::Rectangle<int> area) const;
};

/**
 * Centre of the window: the GPU-rendered 3-D engine with its floating
 * overlays, or the 2-D cutaway.
 *
 * The message thread feeds the simulator's crank angle and speed about 30
 * times a second; the OpenGL thread draws at the chosen frame rate and turns
 * the crank with a render::CrankClock, so the picture runs at exactly the
 * simulated speed. Overlays are child components painted by JUCE on top of
 * the OpenGL frame.
 */
class EngineViewport final : public juce::Component, private juce::OpenGLRenderer {
public:
    explicit EngineViewport(const DashboardModel&);
    ~EngineViewport() override;

    void setWheelModifierCheck(std::function<bool()>);
    /** Selects cylinder 1's injector, with its inspector (the layer goes back
        to All when the current one hides it). False without a drawn engine. */
    bool showInjectors();
    /** Where the gas field comes from: asks for the field at a crank angle
        (negative: the latest) and copies a newer one into the snapshot. */
    void setGasFieldSource(std::function<bool(double crankAngleDegrees, GasFieldSnapshot&)>);
    /** Where the inspector's oscilloscope comes from: asks for the pressure
        trace of a gas field element and sample, and copies a newer one. */
    void setGasProbeSource(std::function<bool(std::int32_t element, std::uint8_t sample, GasProbeTrace&)>);
    /** Where a resize made in the part inspector goes: applies the edited
        configuration to the engine, or refuses it. */
    void setConfigEditor(std::function<bool(const EngineConfig&)>);
    /** Where a bore and stroke change goes, with its ramp in seconds (0: at
        the next cycle). */
    void setCylinderEditor(std::function<bool(const EngineConfig&, double rampSeconds)>);
    /** Seconds over which a gradual bore and stroke change happens. */
    static constexpr double cylinderRampSeconds = 3.0;
    /** The exhaust gas network's cell length chosen in the view settings:
        the finer mesh shows the waves' shape, at a cost in simulation time.
        Empty: the production mesh. */
    [[nodiscard]] std::optional<double> exhaustCellLengthM() const noexcept {
        return settings_.fineExhaustWaves ? std::optional<double> { 0.180 } : std::nullopt;
    }
    /** Called when that choice changes. */
    std::function<void()> onExhaustResolutionChanged;
    /** With the camera microphone on, the two microphones at the camera
        where it is heading (render::ListenerFrame), spaced as the engine's
        authored pair; empty when off or with no 3-D model. */
    [[nodiscard]] std::optional<std::array<AcousticPoint3M, 2>> cameraMicrophones() const;
    /** Where the 3-D model draws the openings and the turbo, in the frame of
        cameraMicrophones() (render::ListenerFrame::sources); empty with no
        3-D model. `soundSourcesVersion` changes whenever they do. */
    [[nodiscard]] const AcousticSourcePlacements& soundSources() const noexcept { return soundSources_; }
    [[nodiscard]] std::uint64_t soundSourcesVersion() const noexcept { return soundSourcesVersion_; }
    /** Called by a click in the view or a finished typed value: the
        keyboard goes back to driving the engine. */
    std::function<void()> onClaimKeyboard;
    /** Asks the runtime to simulate at this pace: physics and sound slow
        down together, unlike the stroboscope, which only slows the picture. */
    std::function<void(double timeScale)> onTimeScaleRequested;
    /** Rebuilds the 3-D model; call after every engine change. */
    void setEngine(const EngineConfig&);
    void stepLayer(int delta);
    /** Called by the main component's 30 Hz timer. */
    void refresh();

    void paint(juce::Graphics&) override;
    void resized() override;
    void mouseDown(const juce::MouseEvent&) override;
    void mouseDrag(const juce::MouseEvent&) override;
    void mouseUp(const juce::MouseEvent&) override;
    void mouseWheelMove(const juce::MouseEvent&, const juce::MouseWheelDetails&) override;
    void mouseDoubleClick(const juce::MouseEvent&) override;

private:
    struct Orbit final {
        render::Vec3 target;
        float yaw {};
        float pitch {};
        float distance { 1000.0F };
    };
    /** What the message thread hands to the OpenGL thread. */
    struct Shared final {
        std::shared_ptr<const render::EngineModel3D> model;
        std::uint64_t modelRevision {};
        double sampleWall {};
        double crankAngle {};
        double rpm {};
        double rate { 1.0 };
        double turboShaftRpm {};
        std::uint64_t sampleSequence {};
        render::ScenePoseInput pose;
        double playbackFactor { 1.0 };
        SceneLayerMode layer { SceneLayerMode::all };
        ViewSettings::PressureWaves pressureWaves { ViewSettings::PressureWaves::hidden };
        bool xray { true };
        bool motionBlur { true };
        bool antiAliasing { true };
        bool vsync { true };
        Orbit goal;
        float smoothingSeconds { 0.08F };
        std::shared_ptr<const GasFieldSnapshot> gasField;
        int selectedPart { -1 };
    };

    void newOpenGLContextCreated() override;
    void renderOpenGL() override;
    void openGLContextClosing() override;

    void applyRenderer();
    void startPacing();
    void stopPacing();
    void pacingLoop();
    void applyView(int view, bool animate);
    void showSettingsMenu();
    void pushSettings();
    void updateOverlayVisibility();
    [[nodiscard]] juce::Rectangle<int> infoArea() const;
    [[nodiscard]] Orbit presetOrbit(int view) const;
    [[nodiscard]] Orbit goal() const;
    void setGoal(const Orbit&, float smoothingSeconds);
    void selectPartAt(juce::Point<float>);
    void setSelectedPart(int part);
    void updateInspector();
    /** Asks for the oscilloscope trace of the selected duct and shows it. */
    void updateScope();
    void applyEdit(double first, double second, bool gradual);
    void applyExhaustEdit(double lengthMm, double diameterMm);
    void applyCylinderEdit(double boreMm, double strokeMm, bool gradual);
    /** Applies the selected part's partEditor_ through configEditor_. */
    void applyPartEdit(double first, double second);
    void updateLegend();

    const DashboardModel& model_;
    EngineCutawayView cutaway_;
    SegmentedControl layers_ { { "All", "Combustion", "Mechanical", "Gas flow" } };
    SegmentedControl views_ { { "Front", "Side", "3/4", "Exhaust" } };
    SegmentedControl shading_ { { "X-ray", "Solid" } };
    SegmentedControl speed_ { { "0.25x", "0.5x", "1x" } };
    SegmentedControl playback_ { { "Live", "Strobe 1:50", "Strobe 1:250", "Freeze" } };
    ActionButton settingsButton_ { {}, ActionButton::Style::tool };
    CyclePanel cycle_;
    WaveLegend legend_;
    PartInspector inspector_;
    std::function<bool()> wheelModifierActive_;
    std::function<bool(double, GasFieldSnapshot&)> gasFieldSource_;
    std::function<bool(std::int32_t, std::uint8_t, GasProbeTrace&)> gasProbeSource_;
    std::function<bool(const EngineConfig&)> configEditor_;
    std::function<bool(const EngineConfig&, double)> cylinderEditor_;
    /** How the selected intake part, turbo or injectors take an edit: the two
        values into a copy of the configuration, or why not. Set by
        updateInspector(). */
    std::function<std::string(EngineConfig&, double, double)> partEditor_;
    /** The part an edit resized: selected again, with the camera kept, when
        the engine comes back. An exhaust component by its ids (the scene's
        parts change), a cylinder part by its index (they do not). */
    struct Reselect final {
        std::uint32_t pathId {};
        std::uint32_t elementId {};
        int part { -1 };
    };
    std::optional<Reselect> reselect_;
    GasProbeTrace gasProbe_;
    /** Where along the selected duct the click landed, 0..1 from its inlet. */
    float probeStation_ { 0.5F };
    /** Latest field on the message thread, and the ducts it binds to. */
    GasFieldSnapshot gasField_;
    std::vector<int> gasFieldBinding_;
    int selectedPart_ { -1 };
    bool dragged_ { false };

    ViewSettings settings_;
    std::shared_ptr<const render::EngineModel3D> scene_;
    render::ListenerFrame listenerFrame_;
    AcousticSourcePlacements soundSources_;
    std::uint64_t soundSourcesVersion_ { 0 };
    double microphoneSpacingM_ { 0.36 };
    juce::String glError_;
    bool glFailed_ { false };
    double attachedAt_ {};
    juce::OpenGLContext context_;

    mutable std::mutex sharedMutex_;
    Shared shared_;
    std::uint64_t modelRevision_ {};
    std::uint64_t sampleSequence_ {};
    int view_ { 2 };
    juce::Point<float> dragStart_ {};
    Orbit dragStartOrbit_ {};
    bool panning_ { false };
    juce::String infoText_;

    // OpenGL thread only.
    std::unique_ptr<EngineSceneRenderer> renderer_;
    render::CrankClock clock_;
    std::uint64_t renderedModelRevision_ {};
    std::uint64_t observedSequence_ {};
    std::uint64_t renderedGasField_ {};
    render::GasFieldView gasFieldView_;
    Orbit orbit_ {};
    bool orbitValid_ { false };
    double lastFrameWall_ {};
    double frameInterval_ { 1.0 / 60.0 };
    double turboShaftDegrees_ {};
    int appliedSwapInterval_ { -1 };
    SceneFrame frame_;

    // Shared atomics.
    std::atomic<double> displayedAngle_ { 0.0 };
    std::atomic<std::uint64_t> framesRendered_ { 0 };
    std::atomic<bool> initialiseFailed_ { false };
    std::atomic<float> pressureScalePa_ { 0.0F };
    std::uint64_t lastFrameCount_ {};
    double lastFpsWall_ {};
    double measuredFps_ {};

    // Frame pacing thread (a capped frame rate triggers repaints).
    std::thread pacer_;
    std::mutex pacerMutex_;
    std::condition_variable pacerWake_;
    bool pacerStop_ { false };
    std::atomic<int> pacerPeriodMicroseconds_ { 16'667 };
    /** Made on the message thread, copied by the pacing thread. */
    juce::Component::SafePointer<EngineViewport> safeThis_;
};

} // namespace enginelab::ui
