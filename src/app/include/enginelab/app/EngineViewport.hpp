#pragma once

#include <enginelab/app/DashboardModel.hpp>
#include <enginelab/app/EngineScene.hpp>
#include <enginelab/app/Widgets.hpp>
#include <enginelab/render/CrankClock.hpp>
#include <atomic>
#include <condition_variable>
#include <functional>
#include <memory>
#include <mutex>
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
        std::uint64_t sampleSequence {};
        render::ScenePoseInput pose;
        double playbackFactor { 1.0 };
        SceneLayerMode layer { SceneLayerMode::all };
        bool xray { true };
        bool motionBlur { true };
        bool antiAliasing { true };
        bool vsync { true };
        Orbit goal;
        float smoothingSeconds { 0.08F };
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

    const DashboardModel& model_;
    EngineCutawayView cutaway_;
    SegmentedControl layers_ { { "All", "Combustion", "Mechanical", "Gas flow" } };
    SegmentedControl views_ { { "Front", "Side", "3/4" } };
    SegmentedControl shading_ { { "X-ray", "Solid" } };
    SegmentedControl playback_ { { "Real time", "1:50", "1:250", "Freeze" } };
    ActionButton settingsButton_ { {}, ActionButton::Style::tool };
    CyclePanel cycle_;
    std::function<bool()> wheelModifierActive_;

    ViewSettings settings_;
    std::shared_ptr<const render::EngineModel3D> scene_;
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
    Orbit orbit_ {};
    bool orbitValid_ { false };
    double lastFrameWall_ {};
    double frameInterval_ { 1.0 / 60.0 };
    int appliedSwapInterval_ { -1 };
    SceneFrame frame_;

    // Shared atomics.
    std::atomic<double> displayedAngle_ { 0.0 };
    std::atomic<std::uint64_t> framesRendered_ { 0 };
    std::atomic<bool> initialiseFailed_ { false };
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
