#pragma once

#include <enginelab/render/EngineModel3D.hpp>
#include <enginelab/render/GasFieldView.hpp>
#include <juce_opengl/juce_opengl.h>
#include <memory>
#include <vector>

namespace enginelab::ui {

/** Which parts the view shows, in the order of the layer selector. */
enum class SceneLayerMode { all, combustion, mechanical, gasFlow };

/** Everything the renderer needs to draw one frame. */
struct SceneFrame final {
    render::Mat4 view;
    render::Mat4 projection;
    render::Vec3 eye;
    SceneLayerMode layer { SceneLayerMode::all };
    bool xray { true };
    bool antiAliasing { true };
    /** Crank angles of the motion-blur sub-frames, oldest first; one angle
        draws a sharp frame. */
    std::vector<double> angles;
    /** Angle the turbo shaft sweeps over the same sub-frames, ending at
        pose.turboShaftDegrees. */
    double turboShaftSweepDegrees {};
    render::ScenePoseInput pose;
    /** The solver's gas along the exhaust, or null. Waves are drawn in the
        All and Gas flow layers; hot walls glow in every layer. */
    const render::GasFieldView* gasField {};
    /** Part highlighted by the inspector, or -1. */
    int selectedPart { -1 };
};

/** Whether a click can select this part in the given layer: hidden parts,
    flames and X-ray shells let the click through. */
[[nodiscard]] bool scenePartPickable(const render::ScenePart&, SceneLayerMode, bool xray) noexcept;

/**
 * OpenGL 3.2 renderer of an EngineModel3D. Lives on the OpenGL thread: create
 * it in newOpenGLContextCreated(), draw in renderOpenGL(), destroy it in
 * openGLContextClosing().
 *
 * Each frame is drawn into a multisampled buffer. With several crank angles
 * (motion blur), every sub-frame is resolved and averaged into a
 * half-float accumulation buffer, which is then copied to the framebuffer the
 * caller had bound. The shading is a small physically based model (two
 * directional lights, a studio environment, ACES tone mapping) tuned to the
 * prototype's look; X-ray shells are translucent with a Fresnel rim.
 */
class EngineSceneRenderer final {
public:
    EngineSceneRenderer();
    ~EngineSceneRenderer();
    EngineSceneRenderer(const EngineSceneRenderer&) = delete;
    EngineSceneRenderer& operator=(const EngineSceneRenderer&) = delete;

    /** Compiles the shaders. Returns false with `error` set when the context
        cannot run them (no OpenGL 3.2). */
    [[nodiscard]] bool initialise(juce::String& error);
    void release();

    /** Uploads the meshes of a new engine. */
    void setModel(std::shared_ptr<const render::EngineModel3D>);
    [[nodiscard]] const render::EngineModel3D* model() const noexcept { return model_.get(); }

    /** Draws into the viewport currently set by JUCE, `width` x `height`
        physical pixels. */
    void render(const SceneFrame&, int width, int height);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    std::shared_ptr<const render::EngineModel3D> model_;
};

} // namespace enginelab::ui
