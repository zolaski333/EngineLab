#pragma once

#include <enginelab/render/RenderSnapshot.hpp>

namespace enginelab {

struct RenderViewport final {
    int widthPixels {};
    int heightPixels {};
    double deviceScale { 1.0 };
};

struct RenderCamera final {
    RenderVector3 positionMm { 0.0, 0.0, 900.0 };
    RenderVector3 targetMm;
    RenderVector3 up { 0.0, 1.0, 0.0 };
    double verticalFieldOfViewDegrees { 42.0 };
    double nearPlaneMm { 1.0 };
    double farPlaneMm { 20'000.0 };
};

/**
 * Backend boundary for the future 3D view.
 *
 * No graphics API appears in this interface. An OpenGL implementation can be
 * added later without leaking JUCE/OpenGL types into simulation or scene data.
 */
class IEngineRenderer {
public:
    virtual ~IEngineRenderer() = default;
    virtual void prepare(const RenderViewport&) = 0;
    virtual void resize(const RenderViewport&) = 0;
    virtual void render(const RenderSnapshot&, const RenderCamera&) = 0;
    virtual void release() noexcept = 0;
};

} // namespace enginelab
