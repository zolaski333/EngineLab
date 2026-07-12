#pragma once

#include <array>
#include <cstddef>

namespace enginelab {

/** One thermodynamic-substep snapshot consumed by the realtime audio path. */
struct CylinderPressureSample final {
    double timeSeconds { 0.0 };
    std::array<float, 32> pressureBar {};
    std::size_t cylinderCount { 0 };
};

} // namespace enginelab
