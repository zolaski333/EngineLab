#include <enginelab/physics/IndicatedWorkModel.hpp>
#include <algorithm>
#include <cmath>

namespace enginelab {

void IndicatedWorkModel::advance(IndicatedWorkState& state, double pressureKpa,
                                 double volumeLitres, double ambientPressureKpa,
                                 bool cycleBoundaryCrossed) noexcept {
    if (!std::isfinite(pressureKpa) || !std::isfinite(volumeLitres)
        || volumeLitres <= 0.0) return;
    if (state.initialised) {
        const auto meanGaugePressurePa = (0.5 * (state.previousPressureKpa + pressureKpa)
            - ambientPressureKpa) * 1'000.0;
        const auto volumeChangeM3 = (volumeLitres - state.previousVolumeLitres) * 0.001;
        state.accumulatedJoules += meanGaugePressurePa * volumeChangeM3;
    }
    if (cycleBoundaryCrossed && state.initialised) {
        state.completedCycleJoules = state.accumulatedJoules;
        state.accumulatedJoules = 0.0;
    }
    state.previousPressureKpa = pressureKpa;
    state.previousVolumeLitres = volumeLitres;
    state.initialised = true;
}

} // namespace enginelab
