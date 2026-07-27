#include <enginelab/physics/IndicatedWorkModel.hpp>
#include <algorithm>
#include <cmath>

namespace enginelab {

void IndicatedWorkModel::advance(IndicatedWorkState& state, double pressureKpa,
                                 double volumeLitres, double ambientPressureKpa,
                                 bool cycleBoundaryCrossed,
                                 bool gasExchangeStroke,
                                 bool exhaustStroke) noexcept {
    if (!std::isfinite(pressureKpa) || !std::isfinite(volumeLitres)
        || volumeLitres <= 0.0) return;
    if (state.initialised) {
        const auto meanGaugePressurePa = (0.5 * (state.previousPressureKpa + pressureKpa)
            - ambientPressureKpa) * 1'000.0;
        const auto volumeChangeM3 = (volumeLitres - state.previousVolumeLitres) * 0.001;
        const auto incrementJoules = meanGaugePressurePa * volumeChangeM3;
        state.accumulatedJoules += incrementJoules;
        // The pumping accumulator takes a share of the same increment, never a
        // separately integrated quantity: gross is then exactly total minus
        // pumping, and the split cannot drift away from the work that drives
        // the crank.
        if (gasExchangeStroke) state.accumulatedPumpingJoules += incrementJoules;
        if (gasExchangeStroke && exhaustStroke)
            state.accumulatedExhaustStrokeJoules += incrementJoules;
    }
    if (cycleBoundaryCrossed && state.initialised) {
        state.completedCycleJoules = state.accumulatedJoules;
        state.accumulatedJoules = 0.0;
        state.completedPumpingCycleJoules = state.accumulatedPumpingJoules;
        state.accumulatedPumpingJoules = 0.0;
        state.completedExhaustStrokeCycleJoules = state.accumulatedExhaustStrokeJoules;
        state.accumulatedExhaustStrokeJoules = 0.0;
    }
    state.previousPressureKpa = pressureKpa;
    state.previousVolumeLitres = volumeLitres;
    state.initialised = true;
}

} // namespace enginelab
