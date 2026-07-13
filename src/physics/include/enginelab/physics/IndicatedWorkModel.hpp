#pragma once

namespace enginelab {

struct IndicatedWorkState final {
    double accumulatedJoules { 0.0 };
    double completedCycleJoules { 0.0 };
    double previousVolumeLitres { 0.0 };
    double previousPressureKpa { 101.325 };
    bool initialised { false };
};

/** Integrates closed-cycle indicated work from the same P/V trajectory used for crank torque. */
class IndicatedWorkModel final {
public:
    static void advance(IndicatedWorkState&, double pressureKpa, double volumeLitres,
                        double ambientPressureKpa, bool cycleBoundaryCrossed) noexcept;
};

} // namespace enginelab
