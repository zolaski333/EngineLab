#pragma once

namespace enginelab {

struct IndicatedWorkState final {
    double accumulatedJoules { 0.0 };
    double completedCycleJoules { 0.0 };
    /** Same integral restricted to the gas-exchange strokes (see below). */
    double accumulatedPumpingJoules { 0.0 };
    double completedPumpingCycleJoules { 0.0 };
    /** And restricted further, to the exhaust stroke alone (see below). */
    double accumulatedExhaustStrokeJoules { 0.0 };
    double completedExhaustStrokeCycleJoules { 0.0 };
    double previousVolumeLitres { 0.0 };
    double previousPressureKpa { 101.325 };
    bool initialised { false };
};

/** Integrates closed-cycle indicated work from the same P/V trajectory used for crank torque.
 *
 * The same integral is additionally split into its two loops. `gasExchangeStroke`
 * selects the pumping loop -- the exhaust and intake strokes, i.e. the half of
 * the cycle where the piston moves gas rather than compressing or expanding it.
 * `exhaustStroke` splits that loop again, into the half the piston spends
 * expelling and the half it spends inducting, because back pressure and intake
 * depression are different defects with different fixes and a single pumping
 * figure cannot tell them apart. Both are telemetry only: `completedCycleJoules`
 * is the sum of both loops and is byte-for-byte what it was before the split
 * existed, so nothing that consumes indicated work changes.
 *
 * Why it is worth publishing separately: pumping mean effective pressure is the
 * one number that says whether an engine can breathe. It sits in a narrow,
 * well-documented band at wide-open throttle (roughly -0.2 to -0.6 bar for a
 * naturally aspirated engine, the loss growing with speed), and a cycle-averaged
 * IMEP hides a gas-exchange failure completely: an engine that cannot evacuate
 * its cylinder still reports a healthy IMEP while paying for it twice, once in
 * pumping work and once in the residual it re-inducts.
 */
class IndicatedWorkModel final {
public:
    static void advance(IndicatedWorkState&, double pressureKpa, double volumeLitres,
                        double ambientPressureKpa, bool cycleBoundaryCrossed,
                        bool gasExchangeStroke, bool exhaustStroke) noexcept;
};

} // namespace enginelab
