#pragma once

#include <enginelab/foundation/EngineTypes.hpp>

namespace enginelab {

struct HelmholtzRunnerState final {
    double pressureAmplitudeKpa { 0.0 };
    double pressureVelocityKpaPerSecond { 0.0 };
};

struct HelmholtzRunnerResult final {
    double pressureAmplitudeKpa { 0.0 };
    double resonanceFrequencyHz { 0.0 };
    double flowAdmittance { 1.0 };
};

/**
 * Lumped runner/plenum acoustic compliance coupled to the conservative mass-flow restriction.
 *
 * Two things about this model are measured, not guessed, and both matter before
 * anyone reaches for `coupling_gain` (see docs/physics-audit.md):
 *
 *  - The frequency is right and the amplitude is starved. On the CP4 at 9013 rpm
 *    the natural frequency is 138.8 Hz against a 75.1 Hz valve-event frequency --
 *    a Helmholtz number of 1.85, Engelman's optimum. But the forcing,
 *    `(plenum - runner)`, spans only -4.0 .. +6.4 kPa, because a 0-D runner is
 *    joined to its plenum by a FULL-AREA orifice and equalises with it almost at
 *    once. The port therefore sits at exactly ambient by IVC: no ram, whatever
 *    the gain. Raising the gain amplifies a signal the topology already
 *    flattened -- it moves the level of the VE curve and never its shape.
 *
 *  - Forcing on the column's inertial reaction instead (dP = -rho*l_eff*du/dt,
 *    the 1-D momentum equation, ~27 kPa at the same point) was implemented and
 *    measured WORSE: CP4 VE at 9000 rpm fell 0.790 -> 0.754 and peak power
 *    80.7 -> 75.4 PS. Two reasons, both fatal to the cheap version. (a) Phase: a
 *    resonator whose period is 7.2 ms cannot pass a term that must act inside a
 *    3.3 ms intake event, so the output inverted -- -3.40 kPa at full lift, where
 *    it opposes the fill, and +0.02 kPa at IVC, where the ram belongs.
 *    (b) Signal: `intakeRunnerGas_.bulkVelocityMps()` is a cell average that does
 *    NOT collapse when the valve shuts (40.9 -> 40.3 m/s across IVC), so the very
 *    event that creates inertia ram is absent from du/dt. Correctly-phased ram
 *    needs a valve-plane velocity (from the valve mass flow) or a real runner
 *    inertance, not a side-car resonator on a cell average.
 *
 * `flowAdmittance` is NOT redundant with routing the amplitude to the valve.
 * Measured: removing it costs CP4 0.023 of VE at 9000 rpm. It looks like a
 * double model and is not.
 */
class HelmholtzRunnerModel final {
public:
    [[nodiscard]] static HelmholtzRunnerResult advance(const RunnerAcousticsConfig&,
        HelmholtzRunnerState&, const CylinderConfig&, const IntakeConfig&,
        double gasTemperatureK, double plenumPressureKpa, double runnerPressureKpa,
        double dtSeconds) noexcept;
};

} // namespace enginelab
