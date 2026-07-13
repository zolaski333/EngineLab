#pragma once

#include <enginelab/foundation/EngineTypes.hpp>

namespace enginelab {

struct ValveTrainState final {
    double intakeAdvanceDegrees { 0.0 };
    double exhaustAdvanceDegrees { 0.0 };
    double liftMultiplier { 1.0 };
};

struct ValveTrainResult final {
    double intakeLiftMm { 0.0 };
    double exhaustLiftMm { 0.0 };
    double intakeDischargeCoefficient { 0.0 };
    double exhaustDischargeCoefficient { 0.0 };
    double intakeAdvanceDegrees { 0.0 };
    double exhaustAdvanceDegrees { 0.0 };
    double liftMultiplier { 1.0 };
};

class ValveTrainModel final {
public:
    [[nodiscard]] static ValveTrainResult evaluate(const CamshaftConfig&, bool highProfile,
                                                    ValveTrainState&, double cyclePhaseDegrees,
                                                    double rpm, double load, double dtSeconds) noexcept;
};

} // namespace enginelab
