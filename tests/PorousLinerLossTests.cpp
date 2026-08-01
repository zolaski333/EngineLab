#include <enginelab/audio/PorousLinerLoss.hpp>

#include <cmath>
#include <iostream>
#include <stdexcept>

namespace {

void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

} // namespace

int main() {
    try {
        constexpr double density = 0.78;
        constexpr double soundSpeed = 510.0;
        constexpr double resistivity = 24'000.0;
        constexpr double thickness = 0.045;
        constexpr double openArea = 0.28;
        const auto absorption1k = enginelab::PorousLinerLoss::absorptionCoefficient(
            1'000.0, density, soundSpeed, resistivity, thickness);
        const auto absorption8k = enginelab::PorousLinerLoss::absorptionCoefficient(
            8'000.0, density, soundSpeed, resistivity, thickness);
        require(absorption1k > 0.0 && absorption1k <= 1.0
                && absorption8k > 0.0 && absorption8k <= 1.0,
            "porous absorption must be passive and non-zero");

        const auto gain1k = enginelab::PorousLinerLoss::traversalGain(
            1'000.0, 0.45, 0.032, density, soundSpeed,
            resistivity, thickness, openArea);
        const auto gain8k = enginelab::PorousLinerLoss::traversalGain(
            8'000.0, 0.45, 0.032, density, soundSpeed,
            resistivity, thickness, openArea);
        require(gain1k > 0.0 && gain1k <= 1.0
                && gain8k > 0.0 && gain8k <= 1.0,
            "porous traversal gain must never add acoustic energy");
        require(enginelab::PorousLinerLoss::traversalGain(
                8'000.0, 0.45, 0.032, density, soundSpeed,
                0.0, thickness, openArea) == 1.0,
            "zero resistivity metadata must be an exact bypass");

        const auto fitted = enginelab::PorousLinerLoss::fit(
            0.45, 0.032, density, soundSpeed, resistivity,
            thickness, openArea, 48'000.0);
        const auto fitted1k = enginelab::DuctWallLoss::magnitude(
            fitted, 1'000.0, 48'000.0);
        const auto fitted15k = enginelab::DuctWallLoss::magnitude(
            fitted, 15'000.0, 48'000.0);
        require(fitted1k > 0.0 && fitted1k <= 1.0
                && fitted15k > 0.0 && fitted15k <= 1.0,
            "fitted realtime liner filter must remain passive");

        const auto bypass = enginelab::PorousLinerLoss::fit(
            0.45, 0.032, density, soundSpeed, 0.0,
            0.0, 0.0, 48'000.0);
        require(bypass.pole == 0.0F && bypass.zero == 0.0F
                && bypass.gain == 1.0F,
            "unauthored packing must compile to an exact identity filter");
        std::cout << "PASS: passive opt-in porous muffler lining\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
