#include <enginelab/physics/CombustionCycleVariation.hpp>

#include <cmath>
#include <cstdint>
#include <iostream>
#include <stdexcept>

namespace {
void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}
}

int main() {
    try {
        std::uint32_t bypassRng = 0x12345678U;
        auto bypassState = 4.0;
        require(enginelab::CombustionCycleVariation::advance(
                    0.0, 0.7, bypassRng, bypassState) == 1.0
                && bypassRng == 0x12345678U && bypassState == 0.0,
            "zero cycle variation must be an exact deterministic bypass");

        std::uint32_t rngA = 0x31415926U;
        std::uint32_t rngB = rngA;
        auto stateA = 0.0;
        auto stateB = 0.0;
        double sum = 0.0;
        double sumSquares = 0.0;
        double lagProduct = 0.0;
        auto previous = 1.0;
        constexpr int samples = 20'000;
        for (int index = 0; index < samples; ++index) {
            const auto a = enginelab::CombustionCycleVariation::advance(
                0.05, 0.65, rngA, stateA);
            const auto b = enginelab::CombustionCycleVariation::advance(
                0.05, 0.65, rngB, stateB);
            require(a == b && a >= 0.55 && a <= 1.45,
                "cycle variation must be reproducible and bounded");
            const auto centred = a - 1.0;
            sum += centred;
            sumSquares += centred * centred;
            if (index > 0) lagProduct += centred * (previous - 1.0);
            previous = a;
        }
        const auto mean = sum / samples;
        const auto variance = sumSquares / samples - mean * mean;
        const auto lagCorrelation = lagProduct / (samples - 1)
            / std::max(1.0e-12, variance);
        require(std::abs(mean) < 0.003,
            "cycle variation mean must remain centred on nominal combustion");
        require(std::sqrt(variance) > 0.045 && std::sqrt(variance) < 0.055,
            "authored coefficient must represent burn-rate standard deviation");
        require(lagCorrelation > 0.60 && lagCorrelation < 0.70,
            "neighbouring cycles must retain the authored correlation");
        std::cout << "PASS: deterministic physical combustion-cycle variation\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL: " << error.what() << '\n';
        return 1;
    }
}
