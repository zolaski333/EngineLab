#pragma once
#include <enginelab/exhaust/IExhaustModel.hpp>
#include <cstdint>
#include <vector>
namespace enginelab {
enum class ExhaustNodeType : std::uint8_t { port, pipe, merge, splitter, resonator, muffler, catalyst, outlet };
struct ExhaustNode final {
    std::uint32_t id {};
    ExhaustNodeType type { ExhaustNodeType::pipe };
    double lengthMm { 0.0 };
    double diameterMm { 42.0 };
    double restriction { 0.0 };
    double resonanceHz { 0.0 };
    double audioGain { 1.0 };
    std::uint32_t pathIndex { 0 };
};
struct ExhaustEdge final { std::uint32_t from {}; std::uint32_t to {}; };

/** Directed exhaust topology shared by back-pressure and acoustic event propagation. */
class ExhaustGraph final : public IExhaustModel {
public:
    [[nodiscard]] static ExhaustGraph makeForEngine(const EngineConfig&);
    [[nodiscard]] double backPressureKpa(const EngineState&) const noexcept override;
    void process(FiringEvent&) const noexcept override;
    [[nodiscard]] const std::vector<ExhaustNode>& nodes() const noexcept { return nodes_; }
private:
    std::vector<ExhaustNode> nodes_;
    std::vector<ExhaustEdge> edges_;
    double effectiveRestriction_ { 0.0 };
    double ambientPressureKpa_ { 101.325 };
};
} // namespace enginelab
