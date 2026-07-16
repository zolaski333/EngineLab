#pragma once

#include <enginelab/foundation/EngineTypes.hpp>

#include <cstdint>
#include <string>

namespace enginelab {

/** Result of one transactional exhaust-path ownership edit. */
struct ExhaustPathEditResult final {
    std::uint32_t selectedPathId { 0 };
    std::string error;

    [[nodiscard]] explicit operator bool() const noexcept { return error.empty(); }
};

/**
 * Transactional path/cylinder operations shared by the UI and tests.
 *
 * A successful operation preserves a complete, valid EngineConfig: every
 * cylinder belongs to exactly one non-empty path and custom DAG mappings are
 * repaired without discarding the user's downstream components.
 */
class ExhaustPathTopologyEditor final {
public:
    [[nodiscard]] static ExhaustPathEditResult createPathFromCylinder(
        EngineConfig& config, std::uint32_t sourcePathId, std::uint32_t cylinderId);

    [[nodiscard]] static ExhaustPathEditResult moveCylinder(
        EngineConfig& config, std::uint32_t cylinderId,
        std::uint32_t sourcePathId, std::uint32_t destinationPathId);

    [[nodiscard]] static ExhaustPathEditResult removePath(
        EngineConfig& config, std::uint32_t pathId, std::uint32_t destinationPathId);
};

} // namespace enginelab
