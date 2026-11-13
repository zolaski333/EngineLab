#pragma once

#include <enginelab/foundation/EngineTypes.hpp>

#include <cstdint>
#include <optional>
#include <string>

namespace enginelab {

/** Result of one transactional exhaust-path ownership edit. */
struct ExhaustPathEditResult final {
    std::uint32_t selectedPathId { 0 };
    std::string error;
    /** What the edit did, when it is not what was asked (a fallback). */
    std::string note;

    [[nodiscard]] explicit operator bool() const noexcept { return error.empty(); }
};

/** How `removePath` joins the removed path's cylinders into the destination. */
enum class ExhaustPathJoin : std::uint8_t {
    /** Both collectors kept: a leg from each into a new Y, then the
     *  destination's own pipe on (shortened by the leg, so the length to its
     *  silencer stays); the removed path's pipes past its collector go. */
    yJunction,
    /** One clean N-into-1: every cylinder gets a primary like the
     *  destination's, all into the destination's collector. */
    rebuild,
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

    /** The moved cylinder gets a copy of a neighbour's primary into the same
     *  collector; a destination with no collector gets one after its pipe. */
    [[nodiscard]] static ExhaustPathEditResult moveCylinder(
        EngineConfig& config, std::uint32_t cylinderId,
        std::uint32_t sourcePathId, std::uint32_t destinationPathId);

    /** Removes `pathId`, its cylinders joining `destinationPathId` as `join`
     *  says. Where a path's cylinders never meet in one junction (an H or X
     *  inside it, twin outlets), each cylinder joins the collector its
     *  nearest neighbour feeds instead, and `note` says so. */
    [[nodiscard]] static ExhaustPathEditResult removePath(
        EngineConfig& config, std::uint32_t pathId, std::uint32_t destinationPathId,
        ExhaustPathJoin join = ExhaustPathJoin::rebuild);
};

/** Local edits of one path's custom graph, shared by the designer and tests.
 *  None of them validates: the designer applies only a valid engine. */
class ExhaustNetworkEditor final {
public:
    /** A component of `type` with a neutral starting geometry. */
    [[nodiscard]] static ExhaustComponentConfig defaultComponent(ExhaustComponentType type,
                                                                 std::uint32_t id) noexcept;

    /** The lowest unused component id (0 if none is left). */
    [[nodiscard]] static std::uint32_t nextComponentId(const ExhaustNetworkConfig& network);

    /** The first component every mapped cylinder reaches (where they all
     *  meet: a collector, or a single cylinder's own pipe). */
    [[nodiscard]] static std::optional<std::uint32_t> firstCommonJunction(const ExhaustNetworkConfig& network);

    /** The cylinder now feeds `componentId` (its old mapping, if any, goes).
     *  Empty, or why not. */
    [[nodiscard]] static std::string connectCylinder(ExhaustNetworkConfig& network, std::uint32_t cylinderId,
                                                     std::uint32_t componentId);

    /** The cylinder feeds nothing; false if it fed nothing. */
    static bool disconnectCylinder(ExhaustNetworkConfig& network, std::uint32_t cylinderId);

    /** `from` -> `to`, an X-pipe's next free port where one is involved.
     *  Empty, or why not. */
    [[nodiscard]] static std::string connectComponents(ExhaustNetworkConfig& network, std::uint32_t from,
                                                       std::uint32_t to);

    /** The component becomes a `type`, with that type's starting values but
     *  its id, its bore, its length where both types have one, and every
     *  connection. Empty, or why not. */
    [[nodiscard]] static std::string replaceComponent(ExhaustNetworkConfig& network, std::uint32_t componentId,
                                                      ExhaustComponentType type);

    /** A silencer bypass with an active valve, in one click: the component
     *  (a silencer, or any part with one part in and one out) gets a splitter
     *  before it, a merge after it and, between them, a straight pipe of its
     *  length and bore with a valve at its inlet. Shut, the gas goes through
     *  the component and the bypass is a closed branch; open, most of it
     *  takes the bypass. Empty, or why not; `bypassPipeId` gets the pipe. */
    [[nodiscard]] static std::string addValvedBypass(ExhaustNetworkConfig& network, std::uint32_t componentId,
                                                     std::uint32_t* bypassPipeId = nullptr);

    /** The Equipment window's "Fit": each exhaust path's silencer nearest its
     *  tailpipe gets the valved bypass of addValvedBypass (a path drawn from
     *  scalar fields first becomes the network it was drawn from). A path
     *  without a silencer, or whose silencer cannot take one, is left as it
     *  is. Empty when at least one valve was fitted, or why none was. */
    [[nodiscard]] static std::string fitSilencerBypass(EngineConfig& config);
};

} // namespace enginelab
