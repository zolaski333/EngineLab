#pragma once
#include <enginelab/exhaust/IExhaustModel.hpp>
#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>
namespace enginelab {

inline constexpr std::size_t maximumExhaustAcousticModes = 8;

/** One resonance retained by the topology compiler.
 *
 * relativeEnergy is normalised within its containing route or cylinder. It is
 * an acoustic energy share, not an amplitude gain.
 */
struct ExhaustAcousticMode final {
    double frequencyHz { 0.0 };
    double relativeEnergy { 0.0 };
};

enum class ExhaustNodeType : std::uint8_t {
    port, pipe, merge, splitter, resonator, muffler, catalyst, outlet,
    crossover
};
struct ExhaustNode final {
    std::uint32_t id {};
    ExhaustNodeType type { ExhaustNodeType::pipe };
    double lengthMm { 0.0 };
    double diameterMm { 42.0 };
    double restriction { 0.0 };
    double resonanceHz { 0.0 };
    double audioGain { 1.0 };
    std::uint32_t pathIndex { 0 };
    // Zero for generated legacy nodes; otherwise the path-local config ID.
    std::uint32_t sourceComponentId { 0 };
    // Relative coupling of this component's local resonance. Route-length
    // modes are compiled separately and therefore do not use this value.
    double resonanceStrength { 0.0 };
    // Physical metadata retained for the component-resolved gas network.
    // volumeLitres is explicit when authored, otherwise it is the geometric
    // volume of the compiled component.
    double volumeLitres { 0.0 };
    double dischargeCoefficient { 1.0 };
    // Authored concentrated loss only. `restriction` above is the legacy
    // reduced K (geometry + local loss) and must not be reused by a duct that
    // already resolves Darcy friction from its dimensions.
    double localLossCoefficient { 0.0 };
    AcousticPoint3M acousticPositionM {};
    AcousticPoint3M acousticAxis { 0.0, 1.0, 0.0 };
    AcousticTerminationType acousticTermination {
        AcousticTerminationType::unflanged };
    /** Optional component outlet diameter; zero means diameterMm at both ends. */
    double outletDiameterMm { 0.0 };
    double packingFlowResistivityPaSPerM2 { 0.0 };
    double packingThicknessMm { 0.0 };
    double perforatedOpenAreaRatio { 0.0 };
    double catalystCellDensityCpsi { 0.0 };
    double catalystOpenAreaRatio { 0.0 };
    double catalystSubstrateVolumetricHeatCapacityJPerM3K { 0.0 };
    double crossoverCoupling { 0.0 };
};
struct ExhaustEdge final {
    std::uint32_t from {};
    std::uint32_t to {};
    std::uint8_t fromPort { unspecifiedExhaustComponentPort };
    std::uint8_t toPort { unspecifiedExhaustComponentPort };
};

/** Acoustic-only terminal branch attached to a mean-flow component.
 *
 * The main gas DAG deliberately excludes this sealed branch: it carries no
 * steady exhaust flow. The audio network compiles it as a side duct with a
 * rigid end or an optional terminal cavity compliance. A non-zero tuning
 * frequency represents a measured/folded acoustic length at the graph's
 * reference gas temperature; zero uses the authored geometric length.
 */
struct ExhaustAcousticSideBranch final {
    std::uint32_t attachmentNodeId { 0 };
    std::uint32_t sourceComponentId { 0 };
    std::uint32_t pathIndex { 0 };
    double lengthM { 0.0 };
    double diameterM { 0.0 };
    double terminalVolumeM3 { 0.0 };
    double referenceTuningHz { 0.0 };
};

/** A condition the topology compiler had to work around.
 *
 * The compiler always yields a usable graph so the audio and solver paths stay
 * safe, but every fallback it takes silently discards or truncates authored
 * intent. Emitting them lets the application tell the user why their exhaust
 * does not sound like what they drew, instead of substituting a default.
 */
enum class ExhaustCompileIssue : std::uint8_t {
    /// Cylinders were not covered exactly once; the authored paths were replaced
    /// by a single generated fallback path. relatedId is the offending cylinder.
    topologyRejected,
    /// maximumCompiledRoutes was reached; further routes were not compiled.
    routeLimitReached,
    /// The generated node-ID space was exhausted. relatedId is 0.
    nodeIdSpaceExhausted,
    /// A node has no finite equivalent restriction (dangling branch or cycle);
    /// it was charged the maximum. relatedId is the cylinder whose route failed.
    unresolvedRestriction,
    /// More acoustic-only branches were authored than the realtime bound;
    /// extras were omitted. relatedId is the first omitted component.
    acousticBranchLimitReached,
};

struct ExhaustCompileDiagnostic final {
    ExhaustCompileIssue issue { ExhaustCompileIssue::topologyRejected };
    std::uint32_t relatedId { 0 };
};

/** Precomputed metrics for one cylinder-to-outlet route through the DAG. */
struct ExhaustRoute final {
    std::uint32_t cylinderId { 0 };
    std::uint32_t pathIndex { 0 };
    std::uint32_t outletNodeId { 0 };
    double lengthMm { 0.0 };
    double delaySeconds { 0.0 };
    double restriction { 0.0 };
    double resonanceHz { 0.0 };
    double audioGain { 1.0 };
    std::size_t modeCount { 0 };
    std::array<ExhaustAcousticMode, maximumExhaustAcousticModes> modes {};
};

/** Energy-combined acoustic metrics for all routes from one cylinder. */
struct ExhaustCylinderAcoustics final {
    std::uint32_t cylinderId { 0 };
    std::uint32_t pathIndex { 0 };
    std::size_t routeCount { 0 };
    double meanLengthMm { 0.0 };
    // Delay of the highest-energy route (shortest route on an exact tie).
    // This is the best causal scalar proxy for consumers that cannot render
    // the individually retained routes.
    double delaySeconds { 0.0 };
    double resonanceHz { 0.0 };
    double equivalentRestriction { 0.0 };
    // Includes path volume, cylinder/component acoustic gains, splitter
    // energy distribution and restriction attenuation exactly once.
    double transmissionGain { 1.0 };
    // Appended diagnostics preserve positional aggregate initialisation of
    // the original scalar API.
    double meanDelaySeconds { 0.0 };
    double firstArrivalDelaySeconds { 0.0 };
    double lastArrivalDelaySeconds { 0.0 };
    double rmsDelaySpreadSeconds { 0.0 };
    std::size_t modeCount { 0 };
    std::array<ExhaustAcousticMode, maximumExhaustAcousticModes> modes {};
};

/** Directed mean-flow topology plus explicitly separated acoustic-only branches. */
class ExhaustGraph final : public IExhaustModel {
public:
    [[nodiscard]] static ExhaustGraph makeForEngine(const EngineConfig&);
    /** Compile with an explicit representative exhaust-gas temperature.
     *
     * The graph is static, so callers with a measured or simulated design
     * temperature can use this overload. The legacy overload selects a
     * conservative nominal temperature derived from ambient conditions.
     */
    [[nodiscard]] static ExhaustGraph makeForEngine(
        const EngineConfig&, double referenceExhaustTemperatureC);
    [[nodiscard]] double backPressureKpa(const EngineState&) const noexcept override;
    void process(FiringEvent&) const noexcept override;
    [[nodiscard]] const std::vector<ExhaustNode>& nodes() const noexcept { return nodes_; }
    [[nodiscard]] const std::vector<ExhaustEdge>& edges() const noexcept { return edges_; }
    [[nodiscard]] const std::vector<ExhaustAcousticSideBranch>&
    acousticSideBranches() const noexcept { return acousticSideBranches_; }
    [[nodiscard]] const std::vector<ExhaustRoute>& routes() const noexcept { return routes_; }
    [[nodiscard]] const AcousticObserverConfig& acousticObserver() const noexcept {
        return acousticObserver_;
    }
    [[nodiscard]] double effectiveRestriction() const noexcept { return effectiveRestriction_; }
    [[nodiscard]] double referenceWaveSpeedMps() const noexcept {
        return waveSpeedMmPerSecond_ * 0.001;
    }
    [[nodiscard]] ExhaustCylinderAcoustics acousticsForCylinder(
        std::uint32_t cylinderId) const noexcept;
    [[nodiscard]] ExhaustPathFlowProperties pathFlowProperties(
        std::size_t pathIndex) const noexcept override;
    [[nodiscard]] ExhaustCylinderFlowProperties cylinderFlowProperties(
        std::uint32_t cylinderId) const noexcept override;
    /** Conditions the compiler worked around. Empty means the authored topology
     *  was compiled as written. */
    [[nodiscard]] const std::vector<ExhaustCompileDiagnostic>& diagnostics() const noexcept {
        return diagnostics_;
    }
private:
    struct CylinderRestriction final {
        std::uint32_t cylinderId { 0 };
        std::uint32_t pathIndex { 0 };
        double equivalentRestriction { 0.0 };
    };
    std::vector<ExhaustNode> nodes_;
    std::vector<ExhaustEdge> edges_;
    std::vector<ExhaustAcousticSideBranch> acousticSideBranches_;
    std::vector<ExhaustRoute> routes_;
    std::vector<CylinderRestriction> cylinderRestrictions_;
    std::vector<ExhaustPathFlowProperties> pathFlowProperties_;
    std::vector<ExhaustCylinderFlowProperties> cylinderFlowProperties_;
    std::vector<ExhaustCompileDiagnostic> diagnostics_;
    double effectiveRestriction_ { 0.0 };
    double legacyEffectiveRestriction_ { 0.0 };
    double ambientPressureKpa_ { 101.325 };
    double waveSpeedMmPerSecond_ { 520'000.0 };
    AcousticObserverConfig acousticObserver_;
};
} // namespace enginelab
