#pragma once

#include <enginelab/audio/OfflineAudioExporter.hpp>
#include <enginelab/audio/RealtimeEngineAudio.hpp>
#include <enginelab/ecu/SimpleEcuModel.hpp>
#include <enginelab/events/FourStrokeEventGenerator.hpp>
#include <enginelab/exhaust/ExhaustGraph.hpp>
#include <enginelab/physics/SimplifiedGasolinePhysics.hpp>
#include <enginelab/runtime/EngineRuntime.hpp>
#include <enginelab/simulation/EngineSimulator.hpp>

#include <cstdint>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace enginelab::offline {

/** The offline renders' frame rate: the application's 240 Hz outer frame. */
inline constexpr double simulationRateHz = 240.0;

/** Telemetry the queues could not take, counted over a render. */
struct DroppedTelemetry final {
    std::uint64_t firingEvents { 0 };
    std::uint64_t pressureSamples { 0 };
    std::uint64_t exhaustAcousticSamples { 0 };
};

/** The shipping chain an offline render runs: the simulator, its telemetry
 *  queues and the realtime renderer, wired as the application wires them.
 *  The HQ export and the loop pack share it, so they cannot drift apart.
 *  It holds an EngineRuntime (~7.4 MB): build it with std::make_unique. */
class RenderChain final {
public:
    /** `config` is the normalised engine the simulator runs. */
    RenderChain(const EngineConfig& config, const OfflineAudioMix& mix);
    RenderChain(const RenderChain&) = delete;
    RenderChain& operator=(const RenderChain&) = delete;

    /** Load the engine's authored impulse responses into the renderer;
     *  relative paths resolve against `assetRoot`. */
    void loadAuthoredImpulseResponses(
        const EngineConfig& config,
        const std::filesystem::path& assetRoot,
        std::vector<std::string>& warnings,
        std::size_t& authored, std::size_t& loaded);

    /** Hand one simulator frame's firing events, pressure and exhaust
     *  samples to the renderer, re-timed onto the render clock
     *  `realtimeSeconds`, and publish the frame's state with this driveline
     *  load (see AudioFramePublication). */
    void forward(
        const SimulationFrame& frame, bool starterEngaged,
        double drivelineLoad, double realtimeSeconds, double dt,
        DroppedTelemetry& dropped);

    [[nodiscard]] EngineSimulator& simulator() noexcept { return *simulator_; }
    [[nodiscard]] RealtimeEngineAudio& renderer() noexcept { return *renderer_; }

private:
    SimpleEcuModel ecu_;
    SimplifiedGasolinePhysics physics_;
    FourStrokeEventGenerator events_;
    ExhaustGraph exhaust_;
    std::unique_ptr<EngineSimulator> simulator_;
    std::unique_ptr<FiringEventQueue> eventQueue_;
    std::unique_ptr<CylinderPressureQueue> pressureQueue_;
    std::unique_ptr<ExhaustAcousticQueue> exhaustAcousticQueue_;
    std::unique_ptr<EngineRuntime> audioConfiguration_;
    std::unique_ptr<RealtimeEngineAudio> renderer_;
};

} // namespace enginelab::offline
