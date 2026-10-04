#include <enginelab/app/DashboardModel.hpp>

#include <enginelab/app/Theme.hpp>
#include <algorithm>
#include <cmath>

namespace enginelab {

void TelemetryRing::push(const EngineState& state) noexcept {
    samples_[write_] = state;
    write_ = (write_ + 1) % capacity;
    count_ = std::min(count_ + 1, capacity);
}

const EngineState& TelemetryRing::at(std::size_t index) const noexcept {
    return samples_[(write_ + capacity - count_ + index) % capacity];
}

const DynoCurvePresentation* DashboardModel::presentationFor(std::uint64_t runId) const {
    const auto found = presentations.find(runId);
    return found != presentations.end() ? &found->second : nullptr;
}

const DynoRun* DashboardModel::selectedArchivedRun() const {
    return selectedRun >= 0 && selectedRun < static_cast<int>(archivedRuns.size())
        ? &archivedRuns[static_cast<std::size_t>(selectedRun)] : nullptr;
}

namespace ui {

const char* runningStateName(RunningState state) noexcept {
    switch (state) {
    case RunningState::stopped: return "STOPPED";
    case RunningState::cranking: return "CRANKING";
    case RunningState::idling: return "IDLING";
    case RunningState::running: return "RUNNING";
    case RunningState::unstable: return "UNSTABLE";
    case RunningState::knocking: return "KNOCKING";
    case RunningState::overheating: return "OVERHEATING";
    case RunningState::damaged: return "DAMAGED";
    case RunningState::destroyed: return "DESTROYED";
    }
    return "UNKNOWN";
}

const char* layoutName(EngineLayout layout) noexcept {
    switch (layout) {
    case EngineLayout::inlineLayout: return "INLINE";
    case EngineLayout::vLayout: return "V";
    case EngineLayout::flat: return "FLAT";
    case EngineLayout::radial: return "RADIAL";
    case EngineLayout::custom: return "CUSTOM";
    }
    return "CUSTOM";
}

juce::String gearName(int gear) {
    if (gear == -2) return "R";
    if (gear == -1) return "N";
    return juce::String(gear + 1);
}

const char* dynoModeToken(DynoMode mode) noexcept {
    switch (mode) {
    case DynoMode::steppedCalibration: return "stepped_calibration";
    case DynoMode::continuousRamp: return "continuous_ramp";
    case DynoMode::hold: return "hold";
    }
    return "unknown";
}

const char* dynoStatusToken(DynoRunStatus status) noexcept {
    switch (status) {
    case DynoRunStatus::idle: return "idle";
    case DynoRunStatus::running: return "running";
    case DynoRunStatus::completed: return "completed";
    case DynoRunStatus::cancelled: return "cancelled";
    case DynoRunStatus::timedOut: return "timed_out";
    case DynoRunStatus::invalid: return "invalid";
    }
    return "unknown";
}

const char* dynoStatusLabel(DynoRunStatus status) noexcept {
    switch (status) {
    case DynoRunStatus::idle: return "IDLE";
    case DynoRunStatus::running: return "RUNNING";
    case DynoRunStatus::completed: return "COMPLETED";
    case DynoRunStatus::cancelled: return "CANCELLED";
    case DynoRunStatus::timedOut: return "TIMEOUT";
    case DynoRunStatus::invalid: return "INVALID";
    }
    return "UNKNOWN";
}

const char* dynoStopReasonToken(DynoStopReason reason) noexcept {
    switch (reason) {
    case DynoStopReason::none: return "none";
    case DynoStopReason::sweepCeilingReached: return "sweep_ceiling_reached";
    case DynoStopReason::operatorFinished: return "operator_finished";
    case DynoStopReason::operatorCancelled: return "operator_cancelled";
    case DynoStopReason::startupTimeout: return "startup_timeout";
    case DynoStopReason::acquisitionTimeout: return "acquisition_timeout";
    case DynoStopReason::runtimeStopped: return "runtime_stopped";
    case DynoStopReason::engineReconfigured: return "engine_reconfigured";
    case DynoStopReason::insufficientValidData: return "insufficient_valid_data";
    case DynoStopReason::controllerFailure: return "controller_failure";
    }
    return "unknown";
}

juce::String groupedInteger(double value) {
    const auto rounded = static_cast<long long>(std::llround(value));
    auto digits = juce::String(std::llabs(rounded));
    juce::String grouped;
    const auto thinSpace = utf8("\xe2\x80\x89");
    for (int index = 0; index < digits.length(); ++index) {
        if (index != 0 && (digits.length() - index) % 3 == 0) grouped << thinSpace;
        grouped << digits[index];
    }
    return rounded < 0 ? "-" + grouped : grouped;
}

juce::String engineSpecLine(const EngineConfig& config) {
    const auto count = static_cast<int>(config.cylinders.size());
    juce::String layout;
    switch (config.layout) {
    case EngineLayout::inlineLayout:
        layout = count == 1 ? juce::String("Single") : (count == 2 ? juce::String("Parallel twin")
                                                                  : "Inline-" + juce::String(count));
        break;
    case EngineLayout::vLayout: {
        layout = "V" + juce::String(count);
        if (config.banks.size() >= 2) {
            const auto included = std::abs(config.banks[1].angleDegrees - config.banks[0].angleDegrees);
            if (included > 0.5) layout << " " << fixed(included, 0) << utf8("\xc2\xb0");
        }
        break;
    }
    case EngineLayout::flat: layout = "Flat-" + juce::String(count); break;
    case EngineLayout::radial: layout = "Radial-" + juce::String(count); break;
    case EngineLayout::custom: layout = juce::String(count) + "-cylinder"; break;
    }
    const auto separator = utf8("  \xc2\xb7  ");
    auto line = layout + separator
        + groupedInteger(engineDisplacementLitres(config) * 1'000.0) + utf8(" cm\xc2\xb3");
    if (!config.cylinders.empty()) {
        const auto& first = config.cylinders.front();
        line << separator << juce::String(first.boreMm, first.boreMm == std::round(first.boreMm) ? 0 : 1)
             << utf8(" \xc3\x97 ")
             << juce::String(first.strokeMm, first.strokeMm == std::round(first.strokeMm) ? 0 : 1)
             << " mm";
    }
    if (config.forcedInduction.enabled)
        line << separator << (config.forcedInduction.type == ForcedInductionType::turbocharger
                                  ? "turbo" : "supercharged");
    if (config.fuel == FuelType::diesel) line << separator << "diesel";
    return line;
}

} // namespace ui
} // namespace enginelab
