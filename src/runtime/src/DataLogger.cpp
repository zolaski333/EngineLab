#include <enginelab/runtime/DataLogger.hpp>

#include <algorithm>
#include <array>
#include <cstdio>
#include <system_error>
#include <utility>

namespace enginelab {
namespace {

/** The active limits as names joined by '+', "none" when nothing holds. */
[[nodiscard]] std::string limitNames(std::uint32_t limits) {
    constexpr std::array<std::pair<ActiveLimit, const char*>, 8> names { {
        { ActiveLimit::launchControl, "launch" },
        { ActiveLimit::revLimiter, "rev_limiter" },
        { ActiveLimit::knockRetard, "knock_retard" },
        { ActiveLimit::coolantProtection, "hot_coolant" },
        { ActiveLimit::dieselSmokeLimit, "smoke_limit" },
        { ActiveLimit::dieselQuantityLimit, "quantity_map" },
        { ActiveLimit::boostAtTarget, "boost_at_target" },
        { ActiveLimit::injectorSaturated, "injectors_saturated" },
    } };
    std::string text;
    for (const auto& [limit, name] : names) {
        if (!hasActiveLimit(limits, limit)) continue;
        if (!text.empty()) text += '+';
        text += name;
    }
    return text.empty() ? std::string("none") : text;
}

void append(std::string& line, double value, int decimals) {
    std::array<char, 48> buffer {};
    const auto written = std::snprintf(buffer.data(), buffer.size(), ",%.*f", decimals, value);
    if (written > 0) line.append(buffer.data(), static_cast<std::size_t>(written));
}

} // namespace

DataLogger::~DataLogger() { stop(); }

bool DataLogger::start(const std::filesystem::path& file, double minimumIntervalSeconds) {
    stop();
    std::error_code error;
    if (file.has_parent_path()) std::filesystem::create_directories(file.parent_path(), error);
    stream_.open(file, std::ios::out | std::ios::trunc);
    if (!stream_.is_open()) return false;
    file_ = file;
    rows_ = 0;
    minimumIntervalSeconds_ = std::max(0.0, minimumIntervalSeconds);
    startSeconds_ = -1.0;
    stream_ << header() << '\n';
    return static_cast<bool>(stream_);
}

void DataLogger::push(const EngineState& state) {
    if (!stream_.is_open()) return;
    const auto now = state.simulationTimeSeconds;
    if (startSeconds_ < 0.0) {
        startSeconds_ = now;
    } else if (now < lastSeconds_) {
        // A restarted engine starts its clock again: the log's time goes on.
        startSeconds_ = now - (lastSeconds_ - startSeconds_) - minimumIntervalSeconds_;
    } else if (now - lastSeconds_ < minimumIntervalSeconds_) {
        return;
    }
    lastSeconds_ = now;
    stream_ << row(state, now - startSeconds_) << '\n';
    ++rows_;
}

void DataLogger::stop() {
    if (!stream_.is_open()) return;
    stream_.flush();
    stream_.close();
}

std::string DataLogger::header() {
    return "time_s,rpm,throttle_pct,map_kpa,boost_ratio,lambda,advance_deg,knock,torque_nm,power_kw,"
           "speed_kmh,gear,clutch,brake,coolant_c,oil_c,egt_c,turbo_krpm,wastegate,peak_cyl_bar,limits";
}

std::string DataLogger::row(const EngineState& state, double timeSeconds) {
    std::string line;
    line.reserve(200);
    std::array<char, 32> time {};
    const auto written = std::snprintf(time.data(), time.size(), "%.3f", timeSeconds);
    if (written > 0) line.append(time.data(), static_cast<std::size_t>(written));
    append(line, state.rpm, 0);
    append(line, state.throttle * 100.0, 1);
    append(line, state.manifoldPressureKpa, 1);
    append(line, state.boostPressureRatio, 3);
    append(line, state.lambda, 3);
    append(line, state.ignitionAdvanceDegrees, 1);
    append(line, state.knockLevel, 3);
    append(line, state.torqueNm, 1);
    append(line, state.powerKw, 2);
    append(line, state.vehicleSpeedMps * 3.6, 1);
    line += ',' + std::to_string(state.gear);
    append(line, state.clutchPressure, 2);
    append(line, state.brakePressure, 2);
    append(line, state.coolantTemperatureC, 1);
    append(line, state.oilTemperatureC, 1);
    append(line, state.exhaustTemperatureC, 0);
    append(line, state.forcedInductionShaftSpeedRpm * 0.001, 1);
    append(line, state.wastegateOpening, 2);
    append(line, state.peakCylinderPressureBar, 1);
    line += ',' + limitNames(state.activeLimits);
    return line;
}

} // namespace enginelab
