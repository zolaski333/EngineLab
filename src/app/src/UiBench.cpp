#include <enginelab/app/UiBench.hpp>

#include <algorithm>
#include <ctime>

#if JUCE_WINDOWS
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

namespace enginelab::ui {

std::optional<UiBenchSettings> parseUiBench(const juce::StringArray& arguments, juce::String& error) {
    error.clear();
    if (!arguments.contains("--ui-bench")) return std::nullopt;
    UiBenchSettings settings;
    for (int index = 0; index < arguments.size(); ++index) {
        const auto& option = arguments[index];
        if (option == "--ui-bench") continue;
        const auto hasValue = index + 1 < arguments.size();
        const auto value = hasValue ? arguments[index + 1].unquoted() : juce::String();
        const auto take = [&] {
            if (!hasValue) error = option + " needs a value";
            ++index;
            return value;
        };
        if (option == "--engine") settings.engine = take();
        else if (option == "--throttle") settings.throttle = std::clamp(take().getDoubleValue(), 0.0, 1.0);
        else if (option == "--warmup") settings.warmupSeconds = std::max(1.0, take().getDoubleValue());
        else if (option == "--seconds") settings.measureSeconds = std::max(1.0, take().getDoubleValue());
        else if (option == "--width") settings.width = std::max(640, take().getIntValue());
        else if (option == "--height") settings.height = std::max(480, take().getIntValue());
        else if (option == "--scale") settings.scale = static_cast<float>(std::clamp(take().getDoubleValue(), 0.5, 3.0));
        else if (option == "--tab") settings.tab = take().toLowerCase();
        else if (option == "--open") settings.open = take().toLowerCase();
        else if (option == "--menu") settings.menu = take().toLowerCase();
        else if (option == "--dyno") settings.dyno = true;
        else if (option == "--out") settings.output = juce::File(take());
        else if (option == "--capture-marker") settings.captureMarker = juce::File(take());
        else if (option == "--capture-hold") settings.captureHoldSeconds = std::max(0.0, take().getDoubleValue());
        else error = "unknown option " + option;
        if (error.isNotEmpty()) return std::nullopt;
    }
    return settings;
}

double currentThreadCpuSeconds() noexcept {
#if JUCE_WINDOWS
    FILETIME created {}, exited {}, kernel {}, user {};
    if (!GetThreadTimes(GetCurrentThread(), &created, &exited, &kernel, &user)) return 0.0;
    const auto ticks = [](const FILETIME& time) {
        return (static_cast<unsigned long long>(time.dwHighDateTime) << 32U) | time.dwLowDateTime;
    };
    return static_cast<double>(ticks(kernel) + ticks(user)) * 1.0e-7;
#else
    timespec time {};
    if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &time) != 0) return 0.0;
    return static_cast<double>(time.tv_sec) + static_cast<double>(time.tv_nsec) * 1.0e-9;
#endif
}

double processCpuSeconds() noexcept {
#if JUCE_WINDOWS
    FILETIME created {}, exited {}, kernel {}, user {};
    if (!GetProcessTimes(GetCurrentProcess(), &created, &exited, &kernel, &user)) return 0.0;
    const auto ticks = [](const FILETIME& time) {
        return (static_cast<unsigned long long>(time.dwHighDateTime) << 32U) | time.dwLowDateTime;
    };
    return static_cast<double>(ticks(kernel) + ticks(user)) * 1.0e-7;
#else
    timespec time {};
    if (clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &time) != 0) return 0.0;
    return static_cast<double>(time.tv_sec) + static_cast<double>(time.tv_nsec) * 1.0e-9;
#endif
}

juce::String uiBenchCsv(const UiBenchSettings& settings, const juce::String& engineName,
                        const std::vector<UiBenchSample>& samples) {
    juce::String csv;
    csv << "# engine=" << engineName << " throttle=" << settings.throttle << " size=" << settings.width << "x"
        << settings.height << " scale=" << settings.scale << " tab=" << settings.tab << " open=" << settings.open
        << " dyno=" << (settings.dyno ? 1 : 0) << "\n";
    csv << "second,rpm,realtime_factor,view_fps,view_render_cpu_ms,message_thread_cpu_ms,process_cpu_ms\n";
    const auto row = [&csv](const juce::String& first, const UiBenchSample& sample) {
        csv << first << "," << juce::String(sample.rpm, 1) << "," << juce::String(sample.realtimeFactor, 4) << ","
            << juce::String(sample.viewFps, 2) << "," << juce::String(sample.viewRenderCpuMs, 3) << ","
            << juce::String(sample.messageThreadCpuMs, 2) << "," << juce::String(sample.processCpuMs, 1) << "\n";
    };
    for (const auto& sample : samples) row(juce::String(sample.wallSeconds, 1), sample);
    if (samples.empty()) return csv;
    const auto median = [&samples](double UiBenchSample::*field) {
        std::vector<double> values;
        values.reserve(samples.size());
        for (const auto& sample : samples) values.push_back(sample.*field);
        std::sort(values.begin(), values.end());
        const auto middle = values.size() / 2;
        return values.size() % 2 == 1 ? values[middle] : 0.5 * (values[middle - 1] + values[middle]);
    };
    UiBenchSample summary;
    summary.rpm = median(&UiBenchSample::rpm);
    summary.realtimeFactor = median(&UiBenchSample::realtimeFactor);
    summary.viewFps = median(&UiBenchSample::viewFps);
    summary.viewRenderCpuMs = median(&UiBenchSample::viewRenderCpuMs);
    summary.messageThreadCpuMs = median(&UiBenchSample::messageThreadCpuMs);
    summary.processCpuMs = median(&UiBenchSample::processCpuMs);
    row("median", summary);
    return csv;
}

} // namespace enginelab::ui
