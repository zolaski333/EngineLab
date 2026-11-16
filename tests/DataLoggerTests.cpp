// The datalogger writes one CSV row per pushed state, no closer than its
// interval in simulated time, with as many fields as its header.

#include <enginelab/runtime/DataLogger.hpp>

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <string_view>
#include <vector>

namespace {
using namespace enginelab;

void require(bool condition, std::string_view message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(EXIT_FAILURE);
    }
}

[[nodiscard]] std::vector<std::string> lines(const std::filesystem::path& file) {
    std::ifstream stream(file);
    std::vector<std::string> result;
    for (std::string line; std::getline(stream, line);) result.push_back(line);
    return result;
}

[[nodiscard]] std::size_t fields(const std::string& line) {
    return static_cast<std::size_t>(std::count(line.begin(), line.end(), ',')) + 1;
}

} // namespace

int main() {
    const auto file = std::filesystem::temp_directory_path() / "enginelab-datalogger-test" / "log.csv";
    std::filesystem::remove(file);
    DataLogger logger;
    require(logger.start(file, 0.02), "the log opens, its folder created");
    require(logger.recording(), "and records");

    EngineState state;
    state.simulationTimeSeconds = 10.0;
    state.rpm = 3'000.0;
    state.throttle = 0.5;
    state.manifoldPressureKpa = 80.0;
    state.gear = 2;
    state.vehicleSpeedMps = 10.0;
    state.activeLimits = static_cast<std::uint32_t>(ActiveLimit::revLimiter)
        | static_cast<std::uint32_t>(ActiveLimit::knockRetard);
    logger.push(state);
    state.simulationTimeSeconds = 10.01;
    logger.push(state);
    state.simulationTimeSeconds = 10.05;
    state.activeLimits = 0U;
    logger.push(state);
    // A restarted engine: its clock starts again, the log's goes on.
    state.simulationTimeSeconds = 0.5;
    logger.push(state);
    logger.stop();
    require(!logger.recording() && logger.rows() == 3, "a row 10 ms after the last is skipped");

    const auto written = lines(file);
    require(written.size() == 4, "a header and three rows");
    require(written[3].rfind("0.070,", 0) == 0, "after a restart the log's time goes on, one interval later");
    require(written[0] == DataLogger::header(), "the header first");
    require(fields(written[1]) == fields(written[0]) && fields(written[2]) == fields(written[0]),
        "every row has the header's fields");
    require(written[1].rfind("0.000,3000,50.0,80.0,", 0) == 0, "time from the start of the log, then the channels");
    require(written[1].find(",36.0,2,") != std::string::npos, "road speed in km/h, then the gear");
    require(written[1].substr(written[1].rfind(',') + 1) == "rev_limiter+knock_retard", "the active limits by name");
    require(written[2].rfind("0.050,", 0) == 0 && written[2].substr(written[2].rfind(',') + 1) == "none",
        "nothing holding the engine is 'none'");
    // An engine changed a second into the log, its clock back at 0: the
    // origin went negative and was taken for "not started", so the time
    // jumped back to 0 (audit RT2-01).
    require(logger.start(file, 0.02), "a second log opens");
    for (const auto seconds : { 100.0, 101.0, 0.0, 0.02, 0.04 }) {
        state.simulationTimeSeconds = seconds;
        logger.push(state);
    }
    logger.stop();
    const auto rewound = lines(file);
    require(rewound.size() == 6, "a header and five rows");
    const char* const expected[] { "0.000,", "1.000,", "1.020,", "1.040,", "1.060," };
    for (std::size_t index = 0; index < 5; ++index)
        require(rewound[index + 1].rfind(expected[index], 0) == 0,
                std::string("the log's time goes on across the restart: row ") + std::to_string(index + 1)
                    + " reads " + rewound[index + 1].substr(0, 6) + ", not " + expected[index]);
    std::filesystem::remove_all(file.parent_path());
    std::cout << "datalogger: ok\n";
    return 0;
}
