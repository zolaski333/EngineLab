// The trackside microphone: a car's sound heard from 7.5 m off its lane
// while it drives past (PassByMicrophone). A pure tone checks the moving
// source's Doppler shift, the 1/r spreading, the ground's reflection and the
// air's absorption against their textbook values; switching it on and off
// must not click, and switched off it is the signal it was given.

#include <enginelab/audio/PassByMicrophone.hpp>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <numbers>
#include <string_view>
#include <vector>

namespace {

void require(bool condition, std::string_view message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(1);
    }
}

constexpr double rate = 48'000.0;

struct Run final {
    std::vector<float> left;
    std::vector<float> right;
};

/** `seconds` of a tone of `hertz` (left and right alike) through the pass-by. */
Run play(enginelab::PassByMicrophone& passBy, double hertz, double seconds) {
    Run run;
    const auto samples = static_cast<std::size_t>(seconds * rate);
    for (std::size_t index = 0; index < samples; ++index) {
        const auto tone = static_cast<float>(std::sin(2.0 * std::numbers::pi * hertz * index / rate));
        const auto heard = passBy.process(tone, tone);
        run.left.push_back(heard.leftPa);
        run.right.push_back(heard.rightPa);
    }
    return run;
}

/** Upward zero crossings over [first, last), interpolated: cycles per second. */
double frequency(const std::vector<float>& signal, std::size_t first, std::size_t last) {
    double firstCrossing = -1.0;
    double lastCrossing = -1.0;
    int crossings = 0;
    for (auto index = first + 1; index < last; ++index) {
        if (signal[index - 1] < 0.0F && signal[index] >= 0.0F) {
            const auto at = static_cast<double>(index - 1)
                + signal[index - 1] / (signal[index - 1] - signal[index]);
            if (firstCrossing < 0.0) firstCrossing = at;
            lastCrossing = at;
            ++crossings;
        }
    }
    return crossings > 1 ? (crossings - 1) * rate / (lastCrossing - firstCrossing) : 0.0;
}

double rms(const std::vector<float>& signal, std::size_t first, std::size_t last) {
    double sum = 0.0;
    for (auto index = first; index < last; ++index) sum += static_cast<double>(signal[index]) * signal[index];
    return std::sqrt(sum / static_cast<double>(last - first));
}

enginelab::PassBySettings bare() {
    enginelab::PassBySettings settings;
    settings.groundReflection = 0.0;
    settings.airAbsorption = false;
    settings.halfLengthM = 400.0;
    return settings;
}

void checkDoppler() {
    // A 500 Hz tone at 30 m/s (108 km/h), far up the lane and far down it.
    // The source's range rate at emission, v x_e / r_e, shifts it to
    // f0 / (1 + rate / c): about 548 Hz coming, 460 Hz going.
    auto settings = bare();
    settings.microphoneSpacingM = 0.0;
    enginelab::PassByMicrophone passBy;
    require(passBy.prepare(rate, settings), "the pass-by prepares");
    constexpr double speed = 30.0;
    constexpr double tone = 500.0;
    passBy.setVehicleSpeed(speed);
    passBy.setEnabled(true);
    passBy.reset(-300.0);
    const auto run = play(passBy, tone, 20.0);
    const auto c = settings.soundSpeedMps;
    const auto lateral = std::hypot(settings.laneDistanceM, settings.microphoneHeightM - settings.sourceHeightM);
    const auto expected = [&](double receivedAt) {
        // The car at x now; emitted tau ago at x - v tau (the class's root).
        const auto x = -300.0 + speed * receivedAt;
        const auto a = c * c - speed * speed;
        const auto tau = (-x * speed + std::sqrt(x * x * speed * speed + a * (x * x + lateral * lateral))) / a;
        const auto emitted = x - speed * tau;
        const auto rangeRate = speed * emitted / std::hypot(emitted, lateral);
        return tone / (1.0 + rangeRate / c);
    };
    const auto measure = [&](double from, double to) {
        return frequency(run.left, static_cast<std::size_t>(from * rate), static_cast<std::size_t>(to * rate));
    };
    const auto coming = measure(2.0, 2.25);
    const auto going = measure(17.0, 17.25);
    const auto comingExpected = expected(2.125);
    const auto goingExpected = expected(17.125);
    std::cout << "Doppler at 30 m/s on 500 Hz: coming " << coming << " Hz (expected " << comingExpected
              << "), going " << going << " Hz (expected " << goingExpected << ")\n";
    require(std::abs(coming / comingExpected - 1.0) < 2.0e-4, "the coming car's tone is shifted up as it must be");
    require(std::abs(going / goingExpected - 1.0) < 2.0e-4, "the going car's tone is shifted down as it must be");
    require(coming > 545.0 && going < 462.0, "about c / (c - v) and c / (c + v) far from the microphone");
}

void checkDistanceLaw() {
    // Standing still, the tone 10 m then 20 m from the left microphone: 1/r
    // is 6.02 dB per doubling (low enough for the air to take 0.003 dB).
    auto settings = bare();
    settings.airAbsorption = true;
    const auto lateral = std::hypot(settings.laneDistanceM, settings.microphoneHeightM - settings.sourceHeightM);
    const auto level = [&](double distance) {
        enginelab::PassByMicrophone passBy;
        require(passBy.prepare(rate, settings), "the pass-by prepares");
        passBy.setEnabled(true);
        passBy.reset(-0.5 * settings.microphoneSpacingM + std::sqrt(distance * distance - lateral * lateral));
        const auto run = play(passBy, 200.0, 0.8);
        return rms(run.left, static_cast<std::size_t>(0.3 * rate), static_cast<std::size_t>(0.8 * rate));
    };
    const auto ten = level(10.0);
    const auto twenty = level(20.0);
    const auto drop = 20.0 * std::log10(ten / twenty);
    std::cout << "10 m to 20 m: " << drop << " dB; at 10 m " << ten * std::sqrt(2.0) << " of a 1 Pa tone heard at "
              << settings.referenceDistanceM << " m\n";
    require(std::abs(drop - 20.0 * std::log10(2.0)) < 0.02, "6 dB per doubling of the distance");
    require(std::abs(ten * std::sqrt(2.0) - settings.referenceDistanceM / 10.0) < 0.002,
        "the level the engine's observer heard, scaled by its distance over 10 m");
}

void checkGround() {
    // Broadside (the car in front of the left microphone), the ground's
    // image path is 0.158 m longer: its first notch is c / (2 dr), the peak
    // above it twice that, in the ratio of (1/r1 - R/r2) to (1/r1 + R/r2).
    auto settings = bare();
    settings.groundReflection = 0.9;
    const auto r1 = std::hypot(settings.laneDistanceM, settings.microphoneHeightM - settings.sourceHeightM);
    const auto r2 = std::hypot(settings.laneDistanceM, settings.microphoneHeightM + settings.sourceHeightM);
    const auto notch = settings.soundSpeedMps / (2.0 * (r2 - r1));
    const auto level = [&](double hertz) {
        enginelab::PassByMicrophone passBy;
        require(passBy.prepare(rate, settings), "the pass-by prepares");
        passBy.setEnabled(true);
        passBy.reset(-0.5 * settings.microphoneSpacingM);
        const auto run = play(passBy, hertz, 0.6);
        return rms(run.left, static_cast<std::size_t>(0.2 * rate), static_cast<std::size_t>(0.6 * rate));
    };
    const auto ratio = level(notch) / level(2.0 * notch);
    const auto expected = (1.0 / r1 - 0.9 / r2) / (1.0 / r1 + 0.9 / r2);
    std::cout << "ground: notch at " << notch << " Hz, " << 20.0 * std::log10(ratio) << " dB under the peak (expected "
              << 20.0 * std::log10(expected) << ")\n";
    require(std::abs(ratio / expected - 1.0) < 0.03, "the ground's reflection digs its notch where it must");
}

void checkAir() {
    // The air over the 90 m between 10 m and 100 m against ISO 9613-1 at
    // 20 C and 70 % relative humidity (2 kHz 9.7, 4 kHz 32.8 dB/km).
    auto settings = bare();
    settings.airAbsorption = true;
    const auto lateral = std::hypot(settings.laneDistanceM, settings.microphoneHeightM - settings.sourceHeightM);
    const auto loss = [&](double hertz) {
        const auto level = [&](double distance) {
            enginelab::PassByMicrophone passBy;
            require(passBy.prepare(rate, settings), "the pass-by prepares");
            passBy.setEnabled(true);
            passBy.reset(-0.5 * settings.microphoneSpacingM + std::sqrt(distance * distance - lateral * lateral));
            const auto run = play(passBy, hertz, 0.8);
            return rms(run.left, static_cast<std::size_t>(0.5 * rate), static_cast<std::size_t>(0.8 * rate));
        };
        return 20.0 * std::log10(level(10.0) / level(100.0)) - 20.0;
    };
    const auto twoKilohertz = loss(2'000.0);
    const auto fourKilohertz = loss(4'000.0);
    std::cout << "air over 90 m: 2 kHz " << twoKilohertz << " dB (ISO 0.87), 4 kHz " << fourKilohertz
              << " dB (ISO 2.95)\n";
    require(std::abs(twoKilohertz - 0.873) < 0.3, "the air takes ISO 9613-1's share at 2 kHz");
    require(std::abs(fourKilohertz - 2.952) < 0.3, "the air takes ISO 9613-1's share at 4 kHz");
}

void checkSwitching() {
    // Off, the signal comes out as it went in, bit for bit. Switched on and
    // off on a 200 Hz tone, no sample steps further than the tone itself.
    enginelab::PassBySettings settings;
    enginelab::PassByMicrophone passBy;
    require(passBy.prepare(rate, settings), "the pass-by prepares");
    for (int index = 0; index < 1'000; ++index) {
        const auto value = static_cast<float>(std::sin(0.01 * index));
        const auto heard = passBy.process(value, -value);
        require(heard.leftPa == value && heard.rightPa == -value, "off, the pass-by is the signal it was given");
    }
    require(!passBy.active(), "off, it has nothing to do");
    std::vector<float> output;
    const auto largestStep = [&output](std::size_t first, std::size_t last) {
        double largest = 0.0;
        for (auto index = first + 1; index < last; ++index)
            largest = std::max(largest, std::abs(static_cast<double>(output[index]) - output[index - 1]));
        return largest;
    };
    for (std::size_t index = 0; index < static_cast<std::size_t>(1.5 * rate); ++index) {
        // On at the tone's crest: an empty delay line faded in would step.
        if (index == static_cast<std::size_t>(0.50125 * rate)) passBy.setEnabled(true);
        if (index == static_cast<std::size_t>(1.0 * rate)) passBy.setEnabled(false);
        const auto tone = static_cast<float>(std::sin(2.0 * std::numbers::pi * 200.0 * index / rate));
        output.push_back(passBy.process(tone, tone).leftPa);
    }
    const auto toneStep = 2.0 * std::numbers::pi * 200.0 / rate;
    const auto switching = std::max(largestStep(static_cast<std::size_t>(0.49 * rate), static_cast<std::size_t>(0.6 * rate)),
                                    largestStep(static_cast<std::size_t>(0.99 * rate), static_cast<std::size_t>(1.1 * rate)));
    std::cout << "switching on and off: largest step " << switching << " (the tone's own " << toneStep << ")\n";
    require(switching <= 1.01 * toneStep, "switching it on or off does not click");
    require(!passBy.active(), "switched off, it lets go once faded");
}

} // namespace

int main() {
    checkDoppler();
    checkDistanceLaw();
    checkGround();
    checkAir();
    checkSwitching();
    std::cout << "EngineLab.PassByMicrophone PASS\n";
    return 0;
}
