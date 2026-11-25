// The trackside microphone: a car's sound heard from the side of its lane
// while it drives past (PassByMicrophone). A pure tone checks the moving
// source's Doppler shift and the 1/r spreading against their textbook
// values; the air against ISO 9613-1's tables, from 10 m to 1 km; the ground
// against the spherical-wave reflection over a Delany-Bazley ground (the
// functions against scipy's Faddeeva function, the heard level against the
// exact sum of the direct and reflected waves, near and 500 m away); the
// exhaust's share is heard from the tailpipe, behind the engine; a run's car
// is where the run puts it, its level real or held at the lane's distance;
// switching on and off, or a run's restart, does not click, and switched off
// it is the signal it was given.

#include <enginelab/audio/PassByMicrophone.hpp>

#include <algorithm>
#include <cmath>
#include <complex>
#include <cstdlib>
#include <iostream>
#include <numbers>
#include <string>
#include <string_view>
#include <vector>

namespace {

using enginelab::PassByGround;
using enginelab::PassByMicrophone;
using enginelab::PassBySettings;

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
Run play(PassByMicrophone& passBy, double hertz, double seconds) {
    Run run;
    const auto samples = static_cast<std::size_t>(seconds * rate);
    for (std::size_t index = 0; index < samples; ++index) {
        const auto tone = static_cast<float>(std::sin(2.0 * std::numbers::pi * hertz * static_cast<double>(index) / rate));
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

PassBySettings bare() {
    PassBySettings settings;
    settings.ground = PassByGround::none;
    settings.airAbsorption = false;
    settings.halfLengthM = 1'200.0;
    settings.microphoneSpacingM = 0.0;
    return settings;
}

/** The car standing `horizontalM` from the microphone (along the lane), a
 *  tone of `hertz` heard once the delay line is full: its rms, steady. */
double standing(const PassBySettings& settings, double horizontalM, double hertz) {
    PassByMicrophone passBy;
    require(passBy.prepare(rate, settings), "the pass-by prepares");
    passBy.setEnabled(true);
    const auto along = std::sqrt(std::max(horizontalM * horizontalM - settings.laneDistanceM * settings.laneDistanceM, 0.0));
    passBy.reset(-along);
    const auto travel = std::hypot(horizontalM, settings.microphoneHeightM + settings.sourceHeightM)
        / settings.soundSpeedMps;
    const auto settled = travel + 0.3;
    const auto run = play(passBy, hertz, settled + 0.3);
    return rms(run.left, static_cast<std::size_t>(settled * rate), static_cast<std::size_t>((settled + 0.3) * rate));
}

double db(double ratio) { return 20.0 * std::log10(ratio); }

void checkDoppler() {
    // A 500 Hz tone at 30 m/s (108 km/h), far up the lane and far down it.
    // The source's range rate at emission, v x_e / r_e, shifts it to
    // f0 / (1 + rate / c): about 548 Hz coming, 460 Hz going.
    auto settings = bare();
    PassByMicrophone passBy;
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

void checkTwoSources() {
    // The tailpipe 3 m behind the engine at 30 m/s passes the microphone
    // 0.1 s after it: a tone given as the exhaust's share comes back to its
    // own pitch (no Doppler shift, the emission broadside) 0.1 s after the
    // same tone given as the engine's. (The level's peak is too flat to time.)
    auto settings = bare();
    settings.exhaustBehindM = 3.0;
    constexpr double speed = 30.0;
    const auto peakAt = [&](bool exhaust) {
        PassByMicrophone passBy;
        require(passBy.prepare(rate, settings), "the pass-by prepares");
        passBy.setVehicleSpeed(speed);
        passBy.setEnabled(true);
        passBy.reset(-60.0);
        std::vector<float> heard;
        for (std::size_t index = 0; index < static_cast<std::size_t>(4.0 * rate); ++index) {
            const auto tone = static_cast<float>(std::sin(2.0 * std::numbers::pi * 500.0 * static_cast<double>(index) / rate));
            const auto share = exhaust ? tone : 0.0F;
            heard.push_back(passBy.process(tone, tone, share, share).leftPa);
        }
        // Upward zero crossings; the period that first exceeds the tone's
        // own, interpolated between the two crossings' mid-points.
        std::vector<double> crossings;
        for (std::size_t index = 1; index < heard.size(); ++index)
            if (heard[index - 1] < 0.0F && heard[index] >= 0.0F)
                crossings.push_back(static_cast<double>(index - 1)
                    + heard[index - 1] / (heard[index - 1] - heard[index]));
        const auto own = rate / 500.0;
        for (std::size_t index = 2; index < crossings.size(); ++index) {
            const auto before = crossings[index - 1] - crossings[index - 2];
            const auto after = crossings[index] - crossings[index - 1];
            if (crossings[index] > 0.5 * rate && before <= own && after > own) {
                const auto midBefore = 0.5 * (crossings[index - 1] + crossings[index - 2]);
                const auto midAfter = 0.5 * (crossings[index] + crossings[index - 1]);
                return (midBefore + (own - before) / (after - before) * (midAfter - midBefore)) / rate;
            }
        }
        return 0.0;
    };
    const auto engine = peakAt(false);
    const auto exhaust = peakAt(true);
    std::cout << "closest approach: engine " << engine << " s, exhaust " << exhaust << " s (3 m behind at "
              << speed << " m/s: 0.1 s later)\n";
    const auto broadside = std::hypot(settings.laneDistanceM, settings.microphoneHeightM - settings.sourceHeightM);
    require(std::abs(engine - (2.0 + broadside / settings.soundSpeedMps)) < 0.002,
            "the engine passes the microphone when the car's point does");
    require(std::abs(exhaust - engine - 3.0 / speed) < 0.002, "the exhaust passes 3 m later");
}

void checkDistanceLaw() {
    // Standing still, the tone 10 m, 20 m and 1 km from the microphone: 1/r
    // is 6.02 dB per doubling, and the level the engine's observer heard is
    // scaled by its distance over the microphone's.
    auto settings = bare();
    const auto slant = [&](double horizontal) { return std::hypot(horizontal, settings.microphoneHeightM - settings.sourceHeightM); };
    const auto ten = standing(settings, 10.0, 200.0);
    const auto twenty = standing(settings, 20.0, 200.0);
    const auto kilometre = standing(settings, 1'000.0, 200.0);
    const auto drop = db(ten / twenty);
    std::cout << "10 m to 20 m: " << drop << " dB; 1 km " << db(kilometre * std::sqrt(2.0)) << " dB of a 1 Pa tone heard at "
              << settings.referenceDistanceM << " m (expected " << db(settings.referenceDistanceM / slant(1'000.0)) << ")\n";
    require(std::abs(drop - db(slant(20.0) / slant(10.0))) < 0.02, "6 dB per doubling of the distance");
    require(std::abs(ten * std::sqrt(2.0) - settings.referenceDistanceM / slant(10.0)) < 0.002,
            "the level the engine's observer heard, scaled by its distance over 10 m");
    require(std::abs(db(kilometre * std::sqrt(2.0)) - db(settings.referenceDistanceM / slant(1'000.0))) < 0.05,
            "and 1 km away: the delay line holds the car's sound that long");
}

void checkAirFormula() {
    // ISO 9613-2 table 2 (from ISO 9613-1), 20 C and 70 %, dB/km at the
    // exact third-octave mid-band frequencies of the nominal octaves.
    struct Row { double hertz; double dbPerKm; };
    const Row rows[] = { { 125.89, 0.3 }, { 251.19, 1.1 }, { 501.19, 2.8 }, { 1'000.0, 5.0 },
                         { 1'995.3, 9.0 }, { 3'981.1, 22.9 }, { 7'943.3, 76.6 } };
    for (const auto& row : rows) {
        const auto model = PassByMicrophone::airAbsorptionDbPerM(row.hertz) * 1'000.0;
        std::cout << "ISO 9613-1 at " << row.hertz << " Hz: " << model << " dB/km (table " << row.dbPerKm << ")\n";
        require(std::abs(model - row.dbPerKm) <= std::max(0.06, 0.015 * row.dbPerKm), "the air's absorption is ISO 9613-1's");
    }
}

void checkAirHeard() {
    // The air between 10 m and 500 m and 1 km, heard: the level lost beyond
    // the spreading is ISO 9613-1's over the extra distance, while it is under
    // 30 dB (past that the car is gone).
    auto settings = bare();
    settings.airAbsorption = true;
    const auto slant = [&](double horizontal) { return std::hypot(horizontal, settings.microphoneHeightM - settings.sourceHeightM); };
    double worst = 0.0;
    for (const auto far : { 500.0, 1'000.0 })
        for (const auto hertz : { 125.0, 500.0, 1'000.0, 2'000.0, 4'000.0, 8'000.0 }) {
            const auto expected = PassByMicrophone::airAbsorptionDbPerM(hertz) * (slant(far) - slant(10.0));
            if (expected > 30.0) continue;
            const auto heard = db(standing(settings, 10.0, hertz) / standing(settings, far, hertz)) - db(slant(far) / slant(10.0));
            std::cout << "air from 10 m to " << far << " m at " << hertz << " Hz: " << heard << " dB (ISO " << expected << ")\n";
            worst = std::max(worst, std::abs(heard - expected));
        }
    require(worst < 1.5, "the air takes ISO 9613-1's share within 1.5 dB from 10 m to 1 km");
}

void checkFaddeeva() {
    // scipy.special.wofz.
    struct Row { std::complex<double> z; std::complex<double> w; };
    const Row rows[] = {
        { { 1.0, 1.0 }, { 0.304744205257, 0.208218938203 } },
        { { 0.5, 2.0 }, { 0.245275990226, 0.0515214783436 } },
        { { 3.0, 0.1 }, { 0.00794268099877, 0.200742343099 } },
        { { 5.0, 5.0 }, { 0.0569654398882, 0.0558387427754 } },
        { { 20.0, 20.0 }, { 0.0141135384705, 0.0140959076493 } },
        { { 0.01, 0.01 }, { 0.988717692955, 0.0110852960575 } },
        { { -1.0, -0.5 }, { 0.155541142454, -1.13783721578 } },
        { { 2.0, -0.3 }, { -0.061872289749, 0.347188167724 } },
    };
    for (const auto& row : rows)
        require(std::abs(PassByMicrophone::faddeeva(row.z) - row.w) < 1.0e-9, "the Faddeeva function is scipy's");
    // The same formulas in Python (Delany-Bazley, Q = Rp + (1 - Rp) F(w)),
    // with scipy's w(z): 0.5 m source, 1.2 m microphone, c = 343 m/s.
    struct Case { double hertz; double horizontal; double sigma; std::complex<double> q; };
    const Case cases[] = {
        { 500.0, 500.0, 200.0, { -0.9914426308, 0.03896803833 } },
        { 4'000.0, 7.5, 30'000.0, { 0.9163476525, 0.09928908706 } },
        { 100.0, 500.0, 200.0, { -0.7391300052, 1.534421363 } },
        { 1'000.0, 50.0, 3'000.0, { 0.2583779513, 0.7583755294 } },
        { 250.0, 2'000.0, 20.0, { -0.9979178391, 0.003645270842 } },
    };
    for (const auto& item : cases) {
        const auto q = PassByMicrophone::sphericalReflection(item.hertz, item.horizontal, 0.5, 1.2, item.sigma, 343.0);
        require(std::abs(q - item.q) < 1.0e-7, "the spherical-wave reflection is the formula's");
    }
    // A rigid ground reflects all of it.
    require(std::abs(PassByMicrophone::sphericalReflection(1'000.0, 100.0, 0.5, 1.2, 1.0e12, 343.0) - 1.0) < 1.0e-3,
            "an infinitely resistive ground reflects as a rigid one");
}

void checkGround() {
    // The heard level against the exact sum of the direct and reflected
    // waves, |1 + Q (r1/r2) exp(ik(r2 - r1))| over the free field, the air
    // off. 500 m over grass and asphalt (no comb: the two paths are 2.4 mm
    // apart) at tones where the sum stays within 25 dB of the free field;
    // 7.5 m over asphalt away from the comb's notches.
    struct Case { PassByGround ground; double horizontal; std::vector<double> tones; };
    const Case cases[] = {
        { PassByGround::grass, 500.0, { 63.0, 125.0, 4'000.0, 8'000.0 } },
        { PassByGround::asphalt, 500.0, { 63.0, 250.0, 1'000.0, 2'000.0, 10'079.0, 12'699.0 } },
        { PassByGround::asphalt, 7.5, { 125.0, 250.0, 2'140.0 } },
        { PassByGround::gravel, 50.0, { 125.0, 500.0, 1'000.0 } },
    };
    auto free = bare();
    double worst = 0.0;
    for (const auto& item : cases) {
        auto settings = free;
        settings.ground = item.ground;
        const auto sigma = PassByMicrophone::flowResistivityKPa(item.ground);
        const auto r1 = std::hypot(item.horizontal, settings.microphoneHeightM - settings.sourceHeightM);
        const auto r2 = std::hypot(item.horizontal, settings.microphoneHeightM + settings.sourceHeightM);
        for (const auto hertz : item.tones) {
            const auto q = PassByMicrophone::sphericalReflection(hertz, item.horizontal, settings.sourceHeightM,
                settings.microphoneHeightM, sigma, settings.soundSpeedMps);
            const auto k = 2.0 * std::numbers::pi * hertz / settings.soundSpeedMps;
            const auto expected = db(std::abs(1.0 + q * std::polar(r1 / r2, k * (r2 - r1))));
            const auto heard = db(standing(settings, item.horizontal, hertz) / standing(free, item.horizontal, hertz));
            std::cout << "ground " << static_cast<int>(item.ground) << " at " << item.horizontal << " m, " << hertz
                      << " Hz: " << heard << " dB over the free field (exact " << expected << ")\n";
            require(expected > -25.0, "a tone the check can hear");
            worst = std::max(worst, std::abs(heard - expected));
        }
    }
    require(worst < 2.0, "the ground's reflection is the spherical wave's within 2 dB");
    // The ground dip: 500 m over grass, the mid band falls deep under the
    // free field while the lowest tones keep the image's doubling.
    auto grass = free;
    grass.ground = PassByGround::grass;
    const auto dip = db(standing(grass, 500.0, 500.0) / standing(free, 500.0, 500.0));
    const auto bass = db(standing(grass, 500.0, 40.0) / standing(free, 500.0, 40.0));
    std::cout << "grass at 500 m: 500 Hz " << dip << " dB, 40 Hz " << bass << " dB over the free field\n";
    require(dip < -20.0, "over grass far away, the mid band falls in the ground's dip");
    require(std::abs(bass - 6.0) < 1.0, "while the bass keeps the reflection's doubling");
}

void checkRunAndLevel() {
    // A run puts the car where it says: 500 m before the microphone, standing,
    // it is heard 500 m away (the free field's 1/r, no loop); normalised,
    // as loud as at the lane's distance. Its gain fades it, and a restart
    // 800 m away does not click.
    auto settings = bare();
    const auto slant = [&](double horizontal) { return std::hypot(horizontal, settings.microphoneHeightM - settings.sourceHeightM); };
    const auto level = [&](bool normalised) {
        auto local = settings;
        local.normalisedLevel = normalised;
        PassByMicrophone passBy;
        require(passBy.prepare(rate, local), "the pass-by prepares");
        passBy.setEnabled(true);
        passBy.setRun(true, -500.0, 1.0);
        require(passBy.running() && passBy.positionM() == -500.0, "the run's car is where the run puts it");
        const auto run = play(passBy, 200.0, 2.2);
        require(passBy.positionM() == -500.0, "standing, it stays there: no loop");
        return rms(run.left, static_cast<std::size_t>(1.8 * rate), static_cast<std::size_t>(2.2 * rate)) * std::sqrt(2.0);
    };
    const auto real = level(false);
    const auto normalised = level(true);
    const auto expected = slant(7.5) / slant(std::hypot(500.0, 7.5));
    std::cout << "a run's car 500 m away: " << db(real) << " dB real, " << db(normalised) << " dB normalised (expected "
              << db(settings.referenceDistanceM / slant(std::hypot(500.0, 7.5))) << ", "
              << db(settings.referenceDistanceM / slant(7.5)) << ")\n";
    require(std::abs(real / normalised - expected) < 0.01 * expected, "real is 1/r; normalised holds the lane's distance");
    require(std::abs(normalised - settings.referenceDistanceM / slant(7.5)) < 0.01, "normalised, as loud as passing by");

    PassByMicrophone passBy;
    require(passBy.prepare(rate, settings), "the pass-by prepares");
    passBy.setEnabled(true);
    passBy.setRun(true, -20.0, 1.0);
    std::vector<float> output;
    double largest = 0.0;
    const auto block = 256;
    for (int index = 0; index < static_cast<int>(3.0 * rate); ++index) {
        if (index % block == 0) {
            const auto t = index / rate;
            // At 1.5 s the run restarts 800 m away; at 2.5 s its gain falls.
            passBy.setRun(true, t < 1.5 ? -20.0 : 780.0, t < 2.5 ? 1.0 : 0.0);
        }
        const auto tone = static_cast<float>(std::sin(2.0 * std::numbers::pi * 200.0 * index / rate));
        output.push_back(passBy.process(tone, tone).leftPa);
        if (index > static_cast<int>(0.5 * rate))
            largest = std::max(largest, std::abs(static_cast<double>(output.back()) - output[output.size() - 2]));
    }
    const auto peak = settings.referenceDistanceM / slant(20.0);
    const auto toneStep = peak * 2.0 * std::numbers::pi * 200.0 / rate;
    std::cout << "a run restarted 800 m away: largest step " << largest << " (the tone's own " << toneStep << ")\n";
    require(largest <= 1.05 * toneStep, "a restart fades out and in again: no click");
    require(std::abs(passBy.positionM() - 780.0) < 1.0e-6, "the restarted car is at its new place");
    require(rms(output, static_cast<std::size_t>(2.8 * rate), output.size()) < 1.0e-4, "its gain fades it out");
    passBy.setRun(false, 0.0, 1.0);
    require(!passBy.running(), "no run: it loops again");
}

void checkSwitching() {
    // Off, the signal comes out as it went in, bit for bit. Switched on and
    // off on a 200 Hz tone, no sample steps further than the tone itself.
    PassBySettings settings;
    PassByMicrophone passBy;
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
        const auto tone = static_cast<float>(std::sin(2.0 * std::numbers::pi * 200.0 * static_cast<double>(index) / rate));
        output.push_back(passBy.process(tone, tone).leftPa);
    }
    const auto toneStep = 2.0 * std::numbers::pi * 200.0 / rate;
    const auto switching = std::max(largestStep(static_cast<std::size_t>(0.49 * rate), static_cast<std::size_t>(0.6 * rate)),
                                    largestStep(static_cast<std::size_t>(0.99 * rate), static_cast<std::size_t>(1.1 * rate)));
    // The heard tone is the input's plus the ground's reflection: its own
    // step is the bound.
    double heard = 0.0;
    for (auto index = static_cast<std::size_t>(0.7 * rate); index < static_cast<std::size_t>(0.95 * rate); ++index)
        heard = std::max(heard, static_cast<double>(std::abs(output[index])));
    const auto bound = std::max(1.0, heard) * toneStep;
    std::cout << "switching on and off: largest step " << switching << " (the tone's own " << bound << ")\n";
    require(switching <= 1.01 * bound, "switching it on or off does not click");
    require(!passBy.active(), "switched off, it lets go once faded");
}

/** The car parked `alongM` from the microphone (negative: before it), a tone
 *  of `hertz` given as the exhaust's share: its rms once heard, steady. */
double parkedExhaust(PassBySettings settings, double alongM, double hertz, double heardDirectivity = 1.0) {
    PassByMicrophone passBy;
    require(passBy.prepare(rate, settings), "the pass-by prepares");
    require(passBy.setExhaustRadiation(settings.exhaustApertureRadiusM, settings.exhaustTermination,
                                       heardDirectivity), "the tailpipe's radiation is taken");
    passBy.setEnabled(true);
    passBy.reset(alongM);
    const auto settled = std::hypot(alongM, settings.laneDistanceM) / settings.soundSpeedMps + 0.3;
    std::vector<float> heard;
    for (std::size_t index = 0; index < static_cast<std::size_t>((settled + 0.3) * rate); ++index) {
        const auto tone = static_cast<float>(std::sin(2.0 * std::numbers::pi * hertz * static_cast<double>(index) / rate));
        heard.push_back(passBy.process(tone, tone, tone, tone).leftPa);
    }
    return rms(heard, static_cast<std::size_t>(settled * rate), heard.size());
}

void checkTailpipeDirectivity() {
    // An 82 mm tailpipe (ka = 1 at 1.33 kHz) pointing back along the lane,
    // the car parked 20 m before the microphone and 20 m past it: the same
    // distance, so only the pipe's directivity tells them apart. Above ka = 1
    // the pipe beams backward: duller before the car has passed. The
    // expected level is the engine's observer's own model (a one-pole split
    // at ka = 1, the high band scaled by sqrt((1 + cos) / 2)).
    auto settings = bare();
    settings.exhaustApertureRadiusM = 0.041;
    const auto expected = [&](double alongM, double hertz, double heardDirectivity) {
        const auto distance = std::hypot(alongM, std::hypot(settings.laneDistanceM,
            settings.microphoneHeightM - settings.sourceHeightM));
        const auto directivity = enginelab::FreeFieldObserver::highBandDirectivity(
            settings.exhaustTermination, alongM / distance) / heardDirectivity;
        const auto transition = settings.soundSpeedMps / (2.0 * std::numbers::pi * settings.exhaustApertureRadiusM);
        const auto a = 1.0 - std::exp(-2.0 * std::numbers::pi * transition / rate);
        const auto z = std::polar(1.0, -2.0 * std::numbers::pi * hertz / rate);
        const auto low = a / (1.0 - (1.0 - a) * z);
        return std::abs(low + directivity * (1.0 - low)) * settings.referenceDistanceM / distance / std::sqrt(2.0);
    };
    for (const auto hertz : { 100.0, 6'000.0 }) {
        const auto before = parkedExhaust(settings, -20.0, hertz);
        const auto after = parkedExhaust(settings, 20.0, hertz);
        const auto expectedDb = db(expected(-20.0, hertz, 1.0) / expected(20.0, hertz, 1.0));
        std::cout << "tailpipe 82 mm, " << hertz << " Hz, 20 m before against 20 m past: " << db(before / after)
                  << " dB (the model: " << expectedDb << " dB); levels " << db(before / expected(-20.0, hertz, 1.0))
                  << " and " << db(after / expected(20.0, hertz, 1.0)) << " dB off it\n";
        require(std::abs(db(before / expected(-20.0, hertz, 1.0))) < 0.2
                    && std::abs(db(after / expected(20.0, hertz, 1.0))) < 0.2,
                "the exhaust is heard with its pipe's directivity");
        require(hertz < 1'000.0 ? std::abs(expectedDb) < 0.1 : expectedDb < -9.0,
                "under ka = 1 it radiates evenly; well above, the pipe beams backward");
    }
    // The engine's observer heard the share at half the on-axis high band:
    // that is undone, so 20 m past is twice as bright above ka = 1.
    const auto halved = parkedExhaust(settings, 20.0, 6'000.0, 0.5);
    require(std::abs(db(halved / expected(20.0, 6'000.0, 0.5))) < 0.2, "the incoming directivity is undone");
    // No opening given: the share radiates evenly, as it did.
    auto even = bare();
    const auto evenDb = db(parkedExhaust(even, -20.0, 6'000.0) / parkedExhaust(even, 20.0, 6'000.0));
    require(std::abs(evenDb) < 0.05, "without an opening the exhaust's share radiates evenly");
}

} // namespace

int main() {
    checkFaddeeva();
    checkAirFormula();
    checkDoppler();
    checkTwoSources();
    checkTailpipeDirectivity();
    checkDistanceLaw();
    checkAirHeard();
    checkGround();
    checkRunAndLevel();
    checkSwitching();
    std::cout << "EngineLab.PassByMicrophone PASS\n";
    return 0;
}
