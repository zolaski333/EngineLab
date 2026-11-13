#include <enginelab/audio/PassByMicrophone.hpp>

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <numbers>

namespace enginelab {
namespace {

constexpr double pi = std::numbers::pi;
// A peaking band one third of an octave wide (RBJ's Q for N = 1/3 octave).
constexpr double bandQuality = 4.318;
// The interaction matrix's prototype level, dB (Valimaki & Liski 2017 use 17).
constexpr double prototypeDb = 12.0;
// The air's low-passes take this much where they start to matter.
constexpr double airStageDb = 20.0;
// A run's place pulls the heard one in with this time constant.
constexpr double runCorrectionSeconds = 0.5;
// A run's gain and a restart's fade.
constexpr double runGainSeconds = 0.020;
// A run's place that jumps further than this restarts the delay line.
constexpr double runJumpM = 50.0;

struct WeidemanTable final {
    static constexpr int terms = 32;
    std::array<double, terms> a {};
    double l { 0.0 };
};

const WeidemanTable& weideman() {
    // Weideman 1994, eq. (38)-(40): w(z) = 2 p(Z) / (L - iz)^2 + 1 / (sqrt(pi) (L - iz)),
    // Z = (L + iz) / (L - iz), the a_n from a 2M-point transform.
    static const WeidemanTable table = [] {
        WeidemanTable out;
        constexpr int n = WeidemanTable::terms;
        constexpr int m = 2 * n;
        constexpr int m2 = 2 * m;
        out.l = std::sqrt(n / std::sqrt(2.0));
        std::array<double, m2> samples {};
        for (int index = 0; index < m2; ++index) {
            const auto k = index < m ? index : index - m2;
            if (k == -m) continue;
            const auto t = out.l * std::tan(0.5 * k * pi / m);
            samples[static_cast<std::size_t>(index)] = std::exp(-t * t) * (out.l * out.l + t * t);
        }
        for (int j = 1; j <= n; ++j) {
            double sum = 0.0;
            for (int index = 0; index < m2; ++index)
                sum += samples[static_cast<std::size_t>(index)] * std::cos(2.0 * pi * j * index / m2);
            out.a[static_cast<std::size_t>(j - 1)] = sum / m2;
        }
        return out;
    }();
    return table;
}

std::complex<double> faddeevaUpper(std::complex<double> z) noexcept {
    const auto& table = weideman();
    const std::complex<double> i { 0.0, 1.0 };
    const auto denominator = table.l - i * z;
    const auto big = (table.l + i * z) / denominator;
    std::complex<double> p { table.a.back(), 0.0 };
    for (auto index = table.a.size() - 1; index-- > 0;) p = p * big + table.a[index];
    return 2.0 * p / (denominator * denominator) + 1.0 / (std::sqrt(pi) * denominator);
}

} // namespace

double PassByMicrophone::airAbsorptionDbPerM(double hertz) noexcept {
    // ISO 9613-1:1993, eq. (3)-(5) and annex B for the saturation pressure.
    constexpr double temperatureK = 293.15;
    constexpr double referenceK = 293.15;
    constexpr double tripleK = 273.16;
    constexpr double relativeHumidity = 70.0;
    constexpr double pressureRatio = 1.0; // pa / pr
    const auto saturation = std::pow(10.0, -6.8346 * std::pow(tripleK / temperatureK, 1.261) + 4.6151);
    const auto h = relativeHumidity * saturation / pressureRatio;
    const auto oxygen = pressureRatio * (24.0 + 4.04e4 * h * (0.02 + h) / (0.391 + h));
    const auto nitrogen = pressureRatio * std::pow(temperatureK / referenceK, -0.5)
        * (9.0 + 280.0 * h * std::exp(-4.170 * (std::pow(temperatureK / referenceK, -1.0 / 3.0) - 1.0)));
    const auto f2 = hertz * hertz;
    return 8.686 * f2 * (1.84e-11 / pressureRatio * std::sqrt(temperatureK / referenceK)
        + std::pow(temperatureK / referenceK, -2.5)
            * (0.01275 * std::exp(-2'239.1 / temperatureK) / (oxygen + f2 / oxygen)
               + 0.1068 * std::exp(-3'352.0 / temperatureK) / (nitrogen + f2 / nitrogen)));
}

std::complex<double> PassByMicrophone::faddeeva(std::complex<double> z) noexcept {
    if (z.imag() >= 0.0) return faddeevaUpper(z);
    // w(-z) = 2 exp(-z^2) - w(z).
    return 2.0 * std::exp(-z * z) - faddeevaUpper(-z);
}

std::complex<double> PassByMicrophone::groundImpedance(double hertz, double flowResistivityKPa) noexcept {
    // Delany & Bazley 1970 in the outdoor form (f / sigma, sigma in kPa s/m2).
    const auto x = std::max(hertz, 1.0) / std::max(flowResistivityKPa, 1.0e-3);
    return { 1.0 + 9.08 * std::pow(x, -0.75), 11.9 * std::pow(x, -0.73) };
}

std::complex<double> PassByMicrophone::sphericalReflection(double hertz, double horizontalM,
    double sourceHeightM, double microphoneHeightM, double flowResistivityKPa,
    double soundSpeedMps) noexcept {
    // Attenborough, Li & Horoshenkov, "Predicting outdoor sound" (2007), ch. 2:
    // Q = Rp + (1 - Rp) F(w), w the numerical distance, F = 1 + i sqrt(pi) w w(w).
    const std::complex<double> i { 0.0, 1.0 };
    const auto k = 2.0 * pi * hertz / soundSpeedMps;
    const auto heights = sourceHeightM + microphoneHeightM;
    const auto imageM = std::max(std::hypot(horizontalM, heights), 1.0e-6);
    const auto grazing = heights / imageM;
    const auto impedance = groundImpedance(hertz, flowResistivityKPa);
    const auto plane = (impedance * grazing - 1.0) / (impedance * grazing + 1.0);
    const auto numerical = std::sqrt(0.5 * k * imageM) * std::complex<double> { std::numbers::sqrt2 / 2.0,
                                                                                 std::numbers::sqrt2 / 2.0 }
        * (grazing + 1.0 / impedance);
    const auto boundaryLoss = 1.0 + i * std::sqrt(pi) * numerical * faddeeva(numerical);
    return plane + (1.0 - plane) * boundaryLoss;
}

double PassByMicrophone::flowResistivityKPa(PassByGround ground) noexcept {
    switch (ground) {
    case PassByGround::asphalt: return 30'000.0;
    case PassByGround::gravel: return 3'000.0;
    case PassByGround::grass: return 200.0;
    case PassByGround::snow: return 20.0;
    case PassByGround::none: break;
    }
    return 0.0;
}

double PassByMicrophone::bandCentreHz(std::size_t band) noexcept {
    return 1'000.0 * std::pow(2.0, (static_cast<double>(band) - 16.0) / 3.0);
}

PassByMicrophone::Biquad PassByMicrophone::peaking(double centreHz, double gainDb, double sampleRateHz) noexcept {
    // RBJ's peaking equaliser.
    const auto amplitude = std::pow(10.0, gainDb / 40.0);
    const auto omega = 2.0 * pi * centreHz / sampleRateHz;
    const auto alpha = std::sin(omega) / (2.0 * bandQuality);
    const auto a0 = 1.0 + alpha / amplitude;
    Biquad out;
    out.b0 = (1.0 + alpha * amplitude) / a0;
    out.b1 = -2.0 * std::cos(omega) / a0;
    out.b2 = (1.0 - alpha * amplitude) / a0;
    out.a1 = out.b1;
    out.a2 = (1.0 - alpha / amplitude) / a0;
    return out;
}

double PassByMicrophone::responseDb(const Biquad& biquad, double hertz, double sampleRateHz) noexcept {
    const auto z = std::polar(1.0, -2.0 * pi * hertz / sampleRateHz);
    const auto numerator = biquad.b0 + biquad.b1 * z + biquad.b2 * z * z;
    const auto denominator = 1.0 + biquad.a1 * z + biquad.a2 * z * z;
    return 20.0 * std::log10(std::max(std::abs(numerator / denominator), 1.0e-12));
}

double PassByMicrophone::airStageLossDb(double hertz) const noexcept {
    if (airCoefficient_ <= 0.0) return 0.0;
    const auto z = std::polar(1.0, -2.0 * pi * hertz / sampleRateHz_);
    const auto response = airCoefficient_ / (1.0 - (1.0 - airCoefficient_) * z);
    return -20.0 * std::log10(std::max(std::abs(response), 1.0e-12)) * static_cast<double>(airStages);
}

bool PassByMicrophone::prepare(double sampleRateHz, const PassBySettings& settings) {
    const auto finitePositive = [](double value) { return std::isfinite(value) && value > 0.0; };
    if (!finitePositive(sampleRateHz) || !finitePositive(settings.laneDistanceM)
        || !finitePositive(settings.soundSpeedMps) || !finitePositive(settings.referenceDistanceM)
        || !finitePositive(settings.halfLengthM) || settings.halfLengthM <= 2.0 * fadeLengthM
        || !std::isfinite(settings.microphoneHeightM) || !std::isfinite(settings.sourceHeightM)
        || !std::isfinite(settings.microphoneSpacingM)
        || settings.microphoneHeightM < 0.0 || settings.sourceHeightM < 0.0
        || static_cast<std::uint8_t>(settings.ground) > static_cast<std::uint8_t>(PassByGround::snow)
        || !std::isfinite(settings.exhaustBehindM) || std::abs(settings.exhaustBehindM) > fadeLengthM)
        return false;
    settings_ = settings;
    sampleRateHz_ = sampleRateHz;
    // The farthest car, coming at up to 100 m/s: its sound left it when it was
    // further still (c tau = |x - v tau|).
    const auto seconds = maximumRangeM / std::max(settings.soundSpeedMps - 100.0, 0.25 * settings.soundSpeedMps)
        + 0.25;
    std::size_t size = 1;
    while (size < static_cast<std::size_t>(seconds * sampleRateHz) + 8) size <<= 1U;
    delay_.assign(size, 0.0F);
    exhaustDelay_.assign(size, 0.0F);
    mask_ = size - 1;
    longestDelaySamples_ = static_cast<double>(size - 8);
    // The pedal and the gearbox move the car's speed at the frame rate:
    // 20 ms smooths their steps out of the Doppler shift.
    speedCoefficient_ = 1.0 - std::exp(-1.0 / (0.020 * sampleRateHz));
    gainCoefficient_ = 1.0 - std::exp(-1.0 / (runGainSeconds * sampleRateHz));
    runCorrection_ = 1.0 / (runCorrectionSeconds * sampleRateHz);
    mixStep_ = static_cast<float>(1.0 / (crossFadeSeconds * sampleRateHz));

    // The equaliser's bands under 0.45 x the sample rate, and the inverse of
    // their interaction (each band's level, per dB of the prototype, at every
    // centre): the levels that put the wanted ones at the centres.
    bands_ = 0;
    while (bands_ < bandCount && bandCentreHz(bands_) < 0.45 * sampleRateHz) ++bands_;
    const auto n = bands_;
    std::vector<double> matrix(n * n);
    for (std::size_t column = 0; column < n; ++column) {
        const auto prototype = peaking(bandCentreHz(column), prototypeDb, sampleRateHz);
        for (std::size_t row = 0; row < n; ++row)
            matrix[row * n + column] = responseDb(prototype, bandCentreHz(row), sampleRateHz) / prototypeDb;
    }
    interactionInverse_.assign(n * n, 0.0);
    for (std::size_t index = 0; index < n; ++index) interactionInverse_[index * n + index] = 1.0;
    for (std::size_t pivot = 0; pivot < n; ++pivot) {
        auto best = pivot;
        for (auto row = pivot + 1; row < n; ++row)
            if (std::abs(matrix[row * n + pivot]) > std::abs(matrix[best * n + pivot])) best = row;
        if (std::abs(matrix[best * n + pivot]) < 1.0e-12) return false;
        if (best != pivot)
            for (std::size_t column = 0; column < n; ++column) {
                std::swap(matrix[best * n + column], matrix[pivot * n + column]);
                std::swap(interactionInverse_[best * n + column], interactionInverse_[pivot * n + column]);
            }
        const auto scale = 1.0 / matrix[pivot * n + pivot];
        for (std::size_t column = 0; column < n; ++column) {
            matrix[pivot * n + column] *= scale;
            interactionInverse_[pivot * n + column] *= scale;
        }
        for (std::size_t row = 0; row < n; ++row) {
            if (row == pivot) continue;
            const auto factor = matrix[row * n + pivot];
            if (factor == 0.0) continue;
            for (std::size_t column = 0; column < n; ++column) {
                matrix[row * n + column] -= factor * matrix[pivot * n + column];
                interactionInverse_[row * n + column] -= factor * interactionInverse_[pivot * n + column];
            }
        }
    }
    running_ = false;
    runGain_ = runGainSmoothed_ = 1.0;
    reset();
    return true;
}

void PassByMicrophone::clearLines() noexcept {
    std::fill(delay_.begin(), delay_.end(), 0.0F);
    std::fill(exhaustDelay_.begin(), exhaustDelay_.end(), 0.0F);
    for (auto& microphone : equaliserState_) microphone = {};
    for (auto& microphone : airState_) microphone = {};
    designCountdown_ = 0;
}

void PassByMicrophone::reset(double positionM) noexcept {
    clearLines();
    write_ = 0;
    speedMps_ = targetSpeedMps_;
    setPosition(positionM);
    runTargetM_ = positionM_;
    mix_ = enabled_ ? 1.0F : 0.0F;
    warmup_ = 0;
}

void PassByMicrophone::setEnabled(bool enabled) noexcept {
    if (enabled && !active()) {
        // Nothing heard since it was last on: start clean, in front.
        clearLines();
        speedMps_ = targetSpeedMps_;
        if (!running_) positionM_ = 0.0;
        warmup_ = longestStartDelay();
    }
    enabled_ = enabled && prepared();
}

void PassByMicrophone::setVehicleSpeed(double metresPerSecond) noexcept {
    if (!std::isfinite(metresPerSecond)) return;
    const auto limit = 0.8 * settings_.soundSpeedMps;
    targetSpeedMps_ = std::clamp(metresPerSecond, -limit, limit);
}

void PassByMicrophone::setReferenceDistance(double metres) noexcept {
    if (std::isfinite(metres) && metres > 0.0) settings_.referenceDistanceM = metres;
}

void PassByMicrophone::setPosition(double positionM) noexcept {
    if (!std::isfinite(positionM)) return;
    if (running_) {
        positionM_ = positionM;
        return;
    }
    const auto length = 2.0 * settings_.halfLengthM;
    positionM_ = positionM - length * std::floor((positionM + settings_.halfLengthM) / length);
}

bool PassByMicrophone::setGeometry(double laneDistanceM, double microphoneHeightM, PassByGround ground,
                                   bool normalisedLevel) noexcept {
    if (!std::isfinite(laneDistanceM) || laneDistanceM <= 0.0 || !std::isfinite(microphoneHeightM)
        || microphoneHeightM < 0.0 || static_cast<std::uint8_t>(ground) > static_cast<std::uint8_t>(PassByGround::snow))
        return false;
    if (settings_.laneDistanceM == laneDistanceM && settings_.microphoneHeightM == microphoneHeightM
        && settings_.ground == ground && settings_.normalisedLevel == normalisedLevel)
        return true;
    settings_.laneDistanceM = laneDistanceM;
    settings_.microphoneHeightM = microphoneHeightM;
    settings_.ground = ground;
    settings_.normalisedLevel = normalisedLevel;
    designCountdown_ = 0;
    return true;
}

void PassByMicrophone::setRun(bool running, double positionM, double gain) noexcept {
    if (!running) {
        if (running_) {
            running_ = false;
            setPosition(positionM_);
        }
        runGain_ = 1.0;
        return;
    }
    if (!std::isfinite(positionM)) return;
    const auto wanted = std::isfinite(gain) ? std::clamp(gain, 0.0, 1.0) : 1.0;
    if (!running_ || std::abs(positionM - runTargetM_) > runJumpM) {
        // A new place: faded out first and in again there, so the delay's
        // jump does not click. The line keeps the engine's past sound, which
        // the new place hears from as far back as its distance asks.
        if (mix_ > 0.0F && runGainSmoothed_ > 1.0e-3) {
            runGain_ = 0.0;
            return;
        }
        running_ = true;
        positionM_ = positionM;
        runGainSmoothed_ = 0.0;
        designCountdown_ = 0;
    }
    runTargetM_ = positionM;
    runGain_ = wanted;
}

void PassByMicrophone::retarded(double alongM, double heightDifferenceM, double offsetM,
                                double& delaySamples, double& distanceM) const noexcept {
    // Emitted tau ago at x - v tau, heard now: c tau = |(x - v tau, D)|,
    // (c^2 - v^2) tau^2 + 2 x v tau - (x^2 + D^2) = 0, the positive root.
    const auto c = settings_.soundSpeedMps;
    const auto v = speedMps_;
    const auto x = alongM - offsetM;
    const auto lateral2 = settings_.laneDistanceM * settings_.laneDistanceM
        + heightDifferenceM * heightDifferenceM;
    const auto a = c * c - v * v;
    const auto tau = (-x * v + std::sqrt(x * x * v * v + a * (x * x + lateral2))) / a;
    distanceM = c * tau;
    delaySamples = std::min(tau * sampleRateHz_, longestDelaySamples_);
}

std::size_t PassByMicrophone::longestStartDelay() const noexcept {
    double longest = 0.0;
    for (const auto along : { positionM_, positionM_ - settings_.exhaustBehindM })
        for (const auto offset : { -0.5 * settings_.microphoneSpacingM, 0.5 * settings_.microphoneSpacingM }) {
            double delaySamples {};
            double distanceM {};
            retarded(along, settings_.microphoneHeightM + settings_.sourceHeightM, offset, delaySamples, distanceM);
            longest = std::max(longest, delaySamples);
        }
    return static_cast<std::size_t>(std::ceil(longest)) + 4;
}

float PassByMicrophone::read(const std::vector<float>& line, double delaySamples) const noexcept {
    // Four-point Hermite between the samples around the delay.
    const auto position = static_cast<double>(write_) - delaySamples;
    const auto floored = std::floor(position);
    const auto t = static_cast<float>(position - floored);
    const auto base = static_cast<std::size_t>(static_cast<long long>(floored)) & mask_;
    const auto ym1 = line[(base - 1) & mask_];
    const auto y0 = line[base];
    const auto y1 = line[(base + 1) & mask_];
    const auto y2 = line[(base + 2) & mask_];
    const auto c1 = 0.5F * (y1 - ym1);
    const auto c2 = ym1 - 2.5F * y0 + 2.0F * y1 - 0.5F * y2;
    const auto c3 = 0.5F * (y2 - ym1) + 1.5F * (y0 - y1);
    return ((c3 * t + c2) * t + c1) * t + y0;
}

float PassByMicrophone::window(double alongM) const noexcept {
    if (running_) return 1.0F;
    const auto fromEnd = settings_.halfLengthM - std::abs(alongM);
    if (fromEnd >= fadeLengthM) return 1.0F;
    const auto s = std::sin(0.5 * pi * std::max(fromEnd, 0.0) / fadeLengthM);
    return static_cast<float>(s * s);
}

void PassByMicrophone::design() noexcept {
    // The car's middle (between the engine and the tailpipe) and the
    // microphones' middle: the two are within a few metres of each other,
    // where neither the air nor the ground changes over a band.
    const auto along = positionM_ - 0.5 * settings_.exhaustBehindM;
    const auto horizontal = std::hypot(along, settings_.laneDistanceM);
    const auto source = settings_.sourceHeightM;
    const auto microphone = settings_.microphoneHeightM;
    const auto directM = std::hypot(horizontal, microphone - source);
    const auto imageM = std::hypot(horizontal, microphone + source);
    const auto ratio = directM / imageM;
    const auto difference = imageM - directM;
    const auto c = settings_.soundSpeedMps;
    const auto nyquistShare = 0.45 * sampleRateHz_;
    std::array<double, bandCount> target {};
    // Four frequencies across each third octave (a semitone apart).
    const auto sampleOf = [&](std::size_t band, int index) {
        return bandCentreHz(band) * std::pow(2.0, (static_cast<double>(index) - 1.5) / 12.0);
    };

    const auto sigma = flowResistivityKPa(settings_.ground);
    if (settings_.ground == PassByGround::none) {
        imageReflection_ = 0.0;
    } else {
        // The comb the delays draw: a real reflection, the spherical wave's
        // magnitude at the first notch a hard ground would dig.
        const auto firstNotch = std::clamp(c / (2.0 * std::max(difference, 1.0e-9)), 100.0, nyquistShare);
        imageReflection_ = std::clamp(
            std::abs(sphericalReflection(firstNotch, horizontal, source, microphone, sigma, c)), 0.0, 1.0);
        for (std::size_t band = 0; band < bands_; ++band) {
            double exact = 0.0;
            double drawn = 0.0;
            for (int index = 0; index < 4; ++index) {
                const auto hertz = sampleOf(band, index);
                const auto phase = std::polar(ratio, 2.0 * pi * hertz / c * difference);
                exact += std::norm(1.0 + sphericalReflection(hertz, horizontal, source, microphone, sigma, c) * phase);
                drawn += std::norm(1.0 + imageReflection_ * phase);
            }
            target[band] += 10.0 * std::log10(std::max(exact, 1.0e-12) / std::max(drawn, 1.0e-12));
        }
    }

    airCoefficient_ = 0.0;
    if (settings_.airAbsorption) {
        const auto rangeM = 0.5 * (directM + imageM);
        if (airAbsorptionDbPerM(nyquistShare) * rangeM > airStageDb) {
            // Where the air takes 20 dB (it rises with frequency).
            auto low = std::log(20.0);
            auto high = std::log(nyquistShare);
            for (int step = 0; step < 40; ++step) {
                const auto middle = 0.5 * (low + high);
                if (airAbsorptionDbPerM(std::exp(middle)) * rangeM > airStageDb) high = middle;
                else low = middle;
            }
            const auto twentyDb = std::exp(0.5 * (low + high));
            const auto cutoff = std::min(
                twentyDb / std::sqrt(std::pow(10.0, airStageDb / (10.0 * static_cast<double>(airStages))) - 1.0),
                nyquistShare);
            airCoefficient_ = 1.0 - std::exp(-2.0 * pi * cutoff / sampleRateHz_);
        }
        for (std::size_t band = 0; band < bands_; ++band) {
            double power = 0.0;
            for (int index = 0; index < 4; ++index) {
                const auto hertz = sampleOf(band, index);
                power += std::pow(10.0, (airStageLossDb(hertz) - airAbsorptionDbPerM(hertz) * rangeM) / 10.0);
            }
            target[band] += 10.0 * std::log10(std::max(0.25 * power, 1.0e-12));
        }
    }

    // The levels: the interaction's inverse, then two corrections of what
    // the cascade really gives at the centres.
    const auto n = bands_;
    std::array<double, bandCount> gains {};
    for (std::size_t row = 0; row < n; ++row) target[row] = std::clamp(target[row], lowestBandDb, highestBandDb);
    const auto solve = [&](const std::array<double, bandCount>& error) {
        for (std::size_t row = 0; row < n; ++row) {
            double sum = 0.0;
            for (std::size_t column = 0; column < n; ++column) sum += interactionInverse_[row * n + column] * error[column];
            gains[row] = std::clamp(gains[row] + sum, lowestBandDb, highestBandDb);
        }
    };
    solve(target);
    for (int iteration = 0; iteration < 2; ++iteration) {
        for (std::size_t band = 0; band < n; ++band) equaliser_[band] = peaking(bandCentreHz(band), gains[band], sampleRateHz_);
        std::array<double, bandCount> error {};
        for (std::size_t row = 0; row < n; ++row) {
            double cascade = 0.0;
            for (std::size_t band = 0; band < n; ++band) cascade += responseDb(equaliser_[band], bandCentreHz(row), sampleRateHz_);
            error[row] = target[row] - cascade;
        }
        solve(error);
    }
    for (std::size_t band = 0; band < n; ++band) equaliser_[band] = peaking(bandCentreHz(band), gains[band], sampleRateHz_);
}

StereoPressure PassByMicrophone::process(float left, float right, float exhaustLeft,
                                         float exhaustRight) noexcept {
    if (!active()) return { left, right };
    delay_[write_] = 0.5F * (left - exhaustLeft + right - exhaustRight);
    exhaustDelay_[write_] = 0.5F * (exhaustLeft + exhaustRight);

    speedMps_ += speedCoefficient_ * (targetSpeedMps_ - speedMps_);
    if (designCountdown_ == 0) {
        design();
        designCountdown_ = designSamples;
    }
    --designCountdown_;
    const auto fade = window(positionM_);
    std::array<float, 2> heard {};
    const auto direct = settings_.microphoneHeightM - settings_.sourceHeightM;
    const auto image = settings_.microphoneHeightM + settings_.sourceHeightM;
    const auto halfSpacing = 0.5 * settings_.microphoneSpacingM;
    const auto closestM = std::hypot(settings_.laneDistanceM, direct);
    for (std::size_t microphone = 0; microphone < 2; ++microphone) {
        const auto offset = microphone == 0 ? -halfSpacing : halfSpacing;
        double sum = 0.0;
        double engineDirectM = closestM;
        for (const auto exhaust : { false, true }) {
            const auto along = exhaust ? positionM_ - settings_.exhaustBehindM : positionM_;
            const auto& line = exhaust ? exhaustDelay_ : delay_;
            double delaySamples {};
            double distanceM {};
            retarded(along, direct, offset, delaySamples, distanceM);
            if (!exhaust) engineDirectM = distanceM;
            sum += static_cast<double>(read(line, delaySamples)) * settings_.referenceDistanceM / distanceM;
            if (imageReflection_ > 0.0) {
                retarded(along, image, offset, delaySamples, distanceM);
                sum += static_cast<double>(read(line, delaySamples)) * imageReflection_
                    * settings_.referenceDistanceM / distanceM;
            }
        }
        if (settings_.normalisedLevel) sum *= engineDirectM / closestM;
        if (airCoefficient_ > 0.0)
            for (auto& stage : airState_[microphone]) {
                stage += airCoefficient_ * (sum - stage);
                sum = stage;
            }
        auto& states = equaliserState_[microphone];
        for (std::size_t band = 0; band < bands_; ++band) {
            const auto& biquad = equaliser_[band];
            auto& state = states[band];
            const auto out = biquad.b0 * sum + state.z1;
            state.z1 = biquad.b1 * sum - biquad.a1 * out + state.z2;
            state.z2 = biquad.b2 * sum - biquad.a2 * out;
            sum = out;
        }
        heard[microphone] = static_cast<float>(sum);
    }
    write_ = (write_ + 1) & mask_;
    const auto travel = speedMps_ / sampleRateHz_;
    if (running_) {
        runTargetM_ += travel;
        positionM_ += travel + runCorrection_ * (runTargetM_ - positionM_);
        runGainSmoothed_ += gainCoefficient_ * (runGain_ - runGainSmoothed_);
    } else {
        setPosition(positionM_ + travel);
        runGainSmoothed_ += gainCoefficient_ * (runGain_ - runGainSmoothed_);
    }
    const auto gain = fade * static_cast<float>(runGainSmoothed_);

    const auto target = enabled_ ? 1.0F : 0.0F;
    if (warmup_ != 0 && enabled_)
        --warmup_;
    else
        mix_ = mix_ < target ? std::min(target, mix_ + mixStep_) : std::max(target, mix_ - mixStep_);
    return { left + mix_ * (gain * heard[0] - left), right + mix_ * (gain * heard[1] - right) };
}

} // namespace enginelab
