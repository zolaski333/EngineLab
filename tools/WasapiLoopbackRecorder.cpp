// Standalone WASAPI loopback recorder.
//
// Captures whatever the default render endpoint is playing and writes it to a
// 32-bit float WAV. Needed because the ES3D prototype is a closed binary with
// no offline render and no WAV export: its audio only exists as sound card
// output, so the only way to measure it with the same third-octave analysis
// EngineLabGeometrySensitivityHarness uses is to capture the endpoint mix.
//
// Loopback capture reads the render mix directly, so it needs neither a
// "Stereo Mix" input nor a virtual audio cable.
//
//   cl /std:c++20 /EHsc /O2 /W4 /WX WasapiLoopbackRecorder.cpp /link ole32.lib
//   WasapiLoopbackRecorder.exe --seconds 12 --out capture.wav

#define WIN32_LEAN_AND_MEAN
#define NOMINMAX

#include <windows.h>

#include <audioclient.h>
#include <mmdeviceapi.h>

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace
{

constexpr REFERENCE_TIME hundredNanosecondsPerSecond = 10'000'000LL;

// Defined locally so the translation unit does not need <ksmedia.h>.
const GUID subtypeIeeeFloat {
    0x00000003, 0x0000, 0x0010, { 0x80, 0x00, 0x00, 0xAA, 0x00, 0x38, 0x9B, 0x71 } };
const GUID subtypePcm {
    0x00000001, 0x0000, 0x0010, { 0x80, 0x00, 0x00, 0xAA, 0x00, 0x38, 0x9B, 0x71 } };

struct ComScope final
{
    HRESULT hr { E_FAIL };

    ComScope() noexcept { hr = CoInitializeEx(nullptr, COINIT_MULTITHREADED); }
    ~ComScope() { if (SUCCEEDED(hr)) { CoUninitialize(); } }

    ComScope(const ComScope&) = delete;
    ComScope& operator=(const ComScope&) = delete;
};

template <typename T>
struct ComPtr final
{
    T* raw { nullptr };

    ComPtr() = default;
    ~ComPtr() { if (raw != nullptr) { raw->Release(); } }

    ComPtr(const ComPtr&) = delete;
    ComPtr& operator=(const ComPtr&) = delete;

    [[nodiscard]] T** put() noexcept { return &raw; }
    [[nodiscard]] void** putVoid() noexcept { return reinterpret_cast<void**>(&raw); }
    T* operator->() const noexcept { return raw; }
    [[nodiscard]] explicit operator bool() const noexcept { return raw != nullptr; }
};

// The format the endpoint actually runs at. Loopback always hands back the
// device mix format, so the recorder adapts rather than requesting one.
enum class SampleEncoding
{
    unsupported,
    float32,
    int16,
    int32
};

[[nodiscard]] SampleEncoding classify(const WAVEFORMATEX& format) noexcept
{
    const GUID* subFormat { nullptr };
    if (format.wFormatTag == WAVE_FORMAT_EXTENSIBLE)
    {
        if (format.cbSize < sizeof(WAVEFORMATEXTENSIBLE) - sizeof(WAVEFORMATEX))
        {
            return SampleEncoding::unsupported;
        }
        subFormat = &reinterpret_cast<const WAVEFORMATEXTENSIBLE&>(format).SubFormat;
    }

    const bool isFloat = (format.wFormatTag == WAVE_FORMAT_IEEE_FLOAT)
        || ((subFormat != nullptr) && IsEqualGUID(*subFormat, subtypeIeeeFloat));
    const bool isPcm = (format.wFormatTag == WAVE_FORMAT_PCM)
        || ((subFormat != nullptr) && IsEqualGUID(*subFormat, subtypePcm));

    if (isFloat && format.wBitsPerSample == 32) { return SampleEncoding::float32; }
    if (isPcm && format.wBitsPerSample == 16) { return SampleEncoding::int16; }
    if (isPcm && format.wBitsPerSample == 32) { return SampleEncoding::int32; }
    return SampleEncoding::unsupported;
}

void appendFrames(std::vector<float>& destination,
                  const BYTE* source,
                  std::uint32_t frameCount,
                  std::uint16_t channels,
                  SampleEncoding encoding)
{
    const std::size_t total = static_cast<std::size_t>(frameCount) * channels;
    const std::size_t base = destination.size();
    destination.resize(base + total);

    switch (encoding)
    {
    case SampleEncoding::float32:
        std::memcpy(destination.data() + base, source, total * sizeof(float));
        break;
    case SampleEncoding::int16:
    {
        const auto* samples = reinterpret_cast<const std::int16_t*>(source);
        for (std::size_t i = 0; i < total; ++i)
        {
            destination[base + i] = static_cast<float>(samples[i]) / 32'768.0F;
        }
        break;
    }
    case SampleEncoding::int32:
    {
        const auto* samples = reinterpret_cast<const std::int32_t*>(source);
        for (std::size_t i = 0; i < total; ++i)
        {
            destination[base + i] = static_cast<float>(
                static_cast<double>(samples[i]) / 2'147'483'648.0);
        }
        break;
    }
    case SampleEncoding::unsupported:
    default:
        break;
    }
}

void appendSilence(std::vector<float>& destination,
                   std::uint32_t frameCount,
                   std::uint16_t channels)
{
    destination.resize(destination.size()
                       + static_cast<std::size_t>(frameCount) * channels,
                       0.0F);
}

template <typename T>
void put(std::vector<std::uint8_t>& header, T value)
{
    const auto* bytes = reinterpret_cast<const std::uint8_t*>(&value);
    header.insert(header.end(), bytes, bytes + sizeof(T));
}

void putTag(std::vector<std::uint8_t>& header, const char (&tag)[5])
{
    header.insert(header.end(), tag, tag + 4);
}

[[nodiscard]] bool writeWav(const std::string& path,
                            const std::vector<float>& interleaved,
                            std::uint32_t sampleRate,
                            std::uint16_t channels)
{
    const auto dataBytes = static_cast<std::uint32_t>(interleaved.size() * sizeof(float));
    const std::uint16_t blockAlign = static_cast<std::uint16_t>(channels * sizeof(float));

    std::vector<std::uint8_t> header;
    header.reserve(44);
    putTag(header, "RIFF");
    put<std::uint32_t>(header, 36U + dataBytes);
    putTag(header, "WAVE");
    putTag(header, "fmt ");
    put<std::uint32_t>(header, 16U);
    put<std::uint16_t>(header, 3U);  // WAVE_FORMAT_IEEE_FLOAT
    put<std::uint16_t>(header, channels);
    put<std::uint32_t>(header, sampleRate);
    put<std::uint32_t>(header, sampleRate * blockAlign);
    put<std::uint16_t>(header, blockAlign);
    put<std::uint16_t>(header, 32U);
    putTag(header, "data");
    put<std::uint32_t>(header, dataBytes);

    std::FILE* file = nullptr;
    if (fopen_s(&file, path.c_str(), "wb") != 0 || file == nullptr)
    {
        std::fprintf(stderr, "error: cannot open '%s' for writing\n", path.c_str());
        return false;
    }

    const bool ok = std::fwrite(header.data(), 1, header.size(), file) == header.size()
        && (dataBytes == 0
            || std::fwrite(interleaved.data(), 1, dataBytes, file) == dataBytes);
    std::fclose(file);

    if (!ok) { std::fprintf(stderr, "error: short write to '%s'\n", path.c_str()); }
    return ok;
}

struct Options final
{
    double seconds { 10.0 };
    std::string outputPath { "capture.wav" };
};

[[nodiscard]] bool parse(int argc, char** argv, Options& options)
{
    for (int i = 1; i < argc; ++i)
    {
        const std::string argument { argv[i] };
        const bool hasValue = (i + 1) < argc;

        if (argument == "--seconds" && hasValue)
        {
            options.seconds = std::strtod(argv[++i], nullptr);
        }
        else if (argument == "--out" && hasValue)
        {
            options.outputPath = argv[++i];
        }
        else
        {
            std::fprintf(stderr, "usage: %s [--seconds N] [--out path.wav]\n", argv[0]);
            return false;
        }
    }

    if (!(options.seconds > 0.0))
    {
        std::fprintf(stderr, "error: --seconds must be positive\n");
        return false;
    }
    return true;
}

}  // namespace

int main(int argc, char** argv)
{
    Options options;
    if (!parse(argc, argv, options)) { return 2; }

    const ComScope com;
    if (FAILED(com.hr))
    {
        std::fprintf(stderr, "error: CoInitializeEx failed (0x%08lX)\n",
                     static_cast<unsigned long>(com.hr));
        return 1;
    }

    ComPtr<IMMDeviceEnumerator> enumerator;
    HRESULT hr = CoCreateInstance(__uuidof(MMDeviceEnumerator), nullptr, CLSCTX_ALL,
                                  __uuidof(IMMDeviceEnumerator), enumerator.putVoid());
    if (FAILED(hr))
    {
        std::fprintf(stderr, "error: no device enumerator (0x%08lX)\n",
                     static_cast<unsigned long>(hr));
        return 1;
    }

    ComPtr<IMMDevice> device;
    hr = enumerator->GetDefaultAudioEndpoint(eRender, eConsole, device.put());
    if (FAILED(hr))
    {
        std::fprintf(stderr, "error: no default render endpoint (0x%08lX)\n",
                     static_cast<unsigned long>(hr));
        return 1;
    }

    ComPtr<IAudioClient> client;
    hr = device->Activate(__uuidof(IAudioClient), CLSCTX_ALL, nullptr, client.putVoid());
    if (FAILED(hr))
    {
        std::fprintf(stderr, "error: cannot activate audio client (0x%08lX)\n",
                     static_cast<unsigned long>(hr));
        return 1;
    }

    WAVEFORMATEX* mixFormat = nullptr;
    hr = client->GetMixFormat(&mixFormat);
    if (FAILED(hr) || mixFormat == nullptr)
    {
        std::fprintf(stderr, "error: cannot read mix format (0x%08lX)\n",
                     static_cast<unsigned long>(hr));
        return 1;
    }

    const SampleEncoding encoding = classify(*mixFormat);
    const std::uint32_t sampleRate = mixFormat->nSamplesPerSec;
    const std::uint16_t channels = mixFormat->nChannels;
    const std::uint16_t frameBytes = mixFormat->nBlockAlign;

    if (encoding == SampleEncoding::unsupported)
    {
        std::fprintf(stderr, "error: unsupported endpoint format (tag %u, %u bits)\n",
                     mixFormat->wFormatTag, mixFormat->wBitsPerSample);
        CoTaskMemFree(mixFormat);
        return 1;
    }

    hr = client->Initialize(AUDCLNT_SHAREMODE_SHARED, AUDCLNT_STREAMFLAGS_LOOPBACK,
                            hundredNanosecondsPerSecond, 0, mixFormat, nullptr);
    CoTaskMemFree(mixFormat);
    mixFormat = nullptr;
    if (FAILED(hr))
    {
        std::fprintf(stderr, "error: cannot initialise loopback capture (0x%08lX)\n",
                     static_cast<unsigned long>(hr));
        return 1;
    }

    ComPtr<IAudioCaptureClient> capture;
    hr = client->GetService(__uuidof(IAudioCaptureClient), capture.putVoid());
    if (FAILED(hr))
    {
        std::fprintf(stderr, "error: cannot obtain capture service (0x%08lX)\n",
                     static_cast<unsigned long>(hr));
        return 1;
    }

    std::printf("endpoint: %u Hz, %u ch, %u-bit\n", sampleRate, channels,
                static_cast<unsigned>(frameBytes * 8U / (channels == 0 ? 1U : channels)));
    std::fflush(stdout);

    const auto targetFrames = static_cast<std::size_t>(
        options.seconds * static_cast<double>(sampleRate));

    std::vector<float> interleaved;
    interleaved.reserve(targetFrames * channels);

    // The loop is driven by the wall clock, never by a frame target. An idle
    // endpoint delivers no packets at all — not even silent ones — so waiting
    // for a frame count hangs forever when nothing is playing, which is
    // precisely the case this recorder has to report rather than survive.
    LARGE_INTEGER counterFrequency {};
    LARGE_INTEGER startCounter {};
    QueryPerformanceFrequency(&counterFrequency);

    hr = client->Start();
    if (FAILED(hr))
    {
        std::fprintf(stderr, "error: cannot start capture (0x%08lX)\n",
                     static_cast<unsigned long>(hr));
        return 1;
    }
    QueryPerformanceCounter(&startCounter);

    const auto elapsedSeconds = [&counterFrequency, &startCounter]() noexcept
    {
        LARGE_INTEGER now {};
        QueryPerformanceCounter(&now);
        return static_cast<double>(now.QuadPart - startCounter.QuadPart)
            / static_cast<double>(counterFrequency.QuadPart);
    };

    std::size_t silentFrames = 0;
    while (elapsedSeconds() < options.seconds)
    {
        UINT32 packetFrames = 0;
        hr = capture->GetNextPacketSize(&packetFrames);
        if (FAILED(hr)) { break; }

        if (packetFrames == 0)
        {
            Sleep(5);
            continue;
        }

        while (packetFrames != 0 && elapsedSeconds() < options.seconds)
        {
            BYTE* data = nullptr;
            UINT32 frames = 0;
            DWORD flags = 0;
            hr = capture->GetBuffer(&data, &frames, &flags, nullptr, nullptr);
            if (FAILED(hr)) { break; }

            // A silent packet carries no valid memory, so it must be
            // synthesised rather than copied. Recording silence is meaningful
            // here: it is how "the app produced nothing" is distinguished from
            // "the capture failed".
            if ((flags & AUDCLNT_BUFFERFLAGS_SILENT) != 0)
            {
                appendSilence(interleaved, frames, channels);
                silentFrames += frames;
            }
            else
            {
                appendFrames(interleaved, data, frames, channels, encoding);
            }

            capture->ReleaseBuffer(frames);
            hr = capture->GetNextPacketSize(&packetFrames);
            if (FAILED(hr)) { break; }
        }
    }

    const double wallSeconds = elapsedSeconds();
    client->Stop();

    const std::size_t frameCount = channels == 0 ? 0 : interleaved.size() / channels;
    const double capturedSeconds = sampleRate == 0
        ? 0.0
        : static_cast<double>(frameCount) / static_cast<double>(sampleRate);
    const double silentShare = frameCount == 0
        ? 1.0
        : static_cast<double>(silentFrames) / static_cast<double>(frameCount);

    double peak = 0.0;
    double sumSquares = 0.0;
    for (const float sample : interleaved)
    {
        const double magnitude = sample < 0.0F ? -static_cast<double>(sample)
                                               : static_cast<double>(sample);
        if (magnitude > peak) { peak = magnitude; }
        sumSquares += static_cast<double>(sample) * static_cast<double>(sample);
    }
    const double rms = interleaved.empty()
        ? 0.0
        : std::sqrt(sumSquares / static_cast<double>(interleaved.size()));

    // The shortfall between audio captured and wall time elapsed is the
    // diagnostic that separates "the source was quiet" from "the endpoint was
    // idle and delivered nothing at all".
    std::printf("captured %.3f s of %.3f s wall (shortfall %.3f s), "
                "peak %.6f, rms %.6f, silent %.1f%%\n",
                capturedSeconds, wallSeconds, wallSeconds - capturedSeconds,
                peak, rms, silentShare * 100.0);

    if (!writeWav(options.outputPath, interleaved, sampleRate, channels)) { return 1; }

    std::printf("wrote %s\n", options.outputPath.c_str());
    return 0;
}
