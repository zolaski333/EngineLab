#pragma once
#include <enginelab/foundation/EngineTypes.hpp>
#include <optional>
#include <string>
#include <string_view>
namespace enginelab {
struct EngineDecodeResult final {
    std::optional<EngineConfig> config;
    std::string error;
    [[nodiscard]] explicit operator bool() const noexcept { return config.has_value(); }
};
class IEngineSerializer {
public:
    virtual ~IEngineSerializer() = default;
    [[nodiscard]] virtual std::string encode(const EngineConfig&) const = 0;
    [[nodiscard]] virtual EngineDecodeResult decode(std::string_view document) const noexcept = 0;
};
} // namespace enginelab
