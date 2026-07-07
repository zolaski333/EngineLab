#pragma once
#include <enginelab/serialization/IEngineSerializer.hpp>
namespace enginelab {
class JsonEngineSerializer final : public IEngineSerializer {
public:
    [[nodiscard]] std::string encode(const EngineConfig&) const override;
    [[nodiscard]] EngineDecodeResult decode(std::string_view) const noexcept override;
};
} // namespace enginelab
