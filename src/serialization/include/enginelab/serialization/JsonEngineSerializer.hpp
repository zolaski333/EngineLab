#pragma once
#include <enginelab/serialization/IEngineSerializer.hpp>
#include <string>
#include <vector>
namespace enginelab {
class JsonEngineSerializer final : public IEngineSerializer {
public:
    [[nodiscard]] std::string encode(const EngineConfig&) const override;
    [[nodiscard]] EngineDecodeResult decode(std::string_view) const noexcept override;
    /** `repairs` non-null: a value outside its range is clamped into it and
     *  reported there instead of refusing the file (a user's hand-edited
     *  engine; see repairEngineConfigRanges()). */
    [[nodiscard]] EngineDecodeResult decode(std::string_view, std::vector<std::string>* repairs) const noexcept;
};
} // namespace enginelab
