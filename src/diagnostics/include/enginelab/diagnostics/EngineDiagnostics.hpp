#pragma once
#include <enginelab/foundation/EngineTypes.hpp>
#include <cstdint>
#include <string>
#include <vector>
namespace enginelab {
enum class DiagnosticSeverity : std::uint8_t { information, warning, critical };
struct Diagnostic final { DiagnosticSeverity severity; std::string code; std::string message; };
class EngineDiagnostics final {
public:
    /** Runs outside the audio callback and may allocate diagnostic text. */
    [[nodiscard]] std::vector<Diagnostic> evaluate(const EngineConfig&, const EngineState&) const;
};
} // namespace enginelab
