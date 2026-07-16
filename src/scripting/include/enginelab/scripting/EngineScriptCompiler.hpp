#pragma once

#include <enginelab/foundation/EngineTypes.hpp>

#include <cstddef>
#include <filesystem>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace enginelab::scripting {

enum class ScriptDiagnosticSeverity { warning, error };

struct ScriptSourceLocation {
    std::filesystem::path source;
    std::size_t line { 1 };
    std::size_t column { 1 };
};

struct EngineScriptDiagnostic {
    ScriptDiagnosticSeverity severity { ScriptDiagnosticSeverity::error };
    std::string code;
    std::string message;
    ScriptSourceLocation location;
};

struct EngineScriptCompileOptions {
    /** Relative include/base paths in compileText() are resolved from this directory. */
    std::filesystem::path baseDirectory;
    std::size_t maximumIncludeDepth { 32 };
    std::size_t maximumSourceBytes { 2U * 1024U * 1024U };
};

struct EngineScriptCompileResult {
    std::optional<EngineConfig> config;
    std::vector<EngineScriptDiagnostic> diagnostics;
    /** Root script, includes and YAML/JSON bases, in deterministic canonical order. */
    std::vector<std::filesystem::path> dependencies;

    [[nodiscard]] explicit operator bool() const noexcept { return config.has_value(); }
};

/** Safe, declarative and unit-aware compiler from EngineLab script to EngineConfig. */
class EngineScriptCompiler final {
public:
    [[nodiscard]] EngineScriptCompileResult compileFile(
        const std::filesystem::path& path,
        const EngineScriptCompileOptions& options = {}) const;

    [[nodiscard]] EngineScriptCompileResult compileText(
        std::string_view sourceText,
        std::filesystem::path sourceName = "<memory>",
        const EngineScriptCompileOptions& options = {}) const;
};

} // namespace enginelab::scripting

