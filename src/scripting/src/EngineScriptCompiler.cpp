#include <enginelab/scripting/EngineScriptCompiler.hpp>

#include <enginelab/serialization/JsonEngineSerializer.hpp>
#include <enginelab/serialization/YamlEngineSerializer.hpp>

#include <algorithm>
#include <array>
#include <charconv>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <map>
#include <numbers>
#include <set>
#include <sstream>
#include <system_error>
#include <utility>

namespace enginelab::scripting {
namespace {

enum class Dimension {
    dimensionless,
    length,
    volume,
    mass,
    pressure,
    temperature,
    angle,
    time,
    frequency,
    engineSpeed,
    area,
    massFlow,
    energyPerMass,
    velocity,
    force,
    viscousFriction,
    inertia,
    power,
    torque
};

struct Quantity {
    double value { 0.0 };
    Dimension dimension { Dimension::dimensionless };
};

struct UnitDefinition {
    std::string_view name;
    Dimension dimension;
    double scale;
};

std::string lowercase(std::string_view text) {
    std::string result(text);
    std::transform(result.begin(), result.end(), result.begin(), [](unsigned char character) {
        return static_cast<char>(std::tolower(character));
    });
    return result;
}

std::optional<UnitDefinition> findUnit(std::string_view text) noexcept {
    static constexpr UnitDefinition units[] {
        { "ratio", Dimension::dimensionless, 1.0 },
        { "percent", Dimension::dimensionless, 0.01 }, { "pct", Dimension::dimensionless, 0.01 },
        { "mm", Dimension::length, 1.0 }, { "cm", Dimension::length, 10.0 },
        { "m", Dimension::length, 1'000.0 },
        { "l", Dimension::volume, 1.0 }, { "litre", Dimension::volume, 1.0 },
        { "liter", Dimension::volume, 1.0 }, { "ml", Dimension::volume, 0.001 },
        { "cc", Dimension::volume, 0.001 }, { "cm3", Dimension::volume, 0.001 },
        { "g", Dimension::mass, 1.0 }, { "kg", Dimension::mass, 1'000.0 },
        { "mg", Dimension::mass, 0.001 },
        { "kpa", Dimension::pressure, 1.0 }, { "bar", Dimension::pressure, 100.0 },
        { "pa", Dimension::pressure, 0.001 },
        { "c", Dimension::temperature, 1.0 }, { "degc", Dimension::temperature, 1.0 },
        { "celsius", Dimension::temperature, 1.0 },
        { "deg", Dimension::angle, 1.0 }, { "degree", Dimension::angle, 1.0 },
        { "rad", Dimension::angle, 180.0 / std::numbers::pi },
        { "s", Dimension::time, 1.0 }, { "sec", Dimension::time, 1.0 },
        { "ms", Dimension::time, 0.001 }, { "us", Dimension::time, 0.000001 },
        { "hz", Dimension::frequency, 1.0 }, { "khz", Dimension::frequency, 1'000.0 },
        { "rpm", Dimension::engineSpeed, 1.0 },
        { "mm2", Dimension::area, 1.0 }, { "cm2", Dimension::area, 100.0 },
        { "m2", Dimension::area, 1'000'000.0 },
        { "mg_s", Dimension::massFlow, 1.0 }, { "mgps", Dimension::massFlow, 1.0 },
        { "mg_per_s", Dimension::massFlow, 1.0 }, { "g_s", Dimension::massFlow, 1'000.0 },
        { "kg_s", Dimension::massFlow, 1'000'000.0 },
        { "kj_kg", Dimension::energyPerMass, 1.0 }, { "j_kg", Dimension::energyPerMass, 0.001 },
        { "m_s", Dimension::velocity, 1.0 }, { "mps", Dimension::velocity, 1.0 },
        { "mm_s", Dimension::velocity, 0.001 },
        { "n", Dimension::force, 1.0 }, { "ns_m", Dimension::viscousFriction, 1.0 },
        { "kg_m2", Dimension::inertia, 1.0 },
        { "w", Dimension::power, 1.0 }, { "kw", Dimension::power, 1'000.0 },
        { "nm", Dimension::torque, 1.0 }
    };
    for (const auto& unit : units) if (unit.name == text) return unit;
    return std::nullopt;
}

std::string_view dimensionName(Dimension dimension) noexcept {
    switch (dimension) {
        case Dimension::dimensionless: return "a dimensionless value";
        case Dimension::length: return "a length";
        case Dimension::volume: return "a volume";
        case Dimension::mass: return "a mass";
        case Dimension::pressure: return "a pressure";
        case Dimension::temperature: return "a temperature";
        case Dimension::angle: return "an angle";
        case Dimension::time: return "a time";
        case Dimension::frequency: return "a frequency";
        case Dimension::engineSpeed: return "an engine speed";
        case Dimension::area: return "an area";
        case Dimension::massFlow: return "a mass flow";
        case Dimension::energyPerMass: return "an energy per mass";
        case Dimension::velocity: return "a velocity";
        case Dimension::force: return "a force";
        case Dimension::viscousFriction: return "a viscous-friction coefficient";
        case Dimension::inertia: return "a moment of inertia";
        case Dimension::power: return "a power";
        case Dimension::torque: return "a torque";
    }
    return "a compatible value";
}

enum class TokenKind {
    identifier,
    number,
    string,
    plus,
    minus,
    star,
    slash,
    leftParenthesis,
    rightParenthesis,
    equal,
    comma,
    dot,
    percent,
    separator,
    end
};

struct Token {
    TokenKind kind { TokenKind::end };
    std::string text;
    double number { 0.0 };
    ScriptSourceLocation location;
};

void addDiagnostic(std::vector<EngineScriptDiagnostic>& diagnostics, std::string code,
                   std::string message, ScriptSourceLocation location,
                   ScriptDiagnosticSeverity severity = ScriptDiagnosticSeverity::error) {
    diagnostics.push_back({ severity, std::move(code), std::move(message), std::move(location) });
}

class Lexer final {
public:
    Lexer(std::string_view source, std::filesystem::path sourceName,
          std::vector<EngineScriptDiagnostic>& diagnostics)
        : source_(source), sourceName_(std::move(sourceName)), diagnostics_(diagnostics) {}

    std::vector<Token> scan() {
        std::vector<Token> tokens;
        while (!atEnd()) {
            const auto startLine = line_;
            const auto startColumn = column_;
            const auto character = peek();
            if (character == ' ' || character == '\t' || character == '\r') {
                advance();
            } else if (character == '\n' || character == ';') {
                advance();
                tokens.push_back(makeToken(TokenKind::separator, {}, startLine, startColumn));
            } else if (character == '#' || (character == '/' && peekNext() == '/')) {
                while (!atEnd() && peek() != '\n') advance();
            } else if (std::isalpha(static_cast<unsigned char>(character)) != 0 || character == '_') {
                tokens.push_back(scanIdentifier(startLine, startColumn));
            } else if (std::isdigit(static_cast<unsigned char>(character)) != 0) {
                tokens.push_back(scanNumber(startLine, startColumn));
            } else if (character == '"') {
                if (auto token = scanString(startLine, startColumn)) tokens.push_back(std::move(*token));
            } else {
                advance();
                switch (character) {
                    case '+': tokens.push_back(makeToken(TokenKind::plus, "+", startLine, startColumn)); break;
                    case '-': tokens.push_back(makeToken(TokenKind::minus, "-", startLine, startColumn)); break;
                    case '*': tokens.push_back(makeToken(TokenKind::star, "*", startLine, startColumn)); break;
                    case '/': tokens.push_back(makeToken(TokenKind::slash, "/", startLine, startColumn)); break;
                    case '(': tokens.push_back(makeToken(TokenKind::leftParenthesis, "(", startLine, startColumn)); break;
                    case ')': tokens.push_back(makeToken(TokenKind::rightParenthesis, ")", startLine, startColumn)); break;
                    case '=': tokens.push_back(makeToken(TokenKind::equal, "=", startLine, startColumn)); break;
                    case ',': tokens.push_back(makeToken(TokenKind::comma, ",", startLine, startColumn)); break;
                    case '.': tokens.push_back(makeToken(TokenKind::dot, ".", startLine, startColumn)); break;
                    case '%': tokens.push_back(makeToken(TokenKind::percent, "%", startLine, startColumn)); break;
                    default:
                        addDiagnostic(diagnostics_, "ES100", "Unexpected character in engine script.",
                                      { sourceName_, startLine, startColumn });
                        break;
                }
            }
        }
        tokens.push_back(makeToken(TokenKind::end, {}, line_, column_));
        return tokens;
    }

private:
    [[nodiscard]] bool atEnd() const noexcept { return offset_ >= source_.size(); }
    [[nodiscard]] char peek() const noexcept { return atEnd() ? '\0' : source_[offset_]; }
    [[nodiscard]] char peekNext() const noexcept {
        return offset_ + 1 >= source_.size() ? '\0' : source_[offset_ + 1];
    }
    char advance() noexcept {
        const auto character = source_[offset_++];
        if (character == '\n') { ++line_; column_ = 1; }
        else { ++column_; }
        return character;
    }
    Token makeToken(TokenKind kind, std::string text, std::size_t line, std::size_t column) const {
        return { kind, std::move(text), 0.0, { sourceName_, line, column } };
    }
    Token scanIdentifier(std::size_t line, std::size_t column) {
        const auto start = offset_;
        while (std::isalnum(static_cast<unsigned char>(peek())) != 0 || peek() == '_') advance();
        return makeToken(TokenKind::identifier, std::string(source_.substr(start, offset_ - start)), line, column);
    }
    Token scanNumber(std::size_t line, std::size_t column) {
        const auto start = offset_;
        while (std::isdigit(static_cast<unsigned char>(peek())) != 0) advance();
        if (peek() == '.' && std::isdigit(static_cast<unsigned char>(peekNext())) != 0) {
            advance();
            while (std::isdigit(static_cast<unsigned char>(peek())) != 0) advance();
        }
        if (peek() == 'e' || peek() == 'E') {
            const auto exponentOffset = offset_;
            const auto exponentColumn = column_;
            advance();
            if (peek() == '+' || peek() == '-') advance();
            if (std::isdigit(static_cast<unsigned char>(peek())) == 0) {
                offset_ = exponentOffset;
                column_ = exponentColumn;
            } else {
                while (std::isdigit(static_cast<unsigned char>(peek())) != 0) advance();
            }
        }
        auto text = std::string(source_.substr(start, offset_ - start));
        double value = 0.0;
        const auto conversion = std::from_chars(text.data(), text.data() + text.size(), value);
        if (conversion.ec != std::errc {} || conversion.ptr != text.data() + text.size()) {
            addDiagnostic(diagnostics_, "ES101", "Invalid numeric literal.", { sourceName_, line, column });
        }
        auto token = makeToken(TokenKind::number, std::move(text), line, column);
        token.number = value;
        return token;
    }
    std::optional<Token> scanString(std::size_t line, std::size_t column) {
        advance();
        std::string value;
        while (!atEnd() && peek() != '"') {
            if (peek() == '\n') {
                addDiagnostic(diagnostics_, "ES102", "String literals cannot span lines.",
                              { sourceName_, line, column });
                return std::nullopt;
            }
            auto character = advance();
            if (character == '\\' && !atEnd()) {
                const auto escaped = advance();
                if (escaped == 'n') character = '\n';
                else if (escaped == 't') character = '\t';
                else if (escaped == '"' || escaped == '\\') character = escaped;
                else {
                    addDiagnostic(diagnostics_, "ES103", "Unsupported string escape sequence.",
                                  { sourceName_, line, column });
                    character = escaped;
                }
            }
            value.push_back(character);
        }
        if (atEnd()) {
            addDiagnostic(diagnostics_, "ES104", "Unterminated string literal.",
                          { sourceName_, line, column });
            return std::nullopt;
        }
        advance();
        return makeToken(TokenKind::string, std::move(value), line, column);
    }

    std::string_view source_;
    std::filesystem::path sourceName_;
    std::vector<EngineScriptDiagnostic>& diagnostics_;
    std::size_t offset_ { 0 };
    std::size_t line_ { 1 };
    std::size_t column_ { 1 };
};

class Parser;

struct CompilationContext {
    EngineScriptCompileOptions options;
    EngineConfig config { makeDefaultInlineFour() };
    std::vector<EngineScriptDiagnostic> diagnostics;
    std::set<std::filesystem::path> dependencies;
    std::map<std::string, Quantity, std::less<>> variables;
    std::vector<std::filesystem::path> includeStack;
    std::filesystem::path rootSource;

    void parseText(std::string_view text, const std::filesystem::path& sourceName,
                   const std::filesystem::path& sourceDirectory);
    void parseFile(const std::filesystem::path& path, const ScriptSourceLocation& includeLocation);
    void loadBase(const std::filesystem::path& path, const ScriptSourceLocation& location);
    void selectPreset(std::string_view name, const ScriptSourceLocation& location);
    [[nodiscard]] std::filesystem::path canonical(const std::filesystem::path& path) const;
};

using GlobalNumericSetter = void (*)(EngineConfig&, double);
using CylinderNumericSetter = void (*)(CylinderConfig&, double);

struct GlobalNumericProperty {
    std::string_view path;
    Dimension dimension;
    GlobalNumericSetter setter;
    bool integer { false };
};

struct CylinderNumericProperty {
    std::string_view path;
    Dimension dimension;
    CylinderNumericSetter setter;
    bool integer { false };
};

template <typename Member>
void setEveryIntake(EngineConfig& config, Member IntakeConfig::* member, double value) {
    config.intake.*member = value;
    for (auto& path : config.intakePaths) path.geometry.*member = value;
    config.plenumVolumeLitres = config.intake.plenumVolumeLitres;
    config.throttleDiameterMm = config.intake.throttleDiameterMm;
}

template <typename Member>
void setEveryExhaust(EngineConfig& config, Member ExhaustConfig::* member, double value) {
    config.exhaust.*member = value;
    for (auto& path : config.exhaustPaths) path.geometry.*member = value;
}

void setIntakePlenum(EngineConfig& config, double value) { setEveryIntake(config, &IntakeConfig::plenumVolumeLitres, value); }
void setIntakeThrottleDiameter(EngineConfig& config, double value) { setEveryIntake(config, &IntakeConfig::throttleDiameterMm, value); }
void setIntakeThrottleCount(EngineConfig& config, double value) {
    const auto count = static_cast<std::uint32_t>(std::clamp(std::llround(value), 1LL, 16LL));
    config.intake.throttleCount = count;
    for (auto& path : config.intakePaths) path.geometry.throttleCount = count;
}
void setIntakeThrottleCoefficient(EngineConfig& config, double value) { setEveryIntake(config, &IntakeConfig::throttleDischargeCoefficient, value); }
void setIntakeRunnerLength(EngineConfig& config, double value) { setEveryIntake(config, &IntakeConfig::runnerLengthMm, value); }
void setIntakeRunnerDiameter(EngineConfig& config, double value) { setEveryIntake(config, &IntakeConfig::runnerDiameterMm, value); }
void setIntakeIdleArea(EngineConfig& config, double value) { setEveryIntake(config, &IntakeConfig::idleBypassAreaMm2, value); }
void setIntakeThrottleGamma(EngineConfig& config, double value) { setEveryIntake(config, &IntakeConfig::throttleGamma, value); }
void setExhaustPrimaryLength(EngineConfig& config, double value) { setEveryExhaust(config, &ExhaustConfig::primaryLengthMm, value); }
void setExhaustPrimaryDiameter(EngineConfig& config, double value) { setEveryExhaust(config, &ExhaustConfig::primaryDiameterMm, value); }
void setExhaustCollectorDiameter(EngineConfig& config, double value) { setEveryExhaust(config, &ExhaustConfig::collectorDiameterMm, value); }
void setExhaustRestriction(EngineConfig& config, double value) { setEveryExhaust(config, &ExhaustConfig::mufflerRestriction, value); }
void setExhaustOutletDiameter(EngineConfig& config, double value) { setEveryExhaust(config, &ExhaustConfig::outletDiameterMm, value); }
void setExhaustCollectorVolume(EngineConfig& config, double value) { setEveryExhaust(config, &ExhaustConfig::collectorVolumeLitres, value); }
void setExhaustOutletCoefficient(EngineConfig& config, double value) { setEveryExhaust(config, &ExhaustConfig::outletDischargeCoefficient, value); }
void setRotatingInertia(EngineConfig& config, double value) {
    config.rotatingInertiaKgM2 = value;
    if (config.crankshafts.empty()) return;
    double effective = 0.0;
    double ratioSquaredSum = 0.0;
    for (const auto& crankshaft : config.crankshafts) {
        const auto ratioSquared = crankshaft.rotationRatio * crankshaft.rotationRatio;
        effective += crankshaft.momentOfInertiaKgM2 * ratioSquared;
        ratioSquaredSum += ratioSquared;
    }
    if (effective > 0.0) {
        const auto scale = value / effective;
        for (auto& crankshaft : config.crankshafts) crankshaft.momentOfInertiaKgM2 *= scale;
    } else if (ratioSquaredSum > 0.0) {
        const auto inertia = value / ratioSquaredSum;
        for (auto& crankshaft : config.crankshafts) crankshaft.momentOfInertiaKgM2 = inertia;
    }
}

const GlobalNumericProperty* findGlobalNumericProperty(std::string_view path) noexcept {
    static const GlobalNumericProperty properties[] {
        { "engine.idle_rpm", Dimension::engineSpeed, [](auto& c, double v) { c.idleRpm = v; } },
        { "engine.redline_rpm", Dimension::engineSpeed, [](auto& c, double v) { c.redlineRpm = v; } },
        { "engine.rotating_inertia_kg_m2", Dimension::inertia, setRotatingInertia },
        { "engine.friction_coefficient", Dimension::dimensionless, [](auto& c, double v) { c.frictionCoefficient = v; } },
        { "engine.octane_rating", Dimension::dimensionless, [](auto& c, double v) { c.octaneRating = v; } },
        { "engine.ambient_pressure_kpa", Dimension::pressure, [](auto& c, double v) { c.ambientPressureKpa = v; } },
        { "engine.ambient_temperature_c", Dimension::temperature, [](auto& c, double v) { c.ambientTemperatureC = v; } },
        { "engine.cooling_efficiency", Dimension::dimensionless, [](auto& c, double v) { c.coolingEfficiency = v; } },
        { "engine.bank_angle_deg", Dimension::angle, [](auto& c, double v) { c.bankAngleDegrees = v; } },
        { "intake.plenum_volume_l", Dimension::volume, setIntakePlenum },
        { "intake.throttle_diameter_mm", Dimension::length, setIntakeThrottleDiameter },
        { "intake.throttle_count", Dimension::dimensionless, setIntakeThrottleCount, true },
        { "intake.throttle_discharge_coefficient", Dimension::dimensionless, setIntakeThrottleCoefficient },
        { "intake.runner_length_mm", Dimension::length, setIntakeRunnerLength },
        { "intake.runner_diameter_mm", Dimension::length, setIntakeRunnerDiameter },
        { "intake.idle_bypass_area_mm2", Dimension::area, setIntakeIdleArea },
        { "intake.throttle_gamma", Dimension::dimensionless, setIntakeThrottleGamma },
        { "exhaust.primary_length_mm", Dimension::length, setExhaustPrimaryLength },
        { "exhaust.primary_diameter_mm", Dimension::length, setExhaustPrimaryDiameter },
        { "exhaust.collector_diameter_mm", Dimension::length, setExhaustCollectorDiameter },
        { "exhaust.muffler_restriction", Dimension::dimensionless, setExhaustRestriction },
        { "exhaust.outlet_diameter_mm", Dimension::length, setExhaustOutletDiameter },
        { "exhaust.collector_volume_l", Dimension::volume, setExhaustCollectorVolume },
        { "exhaust.outlet_discharge_coefficient", Dimension::dimensionless, setExhaustOutletCoefficient },
        { "ignition.rev_limit_rpm", Dimension::engineSpeed, [](auto& c, double v) { c.ignition.revLimitRpm = v; } },
        { "ignition.limiter_duration_s", Dimension::time, [](auto& c, double v) { c.ignition.limiterDurationSeconds = v; } },
        { "injection.start_angle_deg", Dimension::angle, [](auto& c, double v) { c.injection.startAngleDegrees = v; } },
        { "injection.end_angle_deg", Dimension::angle, [](auto& c, double v) { c.injection.endAngleDegrees = v; } },
        { "injection.injector_flow_mg_s", Dimension::massFlow, [](auto& c, double v) { c.injection.injectorFlowMgPerSecond = v; } },
        { "injection.fuel_temperature_c", Dimension::temperature, [](auto& c, double v) { c.injection.fuelTemperatureC = v; } },
        { "injection.rail_pressure_bar", Dimension::pressure, [](auto& c, double v) { c.injection.railPressureBar = v / 100.0; } },
        { "injection.reference_pressure_bar", Dimension::pressure, [](auto& c, double v) { c.injection.referencePressureBar = v / 100.0; } },
        { "injection.wall_film_fraction", Dimension::dimensionless, [](auto& c, double v) { c.injection.wallFilmFraction = v; } },
        { "injection.vaporisation_time_s", Dimension::time, [](auto& c, double v) { c.injection.vaporisationTimeConstantSeconds = v; } },
        { "injection.latent_heat_kj_kg", Dimension::energyPerMass, [](auto& c, double v) { c.injection.latentHeatKjPerKg = v; } },
        { "injection.direct_charge_cooling_efficiency", Dimension::dimensionless, [](auto& c, double v) { c.injection.directChargeCoolingEfficiency = v; } },
        { "injection.port_charge_cooling_efficiency", Dimension::dimensionless, [](auto& c, double v) { c.injection.portChargeCoolingEfficiency = v; } },
        { "solver.mechanical_frequency_hz", Dimension::frequency, [](auto& c, double v) { c.solver.mechanicalFrequencyHz = v; } },
        { "solver.maximum_frequency_hz", Dimension::frequency, [](auto& c, double v) { c.solver.maximumMechanicalFrequencyHz = v; } },
        { "solver.maximum_crank_deg_per_step", Dimension::angle, [](auto& c, double v) { c.solver.maximumCrankDegreesPerStep = v; } },
        { "solver.gas_substeps", Dimension::dimensionless,
          [](auto& c, double v) { c.solver.gasSubsteps = static_cast<std::uint32_t>(v); }, true },
        { "forced_induction.pressure_ratio", Dimension::dimensionless, [](auto& c, double v) { c.forcedInduction.pressureRatio = v; } },
        { "forced_induction.full_boost_rpm", Dimension::engineSpeed, [](auto& c, double v) { c.forcedInduction.fullBoostRpm = v; } },
        { "forced_induction.compressor_efficiency", Dimension::dimensionless, [](auto& c, double v) { c.forcedInduction.compressorEfficiency = v; } },
        { "forced_induction.charge_temperature_rise_c", Dimension::temperature, [](auto& c, double v) { c.forcedInduction.chargeTemperatureRiseC = v; } },
        { "forced_induction.turbine_efficiency", Dimension::dimensionless, [](auto& c, double v) { c.forcedInduction.turbineEfficiency = v; } },
        { "forced_induction.shaft_inertia_kg_m2", Dimension::inertia, [](auto& c, double v) { c.forcedInduction.shaftInertiaKgM2 = v; } },
        { "forced_induction.wastegate_pressure_ratio", Dimension::dimensionless, [](auto& c, double v) { c.forcedInduction.wastegatePressureRatio = v; } },
        { "forced_induction.design_shaft_speed_rpm", Dimension::engineSpeed, [](auto& c, double v) { c.forcedInduction.designShaftSpeedRpm = v; } },
        { "forced_induction.bearing_friction_power_w", Dimension::power, [](auto& c, double v) { c.forcedInduction.bearingFrictionPowerWatts = v; } },
        { "forced_induction.turbine_flow_area_mm2", Dimension::area, [](auto& c, double v) { c.forcedInduction.turbineFlowAreaMm2 = v; } },
        { "forced_induction.wastegate_flow_area_mm2", Dimension::area, [](auto& c, double v) { c.forcedInduction.wastegateFlowAreaMm2 = v; } }
    };
    for (const auto& property : properties) if (property.path == path) return &property;
    return nullptr;
}

const CylinderNumericProperty* findCylinderNumericProperty(std::string_view path) noexcept {
    static const CylinderNumericProperty properties[] {
        { "bore_mm", Dimension::length, [](auto& c, double v) { c.boreMm = v; } },
        { "stroke_mm", Dimension::length, [](auto& c, double v) { c.strokeMm = v; } },
        { "connecting_rod_mm", Dimension::length, [](auto& c, double v) { c.connectingRodMm = v; } },
        { "piston_mass_g", Dimension::mass, [](auto& c, double v) { c.pistonMassGrams = v; } },
        { "compression_ratio", Dimension::dimensionless, [](auto& c, double v) { c.compressionRatio = v; } },
        { "ignition_offset_deg", Dimension::angle, [](auto& c, double v) { c.ignitionOffsetDegrees = v; } },
        { "efficiency_offset", Dimension::dimensionless, [](auto& c, double v) { c.efficiencyOffset = v; } },
        { "crank_offset_deg", Dimension::angle, [](auto& c, double v) { c.crankOffsetDegrees = v; } },
        { "crank_journal_id", Dimension::dimensionless, [](auto& c, double v) { c.crankJournalId = static_cast<std::uint32_t>(v); }, true },
        { "bank_offset_deg", Dimension::angle, [](auto& c, double v) { c.bankOffsetDegrees = v; } },
        { "bank_id", Dimension::dimensionless, [](auto& c, double v) { c.bankId = static_cast<std::uint32_t>(v); }, true },
        { "intake_runner_length_mm", Dimension::length, [](auto& c, double v) { c.intakeRunnerLengthMm = v; } },
        { "intake_runner_diameter_mm", Dimension::length, [](auto& c, double v) { c.intakeRunnerDiameterMm = v; } },
        { "exhaust_primary_length_mm", Dimension::length, [](auto& c, double v) { c.exhaustPrimaryLengthMm = v; } },
        { "sound_attenuation", Dimension::dimensionless, [](auto& c, double v) { c.soundAttenuation = v; } },
        { "blow_by_coefficient", Dimension::dimensionless, [](auto& c, double v) { c.blowByCoefficient = v; } },
        { "connecting_rod_mass_g", Dimension::mass, [](auto& c, double v) { c.connectingRodMassGrams = v; } },
        { "piston_friction_coefficient", Dimension::dimensionless, [](auto& c, double v) { c.pistonFrictionCoefficient = v; } },
        { "piston_breakaway_force_n", Dimension::force, [](auto& c, double v) { c.pistonBreakawayForceN = v; } },
        { "piston_breakaway_velocity_m_s", Dimension::velocity, [](auto& c, double v) { c.pistonBreakawayVelocityMps = v; } },
        { "piston_viscous_friction_ns_m", Dimension::viscousFriction, [](auto& c, double v) { c.pistonViscousFrictionNsPerM = v; } },
        { "master_cylinder_id", Dimension::dimensionless, [](auto& c, double v) { c.masterCylinderId = static_cast<std::uint32_t>(v); }, true },
        { "articulated_journal_radius_mm", Dimension::length, [](auto& c, double v) { c.articulatedJournalRadiusMm = v; } },
        { "articulated_journal_angle_deg", Dimension::angle, [](auto& c, double v) { c.articulatedJournalAngleDegrees = v; } },
        { "deck_height_mm", Dimension::length, [](auto& c, double v) { c.deckHeightMm = v; } },
        { "compression_height_mm", Dimension::length, [](auto& c, double v) { c.compressionHeightMm = v; } },
        { "wrist_pin_offset_mm", Dimension::length, [](auto& c, double v) { c.wristPinOffsetMm = v; } },
        { "piston_crown_volume_cc", Dimension::volume, [](auto& c, double v) { c.pistonCrownVolumeCc = v * 1'000.0; } },
        { "head_chamber_volume_cc", Dimension::volume, [](auto& c, double v) { c.headChamberVolumeCc = v * 1'000.0; } },
        { "head_gasket_thickness_mm", Dimension::length, [](auto& c, double v) { c.headGasketThicknessMm = v; } },
        { "connecting_rod_inertia_kg_m2", Dimension::inertia, [](auto& c, double v) { c.connectingRodMomentOfInertiaKgM2 = v; } },
        { "intake_valve_count", Dimension::dimensionless, [](auto& c, double v) { c.intakeValveCount = static_cast<std::uint32_t>(v); }, true },
        { "exhaust_valve_count", Dimension::dimensionless, [](auto& c, double v) { c.exhaustValveCount = static_cast<std::uint32_t>(v); }, true },
        { "intake_valve_diameter_mm", Dimension::length, [](auto& c, double v) { c.intakeValveDiameterMm = v; } },
        { "exhaust_valve_diameter_mm", Dimension::length, [](auto& c, double v) { c.exhaustValveDiameterMm = v; } }
    };
    for (const auto& property : properties) if (property.path == path) return &property;
    return nullptr;
}

class Parser final {
public:
    Parser(CompilationContext& context, std::vector<Token> tokens,
           std::filesystem::path sourceDirectory)
        : context_(context), tokens_(std::move(tokens)), sourceDirectory_(std::move(sourceDirectory)) {}

    void parse() {
        while (!check(TokenKind::end)) {
            while (match(TokenKind::separator)) {}
            if (check(TokenKind::end)) break;
            const auto statementStart = current_;
            if (!check(TokenKind::identifier)) {
                error(peek(), "ES200", "Expected an engine-script statement.");
                skipStatement();
                continue;
            }
            const auto command = lowercase(advance().text);
            if (command == "preset") parsePreset();
            else if (command == "base") parseBase();
            else if (command == "include") parseInclude();
            else if (command == "engine" || command == "name") parseEngineName(command == "engine");
            else if (command == "let") parseLet();
            else if (command == "set") parseSet();
            else if (command == "ignition") parseIgnition();
            else {
                error(tokens_[statementStart], "ES201", "Unknown statement '" + command + "'.");
                skipStatement();
                continue;
            }
            finishStatement();
        }
    }

private:
    [[nodiscard]] const Token& peek() const noexcept { return tokens_[current_]; }
    [[nodiscard]] const Token& previous() const noexcept { return tokens_[current_ - 1]; }
    [[nodiscard]] bool check(TokenKind kind) const noexcept { return peek().kind == kind; }
    const Token& advance() noexcept {
        if (!check(TokenKind::end)) ++current_;
        return previous();
    }
    bool match(TokenKind kind) noexcept {
        if (!check(kind)) return false;
        advance();
        return true;
    }
    void error(const Token& token, std::string code, std::string message) {
        addDiagnostic(context_.diagnostics, std::move(code), std::move(message), token.location);
    }
    void skipStatement() noexcept {
        while (!check(TokenKind::separator) && !check(TokenKind::end)) advance();
        while (match(TokenKind::separator)) {}
    }
    void finishStatement() {
        if (!check(TokenKind::separator) && !check(TokenKind::end)) {
            error(peek(), "ES202", "Unexpected token at the end of the statement.");
            skipStatement();
        } else {
            while (match(TokenKind::separator)) {}
        }
    }
    bool require(TokenKind kind, std::string_view description) {
        if (match(kind)) return true;
        error(peek(), "ES203", "Expected " + std::string(description) + ".");
        return false;
    }
    std::optional<std::string> stringOrIdentifier(std::string_view description) {
        if (check(TokenKind::string) || check(TokenKind::identifier)) return advance().text;
        error(peek(), "ES204", "Expected " + std::string(description) + ".");
        return std::nullopt;
    }

    void parsePreset() {
        const auto location = peek().location;
        if (const auto name = stringOrIdentifier("a preset name")) context_.selectPreset(*name, location);
    }

    void parseBase() {
        const auto location = peek().location;
        if (!check(TokenKind::string)) {
            error(peek(), "ES205", "base requires a quoted YAML or JSON path.");
            return;
        }
        const auto relative = std::filesystem::path(advance().text);
        context_.loadBase(sourceDirectory_ / relative, location);
    }

    void parseInclude() {
        const auto location = peek().location;
        if (!check(TokenKind::string)) {
            error(peek(), "ES206", "include requires a quoted engine-script path.");
            return;
        }
        const auto relative = std::filesystem::path(advance().text);
        context_.parseFile(sourceDirectory_ / relative, location);
    }

    void parseEngineName(bool engineKeyword) {
        if (engineKeyword) {
            if (match(TokenKind::slash)) {
                if (!check(TokenKind::identifier) || lowercase(peek().text) != "name") {
                    error(peek(), "ES207", "Only engine/name is supported after '/'.");
                    return;
                }
                advance();
            } else if (check(TokenKind::identifier) && lowercase(peek().text) == "name") {
                advance();
            }
        }
        match(TokenKind::equal);
        if (!check(TokenKind::string)) {
            error(peek(), "ES208", "An engine name must be a quoted string.");
            return;
        }
        context_.config.name = advance().text;
    }

    void parseLet() {
        if (!check(TokenKind::identifier)) {
            error(peek(), "ES209", "let requires a variable identifier.");
            return;
        }
        const auto variableToken = advance();
        const auto variableName = lowercase(variableToken.text);
        if (!require(TokenKind::equal, "'=' after the variable name")) return;
        const auto value = parseExpression();
        if (!value) return;
        if (context_.variables.contains(variableName)) {
            error(variableToken, "ES210", "Variable '" + variableName + "' is already defined.");
            return;
        }
        context_.variables.emplace(variableName, *value);
    }

    std::optional<std::string> parsePropertyPath() {
        std::string result;
        auto expectingSegment = true;
        while (true) {
            if (expectingSegment) {
                if (check(TokenKind::identifier) || check(TokenKind::number)) {
                    if (!result.empty()) result.push_back('.');
                    result += lowercase(advance().text);
                    expectingSegment = false;
                } else {
                    error(peek(), "ES211", "Expected a property path segment.");
                    return std::nullopt;
                }
            } else if (match(TokenKind::dot)) {
                expectingSegment = true;
            } else {
                break;
            }
        }
        if (expectingSegment) {
            error(peek(), "ES212", "Property paths cannot end with '.'.");
            return std::nullopt;
        }
        return result;
    }

    void parseSet() {
        const auto location = peek().location;
        auto path = parsePropertyPath();
        if (!path || !require(TokenKind::equal, "'=' after the property path")) return;
        if (path->find('.') == std::string::npos && *path != "name") *path = "engine." + *path;
        applyProperty(*path, location);
    }

    std::optional<bool> parseBoolean() {
        if (!check(TokenKind::identifier)) {
            error(peek(), "ES213", "Expected true or false.");
            return std::nullopt;
        }
        const auto token = advance();
        const auto value = lowercase(token.text);
        if (value == "true" || value == "on" || value == "yes") return true;
        if (value == "false" || value == "off" || value == "no") return false;
        error(token, "ES214", "Expected true or false.");
        return std::nullopt;
    }

    std::optional<std::string> parseSymbol() {
        if (!check(TokenKind::identifier) && !check(TokenKind::string)) {
            error(peek(), "ES215", "Expected a symbolic value.");
            return std::nullopt;
        }
        return lowercase(advance().text);
    }

    bool validateNumeric(const Quantity& value, Dimension expected, bool integer,
                         const ScriptSourceLocation& location) {
        if (value.dimension != expected) {
            addDiagnostic(context_.diagnostics, "ES216",
                          "This property requires " + std::string(dimensionName(expected))
                              + ", but the expression has incompatible units.", location);
            return false;
        }
        if (!std::isfinite(value.value)) {
            addDiagnostic(context_.diagnostics, "ES217", "Numeric expressions must remain finite.", location);
            return false;
        }
        if (integer && (value.value < 0.0
                || value.value > static_cast<double>(std::numeric_limits<std::uint32_t>::max())
                || std::floor(value.value) != value.value)) {
            addDiagnostic(context_.diagnostics, "ES218",
                          "This property requires a non-negative whole number.", location);
            return false;
        }
        return true;
    }

    void applyProperty(const std::string& path, const ScriptSourceLocation& location) {
        if (path == "engine.name" || path == "name") {
            if (!check(TokenKind::string)) {
                error(peek(), "ES219", "engine.name requires a quoted string.");
                return;
            }
            context_.config.name = advance().text;
            return;
        }
        if (path == "forced_induction.enabled") {
            if (const auto value = parseBoolean()) context_.config.forcedInduction.enabled = *value;
            return;
        }
        if (path == "engine.cycle") {
            if (const auto value = parseSymbol()) {
                if (*value == "four_stroke" || *value == "fourstroke") context_.config.cycle = EngineCycle::fourStroke;
                else if (*value == "two_stroke" || *value == "twostroke") context_.config.cycle = EngineCycle::twoStroke;
                else addDiagnostic(context_.diagnostics, "ES220", "Unknown engine cycle '" + *value + "'.", location);
            }
            return;
        }
        if (path == "engine.layout") {
            if (const auto value = parseSymbol()) {
                if (*value == "inline") context_.config.layout = EngineLayout::inlineLayout;
                else if (*value == "v") context_.config.layout = EngineLayout::vLayout;
                else if (*value == "flat" || *value == "boxer") context_.config.layout = EngineLayout::flat;
                else if (*value == "radial") context_.config.layout = EngineLayout::radial;
                else if (*value == "custom") context_.config.layout = EngineLayout::custom;
                else addDiagnostic(context_.diagnostics, "ES221", "Unknown engine layout '" + *value + "'.", location);
            }
            return;
        }
        if (path == "engine.fuel") {
            if (const auto value = parseSymbol()) {
                if (*value == "gasoline" || *value == "petrol") context_.config.fuel = FuelType::gasoline;
                else if (*value == "diesel") context_.config.fuel = FuelType::diesel;
                else addDiagnostic(context_.diagnostics, "ES222", "Unknown fuel type '" + *value + "'.", location);
            }
            return;
        }
        if (path == "injection.mode") {
            if (const auto value = parseSymbol()) {
                if (*value == "port") context_.config.injection.mode = InjectionMode::port;
                else if (*value == "direct" || *value == "di") context_.config.injection.mode = InjectionMode::direct;
                else addDiagnostic(context_.diagnostics, "ES223", "Unknown injection mode '" + *value + "'.", location);
            }
            return;
        }
        if (path == "forced_induction.type") {
            if (const auto value = parseSymbol()) {
                if (*value == "turbo" || *value == "turbocharger")
                    context_.config.forcedInduction.type = ForcedInductionType::turbocharger;
                else if (*value == "supercharger")
                    context_.config.forcedInduction.type = ForcedInductionType::supercharger;
                else addDiagnostic(context_.diagnostics, "ES224", "Unknown forced-induction type '" + *value + "'.", location);
            }
            return;
        }
        if (path.rfind("cylinder.", 0) == 0) {
            applyCylinderProperty(path, location);
            return;
        }
        if (const auto* property = findGlobalNumericProperty(path)) {
            const auto value = parseExpression();
            if (value && validateNumeric(*value, property->dimension, property->integer, location))
                property->setter(context_.config, value->value);
            return;
        }
        addDiagnostic(context_.diagnostics, "ES225", "Unknown or unsupported property '" + path + "'.", location);
    }

    void applyCylinderProperty(const std::string& path, const ScriptSourceLocation& location) {
        constexpr std::string_view prefix = "cylinder.";
        const auto propertySeparator = path.find('.', prefix.size());
        if (propertySeparator == std::string::npos) {
            addDiagnostic(context_.diagnostics, "ES226",
                          "Cylinder paths use cylinder.<id|all>.<property>.", location);
            return;
        }
        const auto selector = std::string_view(path).substr(prefix.size(), propertySeparator - prefix.size());
        const auto propertyName = std::string_view(path).substr(propertySeparator + 1);
        std::vector<CylinderConfig*> targets;
        if (selector == "all") {
            targets.reserve(context_.config.cylinders.size());
            for (auto& cylinder : context_.config.cylinders) targets.push_back(&cylinder);
            if (targets.empty()) {
                addDiagnostic(context_.diagnostics, "ES228",
                              "The selected base engine has no cylinders to edit.", location);
                return;
            }
        } else {
            std::uint32_t id = 0;
            const auto conversion = std::from_chars(selector.data(), selector.data() + selector.size(), id);
            if (conversion.ec != std::errc {} || conversion.ptr != selector.data() + selector.size()) {
                addDiagnostic(context_.diagnostics, "ES227", "Invalid cylinder identifier.", location);
                return;
            }
            const auto found = std::find_if(context_.config.cylinders.begin(), context_.config.cylinders.end(),
                                            [id](const CylinderConfig& cylinder) { return cylinder.id == id; });
            if (found == context_.config.cylinders.end()) {
                addDiagnostic(context_.diagnostics, "ES228",
                              "Cylinder " + std::to_string(id) + " does not exist in the selected base engine.", location);
                return;
            }
            targets.push_back(&*found);
        }

        if (propertyName == "connecting_rod_type") {
            const auto value = parseSymbol();
            if (!value) return;
            std::optional<ConnectingRodType> type;
            if (*value == "conventional") type = ConnectingRodType::conventional;
            else if (*value == "master") type = ConnectingRodType::master;
            else if (*value == "articulated") type = ConnectingRodType::articulated;
            else {
                addDiagnostic(context_.diagnostics, "ES229", "Unknown connecting-rod type '" + *value + "'.", location);
                return;
            }
            for (auto* cylinder : targets) cylinder->connectingRodType = *type;
            return;
        }
        const auto* property = findCylinderNumericProperty(propertyName);
        if (property == nullptr) {
            addDiagnostic(context_.diagnostics, "ES230",
                          "Unknown or unsupported cylinder property '" + std::string(propertyName) + "'.", location);
            return;
        }
        const auto value = parseExpression();
        if (!value || !validateNumeric(*value, property->dimension, property->integer, location)) return;
        for (auto* cylinder : targets) {
            property->setter(*cylinder, value->value);
            if (propertyName == "stroke_mm" || propertyName == "crank_offset_deg"
                || propertyName == "bank_offset_deg") {
                const auto journal = std::find_if(context_.config.crankJournals.begin(),
                                                  context_.config.crankJournals.end(),
                    [cylinder](const CrankJournalConfig& item) { return item.id == cylinder->crankJournalId; });
                if (journal != context_.config.crankJournals.end()) {
                    if (propertyName == "stroke_mm") journal->throwMm = cylinder->strokeMm * 0.5;
                    else journal->angleDegrees = cylinder->crankOffsetDegrees - cylinder->bankOffsetDegrees;
                }
            }
        }
    }

    void parseIgnition() {
        if (!check(TokenKind::identifier)) {
            error(peek(), "ES231", "ignition expects clear or point.");
            return;
        }
        const auto operation = lowercase(advance().text);
        if (operation == "clear") {
            context_.config.ignition.timingCurve.clear();
            return;
        }
        if (operation != "point") {
            error(previous(), "ES232", "ignition expects clear or point.");
            return;
        }
        const auto location = peek().location;
        const auto speed = parseExpression();
        if (!speed) return;
        if (!require(TokenKind::comma, "',' between ignition speed and advance")) return;
        const auto advanceValue = parseExpression();
        if (!advanceValue) return;
        if (!validateNumeric(*speed, Dimension::engineSpeed, false, location)
            || !validateNumeric(*advanceValue, Dimension::angle, false, location)) return;
        const auto existing = std::find_if(context_.config.ignition.timingCurve.begin(),
                                           context_.config.ignition.timingCurve.end(),
            [&speed](const IgnitionMapSample& sample) {
                return std::abs(sample.rpm - speed->value) <= 1.0e-9;
            });
        if (existing == context_.config.ignition.timingCurve.end())
            context_.config.ignition.timingCurve.push_back({ speed->value, advanceValue->value });
        else
            existing->advanceDegrees = advanceValue->value;
    }

    std::optional<Quantity> parseExpression() { return parseAddition(); }

    std::optional<Quantity> parseAddition() {
        auto left = parseMultiplication();
        while (left && (check(TokenKind::plus) || check(TokenKind::minus))) {
            const auto operation = advance();
            const auto right = parseMultiplication();
            if (!right) return std::nullopt;
            if (left->dimension != right->dimension) {
                error(operation, "ES233", "Addition and subtraction require matching units.");
                return std::nullopt;
            }
            left->value = operation.kind == TokenKind::plus
                ? left->value + right->value : left->value - right->value;
            if (!std::isfinite(left->value)) {
                error(operation, "ES234", "Expression overflowed to a non-finite value.");
                return std::nullopt;
            }
        }
        return left;
    }

    std::optional<Quantity> parseMultiplication() {
        auto left = parseUnary();
        while (left && (check(TokenKind::star) || check(TokenKind::slash))) {
            const auto operation = advance();
            const auto right = parseUnary();
            if (!right) return std::nullopt;
            if (operation.kind == TokenKind::star) {
                if (left->dimension != Dimension::dimensionless
                    && right->dimension != Dimension::dimensionless) {
                    error(operation, "ES235", "Multiplication of two dimensional values is not supported.");
                    return std::nullopt;
                }
                if (left->dimension == Dimension::dimensionless) left->dimension = right->dimension;
                left->value *= right->value;
            } else {
                if (right->value == 0.0) {
                    error(operation, "ES236", "Division by zero.");
                    return std::nullopt;
                }
                if (right->dimension == Dimension::dimensionless) {
                    left->value /= right->value;
                } else if (left->dimension == right->dimension) {
                    left = Quantity { left->value / right->value, Dimension::dimensionless };
                } else {
                    error(operation, "ES237", "Division requires a dimensionless divisor or matching units.");
                    return std::nullopt;
                }
            }
            if (!std::isfinite(left->value)) {
                error(operation, "ES238", "Expression overflowed to a non-finite value.");
                return std::nullopt;
            }
        }
        return left;
    }

    std::optional<Quantity> parseUnary() {
        if (match(TokenKind::plus)) return parseUnary();
        if (match(TokenKind::minus)) {
            const auto operation = previous();
            auto value = parseUnary();
            if (!value) return std::nullopt;
            value->value = -value->value;
            if (!std::isfinite(value->value)) {
                error(operation, "ES239", "Expression produced a non-finite value.");
                return std::nullopt;
            }
            return value;
        }
        return parsePrimary();
    }

    std::optional<Quantity> parsePrimary() {
        if (check(TokenKind::number)) {
            const auto number = advance();
            Quantity result { number.number, Dimension::dimensionless };
            if (match(TokenKind::percent)) {
                result.value *= 0.01;
            } else if (check(TokenKind::identifier)) {
                const auto unitText = lowercase(peek().text);
                if (const auto unit = findUnit(unitText)) {
                    advance();
                    result.value *= unit->scale;
                    result.dimension = unit->dimension;
                }
            }
            return result;
        }
        if (check(TokenKind::identifier)) {
            const auto token = advance();
            const auto name = lowercase(token.text);
            if (name == "pi") return Quantity { std::numbers::pi, Dimension::dimensionless };
            const auto found = context_.variables.find(name);
            if (found == context_.variables.end()) {
                error(token, "ES240", "Unknown variable or missing numeric literal before unit '" + name + "'.");
                return std::nullopt;
            }
            return found->second;
        }
        if (match(TokenKind::leftParenthesis)) {
            const auto value = parseExpression();
            if (!require(TokenKind::rightParenthesis, "')' after the expression")) return std::nullopt;
            return value;
        }
        error(peek(), "ES241", "Expected a numeric expression.");
        return std::nullopt;
    }

    CompilationContext& context_;
    std::vector<Token> tokens_;
    std::filesystem::path sourceDirectory_;
    std::size_t current_ { 0 };
};

std::filesystem::path CompilationContext::canonical(const std::filesystem::path& path) const {
    std::error_code error;
    auto absolute = path;
    if (!absolute.is_absolute()) absolute = std::filesystem::absolute(path, error);
    if (error) absolute = path;
    error.clear();
    const auto resolved = std::filesystem::weakly_canonical(absolute, error);
    return (error ? absolute.lexically_normal() : resolved).lexically_normal();
}

void CompilationContext::parseText(std::string_view text, const std::filesystem::path& sourceName,
                                   const std::filesystem::path& sourceDirectory) {
    Lexer lexer(text, sourceName, diagnostics);
    Parser parser(*this, lexer.scan(), sourceDirectory);
    parser.parse();
}

void CompilationContext::parseFile(const std::filesystem::path& path,
                                   const ScriptSourceLocation& includeLocation) {
    const auto resolved = canonical(path);
    dependencies.insert(resolved);
    if (includeStack.size() >= options.maximumIncludeDepth) {
        addDiagnostic(diagnostics, "ES300", "Maximum include depth exceeded.", includeLocation);
        return;
    }
    if (std::find(includeStack.begin(), includeStack.end(), resolved) != includeStack.end()) {
        std::ostringstream chain;
        for (const auto& item : includeStack) chain << item.filename().string() << " -> ";
        chain << resolved.filename().string();
        addDiagnostic(diagnostics, "ES301", "Include cycle detected: " + chain.str(), includeLocation);
        return;
    }

    std::error_code error;
    const auto size = std::filesystem::file_size(resolved, error);
    if (error) {
        addDiagnostic(diagnostics, "ES302", "Cannot read engine script '" + resolved.string() + "'.",
                      includeLocation);
        return;
    }
    if (size > options.maximumSourceBytes) {
        addDiagnostic(diagnostics, "ES303", "Engine script exceeds the configured size limit.",
                      includeLocation);
        return;
    }
    std::ifstream input(resolved, std::ios::binary);
    if (!input) {
        addDiagnostic(diagnostics, "ES302", "Cannot open engine script '" + resolved.string() + "'.",
                      includeLocation);
        return;
    }
    std::string text(static_cast<std::size_t>(size), '\0');
    if (size > 0) input.read(text.data(), static_cast<std::streamsize>(size));
    if (!input && size > 0) {
        addDiagnostic(diagnostics, "ES304", "Failed while reading engine script '" + resolved.string() + "'.",
                      includeLocation);
        return;
    }

    includeStack.push_back(resolved);
    parseText(text, resolved, resolved.parent_path());
    includeStack.pop_back();
}

void CompilationContext::loadBase(const std::filesystem::path& path,
                                  const ScriptSourceLocation& location) {
    const auto resolved = canonical(path);
    dependencies.insert(resolved);
    std::error_code error;
    const auto size = std::filesystem::file_size(resolved, error);
    if (error) {
        addDiagnostic(diagnostics, "ES305", "Cannot read base engine file '" + resolved.string() + "'.", location);
        return;
    }
    if (size > options.maximumSourceBytes) {
        addDiagnostic(diagnostics, "ES306", "Base engine file exceeds the configured size limit.", location);
        return;
    }
    std::ifstream input(resolved, std::ios::binary);
    std::string text(static_cast<std::size_t>(size), '\0');
    if (size > 0) input.read(text.data(), static_cast<std::streamsize>(size));
    if (!input || (size > 0 && !input.good() && !input.eof())) {
        addDiagnostic(diagnostics, "ES307", "Failed while reading base engine file '" + resolved.string() + "'.", location);
        return;
    }

    auto extension = lowercase(resolved.extension().string());
    EngineDecodeResult decoded;
    if (extension == ".json") {
        decoded = JsonEngineSerializer {}.decode(text);
    } else if (extension == ".yaml" || extension == ".yml") {
        decoded = YamlEngineSerializer {}.decode(text);
    } else {
        const auto first = std::find_if_not(text.begin(), text.end(), [](unsigned char character) {
            return std::isspace(character) != 0;
        });
        decoded = first != text.end() && *first == '{'
            ? JsonEngineSerializer {}.decode(text) : YamlEngineSerializer {}.decode(text);
    }
    if (!decoded) {
        addDiagnostic(diagnostics, "ES308", "Invalid base engine: " + decoded.error, location);
        return;
    }
    config = std::move(*decoded.config);
}

void CompilationContext::selectPreset(std::string_view name, const ScriptSourceLocation& location) {
    const auto normalized = lowercase(name);
    if (normalized == "inline_four" || normalized == "i4") config = makeDefaultInlineFour();
    else if (normalized == "inline_two" || normalized == "i2") config = makeDefaultInlineTwo();
    else if (normalized == "inline_five" || normalized == "i5") config = makeDefaultInlineFive();
    else if (normalized == "v6") config = makeDefaultV6();
    else if (normalized == "v8") config = makeDefaultV8();
    else if (normalized == "flat_six" || normalized == "boxer_six") config = makeDefaultFlatSix();
    else if (normalized == "radial_five") config = makeDefaultRadialFive();
    else addDiagnostic(diagnostics, "ES309", "Unknown built-in preset '" + normalized + "'.", location);
}

bool hasErrors(const std::vector<EngineScriptDiagnostic>& diagnostics) noexcept {
    return std::any_of(diagnostics.begin(), diagnostics.end(), [](const auto& diagnostic) {
        return diagnostic.severity == ScriptDiagnosticSeverity::error;
    });
}

EngineScriptCompileResult finishCompilation(CompilationContext context) {
    EngineScriptCompileResult result;
    result.dependencies.assign(context.dependencies.begin(), context.dependencies.end());
    if (!hasErrors(context.diagnostics)) {
        std::sort(context.config.ignition.timingCurve.begin(), context.config.ignition.timingCurve.end(),
                  [](const IgnitionMapSample& left, const IgnitionMapSample& right) {
                      return left.rpm < right.rpm;
                  });
        normaliseEngineConfig(context.config);
        if (const auto validationError = validateEngineConfig(context.config)) {
            addDiagnostic(context.diagnostics, "ES400", "Final engine validation failed: " + *validationError,
                          { context.rootSource, 1, 1 });
        }
    }
    if (!hasErrors(context.diagnostics)) result.config = std::move(context.config);
    result.diagnostics = std::move(context.diagnostics);
    return result;
}

} // namespace

EngineScriptCompileResult EngineScriptCompiler::compileFile(
    const std::filesystem::path& path, const EngineScriptCompileOptions& options) const {
    CompilationContext context;
    context.options = options;
    context.rootSource = context.canonical(path);
    context.parseFile(path, { context.rootSource, 1, 1 });
    return finishCompilation(std::move(context));
}

EngineScriptCompileResult EngineScriptCompiler::compileText(
    std::string_view sourceText, std::filesystem::path sourceName,
    const EngineScriptCompileOptions& options) const {
    CompilationContext context;
    context.options = options;
    context.rootSource = std::move(sourceName);
    auto directory = options.baseDirectory;
    if (directory.empty()) {
        std::error_code error;
        directory = std::filesystem::current_path(error);
        if (error) directory = ".";
    }
    if (sourceText.size() > options.maximumSourceBytes) {
        addDiagnostic(context.diagnostics, "ES303", "Engine script exceeds the configured size limit.",
                      { context.rootSource, 1, 1 });
    } else {
        context.parseText(sourceText, context.rootSource, context.canonical(directory));
    }
    return finishCompilation(std::move(context));
}

} // namespace enginelab::scripting
