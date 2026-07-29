#include <enginelab/audio/StructuralModalRadiator.hpp>
#include <enginelab/catalog/EngineCatalog.hpp>
#include <enginelab/foundation/EngineTypes.hpp>
#include <enginelab/serialization/JsonEngineSerializer.hpp>
#include <enginelab/serialization/YamlEngineSerializer.hpp>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <numbers>
#include <string_view>
#include <system_error>

namespace {
void require(bool condition, std::string_view message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(1);
    }
}

class TemporaryCatalog final {
public:
    TemporaryCatalog() {
        const auto stamp = std::chrono::high_resolution_clock::now()
            .time_since_epoch().count();
        root_ = std::filesystem::temp_directory_path()
            / ("enginelab-structural-nvh-"
                + std::to_string(stamp));
        std::filesystem::create_directories(root_ / "parts");
        std::filesystem::create_directories(root_ / "engines");

        for (const auto name : {
                 "fuels.yaml", "injections.yaml",
                 "camshafts.yaml", "exhausts.yaml",
                 "transmissions.yaml", "vehicles.yaml" }) {
            std::ofstream(root_ / "parts" / name) << "{}\n";
        }

        std::ofstream(root_ / "engines" / "01_nvh_fixture.engine.yaml")
            << R"(schema_version: 5
family: test fixture
engine:
  name: Structural NVH fixture
  layout: inline
  cylinder_count: 4
  firing_order: [1, 3, 4, 2]
  idle_rpm: 850
  redline_rpm: 7000
  rotating_inertia_kg_m2: 0.24
  friction_coefficient: 0.12
  octane_rating: 98
  plenum_volume_l: 3.0
  throttle_diameter_mm: 60.0
  cylinder_template:
    bore_mm: 86.0
    stroke_mm: 86.0
    connecting_rod_mm: 143.0
    piston_mass_g: 420.0
    compression_ratio: 10.5
  structural_nvh:
    provenance: measured
    source: "fixture modal survey, accelerometer set A, revision 2"
    modes:
      - name: "block vertical bending 1"
        drive: bearing_axial
        frequency_hz: 1234.5
        damping_ratio: 0.025
        modal_mass_kg: 4.2
        radiating_area_m2: 0.18
        radiation_efficiency: 0.42
        surface_velocity_rms_scale: 0.50
        torque_radius_m: 0.06
        cylinder_participation: [1.0, -0.65, 0.65, -1.0]
)";
    }

    ~TemporaryCatalog() {
        std::error_code ignored;
        std::filesystem::remove_all(root_, ignored);
    }

    [[nodiscard]] const std::filesystem::path& root() const noexcept {
        return root_;
    }

private:
    std::filesystem::path root_;
};
} // namespace

int main() {
    const auto fallbackConfig = enginelab::makeDefaultInlineFour();
    const enginelab::StructuralModalRadiator fallback(
        fallbackConfig);
    require(fallback.valid() && fallback.modeCount() >= 8
            && fallback.provenance()
                == enginelab::StructuralNvhProvenance::estimatedFamily
            && !fallback.source().empty(),
        "an engine without authored data must retain a labelled family estimate");

    TemporaryCatalog fixture;
    const auto loaded =
        enginelab::loadEngineCatalog(fixture.root());
    require(loaded.errors.empty() && loaded.entries.size() == 1,
        "catalogue YAML must accept a complete measured structural mode");
    const auto& config = loaded.entries.front().config;
    require(!enginelab::validateEngineConfig(config)
            && config.structuralNvh.provenance
                == enginelab::StructuralNvhProvenance::measured
            && config.structuralNvh.modes.size() == 1,
        "measured catalogue NVH must validate without falling back");

    enginelab::StructuralModalRadiator measured(config);
    require(measured.valid() && measured.modeCount() == 1
            && measured.provenance()
                == enginelab::StructuralNvhProvenance::measured
            && measured.source() == config.structuralNvh.source,
        "the radiator must expose measured provenance and its auditable source");
    const auto mode = measured.mode(0);
    require(std::abs(mode.frequencyHz - 1'234.5) < 1.0e-9
            && std::abs(mode.dampingRatio - 0.025) < 1.0e-12
            && std::abs(mode.modalMassKg - 4.2) < 1.0e-12
            && std::abs(mode.radiatingAreaM2 - 0.18) < 1.0e-12
            && std::abs(mode.radiationEfficiency - 0.42) < 1.0e-12,
        "authored SI modal parameters must reach the runtime exactly");
    require(measured.prepare(48'000.0),
        "a measured audio-band mode must prepare");

    enginelab::StructuralExcitationSample excitation;
    excitation.cylinderCount = config.cylinders.size();
    auto energy = 0.0;
    for (std::size_t sample = 0; sample < 48'000; ++sample) {
        excitation.bearingReactionForceN[0] =
            800.0F * static_cast<float>(std::sin(
                2.0 * std::numbers::pi * mode.frequencyHz
                * static_cast<double>(sample) / 48'000.0));
        const auto pressure = measured.process(excitation);
        require(std::isfinite(pressure),
            "measured modal integration must remain finite");
        if (sample >= 24'000)
            energy += static_cast<double>(pressure) * pressure;
    }
    const auto rmsPa = std::sqrt(energy / 24'000.0);
    require(rmsPa > 1.0e-6,
        "the authored bearing mode must be physically excitable");

    const enginelab::JsonEngineSerializer json;
    const enginelab::YamlEngineSerializer yaml;
    const auto jsonRoundTrip = json.decode(json.encode(config));
    const auto yamlRoundTrip = yaml.decode(yaml.encode(config));
    require(jsonRoundTrip && yamlRoundTrip
            && jsonRoundTrip.config->structuralNvh.provenance
                == enginelab::StructuralNvhProvenance::measured
            && yamlRoundTrip.config->structuralNvh.source
                == config.structuralNvh.source
            && jsonRoundTrip.config->structuralNvh.modes.front()
                .drive == enginelab::StructuralModeDrive::bearingAxial
            && yamlRoundTrip.config->structuralNvh.modes.front()
                .cylinderParticipation.size() == 4,
        "JSON and YAML must preserve provenance, source, drive and mode shape");

    auto falseMeasured = fallbackConfig;
    falseMeasured.structuralNvh.provenance =
        enginelab::StructuralNvhProvenance::measured;
    falseMeasured.structuralNvh.source = "claim without data";
    require(enginelab::validateEngineConfig(falseMeasured).has_value(),
        "measured provenance without measured modes must be rejected");
    auto wrongShape = config;
    wrongShape.structuralNvh.modes.front()
        .cylinderParticipation.pop_back();
    require(enginelab::validateEngineConfig(wrongShape).has_value(),
        "a modal shape that omits a cylinder must be rejected");

    std::cout
        << "provenance=measured modes=1 frequency_hz="
        << mode.frequencyHz << " damping=" << mode.dampingRatio
        << " modal_mass_kg=" << mode.modalMassKg
        << " response_rms_pa=" << rmsPa << '\n'
        << "PASS: measured/configured structural NVH path and provenance gates\n";
    return 0;
}
