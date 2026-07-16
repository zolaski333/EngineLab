#include <enginelab/calibration/CalibrationJson.hpp>
#include <enginelab/calibration/CalibrationFileHotReloader.hpp>
#include <enginelab/calibration/EcuCalibration.hpp>
#include <enginelab/calibration/EcuCalibrationKeys.hpp>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <thread>
#include <variant>
#include <vector>

using namespace enginelab::calibration;

namespace {

void require(bool condition, const char* message) {
    if (condition) return;
    std::cerr << "Calibration test failure: " << message << '\n';
    std::exit(1);
}

void requireNear(double actual, double expected, double tolerance, const char* message) {
    require(std::isfinite(actual) && std::abs(actual - expected) <= tolerance, message);
}

CalibrationMetadata metadata(std::string id, std::string displayName, CalibrationUnit unit,
                             std::optional<double> minimum = {}, std::optional<double> maximum = {}) {
    CalibrationMetadata result;
    result.id = std::move(id);
    result.displayName = std::move(displayName);
    result.unit = unit;
    result.hardLimits = { minimum, maximum };
    return result;
}

CalibrationAxis rpmAxis() {
    return { { "rpm", "Engine speed", AxisQuantity::engineSpeed,
               CalibrationUnit::revolutionsPerMinute },
             { 0.0, 1'000.0, 2'000.0 } };
}

CalibrationAxis loadAxis() {
    return { { "load", "Normalized load", AxisQuantity::normalizedLoad,
               CalibrationUnit::ratio },
             { 0.0, 1.0 } };
}

AxisCoordinate rpm(double value) {
    return { AxisQuantity::engineSpeed, CalibrationUnit::revolutionsPerMinute, value };
}

AxisCoordinate load(double value) {
    return { AxisQuantity::normalizedLoad, CalibrationUnit::ratio, value };
}

CalibrationDraft representativeDraft(double offset = 0.0) {
    CalibrationDraft draft;
    draft.name = "Road calibration";
    draft.description = "Deterministic test calibration";
    draft.set(ScalarCalibration {
        metadata(std::string(keys::revLimit), "Rev limit", CalibrationUnit::revolutionsPerMinute,
                 ecuLimits::minimumRevLimitRpm, ecuLimits::maximumRevLimitRpm),
        7'500.0 + offset
    });
    draft.set(CalibrationCurve1D {
        metadata(std::string(keys::targetAirFuelRatio), "Target AFR", CalibrationUnit::airFuelRatio,
                 ecuLimits::minimumAirFuelRatio, ecuLimits::maximumAirFuelRatio),
        rpmAxis(),
        { 14.7 + offset, 14.0 + offset, 13.0 + offset }
    });
    draft.set(CalibrationTable2D {
        metadata(std::string(keys::ignitionAdvance), "Ignition advance",
                 CalibrationUnit::degreeCrankshaft,
                 ecuLimits::minimumIgnitionAdvanceDegrees,
                 ecuLimits::maximumIgnitionAdvanceDegrees),
        rpmAxis(), loadAxis(),
        { 0.0 + offset, 10.0 + offset, 20.0 + offset,
          30.0 + offset, 40.0 + offset, 50.0 + offset }
    });
    return draft;
}

void testAxisValidationAndInterpolation() {
    auto draft = representativeDraft();
    require(validate(draft).empty(), "representative draft should validate");
    CalibrationStore store;
    const auto result = store.publish(draft, { 0, "unit-test" });
    require(result.published && result.activeRevision == 1, "first transaction should publish revision one");
    const auto snapshot = store.snapshot();

    requireNear(snapshot->sampleCurveOr(keys::targetAirFuelRatio, rpm(500.0), -1.0), 14.35, 1.0e-12,
                "1D curve should interpolate linearly");
    requireNear(snapshot->sampleCurveOr(keys::targetAirFuelRatio, rpm(-500.0), -1.0), 14.7, 1.0e-12,
                "1D curve should clamp below its axis");
    requireNear(snapshot->sampleCurveOr(keys::targetAirFuelRatio, rpm(9'000.0), -1.0), 13.0, 1.0e-12,
                "1D curve should clamp above its axis");
    requireNear(snapshot->sampleTableOr(keys::ignitionAdvance, rpm(500.0), load(0.25), -1.0), 12.5,
                1.0e-12, "2D table should use row-major bilinear interpolation");
    requireNear(snapshot->sampleTableOr(keys::ignitionAdvance, rpm(-1.0), load(2.0), -1.0), 30.0,
                1.0e-12, "2D table should clamp both axes");

    const AxisCoordinate wrongUnit { AxisQuantity::engineSpeed, CalibrationUnit::hertz, 500.0 };
    require(!snapshot->sampleTable(keys::ignitionAdvance, wrongUnit, load(0.5)).has_value(),
            "typed coordinates should reject a unit mismatch");
    const AxisCoordinate wrongQuantity { AxisQuantity::vehicleSpeed,
                                         CalibrationUnit::revolutionsPerMinute, 500.0 };
    require(!snapshot->sampleCurve(keys::targetAirFuelRatio, wrongQuantity).has_value(),
            "typed coordinates should reject a quantity mismatch");
}

void testInvalidDraftNeverPublishes() {
    CalibrationStore store;
    require(store.publish(representativeDraft()).published, "valid baseline should publish");
    const auto baseline = store.snapshot();

    auto invalid = representativeDraft();
    auto badAxis = rpmAxis();
    badAxis.metadata.unit = CalibrationUnit::hertz;
    badAxis.breakpoints = { 0.0, 1'000.0, 1'000.0 };
    invalid.set(CalibrationCurve1D {
        metadata("invalid.curve", "Invalid curve", CalibrationUnit::ratio, 0.0, 1.0),
        badAxis, { 0.1, std::numeric_limits<double>::quiet_NaN(), 2.0 }
    });
    const auto rejected = store.publish(invalid);
    require(!rejected.published && !rejected.issues.empty(), "invalid transaction should be rejected");
    require(std::any_of(rejected.issues.begin(), rejected.issues.end(), [](const auto& issue) {
        return issue.code == CalibrationErrorCode::incompatibleUnit;
    }), "axis quantity/unit incompatibility should be reported");
    require(std::any_of(rejected.issues.begin(), rejected.issues.end(), [](const auto& issue) {
        return issue.code == CalibrationErrorCode::invalidValue;
    }), "non-finite and out-of-range calibration values should be reported");
    const auto after = store.snapshot();
    require(after.get() == baseline.get() && after->revision() == 1,
            "rejection must leave the exact previous snapshot active");

    const auto conflict = store.publish(representativeDraft(0.1), { 0, "stale-editor" });
    require(!conflict.published && conflict.issues.size() == 1
            && conflict.issues.front().code == CalibrationErrorCode::revisionConflict,
            "optimistic concurrency should reject a stale editor transaction");
    require(store.snapshot().get() == baseline.get(), "revision conflict must not replace the snapshot");
}

void testKnownEcuContractRejectsSilentlyIgnoredMaps() {
    CalibrationStore store;

    CalibrationDraft scalarAfr;
    scalarAfr.set(ScalarCalibration {
        metadata(std::string(keys::targetAirFuelRatio), "Target AFR",
                 CalibrationUnit::airFuelRatio,
                 ecuLimits::minimumAirFuelRatio, ecuLimits::maximumAirFuelRatio),
        14.2
    });
    const auto wrongKind = store.publish(scalarAfr);
    require(!wrongKind.published && std::any_of(wrongKind.issues.begin(), wrongKind.issues.end(),
        [](const auto& issue) { return issue.code == CalibrationErrorCode::invalidDimensions; }),
        "known mapped ECU keys must reject scalar data");

    auto percentLoad = loadAxis();
    percentLoad.metadata.unit = CalibrationUnit::percent;
    percentLoad.breakpoints = { 0.0, 100.0 };
    CalibrationDraft wrongLoadUnit;
    wrongLoadUnit.set(CalibrationTable2D {
        metadata(std::string(keys::targetAirFuelRatio), "Target AFR",
                 CalibrationUnit::airFuelRatio,
                 ecuLimits::minimumAirFuelRatio, ecuLimits::maximumAirFuelRatio),
        rpmAxis(), percentLoad,
        { 14.7, 14.2, 13.8, 14.5, 14.0, 13.5 }
    });
    const auto incompatibleAxis = store.publish(wrongLoadUnit);
    require(!incompatibleAxis.published && std::any_of(
        incompatibleAxis.issues.begin(), incompatibleAxis.issues.end(), [](const auto& issue) {
            return issue.code == CalibrationErrorCode::invalidAxis;
        }), "a percent load table must not validate then be ignored by a ratio reader");

    CalibrationDraft tooWide;
    tooWide.set(CalibrationCurve1D {
        metadata(std::string(keys::ignitionAdvance), "Ignition advance",
                 CalibrationUnit::degreeCrankshaft, -20.0, 80.0),
        rpmAxis(), { 0.0, 10.0, 20.0 }
    });
    const auto incompatibleLimits = store.publish(tooWide);
    require(!incompatibleLimits.published && std::any_of(
        incompatibleLimits.issues.begin(), incompatibleLimits.issues.end(), [](const auto& issue) {
            return issue.code == CalibrationErrorCode::invalidMetadata;
        }), "known ECU metadata must not promise ranges wider than runtime behavior");
}

void testJsonRoundTripAndTransactionalFailure() {
    const auto encoded = CalibrationJson::serialize(representativeDraft());
    require(encoded.hasValue(), "a valid draft should serialize");
    const auto parsed = CalibrationJson::parse(encoded.value());
    require(parsed.hasValue(), "serialized calibration should parse");
    require(parsed.value().entries().size() == 3, "round trip should preserve every calibration");

    CalibrationStore store;
    const auto published = publishJson(store, encoded.value(), { 0, "memory://round-trip" });
    require(published.published, "valid JSON should publish atomically");
    const auto baseline = store.snapshot();
    requireNear(baseline->sampleTableOr(keys::ignitionAdvance, rpm(1'500.0), load(0.5), -1.0),
                30.0, 1.0e-12, "round-trip table should retain orientation and values");

    const auto rejected = publishJson(store,
        R"({"schema_version":1,"calibrations":[{"kind":"scalar","metadata":{"id":"x","display_name":"X","unit":"rpm"},"value":"not-a-number"}]})");
    require(!rejected.published && !rejected.issues.empty(), "malformed JSON calibration should be rejected");
    require(store.snapshot().get() == baseline.get(), "JSON error must leave the previous calibration active");

    const auto snapshotText = CalibrationJson::serialize(*baseline);
    const auto snapshotDocument = CalibrationJson::parse(snapshotText);
    require(snapshotDocument.hasValue(), "snapshot JSON should remain a valid editable document");
}

void testReadersOnlyObserveCompleteTransactions() {
    CalibrationStore store;
    CalibrationDraft initial;
    initial.set(ScalarCalibration { metadata("transaction.a", "A", CalibrationUnit::dimensionless), 0.0 });
    initial.set(ScalarCalibration { metadata("transaction.b", "B", CalibrationUnit::dimensionless), 0.0 });
    require(store.publish(initial).published, "concurrency baseline should publish");

    std::atomic<bool> running { true };
    std::atomic<bool> consistent { true };
    std::thread reader([&] {
        std::uint64_t lastRevision = 0;
        while (running.load(std::memory_order_acquire)) {
            const auto current = store.snapshot();
            const auto a = current->scalarOr("transaction.a", -1.0);
            const auto b = current->scalarOr("transaction.b", -2.0);
            if (a != b || current->revision() < lastRevision) {
                consistent.store(false, std::memory_order_release);
                break;
            }
            lastRevision = current->revision();
        }
    });

    for (int revision = 1; revision <= 250; ++revision) {
        CalibrationDraft next;
        const auto value = static_cast<double>(revision);
        next.set(ScalarCalibration { metadata("transaction.a", "A", CalibrationUnit::dimensionless), value });
        next.set(ScalarCalibration { metadata("transaction.b", "B", CalibrationUnit::dimensionless), value });
        require(store.publish(next).published, "writer transaction should publish");
    }
    running.store(false, std::memory_order_release);
    reader.join();
    require(consistent.load(std::memory_order_acquire),
            "reader should never observe mixed entries or decreasing revisions");
}

void testDefaultEcuMapsAndFileHotReload() {
    const auto config = enginelab::makeDefaultInlineFour();
    const auto defaults = makeDefaultEcuCalibration(config);
    require(validate(defaults).empty(), "generated ECU maps should validate");
    CalibrationStore store;
    require(store.publish(defaults).published, "generated ECU maps should publish");
    const auto baseline = store.snapshot();
    require(baseline->find(keys::targetAirFuelRatio) != nullptr,
            "generated ECU maps should contain AFR");
    require(baseline->find(keys::ignitionAdvance) != nullptr,
            "generated ECU maps should contain ignition advance");
    requireNear(baseline->scalarOr(keys::revLimit, -1.0), config.ignition.revLimitRpm,
                1.0e-12, "generated rev limiter should match the engine configuration");
    const auto* afrEntry = baseline->find(keys::targetAirFuelRatio);
    const auto* afrTable = afrEntry == nullptr ? nullptr : std::get_if<CalibrationTable2D>(afrEntry);
    require(afrTable != nullptr && afrTable->yAxis.breakpoints.back() == 4.0,
            "default ECU maps should cover boosted load through 400 percent");
    require(baseline->sampleTableOr(keys::targetAirFuelRatio, rpm(3'500.0), load(2.0), 99.0)
            < baseline->sampleTableOr(keys::targetAirFuelRatio, rpm(3'500.0), load(1.0), 99.0),
            "boosted load cells should command additional enrichment");

    const auto path = std::filesystem::temp_directory_path()
        / "enginelab-calibration-hot-reload-test.json";
    std::error_code error;
    std::filesystem::remove(path, error);
    CalibrationFileHotReloader reloader(store);
    reloader.watch(path);
    const auto encoded = CalibrationJson::serialize(representativeDraft(0.25));
    require(encoded.hasValue(), "hot-reload fixture should serialize");
    {
        std::ofstream output(path, std::ios::binary | std::ios::trunc);
        output << encoded.value();
    }
    const auto loaded = reloader.poll();
    require(loaded.changed && loaded.published(), "new calibration file should hot reload");
    const auto validSnapshot = store.snapshot();
    require(validSnapshot->revision() == baseline->revision() + 1,
            "hot reload should publish exactly one revision");

    {
        std::ofstream output(path, std::ios::binary | std::ios::trunc);
        output << "{ invalid calibration";
    }
    const auto rejected = reloader.poll();
    require(rejected.changed && !rejected.published() && !rejected.error.empty(),
            "invalid external save should be reported");
    require(store.snapshot().get() == validSnapshot.get(),
            "invalid external save must retain the last valid live snapshot");
    std::filesystem::remove(path, error);
    const auto missing = reloader.poll();
    require(missing.changed && !missing.error.empty(),
            "deleting a watched calibration should report one state transition");
    const auto stillMissing = reloader.poll();
    require(!stillMissing.changed && !stillMissing.error.empty(),
            "a persistent file error should be deduplicated instead of alerting every poll");
}

} // namespace

int main() {
    testAxisValidationAndInterpolation();
    testInvalidDraftNeverPublishes();
    testKnownEcuContractRejectsSilentlyIgnoredMaps();
    testJsonRoundTripAndTransactionalFailure();
    testReadersOnlyObserveCompleteTransactions();
    testDefaultEcuMapsAndFileHotReload();
    std::cout << "EngineLab calibration tests passed\n";
    return 0;
}
