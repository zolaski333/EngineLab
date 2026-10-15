// Block edits of ECU cells: add, scale, set, smooth and interpolate over a
// selection, clamped to the entry's limits.

#include <enginelab/calibration/CalibrationEditing.hpp>

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string_view>
#include <variant>
#include <vector>

namespace {
using namespace enginelab::calibration;

void require(bool condition, std::string_view message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(EXIT_FAILURE);
    }
}

[[nodiscard]] bool near(double a, double b) { return std::abs(a - b) < 1.0e-9; }

/** A 3 x 3 table, values 0..8 row-major, limits 0..20. */
[[nodiscard]] CalibrationEntry table() {
    CalibrationTable2D grid;
    grid.metadata.id = "test.table";
    grid.metadata.hardLimits = { 0.0, 20.0 };
    grid.metadata.displayPrecision = 2;
    grid.xAxis.breakpoints = { 1.0, 2.0, 3.0 };
    grid.yAxis.breakpoints = { 1.0, 2.0, 3.0 };
    grid.values = { 0, 1, 2, 3, 4, 5, 6, 7, 8 };
    return grid;
}

[[nodiscard]] const std::vector<double>& values(const CalibrationEntry& entry) {
    return std::get<CalibrationTable2D>(entry).values;
}

} // namespace

int main() {
    require(cellRectangle(0, 8, 3) == std::vector<std::size_t> { 0, 1, 2, 3, 4, 5, 6, 7, 8 },
        "corner to corner spans the whole grid");
    require(cellRectangle(5, 1, 3) == std::vector<std::size_t> { 1, 2, 4, 5 }, "any two corners span their block");

    auto entry = table();
    require(applyCellOperation(entry, { 0, 4 }, CellOperation::add, 1.5), "adding changes the cells");
    require(near(values(entry)[0], 1.5) && near(values(entry)[4], 5.5) && near(values(entry)[1], 1.0),
        "only the selected cells move");
    require(applyCellOperation(entry, { 8 }, CellOperation::scalePercent, 300.0)
            && near(values(entry)[8], 20.0), "results clamp to the hard limits");
    require(!applyCellOperation(entry, { 8 }, CellOperation::add, 5.0), "a clamped cell that cannot move reports no change");
    require(!applyCellOperation(entry, { 42 }, CellOperation::add, 1.0), "cells outside the grid are ignored");

    entry = table();
    require(!applyCellOperation(entry, { 4 }, CellOperation::smooth), "a plane is already smooth");
    std::get<CalibrationTable2D>(entry).values[4] = 14.0;
    require(applyCellOperation(entry, { 4 }, CellOperation::smooth), "smoothing changes a peak");
    require(near(values(entry)[4], (14.0 + 1.0 + 3.0 + 5.0 + 7.0) / 5.0), "the centre becomes the mean of its cross");

    entry = table();
    auto& grid = std::get<CalibrationTable2D>(entry);
    grid.values = { 0, 9, 9, 9, 9, 9, 9, 9, 8 };
    grid.values[2] = 2.0;
    grid.values[6] = 6.0;
    require(applyCellOperation(entry, cellRectangle(0, 8, 3), CellOperation::interpolate),
        "interpolating fills the block");
    require(near(values(entry)[1], 1.0) && near(values(entry)[4], 4.0) && near(values(entry)[7], 7.0)
            && near(values(entry)[3], 3.0), "bilinearly between its four corners");

    CalibrationCurve1D curve;
    curve.metadata.hardLimits = { 0.0, 100.0 };
    curve.axis.breakpoints = { 1.0, 2.0, 3.0, 4.0 };
    curve.values = { 10.0, 0.0, 0.0, 40.0 };
    CalibrationEntry curveEntry = curve;
    applyCellOperation(curveEntry, { 0, 1, 2, 3 }, CellOperation::interpolate);
    const auto& line = std::get<CalibrationCurve1D>(curveEntry).values;
    require(near(line[1], 20.0) && near(line[2], 30.0), "a curve interpolates between its two ends");

    ScalarCalibration scalar;
    scalar.metadata.hardLimits = { 600.0, 25'000.0 };
    scalar.metadata.displayPrecision = 0;
    scalar.value = 7'000.0;
    CalibrationEntry scalarEntry = scalar;
    require(applyCellOperation(scalarEntry, { 0 }, CellOperation::add, cellStep(scalar.metadata))
            && near(std::get<ScalarCalibration>(scalarEntry).value, 7'010.0), "a whole-number entry steps by 10");
    require(near(cellStep(grid.metadata), 0.1), "a two-decimal entry steps by 0.1");
    std::cout << "calibration editing: ok\n";
    return 0;
}
