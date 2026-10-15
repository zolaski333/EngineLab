#include <enginelab/calibration/CalibrationEditing.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <type_traits>
#include <variant>

namespace enginelab::calibration {
namespace {

[[nodiscard]] double clampToLimits(const CalibrationMetadata& metadata, double value) noexcept {
    const auto low = metadata.hardLimits.minimum.value_or(-std::numeric_limits<double>::infinity());
    const auto high = metadata.hardLimits.maximum.value_or(std::numeric_limits<double>::infinity());
    return std::clamp(value, low, high);
}

/** The grid behind an entry: its values and its width and height. */
struct Grid final {
    std::vector<double>* values { nullptr };
    std::size_t columns { 1 };
    std::size_t rows { 1 };
};

[[nodiscard]] double operated(double value, CellOperation operation, double amount) noexcept {
    switch (operation) {
        case CellOperation::add: return value + amount;
        case CellOperation::scalePercent: return value * (1.0 + amount / 100.0);
        case CellOperation::set: return amount;
        case CellOperation::smooth:
        case CellOperation::interpolate: break;
    }
    return value;
}

bool applyToGrid(const CalibrationMetadata& metadata, const Grid& grid,
                 const std::vector<std::size_t>& selected, CellOperation operation, double amount) {
    auto& values = *grid.values;
    std::vector<std::size_t> cells;
    for (const auto index : selected)
        if (index < values.size()) cells.push_back(index);
    std::sort(cells.begin(), cells.end());
    cells.erase(std::unique(cells.begin(), cells.end()), cells.end());
    if (cells.empty()) return false;
    const auto before = values;
    if (operation == CellOperation::smooth) {
        // Read from the unmodified values so the result does not depend on
        // the order cells are visited in.
        for (const auto index : cells) {
            const auto x = index % grid.columns;
            const auto y = index / grid.columns;
            auto sum = before[index];
            auto count = 1.0;
            const auto add = [&](std::size_t neighbour) { sum += before[neighbour]; count += 1.0; };
            if (x > 0) add(index - 1);
            if (x + 1 < grid.columns) add(index + 1);
            if (y > 0) add(index - grid.columns);
            if (y + 1 < grid.rows) add(index + grid.columns);
            values[index] = sum / count;
        }
    } else if (operation == CellOperation::interpolate) {
        auto left = grid.columns, right = std::size_t { 0 }, top = grid.rows, bottom = std::size_t { 0 };
        for (const auto index : cells) {
            left = std::min(left, index % grid.columns);
            right = std::max(right, index % grid.columns);
            top = std::min(top, index / grid.columns);
            bottom = std::max(bottom, index / grid.columns);
        }
        const auto at = [&](std::size_t x, std::size_t y) { return before[y * grid.columns + x]; };
        const auto topLeft = at(left, top);
        const auto topRight = at(right, top);
        const auto bottomLeft = at(left, bottom);
        const auto bottomRight = at(right, bottom);
        for (const auto index : cells) {
            const auto x = index % grid.columns;
            const auto y = index / grid.columns;
            const auto u = right > left ? static_cast<double>(x - left) / static_cast<double>(right - left) : 0.0;
            const auto v = bottom > top ? static_cast<double>(y - top) / static_cast<double>(bottom - top) : 0.0;
            values[index] = std::lerp(std::lerp(topLeft, topRight, u), std::lerp(bottomLeft, bottomRight, u), v);
        }
    } else {
        for (const auto index : cells) values[index] = operated(values[index], operation, amount);
    }
    for (const auto index : cells) values[index] = clampToLimits(metadata, values[index]);
    return values != before;
}

} // namespace

bool applyCellOperation(CalibrationEntry& entry, const std::vector<std::size_t>& selected,
                        CellOperation operation, double amount) {
    return std::visit([&](auto& calibrationEntry) {
        using T = std::decay_t<decltype(calibrationEntry)>;
        if constexpr (std::is_same_v<T, ScalarCalibration>) {
            if (std::find(selected.begin(), selected.end(), std::size_t { 0 }) == selected.end()) return false;
            if (operation == CellOperation::smooth || operation == CellOperation::interpolate) return false;
            const auto value = clampToLimits(calibrationEntry.metadata,
                operated(calibrationEntry.value, operation, amount));
            const auto changed = value != calibrationEntry.value;
            calibrationEntry.value = value;
            return changed;
        } else if constexpr (std::is_same_v<T, CalibrationCurve1D>) {
            return applyToGrid(calibrationEntry.metadata,
                Grid { &calibrationEntry.values, std::max<std::size_t>(1, calibrationEntry.values.size()), 1 },
                selected, operation, amount);
        } else {
            const auto columns = std::max<std::size_t>(1, calibrationEntry.xAxis.breakpoints.size());
            return applyToGrid(calibrationEntry.metadata,
                Grid { &calibrationEntry.values, columns, calibrationEntry.values.size() / columns },
                selected, operation, amount);
        }
    }, entry);
}

double cellStep(const CalibrationMetadata& metadata) noexcept {
    return std::pow(10.0, 1.0 - static_cast<double>(std::clamp(metadata.displayPrecision, 0, 6)));
}

std::vector<std::size_t> cellRectangle(std::size_t anchor, std::size_t corner, std::size_t columns) {
    const auto width = std::max<std::size_t>(1, columns);
    const auto left = std::min(anchor % width, corner % width);
    const auto right = std::max(anchor % width, corner % width);
    const auto top = std::min(anchor / width, corner / width);
    const auto bottom = std::max(anchor / width, corner / width);
    std::vector<std::size_t> cells;
    for (auto y = top; y <= bottom; ++y)
        for (auto x = left; x <= right; ++x) cells.push_back(y * width + x);
    return cells;
}

} // namespace enginelab::calibration
