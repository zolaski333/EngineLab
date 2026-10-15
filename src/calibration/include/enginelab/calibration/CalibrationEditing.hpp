#pragma once

#include <enginelab/calibration/CalibrationTypes.hpp>

#include <cstddef>
#include <vector>

namespace enginelab::calibration {

/** What a tuner does to a block of selected cells at once. */
enum class CellOperation {
    /** Adds `amount` to every selected cell. */
    add,
    /** Scales every selected cell by (1 + amount / 100). */
    scalePercent,
    /** Sets every selected cell to `amount`. */
    set,
    /** Each selected cell becomes the mean of itself and its neighbours along
     *  both axes (selected or not): one pass of a 3x3 cross filter. */
    smooth,
    /** Fills the selection's rectangle by bilinear interpolation between its
     *  four corner cells (a curve: linearly between its two ends). */
    interpolate
};

/**
 * Applies `operation` to the cells `selected` (row-major indices: y * columns
 * + x; a scalar has the one cell 0). Results are clamped to the entry's hard
 * limits; indices outside the entry are ignored. Returns whether any value
 * changed.
 */
bool applyCellOperation(CalibrationEntry& entry, const std::vector<std::size_t>& selected,
                        CellOperation operation, double amount = 0.0);

/** The step of a "+" or "-" click: ten units of the entry's last displayed
 *  digit (0.1 for an entry shown with two decimals, 10 for whole numbers). */
[[nodiscard]] double cellStep(const CalibrationMetadata& metadata) noexcept;

/** The cells of the rectangle spanned by two cells of a grid `columns` wide. */
[[nodiscard]] std::vector<std::size_t> cellRectangle(std::size_t anchor, std::size_t corner,
                                                     std::size_t columns);

} // namespace enginelab::calibration
