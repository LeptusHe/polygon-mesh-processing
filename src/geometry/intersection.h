#pragma once

#include "line.h"


namespace meshlib {

float calc_2d_line_and_segment_intersection(const Line2D& line, const LineSegment2D& segment);

} // namespace meshlib