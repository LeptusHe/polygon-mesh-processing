#pragma once

#include "line.h"


namespace meshlib {

float calc_2d_intersection_in_y_plane(const Line& line, const LineSegment& segment);

} // namespace meshlib