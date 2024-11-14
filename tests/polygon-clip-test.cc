#include <catch.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "geometry/line.h"
#include "geometry/intersection.h"

namespace meshlib {

TEST_CASE("line and segment intersection", "[calc_2d_intersection_in_y_plane]")
{
    SECTION("intersection: mid-point") {
        const Line line = Line({0, 0, 0}, {0, 0, 1});
        const LineSegment segment = LineSegment({-0.5, 0, 0.5}, {0.5, 0, 0.5});

        const auto t = calc_2d_intersection_in_y_plane(line, segment);
        REQUIRE_THAT(t, Catch::Matchers::WithinRel(0.5, 0.01));
    }

    SECTION("intersection: start point") {
        const Line line = Line({0, 0, 0}, {0, 0, 1});
        const LineSegment segment = LineSegment({0, 0, 0.5}, {0.5, 0, 0.5});

        const auto t = calc_2d_intersection_in_y_plane(line, segment);
        REQUIRE_THAT(t, Catch::Matchers::WithinRel(0.0, 0.01));
    }

    SECTION("intersection: end point") {
        const Line line = Line({0, 0, 0}, {0, 0, 1});
        const LineSegment segment = LineSegment({-0.5, 0, 0.5}, {0, 0, 0.5});

        const auto t = calc_2d_intersection_in_y_plane(line, segment);
        REQUIRE_THAT(t, Catch::Matchers::WithinRel(1.0, 0.01));
    }

    SECTION("intersection: simple calculation") {
        const Line line = Line({-0.3, 0, 0.5}, {0.5, 0, -0.5});
        const LineSegment segment = LineSegment({-1, 0, -1}, {1, 0, 0});

        const auto t = calc_2d_intersection_in_y_plane(line, segment);
        REQUIRE_THAT(t, Catch::Matchers::WithinRel(0.678571, 0.01));
    }
}

} // namespace meshlib