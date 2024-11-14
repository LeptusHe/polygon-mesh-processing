#include <catch.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "geometry/line.h"
#include "geometry/intersection.h"

namespace meshlib {

TEST_CASE("line and segment intersection", "[calc_2d_line_and_segment_intersection]")
{
    SECTION("intersection: mid-point") {
        const auto line = Line2D({0, 0}, {0, 1});
        const auto segment = LineSegment2D({-0.5, 0.5}, {0.5, 0.5});

        const auto t = calc_2d_line_and_segment_intersection(line, segment);
        REQUIRE_THAT(t, Catch::Matchers::WithinRel(0.5, 0.01));
    }

    SECTION("intersection: start point") {
        const auto line = Line2D({0, 0}, {0, 1});
        const auto segment = LineSegment2D({0, 0.5}, {0.5, 0.5});

        const auto t = calc_2d_line_and_segment_intersection(line, segment);
        REQUIRE_THAT(t, Catch::Matchers::WithinRel(0.0, 0.01));
    }

    SECTION("intersection: end point") {
        const auto line = Line2D({0, 0}, {0, 1});
        const auto segment = LineSegment2D({-0.5, 0.5}, {0, 0.5});

        const auto t = calc_2d_line_and_segment_intersection(line, segment);
        REQUIRE_THAT(t, Catch::Matchers::WithinRel(1.0, 0.01));
    }

    SECTION("intersection: simple calculation") {
        const auto line = Line2D({-0.3, 0.5}, {0.5, -0.5});
        const auto segment = LineSegment2D({-1, -1}, {1, 0});

        const auto t = calc_2d_line_and_segment_intersection(line, segment);
        REQUIRE_THAT(t, Catch::Matchers::WithinRel(0.678571, 0.01));
    }
}

} // namespace meshlib