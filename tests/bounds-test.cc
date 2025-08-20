#include <catch.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "geometry/line.h"
#include "geometry/bounds.h"

namespace meshlib {

TEST_CASE("bounds", "[min and max funcs]")
{
    Bounds<glm::vec2> bounds = Bounds<glm::vec2>::FromMinAndMax({-1, 0}, {1, 4});

    SECTION("min of bounds") {
        const auto min = bounds.min();

        REQUIRE_THAT(min.x, Catch::Matchers::WithinRel(-1, 0.01));
        REQUIRE_THAT(min.y, Catch::Matchers::WithinRel(0, 0.01));
    }

    SECTION("max of bounds") {
        const auto max = bounds.max();

        REQUIRE_THAT(max.x, Catch::Matchers::WithinRel(1, 0.01));
        REQUIRE_THAT(max.y, Catch::Matchers::WithinRel(4, 0.01));
    }

    SECTION("size of bounds") {
        const auto size = bounds.size();

        REQUIRE_THAT(size.x, Catch::Matchers::WithinRel(2.0, 0.01));
        REQUIRE_THAT(size.y, Catch::Matchers::WithinRel(4.0, 0.01));
    }
}

} // namespace meshlib