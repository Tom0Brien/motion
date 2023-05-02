#define CATCH_QUINTICSPLINE_TEST
#include "../include/QuinticSpline.hpp"

#include <iostream>

#include "catch2/catch.hpp"

#define CATCH_QUINTICSPLINE_TEST
#include <iostream>

#include "catch2/catch.hpp"

TEST_CASE("Test QuinticSpline simple", "[QuinticSpline]") {
    using Scalar = double;

    Eigen::Matrix<Scalar, 3, 1> start_waypoint;
    start_waypoint << 0, 0, 0;
    Eigen::Matrix<Scalar, 3, 1> end_waypoint;
    end_waypoint << 1, 0, 0;
    Scalar duration = 1;

    motion::QuinticSpline<Scalar> spline(start_waypoint, end_waypoint, duration);

    // Test position
    REQUIRE(spline.position(0) == Approx(0));
    REQUIRE(spline.position(duration) == Approx(1));

    // Test velocity
    REQUIRE(spline.velocity(0) == Approx(0));
    REQUIRE(spline.velocity(duration) == Approx(0));

    // Test acceleration
    REQUIRE(spline.acceleration(0) == Approx(0));
    REQUIRE(spline.acceleration(duration) == Approx(0));
}

TEST_CASE("Test QuinticSpline with nonzero velocities", "[QuinticSpline]") {
    using Scalar = double;

    Eigen::Matrix<Scalar, 3, 1> start_waypoint;
    start_waypoint << 0, 1, 2;
    Eigen::Matrix<Scalar, 3, 1> end_waypoint;
    end_waypoint << 1, -1, 1;
    Scalar duration = 2;

    motion::QuinticSpline<Scalar> spline(start_waypoint, end_waypoint, duration);

    // Test position
    REQUIRE(spline.position(0) == Approx(0));
    REQUIRE(spline.position(duration) == Approx(1));

    // Test velocity
    REQUIRE(spline.velocity(0) == Approx(1));
    REQUIRE(spline.velocity(duration) == Approx(-1));

    // Test acceleration
    REQUIRE(spline.acceleration(0) == Approx(2));
    REQUIRE(spline.acceleration(duration) == Approx(1));
}

TEST_CASE("Test invalid duration", "[QuinticSpline]") {
    using Scalar = double;

    Eigen::Matrix<Scalar, 3, 1> start_waypoint;
    start_waypoint << 0, 1, 2;
    Eigen::Matrix<Scalar, 3, 1> end_waypoint;
    end_waypoint << 1, -1, 1;
    Scalar duration = -1;

    REQUIRE_THROWS(motion::QuinticSpline<Scalar>(start_waypoint, end_waypoint, duration));
}
