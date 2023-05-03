#define CATCH_MOTIONGENERATION_TEST
#include "../include/MotionGeneration.hpp"

#include <iostream>

#include "../include/math.hpp"
#include "catch2/catch.hpp"

TEST_CASE("Test computing next foot step", "[MotionGeneration]") {
    motion::MotionGenerationOptions<double> options;
    options.step_period = 1.0;
    options.step_height = 0.1;
    options.step_width  = 0.1;
    options.step_limits = Eigen::Vector3d(1.0, 1.0, 1.0);
    Eigen::Matrix<double, 3, 1> walk_command;
    motion::MotionGeneration<double> walk_engine;
    walk_engine.configure(options);


    SECTION("Forward walk command") {
        walk_command << 1.0, 0.0, 0.0;
        Eigen::Transform<double, 3, Eigen::Isometry> expected_foot_placement;
        expected_foot_placement.setIdentity();
        expected_foot_placement.translation().x() = 1.0;
        expected_foot_placement.translation().y() = -0.1;
        walk_engine.compute_walking_footstep(walk_command);
        auto footstep_end_pose = walk_engine.get_end_swingfoot_pose();
        REQUIRE(footstep_end_pose.isApprox(expected_foot_placement));
    }
}

TEST_CASE("Test generating swing foot trajectory", "[MotionGeneration]") {
    motion::MotionGenerationOptions<double> options;
    options.step_period = 1.0;
    options.step_height = 0.1;
    options.step_width  = 0.1;
    options.step_limits = Eigen::Vector3d(1.0, 1.0, 1.0);
    Eigen::Matrix<double, 3, 1> walk_command;
    motion::MotionGeneration<double> walk_engine;
    walk_engine.configure(options);

    SECTION("Forward walk command") {
        walk_command << 1.0, 0.0, 0.0;
        walk_engine.compute_walking_footstep(walk_command);
        walk_engine.generate_swingfoot_trajectory();

        // Test trajectory at t = 0 (start)
        Eigen::Matrix<double, 3, 1> start_position = walk_engine.get_swingfoot_trajectory().position(0.0);
        Eigen::Matrix<double, 3, 1> expected_start_position(0.0, -options.step_width, 0);
        REQUIRE(expected_start_position.isApprox(start_position, 1e-6));

        // Test trajectory at t = step_period / 2 (mid)
        Eigen::Matrix<double, 3, 1> mid_position =
            walk_engine.get_swingfoot_trajectory().position(options.step_period / 2.0);
        Eigen::Matrix<double, 3, 1> expected_mid_position(0.5, -options.step_width, 0.1);
        REQUIRE(expected_mid_position.isApprox(mid_position, 1e-6));

        // Test trajectory at t = step_period (end)
        Eigen::Matrix<double, 3, 1> end_position = walk_engine.get_swingfoot_trajectory().position(options.step_period);
        Eigen::Matrix<double, 3, 1> expected_end_position(1.0, -options.step_width, 0.0);
        REQUIRE(expected_end_position.isApprox(end_position, 1e-6));
    }
}

TEST_CASE("Test generating torso trajectory", "[MotionGeneration]") {
    motion::MotionGenerationOptions<double> options;
    options.step_period  = 1.0;
    options.step_height  = 0.1;
    options.step_width   = 0.1;
    options.step_limits  = Eigen::Vector3d(1.0, 1.0, 1.0);
    options.torso_height = 0.5;
    options.torso_pitch  = 0.0;
    Eigen::Matrix<double, 3, 1> walk_command;
    motion::MotionGeneration<double> walk_engine;
    walk_engine.configure(options);

    SECTION("Forward walk command") {
        walk_command << 1.0, 0.0, 0.0;
        walk_engine.compute_walking_footstep(walk_command);
        walk_engine.generate_torso_trajectory();

        // Test trajectory at t = 0 (start)
        Eigen::Matrix<double, 3, 1> start_position = walk_engine.get_torso_trajectory().position(0.0);
        Eigen::Matrix<double, 3, 1> expected_start_position(0.0, -options.step_width / 2, options.torso_height);
        REQUIRE(expected_start_position.isApprox(start_position, 1e-6));

        // Test trajectory at t = step_period / 2 (mid)
        Eigen::Matrix<double, 3, 1> mid_position =
            walk_engine.get_torso_trajectory().position(options.step_period / 2.0);
        Eigen::Matrix<double, 3, 1> expected_mid_position(0, 0, options.torso_height);
        REQUIRE(expected_mid_position.isApprox(mid_position, 1e-6));

        // Test trajectory at t = step_period (end)
        Eigen::Matrix<double, 3, 1> end_position = walk_engine.get_torso_trajectory().position(options.step_period);
        Eigen::Matrix<double, 3, 1> expected_end_position(0.5, -options.step_width / 2, options.torso_height);
        REQUIRE(expected_end_position.isApprox(end_position, 1e-6));
    }
}
