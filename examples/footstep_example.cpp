#include <iostream>
#include <vector>

#include "../include/MotionGeneration.hpp"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

int main(int argc, char* argv[]) {
    using Scalar = double;

    // Configure motion generation options.
    motion::MotionGenerationOptions<Scalar> options;
    options.step_limits << 0.5, 0.5, 0.1;
    options.step_height = 0.02;
    options.step_period = 0.5;
    options.step_width  = 0.2;

    // Create and configure the MotionGeneration object.
    motion::MotionGeneration<Scalar> motion_gen;
    motion_gen.configure(options);

    // Create walk command.
    Eigen::Matrix<Scalar, 3, 1> walk_command(0, 0.5, 0.0);

    // Number of steps.
    int num_steps = 10;

    // Create vectors for foot positions.
    std::vector<double> x_positions;
    std::vector<double> y_positions;
    std::vector<double> angles;

    // Initialize world transform.
    Eigen::Transform<Scalar, 3, Eigen::Isometry> Hw = Eigen::Transform<Scalar, 3, Eigen::Isometry>::Identity();

    // Include the first footstep.
    x_positions.push_back(Hw.translation().x());
    y_positions.push_back(Hw.translation().y());
    angles.push_back(Eigen::AngleAxis<Scalar>(Hw.rotation()).angle());

    // Compute steps and store foot positions in world coordinates.
    for (int i = 0; i < num_steps; ++i) {
        Eigen::Transform<Scalar, 3, Eigen::Isometry> Hps_next = motion_gen.compute_next_footstep(walk_command);
        Hw                                                    = Hw * Hps_next;
        x_positions.push_back(Hw.translation().x());
        y_positions.push_back(Hw.translation().y());
        angles.push_back(Eigen::AngleAxis<Scalar>(Hw.rotation()).angle());
        motion_gen.switch_planted_foot();
    }

    // Set up the plot.
    plt::figure_size(1200, 800);
    plt::title("Foot positions");

    // Plot foot positions as rotated boxes.
    for (size_t i = 0; i < x_positions.size(); ++i) {
        double x      = x_positions[i];
        double y      = y_positions[i];
        double angle  = angles[i];
        double width  = 0.2;
        double height = 0.1;

        Eigen::Matrix<Scalar, 2, 2> rotation_matrix;
        rotation_matrix << cos(angle), -sin(angle), sin(angle), cos(angle);

        Eigen::Matrix<Scalar, 2, 1> bottom_left(-width / 2, -height / 2);
        Eigen::Matrix<Scalar, 2, 1> bottom_right(width / 2, -height / 2);
        Eigen::Matrix<Scalar, 2, 1> top_right(width / 2, height / 2);
        Eigen::Matrix<Scalar, 2, 1> top_left(-width / 2, height / 2);

        bottom_left  = rotation_matrix * bottom_left + Eigen::Matrix<Scalar, 2, 1>(x, y);
        bottom_right = rotation_matrix * bottom_right + Eigen::Matrix<Scalar, 2, 1>(x, y);
        top_right    = rotation_matrix * top_right + Eigen::Matrix<Scalar, 2, 1>(x, y);
        top_left     = rotation_matrix * top_left + Eigen::Matrix<Scalar, 2, 1>(x, y);
        plt::plot({bottom_left.x(), bottom_right.x(), top_right.x(), top_left.x(), bottom_left.x()},
                  {bottom_left.y(), bottom_right.y(), top_right.y(), top_left.y(), bottom_left.y()},
                  "b-");
    }

    // Show the plot.
    plt::grid(true);
    plt::xlabel("X-axis");
    plt::ylabel("Y-axis");
    plt::show();

    return EXIT_SUCCESS;
}
