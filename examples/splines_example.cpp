#include <iostream>
#include <vector>

#include "../include/QuinticSpline.hpp"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

int main(int argc, char* argv[]) {

    // Define the start and end points of the spline
    Eigen::Vector3d start_waypoint(0, 0, 0);
    Eigen::Vector3d end_waypoint(1, 0, 0);

    // Define the start and end times of the spline
    double duration = 1;

    // Create the spline and initialize it with the given parameters
    motion::QuinticSpline<double> spline(start_waypoint, end_waypoint, duration);

    // Evaluate the spline at 100 points along its length
    int num_points = 100;
    std::vector<double> time(num_points);
    std::vector<double> position(num_points);
    std::vector<double> velocity(num_points);
    std::vector<double> acceleration(num_points);

    for (int i = 0; i < num_points; i++) {
        time[i]         = (i / static_cast<double>(num_points - 1)) * duration;
        position[i]     = spline.position(time[i]);
        velocity[i]     = spline.velocity(time[i]);
        acceleration[i] = spline.acceleration(time[i]);
    }

    // Plot the spline
    plt::plot(time, position);
    plt::show();

    plt::plot(time, velocity);
    plt::show();

    plt::plot(time, acceleration);
    plt::show();

    return EXIT_SUCCESS;
}
