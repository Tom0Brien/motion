#ifndef MOTION_MOTIONGENERATION_HPP
#define MOTION_MOTIONGENERATION_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "Math.hpp"
#include "Trajectory.hpp"

/** \file MotionGeneration.hpp
 * @brief Contains various functions for motion generation.
 */
namespace motion {

    /// @brief Motion generation options.
    template <typename Scalar>
    struct MotionGenerationOptions {
        /// @brief Maximum step limits in x, y, and theta.
        Eigen::Matrix<Scalar, 3, 1> step_limits = Eigen::Matrix<Scalar, 3, 1>::Zero();

        /// @brief Step height.
        Scalar step_height = 0.0;

        /// @brief Step period (in seconds). Time for one complete step.
        Scalar step_period = 0.0;

        /// @brief Lateral distance between feet. (how spread apart the feet should be)
        Scalar step_width = 0.0;
    };

    template <typename Scalar>
    class MotionGeneration {

    public:
        /**
         * @brief Reset foot placement.
         */
        void reset() {
            Hps.setIdentity();
            Hps.translate(Eigen::Matrix<Scalar, 3, 1>(0.0, -step_width, 0.0));
            left_foot_is_planted = true;
        }

        /**
         * @brief Configure motion generation options.
         * @param options Motion generation options.
         */
        void configure(const MotionGenerationOptions<Scalar>& options) {
            step_limits      = options.step_limits;
            step_height      = options.step_height;
            step_period      = options.step_period;
            half_step_period = step_period / 2.0;
            step_width       = options.step_width;

            // Initialize foot placement
            reset();
        }

        /**
         * @brief Compute next foot placement for a given walk command.
         * @param walk_command Walk command (dx, dy, dtheta).
         * @return Transform from planted foot to next foot placement of swing foot.
         */
        Eigen::Transform<Scalar, 3, Eigen::Isometry> compute_next_footstep(
            const Eigen::Matrix<Scalar, 3, 1>& walk_command) {
            // Compute next foot placement.
            Eigen::Transform<Scalar, 3, Eigen::Isometry> Hps_next = Hps;
            Hps_next.translate(
                Eigen::Matrix<Scalar, 3, 1>(walk_command.x() * step_period, walk_command.y() * step_period, 0.0));
            Hps_next.rotate(
                Eigen::AngleAxis<Scalar>(walk_command.z() * step_period, Eigen::Matrix<Scalar, 3, 1>::UnitZ()));

            // // Update foot placement.
            // Hps = Hps_next.inverse();

            // // Switch feet.
            // left_foot_is_planted = !left_foot_is_planted;

            return Hps_next;
        }

        /**
         * @brief Generate swing foot trajectory.
         * @param walk_command Walk command (dx, dy, dtheta).
         * @return Trajectory of swing foot to follow to reach next foot placement.
         */
        void generate_swingfoot_trajectory(const Eigen::Matrix<Scalar, 3, 1>& walk_command) {
            // Compute the next foot placement
            Eigen::Transform<Scalar, 3, Eigen::Isometry> Hps_next = compute_next_footstep(walk_command);
            Eigen::Matrix<Scalar, 3, 1> rSPp_mid                  = (Hps.translation() + Hps_next.translation()) / 2.0;
            Eigen::Matrix<Scalar, 3, 1> rpy_current               = Hps.rotation().eulerAngles(0, 1, 2);
            Eigen::Matrix<Scalar, 3, 1> rpy_next                  = Hps_next.rotation().eulerAngles(0, 1, 2);
            Eigen::Matrix<Scalar, 3, 1> rpy_mid_point             = (rpy_current + rpy_next) / 2.0;

            // Reset trajectory
            swingfoot_trajectory.clear();

            // X position trajectory
            swingfoot_trajectory.add_waypoint(X, 0, Eigen::Matrix<Scalar, 3, 1>(Hps.translation().x(), 0.0, 0.0));
            swingfoot_trajectory.add_waypoint(X, half_step_period, Eigen::Matrix<Scalar, 3, 1>(rSPp_mid.x(), 0.0, 0.0));
            swingfoot_trajectory.add_waypoint(X,
                                              step_period,
                                              Eigen::Matrix<Scalar, 3, 1>(Hps_next.translation().x(), 0.0, 0.0));

            // Y position trajectory
            swingfoot_trajectory.add_waypoint(Y, 0, Eigen::Matrix<Scalar, 3, 1>(Hps.translation().y(), 0.0, 0.0));
            swingfoot_trajectory.add_waypoint(Y, half_step_period, Eigen::Matrix<Scalar, 3, 1>(rSPp_mid.y(), 0.0, 0.0));
            swingfoot_trajectory.add_waypoint(Y,
                                              step_period,
                                              Eigen::Matrix<Scalar, 3, 1>(Hps_next.translation().y(), 0.0, 0.0));

            // Z position trajectory
            swingfoot_trajectory.add_waypoint(Z, 0, Eigen::Matrix<Scalar, 3, 1>(Hps.translation().z(), 0.0, 0.0));
            swingfoot_trajectory.add_waypoint(Z,
                                              half_step_period,
                                              Eigen::Matrix<Scalar, 3, 1>(rSPp_mid.z() + step_height, 0.0, 0.0));
            swingfoot_trajectory.add_waypoint(Z,
                                              step_period,
                                              Eigen::Matrix<Scalar, 3, 1>(Hps_next.translation().z(), 0.0, 0.0));

            // Roll trajectory
            swingfoot_trajectory.add_waypoint(ROLL, 0, Eigen::Matrix<Scalar, 3, 1>(rpy_current.x(), 0.0, 0.0));
            swingfoot_trajectory.add_waypoint(ROLL,
                                              half_step_period,
                                              Eigen::Matrix<Scalar, 3, 1>(rpy_mid_point.x(), 0.0, 0.0));
            swingfoot_trajectory.add_waypoint(ROLL, step_period, Eigen::Matrix<Scalar, 3, 1>(rpy_next.x(), 0.0, 0.0));

            // Pitch trajectory
            swingfoot_trajectory.add_waypoint(PITCH, 0, Eigen::Matrix<Scalar, 3, 1>(rpy_current.y(), 0.0, 0.0));
            swingfoot_trajectory.add_waypoint(PITCH,
                                              half_step_period,
                                              Eigen::Matrix<Scalar, 3, 1>(rpy_mid_point.y(), 0.0, 0.0));
            swingfoot_trajectory.add_waypoint(PITCH, step_period, Eigen::Matrix<Scalar, 3, 1>(rpy_next.y(), 0.0, 0.0));

            // Yaw trajectory
            swingfoot_trajectory.add_waypoint(YAW, 0, Eigen::Matrix<Scalar, 3, 1>(rpy_current.z(), 0.0, 0.0));
            swingfoot_trajectory.add_waypoint(YAW,
                                              half_step_period,
                                              Eigen::Matrix<Scalar, 3, 1>(rpy_mid_point.z(), 0.0, 0.0));
            swingfoot_trajectory.add_waypoint(YAW, step_period, Eigen::Matrix<Scalar, 3, 1>(rpy_next.z(), 0.0, 0.0));
        }

        /// @brief Get the current swing foot trajectory.
        Trajectory<Scalar> get_swingfoot_trajectory() const {
            return swingfoot_trajectory;
        }

    private:
        // ******************************** Options ********************************

        /// @brief Maximum step limits in x, y, and theta.
        Eigen::Matrix<Scalar, 3, 1> step_limits = Eigen::Matrix<Scalar, 3, 1>::Zero();

        /// @brief Step height.
        Scalar step_height = 0.0;

        /// @brief Step period (in seconds). Time for one complete step.
        Scalar step_period = 0.0;

        /// @brief Half of step period (in seconds).
        Scalar half_step_period = 0.0;

        /// @brief Lateral distance between feet. (how spread apart the feet should be)
        Scalar step_width = 0.0;

        // ******************************** State ********************************

        /// @brief  Transform from planted {p} foot to swing {s} foot current placement.
        Eigen::Transform<Scalar, 3, Eigen::Isometry> Hps = Eigen::Transform<Scalar, 3, Eigen::Isometry>::Identity();

        /// @brief Whether the left foot is planted.
        bool left_foot_is_planted = true;

        // ******************************** Trajectories ********************************

        // 6D piecewise polynomial trajectory for swing foot.
        Trajectory<Scalar> swingfoot_trajectory;

        // 6D piecewise polynomial trajectory for torso.
        Trajectory<Scalar> torso_trajectory;

        // 6D piecewise polynomial trajectory for center of mass.
        Trajectory<Scalar> com_trajectory;
    };

}  // namespace motion

#endif