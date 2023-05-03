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

        /// @brief Step period (in seconds). Time for one complete step.
        Scalar step_period = 0.0;

        /// @brief Step height.
        Scalar step_height = 0.0;

        /// @brief Lateral distance between feet. (how spread apart the feet should be)
        Scalar step_width = 0.0;

        /// @brief Torso height.
        Scalar torso_height = 0.0;

        /// @brief Torso pitch.
        Scalar torso_pitch = 0.0;
    };

    enum class WalkEngineState { WALK, STOP };

    template <typename Scalar>
    class MotionGeneration {

    public:
        /**
         * @brief Reset foot placement.
         */
        void reset() {
            // Assume left foot is planted foot to start
            left_foot_is_planted = true;

            // Initialize planted foot placement
            Hps_start.setIdentity();
            Hps_start.translate(Eigen::Matrix<Scalar, 3, 1>(0.0, -step_width, 0.0));

            // Initialize torso placement
            Hpt_start.setIdentity();
            Hpt_start.translate(Eigen::Matrix<Scalar, 3, 1>(0.0, -step_width / 2, torso_height));
            Hpt_start.rotate(Eigen::AngleAxis<Scalar>(torso_pitch, Eigen::Matrix<Scalar, 3, 1>::UnitY()));
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
            torso_height     = options.torso_height;
            torso_pitch      = options.torso_pitch;

            // Initialize foot/torso placement
            reset();
        }

        /**
         * @brief Compute transform from planted foot to swing foot and torso at midpoint and end of step for walking.
         * @param walk_command Walk command (dx, dy, dtheta).
         */
        void compute_walking_footstep(const Eigen::Matrix<Scalar, 3, 1>& walk_command) {

            // Compute midpoint foot placement.
            Eigen::Transform<Scalar, 3, Eigen::Isometry> Hps_start_copy = Hps_start;
            Hps_midpoint = Hps_start_copy.translate(Eigen::Matrix<Scalar, 3, 1>(walk_command.x() * half_step_period,
                                                                                walk_command.y() * half_step_period,
                                                                                step_height));
            Hps_midpoint = Hps_start_copy.rotate(
                Eigen::AngleAxis<Scalar>(walk_command.z() * half_step_period, Eigen::Matrix<Scalar, 3, 1>::UnitZ()));

            // Compute end foot placement.
            Hps_start_copy = Hps_start;
            Hps_end        = Hps_start_copy.translate(
                Eigen::Matrix<Scalar, 3, 1>(walk_command.x() * step_period, walk_command.y() * step_period, 0.0));

            Hps_end = Hps_start_copy.rotate(
                Eigen::AngleAxis<Scalar>(walk_command.z() * step_period, Eigen::Matrix<Scalar, 3, 1>::UnitZ()));

            // Compute midpoint torso placement.
            Hpt_midpoint.setIdentity();
            Hpt_midpoint.translation() = Eigen::Matrix<Scalar, 3, 1>(0.0, 0.0, torso_height);  // TODO: Add some offset
            Hpt_midpoint.rotate(Eigen::AngleAxis<Scalar>(torso_pitch, Eigen::Matrix<Scalar, 3, 1>::UnitY()));

            // Compute end torso placement (midpoint between start and end foot placement).
            Eigen::Transform<Scalar, 3, Eigen::Isometry> Hpt_start_copy = Hpt_start;
            Hpt_end = Hpt_start_copy.translate(Eigen::Matrix<Scalar, 3, 1>(walk_command.x() * step_period / 2.0,
                                                                           walk_command.y() * step_period / 2.0,
                                                                           0.0));
            // Orientate to have torso face direction of end foot placement.
            Hpt_end = Hpt_start_copy.rotate(
                Eigen::AngleAxis<Scalar>(walk_command.z() * step_period, Eigen::Matrix<Scalar, 3, 1>::UnitZ()));
        }

        /**
         * @brief Compute transform from planted foot to swing foot and torso at midpoint and end of step for stopping.
         */
        void compute_stopping_footstep() {
            // Compute end foot placement.
            Scalar foot_width_offset = left_foot_is_planted ? -step_width : step_width;
            Hps_end.translation()    = Eigen::Matrix<Scalar, 3, 1>(0.0, foot_width_offset, 0.0);
            Hps_end.rotation()       = Eigen::Matrix<Scalar, 3, 3>::Identity();

            // Compute midpoint foot placement.
            Hps_midpoint.translation() = Eigen::Matrix<Scalar, 3, 1>(Hps_end.translation().x() / 2.0,
                                                                     Hps_end.translation().y() / 2.0,
                                                                     step_height);
            Hps_midpoint.rotation()    = Eigen::Matrix<Scalar, 3, 3>::Identity();

            // Compute midpoint torso placement.
            Hpt_midpoint.setIdentity();
            Hpt_midpoint.translation() = Eigen::Matrix<Scalar, 3, 1>(0.0, 0.0, torso_height);
            Hpt_midpoint.rotation() =
                Eigen::AngleAxis<Scalar>(torso_pitch, Eigen::Matrix<Scalar, 3, 1>::UnitY()).toRotationMatrix();

            // Compute end torso placement (midpoint between start and end foot placement).
            Hpt_end.translation() = Eigen::Matrix<Scalar, 3, 1>(0.0, foot_width_offset / 2, torso_height);
            Hpt_end.rotation() =
                Eigen::AngleAxis<Scalar>(torso_pitch, Eigen::Matrix<Scalar, 3, 1>::UnitY()).toRotationMatrix();
        }

        /**
         * @brief Generate swing foot trajectory.
         * @param walk_command Walk command (dx, dy, dtheta).
         * @param Hps_end Next foot placement.
         * @return Trajectory of swing foot to follow to reach next foot placement.
         */
        void generate_swingfoot_trajectory() {
            // Orientation keypoints
            Eigen::Matrix<Scalar, 3, 1> rpy_Rps_start    = Hps_start.rotation().eulerAngles(0, 1, 2);
            Eigen::Matrix<Scalar, 3, 1> rpy_Rps_midpoint = Hps_midpoint.rotation().eulerAngles(0, 1, 2);
            Eigen::Matrix<Scalar, 3, 1> rpy_Rps_end      = Hps_end.rotation().eulerAngles(0, 1, 2);

            // Clear current trajectory
            swingfoot_trajectory.clear();

            // X position trajectory
            swingfoot_trajectory.add_waypoint(X, 0, Hps_start.translation().x());
            swingfoot_trajectory.add_waypoint(X, half_step_period, Hps_midpoint.translation().x());
            swingfoot_trajectory.add_waypoint(X, step_period, Hps_end.translation().x());

            // Y position trajectory
            swingfoot_trajectory.add_waypoint(Y, 0, Hps_start.translation().y());
            swingfoot_trajectory.add_waypoint(Y, half_step_period, Hps_midpoint.translation().y());
            swingfoot_trajectory.add_waypoint(Y, step_period, Hps_end.translation().y());

            // Z position trajectory
            swingfoot_trajectory.add_waypoint(Z, 0, Hps_start.translation().z());
            swingfoot_trajectory.add_waypoint(Z, half_step_period, Hps_midpoint.translation().z());
            swingfoot_trajectory.add_waypoint(Z, step_period, Hps_end.translation().z());

            // Roll trajectory
            swingfoot_trajectory.add_waypoint(ROLL, 0, rpy_Rps_start.x());
            swingfoot_trajectory.add_waypoint(ROLL, half_step_period, rpy_Rps_midpoint.x());
            swingfoot_trajectory.add_waypoint(ROLL, step_period, rpy_Rps_end.x());

            // Pitch trajectory
            swingfoot_trajectory.add_waypoint(PITCH, 0, rpy_Rps_start.y());
            swingfoot_trajectory.add_waypoint(PITCH, half_step_period, rpy_Rps_midpoint.y());
            swingfoot_trajectory.add_waypoint(PITCH, step_period, rpy_Rps_end.y());

            // Yaw trajectory
            swingfoot_trajectory.add_waypoint(YAW, 0, rpy_Rps_start.z());
            swingfoot_trajectory.add_waypoint(YAW, half_step_period, rpy_Rps_midpoint.z());
            swingfoot_trajectory.add_waypoint(YAW, step_period, rpy_Rps_end.z());
        }

        /**
         * @brief Generate torso trajectory.
         * @param walk_command Walk command (dx, dy, dtheta).
         * @param Hps_end Next foot placement.
         * @return Trajectory of torso to follow to reach next torso placement.
         */
        void generate_torso_trajectory() {
            // Orientation keypoints
            Eigen::Matrix<Scalar, 3, 1> rpy_Rpt_start    = Hpt_start.rotation().eulerAngles(0, 1, 2);
            Eigen::Matrix<Scalar, 3, 1> rpy_Rpt_midpoint = Hpt_midpoint.rotation().eulerAngles(0, 1, 2);
            Eigen::Matrix<Scalar, 3, 1> rpy_Rpt_end      = Hpt_end.rotation().eulerAngles(0, 1, 2);

            // Clear current trajectory
            torso_trajectory.clear();

            // X position trajectory
            torso_trajectory.add_waypoint(X, 0, Hpt_start.translation().x());
            torso_trajectory.add_waypoint(X, half_step_period, Hpt_midpoint.translation().x());
            torso_trajectory.add_waypoint(X, step_period, Hpt_end.translation().x());

            // Y position trajectory
            torso_trajectory.add_waypoint(Y, 0, Hpt_start.translation().y());
            torso_trajectory.add_waypoint(Y, half_step_period, Hpt_midpoint.translation().y());
            torso_trajectory.add_waypoint(Y, step_period, Hpt_end.translation().y());

            // Z position trajectory
            torso_trajectory.add_waypoint(Z, 0, Hpt_start.translation().z());
            torso_trajectory.add_waypoint(Z, half_step_period, Hpt_midpoint.translation().z());
            torso_trajectory.add_waypoint(Z, step_period, Hpt_end.translation().z());

            // Roll trajectory
            torso_trajectory.add_waypoint(ROLL, 0, rpy_Rpt_start.x());
            torso_trajectory.add_waypoint(ROLL, half_step_period, rpy_Rpt_midpoint.x());
            torso_trajectory.add_waypoint(ROLL, step_period, rpy_Rpt_end.x());

            // Pitch trajectory
            torso_trajectory.add_waypoint(PITCH, 0, rpy_Rpt_start.y());
            torso_trajectory.add_waypoint(PITCH, half_step_period, rpy_Rpt_midpoint.y());
            torso_trajectory.add_waypoint(PITCH, step_period, rpy_Rpt_end.y());

            // Yaw trajectory
            torso_trajectory.add_waypoint(YAW, 0, rpy_Rpt_start.z());
            torso_trajectory.add_waypoint(YAW, half_step_period, rpy_Rpt_midpoint.z());
            torso_trajectory.add_waypoint(YAW, step_period, rpy_Rpt_end.z());
        }

        /**
         * @brief Generate walking trajectory.
         * @param walk_command Walk command (dx, dy, dtheta).
         * @param Hps_end Next foot placement.
         * @return Trajectory of torso to follow to reach next torso placement.
         */
        void generate_walking_trajectory(const Eigen::Transform<Scalar, 3, Eigen::Isometry>& Hps_end) {
            // Generate swing foot trajectory
            generate_swingfoot_trajectory(Hps_end);
            // Generate torso trajectory
            generate_torso_trajectory(Hps_end);
        }

        /**
         * @brief Switch planted foot.
         * @param walk_command Walk command (dx, dy, dtheta).
         */
        void switch_planted_foot() {
            // Transform torso into end torso frame at end foot placement
            Hpt_start = Hps_end.inverse() * Hpt_end;

            // Transform planted foot into swing foot frame at next foot placement
            Hps_start = Hps_end.inverse() * Hps_start;

            // Switch planted foot indicator
            left_foot_is_planted = !left_foot_is_planted;

            // Reset time
            t = 0;
        }

        /// @brief Get the current swing foot trajectory.
        Trajectory<Scalar> get_swingfoot_trajectory() const {
            return swingfoot_trajectory;
        }

        /// @brief Get the swing foot pose at the current time.
        Eigen::Transform<Scalar, 3, Eigen::Isometry> get_swingfoot_pose() const {
            return swingfoot_trajectory.pose(t);
        }

        /// @brief Get the current torso trajectory.
        Trajectory<Scalar> get_torso_trajectory() const {
            return torso_trajectory;
        }

        /// @brief Get the torso pose at the current time.
        Eigen::Transform<Scalar, 3, Eigen::Isometry> get_torso_pose() const {
            return torso_trajectory.pose(t);
        }

        /// @brief Get the end foot placement.
        Eigen::Transform<Scalar, 3, Eigen::Isometry> get_end_swingfoot_pose() const {
            return Hps_end;
        }

        /// @brief Get the end torso placement.
        Eigen::Transform<Scalar, 3, Eigen::Isometry> get_end_torso_pose() const {
            return Hpt_end;
        }

        /// @brief Increment the current time by dt.
        void update_time(const float& dt) {
            // Check for negative time step
            if (dt <= 0.0f) {
                // std::cout << "dt <= 0.0f" << std::endl;
                return;
            }
            // Check for too long dt
            if (dt > step_period) {
                // std::cout << "dt > params.step_period" << std::endl;
                return;
            }

            // Update the phase
            t += dt;

            // Clamp time to step period
            if (t >= step_period) {
                t = step_period;
            }
        }

        void update_state(const Scalar& dt, const Eigen::Matrix<Scalar, 3, 1>& walk_command) {

            const bool walk_command_zero = walk_command.isZero();

            if (walk_command_zero) {
                engine_state = WalkEngineState::STOP;
            }
            else {
                engine_state = WalkEngineState::WALK;
            }

            // small state machine
            switch (engine_state) {
                case WalkEngineState::WALK:
                    // Update the time
                    update_time(dt);
                    // Compute the desired end foot placement of current step
                    compute_walking_footstep(walk_command);
                    // Update the swing foot and torso trajectories such that they will reach the desired step position
                    generate_walking_trajectory();

                    // If we are at the end of the step, switch the planted foot to the swing foot
                    if (t >= step_period) {
                        switch_planted_foot();
                    }

                    break;
                case WalkEngineState::STOP:
                    // Update the time
                    update_time(dt);
                    compute_stopping_footstep();
                    generate_walking_trajectory();
                    // Once the time has reached step_period, the standing state is achieved
                    break;
                default:
                    // std::cout << "Unknown state" << std::endl;
                    break;
            }
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

        /// @brief Torso height.
        Scalar torso_height = 0.0;

        /// @brief Torso pitch.
        Scalar torso_pitch = 0.0;

        // ******************************** State ********************************

        /// @brief Current engine state.
        WalkEngineState engine_state{};

        /// @brief Transform from planted {p} foot to swing {s} foot current placement at start of step.
        Eigen::Transform<Scalar, 3, Eigen::Isometry> Hps_start =
            Eigen::Transform<Scalar, 3, Eigen::Isometry>::Identity();

        /// @brief Transform from planted {p} foot to swing {s} foot current placement at midpoint of step.
        Eigen::Transform<Scalar, 3, Eigen::Isometry> Hps_midpoint =
            Eigen::Transform<Scalar, 3, Eigen::Isometry>::Identity();

        /// @brief Transform from planted {p} foot to swing {s} foot current placement at end of step.
        Eigen::Transform<Scalar, 3, Eigen::Isometry> Hps_end = Eigen::Transform<Scalar, 3, Eigen::Isometry>::Identity();

        /// @brief Transform from planted {p} foot to the torso {t} at start of step.
        Eigen::Transform<Scalar, 3, Eigen::Isometry> Hpt_start =
            Eigen::Transform<Scalar, 3, Eigen::Isometry>::Identity();

        /// @brief Transform from planted {p} foot to the torso {t} at midpoint of step.
        Eigen::Transform<Scalar, 3, Eigen::Isometry> Hpt_midpoint =
            Eigen::Transform<Scalar, 3, Eigen::Isometry>::Identity();

        /// @brief Transform from planted {p} foot to the torso {t} at end of step.
        Eigen::Transform<Scalar, 3, Eigen::Isometry> Hpt_end = Eigen::Transform<Scalar, 3, Eigen::Isometry>::Identity();

        /// @brief Whether the left foot is planted.
        bool left_foot_is_planted = true;

        /// @brief Current time in the step cycle [0, step_period]
        Scalar t = 0.0;

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