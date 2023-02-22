#include <vector>
#include <Eigen/Dense>
#include <cmath>
#include "safety_shield/motion.h"

#ifndef GYM_TRAJ_H
#define GYM_TRAJ_H

namespace safety_shield
{

    class GymTrajPlanner
    {
    private:
        /**
         * @brief How many steps to compute the trajectory for
         */
        int steps_ahead_;
        /**
         * @brief How many safe actions to sample
         */
        int n_tries_;
        /**
         * @brief How many actions to try for safe sampling
         */
        int max_tries_;

        // point data
        /**
         * @brief point robot mass
         */
        const double point_mass_ = 0.00519;

        // car data
        /**
         * @brief car robot mass
         */
        const double car_mass_ = 0.03045;
        /**
         * @brief Wheel radius
         */
        const double r_ = 0.05;
        /**
         * @brief Half-distance between center of wheels
         */
        const double L_ = 0.125;
        /**
         * @brief Distance between center of robot and center of wheels
         */
        const double D_ = 0.1;

        const double rho_ = D_ / L_;

        /**
         * @brief Mujoco timestep
         */
        double timestep_;

        // Static obstacle data
        /**
         * @brief List of static obstacle position
         */
        std::vector<Eigen::Vector2d> obstacles_;
        /**
         * @brief List of static obstacle radiuses
         */
        std::vector<float> obstacles_radius_;

        /**
         * @brief Action to take (either agent, slowdown, or resampled action)
         */
        std::vector<double> selected_action_;

    public:
        /**
         * @brief Construct a new GymTrajPlanner object
         *
         * @param steps_ahead the maximum number of steps to compute
         * @param timestep the simulation time step
         * @param obstacles statci obstacle positions
         * @param obstacles_radius static obstacle radiuses
         * @param n_tries number of safe action to resample in case a collision is detected
         */
        GymTrajPlanner(int steps_ahead, double timestep, std::vector<Eigen::Vector2d> obstacles, std::vector<float> obstacles_radius, int n_tries, int max_tries) : steps_ahead_(steps_ahead), timestep_(timestep), obstacles_(obstacles), obstacles_radius_(obstacles_radius), n_tries_(n_tries), max_tries_(max_tries)
        {
            selected_action_ = {0, 0};
        }

        /**
         * @brief Plan a trajectory for the point robot.
         * Can resample actions if a collision is detected during trajectory computation.
         *
         * @param action chosen action by the agent
         * @param robot_vel 2D-Velocity of the robot
         * @param robot_rot Rotation matrix of the robot on the 2D plane
         * @param robot_com 2D-Position of the center of the robot
         *
         * @return a list of motions describing the trajectory
         */
        std::vector<Motion> planner_point(Eigen::Vector2d action, Eigen::Vector2d robot_vel, Eigen::Matrix2d robot_rot, Eigen::Vector2d robot_com, bool policy_step);
        /**
         * @brief Plan a trajectory for the point robot.
         *
         * @param action chosen action by the agent
         * @param robot_vel 2D-Velocity of the robot
         * @param robot_rot Rotation matrix of the robot on the 2D plane
         * @param robot_com 2D-Position of the center of the robot
         * @param planned_motions a list of motions describing the trajectory
         *
         * @return true if computed trajectory is collision-free, else false.
         */
        bool planner_point_loop(Eigen::Vector2d action, Eigen::Vector2d robot_vel, Eigen::Matrix2d robot_rot, Eigen::Vector2d robot_com, bool policy_step, std::vector<Motion> &planned_motions);

        /**
         * @brief Computes a slowdown action
         *
         * @param robot_vel 2D-Velocity of the robot
         * @param action_0 acceleration action
         * @param action_1 rotation action
         * @param robot_rot Rotation matrix of the robot on the 2D plane
         * 
         * @return the slowdown action following the trajectory set by (action_0, action_1)
         */
        Eigen::Vector2d point_slowdown(Eigen::Vector2d robot_vel, double action_0, double action_1, Eigen::Matrix2d robot_rot);
        Eigen::Vector2d point_slowdown_old(Eigen::Vector2d robot_vel, double action_0, double action_1, Eigen::Matrix2d robot_rot);

        /**
         * @brief Plan a trajectory for the car robot.
         * Can resample actions if a collision is detected during trajectory computation.
         *
         * @param action chosen action by the agent
         * @param robot_vel 2D-Velocity of the robot
         * @param robot_rot Rotation matrix of the robot on the 2D plane
         * @param robot_com 2D-Position of the center of the robot
         *
         * @return a list of motions describing the trajectory
         */
        std::vector<Motion> planner_car(Eigen::Vector2d action, Eigen::Vector2d robot_vel, Eigen::Matrix2d robot_rot, Eigen::Vector2d robot_com);
        /**
         * @brief Plan a trajectory for the car robot.
         *
         * @param action chosen action by the agent
         * @param robot_vel 2D-Velocity of the robot
         * @param robot_rot Rotation matrix of the robot on the 2D plane
         * @param robot_com 2D-Position of the center of the robot
         * @param planned_motions a list of motions describing the trajectory
         *
         * @return true if computed trajectory is collision-free, else false.
         */
        bool planner_car_loop(Eigen::Vector2d action, Eigen::Vector2d robot_vel, Eigen::Matrix2d robot_rot, Eigen::Vector2d robot_com, std::vector<Motion> &planned_motions);

        /**
         * @brief Computes a slowdown action
         *
         * @param robot_vel 2D-Velocity of the robot
         * @param action_0 left wheel rotation action
         * @param action_1 right wheel rotation action
         * @param robot_rot Rotation matrix of the robot on the 2D plane
         * 
         * @return the slowdown action following the trajectory set by (action_0, action_1)
         */
        Eigen::Vector2d car_slowdown(Eigen::Vector2d robot_vel, double action_0, double action_1, Eigen::Matrix2d robot_rot);

        /**
         * @brief Returns the action selected after trajectory computation
         * 
         * @return action
        */
        inline std::vector<double> get_action() { return selected_action_; };
    };
}

#endif // GYM_TRAJ_H
