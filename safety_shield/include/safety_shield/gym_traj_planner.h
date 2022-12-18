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
        int steps_ahead_;
        int n_tries_;

        // point data
        const double point_mass_ = 0.00519;

        // car data
        const double car_mass_ = 0.03045;
        const double r_ = 0.05; // wheel radius
        const double L_ = 0.125; // half-distance between center of wheels 
        const double D_ = 0.1; // distance between center of robot and center of wheels 
        const double rho_ = D_/L_;
        double timestep_;
        std::vector<Eigen::Vector2d> obstacles_;
        std::vector<float> obstacles_radius_;
        std::vector<double> selected_action_;

    public:
        GymTrajPlanner(int steps_ahead, double timestep, std::vector<Eigen::Vector2d> obstacles, std::vector<float> obstacles_radius, int n_tries) : steps_ahead_(steps_ahead), timestep_(timestep), obstacles_(obstacles), obstacles_radius_(obstacles_radius), n_tries_(n_tries)
        {
            selected_action_ = {0, 0};
        }

        std::vector<Motion> planner_point(Eigen::Vector2d action, Eigen::Vector2d robot_vel, Eigen::Vector2d previous_ctrl, Eigen::Matrix2d robot_rot, Eigen::Vector2d robot_com);
        bool planner_point_loop(Eigen::Vector2d action, Eigen::Vector2d robot_vel, Eigen::Vector2d previous_ctrl, Eigen::Matrix2d robot_rot, Eigen::Vector2d robot_com, std::vector<Motion> &planned_motions);
        Eigen::Vector2d point_slowdown(Eigen::Vector2d robot_vel, double action_0, double action_1, Eigen::Matrix2d robot_rot);
        std::vector<Motion> planner_car(Eigen::Vector2d action, Eigen::Vector2d robot_vel, Eigen::Vector2d previous_ctrl, Eigen::Matrix2d robot_rot, Eigen::Vector2d robot_com);
        bool planner_car_loop(Eigen::Vector2d action, Eigen::Vector2d robot_vel, Eigen::Vector2d previous_ctrl, Eigen::Matrix2d robot_rot, Eigen::Vector2d robot_com, std::vector<Motion> &planned_motions);
        Eigen::Vector2d car_slowdown(Eigen::Vector2d robot_vel, double action_0, double action_1, Eigen::Matrix2d robot_rot);
        

        inline std::vector<double> get_action() { return selected_action_; };
    };
}

#endif // GYM_TRAJ_H
