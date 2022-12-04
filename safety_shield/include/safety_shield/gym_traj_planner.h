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
        const double point_mass_ = 0.00519;
        const double car_mass_ = 0.03045;
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
        void planner_car(Eigen::Vector2d action, Eigen::Vector2d robot_vel, Eigen::Vector2d previous_ctrl, Eigen::Matrix2d robot_rot, Eigen::Vector2d robot_com){};

        inline std::vector<double> get_action() { return selected_action_; };
    };
}

#endif // GYM_TRAJ_H
