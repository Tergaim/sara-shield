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
        int steps_ahead;
        const double point_mass = 0.00519;
        const double car_mass = 0.03045;
        double timestep;

    public:
        GymTrajPlanner(int steps_ahead, double timestep) : steps_ahead(steps_ahead), timestep(timestep) {}

        std::vector<Motion> planner_point(Eigen::Vector2d action, Eigen::Vector2d robot_vel, Eigen::Vector2d previous_ctrl, Eigen::Matrix2d robot_rot, Eigen::Vector2d robot_com);
        void planner_car(Eigen::Vector2d action, Eigen::Vector2d robot_vel, Eigen::Vector2d previous_ctrl, Eigen::Matrix2d robot_rot, Eigen::Vector2d robot_com){};
    };
}

#endif // GYM_TRAJ_H
