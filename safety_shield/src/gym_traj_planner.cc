#include "safety_shield/gym_traj_planner.h"

// #include <iostream>

namespace safety_shield
{
    std::vector<Motion> GymTrajPlanner::planner_point(Eigen::Vector2d action, Eigen::Vector2d robot_vel, Eigen::Vector2d previous_ctrl, Eigen::Matrix2d robot_rot, Eigen::Vector2d robot_com)
    {
        // std::cout << "action = "<< action.transpose() << std::endl;
        std::vector<Motion> planned_motions;
        planned_motions.reserve(steps_ahead);

        Eigen::Vector2d acceleration(0, 0);
        std::vector<double> pos(robot_com.data(), robot_com.data() + robot_com.size());
        std::vector<double> vel(robot_vel.data(), robot_vel.data() + robot_vel.size());
        std::vector<double> acc(acceleration.data(), acceleration.data() + acceleration.size());

        // std::cout << "initial motion" << std::endl;
        planned_motions.push_back(Motion(0, pos, vel, acc));

        action.cwiseMax(0.05).cwiseMin(-0.05); // clip force to Mujoco actuator range
        for (int i = 1; i < steps_ahead; i++)
        {
            // std::cout << "Start : " << i << std::endl;

            double acc_h = action(0) / point_mass;
            const double theta = action(1);

            // integrate rotation matrix
            Eigen::Matrix2d thetadot;
            double ct = std::cos(theta * timestep);
            double st = std::sin(theta * timestep);
            thetadot << ct, -st, st, ct;
            robot_rot = robot_rot * thetadot;

            Eigen::Vector2d acc_h_vector;
            acc_h_vector << acc_h, 0;
            acceleration = robot_rot * acc_h_vector;
            robot_vel += timestep * acceleration;
            robot_com += timestep * robot_vel;

            // std::cout << "VectorXd to vector" << std::endl;
            // std::cout<< "Pos at step " << i << " : " << robot_com.transpose() << std::endl;
            Eigen::Map<Eigen::Vector2d>(pos.data(), pos.size()) = robot_com;
            Eigen::Map<Eigen::Vector2d>(vel.data(), vel.size()) = robot_vel;
            Eigen::Map<Eigen::Vector2d>(acc.data(), acc.size()) = acceleration;
            // std::cout << "Assign motion" << std::endl;

            planned_motions.push_back(Motion(0, pos, vel, acc));
        }
        return planned_motions;
    }

}