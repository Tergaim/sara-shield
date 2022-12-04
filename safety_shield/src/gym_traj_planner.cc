#include "safety_shield/gym_traj_planner.h"

// #include <iostream>

namespace safety_shield
{
    std::vector<Motion> GymTrajPlanner::planner_point(Eigen::Vector2d action, Eigen::Vector2d robot_vel, Eigen::Vector2d previous_ctrl, Eigen::Matrix2d robot_rot, Eigen::Vector2d robot_com)
    {
        // std::cout << "action = "<< action.transpose() << std::endl;
        Eigen::Map<Eigen::Vector2d>(selected_action_.data(), selected_action_.size()) = action;

        std::vector<Motion> planned_motions;
        planned_motions.reserve(steps_ahead_);

        bool initial_loop_ok = planner_point_loop(action, robot_vel, previous_ctrl, robot_rot, robot_com, planned_motions);

        if (initial_loop_ok || n_tries_ < 1)
            return planned_motions;

        int valid_actions_found = 0;
        int total_tries = 0;

        Motion original_goal = planned_motions[planned_motions.size() - 1];
        float best_dist = 10000000;

        while (valid_actions_found < n_tries_ && total_tries < 100)
        {
            for (int i = 1; i < steps_ahead_; i++)
            {
                total_tries++;
                std::vector<Motion> replanned_motions;
                replanned_motions.reserve(steps_ahead_);
                Eigen::Vector2d action = 0.05 * Eigen::Vector2d::Random(); // select random action in [-0.05; 0.05], ctrl range limits
                bool loop_ok = planner_point_loop(action, robot_vel, previous_ctrl, robot_rot, robot_com, replanned_motions);

                if (loop_ok)
                {
                    valid_actions_found++;
                    Motion new_goal = replanned_motions[replanned_motions.size() - 1];
                    float dist = new_goal.squaredDist(original_goal);
                    if (dist < best_dist)
                    {
                        planned_motions = replanned_motions;
                        best_dist = dist;
                        Eigen::Map<Eigen::Vector2d>(selected_action_.data(), selected_action_.size()) = action;
                    }
                }
            }
        }
        return planned_motions;
    }

    bool GymTrajPlanner::planner_point_loop(Eigen::Vector2d action, Eigen::Vector2d robot_vel, Eigen::Vector2d previous_ctrl, Eigen::Matrix2d robot_rot, Eigen::Vector2d robot_com, std::vector<Motion> &planned_motions)
    {
        bool no_collision = true;
        Eigen::Vector2d acceleration(0, 0);
        std::vector<double> pos(robot_com.data(), robot_com.data() + robot_com.size());
        std::vector<double> vel(robot_vel.data(), robot_vel.data() + robot_vel.size());
        std::vector<double> acc(acceleration.data(), acceleration.data() + acceleration.size());

        planned_motions.push_back(Motion(0, pos, vel, acc));

        action.cwiseMax(0.05).cwiseMin(-0.05); // clip force to Mujoco actuator range

        for (int i = 1; i < steps_ahead_; i++)
        {
            if (i >= 10)
            {
                action = point_slowdown(robot_vel, action(0), action(1), robot_rot);
                action.cwiseMax(0.05).cwiseMin(-0.05);
            }
            double acc_h = action(0) / point_mass_;
            const double theta = action(1);

            // integrate rotation matrix
            Eigen::Matrix2d thetadot;
            double ct = std::cos(theta * timestep_);
            double st = std::sin(theta * timestep_);
            thetadot << ct, -st, st, ct;
            robot_rot = robot_rot * thetadot;

            Eigen::Vector2d acc_h_vector;
            acc_h_vector << acc_h, 0;
            acceleration = robot_rot * acc_h_vector;
            robot_vel += timestep_ * acceleration;
            robot_com += timestep_ * robot_vel;

            for (int i = 0; i < obstacles_.size(); i++)
                if ((robot_com - obstacles_[i]).squaredNorm() < (obstacles_radius_[i] + 0.15) * (obstacles_radius_[i] + 0.15))
                    no_collision = false;

            Eigen::Map<Eigen::Vector2d>(pos.data(), pos.size()) = robot_com;
            Eigen::Map<Eigen::Vector2d>(vel.data(), vel.size()) = robot_vel;
            Eigen::Map<Eigen::Vector2d>(acc.data(), acc.size()) = acceleration;
            planned_motions.push_back(Motion(0, pos, vel, acc));
        }
        return no_collision;
    }

    // Eigen::Vector2d point_slowdown(Eigen::Vector2d robot_vel, Eigen::Vector2d planned_action, Eigen::Matrix2d robot_rot)
    Eigen::Vector2d GymTrajPlanner::point_slowdown(Eigen::Vector2d robot_vel, double action_0, double action_1, Eigen::Matrix2d robot_rot)
    {
        // Eigen::Vector2d planned_action;
        Eigen::Vector2d v_h = robot_rot.inverse() * robot_vel; // project velocity on x axis
        double sign_vx = (v_h(0) > 0.01) - (v_h(0) < -0.01);
        Eigen::Vector2d action;
        action << -sign_vx, 0;
        if (sign_vx > 0.5 || sign_vx < 0.5)
        {
            double l1 = v_h(0) * timestep_ + timestep_ * timestep_ * (action_0 / 2); // distance traveled in next timestep with planned action
            double l2 = v_h(0) * timestep_ - timestep_ * timestep_ * (sign_vx * 0.05 / 2);    // distance traveled in next timestep with slowdown
            action(1) = action_1 * (l2 / l1);
        }
        return action;
    }
}