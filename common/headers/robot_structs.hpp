#ifndef ROBOT_STRUCTS_HPP_
#define ROBOT_STRUCTS_HPP_

#include "stdafx.hpp"
#include "common.hpp"

struct grasp_vars
{
    double grasping_width = 0.05; 
    double grasping_speed = 0.05;
    double grasping_force = 60.0;
    double grasping_epsilon_inner = 0.05;
    double grasping_epsilon_outer = 0.05;
};

struct robot_collision_behavior
{
    std::array<double, 7> lower_torque_thresholds_nominal;
    std::array<double, 7> upper_torque_thresholds_nominal;
    std::array<double, 7> lower_torque_thresholds_acceleration;
    std::array<double, 7> upper_torque_thresholds_acceleration;
    std::array<double, 6> lower_force_thresholds_nominal;
    std::array<double, 6> upper_force_thresholds_nominal;   
    std::array<double, 6> lower_force_thresholds_acceleration;
    std::array<double, 6> upper_force_thresholds_acceleration;

    robot_collision_behavior()
    {
        lower_torque_thresholds_nominal         = {{25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.}};
        upper_torque_thresholds_nominal         = {{35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0}};
        lower_torque_thresholds_acceleration    = {{25.0, 25.0, 22.0, 20.0, 19.0, 17.0, 14.0}};
        upper_torque_thresholds_acceleration    = {{35.0, 35.0, 32.0, 30.0, 29.0, 27.0, 24.0}};
        lower_force_thresholds_nominal          = {{30.0, 30.0, 30.0, 25.0, 25.0, 25.0}};
        upper_force_thresholds_nominal          = {{40.0, 40.0, 40.0, 35.0, 35.0, 35.0}};
        lower_force_thresholds_acceleration     = {{30.0, 30.0, 30.0, 25.0, 25.0, 25.0}};
        upper_force_thresholds_acceleration     = {{40.0, 40.0, 40.0, 35.0, 35.0, 35.0}};
    };
};

#endif
