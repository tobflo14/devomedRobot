#ifndef GLOBAL_STRUCT_H
#define GLOBAL_STRUCT_H

//#include <eigen3/Eigen/Core>
//#include <eigen3/Eigen/Geometry>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include "functions.h"

#define ROBOT_IP "172.16.0.2"

struct shared_robot_data
{
    franka::RobotState robot_state;
    franka::RobotMode robot_mode;
    Eigen::Vector3d robot_position;
    Eigen::Vector3d robot_velocity;
    Eigen::Vector3d robot_ang_velocity;
    Eigen::Vector3d robot_acceleration;
    Eigen::Vector3d robot_ang_acceleration;
    Eigen::Vector3d robot_jerk;
    Eigen::Vector3d robot_ang_jerk;
    Eigen::Vector3d desired_velocity;
    Eigen::Vector3d setpoint_velocity;
    Eigen::Vector3d setpoint_ang_velocity;
    Eigen::Vector3d external_force;
    Eigen::Vector3d external_ang_force;
    Eigen::Vector3d setpoint_acc;
    Eigen::Vector3d setpoint_ang_acc;
    std::vector<Point> plot1;
    std::vector<Point> plot2;
    std::vector<std::vector<double>> tracking_data;
    double ki;
    double kp;
    double kd;
    double timer;
    double fake_mass;
    double fractionCompleted;
    double wanted_force;
    bool run;
    bool shutdown;
    bool track_position;
    bool floating_mode;
    std::string open_file;
    Eigen::MatrixXd track_path;
    std::string error_message;
};

#endif