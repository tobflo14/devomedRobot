#ifndef GLOBAL_STRUCT_H
#define GLOBAL_STRUCT_H

//#include <eigen3/Eigen/Core>
//#include <eigen3/Eigen/Geometry>
#include <Eigen/Core>
#include <Eigen/Geometry>


#define ROBOT_IP "172.16.0.2"

struct shared_robot_data
{
    Eigen::Vector3d robot_position;
    Eigen::Vector3d robot_velocity;
    Eigen::Vector3d robot_acceleration;
    Eigen::Vector3d robot_jerk;
    Eigen::Vector3d desired_velocity;
    double timer;
    bool run;
};

#endif