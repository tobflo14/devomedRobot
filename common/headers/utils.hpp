#ifndef CONTROLLER_UTILS_HPP_
#define CONTROLLER_UTILS_HPP_

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <franka/robot.h>
#include <franka/model.h>

using namespace Eigen;

void pd_velocity();

//Vector3d get_position(const franka::RobotState& robot_state);

Quaterniond get_orientation(const franka::RobotState& robot_state);

Vector3d get_rotation(const franka::RobotState& robot_state);

Vector3d get_velocity(const franka::RobotState& robot_state);

Vector3d get_impulse(const franka::RobotState& robot_state);

Vector3d get_acceleration(const franka::RobotState& robot_state);

MatrixXd angular_vel_transform_quat(Quaterniond quat);

MatrixXd angular_vel_transform_euler(Vector3d euler_ang);

double precision_floor(double input, unsigned int precision);

Vector3d quaternion_to_euler(Quaterniond quat_input);

Quaterniond conj_of_quaternion(Quaterniond qua_in);

Quaterniond cross_prod_of_quaternion(Quaterniond quaA, Quaterniond quaB);

Vector3d rot_mat_to_euler(Matrix3d rot_mat);


#endif
