//functions.h

#include <iostream>
#include <cmath>
#include <utility>
#include <vector>
#include <stdexcept>
#include <fstream>

#include <Eigen/Dense>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>

#define MAXBUFSIZE  ((int) 1e6)

using namespace std;
using namespace Eigen;

typedef std::pair<double, double> Point;

Vector3d get_position(const franka::RobotState& robot_state);
Vector3d get_measured_cartesian_velocity(const franka::RobotState& robot_state, const Eigen::Matrix<double, 6, 7> jacobian);
Vector3d get_desired_acceleration(const franka::RobotState& robot_state);
Vector3d get_ang_velocity(const franka::RobotState& robot_state);
Vector3d get_ang_acceleration(const franka::RobotState& robot_state);
Vector3d get_ext_force_filtered(const franka::RobotState& robot_state, Vector3d prev_force, double cutoff);
Vector3d get_ext_force(const franka::RobotState& robot_state);
Vector3d get_ext_ang_force(const franka::RobotState& robot_state);

void limitVector(Eigen::Vector3d& v, double limit);
void limitEach(Eigen::VectorXd& v, std::vector<double> limit);
double limitValue(double value, double limit);
double flattenZero(double value, double range);

void RamerDouglasPeucker(const vector<Point> &pointList, double epsilon, vector<Point> &out);

MatrixXd readMatrix(const char *filename);
