//functions.h

#include <iostream>
#include <cmath>
#include <utility>
#include <vector>
#include <stdexcept>

#include <Eigen/Dense>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>

using namespace std;
using namespace Eigen;

typedef std::pair<double, double> Point;

Vector3d get_velocity(const franka::RobotState& robot_state);
Vector3d get_acceleration(const franka::RobotState& robot_state);

void RamerDouglasPeucker(const vector<Point> &pointList, double epsilon, vector<Point> &out);