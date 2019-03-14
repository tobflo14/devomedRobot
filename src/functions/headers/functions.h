//functions.h

#include <iostream>
#include <cmath>
#include <utility>
#include <vector>
#include <stdexcept>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>

using namespace std;

typedef std::pair<double, double> Point;

void RamerDouglasPeucker(const vector<Point> &pointList, double epsilon, vector<Point> &out);