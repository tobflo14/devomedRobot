//2D implementation of the Ramer-Douglas-Peucker algorithm
//By Tim Sheerman-Chase, 2016
//Released under CC0
//https://en.wikipedia.org/wiki/Ramer%E2%80%93Douglas%E2%80%93Peucker_algorithm
#include "functions.h"

Vector3d get_velocity(const franka::RobotState& robot_state) {
  return Vector3d(
    robot_state.O_dP_EE_c[0],
    robot_state.O_dP_EE_c[1],
    robot_state.O_dP_EE_c[2]
  );
}

Vector3d get_acceleration(const franka::RobotState& robot_state) {
  return Vector3d(
    robot_state.O_ddP_EE_c[0],
    robot_state.O_ddP_EE_c[1],
    robot_state.O_ddP_EE_c[2]
  );
}

Vector3d get_ext_force_filtered(const franka::RobotState& robot_state, Vector3d prev_force, double cutoff) {
  return Vector3d(
    franka::lowpassFilter(0.001, -robot_state.K_F_ext_hat_K[0]-0.33, prev_force[0], cutoff),
    franka::lowpassFilter(0.001, robot_state.K_F_ext_hat_K[1]+0.66, prev_force[1], cutoff),
    franka::lowpassFilter(0.001, robot_state.K_F_ext_hat_K[2]+1.65, prev_force[2], cutoff)
  );
}

Vector3d get_ext_force(const franka::RobotState& robot_state) {
  return Vector3d(
    -robot_state.K_F_ext_hat_K[0]-0.33,
    robot_state.K_F_ext_hat_K[1]+0.7,
    robot_state.K_F_ext_hat_K[2]+1.65
  );
}

double limitValue(double value, double limit) {
  value = min(value, limit);
  value = max(value, -limit);
  return value;
}

//Flattens the area around zero values with a size of 'range'
double flattenZero(double value, double range) {
  return (1- exp(-pow(value, 2)/pow(range, 2)))*value;
}

double PerpendicularDistance(const Point &pt, const Point &lineStart, const Point &lineEnd)
{
	double dx = lineEnd.first - lineStart.first;
	double dy = lineEnd.second - lineStart.second;

	//Normalise
	double mag = pow(pow(dx,2.0)+pow(dy,2.0),0.5);
	if(mag > 0.0)
	{
		dx /= mag; dy /= mag;
	}

	double pvx = pt.first - lineStart.first;
	double pvy = pt.second - lineStart.second;

	//Get dot product (project pv onto normalized direction)
	double pvdot = dx * pvx + dy * pvy;

	//Scale line direction vector
	double dsx = pvdot * dx;
	double dsy = pvdot * dy;

	//Subtract this from pv
	double ax = pvx - dsx;
	double ay = pvy - dsy;

	return pow(pow(ax,2.0)+pow(ay,2.0),0.5);
}

void RamerDouglasPeucker(const vector<Point> &pointList, double epsilon, vector<Point> &out)
{
	if(pointList.size()<2) {
		out = pointList;
  } else {
    // Find the point with the maximum distance from line between start and end
    double dmax = 0.0;
    size_t index = 0;
    size_t end = pointList.size()-1;
    for(size_t i = 1; i < end; i++)
    {
      double d = PerpendicularDistance(pointList[i], pointList[0], pointList[end]);
      if (d > dmax)
      {
        index = i;
        dmax = d;
      }
    }

    // If max distance is greater than epsilon, recursively simplify
    if(dmax > epsilon)
    {
      // Recursive call
      vector<Point> recResults1;
      vector<Point> recResults2;
      vector<Point> firstLine(pointList.begin(), pointList.begin()+index+1);
      vector<Point> lastLine(pointList.begin()+index, pointList.end());
      RamerDouglasPeucker(firstLine, epsilon, recResults1);
      RamerDouglasPeucker(lastLine, epsilon, recResults2);
  
      // Build the result list
      out.assign(recResults1.begin(), recResults1.end()-1);
      out.insert(out.end(), recResults2.begin(), recResults2.end());
      if(out.size()<2)
        throw runtime_error("Problem assembling output");
    } 
    else 
    {
      //Just return start and end points
      out.clear();
      out.push_back(pointList[0]);
      out.push_back(pointList[end]);
    }
  }
}