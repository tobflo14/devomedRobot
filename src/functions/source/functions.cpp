//2D implementation of the Ramer-Douglas-Peucker algorithm
//By Tim Sheerman-Chase, 2016
//Released under CC0
//https://en.wikipedia.org/wiki/Ramer%E2%80%93Douglas%E2%80%93Peucker_algorithm
#include "functions.h"
/*
Vector3d get_position(const franka::RobotState& robot_state) {
  return Vector3d(
    robot_state.O_T_EE[12],
    robot_state.O_T_EE[13],
    robot_state.O_T_EE[14]
  );
}
*/
Vector3d get_measured_cartesian_velocity(const franka::RobotState& robot_state, const Eigen::Matrix<double, 6, 7> jacobian) {

  Eigen::Map<const Eigen::Matrix<double, 7, 1> > dq(robot_state.dq_d.data());
  VectorXd cartesian_velocity = jacobian * dq;
  return cartesian_velocity.head(3);
}
/*
Vector3d get_desired_acceleration(const franka::RobotState& robot_state) {
  return Vector3d(
    robot_state.O_ddP_EE_d[0],
    robot_state.O_ddP_EE_d[1],
    robot_state.O_ddP_EE_d[2]
  );
}
*/
Vector3d get_ang_velocity(const franka::RobotState& robot_state) {
  return Vector3d(
    robot_state.O_dP_EE_c[3],
    robot_state.O_dP_EE_c[4],
    robot_state.O_dP_EE_c[5]
  );
}

Vector3d get_ang_acceleration(const franka::RobotState& robot_state) {
  return Vector3d(
    robot_state.O_ddP_EE_c[3],
    robot_state.O_ddP_EE_c[4],
    robot_state.O_ddP_EE_c[5]
  );
}

Vector3d get_ext_force_filtered(const franka::RobotState& robot_state, Vector3d prev_force, double cutoff) {
  Vector3d force = get_ext_force(robot_state);
  //Filters the magnitude of the vector
  return force.normalized()*franka::lowpassFilter(0.001, force.norm(), prev_force.norm(), cutoff);
}

Vector3d get_ext_force(const franka::RobotState& robot_state) {
  return Vector3d(
    -robot_state.K_F_ext_hat_K[0]-0.33,
    robot_state.K_F_ext_hat_K[1]+0.7,
    robot_state.K_F_ext_hat_K[2]+1.65
  );
}

Vector3d get_ext_ang_force(const franka::RobotState& robot_state) {
  return Vector3d(
    -(robot_state.K_F_ext_hat_K[3]+0.23),
    robot_state.K_F_ext_hat_K[4]+0.05,
    robot_state.K_F_ext_hat_K[5]-0.08
  );
}

//Limit the length of a 3D vector to a specific value
void limitVector(Eigen::Vector3d& v, double limit) {
  if (v.norm() > limit) {
    v.normalize();
    v *= limit;
  }
}

//Limit each value of a vector of size n, with corresponding limits from an equal sized vector
void limitEach(Eigen::VectorXd& v, std::vector<double> limit) {
  for (size_t i = 0; i < v.size(); i++) {
    v[i] = limitValue(v[i], limit[i]);
  }
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
/*
MatrixXd RamerDouglasPeucker(const MatrixXd pointList, double epsilon) {
  std::vector<Vector3d> output;
	if(pointList.cols()<3) {
		return pointList;
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
*/
MatrixXd readMatrix(const char *filename)
    {
    std::ifstream indata;
    indata.open(filename);
    std::string line;
    std::vector<double> values;
    uint rows = 0;
    while (std::getline(indata, line)) {
        std::stringstream lineStream(line);
        std::string cell;
        while (std::getline(lineStream, cell, ';')) {
            values.push_back(std::stod(cell));
        }
        ++rows;
    }
    return Map<const MatrixXd>(values.data(), rows, values.size()/rows);
  };