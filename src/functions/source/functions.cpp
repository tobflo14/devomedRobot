//2D implementation of the Ramer-Douglas-Peucker algorithm
//By Tim Sheerman-Chase, 2016
//Released under CC0
//https://en.wikipedia.org/wiki/Ramer%E2%80%93Douglas%E2%80%93Peucker_algorithm
#include "functions.h"

Vector3d closestPointOnLine(const Vector3d linePoint1, const Vector3d linePoint2, const Vector3d point) {
  Vector3d lineDirection = (linePoint2 -  linePoint1).normalized();
  double projectedDistance = (point - linePoint1).dot(lineDirection);
  if (projectedDistance <= 0) return linePoint1;
  if (projectedDistance >= (linePoint2 - linePoint1).norm()) return linePoint2;
  return linePoint1 + lineDirection*projectedDistance;
}

Vector3d closestPointOnLineSegment(const MatrixXd lines, Vector3d point, double &fractionCompleted) {
  int closestIndex = 0;
  double closestDistance = 10000;
  Vector3d closestPointOfAll;
  int num_lines = lines.cols()-1;
  for (int i = 0; i < num_lines; i++) {
    Vector3d closestPoint = closestPointOnLine(lines.col(i), lines.col(i+1), point);
    double distance = (closestPoint - point).squaredNorm();
    if (distance < closestDistance) {
      closestDistance = distance;
      closestPointOfAll = closestPoint;
      closestIndex = i;
    }
  }
  fractionCompleted = (double)closestIndex/(num_lines-1);
  return closestPointOfAll;
}

//Return a simple friction force with static friction up to a desired "max_friction_force", and then kinetic friction.
Vector3d frictionForce(Vector3d velocity, Vector3d external_force, double max_friction_force) {
  //Kinetic friction if we have velocity, or force exceeds the desired force value
  if (velocity.norm() > 0.01 || external_force.norm() > max_friction_force) {
      //Apply friction in the opposite direction of velocity
      return -velocity.normalized()*max_friction_force;
  } else {
      //Static friction: sum F = 0, and there is no movement.
      return -external_force;
  }
}

Vector3d get_position(const franka::RobotState& robot_state) {
  return Vector3d(
    robot_state.O_T_EE[12],
    robot_state.O_T_EE[13],
    robot_state.O_T_EE[14]
  );
}

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

//Filters the magnitude of the vector
Vector3d get_ext_force_magnitude_filtered(const franka::RobotState& robot_state, Vector3d prev_force, double cutoff) {
  Vector3d force = get_ext_force(robot_state);
  return force.normalized()*franka::lowpassFilter(0.001, force.norm(), prev_force.norm(), cutoff);
}

//Filters each element of the vector individually
Vector3d get_ext_force_filtered(const franka::RobotState& robot_state, Vector3d prev_force, double cutoff) {
  Vector3d force = get_ext_force(robot_state);
    return Vector3d(
    franka::lowpassFilter(0.001, force[0], prev_force[0], cutoff),
    franka::lowpassFilter(0.001, force[1], prev_force[1], cutoff),
    franka::lowpassFilter(0.001, force[2], prev_force[2], cutoff)
  );
}

Vector3d get_ext_force(const franka::RobotState& robot_state) {
  return Vector3d(
    -robot_state.K_F_ext_hat_K[0]-0.33,
    robot_state.K_F_ext_hat_K[1]+0.7,
    robot_state.K_F_ext_hat_K[2]+1.65
  );
}

//Filters the magnitude of the vector
Vector3d get_ext_ang_force_magnitude_filtered(const franka::RobotState& robot_state, Vector3d prev_force, double cutoff) {
  Vector3d force = get_ext_ang_force(robot_state);
  return force.normalized()*franka::lowpassFilter(0.001, force.norm(), prev_force.norm(), cutoff);
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
MatrixXd readMatrix(std::string filename) {
    std::ifstream indata;
    try {
      indata.open(filename.c_str());
    } catch (std::system_error& e) {
      std::cerr << e.what() << std::endl;
    }
    std::string line;
    std::vector<double> values;
    uint rows = 0;
    while (std::getline(indata, line)) {
        std::stringstream lineStream(line);
        std::string cell;
        while (std::getline(lineStream, cell, ';')) {
          
          std::stringstream cellstream(cell);
          double value;
          cellstream >> value;
          values.push_back(value);
          
          //values.push_back(std::stod(cell));
        }
        ++rows;
    }
    return Map<const MatrixXd>(values.data(), values.size()/rows, rows);
  };