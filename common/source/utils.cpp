#include "utils.hpp"
#include "robot_structs.hpp"

#include <cmath>

#include <unsupported/Eigen/EulerAngles>



typedef EulerSystem<EULER_X, EULER_Y, EULER_Z> EulerSystem;
// typedef EulerAngles<float, rotationEulerConv> rotationEulerAngles;
/*
Vector3d get_position(const franka::RobotState& robot_state) {
      		return Vector3d(robot_state.O_T_EE[12], robot_state.O_T_EE[13],
                             robot_state.O_T_EE[14]);
}

    	*/
Quaterniond get_orientation(const franka::RobotState& robot_state) {

    // Taking out the Rotation part for O_T_EE from robot, and push it through Quaterniond
    // Makes a quaternion of input robot state.O_T_EE
    Matrix3d rot_mat;

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            rot_mat(i, j) = robot_state.O_T_EE[j * 4 + i];
        }
    }
    // Checking euler is done correct
        // Eigen::Vector3d E_ang = rot_mat_to_euler(rot_mat);
        // // Current quaternion state for robot
        // Eigen::Quaterniond q_state_curr = Quaterniond(rot_mat);
        // Eigen::Vector3d E_quad = quaternion_to_euler(q_state_curr);
        // printf("E_ang [%+4f %+.4f %+.4f] E_quat [%+4f %+.4f %+.4f]\n",
        //             E_ang[0], E_ang[1], E_ang[2], E_quad[0], E_quad[1], E_quad[2]);

    return Quaterniond(rot_mat);

    // Noted in file earlier: yaw = z, roll = y, pitch = x

    ////// Coppied from VIVE-controller native setup for calcualtion of their quaternion.

        // double r_w = sqrt(abs(1+robot_state.O_T_EE[0]+robot_state.O_T_EE[5]+robot_state.O_T_EE[10]))/2;

        // if (abs(r_w) < 0.0001)
        // {
        //     r_w = 0.0001;
        // }

        // double r_x = (robot_state.O_T_EE[9]-robot_state.O_T_EE[6])/(4*r_w);
        // double r_y = (robot_state.O_T_EE[2]-robot_state.O_T_EE[8])/(4*r_w);
        // double r_z = (robot_state.O_T_EE[4]-robot_state.O_T_EE[1])/(4*r_w);

        // Quaterniond rot_quat = Quaterniond(r_w,r_x, r_y, r_z);


    ////// From Haakons code-bit.
    
        // double theta1 = -asin(robot_state.O_T_EE[2]);
        // double rollo = atan2(robot_state.O_T_EE[6]/cos(theta1), robot_state.O_T_EE[10]/cos(theta1));
        // Vector3d rot_vec = Vector3d(
        //                              rollo > 0 ? rollo - M_PI : rollo + M_PI, 
        //                              theta1,
        //                              atan2(robot_state.O_T_EE[1]/cos(theta1),robot_state.O_T_EE[0]/cos(theta1))
        //                             );
        //
        // Quaterniond rot_quat =    AngleAxisd(rot_vec[2], Vector3d::UnitZ())
        //                          *AngleAxisd(rot_vec[1], Vector3d::UnitY())
        //                          *AngleAxisd(rot_vec[0], Vector3d::UnitX());
}


Vector3d get_rotation(const franka::RobotState& robot_state) {
      		return Vector3d(robot_state.O_dP_EE_c[3], robot_state.O_dP_EE_c[4],
                             robot_state.O_dP_EE_c[5]);
}


Vector3d get_velocity(const franka::RobotState& robot_state) {
      		return Vector3d(robot_state.O_dP_EE_c[0], robot_state.O_dP_EE_c[1],
                             robot_state.O_dP_EE_c[2]);
}

Vector3d get_impulse(const franka::RobotState& robot_state) {
      		return Vector3d(robot_state.O_ddP_EE_c[3], robot_state.O_ddP_EE_c[4],
                             robot_state.O_ddP_EE_c[5]);
}


Vector3d get_acceleration(const franka::RobotState& robot_state) {
      		return Vector3d(robot_state.O_ddP_EE_c[0], robot_state.O_ddP_EE_c[1],
                             robot_state.O_ddP_EE_c[2]);
}


///////////////////////////////////////////////////////////////////////////////
/// NOT PROVEN TO WORK
MatrixXd angular_vel_transform_quat(Quaterniond quat)
{
	MatrixXd T_q(4, 3);
	T_q(0, 0) = -quat.x();
	T_q(0, 1) = -quat.y();
	T_q(0, 2) = -quat.z();

	T_q(1, 0) = quat.w();
	T_q(1, 1) = -quat.z();
	T_q(1, 2) = quat.y();

	T_q(2, 0) = quat.z();
	T_q(2, 1) = quat.w();
	T_q(2, 2) = -quat.x();

	T_q(3, 0) = -quat.y();
	T_q(3, 1) = quat.x();
	T_q(3, 2) = quat.w();

	return T_q;
}

///////////////////////////////////////////////////////////////////////////////
/// NOT PROVEN TO WORK
MatrixXd angular_vel_transform_euler(Vector3d euler_ang)
{   
    // ROLL PITCH YAW
    // X    Y     Z
    // PSI  THETA PHI 
    double c_theta = cos(euler_ang[1]);
    double c_phi = cos(euler_ang[2]);

    double s_phi = sin(euler_ang[2]);

    double t_theta = tan(euler_ang[1]);

	MatrixXd T_e(3, 3);
	T_e(0, 0) = 1;
	T_e(0, 1) = 0;
	T_e(0, 2) = 0;

	T_e(1, 0) = s_phi*t_theta;
	T_e(1, 1) = c_phi;
	T_e(1, 2) = s_phi/c_theta;

	T_e(2, 0) = c_phi*t_theta;
	T_e(2, 1) = -s_phi;
	T_e(2, 2) = c_phi/c_theta;

	return T_e;
}

double precision_floor(double input, unsigned int precision)
{
    double factor = std::pow(10, precision);
    double sgn = (input < 0.0) ? -1.0 : 1.0;
    return std::floor(sgn * input * factor) / factor;
}


// Function for quaternion to euler anlge
Vector3d quaternion_to_euler(Quaterniond quat_input) // return vector of euler (x, y, z) i.e. (roll, pitch, yaw)
{
    double roll;
    double pitch;
    double yaw;

	// roll (x-axis rotation)
	double sinr = +2.0 * (quat_input.w() * quat_input.x() + quat_input.y() * quat_input.z());
	double cosr = +1.0 - 2.0 * (quat_input.x() * quat_input.x() + quat_input.y() * quat_input.y());
	roll = atan2(sinr, cosr);

	// pitch (y-axis rotation)
	double sinp = +2.0 * (quat_input.w() * quat_input.y() - quat_input.z() * quat_input.x());
	if (fabs(sinp) >= 1)
    {
		pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    }
    else
    {
		pitch = asin(sinp);
    }

	// yaw (z-axis rotation)
	double siny = +2.0 * (quat_input.w() * quat_input.z() + quat_input.x() * quat_input.y());
	double cosy = +1.0 - 2.0 * (quat_input.y() * quat_input.y() + quat_input.z() * quat_input.z());  
	yaw = atan2(siny, cosy);

    return Vector3d(roll, pitch, yaw); 
}

Quaterniond conj_of_quaternion(Quaterniond qua_in)
{
    // Eq. bellow is from eq. (4) in Fersk et al. (2013) ECC in CH, 'Full Quaternion Based Attitude...'
    // Negate only vector
    qua_in.w() =  qua_in.w();
    qua_in.x() = -qua_in.x();
    qua_in.y() = -qua_in.y();
    qua_in.z() = -qua_in.z();

    return qua_in;
}

Quaterniond cross_prod_of_quaternion(Quaterniond quaA, Quaterniond quaB)
{
    // Cross prod of quaternion is non-trivial
    // Eq. bellow follows from Pham et al. (2010) IEEE/RSJ in TW, 'Position and Orientation Control of Robot...'
    // Eq. (6) show how two quaternions are crossed (outer product).

    // Re-write for readability functions: Setting specific omega, and defining [r_x, r_y, r_z] as vector
    float w_A = quaA.w();
    float w_B = quaB.w();
    Vector3d vec_A = Vector3d(quaA.x(), quaA.y(), quaA.z());
    Vector3d vec_B = Vector3d(quaB.x(), quaB.y(), quaB.z());

    // Calculating new: w
    // float cross_w = (w_A * w_B) - (vec_A[0]*vec_B[0] + vec_A[1]*vec_B[1] + vec_A[2]*vec_B[2]);
    float cross_w = (w_A * w_B) - (vec_A.dot(vec_B));
    // Calculating new: [r_x, r_y, r_z]
    Vector3d cross_vec = (w_A * vec_B) + (w_B * vec_A) + vec_A.cross(vec_B);

    Quaterniond cross_qau = Quaterniond(cross_w, cross_vec[0],cross_vec[1],cross_vec[2]);

    return cross_qau;
}

// Function for rotation matrix to euler angles
Vector3d rot_mat_to_euler(Matrix3d rot_mat)
{
    // Calclation our rot_mat to Euler
    double sy = sqrt(rot_mat(0,0) * rot_mat(0,0) +  rot_mat(1,0) * rot_mat(1,0) );
    bool singular = sy < 1e-6;
    double ex, ey, ez;
    if (singular)
    {
        ex = atan2(-rot_mat(1,2), rot_mat(1,1));
        ey = atan2(-rot_mat(2,0), sy);
        ez = 0;
    }
    else
    {
        ex = atan2(rot_mat(2,1) , rot_mat(2,2));
        ey = atan2(-rot_mat(2,0), sy);
        ez = atan2(rot_mat(1,0), rot_mat(0,0));
    }

    return Vector3d(ex, ey, ez);
}
