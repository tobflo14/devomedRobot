#ifndef COMMON_HPP_
#define COMMON_HPP_

#include <sys/syscall.h>
#include <sys/types.h>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>

// Operating modes
#define VIVE_POS 1
#define VIVE_ORIENT 2
#define VIVE_POS_AND_ORIENT 3

// Possible endeffector configurations
#define FRANKA_HAND 1
#define RHR_HAND 2

// Closed or open loop end effector command
#define ENDEFFECTOR_OPEN 1
#define ENDEFFECTOR_CLOSED 2

// Possible robot's to control
#define PANDA_ONE "172.16.0.2"
#define PANDA_TWO "10.0.0.2"

// Color to printf("%s YOUR TEXT %s\n", KRED, KNRM);
#define KNRM  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KYEL  "\x1B[33m"
#define KBLU  "\x1B[34m"
#define KMAG  "\x1B[35m"
#define KCYN  "\x1B[36m"
#define KWHT  "\x1B[37m"


///// CONFIG SHOULD BE MOVED /////
// Define operating mode
#define OPERATING_MODE VIVE_POS_AND_ORIENT // For no orientaion VIVE_POS

// Define command mode
#define COMMAND_MODE ENDEFFECTOR_OPEN // RHR needs Closed // Franka needs OPEN

// Define endeffector mounted on robot
#define ENDEFFECTOR FRANKA_HAND // Choice: RHR_HAND or FRANKA_HAND // Go to zmq_eefector_thread.hpp change ip address choice
// Robot selected for control
#define ROBOT_IP PANDA_ONE
///////////////////////////////////


struct finger
{
	float dst;
	float prx;
	float mot;
	float vel;
	float prs[9];
};

struct rhr_state
{
	int control;
	finger fingers[3];
	float preshape[2];
};

struct eeffector_pose
{
    bool move;
    Eigen::Vector3d position;
    bool rotate;
    Eigen::Quaterniond orientation;
    bool gripp;

    eeffector_pose() {
        move = false;
        position = Eigen::Vector3d(0.0, 0.0, 0.0);
        rotate = false;
        orientation = Eigen::Quaterniond(0.0, 0.0, 0.0, 0.0);
        gripp = false;
    }
};

struct robot_joint_states
{
    std::array<double, 7> q_prev;
    std::array<double, 7> dq_prev;
    std::array<double, 7> dq_prevprev;
};

struct shared_memory
{
    eeffector_pose *eeffector;
    robot_joint_states *robot_joints;
    Eigen::Quaterniond robot_orientation;
    Eigen::Quaterniond robot_qua_ref_zero;
    Eigen::Quaterniond robot_qua_err;
    Eigen::Quaterniond robot_qua_ref;
    Eigen::Vector3d robot_position;
    Eigen::Vector3d robot_pos_ref_zero;
    Eigen::Vector3d robot_pos_err;
    Eigen::Vector3d robot_pos_ref;
    Eigen::Vector3d robot_rotation;
    Eigen::Vector3d robot_velocity;
    Eigen::Vector3d robot_impulse;
    Eigen::Vector3d robot_acceleration;
    std::array<double, 42> jacobian;
    double *z;
    std::array<double, 7> dq;
    double timer;
    bool run;
    // rhr_state *rhr;
};

struct rhr_state_and_data{
    rhr_state *rhr;
    shared_memory *data;
};

#endif
