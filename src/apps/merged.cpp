
#include "common.hpp"
#include "cartesian_velocity_thread.hpp"
#include "solver_thread.hpp"
#include "robot_structs.hpp"

#include <thread>

#define NUM_THREADS 2

static shared_memory data;

// RHR Data... remove when not RHR by some means?
rhr_state rhr;
rhr_state_and_data combo_pack;


eeffector_pose eeffector;
robot_joint_states robot_joints = {};
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
double *z = new double[13];
double timer;
bool run = false;

//////////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv) 
{

    if (ENDEFFECTOR == RHR_HAND)
    {
        combo_pack.rhr = &rhr;
        combo_pack.data = &data;
    }

    pthread_t threads[NUM_THREADS];
    // std::thread threads[NUM_THREADS];

    data.eeffector = &eeffector;
    data.robot_joints  = &robot_joints;
    data.robot_orientation  = robot_orientation;
    data.robot_qua_ref_zero  = robot_qua_ref_zero;
    data.robot_qua_err  = robot_qua_err;
    data.robot_qua_ref  = robot_qua_ref;
    data.robot_position = robot_position;
    data.robot_pos_ref_zero = robot_pos_ref_zero;
    data.robot_pos_err = robot_pos_err;
    data.robot_pos_ref = robot_pos_ref;
    data.robot_rotation = robot_rotation;
    data.robot_velocity = robot_velocity;
    data.robot_impulse = robot_impulse;
    data.robot_acceleration = robot_acceleration;
    data.jacobian = jacobian;
    data.z = z;
    data.timer = timer;
    data.run = run;

    std::cout << "Main ID : " << syscall(SYS_gettid) << std::endl;

    int rc;
    int i; 
    for( i = 1; i <= NUM_THREADS; i++ ) 
    {

        if (i == 1)
        {
            rc = pthread_create(&threads[i-1], NULL, CartesianVelocityThread, &data);
        }

        else if (i == 2)
        {
            rc = pthread_create(&threads[i-1], NULL, SolverThread, &data);
        }
         
        if (rc) {
            std::cout << "Error:unable to create thread," << rc << std::endl;
            exit(EXIT_FAILURE);
        }
    }

    for( i = 0; i < NUM_THREADS; i++ ) 
    {
        pthread_join(threads[i],NULL);
    }


    pthread_exit(NULL);
    delete[] z;

    return 0;
}
