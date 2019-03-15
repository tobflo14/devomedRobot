
#include "global_struct.h"
#include "robot_loop_thread.h"
#include "calculate_velocity_thread.h"
#include "plot_thread.h"

#include <iostream>

#include <thread>


#define NUM_THREADS 3


static shared_robot_data robot_data;

Eigen::Vector3d robot_position;
Eigen::Vector3d robot_velocity;
Eigen::Vector3d robot_acceleration;
Eigen::Vector3d robot_jerk;
Eigen::Vector3d desired_velocity;
double timer;
bool run;

int main(int argc, char** argv) {
    int rc;

    desired_velocity.setZero();
    run = false;

    robot_data.robot_position = robot_position;
    robot_data.robot_velocity = robot_velocity;
    robot_data.robot_acceleration = robot_acceleration;
    robot_data.robot_jerk = robot_jerk;
    robot_data.desired_velocity = desired_velocity;
    robot_data.timer = timer;
    robot_data.run = run;

    pthread_t threads[NUM_THREADS];
    int i;

    for( i = 0; i < NUM_THREADS; i++ ) {
        if (i == 0) {
            rc = pthread_create(&threads[i], NULL, RobotLoopThread, &robot_data);
        }
        else if (i == 1) {
            rc = pthread_create(&threads[i], NULL, CalculateVelocityThread, &robot_data);
        }
        else if (i == 2) {
            rc = pthread_create(&threads[i], NULL, PlotThread, &robot_data);
        }
        if (rc) {
            std::cout << "Error:unable to create thread," << rc << std::endl;
            exit(EXIT_FAILURE);
        }
    }

    for(i = 0; i < NUM_THREADS; i++ ) 
    {
        pthread_join(threads[i],NULL);
    }

    pthread_exit(NULL);

    return 0;

}