
#include "global_struct.h"
#include "robot_loop_thread.h"
#include "calculate_velocity_thread.h"
#include "plot_thread.h"
#include "keyboard_input_thread.h"
#include "gui_thread.h"

#include <iostream>
#include <thread>


#define NUM_THREADS 5


static shared_robot_data robot_data;

Eigen::Vector3d robot_position;
Eigen::Vector3d robot_velocity;
Eigen::Vector3d robot_acceleration;
Eigen::Vector3d robot_jerk;
Eigen::Vector3d desired_velocity;
Eigen::Vector3d setpoint_velocity;
Eigen::Vector3d external_force;
Eigen::Vector3d setpoint_acc;
std::vector<Point> plot1;
std::vector<Point> plot2;
double kp = 0.9;
double ki = 0.0;
double kd = 0.01;
double fake_mass = 1.0; //Simulated mass in kg
double timer;
bool run;
bool shutdown;

int main(int argc, char** argv) {
    int rc;

    desired_velocity.setZero();
    run = false;
    shutdown = false;

    robot_data.robot_position = robot_position;
    robot_data.robot_velocity = robot_velocity;
    robot_data.robot_acceleration = robot_acceleration;
    robot_data.robot_jerk = robot_jerk;
    robot_data.desired_velocity = desired_velocity;
    robot_data.setpoint_velocity = setpoint_velocity;
    robot_data.external_force = external_force;
    robot_data.setpoint_acc = setpoint_acc;
    robot_data.plot1 = plot1;
    robot_data.plot2 = plot2;
    robot_data.kp = kp;
    robot_data.ki = ki;
    robot_data.kd = kd;
    robot_data.fake_mass = fake_mass;
    robot_data.timer = timer;
    robot_data.run = run;
    robot_data.shutdown = shutdown;

    pthread_t threads[NUM_THREADS];

    int i;
    for( i = 0; i < NUM_THREADS; i++ ) {
        if (i == 0) {
            rc = pthread_create(&threads[i], NULL, GuiThread, &robot_data);
        }
        else if (i == 1) {
            rc = pthread_create(&threads[i], NULL, CalculateVelocityThread, &robot_data);
        }
        else if (i == 2) {
            rc = pthread_create(&threads[i], NULL, PlotThread, &robot_data);
        }
        else if (i == 3) {
            rc = pthread_create(&threads[i], NULL, KeyboardInputThread, &robot_data);
        }
        else if (i == 4) {
            rc = pthread_create(&threads[i], NULL, RobotLoopThread, &robot_data);
        }

        if (rc) {
            std::cout << "Error: unable to create thread, " << rc << std::endl;
            exit(EXIT_FAILURE);
        }
    }
    for( i = 0; i < NUM_THREADS; i++ ) 
    {
        std::cout << "Joining thread " << i << std::endl;
        pthread_join(threads[i],NULL);
        std::cout << "Joined thread " << i << std::endl;
    }
    std::cout << "DONE DONE DONE" << std::endl;
    //pthread_exit(NULL);

    return 0;

}