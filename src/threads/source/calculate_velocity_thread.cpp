#include "calculate_velocity_thread.h"
#include "global_struct.h"
//#include "functions.h"
//#include "examples_common.h"
#include "pid.h"
#include <unistd.h>
#include <iostream>

void* CalculateVelocityThread(void* arg) {
    std::cout << "Starting calculateVelocity thread" << std::endl;

    shared_robot_data *robot_data = (shared_robot_data *)arg;
    //Pid velocityPid = Pid();
    double dt;
    double last_time = robot_data->timer;
    double time = 0.0;
    double pid_frequency = 1000; //Frequency of the PID loop in Hz
    bool finished = false;
    //double vel_prev = 0.0;

    while (!finished) {
        while (robot_data->run) {
            //dt = robot_data->timer - last_time;
            dt = 1/pid_frequency;
            last_time = robot_data->timer;
            

            //If we are shutting down while the robot is running, set the speed to zero. The robot loop will wait for the speed to be zero before it sets run=false.
            if (robot_data->shutdown) {
                robot_data->setpoint_velocity = {0.0, 0.0, 0.0};
            } else {
                /*
                time += dt;
                if (time > 1.0) {
                    robot_data->setpoint_velocity(0,0) = 0.2;
                }
                if (time > 2.0) {
                    robot_data->setpoint_velocity(0,0) = 0.0;
                }
                if (time > 3.0) {
                    robot_data->setpoint_velocity(0,0) = -0.2;
                }
                if (time > 4.0) {
                    robot_data->setpoint_velocity(0,0) = 0.0;
                    time = 0.0;
                }*/
                //robot_data->setpoint_acc = (1.0/robot_data->fake_mass)*robot_data->external_force; //a=F/m
                //robot_data->setpoint_acc = (1.0/robot_data->fake_mass)*(robot_data->external_force - robot_data->kp*robot_data->robot_velocity); //a=1/m * (F - Bv) Admittance controller
                //Set velocity setpoint to be fed to PID from acceleration setpoint
                //robot_data->setpoint_velocity = robot_data->robot_velocity + robot_data->setpoint_acc * dt;
                
            }/*
            //robot_data->robot_velocity(0,0) = franka::lowpassFilter(dt, robot_data->robot_velocity[0], vel_prev, 1);
            //vel_prev = robot_data->robot_velocity(0,0);
            velocityPid.setParameters(robot_data->kp, robot_data->ki, robot_data->kd);
            robot_data->desired_velocity = velocityPid.computePID(robot_data->setpoint_velocity-robot_data->robot_velocity ,dt);
            //robot_data->desired_velocity = robot_data->setpoint_velocity;
            usleep(1000000/pid_frequency); //Set frequency of PID loop
            */
        }
        //If we are shutting down without robot running, finish this thread
        if (robot_data->shutdown) {
            finished = true;
        }
        usleep(100000);
    }

    std::cout << "Calculate velocity thread shutting down" << std::endl;
    return NULL;
}