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
    Pid pid = Pid(0.005, 0.0, 0.0);
  //  bool has_started = false;
    double dt;
    double last_time = robot_data->timer;
    double time = 0.0;
    int pid_frequency = 1000; //Frequency of the PID loop in Hz

    while (!(robot_data->shutdown)) {
        while (robot_data->run) {
            dt = robot_data->timer - last_time;
            last_time = robot_data->timer;
            /*
            time += dt;
            if (time > 1.0) {
                robot_data->external_velocity(0,0) = 0.1;
            }
            if (time > 2.0) {
                robot_data->external_velocity(0,0) = 0.0;
            }
            if (time > 3.0) {
                robot_data->external_velocity(0,0) = -0.1;
            }
            if (time > 4.0) {
                robot_data->external_velocity(0,0) = 0.0;
                time = 0.0;
            }
            */
            robot_data->external_velocity(0,0) = -1.0*robot_data->external_force[0];
            pid.regulateVelocity(dt, arg);
            usleep(1000000/pid_frequency); //Set frequency of PID loop
        }
        usleep(100000);
    }

    std::cout << "Calculate velocity thread shutting down" << std::endl;
    return NULL;
}