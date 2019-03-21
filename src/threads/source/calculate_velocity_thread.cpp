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
    Pid pid = Pid(0.5, 0, 0.01);
  //  bool has_started = false;
    double dt = 0.001;
    double last_time = robot_data->timer;

    while (!(robot_data->shutdown)) {
        while (robot_data->run) {
            dt = robot_data->timer - last_time;
            last_time = robot_data->timer;
            pid.regulateVelocity(dt, arg);
            usleep(1000); //sleep for 1ms
        }
        usleep(100000);
    }

    std::cout << "Calculate velocity thread shutting down" << std::endl;
    return NULL;
}