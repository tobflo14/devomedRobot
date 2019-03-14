#include "calculate_velocity_thread.h"
#include "global_struct.h"
#include "functions.h"
#include "examples_common.h"
#include "pid.h"

void* CalculateVelocityThread(void* arg) {
    std::cout << "Starting calculateVelocity thread" << std::endl;

    shared_robot_data *robot_data = (shared_robot_data *)arg;

    do {
    Pid pid = Pid(0.5, 0, 0);
    pid.regulateVelocity(robot_data);
    } while (robot_data->run);

    return NULL;
}