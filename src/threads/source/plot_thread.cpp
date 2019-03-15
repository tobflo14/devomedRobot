#include "plot_thread.h"
#include "global_struct.h"
#include "plot2d.h"
#include <iostream>
#include <unistd.h>

void* PlotThread(void* arg) {
    std::cout << "Starting plot thread" << std::endl;

    shared_robot_data *robot_data = (shared_robot_data *)arg;
    Plot2d plot = Plot2d(1000, 1);

    bool has_started = false;
    double dt;
    double last_time;

    plot.initPlotWindow();
    plot.initializeBuffer(1);

    while (!has_started) {
        if (robot_data->run) {
            has_started = true;
            std::cout << "velocity lopp started" << std::endl;
            dt = 0.001;
            last_time = robot_data->timer;
        }
        usleep(10000); //Sleep for 10ms
    }

    while (robot_data->run) {
        dt = robot_data->timer - last_time;
        last_time = robot_data->timer;
        plot.graph_update(robot_data->robot_velocity[2], robot_data->desired_velocity[2]);
        plot.drawGraph();
        plot.swapBuffers();
        usleep(10000); //sleep for 10ms
    }
    plot.deleteBuffers();

    return NULL;

}
