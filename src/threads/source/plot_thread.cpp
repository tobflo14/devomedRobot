#include "plot_thread.h"
#include "global_struct.h"
#include "plot2d.h"

#include <iostream>
#include <unistd.h>
#include <sys/syscall.h>

void* PlotThread(void* arg) {
    std::cout << "Starting plot thread" << std::endl;
    std::cout << "Plot thread ID : " << syscall(SYS_gettid) << std::endl;

    shared_robot_data *robot_data = (shared_robot_data *)arg;
    Plot2d plot = Plot2d(1000, 1.0f, 2);
  
   // plot.deleteBuffers();
    plot.initPlotWindow();
    plot.initializeBuffer();
    while (!(robot_data->shutdown)) {
        while (robot_data->run) {
            double values[] = {robot_data->robot_velocity[0], robot_data->robot_acceleration[0]};
            plot.graph_update(values);
            plot.drawGraph();
            plot.swapBuffers();
            usleep(1000);
        }
        usleep(100000);
    }

    plot.deleteBuffers();
    std::cout << "Plot thread shutting down" << std::endl;
    return NULL;

}
