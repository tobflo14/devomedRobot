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
    Plot2d plot = Plot2d(200, 1.0f, 1);
  
   // plot.deleteBuffers();
    plot.initPlotWindow();
    plot.initializeBuffer();
    while (!(robot_data->shutdown)) {
        while (robot_data->run) {
            double values[] = {robot_data->robot_velocity[2], 0.5};
            plot.graph_update(values);
            plot.drawGraph();
            plot.swapBuffers();
            sleep(0.5);
        }

        usleep(100000);
        robot_data->run = true;
        std::cout << "outer plot loop" << std::endl;
    }

    plot.deleteBuffers();
    std::cout << "Plot thread shutting down" << std::endl;
    return NULL;

}
