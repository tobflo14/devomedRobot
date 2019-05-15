#include "plot_thread.h"
#include "global_struct.h"

#include "plot3d.h"
#include "plot2d.h"

#include <iostream>
#include <unistd.h>
#include <sys/syscall.h>

void* PlotThread(void* arg) {
    std::cout << "Starting plot thread" << std::endl;
    std::cout << "Plot thread ID : " << syscall(SYS_gettid) << std::endl;

    shared_robot_data *robot_data = (shared_robot_data *)arg;
    Plot2d plot = Plot2d(1000, 2.0f, 2);
    
    
    //Plot3d plot3 = Plot3d();
    
  
   // plot.deleteBuffers();
    plot.initGLFW();
    plot.initPlotWindow();
    plot.initializeBuffer();
    
    //plot3.initWindow();
    //plot3.realize();
    do{
        /*
        double value1 = (double) rand() / (double) RAND_MAX;
        double value2 = rand() / RAND_MAX;
        robot_data->plot1.push_back(Point(value1, value1));
        robot_data->plot2.push_back(Point(value2, 0));
*/
         for (size_t i = 0; i < robot_data->plot1.size(); i++) {
             Point values[] = {robot_data->plot1[i], robot_data->plot2[i]};
            //Point values[] = {robot_data->plot1[0], robot_data->plot2[0]};
            plot.graph_update(values);
        }
        robot_data->plot1.clear();
        robot_data->plot2.clear();


        plot.drawGraph();
        plot.swapBuffers(); 


         /// 3D MODEL   
        //plot3.drawModel();
        //plot3.swapBuffers();
        usleep(16000);
    } // Check if the ESC key was pressed or the window was closed
    while( plot.openWindow );// || plot3.openWindow );


   /*
    while (!(robot_data->shutdown)) {
        while (true) {//robot_data->run) {
            //double values[] = {robot_data->robot_velocity[0], robot_data->robot_acceleration[0]};
      
            //plot.graph_update(values);
      /*      double value1 = (double) rand() / (double) RAND_MAX;
            double value2 = rand() / RAND_MAX;
            robot_data->plot1.push_back(Point(value1, value1));
            robot_data->plot2.push_back(Point(value2, 0));

           // for (size_t i = 0; i < robot_data->plot1.size(); i++) {
               // Point values[] = {robot_data->plot1[i], robot_data->plot2[i]};
               Point values[] = {robot_data->plot1[0], robot_data->plot2[0]};
                plot.graph_update(values);
            //}
            robot_data->plot1.clear();
            robot_data->plot2.clear();
            
          //  plot.drawGraph();
           // plot.drawModel();
            plot3.drawModel();
            plot3.swapBuffers();
            usleep(16000);
        }
        usleep(100000);
    }
    */

   plot.deleteBuffers();
  //  plot3.cleanupModel();
    std::cout << "Plot thread shutting down" << std::endl;
    return NULL;

}
