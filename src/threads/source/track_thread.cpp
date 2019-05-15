#include "track_thread.h"
#include "global_struct.h"

#include <iostream>
#include <fstream>
#include <unistd.h>

void* TrackThread(void* arg) {
    std::cout << "starting tracking thread" << std::endl;
    shared_robot_data *robot_data = (shared_robot_data *)arg;

    bool tracking_started = false;
    bool finished = false;

    while (!finished) {
        while (robot_data->run) {

            if (robot_data->track_position) {
                tracking_started = true;
                std::cout << "Tracking started" << std::endl;
            } else if (tracking_started) {
                //Save tracking data.
                std::ofstream file_to_write;
                std::cout << "Saving tracking data..." << std::endl;
                try {
                    file_to_write.open("output.csv", std::ofstream::trunc);
                    //file_to_write << "Time;Pos X;Pos Y;Pos Z\n";
                    for (size_t i = 0; i < robot_data->tracking_data.size(); i += 10) {
                        file_to_write << robot_data->tracking_data[i][0] << ";";
                        file_to_write << robot_data->tracking_data[i][1] << ";";
                        file_to_write << robot_data->tracking_data[i][2] << ";\n";
                    }
                    file_to_write.close();
                } catch (std::system_error& e) {
                    std::cerr << "Error opening file: " << e.what() << std::endl;
                }
                std::cout << "Tracking data saved." << std::endl;
                tracking_started = false;
            }
            usleep(100000);
        }

        //If we are shutting down without robot running, finish this thread
        if (robot_data->shutdown) {
            finished = true;
        }
        usleep(100000);
    }

    return NULL;
}