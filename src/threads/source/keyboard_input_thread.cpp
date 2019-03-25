#include "keyboard_input_thread.h"
#include "global_struct.h"

#include <iostream>
#include <fstream>

void* KeyboardInputThread(void* arg) {
    std::cout << "starting keyboard input thread" << std::endl;
    shared_robot_data *robot_data = (shared_robot_data *)arg;

    string inputString;
    std::cout << "Waiting for user input..." << std::endl;
    std::cin >> inputString;
    std::cout << "input is: " << inputString << std::endl;
    if (inputString == "q") {
        std::cout << "Shutdown true" << std::endl;
        robot_data->shutdown = true;
    }

    return NULL;
}