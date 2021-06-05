#include <iostream>
#include <string>

#include <asio.hpp>
#include <asio/buffer.hpp>
#include <asio/io_service.hpp>
#include "global_struct.h"

void* communication_read_thread(void* arg);
void setTrackingVariable(shared_robot_data *robot_data, std::string string);
void receiveMessageHandler(shared_robot_data *robot_data, std::string message);
std::string message = "1-1;2;3;4;5;6\n2-TheRestOftheString\n";
std::string server_ip = "10.22.25.53";

std::string bufferToString(asio::streambuf &buffer) {
    std::string string( (std::istreambuf_iterator<char>(&buffer)), std::istreambuf_iterator<char>() );
    return string;
}
std::string linearization(shared_robot_data *robot_data) {
    std::string string = "1:";
    std::array<double,16> pose = robot_data->robot_state.O_T_EE;
    string += std::to_string(pose[0]);
    for (int i = 1; i < 16; i++){
        string += ";" + std::to_string(pose[i]);
    }
    string += "\n";
    return string;
}
void* communication_thread(void* arg) {
    std::cout << "starting communication thread" << std::endl;
    shared_robot_data *robot_data = (shared_robot_data *)arg;

    asio::error_code ec;

    asio::io_service service;
    asio::ip::tcp::endpoint endpoint(asio::ip::address::from_string(server_ip, ec), 8080);
    asio::ip::tcp::socket socket(service);
    socket.connect(endpoint, ec);

    if (!ec) {
        std::cout << " WriteConnected!" << std::endl;
    } else {
        std::cout << "Failed to connect to address:\n" << ec.message() << std::endl;
    }
    pthread_t readThread;
    int rc = pthread_create(&readThread, NULL, communication_read_thread , &robot_data);
    if (rc) {
            std::cout << "Error: unable to create thread, " << rc << std::endl;
            exit(EXIT_FAILURE);
        }

    while (!robot_data->shutdown) {
        socket.send(asio::buffer(linearization(robot_data)));
        usleep(300000);
    }
    
    socket.close();
    pthread_join(readThread,NULL);
    return NULL;
}

void* communication_read_thread(void* arg) {
    std::cout << "starting read thread" << std::endl;
    shared_robot_data *robot_data = (shared_robot_data *)arg;

    asio::error_code ec;

    asio::io_service service;
    asio::ip::tcp::endpoint endpoint(asio::ip::address::from_string(server_ip, ec), 8081);
    asio::ip::tcp::socket socket(service);
    socket.connect(endpoint, ec);

    if (!ec) {
        std::cout << "Read Connected!" << std::endl;
    } else {
        std::cout << "Failed to connect to address:\n" << ec.message() << std::endl;
    }

    while (!robot_data->shutdown) {
        asio::streambuf read_buffer;
        asio::read_until(socket, read_buffer, '\n');
        //std::cout << bufferToString(read_buffer);
        receiveMessageHandler(robot_data, bufferToString(read_buffer));
    }
    socket.close();

    return NULL;
}

void receiveMessageHandler(shared_robot_data *robot_data, std::string message) {
    if (message[0] == '1') {
        setTrackingVariable(robot_data, message.substr(2,4));
    }
}

void setTrackingVariable(shared_robot_data *robot_data, std::string string) {
    if (string == "true") {
        robot_data->track_position = true;
        std::cout << "Tracking started" << std::endl;
    }
    if (string == "fals") {
        robot_data->track_position = false;
        std::cout << "Tracking stopped" << std::endl;
    }
}