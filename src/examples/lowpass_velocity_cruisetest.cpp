// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <array>
#include <atomic>
#include <cmath>
#include <functional>
#include <fstream>
#include <iterator>
#include <mutex>

#include <Eigen/Dense>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>

#include <thread>
#include <iostream>
#include <chrono>

//Headers for arduino input
#include <stdio.h> // standard input / output functions
#include <string.h> // string function definitions
#include <unistd.h> // UNIX standard function definitions
#include <fcntl.h> // File control definitions

#include <stdlib.h>
#include <errno.h>
#include <termios.h>

#include "examples_common.h"

#include "plot2dTest.h"
#include "pid.h"

int main(int argc, char** argv) {
  // Check whether the required arguments were passed.
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }
  //Open an output file to write data to
  std::ofstream outputfile;
  outputfile.open("output.csv");

  if (outputfile.is_open()) {
    std::cout << "output.csv is opened" << std::endl;
  }
  

  std::vector<double> pid_constants = {0.0, 0.0, 0.0};
  std::vector<double> arduino_input;

  std::atomic_bool running{true};
  std::atomic_bool started{false};

  struct {
    std::mutex mutex;
    bool has_data;
    std::vector<double> data;
    //double data;
    std::vector<double> data2;
  } print_data{};

  std::vector<std::vector<double>> velocity_array;
  double last_force = 0.0;

  //Initialize desired forces and set them to zero
 
  Eigen::VectorXd vel_desired(6);
  vel_desired.setZero();

  Plot2d plot = Plot2d(1000, 5);
  Pid pid = Pid(0.5, 0, 0);

  // Start print thread.
  std::thread print_thread([&print_data, &arduino_input, &running, &pid_constants, &started, &plot, &pid]() {
    
    
    int USB = open( "/dev/ttyUSB0", O_RDWR| O_NOCTTY );

    if (USB < 0) {
      std::cout << "Error while opening device: " << strerror(errno) << std::endl;
    } else {
      struct termios tty;
      struct termios tty_old;
      memset (&tty, 0, sizeof tty);

    if ( tcgetattr(USB, &tty) != 0) {
      std::cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
    } else {

        tty_old = tty;

        cfsetospeed (&tty, (speed_t)B9600);
        cfsetispeed (&tty, (speed_t)B9600);

        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;

        tty.c_cflag &= ~CRTSCTS;
        tty.c_cc[VMIN] = 1;
        tty.c_cc[VTIME] = 5;
        tty.c_cflag |= CREAD | CLOCAL;

        cfmakeraw(&tty);

        tcflush( USB, TCIFLUSH);
        if (tcsetattr(USB, TCSANOW, &tty) != 0) {
          std::cout << "Error " << errno << " from tcsetattr" << std::endl;
        } else {
          std::cout << "Arduino Setup complete" << std::endl;
          
          //plot.initPlotWindow();
          //plot.initializeBuffer(1);

          struct timespec ts, last_ts;
	        timespec_get(&last_ts, TIME_UTC);
          double dt;
        
          
          while (running) {
            if (started) {
              timespec_get(&ts, TIME_UTC);
		          dt = ts.tv_sec - last_ts.tv_sec + (ts.tv_nsec - last_ts.tv_nsec) / 1E9;

		          if (dt <= 0.01) continue; // limit the program to 20fps
		          last_ts = ts;

             

              int n = 0,
                  spot = 0;
              char buf = '\0';
              char response[1024];
              memset(response, '\0', sizeof response);

              do {
                  n = read( USB, &buf, 1 );
                  sprintf( &response[spot], "%c", buf );
                  spot += n;
              } while( buf != '\r' && n > 0);

              if (n < 0) {
                std::cout << "Error reading: " << strerror(errno) << std::endl;
              } else if (n == 0) {
                std::cout << "Read nothing!" << std::endl;
              } else {//} if (response[0] != '\0' && response[0] != '\n') { //Successful response
                std::string string_response = std::string(response);
                double Kp = stod(string_response.substr(0, string_response.find(",")));
                Kp = 4;//(Kp/1023)*4;
                string_response.erase(0, string_response.find(",")+1);
                double Ki = stod(string_response.substr(0, string_response.find(",")));
                Ki = 0;//(Ki/1023)*0;
                string_response.erase(0, string_response.find(",")+1);
                double Kd = stod(string_response);
                Kd = 0; //0.0000001;//(Kd/1023)*0;
                //std::cout << "Kd:";
                pid_constants[0] = Kp;
                pid_constants[1] = Ki;
                pid_constants[2] = Kd;
               // std::cout << pid_constants[2] << std::endl;
                
                if (print_data.mutex.try_lock()) {
                  if (print_data.has_data) {
                    //std::cout << print_data.data.size() << std::endl;
                   // _dataTail = graph_update1(print_data.data.back(), print_data.data2, _dataTail, vertexDataBuffer);
                   // for (uint i = 0; i < print_data.data.size(); i++) {
                      
                     // std::cout << print_data.data2[i] << std::endl;
                    // plot.graph_update(print_data.data[i]);
                  //  }
                    std::cout << "vel_desired : " << print_data.data.back() << std::endl;
                   // std::cout << "vel_desired2: " << print_data.data2.back() << std::endl;
                    //plot.drawGraph();
                    //DrawGrid(vertexArrayGrid);
                    print_data.data.clear();
                    print_data.data2.clear();
                    print_data.has_data = false;
                  }
                  else {
                     //plot.graph_update(0);
                  }
                  print_data.mutex.unlock();
                }
                else {
                  //plot.graph_update(0);
                  //plot.drawGraph();
                }
                // Swap buffers
                //plot.swapBuffers();
              }
            }
          } // end while running
        }
      }
    }
    auto timenow = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    //plot.saveScreenshotToFile(std::strcat(ctime(&timenow), ".tga"), 1024, 768);
    //plot.deleteBuffers();
  });

  try {
    // Connect to robot.
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);
    
    // First move the robot to a suitable joint configuration
    //std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    std::array<double, 7> q_goal = {{0, M_PI_4, 0, -2 * M_PI_4, 0, 3 * M_PI_4, M_PI_4}};
    MotionGenerator motion_generator(0.5, q_goal);
    /*
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    */
    robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl;
    started = true;
    
    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    // Load the kinematics and dynamics model.
    franka::Model model = robot.loadModel();
    std::vector<double> inloop_data;
    std::vector<double> inloop_data2;
    // Define callback function to send Cartesian pose goals to get inverse kinematics solved.
    auto cartesian_pose_callback = [=, &inloop_data, &inloop_data2, &print_data, &running, &pid, &vel_desired]
      (const franka::RobotState& robot_state, franka::Duration period) -> franka::CartesianVelocities {

      double dt = 0.001;//period.toSec();
      //Guess a dt if no dt, to make sure we don't divide by 0
      if (!dt) { 
        dt = 0.001;
      }
      //Get forces on the flange
     // Eigen::Map<const Eigen::Matrix<double, 6, 1>> force_input(robot_state.K_F_ext_hat_K.data());

      //Get last commanded velocity (commanded or actually performed?)
      Eigen::Map<const Eigen::Matrix<double, 6, 1>> vel_commanded_previous(robot_state.O_dP_EE_c.data());
      Eigen::Map<const Eigen::Matrix<double, 6, 1>> acc_commanded_previous(robot_state.O_ddP_EE_c.data());

      vel_desired = pid.regulateVelocity(vel_commanded_previous, acc_commanded_previous);
     
      //franka::CartesianVelocities velocity_desired = {{vel_desired[0], vel_desired[1], vel_desired[2], 0.0, 0.0, 0.0}};
      //franka::CartesianVelocities velocity_desired = {{0.0, 0.0, 0.0, -vel_desired[3], vel_desired[4], vel_desired[5]}};
      franka::CartesianVelocities velocity_desired = {{0.0, 0.0, vel_desired[2], 0.0, 0.0, 0.0}};
      //franka::CartesianVelocities velocity_desired = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  

      inloop_data.push_back(vel_desired[2]);
     // inloop_data2.push_back(vel_desired[2]);

      //inloop_data2.push_back(pd[2]*10);
      //inloop_data.push_back(pid_constants[2]*derivative[2]);

        if (print_data.mutex.try_lock()) {
          print_data.has_data = true;
          for (uint i=0; i < inloop_data.size(); i++) {
            print_data.data.push_back(inloop_data[i]);
           // print_data.data2.push_back(inloop_data2[i]);
          } 
          inloop_data.clear();
          //inloop_data2.clear();
          print_data.mutex.unlock();
        }
    
      return velocity_desired;
    };
    // Start real-time control loop.
    robot.control(cartesian_pose_callback);
    
  } catch (const franka::Exception& ex) {
   // saveScreenshotToFile("2019-02-05test.tga", 1024, 768);
    running = false;
    //Print data to output file
    /*
    outputfile << "vel,acc\n";
    for (int i = 0; i < velocity_array.size(); i++) {
      outputfile << velocity_array.at(i)[0] << "," << velocity_array.at(i)[1] << "\n";
    }
    std::cout << "Printed data to file" << std::endl;
    */
    std::cerr << ex.what() << std::endl;
  }
  catch (...) {
    std::exception_ptr p = std::current_exception();
    std::clog <<(p ? p.__cxa_exception_type()->name() : "null") << std::endl;
  }

  if (print_thread.joinable()) {
    print_thread.join();
  }
  outputfile.close();
  return 0;
}