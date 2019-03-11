// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <array>
#include <atomic>
#include <cmath>
#include <functional>
#include <fstream>
#include <iterator>
#include <mutex>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>

#include <thread>
//#include <gnuplot-iostream/gnuplot-iostream.h>
#include <iostream>

#include "examples_common.h"

#include "functions.h"


int main(int argc, char** argv) {
  // Check whether the required arguments were passed.
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }
  
  double vel_current[3] = {0.0};
  double vel_desired[3] = {0.0};
  double vel_desired_previous = 0.0;
  //double angle = 0.0;
  double time = 0.0;
  double zvalue = 0.0;
  double lastvalue = 0.0;
  unsigned long plotLength = 5000; //Max number of plotted points in a graph

double past_error[3] = {0.0};
double acc = 0.0;
double acc_previous = 0.0;
double jerk_limit = 400;
double Kp = 0.2;
double Ki = 0.4;
double Kd = -0.001;
double integral = 0.0;

  std::vector<Point> plot_Data1;
  std::vector<Point> plot_Data2;

  struct {
      std::mutex mutex;
      bool has_data;
      double printData;
      std::vector<Point> plotData1;
      std::vector<Point> simplifiedData1;
      std::vector<Point> simplifiedData2;
      std::vector<Point> plotData2;
  } plot_input{};

  std::atomic_bool running{true};
  std::atomic_bool started{false};


  // Start print thread.
  std::thread print_thread([&Kp, &Ki, &Kd, &plot_input, &running, plotLength]() {

    Gnuplot gp;
    gp << "set terminal wxt size 800, 400\n";
    gp << "set yrange[-1:1]\n";
    while (running) {
      /*if (started) {
        std::cout << "Change Kp to ";
        std::cin >> Kp;
        std::cout << "Change Ki to ";
        std::cin >> Ki;
        std::cout << "Change Kd to ";
        std::cin >> Kd;
      }*/
      //std::this_thread::sleep_for(std::chrono::milliseconds(static_cast<int>((1.0 / 25.0 * 1000.0))));
      // Try to lock data to avoid read write collisions.
      if (plot_input.mutex.try_lock()) {
        if (plot_input.has_data) {

          if (plot_input.plotData2.empty()) {
            gp << "plot '-' with lines title 'plotData1'\n";
            gp.send1d(plot_input.plotData1);
          } else {
            gp << "plot '-' with lines title 'plotData1', '-' with lines title 'plotData2'\n";
            gp.send1d(plot_input.plotData1);
            gp.send1d(plot_input.plotData2);
          }
          gp.flush();
          /*
            std::cout << plot_input.printData << std::endl;
          //std::cout << std::fixed << std::setprecision(3);
          if (plot_input.printData < 0) {
            std::cout << plot_input.printData << std::endl;
          } else {
            std::cout << "+" << plot_input.printData << std::endl;
          }
          */
          plot_input.has_data = false;
        }
        plot_input.mutex.unlock();
      }
    }
  });

  try {
    // Connect to robot.
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);
    
    // First move the robot to a suitable joint configuration
    //std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    std::array<double, 7> q_goal = {{0, M_PI_4, 0, -2 * M_PI_4, 0, 3 * M_PI_4, M_PI_4}};
    MotionGenerator motion_generator(0.5, q_goal);
    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl;
    started = true;

    // Load the kinematics and dynamics model.
    franka::Model model = robot.loadModel();
   
    //std::array<double, 16> initial_pose;
    std::array<double, 6> initial_pose;

    // Define callback function to send Cartesian pose goals to get inverse kinematics solved.
    auto cartesian_pose_callback = [=, &time, &integral, &Kp, &Ki, &Kd, &acc, &acc_previous, &jerk_limit, &past_error, &lastvalue, &plot_input, &plot_Data1, &plot_Data2, &vel_current, &vel_desired, &vel_desired_previous, &running, &zvalue, &initial_pose]
      (const franka::RobotState& robot_state, franka::Duration period) -> franka::CartesianVelocities {
      // Update time.
      time += period.toSec();

      if (time == 0.0) {
        // Read the initial pose to start the motion from in the first time step.
        //initial_pose = robot_state.O_T_EE_c;
        initial_pose = robot_state.O_dP_EE_c;
      }

      //read force in z-direction on end effector
      double zforce = robot_state.K_F_ext_hat_K[2];
      /*
      double error = zforce - 0.0;
      double delta_error = error - past_error;
      past_error = error;
      integral += error * period.toSec();

      double pd = Kp * error + Ki * integral + Kd * (delta_error / (period.toSec()));
      acc_previous = acc;
      acc = pd;
      /*
      acc = std::fmax(acc, -3);
      acc = std::fmin(acc, 3);

      double jerk = (acc - acc_previous)/period.toSec();
      if (jerk > jerk_limit) {
        acc = acc_previous + jerk_limit*period.toSec();
      } else if (jerk < -jerk_limit) {
        acc = acc_previous - jerk_limit*period.toSec();
      }
*//*
      vel_desired_previous = vel_desired;
      vel_desired = vel_desired_previous + acc * period.toSec();
      //double delta_v = vel_desired - vel_desired_previous;
      vel_desired==vel_desired? vel_desired = vel_desired:
        vel_desired = vel_desired_previous;
*/
      franka::CartesianVelocities velocity_desired = initial_pose;
      velocity_desired.O_dP_EE[0] = velocity_desired.O_dP_EE[1] = velocity_desired.O_dP_EE[3] = velocity_desired.O_dP_EE[4] = velocity_desired.O_dP_EE[5] = 0;
      velocity_desired.O_dP_EE[2] = 0;//vel_desired;
      /*
      lastvalue = zforce;//franka::lowpassFilter(0.001, zforce, lastvalue, 100.0);
      //Set desired velocity based on force
      if (lastvalue > 0.8) {
        vel_desired = vel_max;
      } else if (lastvalue < -0.8) {
        vel_desired = -vel_max;
      } else {
        vel_desired = 0.0;
      }

      //calculate velocities with acceleration
      if (vel_current < vel_desired) {
        vel_current += period.toSec() * std::fabs(vel_max / acceleration_time);
      }
      if (vel_current > vel_desired) {
        vel_current -= period.toSec() * std::fabs(vel_max / acceleration_time);
      }
      //limit the velocity if larger than max
      vel_current = std::fmax(vel_current, -vel_max);
      vel_current = std::fmin(vel_current, vel_max);

      //Calculate position from velocity
      zvalue += period.toSec() * vel_current;

      franka::CartesianPose pose_desired = initial_pose;
      pose_desired.O_T_EE[14] += zvalue;
      */
/*
      // Stop running after max time
      if (time >= run_time + acceleration_time) {
        running = false;
        return franka::MotionFinished(pose_desired);
      }
*/
      plot_Data1.push_back(Point(time, robot_state.tau_J[0]));

      if (plot_input.mutex.try_lock()) {
        plot_input.has_data = true;
        plot_input.plotData1 = plot_Data1;
        plot_input.mutex.unlock();
      }

      return velocity_desired;

      //return pose_desired;

    };
    // Start real-time control loop.
    //robot.control(impedance_control_callback, cartesian_pose_callback);
    robot.control(cartesian_pose_callback);
    
    
  } catch (const franka::Exception& ex) {
    running = false;
    std::cerr << ex.what() << std::endl;
  }

  if (print_thread.joinable()) {
    print_thread.join();
  }

  return 0;
}
