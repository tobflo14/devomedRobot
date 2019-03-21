#include "pid.h"
#include "global_struct.h"
#include <iostream>

#include <Eigen/Dense>

Pid::Pid(double Kp, double Ki, double Kd){
    init();
    this->Kp = Kp;
    this->Ki = Ki;
    this->Kd = Kd;
}

void Pid::init() {
    this->past_velocity = Eigen::Vector3d(0.0, 0.0, 0.0);
    this->error = Eigen::Vector3d(0.0, 0.0, 0.0);
    this->integral = Eigen::Vector3d(0.0, 0.0, 0.0);
    this->derivative = Eigen::Vector3d(0.0, 0.0, 0.0);
    this->external_velocity = Eigen::Vector3d(0.0, 0.0, 0.0);
    this->pd = Eigen::Vector3d(0.0, 0.0, 0.0);
    this->time = 0.0;
}

void Pid::regulateVelocity(double dt, void* arg){
  if (dt) { //Run only if dt > 0
    shared_robot_data *robot_data = (shared_robot_data *)arg;

    this->past_velocity = vel_commanded_previous;
    this->vel_commanded_previous = robot_data->robot_velocity;
      
    time += dt;
    if (time > 2.0) {
        external_velocity(2,0) = 0.2;
    }
    if (time > 4.0) {
        external_velocity(2,0) = 0.0;
    }
    if (time > 6.0) {
        external_velocity(2,0) = -0.2;
    }
    if (time > 8.0) {
        external_velocity(2,0) = 0.0;
        time = 0.0;
    }

    this->error = external_velocity - vel_commanded_previous;

/*
      //Y and Z axis needs to be inverted from force input to velocity output for some reason.
      error(1,0) *= -1;
      //error(2,0) *= -1;

      //Calibrate the force input for some reason.
      error(0,0) -= 1.55;
      error(1,0) += 0.0;
      error(2,0) += 1.56;

      //Run a lowpass filter on the input signal before the pid controller
      error(0,0) = franka::lowpassFilter(dt, error(0,0), past_error(0,0), 1);
      error(1,0) = franka::lowpassFilter(dt, error(1,0), past_error(1,0), 1);
      error(2,0) = franka::lowpassFilter(dt, error(2,0), past_error(2,0), 1);
*/

      //error = std::max(error.norm()-force_limit, 0.0) * error.normalized();
      
      //integral = integral + error*dt;
      integral += error * dt;
      
      derivative = -(vel_commanded_previous - past_velocity) / dt;

      pd = Kp * error + Ki * integral + Kd * derivative;
      //acc = pd;
      //vel_desired = pd;

      //std::cout << "vel_desired calculated " << vel_desired[2] << std::endl;

      robot_data->desired_velocity = pd;
      //std::cout << vel_desired[2] << std::endl;
  } // end if
}
