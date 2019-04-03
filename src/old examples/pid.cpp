#include "pid.h"
#include "global_struct.h"
#include <iostream>

#include <Eigen/Dense>

Pid::Pid(){
    init();
}

void Pid::init() {
    this->past_velocity = Eigen::Vector3d(0.0, 0.0, 0.0);
    this->error = Eigen::Vector3d(0.0, 0.0, 0.0);
    this->past_error = Eigen::Vector3d(0.0, 0.0, 0.0);
    this->integral = Eigen::Vector3d(0.0, 0.0, 0.0);
    this->derivative = Eigen::Vector3d(0.0, 0.0, 0.0);
    this->external_velocity = Eigen::Vector3d(0.0, 0.0, 0.0);
    this->pd = Eigen::Vector3d(0.0, 0.0, 0.0);
}

void Pid::regulateVelocity(double dt, void* arg){
    //Run only if dt > 0 because derivative can't divide by 0!
    if (dt) { 
        //std::cout << dt << std::endl;
        shared_robot_data *robot_data = (shared_robot_data *)arg;

        //Set past values
        this->past_velocity = vel_commanded_previous;
        this->past_error = error;

        this->vel_commanded_previous = robot_data->robot_velocity;
    
        //this->vel_commanded_previous(0,0) = franka::lowpassFilter(dt, robot_data->robot_velocity[0], past_velocity[0], 1);

        this->error = robot_data->setpoint_velocity - vel_commanded_previous;

    /*
        //Run a lowpass filter on the input signal before the pid controller
        error(0,0) = franka::lowpassFilter(dt, error(0,0), past_error(0,0), 1);
        error(1,0) = franka::lowpassFilter(dt, error(1,0), past_error(1,0), 1);
        error(2,0) = franka::lowpassFilter(dt, error(2,0), past_error(2,0), 1);
    */

        //error = std::max(error.norm()-force_limit, 0.0) * error.normalized();
        
        //integral = integral + error*dt;
        integral += error * dt;
        
        //Use negative change of velocity instead of change of error to prevent "derivative kick"
        derivative = -(vel_commanded_previous - past_velocity) / dt;

        //Derivative of standard PID controller
        //derivative = (error - past_error) / dt;

        pd = robot_data->kp * error + robot_data->ki * integral + robot_data->kd * derivative;
        //acc = pd;
        //vel_desired = pd;

        //std::cout << "vel_desired calculated " << vel_desired[2] << std::endl;
        if (pd.norm() < 0.01) {
            robot_data->desired_velocity = {0.0,0.0,0.0};
        } else {
            robot_data->desired_velocity = pd;
        }
        //std::cout << vel_desired[2] << std::endl;
    } // end if
}
