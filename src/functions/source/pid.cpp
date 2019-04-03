#include "pid.h"
#include "global_struct.h"
#include <iostream>

#include <Eigen/Dense>

Pid::Pid(){
    init();
    setParameters(0.0,0.0,0.0);
}

void Pid::init() {
    this->past_error = Eigen::Vector3d(0.0, 0.0, 0.0);
    this->integral = Eigen::Vector3d(0.0, 0.0, 0.0);
    this->derivative = Eigen::Vector3d(0.0, 0.0, 0.0);
}

void Pid::setParameters(double kp, double ki, double kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

Eigen::Vector3d Pid::computePID(Eigen::Vector3d error, double dt){
    //Run only if dt > 0 because derivative can't divide by 0!
    if (dt) { 

        integral += error*dt;
        derivative = (error - past_error)/dt;

        pid = kp*error + ki*integral + kd*derivative;
        
        this->past_error = error;

        return pid;
    }
}
