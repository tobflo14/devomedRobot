#include "pid.h"

Pid::Pid(){
    init();
    setParameters(0.0,0.0,0.0);
}

void Pid::init() {
    this->pid.setZero();
    this->past_pid.setZero();
    this->past_error.setZero();
    this->integral.setZero();
    this->derivative.setZero();
}

void Pid::setParameters(double kp, double ki, double kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

Eigen::Vector3d Pid::computePID(Eigen::Vector3d error, double dt){
    //Get new pid only if dt > 0
    if (dt) { 

        integral += error*dt;
        derivative = (error - past_error)/dt;
        //derivative = -(pid - past_pid)/dt;
        //derivative = -pid;

        this->past_pid = pid;
        pid = kp*error + ki*integral + kd*derivative;

        this->past_error = error;
    }

    return pid;

}
