#include "pid.h"

Pid::Pid(){
    
}

void Pid::init(Eigen::VectorXd zero_vector) {
    this->pid = zero_vector;
    this->past_error = zero_vector;
    this->integral = zero_vector;
    this->derivative = zero_vector;
}

void Pid::setParameters(double kp, double ki, double kd) {
    this->kp = kp;
    this->ki = ki;
    this->kd = kd;
}

Eigen::VectorXd Pid::computePID(Eigen::VectorXd error, double dt){
    //Get new pid only if dt > 0
    if (dt) { 

        integral += error*dt;
        derivative = (error - past_error)/dt;
        
        pid = kp*error + ki*integral + kd*derivative;

        this->past_error = error;
    }

    return pid;

}
