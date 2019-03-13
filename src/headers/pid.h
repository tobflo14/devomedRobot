#ifndef PID_H
#define PID_H

#include <Eigen/Dense>

class Pid {

    Eigen::VectorXd vel_desired;

    public:
        Pid(double Kp, double Ki, double Kd);
        void init();
        Eigen::VectorXd regulateVelocity(Eigen::VectorXd vel_commanded_previous, Eigen::VectorXd acc_commanded_previous);

    private:
        double Kp;
        double Ki;
        double Kd;
        double jerk_limit;
        double acceleration_limit;
        double force_limit;
        double time;
        Eigen::VectorXd vel_desired_previous;
        Eigen::VectorXd vel_commanded_previous;
        Eigen::VectorXd acc_commanded_previous;
        Eigen::VectorXd acc;
        Eigen::VectorXd acc_previous;
        Eigen::VectorXd past_error;
        Eigen::VectorXd error;
        Eigen::VectorXd integral;
        Eigen::VectorXd derivative;
        Eigen::VectorXd external_velocity;
        Eigen::VectorXd jerk;
        Eigen::VectorXd pd;
        double dt;
        

};

#endif