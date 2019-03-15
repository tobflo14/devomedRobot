#ifndef PID_H
#define PID_H

#include <Eigen/Dense>

class Pid {

    Eigen::Vector3d vel_desired;

    public:
        Pid(double Kp, double Ki, double Kd);
        void init();
        void regulateVelocity(double dt, void* arg);

    private:
        double Kp;
        double Ki;
        double Kd;
        double jerk_limit;
        double acceleration_limit;
        double force_limit;
        double time;
        Eigen::Vector3d vel_desired_previous;
        Eigen::Vector3d vel_commanded_previous;
        Eigen::Vector3d acc_commanded_previous;
        Eigen::Vector3d acc;
        Eigen::Vector3d acc_previous;
        Eigen::Vector3d past_error;
        Eigen::Vector3d error;
        Eigen::Vector3d integral;
        Eigen::Vector3d derivative;
        Eigen::Vector3d external_velocity;
        Eigen::Vector3d jerk;
        Eigen::Vector3d pd;
        //double dt;
        

};

#endif