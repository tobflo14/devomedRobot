#ifndef PID_H
#define PID_H

#include <Eigen/Dense>

class Pid {

    public:
        Pid();
        void init();
        Eigen::Vector3d computePID(Eigen::Vector3d error, double dt);
        void setParameters(double kp, double ki, double kd);

    private:
        double kp;
        double ki;
        double kd;
        Eigen::Vector3d pid;
        Eigen::Vector3d past_error;
        Eigen::Vector3d integral;
        Eigen::Vector3d derivative;
        Eigen::Vector3d past_pid;
     
};

#endif