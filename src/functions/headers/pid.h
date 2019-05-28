#ifndef PID_H
#define PID_H

#include <Eigen/Dense>

class Pid {

    public:
        Pid();
        void init(Eigen::VectorXd zero_vector);
        Eigen::VectorXd computePID(Eigen::VectorXd error, double dt);
        void setParameters(double kp, double ki, double kd);

    private:
        double kp;
        double ki;
        double kd;
        Eigen::VectorXd pid;
        Eigen::VectorXd past_error;
        Eigen::VectorXd integral;
        Eigen::VectorXd derivative;
     
};

#endif