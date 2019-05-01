#ifndef PD_VEC3D_HPP_
#define PD_VEC3D_HPP_

#include "pd.hpp"
#include <vector>


class PD_Vec3d: public PD
{
    public:
        controller_state state;
        
        PD_Vec3d(double mass_factor, double pos_gain, double der_gain);
        void init();
        void updateMeas(Eigen::Vector3d meas);
        void updateRef(Eigen::Vector3d ref, bool first_run, void* arg);
        void updateRefZero(Eigen::Vector3d ref_zero, void* arg);
        void updateErr(bool first_run, void* arg);
        std::vector<double> returnData();
        Eigen::Vector3d calcVel(double dT, void* arg);
        void stateMachine(bool command);


    private:
        double K;
        double Kp;
        double Kd;
        Eigen::Vector3d meas;
        Eigen::Vector3d ref;
        Eigen::Vector3d vel;
        Eigen::Vector3d vel_prev;
        Eigen::Vector3d err;
        Eigen::Vector3d err_prev;
        Eigen::Vector3d err_dot;
        Eigen::Vector3d force;
        Eigen::Vector3d ref_zero;
        Eigen::Vector3d acc;
};

#endif
