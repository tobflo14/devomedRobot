#ifndef PD_QUAT_HPP_
#define PD_QUAT_HPP_

#include "pd.hpp"




class PD_Quat: public PD
{
    public:
        controller_state state;
        
        PD_Quat(double pos_gain, double der_gain);
        void init();
        void updateMeas(Eigen::Quaterniond meas);
        void updateRef(Eigen::Quaterniond delta, bool first_run, void* arg);
        void updateRefZero(Eigen::Quaterniond ref_zero, void* arg);
        void updateErr(bool first_run, void* arg);
        Eigen::Vector3d calcVel(Eigen::Vector3d w_curr, void* arg);
        void stateMachine(bool command);


    private:
        Eigen::Matrix3d Kp;
        Eigen::Matrix3d Kd;

        Eigen::Quaterniond meas;
        
        Eigen::Quaterniond err_q;
        Eigen::Quaterniond ref_q;
        Eigen::Quaterniond ref_zero_q;

};

#endif
