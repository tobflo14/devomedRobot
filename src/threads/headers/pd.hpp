#ifndef PD_HPP_
#define PD_HPP_

#include "common.hpp"
#include "stdafx.hpp"
#include "robot_structs.hpp"

#define MAX_VEL   1.0
#define REDUCED_MAX_VEL 0.8
#define MAX_ACCEL 0.2


typedef enum {
    idle,
    initiate_move,
    move,
    collision_avoidance
}controller_state;


class PD
{
    public:
        controller_state state;
        void init();
        void stateMachine(bool command);


    private:
        double K;
        double Kp;
        double Kd;
};

#endif
