#ifndef QP_CONTROL_HPP_
#define QP_CONTROL_HPP_

#include "stdafx.hpp"
#include "solver.h"

#define GAMMA 	1
#define KAPPA 	5
#define CHI   	1
#define MU		0.01
#define LAMBDA	8
#define DELTA_T 0.001
#define EPSILON_VEL 0.65
#define EPSILON_ACC 0.65

////////////////////////////////////////////////////////////////////////////////
void qp_init();

////////////////////////////////////////////////////////////////////////////////
void qp_params_update( const std::array<double, 42> & jacobian
					,Eigen::Vector3d v_ref
					,Eigen::Vector3d w_ref
					,const std::array<double, 7> & q_prev
					,const std::array<double, 7> & dq_prev
					,const std::array<double, 7> & dq_prevprev);

#endif
