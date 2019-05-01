#include <iostream>
#include <franka/robot.h>
#include <franka/model.h>
#include <eigen3/Eigen/Core>
#include <vector>

#include "qp_control.hpp"

static const double DDDq_MAX_[7] 	= {7500.0, 3750.0, 5000.0, 6250.0, 7500.0, 10000.0, 10000.0};
// static const double DDq_MAX_[7]   	= {16.5, 8.25, 13.75, 13.75, 16.5, 22, 22};
static const double DDq_MAX_[7]   	= {15.0, 7.5, 10.0, 12.5, 15.0, 20.0, 20.0};
static const double Dq_MAX_SOFT_[7] = {2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100};
static const double q_MAX_SOFT_[7] 	= {2.8973, 1.7628, 2.8973, 0.0175, 2.8973, 3.77525, 2.8973};
static const double q_MIN_SOFT_[7]  = {-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973};


////////////////////////////////////////////////////////////////////////////////
void qp_init() 
{

	// Fill Q with ZERO
	for (int i = 0; i < 13; i++) 
	{
		for (int j = 0; j < 13; j++)
		{
			if (i == j)
			{
				if (i < 3)		params.Q[j * 13 + i] = GAMMA;
				else if (i < 6)	params.Q[j * 13 + i] = KAPPA;
				else 			params.Q[j * 13 + i] = MU;
			}
		}
	}

	// // Weighting of movement in spesific joints
	 // // params.Q[ j * 13 + i] = MU * 1.0 -- gives std control
	 // // 6's are base, and runs out
	 // params.Q[ 6 * 13 +  6] = MU * 4.0;  // Base (rot)
	 // params.Q[ 7 * 13 +  7] = MU * 3.0;  // 2nd joint (joi)
	 // params.Q[ 8 * 13 +  8] = MU * 1.0;  // 3rd joint (rot)
	 // params.Q[ 9 * 13 +  9] = MU * 4.0;  // Elbow (joi)
	 // params.Q[10 * 13 + 10] = MU * 1.0;  // Elbow rotation (rot)
	 // params.Q[11 * 13 + 11] = MU * 1.0;  // 1st Hand joint (joi)
	 // params.Q[12 * 13 + 12] = MU * 5.0;  // Hand rotation (rot)

	// Fill Aeq
	for (int i = 0; i < 6; i++)
	{
		for (int j = 0; j < 13; j++)
		{
			int idx = i + j * 6;
			if (j < 6)
			{
				if (i == j) params.Aeq[idx] = 1.0;
				else 		params.Aeq[idx] = 0.0;
			}
		}
	}

	// fill Aineq
	for (int i = 0; i < 14 * 13; i++)  params.Aineq[i] = 0;

	params.Aineq[84] = 1;
    params.Aineq[99] = 1;
	params.Aineq[114] = 1;
	params.Aineq[129] = 1;
	params.Aineq[144] = 1;
	params.Aineq[159] = 1;
	params.Aineq[174] = 1;

	params.Aineq[91] = -1;
	params.Aineq[106] = -1;
	params.Aineq[121] = -1;
	params.Aineq[136] = -1;
	params.Aineq[151] = -1;
	params.Aineq[166] = -1;
	params.Aineq[181] = -1;
}

////////////////////////////////////////////////////////////////////////////////
void qp_params_update(
	const std::array<double, 42> & jacobian, Eigen::Vector3d v_ref,
	Eigen::Vector3d w_ref, const std::array<double, 7> & q_prev,
	const std::array<double, 7> & dq_prev,
	const std::array<double, 7> & dq_prevprev)
{
	
	// fill q0
	for (int i = 0; i < 13; i++) {

		if (i < 3)
			params.q0[i] = -2 * v_ref[i] * GAMMA;
		else if (i < 6)
			params.q0[i] = -2 * w_ref[i - 3] * KAPPA;
		else
		{
			params.q0[i] = 	CHI * 1000 * (  (-LAMBDA * DELTA_T)	* exp( -LAMBDA * (q_prev[i-6] - q_MIN_SOFT_[i - 6]) )
							+ (LAMBDA * DELTA_T)* exp( -LAMBDA * (-q_prev[i - 6] + q_MAX_SOFT_[i - 6]) )  );
		}
	}

	// update Aeq
	for (int i = 36; i < 78; i++) params.Aeq[i] = -jacobian[i - 36];	

	// fill b
	std::vector<double> temp_max;
	std::vector<double> temp_min;

	for (int i = 0; i < 7; i++) {
		
		temp_max.clear();
		temp_max.push_back((q_MAX_SOFT_[i] - q_prev[i])/DELTA_T); // position
		temp_max.push_back(EPSILON_VEL * Dq_MAX_SOFT_[i]); // velocity
		temp_max.push_back(dq_prev[i] + DELTA_T * EPSILON_ACC * DDq_MAX_[i]); // acceleration
		temp_max.push_back(pow(DELTA_T, 2) * EPSILON_ACC * DDDq_MAX_[i] + 2 *
			dq_prev[i] - dq_prevprev[i]); // jerk

		temp_min.clear();
		temp_min.push_back((q_MIN_SOFT_[i] - q_prev[i])/DELTA_T); // position
		temp_min.push_back(-EPSILON_VEL * Dq_MAX_SOFT_[i]); // velocity
		temp_min.push_back(dq_prev[i] - DELTA_T * EPSILON_ACC * DDq_MAX_[i]); // acceleration
		temp_min.push_back(-pow(DELTA_T, 2) * EPSILON_ACC * DDDq_MAX_[i] +
			2 * dq_prev[i] - dq_prevprev[i]); // jerk

		// b
		params.b[i] = *min_element(temp_max.begin(), temp_max.end());
		params.b[i+7] = -(*max_element(temp_min.begin(), temp_min.end()));
	}
}
