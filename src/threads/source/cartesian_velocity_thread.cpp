#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/model.h>
#include <sys/syscall.h>
#include <sys/types.h>
#include <signal.h>

#include "utils.hpp"
#include "cartesian_velocity_thread.hpp"
#include "robot_structs.hpp"

int retries = 100;

///////////////////////////////////////////////////////////////////////////////
void* CartesianVelocityThread(void* arg)
{
	std::cout << "   Cart_vel (ROBOT) thread ID : " << syscall(SYS_gettid) << std::endl;

	do
	{
		shared_memory *data = (shared_memory *)arg;
		franka::Robot robot(ROBOT_IP);
	
		//printf("%sFrom Cart_vel (ROBOT) Thread: I'm alive!%s\n", KGRN, KNRM);
		try{
			// Load robot model
			franka::Model model = robot.loadModel();
			franka::JointVelocities output = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

			// Set additional parameters always before the control loop, NEVER in the
			// control loop! Set the joint impedance.
			try{
				robot.setJointImpedance({ {3000, 3000, 3000, 2500, 2500, 2000, 2000} });
			}
			catch (const franka::ControlException &e){
				robot.automaticErrorRecovery();
			}

			// Set the collision behavior.
			robot_collision_behavior rcb;       //  Constructor sets correct initial values
			robot.setCollisionBehavior(rcb.lower_torque_thresholds_acceleration, rcb.upper_torque_thresholds_acceleration,
				rcb.lower_torque_thresholds_nominal, rcb.upper_torque_thresholds_nominal,
				rcb.lower_force_thresholds_acceleration, rcb.upper_force_thresholds_acceleration,
				rcb.lower_force_thresholds_nominal, rcb.upper_force_thresholds_nominal);

			// Initiating robot.control parameters
			bool first_run = true;

			robot.control([=, &model, &data ,&output ,&first_run](const franka::RobotState & state,
					franka::Duration time_step) -> franka::JointVelocities
			{
				double dT = time_step.toSec();
				if (dT == 0.0 || dT != dT)
				{
					dT = 0.001;
				}
				data->timer += dT;

				data->jacobian = model.zeroJacobian(franka::Frame::kEndEffector, state);


				// Update position with state array
				data->robot_position = get_position(state);
				// Update orientation with quaternion from state array
				data->robot_orientation = get_orientation(state);
				// GET W_CURR FROM ROBOT STATE
				data->robot_rotation = get_rotation(state);
				// GET VELOCITY FROM ROBOT STATE
				data->robot_velocity = get_velocity(state);

				// CONVERGING PROBLEM SOLVER():
					// We are getting convergeing problems around ZERO. The robot should not break
					// just stopp moving.

				// CONFERMATION ON VIVE:
					// We should add a message for revivceing and not reciving from VIVE

				
				// Eigen::Quaterniond curr_robot_qua = get_orientation(state);
				 // printf("curr_robot_qau . [%+.3f %+.3f %+.3f %+.3f]\n",
				 //         curr_robot_qua.w(), curr_robot_qua.x(), curr_robot_qua.y(), curr_robot_qua.z() );
			
				// Update variables for memory
				data->robot_joints->dq_prevprev = data->robot_joints->dq_prev;
				data->robot_joints->dq_prev = state.dq_d;
				data->robot_joints->q_prev = state.q;

				// printf("franka_vars robot side: [%+.4f %+.4f %+.4f,%s %+.4f %+.4f %+.4f%s, %+.4f %+.4f %+.4f %+.4f %+.4f %+.4f %+.4f]\n",
				 // data->z[0], data->z[1], data->z[2], KBLU,
				 // data->z[3], data->z[4], data->z[5], KNRM,
				 // data->z[6], data->z[7], data->z[8], data->z[9], data->z[10], data->z[11], data->z[12]);  

				// Command robot joint velocity
				if (first_run){
					data->run = true;
					output = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };	
					first_run = false;
					std::cout << "Started running" << std::endl;
				}
				else{
					//output = { data->z[6], data->z[7], data->z[8], data->z[9], data->z[10], data->z[11], data->z[12] };	
					//output = { data->z[6], data->z[7], data->z[8], data->z[9], 0.0, 0.0, 0.0 };
					output = data->dq;	
				}
				return output;

			});

			printf("%sFrom Cart_vel (ROBOT) Thread is dead%s\n", KRED, KNRM);
		}

		catch (const franka::Exception& e)
		{
			data->run = false;
			std::cout << e.what() << std::endl;
			std::cout << "Running error recovery..." << std::endl;

			try{
				robot.automaticErrorRecovery();
			}
			catch(const franka::Exception& er){
				std::cout << KRED << "Recovery failed: " << KNRM << er.what() << std::endl;
				exit(EXIT_FAILURE);
				return nullptr;
			}
			
			retries--;
			sleep(5);
		}

	} while (retries > 0);
    return NULL;
}
