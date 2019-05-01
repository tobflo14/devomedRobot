#include "utils.hpp"
#include "pd_vec3d.hpp"
#include "pd_quat.hpp"
#include "qp_control.hpp"
#include "solver_thread.hpp"
#include "robot_structs.hpp"
#include <fstream>
#include <vector>


// CVXGEN solver globals
Vars vars;
Params params;
Workspace work;
Settings settings;

void* SolverThread(void* arg)
{
    printf("%sFrom SOLVER Thread: I'm alive!%s\n", KGRN, KNRM);
    std::cout << "   Solver thread ID : " << syscall(SYS_gettid) << std::endl;
    shared_memory *data = (shared_memory *)arg;
    bool first_run = true;

    // Initiating solver()
    set_defaults();
    // Setting number of iterations
    settings.max_iters = 1000;
    settings.verbose = 0;
 
    // Initiating qp_controller_minimal
    qp_init();

    double dT;
    double timer_prev;
    double time;
    std::vector<std::vector<double>> savedata;
    bool hassaved = false;

    // Intiating vectors:
    // v_nxt, w_nxt: is nxt speed to set
    Eigen::Vector3d v_nxt(0.0, 0.0, 0.0);
    Eigen::Vector3d w_nxt(0.0, 0.0, 0.0);
    Eigen::Vector3d w_curr(0.0, 0.0, 0.0);

     // Initiating PD regulator for postion (XYZ), (mass_factor, K_prop, K_der)
    PD_Vec3d pd_position = PD_Vec3d(1, 64, 0.8);
    // Initiating PD regulator for orientation (XYZ), (K_prop, K_der)
    PD_Quat pd_orientation = PD_Quat(3, 1);
    // std::cout << data->run << std::endl;
    while(true)
    {
        while (data->run)
        {
            // Update position with state array
            pd_position.updateMeas(data->robot_position);
            // Update orientation with quaternion from state array
            pd_orientation.updateMeas(data->robot_orientation);

            if (first_run)
            {
                // Reseting all values for new reference movement:
                // Setting new values to position and orientaion
                pd_position.updateRef(data->robot_position, first_run, data);
                pd_position.updateErr(first_run, data);

                pd_orientation.updateRef(data->robot_orientation, first_run, data);
                pd_orientation.updateErr(first_run, data);

                // No longer first run
                
                dT = 0.001;
                timer_prev = data->timer;
                // Move gripper to initial position
                // gripper.move(gripper_state.max_width, 0.1);

                 if ((OPERATING_MODE != VIVE_ORIENT) && (OPERATING_MODE != VIVE_POS_AND_ORIENT) )
                 {
                    pd_orientation.updateRefZero(data->robot_orientation, data);
                    printf("%s   ROT: ZERO orientations%s\n", KYEL, KNRM);
                 }

                first_run = false;
            }
            else
            {
                dT = data->timer - timer_prev;
                timer_prev = data->timer;
                if (dT == 0.0 || dT != dT)
                {
                    dT = 0.001;
                }
                time += dT;
                if (data->timer > 0.0 && data->timer < 5.0) {
                    savedata.push_back(pd_position.returnData());
                }
                if (data->timer > 5.0 && !hassaved) {
                    
                    std::ofstream file_to_write;
                    std::cout << "Saving tracking data..." << std::endl;
                    file_to_write.open("output.csv");
                    if (file_to_write.is_open()) {
                        //file_to_write << "Time;Pos X;Pos Y;Pos Z\n";
                        for (size_t i = 0; i < savedata.size(); i++) {
                            for (size_t j = 0; j < savedata[i].size(); j++) {
                                file_to_write << savedata[i][j] << ";";
                            }
                            file_to_write << "\n";
                        }
                        file_to_write.close();
                        std::cout << "Saved tracking data." << std::endl;
                    }
                    hassaved = true;
                }
                if (time > 0.5) {
                    data->eeffector->move = true;
                    data->eeffector->position(0,0) = 0.01;
                }/*
                if (time > 10.0) {
                    data->eeffector->position(0,0) = 0.0;
                    time = 0.0;
                }*/

                // State Machine for XYZ movement of end-effector
                pd_position.stateMachine(data->eeffector->move);
                switch (pd_position.state)
                {
                case idle:
                    break;

                case initiate_move:
                    pd_position.updateRefZero(data->robot_position, data);
                    break;

                case move:
                    pd_position.updateRef(data->eeffector->position, first_run, data);
                    break;

                default:
                    break;

                }

                // State Machine for ROT movement around end-effector position XYZ
                // Getting current angle velocity of end-effector for use in PD_orientation
                w_curr = data->robot_rotation;
                // w_curr = Eigen::Vector3d(vars.z[3], vars.z[4], vars.z[5]);

                if (OPERATING_MODE == VIVE_ORIENT || OPERATING_MODE == VIVE_POS_AND_ORIENT )
                    {
                        pd_orientation.stateMachine(data->eeffector->rotate);
                        switch (pd_orientation.state)
                        {
                        case idle:
                            break;

                        case initiate_move:
                            pd_orientation.updateRefZero(data->robot_orientation, data);
                            break;

                        case move:
                            pd_orientation.updateRef(data->eeffector->orientation, first_run, data);
                            break;

                        default:
                            break;
                    }

                }

            }

            v_nxt = pd_position.calcVel(dT, data);
            // printf("v_nxt: [%+.4f %+.4f %+.4f]", v_nxt[0], v_nxt[1], v_nxt[2]);
            // printf("dP: [%+.4f %+.4f %+.4f] ",
            //     (data->eeffector->position)[0], (data->eeffector->position)[1],
            //     (data->eeffector->position)[2]);
            // printf("dQ: [%+.4f %+.4f %+.4f %+.4f] \n",
            //     (data->eeffector->orientation).w(), (data->eeffector->orientation).x(),
            //     (data->eeffector->orientation).y(), (data->eeffector->orientation).z());
            // Eigen::Vector3d v_nxt(0.0, 0.0, 0.0);

            w_nxt = pd_orientation.calcVel(data->robot_rotation, data);
            // printf("  w_nxt: [%+.4f %+.4f %+.4f]\n", w_nxt[0], w_nxt[1], w_nxt[2]);
            // w_nxt = Eigen::Vector3d(0.0, 0.0, 0.0);

            // QP solver for cartesian velocity to joint velocity calculation
            qp_params_update(data->jacobian, v_nxt, w_nxt, data->robot_joints->q_prev, data->robot_joints->dq_prev, data->robot_joints->dq_prevprev);
            solve();

            //memcpy(data->z, vars.z, 13 * sizeof(double));
            data->dq = {vars.z[6], vars.z[7], vars.z[8], vars.z[9], vars.z[10], vars.z[11], vars.z[12] };

            // printf("franka_vars: [%+.4f %+.4f %+.4f,%s %+.4f %+.4f %+.4f%s, %+.4f %+.4f %+.4f %+.4f %+.4f %+.4f %+.4f]\n",
            //             vars.z[0], vars.z[1], vars.z[2], KBLU,
            //             vars.z[3], vars.z[4], vars.z[5], KNRM,
            //             vars.z[6], vars.z[7], vars.z[8], vars.z[9], vars.z[10], vars.z[11], vars.z[12]);  



            if (!work.converged) {
                // std::cerr << "Error: Did not converge" << std::endl;
                // printf("%sError:%s Did not converge\n", KRED, KNRM);
                // exit(-1);
            }
            
            // std::cout << data->run << std::endl;
            usleep(0.01*1000000);
        }
        usleep(0.1*1000000);

    }
    // std::cout << data->run << std::endl;
    return NULL;
}