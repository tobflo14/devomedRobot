#include "robot_loop_thread.h"
#include "global_struct.h"
#include "functions.h"
#include "examples_common.h"
#include "pid.h"

#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <unistd.h>
#include <sys/syscall.h>

#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/robot.h>
#include <franka/robot_state.h>
#include <franka/exception.h>

int retries = 100;

void* RobotLoopThread(void* arg) {
    std::cout << "Starting robot thread" << std::endl;
    std::cout << "robot loop thread ID : " << syscall(SYS_gettid) << std::endl;
    Eigen::Vector3d vel_desired;
    Eigen::Vector3d vel_desired_ang;
    Eigen::Vector3d acc;
    Eigen::Vector3d acc_ang;
    vel_desired.setZero();
    double jerk_limit = 1000;
    double acceleration_limit = 2;

    struct {
        std::vector<double> dt;
        std::vector<double> desired_velocity;
        std::vector<double> desired_acc;
        std::vector<double> desired_jerk;
        std::vector<double> limited_jerk;
        std::vector<double> acc_limited_of_jerk;
        std::vector<double> limited_acc;
        std::vector<double> robot_velocity;
        std::vector<double> robot_acc;
        std::vector<double> commanded_velocity;
        std::vector<double> setpoint_velocity;
        std::vector<double> ext_force;
        std::vector<double> robot_position;
    } writeData{};

    shared_robot_data *robot_data = (shared_robot_data *)arg;
    std::cout << "Created shared robot data" << std::endl;
    // Connect to robot.
    franka::Robot robot(ROBOT_IP);
    std::cout << "Connected to robot" << std::endl;

    while (retries > 0 && !robot_data->shutdown) {

        try {
            //setDefaultBehavior(robot);

            try {
                //robot.setJointImpedance({ {100, 100, 100, 50, 50, 50, 50} });
                robot.setJointImpedance({ {6000, 6000, 6000, 4500, 4500, 4000, 4000} });
                std::cout << "set joint impedance" << std::endl;

            }
            catch (const franka::ControlException &ex){
                std::cerr << ex.what() << std::endl;
            }
            
            // First move the robot to a suitable joint configuration
            //std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
            std::array<double, 7> q_goal = {{0, M_PI_4/2, 0, -3 * M_PI_4, 0, 3.5 * M_PI_4, 0}};
            MotionGenerator motion_generator(0.2, q_goal);

            robot.control(motion_generator);
            std::cout << "Finished moving to initial joint configuration." << std::endl;
            franka::RobotState initial_state = robot.readOnce();
            std::cout << initial_state << std::endl;
            
            // Set additional parameters always before the control loop, NEVER in the control loop!
            // Set collision behavior.
            robot.setCollisionBehavior(
                {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

            // Load the kinematics and dynamics model.
            franka::Model model = robot.loadModel();
            double time = 0.0;
            Pid velocityPid = Pid();

            auto zero_torques = [&](const franka::RobotState& robot_state,
                                                    franka::Duration /*duration*/) -> franka::Torques {
                robot_data->robot_state = robot_state;
                return {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            };


            auto joint_velocities = [&](const franka::RobotState& robot_state, franka::Duration period) -> franka::JointVelocities {

                double dt = period.toSec();

                std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
                std::array<double, 7> gravity_array = model.gravity(robot_state);
                std::array<double, 7> coriolis_array = model.coriolis(robot_state);
                std::array<double, 49> mass_array = model.mass(robot_state);
                Eigen::Map<const Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());
                Eigen::Map<const Eigen::Matrix<double, 7, 1> > q(robot_state.q.data());
                Eigen::Map<const Eigen::Matrix<double, 7, 1> > dq(robot_state.dq.data());
                Eigen::Map<const Eigen::Matrix<double, 7, 1> > ddq_d(robot_state.ddq_d.data());
                Eigen::Map<const Eigen::Matrix<double, 7, 1> > tau_J(robot_state.tau_J.data());
                Eigen::Map<const Eigen::Matrix<double, 7, 1> > tau_ext_hat_filtered(robot_state.tau_ext_hat_filtered.data());
                Eigen::Map<const Eigen::Matrix<double, 7, 1> > gravity(gravity_array.data());
                Eigen::Map<const Eigen::Matrix<double, 7, 1> > coriolis(coriolis_array.data());
                Eigen::Map<const Eigen::Matrix<double, 7, 7> > mass(mass_array.data());
                const Eigen::Matrix<double, 7, 1> tau_J_comp = tau_J - gravity - mass*ddq_d + coriolis;
                const Eigen::Matrix<double, 6, 1> F = jacobian * tau_ext_hat_filtered;


                robot_data->timer += dt;
                robot_data->plot2.push_back(Point(robot_data->timer,F[2]));
                robot_data->plot1.push_back(Point(robot_data->timer,-robot_state.K_F_ext_hat_K[2]));
                robot_data->run = true;

                if (robot_data->shutdown ) {
                    robot_data->run = false;
                    return franka::MotionFinished(franka::JointVelocities {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
                }
                double v = 0.4*sin(robot_data->timer*8);
                return franka::JointVelocities {0.0, v, 0.0, 0.0 ,0.0, 0.0, 0.0};
            };

            double damping = 500.0;
            
            // Define callback function to send Cartesian pose goals to get inverse kinematics solved.
            auto cartesian_pose_callback = [&]
            (const franka::RobotState& robot_state, franka::Duration period) -> franka::CartesianVelocities {

                //Franka states that the dt=0.001 when computing limiting values. Ref: https://frankaemika.github.io/docs/libfranka.html#errors-due-to-noncompliant-commanded-values
                double dt = 0.001;
                writeData.dt.push_back(dt);
/*
                time += dt;
                if (time > 1.0) {
                    robot_data->setpoint_ang_velocity(0,0) = 0.2;
                }
                if (time > 2.0) {
                    robot_data->setpoint_ang_velocity(0,0) = 0.0;
                }
                if (time > 3.0) {
                    robot_data->setpoint_ang_velocity(0,0) = -0.2;
                }
                if (time > 4.0) {
                    robot_data->setpoint_ang_velocity(0,0) = 0.0;
                    time = 0.0;
                }
*/
                robot_data->timer += period.toSec();

                if (robot_data->timer > 2.0 && robot_data->timer < 4.0) {
                    robot_data->track_position = true;
                } else {
                    robot_data->track_position = false;
                }
                
                robot_data->robot_position = get_position(robot_state);
                writeData.robot_position.push_back(robot_data->robot_position.norm());
                robot_data->robot_velocity = get_velocity(robot_state);
                robot_data->robot_ang_velocity = get_ang_velocity(robot_state);
                writeData.commanded_velocity.push_back(robot_data->robot_velocity.norm());
                robot_data->robot_acceleration = get_acceleration(robot_state);
                robot_data->robot_ang_acceleration = get_ang_acceleration(robot_state);
                writeData.robot_acc.push_back(robot_data->robot_acceleration.norm());

                //Get forces on the flange
                //robot_data->external_force = get_ext_force_filtered(robot_state, robot_data->external_force, robot_data->kd);
                robot_data->external_force = get_ext_force(robot_state);
                robot_data->external_ang_force = get_ext_ang_force(robot_state);
                writeData.ext_force.push_back(robot_data->external_force.norm());
                double wanted_force = robot_data->kp*9.81; 
                Eigen::Vector3d total_force = robot_data->external_force.normalized()*std::max(robot_data->external_force.norm() - wanted_force, 0.0);
                
                Eigen::Vector3d total_ang_force = robot_data->external_ang_force.normalized()*std::max(robot_data->external_ang_force.norm() - wanted_force, 0.0);
        
                //Gradually increase damping to a high value if external force is zero to prevent drifting
                damping = std::min(damping + 0.5, 500.0);
                if (total_force.norm() > 0.0) {
                    damping = robot_data->ki;
                }
                robot_data->setpoint_acc = (1.0/robot_data->fake_mass)*(total_force - damping*robot_data->robot_velocity); //a=1/m * (F - Bv) Admittance controller
                
                double inertia = 0.66*robot_data->fake_mass*pow(0.05,2); //Inertia for a ball with certain radius
                robot_data->setpoint_ang_acc = (1.0/inertia)*(robot_data->external_ang_force - 3.0*robot_data->robot_ang_velocity); //a=1/m * (F - Bv) Admittance controller
                
                //Set velocity setpoint to be fed to PID from acceleration setpoint
                robot_data->setpoint_velocity = robot_data->robot_velocity + robot_data->setpoint_acc * dt;
                
                Vector3d constrain_direction = {0.5774,0.5774,0.5774};
                robot_data->setpoint_velocity = robot_data->setpoint_velocity.dot(constrain_direction)*constrain_direction;

                //
                //velocityPid.setParameters(robot_data->kp, 0.0, robot_data->kd);
                //robot_data->desired_velocity = velocityPid.computePID(robot_data->setpoint_velocity-robot_data->robot_velocity ,dt);
                robot_data->desired_velocity = robot_data->setpoint_velocity;
                //
               
                writeData.desired_velocity.push_back(robot_data->desired_velocity.norm());

                //acc = robot_data->setpoint_acc;
                acc = (robot_data->desired_velocity - robot_data->robot_velocity) / dt;
                //acc_ang = (robot_data->setpoint_ang_velocity - robot_data->robot_ang_velocity) / dt;
                acc_ang = robot_data->setpoint_ang_acc;
                writeData.desired_acc.push_back(acc.norm());
                Eigen::Vector3d jerk = (acc - robot_data->robot_acceleration) / dt;
                Eigen::Vector3d jerk_ang = (acc_ang - robot_data->robot_ang_acceleration) / dt;
                writeData.desired_jerk.push_back(jerk.norm());
                
                limitVector(jerk, jerk_limit);
                limitVector(jerk_ang, jerk_limit);
                writeData.limited_jerk.push_back(jerk.norm());
                
                acc = robot_data->robot_acceleration + jerk*dt;
                acc_ang = robot_data->robot_ang_acceleration + jerk_ang*dt;
                writeData.acc_limited_of_jerk.push_back(acc.norm());

                limitVector(acc, acceleration_limit);
                limitVector(acc_ang, acceleration_limit);
                writeData.limited_acc.push_back(acc.norm());
                acc *= 2.59155; //Desired and performed acceleration is different by this constant for some reason.

                vel_desired = robot_data->robot_velocity + acc * dt;
                vel_desired_ang = robot_data->robot_ang_velocity + acc_ang * dt;
                writeData.robot_velocity.push_back(vel_desired.norm());
                writeData.setpoint_velocity.push_back(robot_data->setpoint_velocity.norm());

                robot_data->plot2.push_back(Point(robot_data->timer,acc.norm()));
                //std::cout << acc_ang.norm()/robot_data->robot_ang_acceleration.norm() << std::endl;
                robot_data->plot1.push_back(Point(robot_data->timer,robot_data->robot_acceleration.norm()));

                robot_data->run = true;

                //franka::CartesianVelocities robot_command = {{vel_desired[0], vel_desired[1], vel_desired[2], vel_desired_ang[0], vel_desired_ang[1], vel_desired_ang[2]}};
                //franka::CartesianVelocities robot_command = {{vel_desired[0], 0.0, 0.0, vel_desired_ang[0], vel_desired_ang[1], vel_desired_ang[2]}};
                franka::CartesianVelocities robot_command = {{vel_desired[0], vel_desired[1], vel_desired[2], 0.0, 0.0, 0.0}};
                
                if (robot_data->track_position) {
                    robot_data->tracking_data.push_back({
                        robot_data->timer,
                        robot_data->robot_position[0],
                        robot_data->robot_position[1],
                        robot_data->robot_position[2]
                    });
                }

                //If we are trying to shut down, wait for the robot to reach zero speed before finishing.
                if (robot_data->shutdown && robot_data->robot_velocity.norm() < 0.01) {
                    std::cout << "Speed is now zero, and we are shutting down." << std::endl;
                    robot_data->run = false;
                    return franka::MotionFinished(franka::CartesianVelocities{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}});
                }
                return robot_command;
            };
            // Start real-time control loop.
            robot.control(cartesian_pose_callback);
            
        } catch (const franka::Exception& e) {
            robot_data->run = false;
            std::cerr << e.what() << std::endl;
            std::cerr << "Running error recovery..." << std::endl;
            try{
                robot.automaticErrorRecovery();
            }
            catch(const franka::Exception& er){
                std::cerr << "Recovery failed: " << er.what() << std::endl;
                exit(EXIT_FAILURE);
                return nullptr;
            }
            
            retries--;
            sleep(5);
        }
    }
    robot_data->run = false;

    std::cout << "Robot loop thread shutting down " << std::endl;
    return NULL;
}