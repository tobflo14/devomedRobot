#include "robot_loop_thread.h"
#include "global_struct.h"
#include "functions.h"
#include "pseudo_inversion.h"
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

int tries = 100;

void* RobotLoopThread(void* arg) {

    std::cout << "Starting robot thread" << std::endl;
    std::cout << "robot loop thread ID : " << syscall(SYS_gettid) << std::endl;

    shared_robot_data *robot_data = (shared_robot_data *)arg;
    std::cout << "Created shared robot data" << std::endl;
    // Connect to robot.
    franka::Robot robot(ROBOT_IP);
    std::cout << "Connected to robot" << std::endl;


    //cout << track_path << endl;

    while (tries > 0 && !robot_data->shutdown) {

        try {
            setDefaultBehavior(robot);
            robot.setJointImpedance({ {600, 600, 600, 450, 450, 400, 400} });
            //robot.setJointImpedance({ {6000, 6000, 6000, 4500, 4500, 4000, 4000} });
            std::cout << "set joint impedance" << std::endl;
            robot.setCollisionBehavior(
                {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});
            // First move the robot to a suitable joint configuration
            //std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
            std::array<double, 7> q_goal = {{0, M_PI_4/2, 0, -3 * M_PI_4, 0, 3.5 * M_PI_4, 0}};
            MotionGenerator motion_generator(0.2, q_goal);

            robot.control(motion_generator);
            std::cout << "Finished moving to initial joint configuration." << std::endl;
            franka::Model model = robot.loadModel();
            franka::RobotState initial_state = robot.readOnce();
            const Eigen::Vector3d initial_cart_pos = get_position(initial_state);


            Eigen::Vector3d force_prev;
            force_prev.setZero();
            Eigen::Vector3d ang_force_prev;
            ang_force_prev.setZero();
            double dt = 0.001;
            double time = 0.0;

            Pid positionPid = Pid();
            Eigen::Vector3d past_error;
            past_error.setZero();
            robot_data->floating_mode = true;
            bool trackingpath_is_initialized = false;


            auto cartesian_joint_velocities = [&](const franka::RobotState& robot_state, franka::Duration period) -> franka::JointVelocities {
                //Constants, to be taken out of the loop when finished
                double filter_freq = 10.0;
                double virtual_mass = 2.0;
                double inertia_radius = 0.8;

                time += dt;

                std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
                Eigen::Map<const Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());
                Eigen::Map<const Eigen::Matrix<double, 7, 1> > dq(robot_state.dq_d.data());
                const Eigen::MatrixXd pseudo_inverse = jacobian.completeOrthogonalDecomposition().pseudoInverse();
                const Eigen::VectorXd robot_velocity = jacobian*dq;
                const Eigen::Vector3d robot_cart_vel = robot_velocity.head(3);
                const Eigen::Vector3d robot_ang_vel = robot_velocity.tail(3);
                const Eigen::Vector3d robot_cart_pos = get_position(robot_state);


                // -- Get force input --
                const Eigen::Vector3d external_force = get_ext_force_magnitude_filtered(robot_state, force_prev, filter_freq);
                const Eigen::Vector3d external_ang_force = get_ext_ang_force_magnitude_filtered(robot_state, ang_force_prev, filter_freq);
                force_prev = external_force;
                ang_force_prev = external_ang_force;
                // -- END Get force input --

                // -- Force Controller --
                double friction = 0.01*virtual_mass*9.81; //F_f = mu*m*g
                double ang_friction = 0.05*virtual_mass*9.81; //F_f = mu*m*g
                friction += (robot_data->wanted_force/1000)*9.81;
                Eigen::Vector3d cart_friction_force;
                cart_friction_force.setZero();
                Eigen::Vector3d ang_friction_force;
                ang_friction_force.setZero();
                //Apply static friction if no speed. Apply kinetic friction if external force exceeds friction force.
                if (robot_cart_vel.norm() > 0.01 || external_force.norm() > friction) {
                    //Apply friction in the opposite direction of velocity
                    cart_friction_force = -robot_cart_vel.normalized()*friction;
                } else {
                    //Apply friction equal to the external force, to make sure sum F = 0, and there is no movement.
                    cart_friction_force = -external_force;
                }
                //Apply static friction if no speed. Apply kinetic friction if external force exceeds friction force.
                if (robot_ang_vel.norm() > 0.01 || external_ang_force.norm() > friction) {
                    //Apply friction in the opposite direction of velocity
                    ang_friction_force = -robot_ang_vel.normalized()*ang_friction;
                } else {
                    //Apply friction equal to the external force, to make sure sum F = 0, and there is no movement.
                    ang_friction_force = -external_ang_force;
                }
                double inertia = 0.66*virtual_mass*pow(inertia_radius,2); //Inertia for a ball with certain radius
                const Eigen::Vector3d total_force = external_force + cart_friction_force;
                const Eigen::Vector3d acc = (1.0/virtual_mass)*total_force; // a = 1/m * F
                const Eigen::Vector3d total_ang_force = external_ang_force + ang_friction_force;
                const Eigen::Vector3d ang_acc = (1.0/inertia)*total_ang_force; // a = 1/I * F
                const Eigen::Vector3d cart_vel_desired = robot_cart_vel + acc * dt;
                const Eigen::Vector3d ang_vel_desired = robot_ang_vel + ang_acc * dt;
                // -- END Force Controller --

                // -- Constrain Position --
                Eigen::Vector3d cart_pos_desired = robot_cart_pos + cart_vel_desired * dt;

                //bool 
                if (!robot_data->floating_mode) {
                    robot_data->plot1.push_back(Point(0.0,external_force.norm()/9.81));
                    if (!trackingpath_is_initialized) {
                        Eigen::Vector3d first_column = robot_data->track_path.col(0);
                        robot_data->track_path = robot_data->track_path.colwise() - first_column;
                        robot_data->track_path = robot_data->track_path.colwise() + robot_cart_pos;
                        trackingpath_is_initialized = true;
                    }
                    const Eigen::Vector3d closestPoint = closestPointOnLineSegment(robot_data->track_path, cart_pos_desired, robot_data->fractionCompleted);
                    // -- PID --
                    Eigen::Vector3d error = closestPoint - cart_pos_desired;
                    positionPid.setParameters(robot_data->kp, robot_data->ki, robot_data->kd);
                    const Eigen::Vector3d pid_position = positionPid.computePID(error, dt);
                    cart_pos_desired += pid_position;
                    // -- END PID --
                } else {
                    trackingpath_is_initialized = false;
                }

                
               
                const Eigen::Vector3d cart_vel_constrained = (cart_pos_desired - robot_cart_pos)/dt;
                //cout << closestPoint[2] << endl;

                // -- END Constrain Position --

                if (robot_data->track_position) {
                    robot_data->tracking_data.push_back({
                        robot_cart_pos[0],
                        robot_cart_pos[1],
                        robot_cart_pos[2]
                    });
                }

                Eigen::VectorXd desired_velocity(6);
                desired_velocity.setZero();
                desired_velocity.head(3) = cart_vel_constrained;
                //desired_velocity.tail(3) = ang_vel_desired;

                //Solve IK
                Eigen::VectorXd qd_desired = pseudo_inverse*desired_velocity;

                robot_data->run = true;

                if (robot_data->shutdown) {
                    qd_desired.setZero();
                    if (!(abs(dq.array()) > 0.001).any()) {
                        robot_data->run = false;
                        return franka::MotionFinished(franka::JointVelocities{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}});
                    }
                    
                }

                return {{qd_desired[0], qd_desired[1], qd_desired[2], qd_desired[3], qd_desired[4], qd_desired[5], qd_desired[6]}};
            };
            // Start real-time control loop.
            robot.control(cartesian_joint_velocities);
            
        } catch (const franka::Exception& e) {
            
            std::cerr << e.what() << std::endl;
            if (!robot_data->shutdown) {
                robot_data->run = false;
                std::cerr << "Running error recovery..." << std::endl;
                try{
                    robot.automaticErrorRecovery();
                }
                catch(const franka::Exception& er){
                    std::cerr << "Recovery failed: " << er.what() << std::endl;
                    exit(EXIT_FAILURE);
                    return nullptr;
                }
                tries--;
                sleep(5);
            }
        }
    }
    robot_data->run = false;

    std::cout << "Robot loop thread shutting down " << std::endl;
    return NULL;
}