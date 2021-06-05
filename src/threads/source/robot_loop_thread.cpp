#include "robot_loop_thread.h"
#include "global_struct.h"
#include "functions.h"
#include "pseudo_inversion.h"
#include "examples_common.h"
#include "pid.h"

#include <thread>
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
    double q_margin = 0.2;
    Eigen::VectorXd q_min(7);
    q_min <<    -2.8973 + q_margin,
                -1.7628 + q_margin,
                -2.8973 + q_margin,
                -3.0718 + q_margin,
                -2.8973 + q_margin,
                -0.0175 + q_margin,
                -2.8973 + q_margin;
    Eigen::VectorXd q_max(7);
    q_max <<    2.8973 - q_margin,
                1.7628 - q_margin,
                2.8973 - q_margin,
                -0.0698 - q_margin,
                2.8973 - q_margin,
                3.7525 - q_margin,
                2.8973 - q_margin;


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
            //std::array<double, 7> q_goal = {{0, 0, 0, -2 * M_PI_4, 0, M_PI_2, 0}};
            std::array<double, 7> q_goal = {{0, M_PI_4/2, 0, -3 * M_PI_4, 0, 3.5 * M_PI_4, 0}};
            MotionGenerator motion_generator(0.2, q_goal);

            robot.control(motion_generator);
            std::cout << "Finished moving to initial joint configuration." << std::endl;
            franka::Model model = robot.loadModel();
            franka::RobotState initial_state = robot.readOnce();
            const Eigen::Vector3d initial_cart_pos = get_position(initial_state);
            robot_data->robot_mode = initial_state.robot_mode;


            Eigen::Vector3d force_prev;
            force_prev.setZero();
            Eigen::Vector3d ang_force_prev;
            ang_force_prev.setZero();
            double dt = 0.001;
            double time = 0.0;

            Pid positionPid = Pid();
            positionPid.init(Eigen::VectorXd::Zero(3));
            Pid jointPid = Pid();
            jointPid.init(Eigen::VectorXd::Zero(7));
            jointPid.setParameters(0.002, 0, 0.0001);
            
            robot_data->floating_mode = true;
            bool trackingpath_is_initialized = false;


            auto cartesian_joint_velocities = [&](const franka::RobotState& robot_state, franka::Duration period) -> franka::JointVelocities {
                //Constants, to be taken out of the loop when finished
                double filter_freq = 10.0;
                double virtual_mass = 2.0;
                double inertia_radius = 0.4;
                //double end_effector_mass = 0.367; //End effector mass in kg
                double end_effector_mass = 0; //End effector mass in kg

                time += dt;
                robot_data->robot_mode = robot_state.robot_mode;
                robot_data->robot_state = robot_state;

                std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, robot_state);
                Eigen::Map<const Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());
                Eigen::Map<const Eigen::Matrix<double, 7, 1> > dq_d(robot_state.dq_d.data());
                Eigen::Map<const Eigen::Matrix<double, 7, 1> > q_d(robot_state.q_d.data());
                const Eigen::MatrixXd pseudo_inverse = jacobian.completeOrthogonalDecomposition().pseudoInverse();
                const Eigen::VectorXd robot_velocity = jacobian*dq_d;
                const Eigen::Vector3d robot_cart_vel = robot_velocity.head(3);
                const Eigen::Vector3d robot_ang_vel = robot_velocity.tail(3);
                const Eigen::Vector3d robot_cart_pos = get_position(robot_state);


                // -- Get force input --
                Eigen::Vector3d external_force = get_ext_force_magnitude_filtered(robot_state, force_prev, filter_freq);
                const Eigen::Vector3d external_ang_force = get_ext_ang_force_magnitude_filtered(robot_state, ang_force_prev, filter_freq);
                force_prev = external_force;
                ang_force_prev = external_ang_force;
                external_force(2,0) += end_effector_mass*9.81;
                // -- END Get force input --

                // -- Force Controller --

                //Cartesian

                //F_fk = mu*m*g
                double friction = 0.01*virtual_mass*9.81; 
                //Wanted force in grams to act as an extra friction
                double wanted_force = (robot_data->wanted_force/1000)*9.81;
                //Convert friction into static or kinetic friction based on velocity
                const Eigen::Vector3d cart_friction_force = frictionForce(robot_cart_vel, external_force, friction);
                const Eigen::Vector3d cart_wanted_force = frictionForce(robot_cart_vel, external_force, wanted_force);
                //F_tot = F_ext + F_f + F_w
                const Eigen::Vector3d total_force = external_force - cart_friction_force - cart_wanted_force;
                //a = 1/m * F_tot
                const Eigen::Vector3d acc = (1.0/virtual_mass)*total_force;
                //v = v_0 + a * dt
                const Eigen::Vector3d cart_vel_desired = robot_cart_vel + acc * dt;

                //Angular
                double ang_friction = 0.05*virtual_mass*9.81; //F_fk = mu*m*g
                const Eigen::Vector3d ang_friction_force = frictionForce(robot_ang_vel, external_ang_force, ang_friction);
                const Eigen::Vector3d total_ang_force = external_ang_force - ang_friction_force*inertia_radius;
                double inertia = 0.66*virtual_mass*pow(inertia_radius,2); //Inertia for a ball with certain radius
                const Eigen::Vector3d ang_acc = (1.0/inertia)*total_ang_force; // a = 1/I * F
                const Eigen::Vector3d ang_vel_desired = robot_ang_vel + ang_acc * dt;
                // -- END Force Controller --

                // -- Constrain Position --
                Eigen::Vector3d cart_pos_desired = robot_cart_pos + cart_vel_desired * dt;

                if (!robot_data->floating_mode) {
                    robot_data->plot1.push_back(Point(0.0,external_force.norm()/9.81));
                   // robot_data->plot2.push_back(Point(0.0,external_force.norm()/9.81))
                    if (!trackingpath_is_initialized) {
                        Eigen::Vector3d first_column = robot_data->track_path.col(0);
                        robot_data->track_path = robot_data->track_path.colwise() - first_column;
                        robot_data->track_path = robot_data->track_path.colwise() + robot_cart_pos;
                        trackingpath_is_initialized = true;
                    }
                    const Eigen::Vector3d closestPoint = closestPointOnLineSegment(robot_data->track_path, cart_pos_desired, robot_data->fractionCompleted);
                    // -- PID --
                    Eigen::Vector3d error = closestPoint - cart_pos_desired;

                    double length_error = external_force.dot(error.normalized());
                    robot_data->plot2.push_back(Point(0.0, (external_force - length_error*error.normalized()).norm()/9.81));
                    positionPid.setParameters(robot_data->kp, robot_data->ki, robot_data->kd);
                    const Eigen::Vector3d pid_position = positionPid.computePID(error, dt);
                    cart_pos_desired += pid_position;
                    // -- END PID --
                } else {
                    trackingpath_is_initialized = false;
                }
                const Eigen::Vector3d cart_vel_constrained = (cart_pos_desired - robot_cart_pos)/dt;
                
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
                Eigen::VectorXd q_desired = q_d + qd_desired*dt;
                
                Eigen::VectorXd q_clamped = q_desired.cwiseMin(q_max).cwiseMax(q_min);
                Eigen::VectorXd joint_error = q_clamped - q_desired;
                
                Eigen::VectorXd q_pid = jointPid.computePID(q_clamped - q_desired, dt);
                Eigen::VectorXd iserror = joint_error.cwiseSign().cwiseAbs();
                
                q_desired = q_desired + q_pid;//*iserror;

                qd_desired = (q_desired - q_d)/dt;

                std::array<double, 7> qd = {qd_desired[0], qd_desired[1], qd_desired[2], qd_desired[3], qd_desired[4], qd_desired[5], qd_desired[6]};
/*
                std::array<double, 7> qd_limited = franka::limitRate(
                    franka::kMaxJointVelocity,
                    franka::kMaxJointAcceleration,
                    {franka::kMaxJointJerk[0]*0.5, franka::kMaxJointJerk[1]*0.5, franka::kMaxJointJerk[2]*0.5, franka::kMaxJointJerk[3]*0.5, franka::kMaxJointJerk[4]*0.5, franka::kMaxJointJerk[5]*0.5, franka::kMaxJointJerk[6]*0.5},
                    qd,
                    robot_state.dq_d,
                    robot_state.ddq_d
                );
*/
                robot_data->run = true;

                if (robot_data->shutdown) {
                    qd = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
                    if (!(abs(dq_d.array()) > 0.001).any()) {
                        robot_data->run = false;
                        return franka::MotionFinished(franka::JointVelocities{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}});
                    }
                    
                }

                return qd;
            };
                        // Define callback function to send Cartesian pose goals to get inverse kinematics solved.
            auto cartesian_velocities = [&]
            (const franka::RobotState& robot_state, franka::Duration period) -> franka::CartesianVelocities {

                //Franka states that the dt=0.001 when computing limiting values. Ref: https://frankaemika.github.io/docs/libfranka.html#errors-due-to-noncompliant-commanded-values
                double dt = 0.001;

                time += dt;
                if (time > 1.0) {
                    robot_data->desired_velocity(0,0) = 0.1;
                }
                if (time > 2.0) {
                    robot_data->desired_velocity(0,0) = 0.0;
                }
                if (time > 3.0) {
                    robot_data->desired_velocity(0,0) = -0.1;
                }
                if (time > 4.0) {
                    robot_data->desired_velocity(0,0) = 0.0;
                    time = 0.0;
                }

                robot_data->robot_velocity = get_velocity(robot_state);
                robot_data->robot_acceleration = get_acceleration(robot_state);

                Eigen::Vector3d acc = (robot_data->desired_velocity - robot_data->robot_velocity) / dt;
                Eigen::Vector3d jerk = (acc - robot_data->robot_acceleration) / dt;
                limitVector(jerk, 1000);
                acc = robot_data->robot_acceleration + jerk*dt;
                limitVector(acc, 6);
                //acc *= 2.59155; //Desired and performed acceleration is different by this constant for some reason.
                Eigen::Vector3d vel_desired = robot_data->robot_velocity + acc * dt;
                cout << robot_data->desired_velocity[0] << "," << vel_desired[0] << endl;

                robot_data->run = true;
                franka::CartesianVelocities robot_command = {{vel_desired[0], vel_desired[1], vel_desired[2], 0.0, 0.0, 0.0}};
                return robot_command;
            };
            // Start real-time control loop.
            robot.control(cartesian_joint_velocities);
            
        } catch (const franka::Exception& e) {
            robot_data->robot_mode = robot.readOnce().robot_mode;
            robot_data->error_message = e.what();
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
    robot_data->robot_mode = robot.readOnce().robot_mode;
    std::cout << "Robot loop thread shutting down " << std::endl;
    return NULL;
}