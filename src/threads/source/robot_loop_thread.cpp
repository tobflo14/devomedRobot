#include "robot_loop_thread.h"
#include "global_struct.h"
#include "functions.h"
//#include "examples_common.h"
#include "pid.h"

#include <iostream>
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
    vel_desired.setZero();
    double jerk_limit = 3000;
    double acceleration_limit = 10;


    shared_robot_data *robot_data = (shared_robot_data *)arg;
    
    std::cout << "Created shared robot data" << std::endl;
    // Connect to robot.
    franka::Robot robot(ROBOT_IP);

    while (retries > 0) {
        try {
            // Connect to robot.
        // franka::Robot robot(ROBOT_IP);
            std::cout << "Connected to robot" << std::endl;
            //setDefaultBehavior(robot);

            try {
                robot.setJointImpedance({ {3000, 3000, 3000, 2500, 2500, 2000, 2000} });
                std::cout << "set joint impedance" << std::endl;

            }
            catch (const franka::ControlException &ex){
                std::cerr << ex.what() << std::endl;
            }
            
            // First move the robot to a suitable joint configuration
            //std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
            //std::array<double, 7> q_goal = {{0, M_PI_4, 0, -2 * M_PI_4, 0, 3 * M_PI_4, M_PI_4}};
            //MotionGenerator motion_generator(0.5, q_goal);

            //robot.control(motion_generator);
            std::cout << "Finished moving to initial joint configuration." << std::endl;
            
            // Set additional parameters always before the control loop, NEVER in the control loop!
            // Set collision behavior.
            robot.setCollisionBehavior(
                {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

            // Load the kinematics and dynamics model.
            franka::Model model = robot.loadModel();
            
            // Define callback function to send Cartesian pose goals to get inverse kinematics solved.
            auto cartesian_pose_callback = [=, &robot_data, &vel_desired]
            (const franka::RobotState& robot_state, franka::Duration period) -> franka::JointVelocities {

                double dt = period.toSec();
                robot_data->timer += dt;
                robot_data->robot_velocity = get_velocity(robot_state);
                robot_data->robot_acceleration = get_acceleration(robot_state);
                //Get forces on the flange
                // Eigen::Map<const Eigen::Matrix<double, 6, 1>> force_input(robot_state.K_F_ext_hat_K.data());

                //Only run if dt > 0 because limits derive on dt
                if (dt) { 
                    //dt = 0.001;
                    vel_desired = robot_data->desired_velocity;

                    Eigen::Vector3d acc = (vel_desired - robot_data->robot_velocity) / dt;
                    Eigen::Vector3d jerk = (acc - robot_data->robot_acceleration) / dt;
                    
                    if (jerk.norm() > jerk_limit) {
                        jerk.normalize();
                        jerk *= jerk_limit;
                    }

                    acc = robot_data->robot_acceleration + jerk*dt;

                    if (acc.norm() > acceleration_limit) {
                        acc.normalize();
                        acc *= acceleration_limit;
                    }

                    vel_desired = robot_data->robot_velocity + acc * dt;
    
                }

                robot_data->run = true;
                return franka::JointVelocities{{vel_desired[0], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
            };
            // Start real-time control loop.
            robot.control(cartesian_pose_callback);
            
        } catch (const franka::Exception& e) {
            robot_data->run = false;
            std::cout << e.what() << std::endl;
            std::cout << "Running error recovery..." << std::endl;

            try{
                robot.automaticErrorRecovery();
            }
            catch(const franka::Exception& er){
                std::cout << "Recovery failed: " << er.what() << std::endl;
                exit(EXIT_FAILURE);
                return nullptr;
            }
            
            retries--;
            sleep(5);
        }
    }
    robot_data->shutdown = true;

    std::cout << "Robot loop thread shutting down " << std::endl;
    return NULL;
}