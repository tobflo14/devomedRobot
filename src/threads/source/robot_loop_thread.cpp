#include "robot_loop_thread.h"
#include "global_struct.h"
#include "functions.h"
//#include "examples_common.h"
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
    Eigen::Vector3d acc;
    vel_desired.setZero();
    double jerk_limit = 500;
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
            //std::cout << "Finished moving to initial joint configuration." << std::endl;
            
            // Set additional parameters always before the control loop, NEVER in the control loop!
            // Set collision behavior.
            robot.setCollisionBehavior(
                {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
                {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

            // Load the kinematics and dynamics model.
            franka::Model model = robot.loadModel();
            double time = 0.0;
            
            // Define callback function to send Cartesian pose goals to get inverse kinematics solved.
            auto cartesian_pose_callback = [=, &robot_data, &vel_desired, &acc, &time, &writeData]
            (const franka::RobotState& robot_state, franka::Duration period) -> franka::CartesianVelocities {

                robot_data->timer += period.toSec();
                
                robot_data->robot_velocity = get_velocity(robot_state);
                writeData.commanded_velocity.push_back(robot_data->robot_velocity[0]);
                robot_data->robot_acceleration = get_acceleration(robot_state);
                writeData.robot_acc.push_back(robot_data->robot_acceleration[0]);
                /*
                robot_data->robot_velocity = vel_desired;
                robot_data->robot_acceleration = acc;
*/

                //Get forces on the flange
                robot_data->external_force = get_ext_force(robot_state);
                writeData.ext_force.push_back(robot_data->external_force[0]);
               
                //Franka states that the dt=0.001 when computing limiting values. Ref: https://frankaemika.github.io/docs/libfranka.html#errors-due-to-noncompliant-commanded-values
                double dt = 0.001;

                writeData.dt.push_back(dt);
                writeData.desired_velocity.push_back(robot_data->desired_velocity[0]);

                acc = (robot_data->desired_velocity - robot_data->robot_velocity) / dt;
                writeData.desired_acc.push_back(acc[0]);
                Eigen::Vector3d jerk = (acc - robot_data->robot_acceleration) / dt;
                writeData.desired_jerk.push_back(jerk[0]);
                
                if (jerk.norm() > jerk_limit) {
                    jerk.normalize();
                    jerk *= jerk_limit;
                }
                writeData.limited_jerk.push_back(jerk[0]);
                
                acc = robot_data->robot_acceleration + jerk*dt;
                writeData.acc_limited_of_jerk.push_back(acc[0]);

                if (acc.norm() > acceleration_limit) {
                    acc.normalize();
                    acc *= acceleration_limit;
                }
                writeData.limited_acc.push_back(acc[0]);
                acc *= 2.59155; //Desired and performed acceleration is different by this constant for some reason.

                vel_desired = robot_data->robot_velocity + acc * dt;
                writeData.robot_velocity.push_back(vel_desired[0]);
                writeData.setpoint_velocity.push_back(robot_data->setpoint_velocity[0]);
                robot_data->plot2.push_back(Point(robot_data->timer,robot_data->setpoint_velocity[0]));
                robot_data->plot1.push_back(Point(robot_data->timer,robot_data->robot_velocity[0]));

                robot_data->run = true;

                franka::CartesianVelocities robot_command = {{vel_desired[0], 0.0, 0.0, 0.0, 0.0, 0.0}};
                //franka::CartesianVelocities robot_command = {{vel_desired[0], vel_desired[1], vel_desired[2], 0.0, 0.0, 0.0}};
                
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
    robot_data->run = false;

    std::ofstream file_to_write;
    file_to_write.open("output.csv");
    if (file_to_write.is_open()) {
        file_to_write << "dt;Desired velocity;Desired acceleration;Desired jerk;Limited jerk;Acc limited of jerk;Limited acceleration;Robot velocity;Robot acceleration;Commanded velocity;Setpoint velocity;External Force\n";
        for (int i = 1100; i < 1300; i++) {
            file_to_write << writeData.dt[i] << ";";
            file_to_write << writeData.desired_velocity[i] << ";";
            file_to_write << writeData.desired_acc[i] << ";";
            file_to_write << writeData.desired_jerk[i] << ";";
            file_to_write << writeData.limited_jerk[i] << ";";
            file_to_write << writeData.acc_limited_of_jerk[i] << ";";
            file_to_write << writeData.limited_acc[i] << ";";
            file_to_write << writeData.robot_velocity[i] << ";";
            file_to_write << writeData.robot_acc[i] << ";";
            file_to_write << writeData.commanded_velocity[i] << ";";
            file_to_write << writeData.setpoint_velocity[i] << ";";
            file_to_write << writeData.ext_force[i] << ";\n";
        }
        file_to_write.close();
    }
    else std::cout << "Unable to open file. :(" << std::endl;

    std::cout << "Robot loop thread shutting down " << std::endl;
    return NULL;
}