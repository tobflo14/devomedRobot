#include "robot_loop_thread.h"
#include "global_struct.h"
#include "functions.h"
#include "examples_common.h"
#include "pid.h"

void* RobotLoopThread(void* arg) {
    std::cout << "Starting robot thread" << std::endl;
    Eigen::VectorXd vel_desired(6);
    vel_desired.setZero();

    shared_robot_data *robot_data = (shared_robot_data *)arg;

    try {
        // Connect to robot.
        franka::Robot robot(ROBOT_IP);
        setDefaultBehavior(robot);
        
        // First move the robot to a suitable joint configuration
        //std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
        std::array<double, 7> q_goal = {{0, M_PI_4, 0, -2 * M_PI_4, 0, 3 * M_PI_4, M_PI_4}};
        MotionGenerator motion_generator(0.5, q_goal);
        /*
        std::cout << "WARNING: This example will move the robot! "
                << "Please make sure to have the user stop button at hand!" << std::endl
                << "Press Enter to continue..." << std::endl;
        std::cin.ignore();
        */
        robot.control(motion_generator);
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
        (const franka::RobotState& robot_state, franka::Duration period) -> franka::CartesianVelocities {

        robot_data->timer += period.toSec();

        double dt = 0.001;//period.toSec();
        //Guess a dt if no dt, to make sure we don't divide by 0
        if (!dt) { 
            dt = 0.001;
        }
        //Get forces on the flange
        // Eigen::Map<const Eigen::Matrix<double, 6, 1>> force_input(robot_state.K_F_ext_hat_K.data());

        //Get last commanded velocity (commanded or actually performed?)
        Eigen::Map<const Eigen::Matrix<double, 6, 1>> vel_commanded_previous(robot_state.O_dP_EE_c.data());
        Eigen::Map<const Eigen::Matrix<double, 6, 1>> acc_commanded_previous(robot_state.O_ddP_EE_c.data());


        robot_data->robot_velocity = vel_commanded_previous.head(3);
        robot_data->robot_acceleration = acc_commanded_previous.head(3);

        vel_desired = robot_data->desired_velocity;
        
        //franka::CartesianVelocities velocity_desired = {{vel_desired[0], vel_desired[1], vel_desired[2], 0.0, 0.0, 0.0}};
        //franka::CartesianVelocities velocity_desired = {{0.0, 0.0, 0.0, -vel_desired[3], vel_desired[4], vel_desired[5]}};
        franka::CartesianVelocities velocity_desired = {{0.0, 0.0, vel_desired[2], 0.0, 0.0, 0.0}};
        //franka::CartesianVelocities velocity_desired = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
        
        robot_data->run = true;
        return velocity_desired;
        };
        // Start real-time control loop.
        robot.control(cartesian_pose_callback);
        
    } catch (const franka::Exception& ex) {
        robot_data->run = false;
        std::cerr << ex.what() << std::endl;
        
    }
    return NULL;
}