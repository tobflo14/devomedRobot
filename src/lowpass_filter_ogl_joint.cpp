// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <array>
#include <atomic>
#include <cmath>
#include <functional>
#include <fstream>
#include <iterator>
#include <mutex>

#include <Eigen/Dense>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>

#include <thread>
#include <iostream>

//Headers for arduino input
#include <stdio.h> // standard input / output functions
#include <string.h> // string function definitions
#include <unistd.h> // UNIX standard function definitions
#include <fcntl.h> // File control definitions

#include <stdlib.h>
#include <errno.h>
#include <termios.h>

#include "examples_common.h"


// ------------- OpenGL-stuff ------------------------
// Include standard headers
#include <time.h>

// Include GLEW
#include <GL/glew.h>

// Include GLFW
#include <GLFW/glfw3.h>
GLFWwindow* window;

// Include GLM
#include <glm/glm.hpp>
//#include <glm/gtc/matrix_transform.hpp>
using namespace glm;

#include <common/shader.hpp>
#include "graphics/plot2d.h"

const int C_SAMPLES_PER_FRAME = 1000;
GLuint vertexBufferArray;
GLuint vertexbuffer;
GLuint vertexDataBuffer;
GLint _dataTail;
double dt;
GLuint programID;
GLuint MatrixID;
glm::mat4 MVP;
	
void generateBuffers()
{
	glGenVertexArrays(1, &vertexBufferArray);
	glGenBuffers(1, &vertexDataBuffer);
}

void testing() {
	// Open a window and create its OpenGL context
	window = glfwCreateWindow(1024, 768, "Playground", NULL, NULL);
	if (window == NULL) {
		fprintf(stderr, "Failed to open GLFW window. If you have an Intel GPU, they are not 3.3 compatible. Try the 2.1 version of the tutorials.\n");
		getchar();
		glfwTerminate();
		//return -1;
	}
	glfwMakeContextCurrent(window);

	glewExperimental = true; // Needed for core profile
	if (glewInit() != GLEW_OK) {
		fprintf(stderr, "Failed to initialize GLEW\n");
		getchar();
		glfwTerminate();
	//	return -1;
	}
	// Ensure we can capture the escape key being pressed below
	glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);

	// Dark blue background
	glClearColor(0.0f, 0.0f, 0.0f, 0.0f);

	// Enable depth test
	glEnable(GL_DEPTH_TEST);

	// Accept fragment if it closer to the camera than the former one
	glDepthFunc(GL_LESS);

	// Create and compile our GLSL program from the shaders
  programID = LoadShaders("SimpleVertexShader.vertexshader", "SimpleFragmentShader.fragmentshader");

	// Get a handle for our "MVP" uniform
	MatrixID = glGetUniformLocation(programID, "MVP");	
}
// -------------- END opengl stuff --------------


void print_vector(std::vector<double> const &input) {
  for (int i = 0; i < input.size(); i++) {
    std::cout << input.at(i) << std::endl;
  }
  std::cout << std::endl;
}


int main(int argc, char** argv) {
  // Check whether the required arguments were passed.
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }
  //Open an output file to write data to
  std::ofstream outputfile;
  outputfile.open("output.csv");

  if (outputfile.is_open()) {
    std::cout << "output.csv is opened" << std::endl;
  }
  
  double time = 0.0;
  double acceleration_limit = 15.0;
  double jerk_limit = 7500.0;

  std::vector<double> pid_constants = {0.001, 0.001, 0.001};
  std::vector<double> arduino_input;

  std::atomic_bool running{true};
  std::atomic_bool started{false};

  struct {
    std::mutex mutex;
    bool has_data;
    //std::vector<double> data;
    double data;
  } print_data{};

  std::vector<std::vector<double>> velocity_array;
  double last_force = 0.0;

  //Initialize desired forces and set them to zero
  Eigen::VectorXd desired_force(7);
  desired_force.setZero();

  Eigen::VectorXd error(7);
  error.setZero();

  Eigen::VectorXd past_error(7);
  past_error.setZero();
  
  Eigen::VectorXd integral(7);
  integral.setZero();

  Eigen::VectorXd derivative(7);
  derivative.setZero();
  
  Eigen::VectorXd vel_desired(7);
  vel_desired.setZero();

  Eigen::VectorXd acc(7);
  acc.setZero();

  Eigen::VectorXd acc_previous(7);
  acc_previous.setZero();

  Eigen::VectorXd jerk(7);
  jerk.setZero();

  Eigen::VectorXd pd(7);
  pd.setZero();


  // Start print thread.
  std::thread print_thread([&print_data, &arduino_input, &running, &pid_constants, &started]() {
    
    
    int USB = open( "/dev/ttyUSB0", O_RDWR| O_NOCTTY );

    if (USB < 0) {
      std::cout << "Error while opening device: " << strerror(errno) << std::endl;
    } else {
      struct termios tty;
      struct termios tty_old;
      memset (&tty, 0, sizeof tty);

    if ( tcgetattr(USB, &tty) != 0) {
      std::cout << "Error " << errno << " from tcgetattr: " << strerror(errno) << std::endl;
    } else {

        tty_old = tty;

        cfsetospeed (&tty, (speed_t)B9600);
        cfsetispeed (&tty, (speed_t)B9600);

        tty.c_cflag &= ~PARENB;
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;

        tty.c_cflag &= ~CRTSCTS;
        tty.c_cc[VMIN] = 1;
        tty.c_cc[VTIME] = 5;
        tty.c_cflag |= CREAD | CLOCAL;

        cfmakeraw(&tty);

        tcflush( USB, TCIFLUSH);
        if (tcsetattr(USB, TCSANOW, &tty) != 0) {
          std::cout << "Error " << errno << " from tcsetattr" << std::endl;
        } else {
          std::cout << "Arduino Setup complete" << std::endl;
          
          
          initGLFW(MVP);
          testing();
          generateBuffers();
          initializeBuffer(vertexBufferArray, vertexDataBuffer, 2);

          struct timespec ts, last_ts;
	        timespec_get(&last_ts, TIME_UTC);
        
          while (running) {
            if (started) {
              timespec_get(&ts, TIME_UTC);
		          double dt = ts.tv_sec - last_ts.tv_sec + (ts.tv_nsec - last_ts.tv_nsec) / 1E9;

		          if (dt <= 0.05) continue; // limit the program to 20fps
		          last_ts = ts;

              int n = 0,
                  spot = 0;
              char buf = '\0';
              char response[1024];
              memset(response, '\0', sizeof response);

              do {
                  n = read( USB, &buf, 1 );
                  sprintf( &response[spot], "%c", buf );
                  spot += n;
              } while( buf != '\r' && n > 0);

              if (n < 0) {
                std::cout << "Error reading: " << strerror(errno) << std::endl;
              } else if (n == 0) {
                std::cout << "Read nothing!" << std::endl;
              } else {//} if (response[0] != '\0' && response[0] != '\n') { //Successful response
                std::string string_response = std::string(response);
                double Kp = stod(string_response.substr(0, string_response.find(",")));
                Kp = (Kp/1023)*1;
                string_response.erase(0, string_response.find(",")+1);
                double Ki = stod(string_response.substr(0, string_response.find(",")));
                Ki = (Ki/1023)*1;
                string_response.erase(0, string_response.find(",")+1);
                double Kd = stod(string_response);
                Kd = 1+(Kd/1023)*1;
                //std::cout << "Kd:";
                pid_constants[0] = Kp;
                pid_constants[1] = Ki;
                pid_constants[2] = Kd;
               // std::cout << pid_constants[2] << std::endl;
                
                if (print_data.mutex.try_lock()) {
                  if (print_data.has_data) {
                    //print_vector(print_data.data);
                    double value = print_data.data;
		                _dataTail = graph_update1(value, _dataTail, vertexDataBuffer);
                    CreateAndDrawGrid1(_dataTail, vertexBufferArray, programID, MatrixID, MVP);
                    //std::cout << print_data.data << std::endl;
                    print_data.has_data = false;
                  }
                  print_data.mutex.unlock();
                }
                // Swap buffers
                glfwSwapBuffers(window);
		            glfwPollEvents();
                
              }
            }
          } // end while running
        }
      }
    }
     // saveScreenshotToFile("test1.tga", 1024, 768);
      glDisableVertexAttribArray(0);	// unbind 
	    glDeleteBuffers(1, &vertexDataBuffer); // delete buffer
  });

  try {
    // Connect to robot.
    franka::Robot robot(argv[1]);
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
    started = true;
    
    // Set additional parameters always before the control loop, NEVER in the control loop!
    // Set collision behavior.
    robot.setCollisionBehavior(
        {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}}, {{20.0, 20.0, 18.0, 18.0, 16.0, 14.0, 12.0}},
        {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}}, {{20.0, 20.0, 20.0, 25.0, 25.0, 25.0}});

    // Load the kinematics and dynamics model.
    franka::Model model = robot.loadModel();

    // Define callback function to send Cartesian pose goals to get inverse kinematics solved.
    auto cartesian_pose_callback = [=, &time, &acceleration_limit, &jerk_limit, &last_force, &velocity_array, &print_data, &desired_force, &error, &past_error, &integral, &derivative, &vel_desired, &acc, &jerk, &pd, &pid_constants, &running]
      (const franka::RobotState& robot_state, franka::Duration period) -> franka::JointVelocities {

      double dt = 0.001;//period.toSec();
      //Guess a dt if no dt, to make sure we don't divide by 0
      if (!dt) { 
        dt = 0.001;
      }
      //Get forces on the flange
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> force_input(robot_state.tau_ext_hat_filtered.data());

      //Get last commanded velocity (commanded or actually performed?)
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> joint_pos(robot_state.q.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> vel_commanded_previous(robot_state.dq_d.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> acc_commanded_previous(robot_state.ddq_d.data());

      //Set past error from the previous loop iteration
      past_error = error;

      //Calculate new error between input force and desired force
      //error.head(3) << desired_force - force_input;   //makes wierd eigen error
      error = desired_force - (force_input - acc_commanded_previous);
      
      //Run a lowpass filter on the input signal before the pid controller
      error(0,0) = franka::lowpassFilter(dt, error(0,0), past_error(0,0), pid_constants[2]);
      error(1,0) = franka::lowpassFilter(dt, error(1,0), past_error(1,0), 1);
      error(2,0) = franka::lowpassFilter(dt, error(2,0), past_error(2,0), 1);
      

      integral += error * dt;
      derivative = (error - past_error) / dt;

      pd = pid_constants[0] * error + pid_constants[1] * integral + pid_constants[2] * derivative;
      acc = pd;
      //acc(0,0) = error[0] - pid_constants[0]*joint_pos[0] - pid_constants[1]*vel_commanded_previous[0];
      //vel_desired = pd;
      //acc = (vel_desired - vel_commanded_previous) / dt;
      jerk = (acc - acc_commanded_previous) / dt;
      jerk.cwiseMin(jerk_limit); //limit the jerk
      jerk.cwiseMax(-jerk_limit); //limit the jerk

      acc = acc_commanded_previous + jerk*dt;
      acc.cwiseMin(acceleration_limit); //limit acceleration
      acc.cwiseMax(-acceleration_limit); //limit acceleration

      vel_desired = vel_commanded_previous + acc * dt;
      
      //franka::JointVelocities velocity_desired = {{vel_desired[0], vel_desired[1], vel_desired[2], vel_desired[3], vel_desired[4], vel_desired[5], vel_desired[6]}};
      franka::JointVelocities velocity_desired = {{vel_desired[0], 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};

      if (print_data.mutex.try_lock()) {
        print_data.has_data = true;
        print_data.data = error[0];
        print_data.mutex.unlock();
      }
      
      return velocity_desired;
    };
    // Start real-time control loop.
    robot.control(cartesian_pose_callback);
    
  } catch (const franka::Exception& ex) {
   // saveScreenshotToFile("2019-02-05test.tga", 1024, 768);
    running = false;
    //Print data to output file
    /*
    outputfile << "vel,acc\n";
    for (int i = 0; i < velocity_array.size(); i++) {
      outputfile << velocity_array.at(i)[0] << "," << velocity_array.at(i)[1] << "\n";
    }
    std::cout << "Printed data to file" << std::endl;
    */
    std::cerr << ex.what() << std::endl;
  }

  if (print_thread.joinable()) {
    print_thread.join();
  }
  outputfile.close();
  return 0;
}