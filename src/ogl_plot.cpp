// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <array>
#include <atomic>
#include <cmath>
#include <functional>
#include <fstream>
#include <iterator>
#include <mutex>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/rate_limiting.h>
#include <franka/robot.h>

#include <thread>
#include <iostream>

#include "examples_common.h"

#include "functions.h"

// ------------- OpenGL-stuff ------------------------
// Include standard headers
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

// Include GLEW
#include <GL/glew.h>

// Include GLFW
#include <GLFW/glfw3.h>
GLFWwindow* window;

// Include GLM
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
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
GLuint vertexArrayGrid;
GLuint vertexBufferGrid;
/*
static const GLfloat color_data[] = {
  1.0f, 0.0f, 0.0f,
  1.0f, 0.0f, 0.0f,
  1.0f, 0.0f, 0.0f,
  1.0f, 0.0f, 0.0f,
  1.0f, 0.0f, 0.0f,
  1.0f, 0.0f, 0.0f,
  1.0f, 0.0f, 0.0f,
  1.0f, 0.0f, 0.0f,
  0.0f, 1.0f, 0.0f,
  0.0f, 1.0f, 0.0f,
  0.0f, 1.0f, 0.0f,
  0.0f, 1.0f, 0.0f,
  0.0f, 1.0f, 0.0f,
  0.0f, 1.0f, 0.0f,
  0.0f, 1.0f, 0.0f,
  0.0f, 1.0f, 0.0f, 
  1.0f, 0.0f, 0.0f 
};
*/
GLuint colorbuffer;
GLuint colorbuffer2;
/*
void genColor(){
  glGenBuffers(1, &colorbuffer);
  glBindBuffer(GL_ARRAY_BUFFER, colorbuffer);
  glBufferData(GL_ARRAY_BUFFER, sizeof(color_data), color_data, GL_STATIC_DRAW);
  
  glEnableVertexAttribArray(1);
  glBindBuffer(GL_ARRAY_BUFFER, colorbuffer);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
}
*/
	
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

int main(int argc, char** argv) {
  // Check whether the required arguments were passed.
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }
  
  double vel_current[3] = {0.0};
  double vel_desired[3] = {0.0};
  double vel_desired_previous = 0.0;
  //double angle = 0.0;
  double time = 0.0;
  double zvalue = 0.0;
  double lastvalue = 0.0;
  unsigned long plotLength = 5000; //Max number of plotted points in a graph

double past_error[3] = {0.0};
double acc = 0.0;
double acc_previous = 0.0;
double jerk_limit = 400;
double Kp = 0.2;
double Ki = 0.4;
double Kd = -0.001;
double integral = 0.0;


  std::vector<Point> plot_Data1;
  std::vector<Point> plot_Data2;

  struct {
      std::mutex mutex;
      bool has_data;
      bool values_read;
      double printData;
      std::vector<Point> plotData1;
      std::vector<Point> simplifiedData1;
      std::vector<Point> simplifiedData2;
      std::vector<Point> plotData2;
  } plot_input{};

  std::atomic_bool running{true};
  std::atomic_bool started{false};


  // Start print thread.
  std::thread print_thread([&Kp, &Ki, &Kd, &plot_input, &running, plotLength, &started]() {

  started = false;
  initGLFW(MVP);
  testing();
 

  struct timespec ts, last_ts;
	timespec_get(&last_ts, TIME_UTC);
  generateBuffers();
	initializeBuffer(vertexBufferArray, vertexDataBuffer, 2); // Initializing
  fillColorBuffer(1.0f, 0.0f, 0.0f);
  genColor(colorbuffer);
 // fillColorBuffer(0.0f, 1.0f, 0.0f);
 // genColor(colorbuffer2);
  CreateGrid(vertexArrayGrid, vertexBufferGrid);
    while (running) {
		timespec_get(&ts, TIME_UTC);
		dt = ts.tv_sec - last_ts.tv_sec + (ts.tv_nsec - last_ts.tv_nsec) / 1E9;

		if (dt <= 0.05) continue; // limit the program to 20fps
		last_ts = ts;

    if (plot_input.mutex.try_lock()) {
        if (plot_input.has_data) {
          double value = plot_input.plotData1.at(plot_input.plotData1.size()-1).second;
          double value2 = plot_input.plotData2.at(plot_input.plotData2.size()-1).second;
         // std::cout << "value2: ";
         // std::cout << value2 << std::endl;
          _dataTail  = graph_update1(value, value2, _dataTail, vertexDataBuffer);
         // CreateAndDrawGrid1(_dataTail, vertexBufferArray, programID, MatrixID, MVP);
          DrawGrid(vertexArrayGrid);
          plot_input.has_data = false;
          plot_input.values_read = true;
          plot_input.plotData1.clear();
        }
        plot_input.mutex.unlock();
    }
		// update graph with new value
		//CreateAndDrawGrid();
    if (_dataTail == C_SAMPLES_PER_FRAME - 1) {
      std::cout << "screenshot being taken \n" << endl; 
      //saveScreenshotToFile("test.tga", 1024, 768);
    }
		// Swap buffers
		glfwSwapBuffers(window);
		glfwPollEvents();

    }  // end while-loop

    glDisableVertexAttribArray(0);	// unbind 
	  glDeleteBuffers(1, &vertexDataBuffer); // delete buffer
    glDeleteBuffers(1, &vertexBufferArray);
  });

  try {
    // Connect to robot.
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);
    
    // First move the robot to a suitable joint configuration
    //std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    std::array<double, 7> q_goal = {{0, M_PI_4, 0, -2 * M_PI_4, 0, 3 * M_PI_4, M_PI_4}};
    MotionGenerator motion_generator(0.5, q_goal);
  /*  std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore(); */

    robot.control(motion_generator);
    std::cout << "Finished moving to initial joint configuration." << std::endl;
    

    // Load the kinematics and dynamics model.
    franka::Model model = robot.loadModel();
   
    //std::array<double, 16> initial_pose;
    std::array<double, 6> initial_pose;
    started = true;

    // Define callback function to send Cartesian pose goals to get inverse kinematics solved.
    auto cartesian_pose_callback = [=, &time, &integral, &Kp, &Ki, &Kd, &acc, &acc_previous, &jerk_limit, &past_error, &lastvalue, &plot_input, &plot_Data1, &plot_Data2, &vel_current, &vel_desired, &vel_desired_previous, &running, &zvalue, &initial_pose]
      (const franka::RobotState& robot_state, franka::Duration period) -> franka::CartesianVelocities {
      // Update time.
      time += period.toSec();

      if (time == 0.0) {
        // Read the initial pose to start the motion from in the first time step.
        //initial_pose = robot_state.O_T_EE_c;
        initial_pose = robot_state.O_dP_EE_c;
      }

      //read force in z-direction on end effector
      double zforce = robot_state.K_F_ext_hat_K[2];
      double velz = robot_state.K_F_ext_hat_K[1];
      franka::CartesianVelocities velocity_desired = initial_pose;
      velocity_desired.O_dP_EE[0] = velocity_desired.O_dP_EE[1] = velocity_desired.O_dP_EE[3] = velocity_desired.O_dP_EE[4] = velocity_desired.O_dP_EE[5] = 0;
      velocity_desired.O_dP_EE[2] = 0;//vel_desired;
      
     // plot_Data1.push_back(Point(time, robot_state.tau_J[0]));
      plot_Data1.push_back(Point(time, zforce));
      plot_Data2.push_back(Point(time, velz));
      if (plot_input.mutex.try_lock()) {
        plot_input.has_data = true;
        plot_input.plotData1 = plot_Data1;
        plot_input.plotData2 = plot_Data2;
        plot_Data1.clear();
        plot_Data2.clear();
        plot_input.mutex.unlock();
        
      }

      return velocity_desired;
      //return pose_desired;
    };
    // Start real-time control loop.
    //robot.control(impedance_control_callback, cartesian_pose_callback);
    robot.control(cartesian_pose_callback);
    
  } catch (const franka::Exception& ex) {
    running = false;
    std::cerr << ex.what() << std::endl;
  }

  if (print_thread.joinable()) {
    print_thread.join();
  }

  return 0;
}
