cmake_minimum_required(VERSION 2.8.3)
project(asdf)

list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_SOURCE_DIR}/cmake)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_compile_options(-Wall -Wextra)

## Dependencies
find_package(Franka REQUIRED)
find_package(Poco REQUIRED COMPONENTS Net Foundation)
find_package(Eigen3 REQUIRED)
find_package(OpenGL REQUIRED) #ogl

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)



add_library(examples_common STATIC
  src/source/examples_common.cpp
)

add_subdirectory (externalGL)

include_directories(
	cmake/
	externalGL/glfw-3.1.2/include/
	externalGL/glm-0.9.7.1/
	externalGL/glew-1.13.0/include/
	src/headers/
	graphics/headers/
	.
)

target_link_libraries(examples_common PUBLIC Franka::Franka Eigen3::Eigen3) # la til eigentest

include_directories(${EIGEN3_INCLUDE_DIR})

set(EXAMPLES 
	#communication_test
  #lowpass_no_eigen
	#lowpass_filter_test
	#lowpass_filter_ogl
	#lowpass_filter_ogl_joint_not
	#lowpass_filter_ogl_joint
  lowpass_velocity_cruisetest
	#ogl_plot
  #control_loop
	#gnuplot
  #joint_impedance_control
)

ADD_CUSTOM_TARGET(copy_shader_dir ALL
COMMAND ${CMAKE_COMMAND} -E copy_directory
"${CMAKE_CURRENT_SOURCE_DIR}/src/shaders" "${CMAKE_BINARY_DIR}"
COMMENT "copy shaders to build directory"
VERBATIM)

foreach(example ${EXAMPLES})
  add_executable(${example} src/examples/${example}.cpp common/shader.cpp common/shader.hpp src/headers/pid.h src/source/pid.cpp
  	graphics/source/plot2d.cpp graphics/headers/plot2d.h graphics/source/plot2dTest.cpp graphics/headers/plot2dTest.h )
  target_link_libraries(${example} OpenGL  Franka::Franka
	Threads::Threads glfw GLEW_1130 examples_common Eigen3::Eigen3) 
  ADD_DEPENDENCIES(${example} copy_shader_dir)
endforeach()


#file(GLOB_RECURSE HEADERS
  #external/*.h
#  src/*.h
#  common/*.h
#)

## Tools
#file(GLOB_RECURSE SOURCES
#  src/*.cpp
#  src/graphics/*.cpp
#  common/*.cpp
#)
#file(GLOB_RECURSE HEADERS
#  include/*.h
#  src/*.h
#  src/graphics/*.h
#  common/*.h
#)




#if(NOT CMAKE_COMPILE_OBJECT)
#  set(CMAKE_C_COMPILE_OBJECT)
#      "<CMAKE_C_COMPILER> <DEFINES> <INCLUDES> <FLAGS> -o <OBJECT> -c <SOURCE>")
#endif()
