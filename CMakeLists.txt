cmake_minimum_required(VERSION 2.8.3)
project(asdf)

list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_SOURCE_DIR}/cmake)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE Release)

# add_compile_options(-Wall -Wextra -g)

## Dependencies
find_package(Franka REQUIRED)
find_package(Poco REQUIRED COMPONENTS Net Foundation)
find_package(Eigen3 REQUIRED)
find_package(OpenGL REQUIRED) #ogl

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

find_package(PkgConfig)
pkg_check_modules(GTKMM gtkmm-3.0)

add_library(examples_common STATIC
  src/functions/source/examples_common.cpp
)

add_subdirectory (externalGL)

include_directories(
	common/headers/
	src/functions/headers/
	src/threads/headers/
	cmake/
	externalGL/glfw-3.1.2/include/
	externalGL/glm-0.9.7.1/
	externalGL/glew-1.13.0/include/
	graphics/headers/
	.
	${EIGEN3_INCLUDE_DIR}
  ${GTKMM_INCLUDE_DIRS}
)

target_link_libraries(examples_common PUBLIC Franka::Franka Eigen3::Eigen3) # la til eigentest


set(EXAMPLES 
	#communication_test
  #lowpass_no_eigen
	#lowpass_filter_test
	#lowpass_filter_ogl
	#lowpass_filter_ogl_joint_not
	#lowpass_filter_ogl_joint
  #lowpass_velocity_cruisetest
	#ogl_plot
  #control_loop
	#gnuplot
	#joint_impedance_control
	main_start_threads
)

set(SOURCES
common/source/shader.cpp 
graphics/source/plot2d.cpp 
src/functions/source/pid.cpp
src/functions/source/functions.cpp
src/threads/source/robot_loop_thread.cpp
src/threads/source/calculate_velocity_thread.cpp
src/threads/source/plot_thread.cpp
src/threads/source/keyboard_input_thread.cpp
src/threads/source/gui_thread.cpp
src/threads/source/gui_thread_1.cpp
)

ADD_CUSTOM_TARGET(copy_shader_dir ALL
COMMAND ${CMAKE_COMMAND} -E copy_directory
"${CMAKE_CURRENT_SOURCE_DIR}/graphics/shaders" "${CMAKE_BINARY_DIR}"
COMMENT "copy shaders to build directory"
VERBATIM)

ADD_CUSTOM_TARGET(copy_glade_dir ALL
COMMAND ${CMAKE_COMMAND} -E copy_directory
"${CMAKE_CURRENT_SOURCE_DIR}/glade" "${CMAKE_BINARY_DIR}"
COMMENT "copy glade to build directory"
VERBATIM)

foreach(example ${EXAMPLES})
	add_executable(${example} src/apps/${example}.cpp ${SOURCES})
  target_link_libraries(${example} OpenGL  Franka::Franka
	Threads::Threads glfw GLEW_1130 examples_common Eigen3::Eigen3 ${GTKMM_LIBRARIES}) 
  ADD_DEPENDENCIES(${example} copy_shader_dir)
endforeach()


#file(GLOB_RECURSE HEADERS
  #external/*.h
#  src/*.h
#  common/*.h
#)

file(GLOB_RECURSE SOURCES
  src/*.cpp
  src/threads/source/*.cpp
  common/*.cpp
)




#if(NOT CMAKE_COMPILE_OBJECT)
#  set(CMAKE_C_COMPILE_OBJECT)
#      "<CMAKE_C_COMPILER> <DEFINES> <INCLUDES> <FLAGS> -o <OBJECT> -c <SOURCE>")
#endif()
