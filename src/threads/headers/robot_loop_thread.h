#ifndef ROBOT_LOOP_THREAD_H
#define ROBOT_LOOP_THREAD_H

#include <franka/robot.h>
#include <franka/robot_state.h>
#include <franka/exception.h>


void* RobotLoopThread(void* arg);

#endif