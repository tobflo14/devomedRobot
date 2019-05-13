#ifndef CONTROLS_HPP
#define CONTROLS_HPP

#include <glm/gtx/euler_angles.hpp>

void computeMatricesFromInputs(GLFWwindow* window);
void computeMatricesFromKeyInputs(int keyval);
glm::mat4 getViewMatrix();
glm::mat4 getProjectionMatrix();
glm::mat4 getModelMatrix();
float getOrientationY();

#endif