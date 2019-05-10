#ifndef CONTROLS_HPP
#define CONTROLS_HPP

void computeMatricesFromInputs(GLFWwindow* window);
void computeMatricesFromKeyInputs(int keyval);
glm::mat4 getViewMatrix();
glm::mat4 getProjectionMatrix();
float getOrientationY();

#endif