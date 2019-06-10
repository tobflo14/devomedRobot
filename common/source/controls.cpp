#include <iostream>

// Include GLFW
#include <GLFW/glfw3.h>
//extern GLFWwindow* window; // The "extern" keyword here is to access the variable "window" declared in tutorialXXX.cpp. This is a hack to keep the tutorials simple. Please avoid this.

// Include GLM
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
//using namespace glm;

#include "controls.h"

glm::mat4 ViewMatrix;
glm::mat4 ProjectionMatrix;
glm::mat4 ModelingMatrix;

glm::mat4 RotationMatrix;
glm::mat4 TranslationMatrix;
glm::mat4 ScalingMatrix; // scaling er vel ikke aktuelt her
float rotationY;

glm::mat4 getViewMatrix(){
	return ViewMatrix;
}
glm::mat4 getProjectionMatrix(){
	return ProjectionMatrix;
}
glm::mat4 getModelMatrix(){
	return ModelingMatrix;
}

float getOrientationY(){
	return rotationY;
}

// Initial position : on +Z
glm::vec3 position;
// Initial horizontal angle : toward -Z
float horizontalAngle = 3.14f;
// Initial vertical angle : none
float verticalAngle = 0.0f;
float angle = 3.14f / 2.0f;
// Initial Field of View
float FoV = 45.0f;
float vector_length = 1.6;
float mod_angle_x = 0.0f;
float mod_angle_y = 5 * 3.14f / 4.0f;
float mod_angle_z = 0.0f;

float x = 0;
float y = 0.4;
float z = 0;
float mod_x = 0;
float mod_y = 0;
float mod_z = 0;




void computeMatricesFromInputs(GLFWwindow* window){

	// Direction : Spherical coordinates to Cartesian coordinates conversion
	glm::vec3 direction(
		cos(verticalAngle) * sin(horizontalAngle), 
		sin(verticalAngle),
		cos(verticalAngle) * cos(horizontalAngle)
	);
	
	// Right vector
	glm::vec3 right = glm::vec3(
		sin(horizontalAngle - 3.14f/2.0f), 
		0,
		cos(horizontalAngle - 3.14f/2.0f)
	);

	// Up vector
	glm::vec3 up = glm::cross( right, direction );

	// Move Up
	if (glfwGetKey( window, GLFW_KEY_UP ) == GLFW_PRESS){
		//std::cout << "Key_Up was pressed" << std::endl;
		y += 0.01f;
	}
	// Move down
	if (glfwGetKey( window, GLFW_KEY_DOWN ) == GLFW_PRESS){
		y -= 0.01f;
	}
	// Turn right
	if (glfwGetKey( window, GLFW_KEY_RIGHT ) == GLFW_PRESS){
		angle += 0.01;
	}
	// Turn left
	if (glfwGetKey( window, GLFW_KEY_LEFT ) == GLFW_PRESS){
		angle -= 0.01;
	}

    // Turn right
	if (glfwGetKey(window, GLFW_KEY_P) == GLFW_PRESS) {
	//	std::cout << "P was pressed" << std::endl;
		angle += 0.01;
	}

	// Zoom out
	if (glfwGetKey(window, GLFW_KEY_N ) == GLFW_PRESS) {
		vector_length += 0.02f;
	}

	// Zoom in
	if (glfwGetKey(window, GLFW_KEY_M) == GLFW_PRESS) {
		vector_length -= 0.02f;
	}
	x = vector_length * cos(angle);
	z = vector_length * sin(angle);

	position = glm::vec3(x, y, z);


	//OrientationY += (xrot, 1.0f, 1.0f);
	//glRotatef(xrot, 1.0f, 0.0f, 0.0f); // deprecated

	// Projection matrix : 45° Field of View, 4:3 ratio, display range : 0.1 unit <-> 100 units
	ProjectionMatrix = glm::perspective(glm::radians(FoV), 4.0f / 3.0f, 0.1f, 100.0f);
	// Camera matrix
	ViewMatrix       = glm::lookAt(
								position,           // Camera is here
								glm::vec3(0,y,0), // and looks here : at the same position, plus "direction"
								up                  // Head is up (set to 0,-1,0 to look upside-down)
						   );

	RotationMatrix = glm::eulerAngleXYZ(getOrientationY(), 0.0f, 0.0f);
	TranslationMatrix = glm::mat4(1.0);
	ScalingMatrix = glm::mat4(1.0);


	ModelingMatrix = TranslationMatrix * RotationMatrix * ScalingMatrix;


}


void computeMatricesFromKeyInputs(int keyval){

	// Direction : Spherical coordinates to Cartesian coordinates conversion
	glm::vec3 direction(
		cos(verticalAngle) * sin(horizontalAngle), 
		sin(verticalAngle),
		cos(verticalAngle) * cos(horizontalAngle)
	);
	
	// Right vector
	glm::vec3 right = glm::vec3(
		sin(horizontalAngle - 3.14f/2.0f), 
		0,
		cos(horizontalAngle - 3.14f/2.0f)
	);

	// Up vector
	glm::vec3 up = glm::cross( right, direction );

	// Move Up
	if (keyval == 119){
		//std::cout << "Key_Up was pressed" << std::endl;
		y -= 0.01f;
	}
	// Move down
	if (keyval == 115){
		y += 0.01f;
	}
	// Turn right
	if (keyval == 97){
		angle -= 0.03;
	}
	// Turn left
	if (keyval == 100){
		angle += 0.03;
	}

	// Zoom out
	if (keyval == 109) {
		vector_length += 0.02f;
	}

	// Zoom in
	if (keyval == 110) {
		vector_length -= 0.02f;
	}


	// Translate model left
	if (keyval == 108){
		mod_x += 0.02f;
	}
	
	if ( keyval == 106){
		mod_x -= 0.02f;
	}

	if (keyval == 105) {
		mod_y += 0.01f;
	}
	if (keyval == 107) {
		mod_y -= 0.01f;
	}

	// Rotation x-axis
	if (keyval == 118) {
		mod_angle_x += 0.01f;
	}
	// Rotation x-axis
	if (keyval == 98) {
		mod_angle_x -= 0.01f;
	}
	// Rotation y-axis
	if (keyval == 102) {
		mod_angle_y += 0.02f;
	}
	// Rotation y-axis
	if (keyval == 103) {
		mod_angle_y -= 0.02f;
	}
	// Rotation z-axis
	if (keyval == 114) {
		mod_angle_z += 0.01f;
	}
	// Rotation z-axis
	if (keyval == 116) {
		mod_angle_z -= 0.01f;
	}
	

	x = vector_length * cos(angle);
	z = vector_length * sin(angle);

	position = glm::vec3(x, y, z);


	//OrientationY += (xrot, 1.0f, 1.0f);
	//glRotatef(xrot, 1.0f, 0.0f, 0.0f); // deprecated

	// Projection matrix : 45° Field of View, 4:3 ratio, display range : 0.1 unit <-> 100 units
	ProjectionMatrix = glm::perspective(glm::radians(FoV), 16.0f / 9.0f, 0.1f, 100.0f);
	// Camera matrix
	ViewMatrix       = glm::lookAt(
								position,           // Camera is here
								glm::vec3(0,y,0), // and looks here : at the same position, plus "direction"
								up                  // Head is up (set to 0,-1,0 to look upside-down)
						   );

	RotationMatrix = glm::eulerAngleXYZ(mod_angle_x, mod_angle_y, mod_angle_z);
	TranslationMatrix = glm::translate(glm::mat4(1.0), glm::vec3(mod_x, mod_y, mod_z));
	ScalingMatrix = glm::mat4(1.0);


	ModelingMatrix = TranslationMatrix * RotationMatrix * ScalingMatrix;


}