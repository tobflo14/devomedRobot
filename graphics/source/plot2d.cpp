// OpenGL-stuff
// Include standard headers
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <iostream>
#include <cstring> // string function definitions

// Include GLEW
#include <GL/glew.h>

// Include GLFW
#include <GLFW/glfw3.h>
//GLFWwindow* window1;

// Include GLM
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
using namespace glm;

#include <common/shader.hpp>
#include "plot2d.h"

static int C_NUM_Y_DIV = 3;
static float C_X_MARGIN_MAX = 0.9999f;
static float C_X_MARGIN_MIN = -0.9999f;
static float C_Y_MARGIN_MAX = 0.9999f;
static float C_Y_MARGIN_MIN = -0.9999f;

static float C_MIN_LSB = -3;
static float C_MAX_LSB = 3;
const int C_SUB_PLOTS = 3;
const int C_SAMPLES_PER_FRAME = 4000;
static float C_Y_STEP = (C_Y_MARGIN_MAX - C_Y_MARGIN_MIN) / C_NUM_Y_DIV;
static float C_GRAPH_WIDTH = C_X_MARGIN_MAX - C_X_MARGIN_MIN;
static float C_GRAPH_HEIGHT = C_Y_MARGIN_MAX - C_Y_MARGIN_MIN;
const int VERTEX_COORDINATE_COUNT = 3;
int buffersize = sizeof(GLfloat)*C_SAMPLES_PER_FRAME*VERTEX_COORDINATE_COUNT;
GLuint gridColorBuffer;
// grid
//GLuint vertexBufferArrayGrid;
//GLuint vertexGridBuffer;
GLfloat xgrid[] = {
	-1.0f, 0.0f, 0.0f,
	 1.0f, 0.0f, 0.0f,
	-1.0f, 0.2f, 0.0f,
	 1.0f, 0.2f, 0.0f, 
};
GLfloat xgrid_color[] = {
	 1.0f, 1.0f, 1.0f,
	 1.0f, 1.0f, 1.0f,
	 0.0f, 1.0f, 0.0f,
	 0.0f, 1.0f, 0.0f, 
};
static GLfloat color_data[C_SAMPLES_PER_FRAME*3*2];

//GLuint vertexBufferArray;
//GLuint vertexbuffer;
//GLuint vertexDataBuffer;







/*
 * Saves a screenshot to .tga file. Instant copy of pixels in window
 */
void saveScreenshotToFile(std::string filename, int windowWidth, int windowHeight) {
  const int numberOfPixels = windowWidth*windowHeight*3;
  unsigned char pixels[numberOfPixels];

  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  glReadBuffer(GL_FRONT);
  glReadPixels(0,0, windowWidth, windowHeight, GL_BGR_EXT, GL_UNSIGNED_BYTE, pixels);

	std::string filepath = "images/";
//	std::string files = std::strcat(filepath.c_str(), filename.c_str());
	std::string files = filepath.c_str() + filename;
 	FILE *outputFile = fopen(files.c_str(), "w");
  //FILE *outputFile = fopen(filename.c_str(), "w");
  short header[] = {0,2,0,0,0,0, (short) windowWidth, (short) windowHeight, 24};

  fwrite(&header, sizeof(header), 1, outputFile);
  fwrite(pixels, numberOfPixels, 1, outputFile);
  fclose(outputFile);

  std::cout << "Finish writing screenshot to file \n" << std::endl;
}

void fillColorBuffer(float red, float green, float blue){
	for (int v=0; v<C_SAMPLES_PER_FRAME; v++){
		color_data[3*v + 0] = red;
		color_data[3*v+1] = green;
		color_data[3*v + 2] = blue;
	}
	for (int v=C_SAMPLES_PER_FRAME; v<2*C_SAMPLES_PER_FRAME; v++){
		color_data[3*v + 0] = red-1.0f;
		color_data[3*v+1] = green+1.0f;
		color_data[3*v + 2] = blue;
	}
}

void genColor(GLuint colorbuffer){
  glGenBuffers(1, &colorbuffer);
  glBindBuffer(GL_ARRAY_BUFFER, colorbuffer);
  glBufferData(GL_ARRAY_BUFFER, sizeof(color_data), color_data, GL_STATIC_DRAW);
  
  glEnableVertexAttribArray(1);
  glBindBuffer(GL_ARRAY_BUFFER, colorbuffer);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);
}


void initGLFW(glm::mat4 MVP) {
    // Initialise GLFW
	if (!glfwInit())
	{
		fprintf(stderr, "Failed to initialize GLFW\n");
		getchar();
		//return -1;
	}

	glfwWindowHint(GLFW_SAMPLES, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	// Projection matrix : 45� Field of View, 4:3 ratio, display range : 0.1 unit <-> 100 units
	glm::mat4 Projection = glm::perspective(glm::radians(45.0f), 4.0f / 3.0f, 0.1f, 100.0f);
	// Camera matrix
	glm::mat4 View = glm::lookAt(
		glm::vec3(4, 4, -5), // Camera is at (4,4,-5), in World Space
		glm::vec3(0, 0, 0), // and looks at the origin
		glm::vec3(0, 1, 0)  // Head is up (set to 0,-1,0 to look upside-down)
	);
	// Model matrix : an identity matrix (model will be at the origin)
	glm::mat4 Model = glm::mat4(1.0f);
	// Our ModelViewProjection : multiplication of our 3 matrices
	MVP = Projection * View * Model; // Remember, matrix multiplication is the other way around
	
}

/*
 * Initialise buffer
 */
void initializeBuffer(GLuint vertexBufferArray, GLuint vertexDataBuffer, int numBuff)
{
//  glewExperimental = true;
	glBindVertexArray(vertexBufferArray);

	glBindBuffer(GL_ARRAY_BUFFER, vertexDataBuffer);
	glBufferData(GL_ARRAY_BUFFER, buffersize*numBuff, NULL, GL_DYNAMIC_DRAW);
	glVertexAttribPointer(
		0,                        // attribute. No particular reason for 0, but must match the layout in the shader.
		VERTEX_COORDINATE_COUNT,  // size (3 coordinates)
		GL_FLOAT,           // type
		GL_FALSE,           // normalized?
		0,                  // stride
		(void*)0            // array buffer offset
	);
	glEnableVertexAttribArray(0);
}


void CreateGrid(GLuint vertexArray, GLuint dataBuffer) {
	//glGenBuffers(1, &vertexGridBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, dataBuffer);
	glBufferData(GL_ARRAY_BUFFER, sizeof (xgrid), xgrid, GL_STREAM_DRAW);
	
	//glGenVertexArrays(1, &vertexArray);
	glBindVertexArray(vertexArray);
	glEnableVertexAttribArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, dataBuffer);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, (void*) 0);

	glGenBuffers(1, &gridColorBuffer);
 	glBindBuffer(GL_ARRAY_BUFFER, gridColorBuffer);
  glBufferData(GL_ARRAY_BUFFER, sizeof(xgrid_color), xgrid_color, GL_STATIC_DRAW);
  
  glEnableVertexAttribArray(1);
  glBindBuffer(GL_ARRAY_BUFFER, gridColorBuffer);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, (void*)0);


}

void DrawGrid(GLuint vertexArray) {
		// Clear the screen
		/*
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Use our shader
	glUseProgram(programID);

	// Send our transformation to the currently bound shader, 
	// in the "MVP" uniform
	glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVP[0][0]);
*/
	glBindVertexArray(vertexArray);
	glDrawArrays(GL_LINES, 0, 4);
}

void DrawGrid(GLuint vertexArray, GLuint programID, GLuint MatrixID, glm::mat4 MVP) {
		// Clear the screen
		/*
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Use our shader
	glUseProgram(programID);

	// Send our transformation to the currently bound shader, 
	// in the "MVP" uniform
	glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVP[0][0]);
*/
	glBindVertexArray(vertexArray);
	glDrawArrays(GL_LINES, 0, 4);
}
/*
 * Updates the buffer with the new values
 */
GLint graph_update1(double mainValue, int _dataTail, GLuint vertexBuffers) {
  //vertexDataBuffer = vertexBuffers;
  glBindBuffer(GL_ARRAY_BUFFER, vertexBuffers);
  float value = (float) mainValue;
	// Calculate x and y position for mainValue and creates a 2D point
	GLfloat point[VERTEX_COORDINATE_COUNT] = {
		C_GRAPH_WIDTH * _dataTail / C_SAMPLES_PER_FRAME -1.0f,
      	2 * (value - C_MIN_LSB) / (C_MAX_LSB - C_MIN_LSB) - 1.0f, // [1,-1]
		0.0f
	};

	// Put the point in current index of buffer
	float current_index_of_buffer = sizeof(GLfloat) * _dataTail * VERTEX_COORDINATE_COUNT;
	float size_of_new_data = sizeof(GLfloat) * VERTEX_COORDINATE_COUNT;
	glBufferSubData(GL_ARRAY_BUFFER, current_index_of_buffer,
		size_of_new_data, point);
	
	// Increment buffer position, start writing to the start of the buffer again when the end is reached
	_dataTail = (_dataTail + 1) % (C_SAMPLES_PER_FRAME);
    return _dataTail;
}

/*
 * Updates the buffer with the new values
 */
GLint graph_update1(double mainValue, double value2, int _dataTail, GLuint vertexBuffers) {
	graph_update1(mainValue, _dataTail, vertexBuffers);
	glBindBuffer(GL_ARRAY_BUFFER, vertexBuffers);

	// Put the point in current index of buffer
	float current_index_of_buffer = sizeof(GLfloat) * _dataTail * VERTEX_COORDINATE_COUNT;
	float size_of_new_data = sizeof(GLfloat) * VERTEX_COORDINATE_COUNT;

	float value = (float) value2;
	// Calculate x and y position for mainValue and creates a 2D point
	GLfloat point2[VERTEX_COORDINATE_COUNT] = {
		C_GRAPH_WIDTH * _dataTail / C_SAMPLES_PER_FRAME -1.0f,
    2 * (value - C_MIN_LSB) / (C_MAX_LSB - C_MIN_LSB)-1.0f, // [1,-1]
		0.0f
	};

	// Put the point in current index of buffer
	glBufferSubData(GL_ARRAY_BUFFER, current_index_of_buffer + buffersize,
		size_of_new_data, point2);
	
	// Increment buffer position, start writing to the start of the buffer again when the end is reached
	_dataTail = (_dataTail + 1) % (C_SAMPLES_PER_FRAME);
    return _dataTail;
}



/*
 * Clear screen, send transformation through MVP matrix and draw content in buffer
 */
void CreateAndDrawGrid1(int _dataTail, GLuint vertexBufferArray, GLuint programID, GLuint MatrixID, glm::mat4 MVP)
{
  	// Clear the screen
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Use our shader
	glUseProgram(programID);

	// Send our transformation to the currently bound shader, 
	// in the "MVP" uniform
	glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVP[0][0]);

	glBindVertexArray(vertexBufferArray);

	//glUniform1f(1, C_GRAPH_WIDTH - C_GRAPH_WIDTH * _dataTail / C_SAMPLES_PER_FRAME ); // hva gjør egentlig dette ? ser ingen forskjell?
	glDrawArrays(GL_LINE_STRIP, 0, _dataTail);
	glDrawArrays(GL_LINE_STRIP, C_SAMPLES_PER_FRAME, _dataTail);
	//glUniform1f(1, -C_GRAPH_WIDTH * _dataTail / C_SAMPLES_PER_FRAME);
	//glDrawArrays(GL_LINE_STRIP, _dataTail, C_SAMPLES_PER_FRAME - _dataTail);
}

void setValueLimits(int min, int max) {
	C_MIN_LSB = min;
	C_MAX_LSB = max;
}