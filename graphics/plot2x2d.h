#ifndef PLOT2D_H
#define PLOT2D_H

void saveScreenshotToFile1(std::string filename, int windowWidth, int windowHeight);

void initGLFW(glm::mat4 MVP); 

void initializeBuffer(GLuint vertexBufferArray, GLuint vertexDataBuffer);

GLint graph_update1(double mainValue, int _dataTail, GLuint vertexBuffer, int id);

void CreateAndDrawGrid1(int _dataTail, GLuint vertexBufferArray, GLuint programID, GLuint MatrixID, glm::mat4 MVP);

#endif