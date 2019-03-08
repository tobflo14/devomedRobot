#ifndef PLOT2D_H
#define PLOT2D_H

void saveScreenshotToFile(std::string filename, int windowWidth, int windowHeight);

void genColor(GLuint colorbuffer);

void fillColorBuffer(float red, float green, float blue);

void initGLFW(glm::mat4 MVP); 

void initializeBuffer(GLuint vertexBufferArray, GLuint vertexDataBuffer, int numBuff);

void CreateGrid(GLuint vertexArray, GLuint dataBuffer);

void DrawGrid(GLuint vertexArray);

//void DrawGrid(GLuint vertexArray, GLuint programID, GLuint MatrixID, glm::mat4 MVP);

GLint graph_update1(double mainValue, int _dataTail, GLuint vertexBuffer);

GLint graph_update1(double mainValue, double value2, int _dataTail, GLuint vertexBuffers);

void CreateAndDrawGrid1(int _dataTail, GLuint vertexBufferArray, GLuint programID, GLuint MatrixID, glm::mat4 MVP);

void setValueLimits(int min, int max);

#endif