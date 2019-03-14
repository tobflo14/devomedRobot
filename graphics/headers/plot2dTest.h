#ifndef PLOT2DTEST_H
#define PLOT2DTEST_H

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <GLFW/glfw3.h>
using namespace glm;

class Plot2d {
    public:
        Plot2d(int samples_per_frame, float y_range);
        void init();
        void initPlotWindow(); 
        void saveScreenshotToFile(std::string filename, int windowWidth, int windowHeight);
        void genColor(GLuint colorbuffer);
        void fillColorBuffer(float red, float green, float blue); 
        void initializeBuffer(int numBuff);
        void CreateGrid();
        void drawGrid();
        void graph_update(double mainValue);
        void graph_update(double mainValue, double value2);
        void drawGraph();
        void setValueLimits(int min, int max);
        void swapBuffers();
        void deleteBuffers();

    private:
        float X_MARGIN_MAX;
        float X_MARGIN_MIN;
        float Y_MARGIN_MAX;
        float Y_MARGIN_MIN;
        float MIN_EXP_VALUE;
        float MAX_EXP_VALUE;
        int NUM_PLOTS;
        int SAMPLES_PER_FRAME;
        GLint INDEX_OF_LAST_ENTRY;
        float GRAPH_WIDTH;
        float GRAPH_HEIGHT;
        int VERTEX_COORDINATE_COUNT;
        int buffersize;
        //GLuint gridColorBuffer;
        GLuint dataArray;
        GLuint dataBuffer;
        GLuint programID;
        GLuint MatrixID;
        glm::mat4 MVP;
        GLFWwindow* window;
       // GLfloat color_data;
        GLuint colorbuffer;
        GLfloat color_data[3]; //hjelp, hvordan init riktig?

        //GLfloat xgrid[];
        //GLfloat xgrid_color[];
        //static GLfloat color_data[];
};

#endif