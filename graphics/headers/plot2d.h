#ifndef PLOT2D_H
#define PLOT2D_H

#include "functions.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <GLFW/glfw3.h>
using namespace glm;

class Plot2d {
    public:
        Plot2d(int samples_per_frame, float y_range, int number_of_plots);
        void initPlotWindow(); 
        void saveScreenshotToFile(std::string filename);
        void genColor(); 
        void initializeBuffer();
        void CreateGrid();
        void drawGrid();
        void graph_update(Point values[]);
        void drawGraph();
        void setValueLimits(int min, int max);
        void swapBuffers();
        void deleteBuffers();
        void draw_triangle();
        bool gl_draw();
        void init_buffers (GLuint  position_index,
              GLuint  color_index,
              GLuint *vao_out);
        static void gl_init ();
        void init_buffers();
        void init_shaders();
        void realize();

    private:
        void init();
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
        int WINDOW_WIDTH;
        int WINDOW_HEIGHT;
        int VERTEX_COORDINATE_COUNT;
        int buffersize;
        //GLuint gridColorBuffer;
        GLuint dataArray;
        GLuint dataBuffer;
        GLuint colorbuffer;
        GLuint programID;
        GLuint MatrixID;
        glm::mat4 MVP;
        GLFWwindow* window;
	    GLint uniform_color;
       // float mvp[16];
        glm::mat4 mvp;


        GLuint vao;
        GLuint program;
        GLuint mvp_location;
        GLuint position_index;
        GLuint color_index;
        

};

#endif