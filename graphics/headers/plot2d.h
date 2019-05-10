#ifndef PLOT2D_H
#define PLOT2D_H

//#include "functions.h"

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <GLFW/glfw3.h>
//using namespace glm;
typedef std::pair<double, double> Point;

class Plot2d {
    public:
        Plot2d(int samples_per_frame, float y_range, int number_of_plots);
        void initGLFW();
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
        bool gl_draw();
        static void gl_init ();
        void init_buffers();
        void init_shaders();
        void realize();
        void unrealize();
        void loadTexture();
        
        bool openWindow;
        

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

        GLFWwindow* window;

        // buffers
        GLuint dataArray;
        GLuint dataBuffer;
        GLuint colorbuffer;
        GLuint modelbuffer;
        GLuint programID;
        GLuint MatrixID;
        glm::mat4 MVP;
        
	    GLint uniform_color;

        GLuint vao;
        GLuint program;
        GLuint position_index;
        GLuint color_index;

        GLuint LightID;
        glm::vec3 lightPos;
        GLuint TextureID;
        GLuint Texture;
    
        glm::mat4 ProjectionMatrix;
        glm::mat4 ViewMatrix;
        glm::mat4 ModelMatrix;
        
        

};

#endif