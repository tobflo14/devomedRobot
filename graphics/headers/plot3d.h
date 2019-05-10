#ifndef PLOT3D_H
#define PLOT3D_H

//#include <epoxy/gl.h>

//#include <glm/gtc/matrix_transform.hpp>
#include <GL/glew.h>
#include <GLFW/glfw3.h>


#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>




class Plot3d {
    public:
        Plot3d();
        void initWindow();
        void initArea();
        void loadOBJ();
        void loadModel();
        void drawModel();
        void realize();
        void unrealize();

        void cleanupModel();
        void loadTexture();
        void swapBuffers();
        void keyUp();
        void updateModel(int keyval);
        bool openWindow;

    private:
        void init();
        int WINDOW_WIDTH;
        int WINDOW_HEIGHT;
        GLFWwindow* window;
        

        GLuint VertexArrayID;
        GLuint vertexbuffer;
        GLuint uvbuffer;
        GLuint normalbuffer;
        GLuint programID;
        GLuint MatrixID;
        GLuint ViewMatrixID;
        GLuint ModelMatrixID;
        GLuint LightID;
        GLuint TextureID;
        GLuint Texture;
        glm::mat4 MVP;
        glm::mat4 ProjectionMatrix;
        glm::mat4 ViewMatrix;
        glm::mat4 ModelMatrix;
        glm::vec3 lightPos;
        int num_model_vertices;

     
};

#endif