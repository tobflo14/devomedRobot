#include "plot3d.h"

//#include <gtkmm.h>
#include "common/headers/shader.hpp"
#include "OBJ_Loader.h"
#include "common/headers/texture.hpp"
#include "common/headers/controls.h"


objl::Loader loader;

Plot3d::Plot3d(){
    init();
	loadOBJ();
}

void Plot3d::init(){
	
    this->WINDOW_WIDTH = 1024;
    this->WINDOW_HEIGHT = 768;
}

void Plot3d::initWindow(){
    // Initialise GLFW
	if (!glfwInit())
	{
		fprintf(stderr, "Failed to initialize GLFW\n");
		getchar();
	}

    glfwWindowHint(GLFW_SAMPLES, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	// Open a window and create its OpenGL context
    
	window = glfwCreateWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "Devomed", NULL, NULL);
	if (window == NULL) {
		fprintf(stderr, "Failed to open GLFW window. If you have an Intel GPU, they are not 3.3 compatible. Try the 2.1 version of the tutorials.\n");
		getchar();
		glfwTerminate();
		return;
	}
	glfwMakeContextCurrent(window);

	// Ensure we can capture the escape key being pressed below
	glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);

	// Set the mouse at the center of the screen
	glfwPollEvents();
	//glfwSetCursorPos(window, 1024 / 2, 768 / 2);
}


void Plot3d::initArea() {
    // Initialize GLEW
	glewExperimental = true; // Needed for core profile
	if (glewInit() != GLEW_OK) {
		fprintf(stderr, "Failed to initialize GLEW\n");
		getchar();
		glfwTerminate();
		return;
	}

	glClearColor(0.0f, 0.0f, 0.4f, 0.0f);

	// Enable depth test
	glEnable(GL_DEPTH_TEST);

	// Accept fragment if it closer to the camera than the former one
	glDepthFunc(GL_LESS);

	// Cull triangles which normal is not towards the camera
	glEnable(GL_CULL_FACE);
 
 	glGenVertexArrays(1, &VertexArrayID);
	glBindVertexArray(VertexArrayID);

	programID = LoadShaders("StandardShading.vertexshader", "StandardShading.fragmentshader");

	// Get a handle for our "MVP" uniform
	MatrixID = glGetUniformLocation(programID, "MVP");
	ViewMatrixID = glGetUniformLocation(programID, "V");
	ModelMatrixID = glGetUniformLocation(programID, "M");	
}

void Plot3d::realize() {
	//loadOBJ();
	initArea();
	loadModel();
}

void Plot3d::unrealize(){

}

void Plot3d::loadOBJ() {
	loader.LoadFile("panda_franka.obj");
}

void Plot3d::loadModel() {
//	loadOBJ();
	loadTexture();

	glBindVertexArray(VertexArrayID);
	std::cout << "loading model" << std::endl;
	

	size_t total_vertices = loader.LoadedVertices.size();
	size_t cur_total_vertices = 0;
	//std::cout << total_vertices << std::endl;
	std::vector<glm::vec3> vertices(total_vertices);
	std::vector<glm::vec2> uvs(total_vertices);
	std::vector<glm::vec3> normals(total_vertices);
//	std::cout << loader.LoadedMeshes.size() << std::endl;
	for (int j = 0; j < loader.LoadedMeshes.size(); j++) {

		objl::Mesh mesh = loader.LoadedMeshes[j];
			
		size_t num_vertices = mesh.Vertices.size();
		
		for (int i = 0; i < num_vertices; i++) {
			vertices[cur_total_vertices + i] = glm::vec3(
                mesh.Vertices[i].Position.X, 
				mesh.Vertices[i].Position.Y, 
                mesh.Vertices[i].Position.Z);/*
                glm::vec3(
                (double)(rand()%100)/100, 
				(double)(rand()%100)/100, 
                (double)(rand()%100)/100);
                */
			 //  if (i < 10) {  std::cout << mesh.Vertices[i].Position.X << std::endl; }
                
			normals[cur_total_vertices + i] = glm::vec3(mesh.Vertices[i].Normal.X,
				mesh.Vertices[i].Normal.Y,
				mesh.Vertices[i].Normal.Z);

			uvs[cur_total_vertices + i] = glm::vec2(mesh.Vertices[i].TextureCoordinate.X,
				mesh.Vertices[i].TextureCoordinate.Y);
		}
		cur_total_vertices += num_vertices;
	}
		// load in buffers
	num_model_vertices = vertices.size();

//	GLuint vertexbuffer;
	glGenBuffers(1, &vertexbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
	glBufferData(GL_ARRAY_BUFFER, num_model_vertices * sizeof(glm::vec3), &vertices[0], GL_STATIC_DRAW);

	glGenBuffers(1, &normalbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, normalbuffer);
	glBufferData(GL_ARRAY_BUFFER, normals.size() * sizeof(glm::vec3), &normals[0], GL_STATIC_DRAW);

	glGenBuffers(1, &uvbuffer);
	glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
	glBufferData(GL_ARRAY_BUFFER, uvs.size() * sizeof(glm::vec2), &uvs[0], GL_STATIC_DRAW);

	// Get a handle for our "LightPosition" uniform
	glUseProgram(programID);
	LightID = glGetUniformLocation(programID, "LightPosition_worldspace");

	int status;
  glGetProgramiv(programID, GL_LINK_STATUS, &status);

//	std::cout << "num model vertices " << num_model_vertices << std::endl;
//	} else {
//		std::cerr << "Model could not be loaded :(" << std::endl;
//	}
}

void Plot3d::drawModel() {
	// Clear the screen
	//glClearColor(1.0,1.0,1.0,0.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


	//glm::mat4 RotationMatrix = getRots;
	glm::mat4 TranslationMatrix = glm::mat4(1.0);
	glm::mat4 ScalingMatrix = glm::mat4(1.0);

	ProjectionMatrix = getProjectionMatrix();
	ViewMatrix = getViewMatrix();
	//ModelMatrix = glm::mat4(1.0);
	//ModelMatrix = TranslationMatrix * RotationMatrix * ScalingMatrix;
	ModelMatrix = getModelMatrix();
	MVP = ProjectionMatrix * ViewMatrix * ModelMatrix;
	
	
	glUseProgram(programID);


	// Send our transformation to the currently bound shader, 
	// in the "MVP" uniform
	glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVP[0][0]);
	glUniformMatrix4fv(ModelMatrixID, 1, GL_FALSE, &ModelMatrix[0][0]);
	glUniformMatrix4fv(ViewMatrixID, 1, GL_FALSE, &ViewMatrix[0][0]);

	lightPos = glm::vec3(4, 4, 4);
	glUniform3f(LightID, lightPos.x, lightPos.y, lightPos.z);

	glBindVertexArray(VertexArrayID);
	// Bind our texture in Texture Unit 0
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, Texture);
	// Set our "myTextureSampler" sampler to use Texture Unit 0
	glUniform1i(TextureID, 0);

	// 1rst attribute buffer : vertices
	glEnableVertexAttribArray(0);
	glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
	glVertexAttribPointer(
		0,                  // attribute
		3,                  // size
		GL_FLOAT,           // type
		GL_FALSE,           // normalized?
		0,                  // stride
		(void*)0            // array buffer offset
	);

	// 2nd attribute buffer : UVs
	glEnableVertexAttribArray(1);
	glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
	glVertexAttribPointer(
		1,                                // attribute
		2,                                // size
		GL_FLOAT,                         // type
		GL_FALSE,                         // normalized?
		0,                                // stride
		(void*)0                          // array buffer offset
	);

	// 3rd attribute buffer : normals
	glEnableVertexAttribArray(2);
	glBindBuffer(GL_ARRAY_BUFFER, normalbuffer);
	glVertexAttribPointer(
		2,                                // attribute
		3,                                // size
		GL_FLOAT,                         // type
		GL_FALSE,                         // normalized?
		0,                                // stride
		(void*)0                          // array buffer offset
	);

// Draw the triangles !
	//std::cout << "ready to draw some triangles: " << num_model_vertices << std::endl;
	glDrawArrays(GL_TRIANGLES, 0, num_model_vertices);

	glDisableVertexAttribArray(0);
	glDisableVertexAttribArray(1);
	glDisableVertexAttribArray(2);
	glBindVertexArray(0);
	glUseProgram(0);

	glFlush();


    openWindow = glfwGetKey(window, GLFW_KEY_ESCAPE ) != GLFW_PRESS && glfwWindowShouldClose(window) == 0;
}

void Plot3d::swapBuffers() {
	glfwSwapBuffers(window);
	glfwPollEvents();
}


void Plot3d::cleanupModel(){
	// Cleanup VBO and shader
	glDeleteBuffers(1, &vertexbuffer);
	glDeleteBuffers(1, &uvbuffer);
	glDeleteBuffers(1, &normalbuffer);
	glDeleteProgram(programID);
	glDeleteTextures(1, &Texture);
	glDeleteVertexArrays(1, &VertexArrayID);

	// Close OpenGL window and terminate GLFW
    
	//glfwTerminate(); // gives segmentation fault, unkown why
}

void Plot3d::loadTexture(){
	// Load the texture
	Texture = loadDDS("uvmap.DDS");

	// Get a handle for our "myTextureSampler" uniform
	TextureID = glGetUniformLocation(programID, "myTextureSampler");
}

void Plot3d::updateModel(int keyval){
    computeMatricesFromKeyInputs(keyval);
}