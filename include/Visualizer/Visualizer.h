#ifndef UTILITIES_VISUALIZER_H
#define UTILITIES_VISUALIZER_H
#include "Data/Cloud.h"

#include <iostream>
#include <memory>
#include <fstream>
#include <string>
#include <iomanip>
#include <vector>

// Include GLEW
#include <GL/glew.h>

// Include GLFW
#include <glfw3.h>

// Include GLM
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtx/euler_angles.hpp>
#include <glm/gtx/norm.hpp>

// Include AntTweakBar
#include <AntTweakBar.h>

//Include helper functions from opengl-tutorial.com
#include "Visualizer/shader.hpp"
#include "Visualizer/texture.hpp"
#include "Visualizer/controls.hpp"
#include "Visualizer/objloader.hpp"
#include "Visualizer/vboindexer.hpp"
#include "Visualizer/quaternion_utils.hpp" // See quaternion_utils.cpp for RotationBetweenVectors, LookAt and RotateTowards

class Visualizer{

public:
    /// Pointer
    typedef std::unique_ptr<Visualizer> Ptr;
    Visualizer(int windowWidth, int windowHeight, const char* windowName);
    Visualizer();
    ~Visualizer();

    int initialize(int windowWidth, int windowHeight, const char* windowName);
    int loadVertices(const char *imagePath, const char *objPath);
    int loadPoints(const char* imagePath, const PointCloud& inputPointCloud);
    int creatSphere(const char* imagePath, float radius, int resolution);
    int creatCircle(const char* imagePath, float radius, int resolution);

    int renderPoints(const PointCloud& inputPointCloud);
    int render();
    int close();
    int checkClose();
    

protected:

private:
    template<typename T>
    int loadToVBO(const std::vector<glm::vec3> &vertices, const std::vector<glm::vec2> &uvs, const std::vector<glm::vec3> &normals, std::vector<T> &indices, GLuint &v, GLuint &u, GLuint &n, GLuint &e);
    int bindBuffers(GLuint &Texture, GLuint &TextureID, GLuint &vertexbuffer, GLuint &uvbuffer, GLuint &normalbuffer, GLuint &elementbuffer);

    GLFWwindow* window;

    // Create and compile our GLSL program from the shaders
    GLuint programID;

    //AntiTweak

    GLuint MatrixID;
    GLuint ViewMatrixID;
    GLuint ModelMatrixID;
    //position and orientation of objects in a scene
    glm::vec3 gPosition1;
    glm::vec3 gOrientation1;
    glm::vec3 gPosition2;
    glm::quat gOrientation2;
    bool gLookAtOther;

    // Get a handle for our buffers
    GLuint vertexPosition_modelspaceID;
    GLuint vertexUVID;
    GLuint vertexNormal_modelspaceID;

    // Load the texture
    GLuint TexturePoints;

    // Get a handle for our "myTextureSampler" uniform
    GLuint TextureIDPoints;
    GLuint vertexbufferPoints;
    GLuint uvbufferPoints;
    GLuint normalbufferPoints;
    // Buffer for the indices as well
    GLuint elementbufferPoints;

    // Load the texture
    GLuint TextureSphere;
    // Get a handle for our "myTextureSampler" uniform
    GLuint TextureIDSphere;
    GLuint vertexbufferSphere;
    GLuint uvbufferSphere;
    GLuint normalbufferSphere;

    // Buffer for the indices as well
    GLuint elementbufferSphere;

    // Load the texture
    GLuint TextureShort;
    // Get a handle for our "myTextureSampler" uniform
    GLuint TextureIDShort;
    GLuint vertexbufferShort;
    GLuint uvbufferShort;
    GLuint normalbufferShort;

    // Buffer for the indices as well
    GLuint elementbufferShort;

    // Get a handle for our "LightPosition" uniform
    GLuint LightID;

    std::vector<unsigned int> indicesPoints;
    std::vector<unsigned short> indicesShort;
    std::vector<unsigned short> indicesSphere;
    //For measuring frame rate
    double lastTime;
    double lastFrameTime;
    int nbFrames;

};


#endif /* UTILITIES_VISUALIZER_H */
