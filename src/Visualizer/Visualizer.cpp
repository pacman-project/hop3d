#include "Visualizer/Visualizer.h"

Visualizer::Visualizer() : FOV(45.0f), gCameraPosition(0.0f, 0.0f, 6.0f), gCameraOrientation(0.0f, 0.0f, 0.0f), gPosition1(0.0f, 0.0f, 0.0f), gPosition2( 1.5f, 0.0f, 0.0f), gLookAtOther(true) {
    initialize(1024,768,"windowName");
}

Visualizer::Visualizer(int windowWidth, int windowHeight, const char* windowName) : FOV(45.0f), gCameraPosition(0.0f, 0.0f, 6.0f), gCameraOrientation(0.0f, 0.0f, 0.0f), gPosition1(0.0f, 0.0f, 0.0f), gPosition2( 1.5f, 0.0f, 0.0f), gLookAtOther(true) {
    initialize(windowWidth,windowHeight,windowName);
}

Visualizer::~Visualizer(){

}

int Visualizer::initialize(int windowWidth, int windowHeight, const char* windowName){
    // Initialise GLFW
    if( !glfwInit() )
    {
        std::cerr << "Failed to initialize GLFW" << std::endl;
        return -1;
    }

    glfwWindowHint(GLFW_SAMPLES, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // Open a window and create its OpenGL context
    window = glfwCreateWindow( windowWidth, windowHeight, windowName, NULL, NULL);
    if( window == NULL ){
        std::cerr << "Failed to open GLFW window. If you have an Intel GPU, they are not 3.3 compatible. Try the 2.1 version of the tutorials." << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);

    // Initialize GLEW
    glewExperimental = true; // Needed for core profile
    if (glewInit() != GLEW_OK) {
        std::cerr << "Failed to initialize GLEW" << std::endl;
        return -1;
    }

    // Initialize the GUI
    TwInit(TW_OPENGL_CORE, NULL);
    TwWindowSize(windowWidth, windowHeight);
    TwBar * ObjectGUI = TwNewBar("Object pose");
    TwBar * CameraGUI = TwNewBar("Camera pose");
    std::stringstream oss;
    oss << 10 << " " << 10;
    TwSetParam(ObjectGUI, NULL, "refresh", TW_PARAM_CSTRING, 1, oss.str().c_str());
    oss.str("");
    oss << windowWidth-210 << " " << 10;
    std::cout << oss.str().c_str() << std::endl;
    TwSetParam(CameraGUI, NULL, "position", TW_PARAM_CSTRING, 1, oss.str().c_str());

    TwAddVarRW(ObjectGUI, "Euler X", TW_TYPE_FLOAT, &gOrientation1.x, "step=0.01");
    TwAddVarRW(ObjectGUI, "Euler Y", TW_TYPE_FLOAT, &gOrientation1.y, "step=0.01");
    TwAddVarRW(ObjectGUI, "Euler Z", TW_TYPE_FLOAT, &gOrientation1.z, "step=0.01");
    TwAddVarRW(ObjectGUI, "Pos X"  , TW_TYPE_FLOAT, &gPosition1.x, "step=0.1");
    TwAddVarRW(ObjectGUI, "Pos Y"  , TW_TYPE_FLOAT, &gPosition1.y, "step=0.1");
    TwAddVarRW(ObjectGUI, "Pos Z"  , TW_TYPE_FLOAT, &gPosition1.z, "step=0.1");

    TwAddVarRW(CameraGUI, "Position X", TW_TYPE_FLOAT, &gCameraPosition.x, "step=0.1");
    TwAddVarRW(CameraGUI, "Position Y", TW_TYPE_FLOAT, &gCameraPosition.y, "step=0.1");
    TwAddVarRW(CameraGUI, "Position Z", TW_TYPE_FLOAT, &gCameraPosition.z, "step=0.1");
    TwAddVarRW(CameraGUI, "FOV"  , TW_TYPE_FLOAT, &FOV, "step=0.5");

    TwAddVarRW(CameraGUI, "Receptive Fields", TW_TYPE_BOOL8 , &gLookAtOther, "help='Turning on/off RF?'");

    // Set GLFW event callbacks. I removed glfwSetWindowSizeCallback for conciseness
    glfwSetMouseButtonCallback(window, (GLFWmousebuttonfun)TwEventMouseButtonGLFW); // - Directly redirect GLFW mouse button events to AntTweakBar
    glfwSetCursorPosCallback(window, (GLFWcursorposfun)TwEventMousePosGLFW);          // - Directly redirect GLFW mouse position events to AntTweakBar
    glfwSetScrollCallback(window, (GLFWscrollfun)TwEventMouseWheelGLFW);    // - Directly redirect GLFW mouse wheel events to AntTweakBar
    glfwSetKeyCallback(window, (GLFWkeyfun)TwEventKeyGLFW);                         // - Directly redirect GLFW key events to AntTweakBar
    glfwSetCharCallback(window, (GLFWcharfun)TwEventCharGLFW);                      // - Directly redirect GLFW char events to AntTweakBar

    // Ensure we can capture the escape key being pressed below
    glfwSetInputMode(window, GLFW_STICKY_KEYS, GL_TRUE);
    glfwSetCursorPos(window, windowWidth/2, windowHeight/2);

    // Dark blue background
    glClearColor(0.0f, 0.0f, 0.4f, 0.0f);

    // Enable depth test
    glEnable(GL_DEPTH_TEST);
    // Accept fragment if it closer to the camera than the former one
    glDepthFunc(GL_LESS);

    // Cull triangles which normal is not towards the camera
    glEnable(GL_CULL_FACE);

    // Create and compile our GLSL program from the shaders
    programID = LoadShaders( "StandardShading.vertexshader", "StandardShading.fragmentshader" );

    // Get a handle for our "MVP" uniform
    MatrixID = glGetUniformLocation(programID, "MVP");
    ViewMatrixID = glGetUniformLocation(programID, "V");
    ModelMatrixID = glGetUniformLocation(programID, "M");
    // For speed computation
    lastTime = glfwGetTime();
    lastFrameTime = lastTime;
    nbFrames = 0;

    return 0;


}

int Visualizer::loadVertices(const char* imagePath, const char* objPath){
    // Get a handle for our buffers
    vertexPosition_modelspaceID = glGetAttribLocation(programID, "vertexPosition_modelspace");
    vertexUVID = glGetAttribLocation(programID, "vertexUV");
    vertexNormal_modelspaceID = glGetAttribLocation(programID, "vertexNormal_modelspace");

    // Load the texture
    TextureShort = loadDDS(imagePath);

    // Get a handle for our "myTextureSampler" uniform
    TextureIDShort  = glGetUniformLocation(programID, "myTextureSampler");

    // Read our .obj file
    std::vector<glm::vec3> vertices;
    std::vector<glm::vec2> uvs;
    std::vector<glm::vec3> normals;
    bool res = loadOBJ(objPath, vertices, uvs, normals);
    if (res) std::cout << "Object not loaded" << std::endl;

    std::vector<glm::vec3> indexed_vertices;
    std::vector<glm::vec2> indexed_uvs;
    std::vector<glm::vec3> indexed_normals;
    indexVBO(vertices, uvs, normals, indicesShort, indexed_vertices, indexed_uvs, indexed_normals);

    // Load it into a VBO
    loadToVBO(indexed_vertices,indexed_uvs,indexed_normals,indicesShort,vertexbufferShort,uvbufferShort,normalbufferShort, elementbufferShort);


    return 0;

}

int Visualizer::loadPoints(const char* imagePath, const hop3d::PointCloud& inputPointCloud){
    // Get a handle for our buffers
    vertexPosition_modelspaceID = glGetAttribLocation(programID, "vertexPosition_modelspace");
    vertexUVID = glGetAttribLocation(programID, "vertexUV");
    vertexNormal_modelspaceID = glGetAttribLocation(programID, "vertexNormal_modelspace");

    // Load the texture
    TexturePoints = loadDDS(imagePath);

    // Get a handle for our "myTextureSampler" uniform
    TextureIDPoints  = glGetUniformLocation(programID, "myTextureSampler");

    // Read our .obj file
    std::vector<glm::vec3> vertices;
    std::vector<glm::vec2> uvs;
    std::vector<glm::vec3> normals;
    for(unsigned int  i=0; i < inputPointCloud.pointCloudNormal.size(); i++ ){
        vertices.push_back(glm::vec3(inputPointCloud.pointCloudNormal[i].position(0), inputPointCloud.pointCloudNormal[i].position(1), inputPointCloud.pointCloudNormal[i].position(2)));
        normals.push_back(glm::vec3(inputPointCloud.pointCloudNormal[i].normal(0), inputPointCloud.pointCloudNormal[i].normal(1), inputPointCloud.pointCloudNormal[i].normal(2)));
        indicesPoints.push_back(i);
        uvs.push_back(glm::vec2(0.1f,0.1f));
    }
    // Load it into a VBO
    loadToVBO(vertices,uvs,normals,indicesPoints,vertexbufferPoints,uvbufferPoints,normalbufferPoints,elementbufferPoints);

    return 0;

}


int Visualizer::createSphere(const char* imagePath, double radius, int resolution){

    // Get a handle for our buffers
    vertexPosition_modelspaceID = glGetAttribLocation(programID, "vertexPosition_modelspace");
    vertexUVID = glGetAttribLocation(programID, "vertexUV");
    vertexNormal_modelspaceID = glGetAttribLocation(programID, "vertexNormal_modelspace");

    // Load the texture
    TextureSphere = loadDDS(imagePath);

    // Get a handle for our "myTextureSampler" uniform
    TextureIDSphere  = glGetUniformLocation(programID, "myTextureSampler");

    std::vector<glm::vec3> vertices;
    std::vector<glm::vec2> uvs;
    std::vector<glm::vec3> normals;
    // iniatiate the variable we are going to use
    double X1,Y1,X2,Y2,Z1,Z2;
    double inc1,inc2,inc3,inc4,Radius1,Radius2;
    double PI = 3.14159;


    for(int w = 0; w < resolution; w++) {
         for(int h = (-resolution/2); h < (resolution/2); h++){

             inc1 = (w/(double)resolution)*2*PI;
             inc2 = ((w+1)/(double)resolution)*2*PI;
             inc3 = (h/(double)resolution)*PI;
             inc4 = ((h+1)/(double)resolution)*PI;

             X1 = sin(inc1);
             Y1 = cos(inc1);
             X2 = sin(inc2);
             Y2 = cos(inc2);

             // store the upper and lower radius, remember everything is going to be drawn as triangles
             Radius1 = radius*cos(inc3);
             Radius2 = radius*cos(inc4);

             Z1 = radius*sin(inc3);
             Z2 = radius*sin(inc4);

            // insert the triangle coordinates
            vertices.push_back(glm::vec3(Radius1*X1,Z1,Radius1*Y1));
            vertices.push_back(glm::vec3(Radius1*X2,Z1,Radius1*Y2));
            vertices.push_back(glm::vec3(Radius2*X2,Z2,Radius2*Y2));
            vertices.push_back(glm::vec3(Radius1*X1,Z1,Radius1*Y1));
            vertices.push_back(glm::vec3(Radius2*X2,Z2,Radius2*Y2));
            vertices.push_back(glm::vec3(Radius2*X1,Z2,Radius2*Y1));
            //insert uv coordinates
            uvs.push_back(glm::vec2(1.0f,1.0f));
            uvs.push_back(glm::vec2(1.0f,1.0f));
            uvs.push_back(glm::vec2(1.0f,1.0f));
            uvs.push_back(glm::vec2(1.0f,1.0f));
            uvs.push_back(glm::vec2(1.0f,1.0f));
            uvs.push_back(glm::vec2(1.0f,1.0f));
            // insert the normal data
           normals.push_back(glm::vec3(X1,Z1,Y1)/ glm::length(glm::vec3(X1,Z1,Y1)));
           normals.push_back(glm::vec3(X2,Z1,Y2)/ glm::length(glm::vec3(X2,Z1,Y2)));
           normals.push_back(glm::vec3(X2,Z2,Y2)/ glm::length(glm::vec3(X2,Z2,Y2)));
           normals.push_back(glm::vec3(X1,Z1,Y1)/ glm::length(glm::vec3(X1,Z1,Y1)));
           normals.push_back(glm::vec3(X2,Z2,Y2)/ glm::length(glm::vec3(X2,Z2,Y2)));
           normals.push_back(glm::vec3(X1,Z2,Y1)/ glm::length(glm::vec3(X1,Z2,Y1)));
          }

 }
    std::vector<glm::vec3> indexed_vertices;
    std::vector<glm::vec2> indexed_uvs;
    std::vector<glm::vec3> indexed_normals;
    //indexing mechanism
    indexVBO(vertices, uvs, normals, indicesSphere, indexed_vertices, indexed_uvs, indexed_normals);

    // Load it into a VBO
    loadToVBO(indexed_vertices,indexed_uvs,indexed_normals,indicesSphere,vertexbufferSphere,uvbufferSphere,normalbufferSphere,elementbufferSphere);

    return 0;

}

int Visualizer::createEllipse(const char* imagePath, double radius1, double radius2, int resolution){

    // Get a handle for our buffers
    vertexPosition_modelspaceID = glGetAttribLocation(programID, "vertexPosition_modelspace");
    vertexUVID = glGetAttribLocation(programID, "vertexUV");
    vertexNormal_modelspaceID = glGetAttribLocation(programID, "vertexNormal_modelspace");

    // Load the texture
    TextureSphere = loadDDS(imagePath);

    // Get a handle for our "myTextureSampler" uniform
    TextureIDSphere  = glGetUniformLocation(programID, "myTextureSampler");

    std::vector<glm::vec3> vertices;
    std::vector<glm::vec2> uvs;
    std::vector<glm::vec3> normals;
    // iniatiate the variable we are going to use
    double X1,Y1,X2,Y2,Z;
    double inc1,inc2;
    double PI = 3.14159;


    for(int w = 0; w < resolution; w++) {
             inc1 = (w/(double)resolution)*2*PI;
             inc2 = ((w+1)/(double)resolution)*2*PI;

             X1 = sin(inc1);
             Y1 = cos(inc1);
             X2 = sin(inc2);
             Y2 = cos(inc2);
             Z = 0;
            // insert the triangle coordinates
            vertices.push_back(glm::vec3(radius1*X1,radius2*Y1,Z));
            vertices.push_back(glm::vec3(radius1*X2,radius2*Y2,Z));
            vertices.push_back(glm::vec3(0,0,Z));
            //insert uv coordinates
            uvs.push_back(glm::vec2(0.5f,0.5f));
            uvs.push_back(glm::vec2(0.5f,0.5f));
            uvs.push_back(glm::vec2(0.5f,0.5f));

            // insert the normal data
           normals.push_back(glm::vec3(0,0,1)/ glm::length(glm::vec3(0,0,1)));
           normals.push_back(glm::vec3(0,0,1)/ glm::length(glm::vec3(0,0,1)));
           normals.push_back(glm::vec3(0,0,1)/ glm::length(glm::vec3(0,0,1)));
 }
    std::vector<glm::vec3> indexed_vertices;
    std::vector<glm::vec2> indexed_uvs;
    std::vector<glm::vec3> indexed_normals;
    //indexing mechanism
    indexVBO(vertices, uvs, normals, indicesSphere, indexed_vertices, indexed_uvs, indexed_normals);
    // Load it into a VBO
    loadToVBO(indexed_vertices,indexed_uvs,indexed_normals,indicesSphere,vertexbufferSphere,uvbufferSphere,normalbufferSphere,elementbufferSphere);

    return 0;

}

template<typename T>
int Visualizer::loadToVBO(const std::vector<glm::vec3> &vertices, const std::vector<glm::vec2> &uvs, const std::vector<glm::vec3> &normals, std::vector<T> &indices, GLuint &v, GLuint &u, GLuint &n, GLuint &e){

    glGenBuffers(1, &v);
    glBindBuffer(GL_ARRAY_BUFFER, v);;
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(glm::vec3), &vertices[0], GL_STATIC_DRAW);

    glGenBuffers(1, &u);
    glBindBuffer(GL_ARRAY_BUFFER, u);
    glBufferData(GL_ARRAY_BUFFER, uvs.size() * sizeof(glm::vec2), &uvs[0], GL_STATIC_DRAW);

    glGenBuffers(1, &n);
    glBindBuffer(GL_ARRAY_BUFFER, n);
    glBufferData(GL_ARRAY_BUFFER, normals.size() * sizeof(glm::vec3), &normals[0], GL_STATIC_DRAW);

    // Generate a buffer for the indices as well
    glGenBuffers(1, &e);
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, e);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(T), &indices[0] , GL_STATIC_DRAW);

    // Get a handle for our "LightPosition" uniform
    glUseProgram(programID);
    LightID = glGetUniformLocation(programID, "LightPosition_worldspace");
    return 0;
}

int Visualizer::bindBuffers(GLuint &Texture, GLuint &TextureID, GLuint &vertexbuffer, GLuint &uvbuffer, GLuint &normalbuffer, GLuint &elementbuffer){
// Bind our texture in Texture Unit 0
glActiveTexture(GL_TEXTURE0);
glBindTexture(GL_TEXTURE_2D, Texture);
// Set our "myTextureSampler" sampler to user Texture Unit 0
glUniform1i(TextureID, 0);

// 1rst attribute buffer : vertices
glEnableVertexAttribArray(0);
glBindBuffer(GL_ARRAY_BUFFER, vertexbuffer);
glVertexAttribPointer(
    vertexPosition_modelspaceID,  // The attribute we want to configure
    3,                            // size
    GL_FLOAT,                     // type
    GL_FALSE,                     // normalized?
    0,                            // stride
    (void*)0                      // array buffer offset
);

// 2nd attribute buffer : UVs
glEnableVertexAttribArray(1);
glBindBuffer(GL_ARRAY_BUFFER, uvbuffer);
glVertexAttribPointer(
    vertexUVID,                   // The attribute we want to configure
    2,                            // size : U+V => 2
    GL_FLOAT,                     // type
    GL_FALSE,                     // normalized?
    0,                            // stride
    (void*)0                      // array buffer offset
);

// 3rd attribute buffer : normals
glEnableVertexAttribArray(2);
glBindBuffer(GL_ARRAY_BUFFER, normalbuffer);
glVertexAttribPointer(
    vertexNormal_modelspaceID,    // The attribute we want to configure
    3,                            // size
    GL_FLOAT,                     // type
    GL_FALSE,                     // normalized?
    0,                            // stride
    (void*)0                      // array buffer offset
);

// Index buffer
glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, elementbuffer);
    return 0;
}


int Visualizer::render()
{

    // Measure speed
    double currentTime = glfwGetTime();
    double deltaTime = (double)(currentTime - lastFrameTime);
    lastFrameTime = currentTime;
    nbFrames++;
    if ( currentTime - lastTime >= 1.0 ){ // If last prinf() was more than 1sec ago
        // printf and reset
        std::cout << 1000.0/double(nbFrames) << "ms/frame" << std::endl;
        nbFrames = 0;
        lastTime += 1.0;
    }

    // Clear the screen
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Use our shader
    glUseProgram(programID);

    //glm::mat4 ProjectionMatrix = glm::perspective(35.0f, 4.0f / 3.0f, 0.1f, 100.0f);
    glm::mat4 ProjectionMatrix = glm::ortho(-1.0f,1.0f,-1.0f,1.0f);
    glm::mat4 ViewMatrix = glm::lookAt(
        glm::vec3( 0, 0, 5 ), // Camera is here
        glm::vec3( 0, 0, 0 ), // and looks here
        glm::vec3( 0, 1, 0 )  // Head is up (set to 0,-1,0 to look upside-down)
    );

    bindBuffers(TextureShort,TextureIDShort,vertexbufferShort,uvbufferShort,normalbufferShort,elementbufferShort);

    glm::vec3 lightPos = glm::vec3(4,4,4);

    glUniform3f(LightID, lightPos.x, lightPos.y, lightPos.z);


    { // Euler

        // As an example, rotate arount the vertical axis at 180�/sec
        gOrientation1.y += float((3.14159f/2.0f) * deltaTime);

        // Build the model matrix
        glm::mat4 RotationMatrix = glm::eulerAngleYXZ(gOrientation1.x, gOrientation1.y, gOrientation1.z);
        glm::mat4 TranslationMatrix = glm::translate(glm::mat4(), gPosition1); // A bit to the left
        glm::mat4 ScalingMatrix = glm::scale(glm::mat4(), glm::vec3(1.0f, 1.0f, 1.0f));
        glm::mat4 ModelMatrix = TranslationMatrix * RotationMatrix * ScalingMatrix;

        glm::mat4 MVP = ProjectionMatrix * ViewMatrix * ModelMatrix;

        // Send our transformation to the currently bound shader,
        // in the "MVP" uniform
        glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVP[0][0]);
        glUniformMatrix4fv(ModelMatrixID, 1, GL_FALSE, &ModelMatrix[0][0]);
        glUniformMatrix4fv(ViewMatrixID, 1, GL_FALSE, &ViewMatrix[0][0]);

        // Draw the triangles !
        glDrawElements(
            GL_TRIANGLES,      // mode
            (unsigned int)indicesShort.size(),    // count
            GL_UNSIGNED_INT,   // type
            (void*)0           // element array buffer offset
        );
    }

    { // Quaternion

        // If the box is checked...
        if (gLookAtOther){
            glm::vec3 desiredDir = gPosition1-gPosition2;
            glm::vec3 desiredUp = glm::vec3(0.0f, 1.0f, 0.0f); // +Y

            // Compute the desired orientation
            glm::quat targetOrientation = glm::normalize(LookAt(desiredDir, desiredUp));

            // And interpolate
            gOrientation2 = RotateTowards(gOrientation2, targetOrientation, 1.0f*deltaTime);
        }

        glm::mat4 RotationMatrix = glm::mat4_cast(gOrientation2);
        glm::mat4 TranslationMatrix = glm::translate(glm::mat4(), gPosition2); // A bit to the right
        glm::mat4 ScalingMatrix = glm::scale(glm::mat4(), glm::vec3(1.0f, 1.0f, 1.0f));
        glm::mat4 ModelMatrix = TranslationMatrix * RotationMatrix * ScalingMatrix;

        glm::mat4 MVP = ProjectionMatrix * ViewMatrix * ModelMatrix;

        // Send our transformation to the currently bound shader,
        // in the "MVP" uniform
        glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVP[0][0]);
        glUniformMatrix4fv(ModelMatrixID, 1, GL_FALSE, &ModelMatrix[0][0]);
        glUniformMatrix4fv(ViewMatrixID, 1, GL_FALSE, &ViewMatrix[0][0]);

        // Draw the triangles !
        glDrawElements(
            GL_TRIANGLES,      // mode
            (unsigned int)indicesShort.size(),    // count
            GL_UNSIGNED_INT,   // type
            (void*)0           // element array buffer offset
        );
    }

    glDisableVertexAttribArray(0);
    glDisableVertexAttribArray(1);
    glDisableVertexAttribArray(2);

    // Draw GUI
    TwDraw();

    // Swap buffers
    glfwSwapBuffers(window);
    glfwPollEvents();

    return 0;
}



int Visualizer::renderPoints(const hop3d::PointCloud &inputPointCloud){


    // Measure speed
    double currentTime = glfwGetTime();
    //double deltaTime = (double)(currentTime - lastFrameTime);
    lastFrameTime = currentTime;
    nbFrames++;
    if ( currentTime - lastTime >= 1.0 ){ // If last prinf() was more than 1sec ago
        // printf and reset
        std::cout << 1000.0/double(nbFrames) << "ms/frame" << std::endl;
        nbFrames = 0;
        lastTime += 1.0;
    }

    // Clear the screen
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    // Use our shader
    glUseProgram(programID);
    glm::mat4 ProjectionMatrix = glm::perspective(FOV, 4.0f / 3.0f, 0.1f, 100.0f);
    glm::mat4 ViewMatrix = glm::lookAt(
        gCameraPosition, // Camera is here
        gCameraPosition-glm::vec3(0.0f,0.0f,1.0f), // and looks here
        glm::vec3( 0, 1, 0 )  // Head is up (set to 0,-1,0 to look upside-down)
    );

    bindBuffers(TexturePoints,TextureIDPoints,vertexbufferPoints,uvbufferPoints,normalbufferPoints,elementbufferPoints);

    glm::vec3 lightPos = glm::vec3(4,4,4);

    glUniform3f(LightID, lightPos.x, lightPos.y, lightPos.z);

    { // PointCloud

        // Build the model matrix
        glm::mat4 RotationMatrix = glm::eulerAngleYXZ(gOrientation1.x, gOrientation1.y, gOrientation1.z);
        glm::mat4 TranslationMatrix = glm::translate(glm::mat4(), gPosition1); // A bit to the left
        glm::mat4 ScalingMatrix = glm::scale(glm::mat4(), glm::vec3(1.0f, 1.0f, 1.0f));
        glm::mat4 ModelMatrix = TranslationMatrix * RotationMatrix * ScalingMatrix;
        glm::mat4 MVP = ProjectionMatrix * ViewMatrix * ModelMatrix;

        // Send our transformation to the currently bound shader,
        // in the "MVP" uniform
        glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVP[0][0]);
        glUniformMatrix4fv(ModelMatrixID, 1, GL_FALSE, &ModelMatrix[0][0]);
        glUniformMatrix4fv(ViewMatrixID, 1, GL_FALSE, &ViewMatrix[0][0]);

        // Draw the triangles !
        glDrawElements(
            GL_POINTS,      // mode
            (unsigned int)indicesPoints.size(),    // count
            GL_UNSIGNED_INT,   // type
            (void*)0           // element array buffer offset
        );
    }

    bindBuffers(TextureSphere,TextureIDSphere,vertexbufferSphere,uvbufferSphere,normalbufferSphere,elementbufferSphere);

    if (gLookAtOther){
        for(unsigned int  i=0; i < inputPointCloud.pointCloudNormal.size(); i+=150 ){
            glm::vec3 gPosition3(inputPointCloud.pointCloudNormal[i].position(0), inputPointCloud.pointCloudNormal[i].position(1), inputPointCloud.pointCloudNormal[i].position(2));
            glm::vec3 gNormal3(inputPointCloud.pointCloudNormal[i].normal(0), inputPointCloud.pointCloudNormal[i].normal(1), inputPointCloud.pointCloudNormal[i].normal(2));
            // Receptive Fields

            glm::mat4 RotToNormal( 1.0f ); // construct identity matrix
            glm::vec3 ObjNormal = glm::vec3(0.0f,0.0f,1.0f);
            if( ObjNormal != gNormal3 && ObjNormal != -gNormal3){
            glm::vec3 xaxis = glm::cross(ObjNormal, -gNormal3);
            xaxis = glm::normalize(xaxis);

            glm::vec3 yaxis = glm::cross(-gNormal3,xaxis);
            yaxis = glm::normalize(yaxis);

            RotToNormal[0].x = xaxis.x;
            RotToNormal[1].x = yaxis.x;
            RotToNormal[2].x = gNormal3.x;
            RotToNormal[0].y = xaxis.y;
            RotToNormal[1].y = yaxis.y;
            RotToNormal[2].y = gNormal3.y;
            RotToNormal[0].z = xaxis.z;
            RotToNormal[1].z = yaxis.z;
            RotToNormal[2].z = gNormal3.z;

            }
            else if (ObjNormal == -gNormal3) {

                RotToNormal[0].x = 1.0f;
                RotToNormal[1].y = 1.0f;
                RotToNormal[2].z = 1.0f;

                //Printing out matrices and vectors from glm, bug in glm/ext.hpp
//                std::cout << gNormal3.x << "; "<< gNormal3.y << "; "<< gNormal3.z << std::endl;
//                std::cout << RotToNormal[0].x << "; "<< RotToNormal[1].x << "; "<< RotToNormal[2].x << "; "<< RotToNormal[3].x << std::endl;
//                std::cout << RotToNormal[0].y << "; "<< RotToNormal[1].y << "; "<< RotToNormal[2].y << "; "<< RotToNormal[3].y << std::endl;
//                std::cout << RotToNormal[0].z << "; "<< RotToNormal[1].z << "; "<< RotToNormal[2].z << "; "<< RotToNormal[3].z << std::endl;
//                std::cout << RotToNormal[0].w << "; "<< RotToNormal[1].w << "; "<< RotToNormal[2].w << "; "<< RotToNormal[3].w << std::endl;
//                std::cout << "<<<<<<<<<<<<<<<" << std::endl;

            }
            else if (ObjNormal == gNormal3) {
                RotToNormal[0].x = -1.0f;
                RotToNormal[1].y = 1.0f;
                RotToNormal[2].z = 1.0f;
            }


            //rotate receptive fields with pointcloud
            glm::mat4 RotationMatrix = glm::eulerAngleYXZ(gOrientation1.x, gOrientation1.y, gOrientation1.z)*glm::translate(glm::mat4(), gPosition3)*RotToNormal;
            //and then translate
            glm::mat4 TranslationMatrix = glm::translate(glm::mat4(), gPosition1); // A bit to the left
            glm::mat4 ScalingMatrix = glm::scale(glm::mat4(), glm::vec3(1.0f, 1.0f, 1.0f));
            glm::mat4 ModelMatrix = TranslationMatrix * RotationMatrix * ScalingMatrix;
            glm::mat4 MVP = ProjectionMatrix * ViewMatrix * ModelMatrix;
             // Send our transformation to the currently bound shader,
            // in the "MVP" uniform
            glUniformMatrix4fv(MatrixID, 1, GL_FALSE, &MVP[0][0]);
            glUniformMatrix4fv(ModelMatrixID, 1, GL_FALSE, &ModelMatrix[0][0]);
            glUniformMatrix4fv(ViewMatrixID, 1, GL_FALSE, &ViewMatrix[0][0]);
            //std::cout<<indicesSphere.size() << std::endl;
            // Draw the triangles !
            glDrawElements(
                GL_TRIANGLES,      // mode
                (unsigned short int)indicesSphere.size(),    // count
                GL_UNSIGNED_SHORT,   // type
                (void*)0           // element array buffer offset
            );
        }
    }

    glDisableVertexAttribArray(0);
    glDisableVertexAttribArray(1);
    glDisableVertexAttribArray(2);

    // Draw GUI
    TwDraw();

    // Swap buffers
    glfwSwapBuffers(window);
    glfwPollEvents();

    return 0;
}


int Visualizer::close(){
    // Cleanup VBO and shader
    glDeleteBuffers(1, &vertexbufferPoints);
    glDeleteBuffers(1, &uvbufferPoints);
    glDeleteBuffers(1, &normalbufferPoints);
    glDeleteBuffers(1, &elementbufferPoints);
    glDeleteProgram(programID);
    glDeleteTextures(1, &TextureIDSphere);
    glDeleteBuffers(1, &vertexbufferSphere);
    glDeleteBuffers(1, &uvbufferSphere);
    glDeleteBuffers(1, &normalbufferSphere);
    glDeleteBuffers(1, &elementbufferSphere);
    glDeleteTextures(1, &TextureIDSphere);

    // Close GUI and OpenGL window, and terminate GLFW
    TwTerminate();
    glfwTerminate();

    return 0;
}


int Visualizer::checkClose(){
    if (glfwGetKey(window, GLFW_KEY_ESCAPE ) != GLFW_PRESS && glfwWindowShouldClose(window) == 0) return 1;
    else return 0;
}



