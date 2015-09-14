#include "../include/QVisualizer/Qvisualizer.h"
#include <memory>
#include <cmath>
#include <stdexcept>
#include <chrono>
#include <GL/glut.h>

using namespace hop3d;

/// A single instance of Visualizer
QGLVisualizer::Ptr visualizer;

class SolidSphere
{
protected:
    std::vector<GLdouble> vertices;
    std::vector<GLdouble> normals;
    std::vector<GLdouble> texcoords;
    std::vector<GLushort> indices;

public:
    SolidSphere(float radius, unsigned int rings, unsigned int sectors) {
        double const R = 1./(double)(rings-1);
        double const S = 1./(double)(sectors-1);
        unsigned int r, s;

        vertices.resize(rings * sectors * 3);
        normals.resize(rings * sectors * 3);
        texcoords.resize(rings * sectors * 2);
        std::vector<GLdouble>::iterator v = vertices.begin();
        std::vector<GLdouble>::iterator n = normals.begin();
        std::vector<GLdouble>::iterator t = texcoords.begin();
        for(r = 0; r < rings; r++) for(s = 0; s < sectors; s++) {
                double const y = sin( -M_PI_2 + M_PI * r * R );
                double const x = cos(2*M_PI * s * S) * sin( M_PI * r * R );
                double const z = sin(2*M_PI * s * S) * sin( M_PI * r * R );

                *t++ = s*S;
                *t++ = r*R;

                *v++ = x * radius;
                *v++ = y * radius;
                *v++ = z * radius;

                *n++ = x;
                *n++ = y;
                *n++ = z;
        }

        indices.resize(rings * sectors * 4);
        std::vector<GLushort>::iterator i = indices.begin();
        for(r = 0; r < rings-1; r++) for(s = 0; s < sectors-1; s++) {
                *i++ = (short unsigned int)(r * sectors + s);
                *i++ = (short unsigned int)(r * sectors + (s+1));
                *i++ = (short unsigned int)((r+1) * sectors + (s+1));
                *i++ = (short unsigned int)((r+1) * sectors + s);
        }
    }

    void draw(GLfloat x, GLfloat y, GLfloat z)
    {
        glMatrixMode(GL_MODELVIEW);
        glPushMatrix();
        glTranslatef(x,y,z);
        glEnable(GL_NORMALIZE);

        glEnableClientState(GL_VERTEX_ARRAY);
        glEnableClientState(GL_NORMAL_ARRAY);

        glVertexPointer(3, GL_FLOAT, 0, &vertices[0]);
        glNormalPointer(GL_FLOAT, 0, &normals[0]);
        glDrawElements(GL_QUADS, (int)indices.size(), GL_UNSIGNED_SHORT, &indices[0]);
        glPopMatrix();
    }
};

QGLVisualizer::QGLVisualizer(void) {
}

/// Construction
QGLVisualizer::QGLVisualizer(Config _config): config(_config){

}

/// Construction
QGLVisualizer::QGLVisualizer(std::string configFile) :
        config(configFile) {
    tinyxml2::XMLDocument configXML;
    std::string filename = "../../resources/" + configFile;
    configXML.LoadFile(filename.c_str());
    if (configXML.ErrorID())
        std::cout << "unable to load visualizer config file.\n";
}

/// Destruction
QGLVisualizer::~QGLVisualizer(void) {
}

/// Draw ellipsoid
void QGLVisualizer::drawEllipsoid(unsigned int uiStacks, unsigned int uiSlices, double fA, double fB, double fC) const {
    double tStep = (M_PI) / uiSlices;
    double sStep = (M_PI) / uiStacks;
    //glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
    glPolygonMode(GL_FRONT_AND_BACK,GL_COLOR);
    for(double t = -M_PI/2.0; t <= (M_PI/2.0)+.0001; t += tStep) {
        glBegin(GL_TRIANGLE_STRIP);
        for(double s = -M_PI; s <= M_PI+.0001; s += sStep) {
            glVertex3d(fA * cos(t) * cos(s), fB * cos(t) * sin(s), fC * sin(t));
            double norm = sqrt(pow(fA * cos(t) * cos(s),2.0)+pow(fB * cos(t) * sin(s),2.0) + pow(fC * sin(t),2.0));
            glNormal3d((fA * cos(t) * cos(s))/norm, (fB * cos(t) * sin(s))/norm, (fC * sin(t))/norm);
            glVertex3d(fA * cos(t+tStep) * cos(s), fB * cos(t+tStep) * sin(s), fC * sin(t+tStep));
            norm = sqrt(pow(fA * cos(t+tStep) * cos(s),2.0)+pow(fB * cos(t+tStep) * sin(s),2.0) + pow(fC * sin(t+tStep),2.0));
            glNormal3d((fA * cos(t+tStep) * cos(s))/norm, (fB * cos(t+tStep) * sin(s))/norm, (fC * sin(t+tStep))/norm);
        }
        glEnd();
    }
}

/// Draw ellipsoid
void QGLVisualizer::drawEllipsoid(const Vec3& pos, const Mat33& covariance) const{
    Eigen::SelfAdjointEigenSolver<Mat33> es;
    es.compute(covariance);
    Mat33 V(es.eigenvectors());
    double GLmat[16]={V(0,0), V(1,0), V(2,0), 0, V(0,1), V(1,1), V(2,1), 0, V(0,2), V(1,2), V(2,2), 0, pos.x(), pos.y(), pos.z(), 1};
    glPushMatrix();
        glMultMatrixd(GLmat);
        //drawEllipsoid(10,10,sqrt(es.eigenvalues()(0))*config.ellipsoidScale, sqrt(es.eigenvalues()(1))*config.ellipsoidScale, sqrt(es.eigenvalues()(2))*config.ellipsoidScale);
    glPopMatrix();
}

/// Observer update
void QGLVisualizer::update(hop3d::Hierarchy& hierarchy) {
    std::cout << hierarchy.firstLayer.size();
}

/// Draw point clouds
void QGLVisualizer::drawPointClouds(void){
/*    mtxPointClouds.lock();
    for (int i = 0;i<pointClouds.size();i++){
        mtxCamTrajectory.lock();
        if (camTrajectory.size()>imagesIds[i]){
            Mat34 camPose = camTrajectory[imagesIds[i]].pose;
            mtxCamTrajectory.unlock();
            float_type GLmat[16]={camPose(0,0), camPose(1,0), camPose(2,0), camPose(3,0), camPose(0,1), camPose(1,1), camPose(2,1), camPose(3,1), camPose(0,2), camPose(1,2), camPose(2,2), camPose(3,2), camPose(0,3), camPose(1,3), camPose(2,3), camPose(3,3)};
            glPushMatrix();
                glMultMatrixd(GLmat);
                glPointSize(config.cloudPointSize);
                glCallList(cloudsList[i]);
            glPopMatrix();
        }
    }
    mtxPointClouds.unlock();
    */
}

/// Create point cloud List
GLuint QGLVisualizer::createCloudList(const std::pair<int,PointCloud>& pointCloud){
    // create one display list
    GLuint index = glGenLists(1);

    // compile the display list, store a triangle in it
    glNewList(index, GL_COMPILE);
        glBegin(GL_POINTS);
        for (size_t n = 0; n < pointCloud.second.pointCloudNormal.size(); n++){
            glColor3ub(200,200,200);
            //glColor3ub(pointCloud.second.pointCloudNormal[n].r, pointCloud.second.pointCloudNormal[n].g, pointCloud.second.pointCloudNormal[n].b);
            glVertex3d(pointCloud.second.pointCloudNormal[n].position(0),
                       pointCloud.second.pointCloudNormal[n].position(1),
                       pointCloud.second.pointCloudNormal[n].position(2));
        }
        glEnd();
    glEndList();
    return index;
}

/// draw objects
void QGLVisualizer::draw(){
    // Here we are in the world coordinate system. Draw unit size axis.
    drawAxis();
}

/// draw objects
void QGLVisualizer::animate(){
}

/// initialize visualizer
void QGLVisualizer::init(){
    // Light setup
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 50.0);
    GLfloat specular_color[4] = { 0.99f, 0.99f, 0.99f, 1.0 };
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,  specular_color);

    //Set global ambient light
    GLfloat black[] = {(float)0.99, (float)0.99, (float)0.99, (float)1};
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, black);

    glEnable(GL_AUTO_NORMAL);
    glEnable(GL_NORMALIZE);
    // Restore previous viewer state.
    //restoreStateFromFile();

    camera()->setZNearCoefficient((float)0.00001);
    camera()->setZClippingCoefficient(100.0);

    setBackgroundColor(config.backgroundColor);

    glEnable(GL_LINE_SMOOTH);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Opens help window
    help();

    startAnimation();
}

/// generate help string
std::string QGLVisualizer::help() const{
    std::string text("S i m p l e V i e w e r");
    text += "Use the mouse to move the camera around the object. ";
    text += "You can respectively revolve around, zoom and translate with the three mouse buttons. ";
    text += "Left and middle buttons pressed together rotate around the camera view direction axis<br><br>";
    text += "Pressing <b>Alt</b> and one of the function keys (<b>F1</b>..<b>F12</b>) defines a camera keyFrame. ";
    text += "Simply press the function key again to restore it. Several keyFrames define a ";
    text += "camera path. Paths are saved when you quit the application and restored at next start.<br><br>";
    text += "Press <b>F</b> to display the frame rate, <b>A</b> for the world axis, ";
    text += "<b>Alt+Return</b> for full screen mode and <b>Control+S</b> to save a snapshot. ";
    text += "See the <b>Keyboard</b> tab in this window for a complete shortcut list.<br><br>";
    text += "Double clicks automates single click actions: A left button double click aligns the closer axis with the camera (if close enough). ";
    text += "A middle button double click fits the zoom of the camera and the right button re-centers the scene.<br><br>";
    text += "A left button double click while holding right button pressed defines the camera <i>Revolve Around Point</i>. ";
    text += "See the <b>Mouse</b> tab and the documentation web pages for details.<br><br>";
    text += "Press <b>Escape</b> to exit the viewer.";
    return text;
}
