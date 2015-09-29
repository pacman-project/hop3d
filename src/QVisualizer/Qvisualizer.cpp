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
    hierarchy.reset(new Hierarchy());
    cloudsListLayers.resize(6);
    clustersList.resize(6);
    linksLists.resize(6);
}

/// Construction
QGLVisualizer::QGLVisualizer(Config _config): config(_config), updateHierarchyFlag(false){
    hierarchy.reset(new Hierarchy("configGlobal.xml"));
    cloudsListLayers.resize(6);
    clustersList.resize(6);
    linksLists.resize(6);
}

/// Construction
QGLVisualizer::QGLVisualizer(std::string configFile) :
        config(configFile), updateHierarchyFlag(false) {
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
    double ellipsoidScale=1.0;
    glPushMatrix();
        glMultMatrixd(GLmat);
        drawEllipsoid(10,10,sqrt(es.eigenvalues()(0))*ellipsoidScale, sqrt(es.eigenvalues()(1))*ellipsoidScale, sqrt(es.eigenvalues()(2))*ellipsoidScale);
    glPopMatrix();
}

/// Observer update
void QGLVisualizer::update(hop3d::Hierarchy& _hierarchy) {
    mtxHierarchy.lock();
    *hierarchy = _hierarchy;
    if (config.verbose==1){
        std::cout << "First layer size: " << hierarchy->firstLayer.size() << "\n";
        std::cout << "View depent layers no: " << hierarchy->viewDependentLayers.size() << "\n";
        for (size_t i=0; i<hierarchy->viewDependentLayers.size(); i++){
            std::cout << "Layer " << i+2 << " size: " << hierarchy->viewDependentLayers[i].size() << "\n";
            std::cout << "Ids in the group: ";
            for (size_t j=0; j<hierarchy->viewDependentLayers[i].size(); j++){
                std::cout << hierarchy->viewDependentLayers[i][j].id << ", ";
            }
            std::cout << "\n";
        }
    }
    mtxHierarchy.unlock();
    updateHierarchyFlag = true;
}

///update hierarchy, prepare gl lists
void QGLVisualizer::updateHierarchy(){
    if (updateHierarchyFlag){
        updateHierarchyFlag = false;
        mtxHierarchy.lock();
        for (size_t i=0; i<hierarchy->firstLayer.size(); i++){
            hop3d::PointCloud cloud;
            int cols = hierarchy->firstLayer[i].patch.cols;
            for (int u=0;u<hierarchy->firstLayer[i].patch.cols;u++){
                for (int v=0;v<hierarchy->firstLayer[i].patch.rows;v++){
                    if (hierarchy->firstLayer[i].mask.at<double>(u,v)!=0){
                        hop3d::PointNormal point(Vec3((u-(cols/2))*config.pixelSize,(v-(cols/2))*config.pixelSize,hierarchy->firstLayer[i].patch.at<double>(u,v)), hierarchy->firstLayer[i].normal);
                        cloud.pointCloudNormal.push_back(point);
                    }
                }
            }
            cloudsListLayers[0].push_back(createCloudList(cloud));
        }
        for (size_t i=0;i<hierarchy->viewDependentLayers.size();i++){
            if (config.verbose==1){
                std::cout << "layer "<< i+1 << " size: " << cloudsListLayers[0].size() << "\n";
            }
            for (auto it = hierarchy->viewDependentLayers[i].begin(); it!=hierarchy->viewDependentLayers[i].end(); it++){
                cloudsListLayers[i+1].push_back(createPartList(*it, int(i+1)));
                clustersList[i+1].push_back(createClustersList(*it, int(i+1)));
            }
            linksLists[i+1].push_back(createLinksList(int(i+1)));
        }
        mtxHierarchy.unlock();
    }
}

/// Draw point clouds
void QGLVisualizer::drawPointClouds(void){
    //mtxPointClouds.lock();
    for (int layerNo=0;layerNo<3;layerNo++){
        for (size_t i = 0;i<cloudsListLayers[layerNo].size();i++){
            double GLmat[16]={1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, (double)(config.partDist[layerNo]*double(i)-((double)cloudsListLayers[layerNo].size()/2)*config.partDist[layerNo]), 0, config.posZ[layerNo], 1};
            glPushMatrix();
                glColor3ub(200,200,200);
                glMultMatrixd(GLmat);
                glPointSize((float)config.cloudPointSize);
                glCallList(cloudsListLayers[layerNo][i]);
            glPopMatrix();
        }
    }
    //mtxPointClouds.unlock();
}

/// Draw clusters
void QGLVisualizer::drawClusters(void){
    //mtxPointClouds.lock();
    for (size_t layerNo=0;layerNo<hierarchy->viewDependentLayers.size();layerNo++){
        for (size_t i = 0;i<clustersList[layerNo+1].size();i++){
            double GLmat[16]={1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, (double)(config.partDist[layerNo+1]*double(i)-((double)cloudsListLayers[layerNo+1].size()/2)*config.partDist[layerNo+1]), 0, config.posZ[layerNo+1], 1};
            glPushMatrix();
                glMultMatrixd(GLmat);
                glCallList(clustersList[layerNo+1][i]);
            glPopMatrix();
        }
    }
    //mtxPointClouds.unlock();
}

/// Create point cloud List
GLuint QGLVisualizer::createPartList(ViewDependentPart& part, int layerNo){
    // create one display list
    GLuint index = glGenLists(1);
    // compile the display list, store a triangle in it
    glNewList(index, GL_COMPILE);
    glPushMatrix();
    for (size_t n = 0; n < part.partIds.size(); n++){
        for (size_t m = 0; m < part.partIds[n].size(); m++){
            Vec3 pos(config.pixelSize*part.gaussians[n][m].mean(0), config.pixelSize*part.gaussians[n][m].mean(1), part.gaussians[n][m].mean(2));
            if ((n==1)&&(m==1)){
                pos(0)=0; pos(1)=0; pos(2)=0;
            }
            /*if ((part.partIds[n][m]==-1)){
                pos(0)=config.pixelSize*5.0*double(n-1); pos(1)=config.pixelSize*5.0*double(m-1); pos(2)=0;
            }*/
            double GLmat[16]={1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, pos(0), pos(1), pos(2), 1};
            glPushMatrix();
                glMultMatrixd(GLmat);
                int id = part.partIds[n][m];
                if ((part.partIds[n][m]==-1)){
                    glColor3ub(100,50,50);
                    id = 0;
                }
                else
                    glColor3ub(200,200,200);
                glCallList(cloudsListLayers[layerNo-1][id]);
            glPopMatrix();
        }
    }
    glPopMatrix();
    glEndList();
    return index;
}

/// Create clusters List
GLuint QGLVisualizer::createClustersList(ViewDependentPart& part, int layerNo){
    // create one display list
    GLuint index = glGenLists(1);
    glNewList(index, GL_COMPILE);
    glPushMatrix();
    int componentNo=0;
    for (auto itComp = part.group.begin(); itComp!=part.group.end();itComp++){
        for (size_t n = 0; n < itComp->partIds.size(); n++){
            for (size_t m = 0; m < itComp->partIds[n].size(); m++){
                Vec3 pos(config.pixelSize*itComp->gaussians[n][m].mean(0), config.pixelSize*itComp->gaussians[n][m].mean(1), itComp->gaussians[n][m].mean(2));
                if ((n==1)&&(m==1)){
                    pos(0)=0; pos(1)=0; pos(2)=0;
                }
                /*if ((itComp->partIds[n][m]==-1)){
                    pos(0)=config.pixelSize*5.0*double(n-1); pos(1)=config.pixelSize*5.0*double(m-1); pos(2)=0;
                }*/
                double GLmat[16]={1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, pos(0), pos(1)-(double)(config.partDist[layerNo]*double(componentNo+1)), pos(2), 1};
                glPushMatrix();
                    glMultMatrixd(GLmat);
                    glColor4d(config.clustersColor.red(), config.clustersColor.green(), config.clustersColor.blue(), config.clustersColor.alpha());
                    int id = itComp->partIds[n][m];
                    if ((layerNo==1)&&(itComp->partIds[n][m]==-1))
                        id = 0;
                    glCallList(cloudsListLayers[layerNo-1][id]);
                glPopMatrix();
            }
        }
        componentNo++;
    }
    glPopMatrix();
    glEndList();
    return index;
}

/// Create point cloud List
GLuint QGLVisualizer::createCloudList(hop3d::PointCloud& pointCloud){
    // create one display list
    GLuint index = glGenLists(1);
    // compile the display list, store a triangle in it
    glNewList(index, GL_COMPILE);
        glBegin(GL_POINTS);
        for (size_t n = 0; n < pointCloud.pointCloudNormal.size(); n++){
            //glColor3ub(pointCloud.second.pointCloudNormal[n].r, pointCloud.second.pointCloudNormal[n].g, pointCloud.second.pointCloudNormal[n].b);
            glVertex3d(pointCloud.pointCloudNormal[n].position(0),
                      pointCloud.pointCloudNormal[n].position(1),
                      pointCloud.pointCloudNormal[n].position(2));
        }
        glEnd();
    glEndList();
    return index;
}

/// Create layer 2 layer List
GLuint QGLVisualizer::createLinksList(int destLayerNo){
    // create one display list
    GLuint index = glGenLists(1);
    // compile the display list, store a triangle in it
    glNewList(index, GL_COMPILE);
        glLineWidth((float)config.layer2LayerWidth);
        glColor4d(config.layer2LayerColor.red(), config.layer2LayerColor.green(), config.layer2LayerColor.blue(), config.layer2LayerColor.alpha());
        glBegin(GL_LINES);
        int partNo=0;
        for (auto it = hierarchy->viewDependentLayers[destLayerNo-1].begin(); it!=hierarchy->viewDependentLayers[destLayerNo-1].end(); it++){
            std::vector<int> filterIds;
            for (size_t n = 0; n < it->partIds.size(); n++){
                for (size_t m = 0; m < it->partIds[n].size(); m++){
                    filterIds.push_back(it->partIds[n][m]);
                }
            }
            auto itFilter = std::unique (filterIds.begin(), filterIds.end());
            filterIds.resize( std::distance(filterIds.begin(),itFilter) );
            for (size_t i=0;i<filterIds.size();i++){
                // position of the i-th filter
                Vec3 filterPos((double)(config.partDist[destLayerNo-1]*double(filterIds[i])-((double)cloudsListLayers[destLayerNo-1].size()/2)*config.partDist[destLayerNo-1]), 0, config.posZ[destLayerNo-1]);
                glVertex3d(filterPos(0), filterPos(1), filterPos(2));
                // position of the i-th part
                Vec3 partPos((double)(config.partDist[destLayerNo]*double(partNo)-((double)cloudsListLayers[destLayerNo].size()/2)*config.partDist[destLayerNo]), 0, config.posZ[destLayerNo]);
                glVertex3d(partPos(0), partPos(1), partPos(2));
            }
            partNo++;
        }
        glEnd();
    glEndList();
    return index;
}

/// Draw layer 2 layer links
void QGLVisualizer::drawLayer2Layer(void){
    for (int layerNo=1;layerNo<3;layerNo++){
        for (size_t i = 0;i<linksLists[layerNo].size();i++){
            glCallList(linksLists[layerNo][i]);
        }
    }
}

/// draw objects
void QGLVisualizer::draw(){
    // Here we are in the world coordinate system. Draw unit size axis.
    drawAxis();
    //drawEllipsoid(Vec3(0,0,0),Mat33::Identity());
    if (config.drawLayer2Layer)
        drawLayer2Layer();
    if (config.drawClusters)
        drawClusters();
    drawPointClouds();
    updateHierarchy();
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

