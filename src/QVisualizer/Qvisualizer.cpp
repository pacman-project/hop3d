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
    layersOfObjects.resize(6);
    objects3Dlist.resize(6);
}

/// Construction
QGLVisualizer::QGLVisualizer(Config _config): config(_config), updateHierarchyFlag(false){
    hierarchy.reset(new Hierarchy("configGlobal.xml"));
    cloudsListLayers.resize(6);
    clustersList.resize(6);
    linksLists.resize(6);
    layersOfObjects.resize(6);
    objects3Dlist.resize(6);
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

/// Update 3D object model
void QGLVisualizer::update(const std::vector<ViewIndependentPart>& objectParts, int objLayerId){
    mtxHierarchy.lock();
    /*Object3D object;
    for (auto & part : objectParts){
        ViewIndependentPart p = part;
        int partIdNo=0;
        for (auto & part3rd : part.parts){
            p.parts[partIdNo].id = hierarchy->interpreter[part3rd.id];
            partIdNo++;
        }
        object.push_back(p);
    }*/
    layersOfObjects[objLayerId].push_back(objectParts);
    mtxHierarchy.unlock();
}

/// Update 3D object models
void QGLVisualizer::update3Dmodels(void){
    update3DModelsFlag = true;
}

/// Update 3D object models
void QGLVisualizer::update3Dobjects(void){
    if (update3DModelsFlag){
        update3DModelsFlag = false;
        int layerNo=0;
        for (auto & layer : layersOfObjects){
            int objectNo=0;
            for (auto & object : layer){
                objects3Dlist[layerNo].push_back(createObjList(object,layerNo));
                objectNo++;
            }
            layerNo++;
        }
    }
}

///update hierarchy, prepare gl lists
void QGLVisualizer::updateHierarchy(){
    if (updateHierarchyFlag){
        updateHierarchyFlag = false;
        mtxHierarchy.lock();
        for (size_t i=0; i<hierarchy->firstLayer.size(); i++){//filters
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
            cloudsListLayers[0].push_back(createCloudList(cloud, hierarchy->firstLayer[i].normal));
        }
        for (size_t i=0;i<hierarchy->viewDependentLayers.size();i++){//view-dependent layers
            backgroundList.push_back(createBackgroundList(int(i+1)));
            if (config.verbose==1){
                std::cout << "layer "<< i+2 << " size: " << hierarchy->viewDependentLayers[i].size() << "\n";
            }
            for (auto it = hierarchy->viewDependentLayers[i].begin(); it!=hierarchy->viewDependentLayers[i].end(); it++){
                cloudsListLayers[i+1].push_back(createPartList(*it, int(i+1)));
                clustersList[i+1].push_back(createClustersList(*it, int(i+1)));
            }
            linksLists[i+1].push_back(createLinksList(int(i+1)));
        }
        for (size_t i=0;i<hierarchy->viewIndependentLayers.size();i++){//view-independent
            if (config.verbose==1){
                std::cout << "layer "<< i+4 << " size: " << hierarchy->viewIndependentLayers[i].size() << "\n";
            }
            for (auto it = hierarchy->viewIndependentLayers[i].begin(); it!=hierarchy->viewIndependentLayers[i].end(); it++){
                cloudsListLayers[i+3].push_back(createVIPartList(*it, int(i+4)));
                clustersList[i+3].push_back(createVIClustersList(*it, int(i+4)));
            }
            linksLists[i+3].push_back(createVILinksList(int(i+4)));
        }
        mtxHierarchy.unlock();
    }
}

/// Draw point clouds
void QGLVisualizer::drawPointClouds(void){
    //mtxPointClouds.lock();
    for (int layerNo=0;layerNo<5;layerNo++){
        for (size_t i = 0;i<cloudsListLayers[layerNo].size();i++){
            double GLmat[16]={1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, (double)(config.partDist[layerNo]*double(i)-((double)cloudsListLayers[layerNo].size()/2)*config.partDist[layerNo]), 0, config.posZ[layerNo], 1};
            glPushMatrix();
                //glColor3ub(200,200,200);
                glMultMatrixd(GLmat);
                glPointSize((float)config.cloudPointSize);
                glCallList(cloudsListLayers[layerNo][i]);
            glPopMatrix();
        }
    }
    //mtxPointClouds.unlock();
}

/// Draw objects
void QGLVisualizer::draw3Dobjects(void){
    //mtxPointClouds.lock();
    for (int layerNo=0;layerNo<2;layerNo++){
        for (size_t i = 0;i<objects3Dlist[layerNo].size();i++){
            double GLmat[16]={1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, (double)(config.objectsDist[layerNo]*double(i)-((double)objects3Dlist[layerNo].size()/2)*config.objectsDist[layerNo]), config.objectsPosY[layerNo], config.objectsPosZ[layerNo], 1};
            glPushMatrix();
                //glColor3ub(200,200,200);
                glMultMatrixd(GLmat);
                glPointSize((float)config.cloudPointSize);
                glCallList(objects3Dlist[layerNo][i]);
            glPopMatrix();
        }
    }
    //mtxPointClouds.unlock();
}

/// Draw clusters
void QGLVisualizer::drawClusters(void){
    //mtxPointClouds.lock();
    for (size_t layerNo=0;layerNo<hierarchy->viewDependentLayers.size()+2;layerNo++){
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

/// Create background List
GLuint QGLVisualizer::createBackgroundList(int layerNo){
    // create single display list
    GLuint index = glGenLists(1);
    glNewList(index, GL_COMPILE);
    glPushMatrix();
        int patchSize = 5*(2*layerNo-1);
        glColor3ub(100,50,50);
        glBegin(GL_POINTS);
        for (int n = 0; n < patchSize; n++){
            for (int m = 0; m < patchSize; m++){
                glVertex3d(config.pixelSize*(n-(patchSize/2)), config.pixelSize*(m-(patchSize/2)), 0);
            }
        }
        glEnd();
    glPopMatrix();
    glEndList();
    return index;
}

/// transpose ids matrix
void QGLVisualizer::transposeIds(std::array<std::array<int,3>,3>& ids){
    std::array<std::array<int,3>,3> idsNew;
    for (int i=0;i<3;i++)
        for (int j=0;j<3;j++)
            idsNew[j][i] = ids[i][j];
    ids = idsNew;
}

/// flip horizontal ids matrix
void QGLVisualizer::flipIds(std::array<std::array<int,3>,3>& ids){
    std::array<std::array<int,3>,3> idsNew;
    for (int i=0;i<3;i++)
        for (int j=0;j<3;j++)
            idsNew[i][j] = ids[j][2-i];
    ids = idsNew;
}

/// flip horizontal gaussians matrix
void QGLVisualizer::flipGaussians(std::array<std::array<hop3d::Gaussian3D,3>,3>& gaussians){
    std::array<std::array<hop3d::Gaussian3D,3>,3> gaussiansNew;
    for (int i=0;i<3;i++)
        for (int j=0;j<3;j++)
            gaussiansNew[i][j] = gaussians[j][2-i];
    gaussians = gaussiansNew;
}

/// transpose gaussians matrix
void QGLVisualizer::transposeGaussians(std::array<std::array<Gaussian3D,3>,3>& gaussians){
    std::array<std::array<Gaussian3D,3>,3> gaussiansNew;
    for (int i=0;i<3;i++)
        for (int j=0;j<3;j++)
            gaussiansNew[j][i] = gaussians[i][j];
    gaussians = gaussiansNew;
}

/// Create view independent part list
GLuint QGLVisualizer::createVIPartList(hop3d::ViewIndependentPart& part, int layerNo){
    // create one display list
    GLuint index = glGenLists(1);
    glNewList(index, GL_COMPILE);
    glPushMatrix();
    if (layerNo==4){
        int id = part.group.begin()->id;
        Mat34 pose = part.pose.inverse();
        double GLmat[16]={pose(0,0), pose(1,0), pose(2,0), 0, pose(0,1), pose(1,1), pose(2,1), 0, pose(0,2), pose(1,2), pose(2,2), 0, 0, 0, 0, 1};
        glPushMatrix();
            glMultMatrixd(GLmat);
            if (id==-1){
                //glColor3ub(100,50,50);
                glCallList(backgroundList[hierarchy->viewDependentLayers.size()]);
            }
            else{
                //glColor3ub(200,200,200);
                glCallList(cloudsListLayers[hierarchy->viewDependentLayers.size()][id]);
            }
        glPopMatrix();
    }
    else {
        for (size_t n = 0; n < part.partIds.size(); n++){
            for (size_t m = 0; m < part.partIds[n].size(); m++){
                for (size_t l = 0; l < part.partIds[l].size(); l++){
                    int id = part.partIds[n][m][l];
                    Mat34 partPose = part.neighbourPoses[n][m][l];
                    Vec3 pos(partPose(0,3), partPose(1,3), partPose(2,3));
                    if (id==-1){
                        pos(0)=config.voxelSize*double(double(m)-1.0);
                        pos(1)=config.voxelSize*double(double(n)-1.0);
                        pos(2)=config.voxelSize*double(double(l)-1.0);
                    }
                    double GLmatrot[16]={partPose(0,0), partPose(1,0), partPose(2,0), 0,
                                      partPose(0,1), partPose(1,1), partPose(2,1), 0,
                                      partPose(0,2), partPose(1,2), partPose(2,2), 0,
                                      partPose(0,3), partPose(1,3), partPose(2,3), 1};
                    if (id==-1){
                        GLmatrot[0]=1; GLmatrot[1]=0; GLmatrot[2]=0;
                        GLmatrot[4]=0; GLmatrot[5]=1; GLmatrot[6]=0;
                        GLmatrot[8]=0; GLmatrot[9]=0; GLmatrot[10]=1;
                        GLmatrot[12]=0; GLmatrot[13]=0; GLmatrot[14]=0;
                    }
                    glPushMatrix();
                        glMultMatrixd(GLmatrot);
                        if (id==-1){
                            glColor3ub(200,200,200);
                            glBegin(GL_POINTS);
                                glVertex3d(pos(0), pos(1), pos(2));
                            glEnd();
                        }
                        else{
                            //this->drawAxis(0.5);
                            //glColor3ub(200,200,200);
                            glCallList(cloudsListLayers[layerNo-2][id]);
                        }
                        //if (n==1&&m==1&&l==1)
                            //this->drawAxis(0.5);
                    glPopMatrix();
                }
            }
        }
    }
    glPopMatrix();
    glEndList();
    return index;
}

/// Create point cloud List
GLuint QGLVisualizer::createPartList(ViewDependentPart& part, int layerNo){
    // create one display list
    GLuint index = glGenLists(1);
    glNewList(index, GL_COMPILE);
    glPushMatrix();
    //flipIds(part.partIds);// because it's more natural for user
    //flipGaussians(part.gaussians);
    for (size_t n = 0; n < part.partIds.size(); n++){
        for (size_t m = 0; m < part.partIds[n].size(); m++){
            Vec3 pos(config.pixelSize*part.gaussians[n][m].mean(0), config.pixelSize*part.gaussians[n][m].mean(1), part.gaussians[n][m].mean(2));
            int id = part.partIds[n][m];
            if ((n==1)&&(m==1)){
                pos(0)=0; pos(1)=0; pos(2)=0;
            }
            /*else if (id==-1){
                double patchSize = 5.0*(2.0*layerNo-1.0);
                pos(0)=config.pixelSize*double(double(m)-1.0)*(patchSize+(patchSize/2.0));
                pos(1)=config.pixelSize*double(double(n)-1.0)*(patchSize+(patchSize/2.0));
                pos(2)=0;
            }*/
            double GLmat[16]={1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, pos(0), pos(1), pos(2), 1};
            //std::cout << id << " pos " << pos.transpose() << "\n";
            glPushMatrix();
                glMultMatrixd(GLmat);
                if (id==-1){
                    //glColor3ub(100,50,50);
                    glCallList(backgroundList[layerNo-1]);
                }
                else{
                    //glColor3ub(200,200,200);
                    glCallList(cloudsListLayers[layerNo-1][id]);
                }
            glPopMatrix();
        }
    }
    glPopMatrix();
    glEndList();
    return index;
}

/// Create point cloud List
GLuint QGLVisualizer::createObjList(const std::vector<ViewIndependentPart>& parts, int layerNo){
    // create one display list
    GLuint index = glGenLists(1);
    glNewList(index, GL_COMPILE);
    glPushMatrix();
    if (layerNo==0){
        Vec3 initPose(parts.begin()->parts.begin()->pose(0,3), parts.begin()->parts.begin()->pose(1,3), parts.begin()->parts.begin()->pose(2,3));
        for (auto & part : parts){
            for (auto & partL : part.parts){
                Vec3 pos(partL.pose(0,3), partL.pose(1,3), partL.pose(2,3));
                double GLmat[16]={1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, pos(0)-initPose(0), pos(1)-initPose(1), pos(2)-initPose(2), 1};
                glPushMatrix();
                    glMultMatrixd(GLmat);
                    glCallList(cloudsListLayers[2][partL.id]);
                glPopMatrix();
            }
            /*
            /// if first view-independent parts are used
            Vec3 pos(part.pose(0,3), part.pose(1,3), part.pose(2,3));
            double GLmat[16]={part.pose(0,0), part.pose(1,0), part.pose(2,0), 0, part.pose(0,1), part.pose(1,1), part.pose(2,1), 0, part.pose(0,2), part.pose(1,2), part.pose(2,2), 0, pos(0)-initPose(0), pos(1)-initPose(1), pos(2)-initPose(2), 1};
            glPushMatrix();
                glMultMatrixd(GLmat);
                glCallList(cloudsListLayers[3][hierarchy.get()->interpreter[part.id]]);
            glPopMatrix();
            */
        }
    }
    if (layerNo==1){
        Vec3 initPose(parts.begin()->pose(0,3), parts.begin()->pose(1,3), parts.begin()->pose(2,3));
        int partNo=0;
        for (auto & part : parts){
            /*std::cout << "part no " << partNo <<"\n";
            std::cout << "part id " << part.id <<"\n";
            //std::cout << "part pose \n" << part.pose.matrix() <<"\n";
            std::cout << "part.offset\n" << part.offset.matrix() << "\n";*/
            partNo++;

            Mat34 pose = part.pose*part.offset;
            Vec3 pos(pose(0,3), pose(1,3), pose(2,3));
            double GLmat[16]={pose(0,0), pose(1,0), pose(2,0), 0, pose(0,1), pose(1,1), pose(1,2), 0, pose(0,2), pose(1,2), pose(2,2), 0, pos(0)-initPose(0), pos(1)-initPose(1), pos(2)-initPose(2), 1};
            glPushMatrix();
                glMultMatrixd(GLmat);
                //glCallList(backgroundList[1]);
                glCallList(cloudsListLayers[4][part.id]);
            glPopMatrix();
        }
    }
    /*for (size_t n = 0; n < part.partIds.size(); n++){
        for (size_t m = 0; m < part.partIds[n].size(); m++){
            Vec3 pos(config.pixelSize*part.gaussians[n][m].mean(0), config.pixelSize*part.gaussians[n][m].mean(1), part.gaussians[n][m].mean(2));
            int id = part.partIds[n][m];
            if ((n==1)&&(m==1)){
                pos(0)=0; pos(1)=0; pos(2)=0;
            }
            else if (id==-1){
                double patchSize = 5.0*(2.0*layerNo-1.0);
                pos(0)=config.pixelSize*double(double(n)-1.0)*(patchSize+(patchSize/2.0));
                pos(1)=config.pixelSize*double(double(m)-1.0)*(patchSize+(patchSize/2.0));
                pos(2)=0;
            }
            double GLmat[16]={1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, pos(0), pos(1), pos(2), 1};
            glPushMatrix();
                glMultMatrixd(GLmat);
                if (id==-1){
                    //glColor3ub(100,50,50);
                    glCallList(backgroundList[layerNo-1]);
                }
                else{
                    //glColor3ub(200,200,200);
                    glCallList(cloudsListLayers[layerNo-1][id]);
                }
            glPopMatrix();
        }
    }*/
    glPopMatrix();
    glEndList();
    return index;
}

/// Create clusters List
GLuint QGLVisualizer::createVIClustersList(ViewIndependentPart& part, int layerNo){
    // create one display list
    GLuint index = glGenLists(1);
    glNewList(index, GL_COMPILE);
    glPushMatrix();
    int componentNo=0;
    if (layerNo==4){
        for (auto & partId : part.group){
            Mat34 pose = partId.pose.inverse();
            double GLmat[16]={pose(0,0), pose(1,0), pose(2,0), 0, pose(0,1), pose(1,1), pose(2,1), 0, pose(0,2), pose(1,2), pose(2,2), 0, 0, (double)(config.partDist[layerNo-1]*double(componentNo+1)), 0, 1};
            glPushMatrix();
                glMultMatrixd(GLmat);
                if (partId.id==-1){
                    //glColor3ub(100,50,50);
                    glCallList(backgroundList[hierarchy->viewDependentLayers.size()]);
                }
                else{
                    //glColor3ub(200,200,200);
                    //glCallList(backgroundList[hierarchy->viewDependentLayers.size()-1]);
                    glCallList(cloudsListLayers[hierarchy->viewDependentLayers.size()][partId.id]);
                }
            glPopMatrix();
            componentNo++;
        }
    }
    else {
        for (auto itComp = part.group.begin(); itComp!=part.group.end();itComp++){
                for (size_t n = 0; n < part.partIds.size(); n++){
                    for (size_t m = 0; m < part.partIds[n].size(); m++){
                        for (size_t l = 0; l < part.partIds[l].size(); l++){
                            int id = itComp->partIds[n][m][l];
                            Mat34 partPose = itComp->neighbourPoses[n][m][l];
                            Vec3 pos(partPose(0,3), partPose(1,3), partPose(2,3));
                            if (id==-1){
                                pos(0)=config.voxelSize*double(double(m)-1.0);
                                pos(1)=config.voxelSize*double(double(n)-1.0);
                                pos(2)=config.voxelSize*double(double(l)-1.0);
                            }
                            double GLmatrot[16]={partPose(0,0), partPose(1,0), partPose(2,0), 0,
                                              partPose(0,1), partPose(1,1), partPose(2,1), 0,
                                              partPose(0,2), partPose(1,2), partPose(2,2), 0,
                                              partPose(0,3), partPose(1,3)+(double)(config.partDist[layerNo-1]*double(componentNo+1)), partPose(2,3), 1};
                            if (id==-1){
                                GLmatrot[0]=1; GLmatrot[1]=0; GLmatrot[2]=0;
                                GLmatrot[4]=0; GLmatrot[5]=1; GLmatrot[6]=0;
                                GLmatrot[8]=0; GLmatrot[9]=0; GLmatrot[10]=1;
                                GLmatrot[12]=0; GLmatrot[13]=0; GLmatrot[14]=0;
                            }
                            glPushMatrix();
                                glMultMatrixd(GLmatrot);
                                if (id==-1){
                                    glColor3ub(200,200,200);
                                    glBegin(GL_POINTS);
                                        glVertex3d(pos(0), pos(1)+(double)(config.partDist[layerNo-1]*double(componentNo+1)), pos(2));
                                    glEnd();
                                }
                                else{
                                    //this->drawAxis(0.5);
                                    //glColor3ub(200,200,200);
                                    glCallList(cloudsListLayers[layerNo-2][id]);
                                }
                            glPopMatrix();
                        }
                    }
                }
                componentNo++;
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
    //flipIds(part.partIds);// because it's more natural for user
    //flipGaussians(part.gaussians);
    for (auto itComp = part.group.begin(); itComp!=part.group.end();itComp++){
        for (size_t n = 0; n < itComp->partIds.size(); n++){
            for (size_t m = 0; m < itComp->partIds[n].size(); m++){
                Vec3 pos(config.pixelSize*itComp->gaussians[n][m].mean(0), config.pixelSize*itComp->gaussians[n][m].mean(1), itComp->gaussians[n][m].mean(2));
                if ((n==1)&&(m==1)){
                    pos(0)=0; pos(1)=0; pos(2)=0;
                }
                /*else{
                    pos(0)=config.pixelSize*double(double(n)-1.0)*5.0;
                    pos(1)=config.pixelSize*double(double(m)-1.0)*5.0;
                }*/
                double GLmat[16]={1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, pos(0), pos(1)+(double)(config.partDist[layerNo]*double(componentNo+1)), pos(2), 1};
                glPushMatrix();
                    glMultMatrixd(GLmat);
                    //glColor4d(config.clustersColor.red(), config.clustersColor.green(), config.clustersColor.blue(), config.clustersColor.alpha());
                    int id = itComp->partIds[n][m];
                    if (id==-1){
                        //glColor3ub(100,50,50);
                        glCallList(backgroundList[layerNo-1]);
                    }
                    else{
                        //glColor3ub(200,200,200);
                        glCallList(cloudsListLayers[layerNo-1][id]);
                    }
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
GLuint QGLVisualizer::createCloudList(hop3d::PointCloud& pointCloud, Vec3& normal){
    // create one display list
    GLuint index = glGenLists(1);
    // compile the display list, store a triangle in it
    glNewList(index, GL_COMPILE);
        glColor3ub(200,200,200);
        glBegin(GL_POINTS);
        for (size_t n = 0; n < pointCloud.pointCloudNormal.size(); n++){
            //glColor3ub(pointCloud.second.pointCloudNormal[n].r, pointCloud.second.pointCloudNormal[n].g, pointCloud.second.pointCloudNormal[n].b);
            glVertex3d(pointCloud.pointCloudNormal[n].position(0),
                      pointCloud.pointCloudNormal[n].position(1),
                      pointCloud.pointCloudNormal[n].position(2)*config.filterDepthScale);
        }
        glEnd();
        if (config.drawNormals){
            glBegin(GL_LINES);
                glVertex3d(0.0, 0.0, 0.0);
                glVertex3d(-normal(0)*config.normalsScale, -normal(1)*config.normalsScale, -normal(2)*config.normalsScale);
            glEnd();
        }
    glEndList();
    return index;
}

/// Create layer 2 layer List
GLuint QGLVisualizer::createVILinksList(int destLayerNo){
    // create one display list
    GLuint index = glGenLists(1);
    // compile the display list, store a triangle in it
    glNewList(index, GL_COMPILE);
        glLineWidth((float)config.layer2LayerWidth);
        glColor4d(config.layer2LayerColor.red(), config.layer2LayerColor.green(), config.layer2LayerColor.blue(), config.layer2LayerColor.alpha());
        glBegin(GL_LINES);
        if (destLayerNo==4){
            int partNo=0;
            for (auto it = hierarchy->viewIndependentLayers[0].begin(); it!=hierarchy->viewIndependentLayers[0].end(); it++){
                // position of the i-th filter
                int id = it->group.begin()->id;
                Vec3 filterPos((double)(config.partDist[destLayerNo-2]*double(id)-((double)cloudsListLayers[destLayerNo-2].size()/2)*config.partDist[destLayerNo-2]), 0, config.posZ[destLayerNo-2]);
                glVertex3d(filterPos(0), filterPos(1), filterPos(2));
                // position of the i-th part
                Vec3 partPos((double)(config.partDist[destLayerNo-1]*double(partNo)-((double)cloudsListLayers[destLayerNo-1].size()/2)*config.partDist[destLayerNo-1]), 0, config.posZ[destLayerNo-1]);
                glVertex3d(partPos(0), partPos(1), partPos(2));
                partNo++;
            }
        }
        else {
            int partNo=0;
            for (auto it = hierarchy->viewIndependentLayers[destLayerNo-4].begin(); it!=hierarchy->viewIndependentLayers[destLayerNo-4].end(); it++){
                std::vector<int> prevPartIds;
                for (size_t n = 0; n < it->partIds.size(); n++){
                    for (size_t m = 0; m < it->partIds[n].size(); m++){
                        for (size_t l = 0; l < it->partIds[n][m].size(); l++){
                            if (it->partIds[n][m][l]!=-1)
                                prevPartIds.push_back(it->partIds[n][m][l]);
                        }
                    }
                }
                auto itPrevPart = std::unique (prevPartIds.begin(), prevPartIds.end());
                prevPartIds.resize( std::distance(prevPartIds.begin(),itPrevPart) );
                for (size_t i=0;i<prevPartIds.size();i++){
                    // position of the i-th l-1 layer part
                    Vec3 prevPartPos((double)(config.partDist[destLayerNo-2]*double(prevPartIds[i])-((double)cloudsListLayers[destLayerNo-2].size()/2)*config.partDist[destLayerNo-2]), 0, config.posZ[destLayerNo-2]);
                    glVertex3d(prevPartPos(0), prevPartPos(1), prevPartPos(2));
                    // position of the i-th part
                    Vec3 partPos((double)(config.partDist[destLayerNo-1]*double(partNo)-((double)cloudsListLayers[destLayerNo-1].size()/2)*config.partDist[destLayerNo-1]), 0, config.posZ[destLayerNo-1]);
                    glVertex3d(partPos(0), partPos(1), partPos(2));
                }
                partNo++;
            }
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
    for (int layerNo=1;layerNo<(int)hierarchy->viewDependentLayers.size()+3;layerNo++){
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
    if (config.draw3Dobjects)
        draw3Dobjects();
    drawPointClouds();
    updateHierarchy();
    update3Dobjects();
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

    //qglviewer::Quaternion q(-0.5,0.5,0.5,0.5);
    //camera()->setOrientation(q);

    glDepthRange(1,0);

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

