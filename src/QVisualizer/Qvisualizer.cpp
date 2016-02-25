#include "hop3d/QVisualizer/Qvisualizer.h"
#include "hop3d/ImageFilter/normalImageFilter.h"
#include <memory>
#include <cmath>
#include <stdexcept>
#include <chrono>
#include <GL/glut.h>
#include <QKeyEvent>

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
                double const y = sin( -(M_PI/2.0) + M_PI * r * R );
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

QGLVisualizer::QGLVisualizer(void) : updatePartsObjectsFlag(false){
    hierarchy.reset(new Hierarchy());
    cloudsListLayers.resize(7);
    clustersList.resize(7);
    linksLists.resize(7);
    layersOfObjects.resize(7);
    layersOfObjectsInference.resize(7);
    objects3Dlist.resize(7);
    objectsFromParts.resize(7);
    objectsFromPartsInference.resize(7);
    activeLayer=0;
    activeOverlapNo = 0;
}

/// Construction
QGLVisualizer::QGLVisualizer(Config _config): config(_config), updateHierarchyFlag(false), updatePartsObjectsFlag(false){
    hierarchy.reset(new Hierarchy());
    cloudsListLayers.resize(7);
    clustersList.resize(7);
    linksLists.resize(7);
    layersOfObjects.resize(7);
    layersOfObjectsInference.resize(7);
    objects3Dlist.resize(7);
    objectsFromParts.resize(7);
    objectsFromPartsInference.resize(7);
    activeLayer=0;
    activeOverlapNo = 0;
}

/// Construction
QGLVisualizer::QGLVisualizer(std::string configFile) :
        config(configFile), updateHierarchyFlag(false) {
    tinyxml2::XMLDocument configXML;
    std::string filename = configFile;
    configXML.LoadFile(filename.c_str());
    if (configXML.ErrorID())
		throw std::runtime_error("unable to load visualizer config file: " + filename);
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

/// update part clouds
void QGLVisualizer::update(std::vector<std::vector<std::vector<hop3d::PointCloudRGBA>>>& objects){
    activeLayer=0;
    activeOverlapNo=0;
    partClouds = objects;
}

/// update coords
void QGLVisualizer::update(const std::vector<hop3d::Mat34>& coords){
    updateCoordinates=true;
    partsCoordinates = coords;
}

/// create coordinates list
void QGLVisualizer::createCoordsList(void){
    updateCoordinates = false;
    // create one display list
    GLuint index = glGenLists(1);
    // compile the display list
    ///Vec3 offset(partClouds[0][0][0].front().position);
    glNewList(index, GL_COMPILE);
        for (auto &coord : partsCoordinates){
            double GLmat[16]={coord(0,0), coord(1,0), coord(2,0), 0, coord(0,1), coord(1,1), coord(2,1), 0, coord(0,2), coord(1,2), coord(2,2), 0, -offsetCoords(0)+coord(0,3), -offsetCoords(1)+coord(1,3), -0.1-offsetCoords(2)+coord(2,3), 1};
            glPushMatrix();
                glMultMatrixd(GLmat);
                drawAxis(float(0.02));
            glPopMatrix();
        }
    glEndList();
    partsCoordinatesList.push_back(index);
}

/// Update parts objects
void QGLVisualizer::updatePartsObjects(void){
    if (updatePartsObjectsFlag){
        updatePartsObjectsFlag = false;
        partObjectsLists.resize(partClouds.size());
        for (size_t overlapNo=0;overlapNo<partClouds.size();overlapNo++){
            for (size_t layerNo=0;layerNo<partClouds[overlapNo].size();layerNo++){
                partObjectsLists[overlapNo].push_back(createPartObjList(partClouds[overlapNo][layerNo]));
            }
        }
        offsetCoords=partClouds[0][0][0].front().position;
    }
    partClouds.clear();
}

/// update second layer part
void QGLVisualizer::updateSecondLayerPart(const hop3d::PointsSecondLayer& part){
    pointParts2update.push_back(part);
}

/// update second layer part
void QGLVisualizer::updateSecondLayerPart(void){
    if (pointParts2update.size()>0){
        Vec3 offset(0.1,0.1,0.1);
        hop3d::PointsSecondLayer part = pointParts2update.back();
        pointParts2update.clear();
        // create one display list
        GLuint index = glGenLists(1);
        // compile the display list, store a triangle in it
        glNewList(index, GL_COMPILE);
        glPushMatrix();
        for (size_t n = 0; n < part.size(); n++){
            for (size_t m = 0; m < part[0].size(); m++){
                Vec3 posPart(part[n][m].mean.block<3,1>(0,0));
                posPart+=offset;
                if (n==1&&m==1)
                    posPart=Vec3(0,0,0);
                double GLmat1[16]={1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, posPart(0), posPart(1), posPart(2), 1};
                glPushMatrix();
                    glMultMatrixd(GLmat1);
                    if (!std::isnan(posPart(0))){
                        glColor3d(0.5,0.5,0.5);
                        drawPatch(Vec3(part[n][m].mean.block<3,1>(3,0)));
                    }
                glPopMatrix();
            }
        }
        glPopMatrix();
        glEndList();
        secondLayerPartList.push_back(index);
    }
}

/// create partObjects
void QGLVisualizer::createPartObjects(){
    updatePartsObjectsFlag = true;
}

/// Update 3D object model
void QGLVisualizer::update(const std::vector<ViewIndependentPart>& objectParts, int objLayerId, bool inference){
    mtxHierarchy.lock();
    if (inference)
        layersOfObjectsInference[objLayerId].push_back(objectParts);
    else
        layersOfObjects[objLayerId].push_back(objectParts);
    mtxHierarchy.unlock();
}

/// update object from parts
void QGLVisualizer::update(std::vector<std::pair<int, Mat34>>& partsPoses, int objectNo, int layerNo, bool inference){
    std::vector<std::vector<std::vector<Part3D>>>* objParts;
    if (inference)
        objParts = &objectsFromPartsInference;
    else
        objParts = &objectsFromParts;
    if ((*objParts)[layerNo].size()<=(size_t)objectNo){
        (*objParts)[layerNo].resize(objectNo+1);
    }
    for (auto& part : partsPoses){
        Part3D partPose;
        partPose.id = part.first;
        partPose.pose = part.second;
        (*objParts)[layerNo][objectNo].push_back(partPose);
    }
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
        for (auto & layer : layersOfObjects){//view independent layers
            int objectNo=0;
            for (auto & object : layer){
                objects3Dlist[layerNo].push_back(createObjList(object,layerNo+(int)hierarchy.get()->viewDepPartsFromLayerNo, false));
                objectNo++;
            }
            layersOfObjects[layerNo].clear();
            layerNo++;
        }
        for (layerNo = 0;layerNo<3;layerNo++){//objects from parts of View-dependent layers
            for (auto& object : objectsFromParts[layerNo]){
                objects3Dlist[layerNo].push_back(createObjList(object, layerNo, false));
            }
            objectsFromParts[layerNo].clear();
        }
        layerNo=0;
        for (auto & layer : layersOfObjectsInference){//view independent layers
            int objectNo=0;
            for (auto & object : layer){
                objects3Dlist[layerNo].push_back(createObjList(object,layerNo+(int)hierarchy.get()->viewDepPartsFromLayerNo, true));
                objectNo++;
            }
            layersOfObjectsInference[layerNo].clear();
            layerNo++;
        }
        for (layerNo = 0;layerNo<3;layerNo++){//objects from parts of View-dependent layers
            for (auto& object : objectsFromPartsInference[layerNo]){
                objects3Dlist[layerNo].push_back(createObjList(object, layerNo, true));
            }
            objectsFromPartsInference[layerNo].clear();
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
                        cloud.push_back(point);
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
            for (auto &part : hierarchy->viewDependentLayers[i]){
                cloudsListLayers[i+1].push_back(createPartList(part, int(i+1)));
                clustersList[i+1].push_back(createClustersList(part, int(i+1)));
            }
            linksLists[i+1].push_back(createLinksList(int(i+1)));
        }
        for (size_t i=0;i<hierarchy->viewIndependentLayers.size();i++){//view-independent
            if (config.verbose==1){
                std::cout << "layer "<< i+hierarchy->viewDependentLayers.size()+2 << " size: " << hierarchy->viewIndependentLayers[i].size() << "\n";
            }
            for (auto it = hierarchy->viewIndependentLayers[i].begin(); it!=hierarchy->viewIndependentLayers[i].end(); it++){
                cloudsListLayers[i+hierarchy->viewDependentLayers.size()+1].push_back(createVIPartList(*it));
                clustersList[i+hierarchy->viewDependentLayers.size()+1].push_back(createVIClustersList(*it, int(i+4)));
            }
            //linksLists[i+3].push_back(createVILinksList(int(i+4)));
        }
        mtxHierarchy.unlock();
    }
    else{
		std::this_thread::sleep_for(std::chrono::microseconds(20000));
    }
}

/// Draw point clouds
void QGLVisualizer::drawPointClouds(void){
    //mtxPointClouds.lock();
    for (int layerNo=0;layerNo<(int)cloudsListLayers.size();layerNo++){
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
    for (int layerNo=0;layerNo<6;layerNo++){
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

/// Draw part objects
void QGLVisualizer::drawPartObjects(void){
    if (activeOverlapNo<(int)partObjectsLists.size())
        if (activeLayer<(int)partObjectsLists[activeOverlapNo].size())
            glCallList(partObjectsLists[activeOverlapNo][activeLayer]);
}

/// Draw clusters
void QGLVisualizer::drawClusters(void){
    //mtxPointClouds.lock();
    for (size_t layerNo=0;layerNo<clustersList.size()-1;layerNo++){
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
         //       glVertex3d(config.pixelSize*(n-(patchSize/2)), config.pixelSize*(m-(patchSize/2)), 0);
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
GLuint QGLVisualizer::createVIPartList(hop3d::ViewIndependentPart& part){
    // create one display list
    GLuint index = glGenLists(1);
    glNewList(index, GL_COMPILE);
    glPushMatrix();
    //int id = part.group.begin()->id;
    Mat34 pose = Mat34::Identity();//part.pose.inverse();
    double GLmat[16]={pose(0,0), pose(1,0), pose(2,0), 0, pose(0,1), pose(1,1), pose(2,1), 0, pose(0,2), pose(1,2), pose(2,2), 0, 0, 0, 0, 1};
    for (auto& patch : part.cloud){
        glPushMatrix();
        GLmat[12] = patch.position(0); GLmat[13] = patch.position(1); GLmat[14] = patch.position(2);
        glMultMatrixd(GLmat);
        glColor3ub(200,200,200);
        drawPatch(patch.normal);
        glPopMatrix();
    }
    glPopMatrix();
    glEndList();
    return index;
}

/// draw coordinates list
void QGLVisualizer::drawCoords(void){
    if (partsCoordinatesList.size()>0){
        for (auto &index : partsCoordinatesList)
            glCallList(index);
    }
}

/// draw part
void QGLVisualizer::drawPart(const ViewDependentPart& part, int layerNo, double r, double g, double b){
    glPushMatrix();
    //flipIds(part.partIds);// because it's more natural for user
    //flipGaussians(part.gaussians);
    for (size_t n = 0; n < part.partIds.size(); n++){
        for (size_t m = 0; m < part.partIds[n].size(); m++){
            int id = part.partIds[n][m];
            Vec3 posPart(part.partsPosNorm[n][m].mean.block<3,1>(0,0));
            if (n==1&&m==1)
                posPart=Vec3(0,0,0);
            double GLmat1[16]={1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, posPart(0), posPart(1), posPart(2), 1};
            glPushMatrix();
                glMultMatrixd(GLmat1);
                if (id<0){
                    //glColor3ub(100,50,50);
                    glCallList(backgroundList[layerNo-1]);
                }
                else{
                    glColor3d(r, g, b);
                    drawPatch(Vec3(part.partsPosNorm[n][m].mean.block<3,1>(3,0)));
                }
            glPopMatrix();
        }
    }
    glPopMatrix();
}

/// draw part
void QGLVisualizer::drawPartMesh(const ViewDependentPart& part, double r, double g, double b){
    glPushMatrix();
    std::array<std::array<Vec6,3>,3> posNormFull;
    for (size_t n = 0; n < part.partIds.size()-1; n++){
        for (size_t m = 0; m < part.partIds[n].size()-1; m++){
            std::vector<Vec6> vertices;
            std::vector<int> ids;
            for (int i=0;i<2;i++){
                for (int j=i;j<2;j++){
                    if ((n+j)==1&&(m+i)==1){
                        Vec6 pos = part.partsPosNorm[n+j][m+i].mean;
                        pos.block<3,1>(0,0)=Vec3(0,0,0);
                        vertices.push_back(pos);
                    }
                    else{
                        if (part.partIds[n+j][m+i]>0){
                            vertices.push_back(part.partsPosNorm[n+j][m+i].mean);
                            posNormFull[n+j][m+i]=part.partsPosNorm[n+j][m+i].mean;
                        }
                        else {
                            double meanDepth = computeMeanDepth(part,(int)n+j,(int)m+i, posNormFull[n+j][m+i]);
                            Vec6 pos = part.partsPosNorm[n+j][m+i].mean;
                            pos(2) = meanDepth;
                            posNormFull[n+j][m+i].block<3,1>(0,0)=pos.block<3,1>(0,0);
                            vertices.push_back(pos);
                        }
                    }
                    ids.push_back(part.partIds[n+j][m+i]);
                }
            }
            //getchar();
            double maxDepthDiff=std::numeric_limits<double>::min();
            double dists[3]={fabs(vertices[0](2)-vertices[1](2)), fabs(vertices[1](2)-vertices[2](2)), fabs(vertices[0](2)-vertices[2](2))};
            for (int vNo=0; vNo<3;vNo++)
                if (dists[vNo]>maxDepthDiff)
                    maxDepthDiff=dists[vNo];
            double maxDepthDistThreshold=0.015;
            if (ids[0]>0&&ids[1]>0&&ids[2]>0&&maxDepthDiff<maxDepthDistThreshold)
                drawTriangle(vertices,ids, r,g,b);
            vertices.clear();
            ids.clear();
            for (int i=0;i<2;i++){
                for (int j=i;j<2;j++){
                    if ((n+i)==1&&(m+j)==1){
                        Vec6 pos = part.partsPosNorm[n+i][m+j].mean;
                        pos.block<3,1>(0,0)=Vec3(0,0,0);
                        vertices.push_back(pos);
                    }
                    else{
                        if (part.partIds[n+i][m+j]>0){
                            vertices.push_back(part.partsPosNorm[n+i][m+j].mean);
                            posNormFull[n+i][m+j]=part.partsPosNorm[n+i][m+j].mean;
                        }
                        else {
                            double meanDepth = computeMeanDepth(part,(int)n+i,(int)m+j, posNormFull[n+i][m+j]);
                            Vec6 pos = part.partsPosNorm[n+i][m+j].mean;
                            pos(2) = meanDepth;
                            posNormFull[n+i][m+j].block<3,1>(0,0)=pos.block<3,1>(0,0);
                            vertices.push_back(pos);
                        }
                    }
                    ids.push_back(part.partIds[n+i][m+j]);
                }
            }
            double maxDepthDiff2=std::numeric_limits<double>::min();
            double dists2[3]={fabs(vertices[0](2)-vertices[1](2)), fabs(vertices[1](2)-vertices[2](2)), fabs(vertices[0](2)-vertices[2](2))};
            for (int vNo=0; vNo<3;vNo++)
                if (dists2[vNo]>maxDepthDiff2)
                    maxDepthDiff2=dists2[vNo];
            if (ids[0]>0&&ids[1]>0&&ids[2]>0&&maxDepthDiff2<maxDepthDistThreshold)
                drawTriangle(vertices,ids, r,g,b);
        }
    }
    drawOctagon(posNormFull,part.partIds, r, g,b);
    if (config.drawNormals){
        std::vector<Vec6> points;
        std::vector<int> ids;
        for (size_t n = 0; n < part.partIds.size(); n++){
            for (size_t m = 0; m < part.partIds[n].size(); m++){
                if (n==1&&m==1){
                    Vec6 pos = part.partsPosNorm[n][m].mean;
                    pos.block<3,1>(0,0) = Vec3(0,0,0);
                    points.push_back(pos);
                }
                else
                    points.push_back(part.partsPosNorm[n][m].mean);
                ids.push_back(part.partIds[n][m]);
            }
        }
        drawNormals(points, ids);
    }
    glPopMatrix();
}

/// draw shaded octagon
void QGLVisualizer::drawOctagon(const std::array<std::array<Vec6,3>,3>& part, const std::array<std::array<int,3>,3>& ids, double r, double g, double b) const{
    double radius = 0.01;
    std::vector<std::pair<int,int>> seq;
    seq.push_back(std::make_pair(1,2)); seq.push_back(std::make_pair(0,2)); seq.push_back(std::make_pair(0,1));
    seq.push_back(std::make_pair(0,0)); seq.push_back(std::make_pair(1,0)); seq.push_back(std::make_pair(2,0));
    seq.push_back(std::make_pair(2,1)); seq.push_back(std::make_pair(2,2)); seq.push_back(std::make_pair(1,2));
    double angle=0;
    for (int i=0;i<8;i++){
        Vec3 point(radius*cos(angle), radius*sin(angle),0);
        angle-=M_PI/4.0;
        Mat33 rot = NormalImageFilter::coordinateFromNormal(part[seq[i].first][seq[i].second].block<3,1>(3,0));
        point=rot*point;
        //point(2)+=part[seq[i].first][seq[i].second](2);

        Vec3 pointNext(radius*cos(angle), radius*sin(angle),0);
        Mat33 rotNext = NormalImageFilter::coordinateFromNormal(part[seq[i+1].first][seq[i+1].second].block<3,1>(3,0));
        pointNext=rotNext*pointNext;
        //pointNext(2)+=part[seq[i+1].first][seq[i+1].second](2);

        std::vector<Vec6> vertices;
        std::vector<int> idsVec;
        Vec6 p; p.block<3,1>(0,0)=point.block<3,1>(0,0); p.block<3,1>(3,0) = part[seq[i].first][seq[i].second].block<3,1>(3,0);
        vertices.push_back(p);
        p.block<3,1>(0,0)=pointNext.block<3,1>(0,0); p.block<3,1>(3,0) = part[seq[i+1].first][seq[i+1].second].block<3,1>(3,0);
        vertices.push_back(p);
        vertices.push_back(part[seq[i].first][seq[i].second]);
        idsVec.push_back(1); idsVec.push_back(1); idsVec.push_back(ids[seq[i].first][seq[i].second]);
        double maxDepthDiff=std::numeric_limits<double>::min();
        double dists[4]={fabs(vertices[0](2)-vertices[1](2)), fabs(vertices[1](2)-vertices[2](2)), fabs(vertices[0](2)-vertices[2](2)), fabs(part[seq[i+1].first][seq[i+1].second](2)-vertices[0](2))};
        for (int vNo=0; vNo<4;vNo++)
            if (dists[vNo]>maxDepthDiff)
                maxDepthDiff=dists[vNo];
        double maxDepthDistThreshold=0.015;
        if (ids[seq[i].first][seq[i].second]>0&&ids[seq[i+1].first][seq[i+1].second]>0&&maxDepthDiff<maxDepthDistThreshold)
            drawTriangle(vertices,idsVec, r,g,b);

        vertices.clear();
        idsVec.clear();
        p.block<3,1>(0,0)=pointNext.block<3,1>(0,0); p.block<3,1>(3,0) = part[seq[i+1].first][seq[i+1].second].block<3,1>(3,0);
        vertices.push_back(p);
        vertices.push_back(part[seq[i+1].first][seq[i+1].second]);
        vertices.push_back(part[seq[i].first][seq[i].second]);
        idsVec.push_back(1); idsVec.push_back(ids[seq[i+1].first][seq[i+1].second]); idsVec.push_back(ids[seq[i].first][seq[i].second]);
        double maxDepthDiff2=std::numeric_limits<double>::min();
        double dists2[3]={fabs(vertices[0](2)-vertices[1](2)), fabs(vertices[1](2)-vertices[2](2)), fabs(vertices[0](2)-vertices[2](2))};
        for (int vNo=0; vNo<3;vNo++)
            if (dists2[vNo]>maxDepthDiff2)
                maxDepthDiff2=dists2[vNo];
        if (ids[seq[i].first][seq[i].second]>0&&ids[seq[i+1].first][seq[i+1].second]>0&&maxDepthDiff2<maxDepthDistThreshold)
            drawTriangle(vertices,idsVec, r,g,b);
    }
}

/// compute mean depth using neighbouring elements in the word
double QGLVisualizer::computeMeanDepth(const ViewDependentPart&part, int u, int v, Vec6& meanPosNorm) const{
    double meanDepth=0;
    int elementsNo=0;
    Vec3 meanNormal(0,0,0);
    for (int i=-1;i<2;i++){
        for (int j=-1;j<2;j++){
            int coords[2]={u+i,v+j};
            if (coords[0]>=0&&coords[0]<3&&coords[1]>=0&&coords[1]<3){
                if (part.partIds[coords[0]][coords[1]]>0){
                    if (coords[0]==1&&coords[1]==1){
                        //meanDepth+=0;
                        meanNormal+=part.partsPosNorm[coords[0]][coords[1]].mean.block<3,1>(3,0);
                    }
                    else{
                        //meanDepth+=part.partsPosNorm[coords[0]][coords[1]].mean(2);
                        meanNormal+=part.partsPosNorm[coords[0]][coords[1]].mean.block<3,1>(3,0);
                    }
                    elementsNo++;
                }
            }
        }
    }
    if (elementsNo>0){
        //meanDepth/=double(elementsNo);
        meanNormal/=double(elementsNo);
    }
    Mat33 rot = NormalImageFilter::coordinateFromNormal(meanNormal);
    Vec3 posRot(part.partsPosNorm[u][v].mean.block<3,1>(0,0));
    posRot(2)=0;
    posRot=rot*posRot;
    meanPosNorm.block<3,1>(0,0) = posRot.block<3,1>(0,0);
    meanPosNorm.block<3,1>(3,0) = meanNormal.block<3,1>(0,0);
    meanDepth = posRot(2);
    return meanDepth;
}

/// Create point cloud List
GLuint QGLVisualizer::createPartList(const ViewDependentPart& part, int layerNo){
    // create one display list
    GLuint index = glGenLists(1);
    glNewList(index, GL_COMPILE);
    if (layerNo==1){
        glColor3d(0.5,0.5,0.5);
        if (config.surfaceType==1)
            drawPartMesh(part,0.5,0.5,0.5);
        else if (config.surfaceType==0)
            drawPart(part, layerNo,0.5,0.5,0.5);
    }
    else if (layerNo==2){
        for (size_t n = 0; n < part.partIds.size(); n++){
            for (size_t m = 0; m < part.partIds[n].size(); m++){
                int id = part.partIds[n][m];
                Vec3 posPart(part.partsPosNorm[n][m].mean.block<3,1>(0,0));
                if (n==1&&m==1)
                    posPart=Vec3(0,0,0);
                double GLmat1[16]={1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, posPart(0), posPart(1), posPart(2), 1};
                glPushMatrix();
                    glMultMatrixd(GLmat1);
                    if (id<0){
                        //glColor3ub(100,50,50);
                        glCallList(backgroundList[layerNo-1]);
                    }
                    else{
                        glColor3d(0.5,0.5,0.5);
                        Mat34 offset = part.offsets[n][m];
                        double GLmat2[16]={offset(0,0), offset(1,0), offset(2,0), 0, offset(0,1), offset(1,1), offset(2,1), 0, offset(0,2), offset(1,2), offset(2,2), 0, offset(0,3), offset(1,3), offset(2,3), 1};
                        glMultMatrixd(GLmat2);
                        glPushMatrix();
                            drawPart(hierarchy.get()->viewDependentLayers[0][id], layerNo-1, 0.5,0.5,0.5);
                        glPopMatrix();
                    }
                glPopMatrix();
            }
        }
    }
    glEndList();
    return index;
}

/// Create point cloud List
GLuint QGLVisualizer::createPartObjList(const std::vector<hop3d::PointCloudRGBA>& objects){
    GLuint index = glGenLists(1);
    glNewList(index, GL_COMPILE);
    glPushMatrix();
    int objectsNo = (int)objects.size();
    int objectNo=0;
    for (auto& cloud : objects){
        if (cloud.size()>0){
            double GLmat[16]={1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, (double)(config.partObjectsPos[0]*double(objectNo)-(objectsNo/2)*config.partObjectsPos[0])-cloud.front().position(0), config.partObjectsPos[1]-cloud.front().position(1), config.partObjectsPos[2]-cloud.front().position(2), 1};
            glPushMatrix();
            glMultMatrixd(GLmat);
            glBegin(GL_POINTS);
            for (auto& point: cloud){
                glPointSize((float)config.cloudPointSize);
                glColor3f((float)point.color[0], (float)point.color[1], (float)point.color[2]);
                //glNormal3d(-normal(0), -normal(1), -normal(2));
                glVertex3d(point.position(0), point.position(1), point.position(2));
            }
            glEnd();
            glPopMatrix();
            objectNo++;
        }
    }
    glPopMatrix();
    glEndList();
    return index;
}

/// Create point cloud List from parts (planar patches)
GLuint QGLVisualizer::createObjList(const std::vector<Part3D>& parts, int layerNo, bool inference){
    // create one display list
    GLuint index = glGenLists(1);
    glNewList(index, GL_COMPILE);
    glPushMatrix();
        Vec3 initPose(parts.begin()->pose(0,3), parts.begin()->pose(1,3), parts.begin()->pose(2,3));
        if (inference)
            initPose(1,3)+=config.objectsOffsetY;
        for (auto & part : parts){
            double GLmat[16]={part.pose(0,0), part.pose(1,0), part.pose(2,0), 0, part.pose(0,1), part.pose(1,1), part.pose(2,1), 0, part.pose(0,2), part.pose(1,2), part.pose(2,2), 0, part.pose(0,3)-initPose(0), part.pose(1,3)-initPose(1), part.pose(2,3)-initPose(2), 1};
            glPushMatrix();
                glMultMatrixd(GLmat);
                //std::cout << "layerNo " << layerNo << " part id " << part.id << "\n";
                //std::cout << "cloudsListLayers[layerNo]  " << cloudsListLayers[layerNo].size() << "\n";
                glCallList(cloudsListLayers[layerNo][part.id]);
            glPopMatrix();
        }
    glPopMatrix();
    glEndList();
    return index;
}

/// Create point cloud List
GLuint QGLVisualizer::createObjList(const std::vector<ViewIndependentPart>& parts, int layerNo, bool inference){
    // create one display list
    GLuint index = glGenLists(1);
    glNewList(index, GL_COMPILE);
    glPushMatrix();
    if (parts.size()>0){
        Vec3 initPose(parts.begin()->pose(0,3), parts.begin()->pose(1,3), parts.begin()->pose(2,3));
        if (inference)
            initPose(1,3)+=config.objectsOffsetY;
        for (auto & part : parts){
            Mat34 pose = part.pose*part.offset;
            double GLmat[16]={pose(0,0), pose(1,0), pose(2,0), 0, pose(0,1), pose(1,1), pose(2,1), 0, pose(0,2), pose(1,2), pose(2,2), 0, pose(0,3)-initPose(0), pose(1,3)-initPose(1), pose(2,3)-initPose(2), 1};
            glPushMatrix();
                glMultMatrixd(GLmat);
                glColor3ub(200,200,200);
                glCallList(cloudsListLayers[layerNo-1][part.id]);
            glPopMatrix();
        }
    }
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
    //int id = part.group.begin()->id;
    Mat34 pose = Mat34::Identity();//part.pose.inverse();
    int componentNo=0;
    for (auto & component : part.group){
        for (auto& patch : component.cloud){
            glPushMatrix();
            double GLmat[16]={pose(0,0), pose(1,0), pose(2,0), 0, pose(0,1), pose(1,1), pose(2,1), 0, pose(0,2), pose(1,2), pose(2,2), 0, patch.position(0), patch.position(1)+(double)(config.partDist[layerNo-1]*double(componentNo+1)), patch.position(2), 1};
            glMultMatrixd(GLmat);
            glColor3ub(200,200,200);
            drawPatch(patch.normal);
            glPopMatrix();
        }
        double GLmat2[16]={component.offset(0,0), component.offset(1,0), component.offset(2,0), 0, component.offset(0,1), component.offset(1,1), component.offset(2,1), 0, component.offset(0,2), component.offset(1,2), component.offset(2,2), 0, component.offset(0,3),component.offset(1,3)+(double)(config.partDist[layerNo-1]*double(componentNo+1)),component.offset(2,3), 1};
        //double GLmat1[16]={1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, posPart(0), posPart(1)+(double)(config.partDist[layerNo]*double(componentNo+1)), posPart(2), 1};
        glPushMatrix();
            glMultMatrixd(GLmat2);
            for (auto& patch : part.cloud){
                glPushMatrix();
                double GLmat[16]={pose(0,0), pose(1,0), pose(2,0), 0, pose(0,1), pose(1,1), pose(2,1), 0, pose(0,2), pose(1,2), pose(2,2), 0, patch.position(0), patch.position(1), patch.position(2), 1};
                glMultMatrixd(GLmat);
                glColor3ub(200,0,0);
                drawPatch(patch.normal);
                glPopMatrix();
            }
        glPopMatrix();
        componentNo++;
    }
    // create one display list
    /*GLuint index = glGenLists(1);
    glNewList(index, GL_COMPILE);
    glPushMatrix();
    int componentNo=0;
    if (layerNo==4){
        for (auto & partId : part.group){
            Mat34 pose = partId.pose.inverse();
            double GLmat[16]={pose(0,0), pose(1,0), pose(2,0), 0, pose(0,1), pose(1,1), pose(2,1), 0, pose(0,2), pose(1,2), pose(2,2), 0, 0, (double)(config.partDist[layerNo-1]*double(componentNo+1)), 0, 1};
            glPushMatrix();
                glMultMatrixd(GLmat);
                if (partId.id<0){
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
                            if (id<0){
                                pos(0)=config.voxelSize*double(double(m)-1.0);
                                pos(1)=config.voxelSize*double(double(n)-1.0);
                                pos(2)=config.voxelSize*double(double(l)-1.0);
                            }
                            double GLmatrot[16]={partPose(0,0), partPose(1,0), partPose(2,0), 0,
                                              partPose(0,1), partPose(1,1), partPose(2,1), 0,
                                              partPose(0,2), partPose(1,2), partPose(2,2), 0,
                                              partPose(0,3), partPose(1,3)+(double)(config.partDist[layerNo-1]*double(componentNo+1)), partPose(2,3), 1};
                            if (id<0){
                                GLmatrot[0]=1; GLmatrot[1]=0; GLmatrot[2]=0;
                                GLmatrot[4]=0; GLmatrot[5]=1; GLmatrot[6]=0;
                                GLmatrot[8]=0; GLmatrot[9]=0; GLmatrot[10]=1;
                                GLmatrot[12]=0; GLmatrot[13]=0; GLmatrot[14]=0;
                            }
                            glPushMatrix();
                                glMultMatrixd(GLmatrot);
                                if (id<0){

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
    }*/
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
        Mat34 estTrans;
        if (layerNo==1)
            ViewDependentPart::distanceInvariant(part,*itComp,3,estTrans);
        else if (layerNo==2)
            ViewDependentPart::distanceInvariant(part,*itComp,3, hierarchy.get()->viewDependentLayers[0],estTrans);
        double GLmat1[16]={1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, (double)(config.partDist[layerNo]*double(componentNo+1)), 0, 1};
        glPushMatrix();
            glMultMatrixd(GLmat1);
            if (layerNo==1){
                drawPart(*itComp, layerNo,0.5,0.5,0.5);
            }
            else if (layerNo==2){
                for (size_t n = 0; n < itComp->partIds.size(); n++){
                    for (size_t m = 0; m < itComp->partIds[n].size(); m++){
                        int id = itComp->partIds[n][m];
                        Vec3 posPart(itComp->partsPosNorm[n][m].mean.block<3,1>(0,0));
                        if (n==1&&m==1)
                            posPart=Vec3(0,0,0);
                        double GLmat1[16]={1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, posPart(0), posPart(1), posPart(2), 1};
                        glPushMatrix();
                            glMultMatrixd(GLmat1);
                            if (id<0){
                                //glColor3ub(100,50,50);
                                glCallList(backgroundList[layerNo-1]);
                            }
                            else{
                                if (layerNo==1){
                                    glColor3d(0.5,0.5,0.5);
                                    drawPatch(Vec3(itComp->partsPosNorm[n][m].mean.block<3,1>(3,0)));
                                }
                                else{
                                    glColor3d(0.5,0.5,0.5);
                                    Mat34 offset = itComp->offsets[n][m];
                                    double GLmat2[16]={offset(0,0), offset(1,0), offset(2,0), 0, offset(0,1), offset(1,1), offset(2,1), 0, offset(0,2), offset(1,2), offset(2,2), 0, offset(0,3), offset(1,3), offset(2,3), 1};
                                    glMultMatrixd(GLmat2);
                                    glPushMatrix();
                                        drawPart(hierarchy.get()->viewDependentLayers[0][id], layerNo-1, 0.5,0.5,0.5);
                                    glPopMatrix();
                                }
                            }
                        glPopMatrix();
                    }
                }
            }
        glPopMatrix();

        /*for (size_t n = 0; n < itComp->partIds.size(); n++){
            for (size_t m = 0; m < itComp->partIds[n].size(); m++){
                int id = itComp->partIds[n][m];
                Vec3 posPart(itComp->partsPosNorm[n][m].mean.block<3,1>(0,0));
                if (n==1&&m==1)
                    posPart=Vec3(0,0,0);
                //double GLmat1[16]={estTrans(0,0), estTrans(1,0), estTrans(2,0), 0, estTrans(0,1), estTrans(1,1), estTrans(2,1), 0, estTrans(0,2), estTrans(1,2), estTrans(2,2), 0, posPart(0), posPart(1)+(double)(config.partDist[layerNo]*double(componentNo+1)), posPart(2), 1};
                double GLmat1[16]={1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, posPart(0), posPart(1)+(double)(config.partDist[layerNo]*double(componentNo+1)), posPart(2), 1};
                glPushMatrix();
                    glMultMatrixd(GLmat1);
                    if (id<0){
                        //glColor3ub(100,50,50);
                        glCallList(backgroundList[layerNo-1]);
                    }
                    else{
                        glColor3d(0.5,0.5,0.5);
                        if (layerNo==1){
                            drawPatch(itComp->partsPosNorm[n][m].mean.block<3,1>(3,0));
                        }
                        else{
                            Mat34 offset = part.offsets[n][m];
                            double GLmat2[16]={offset(0,0), offset(1,0), offset(2,0), 0, offset(0,1), offset(1,1), offset(2,1), 0, offset(0,2), offset(1,2), offset(2,2), 0, offset(0,3), offset(1,3), offset(2,3), 1};
                            glMultMatrixd(GLmat2);
                            glPushMatrix();
                                drawPart(hierarchy.get()->viewDependentLayers[0][id], layerNo-1, 0.5,0.5,0.5);
                            glPopMatrix();
                        }
                    }
                glPopMatrix();
            }
        }*/
        double GLmat2[16]={estTrans(0,0), estTrans(1,0), estTrans(2,0), 0, estTrans(0,1), estTrans(1,1), estTrans(2,1), 0, estTrans(0,2), estTrans(1,2), estTrans(2,2), 0, estTrans(0,3),estTrans(1,3)+(double)(config.partDist[layerNo]*double(componentNo+1)),estTrans(2,3), 1};
        //double GLmat1[16]={1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, posPart(0), posPart(1)+(double)(config.partDist[layerNo]*double(componentNo+1)), posPart(2), 1};
        glPushMatrix();
            glMultMatrixd(GLmat2);
            if (layerNo==1){
                drawPart(part, layerNo,0.5,0,0);
            }
            else if (layerNo==2){
                for (size_t n = 0; n < part.partIds.size(); n++){
                    for (size_t m = 0; m < part.partIds[n].size(); m++){
                        int id = part.partIds[n][m];
                        Vec3 posPart(part.partsPosNorm[n][m].mean.block<3,1>(0,0));
                        if (n==1&&m==1)
                            posPart=Vec3(0,0,0);
                        double GLmat1[16]={1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, posPart(0), posPart(1), posPart(2), 1};
                        glPushMatrix();
                            glMultMatrixd(GLmat1);
                            if (id<0){
                                //glColor3ub(100,50,50);
                                glCallList(backgroundList[layerNo-1]);
                            }
                            else{
                                if (layerNo==1){
                                    glColor3d(0.5,0.5,0.5);
                                    drawPatch(Vec3(part.partsPosNorm[n][m].mean.block<3,1>(3,0)));
                                }
                                else{
                                    glColor3d(0.5,0.0,0.0);
                                    Mat34 offset = part.offsets[n][m];
                                    double GLmat2[16]={offset(0,0), offset(1,0), offset(2,0), 0, offset(0,1), offset(1,1), offset(2,1), 0, offset(0,2), offset(1,2), offset(2,2), 0, offset(0,3), offset(1,3), offset(2,3), 1};
                                    glMultMatrixd(GLmat2);
                                    glPushMatrix();
                                        drawPart(hierarchy.get()->viewDependentLayers[0][id], layerNo-1, 0.5,0,0);
                                    glPopMatrix();
                                }
                            }
                        glPopMatrix();
                    }
                }
            }
        glPopMatrix();
        componentNo++;
    }
    glPopMatrix();
    glEndList();
    return index;
}

/// draw flat patch
void QGLVisualizer::drawPatch(const Vec3& normal) const{
    if (config.drawSurfaces){
        Mat33 rot = NormalImageFilter::coordinateFromNormal(normal);
        glBegin(GL_POLYGON);
        int quadSize=5;
        Vec3 rightup(quadSize*config.pixelSize,-quadSize*config.pixelSize,-0.0001);
        rightup=rot*rightup;
        if (config.useNormalSurf) glNormal3d(-normal(0), -normal(1), -normal(2));
        glVertex3d(rightup(0), rightup(1),rightup(2));
        Vec3 rightdown(quadSize*config.pixelSize,quadSize*config.pixelSize,-0.0001);
        rightdown=rot*rightdown;
        if (config.useNormalSurf) glNormal3d(-normal(0), -normal(1), -normal(2));
        glVertex3d(rightdown(0), rightdown(1),rightdown(2));
        Vec3 leftdown(-quadSize*config.pixelSize,quadSize*config.pixelSize,-0.0001);
        leftdown=rot*leftdown;
        if (config.useNormalSurf) glNormal3d(-normal(0), -normal(1), -normal(2));
        glVertex3d(leftdown(0), leftdown(1), leftdown(2));
        Vec3 leftup(-quadSize*config.pixelSize,-quadSize*config.pixelSize,-0.0001);
        leftup=rot*leftup;
        if (config.useNormalSurf) glNormal3d(-normal(0), -normal(1), -normal(2));
        glVertex3d(leftup(0), leftup(1), leftup(2));
        glEnd();
        glBegin(GL_POLYGON);
        Vec3 rightupA(quadSize*config.pixelSize,-quadSize*config.pixelSize,0.0001);
        rightupA=rot*rightupA;
        if (config.useNormalSurf) glNormal3d(normal(0), normal(1), normal(2));
        glVertex3d(rightupA(0), rightupA(1),rightupA(2));
        Vec3 leftupA(-quadSize*config.pixelSize,-quadSize*config.pixelSize,0.0001);
        leftupA=rot*leftupA;
        if (config.useNormalSurf) glNormal3d(normal(0), normal(1), normal(2));
        glVertex3d(leftupA(0), leftupA(1), leftupA(2));
        Vec3 leftdownA(-quadSize*config.pixelSize,quadSize*config.pixelSize,0.0001);
        leftdownA=rot*leftdownA;
        if (config.useNormalSurf) glNormal3d(normal(0), normal(1), normal(2));
        glVertex3d(leftdownA(0), leftdownA(1), leftdownA(2));
        Vec3 rightdownA(quadSize*config.pixelSize,quadSize*config.pixelSize,0.0001);
        rightdownA=rot*rightdownA;
        if (config.useNormalSurf) glNormal3d(normal(0), normal(1), normal(2));
        glVertex3d(rightdownA(0), rightdownA(1),rightdownA(2));
        glEnd();
    }
    if (config.drawPoints){
        Mat33 rot = NormalImageFilter::coordinateFromNormal(normal);
        glBegin(GL_POINTS);
        int quadSize=10;
        for (int i=0;i<10;i++){
            for (int j=0;j<10;j++){
                Mat34 trans(Mat34::Identity()), point(Mat34::Identity());
                trans.matrix().block<3,3>(0,0) = rot;
                point(0,3) = (i-5)*((quadSize*config.pixelSize)/10);
                point(1,3) = (j-5)*((quadSize*config.pixelSize)/10);
                point = trans*point;
                if (sqrt(pow(point(0,3),2.0)+pow(point(1,3),2.0))<((quadSize-1)*config.pixelSize/2.0)){
                    glNormal3d(-point(0,2), -point(1,2), -point(2,2));
                    glVertex3d(point(0,3), point(1,3), point(2,3));
                }

            }
        }
        glEnd();
    }
    if (config.drawNormals){
        glBegin(GL_LINES);
            //glColor3d(config.normalsColor.redF(),config.normalsColor.greenF(),config.normalsColor.blueF());
            glVertex3d(0.0, 0.0, 0.0);
            glVertex3d(-normal(0)*config.normalsScale, -normal(1)*config.normalsScale, -normal(2)*config.normalsScale);
        glEnd();
    }
}

/// draw flat patch
void QGLVisualizer::drawTriangle(const std::vector<Vec6>& vertices, const std::vector<int>& ids, double r, double g, double b) const{
    glBegin(GL_TRIANGLES);
    double alpha;
    for (int i=0;i<3;i++){
        alpha = (ids[i]<0) ? 0.0 : 1.0;
        glColor4d(r,g,b,alpha);
        if (config.useNormalSurf) glNormal3d(-vertices[i](3), -vertices[i](4), -vertices[i](5));
        glVertex3d( vertices[i](0),vertices[i](1),vertices[i](2));
    }
    glEnd();
    glBegin(GL_TRIANGLES);
    for (int i=2;i>=0;i--){
        alpha = (ids[i]<0) ? 0.0 : 1.0;
        glColor4d(r,g,b,alpha);
        if (config.useNormalSurf) glNormal3d(vertices[i](3), vertices[i](4), vertices[i](5));
        glVertex3d( vertices[i](0),vertices[i](1),vertices[i](2)+0.0001);
    }
    glEnd();
}

/// draw flat patch
void QGLVisualizer::drawNormals(const std::vector<Vec6>& vertices, const std::vector<int>& ids){
    for (size_t i=0;i<vertices.size();i++){
        glBegin(GL_LINES);
            glColor3d(config.normalsColor.redF(),config.normalsColor.greenF(),config.normalsColor.blueF());
            if (ids[i]>0){
                glVertex3d(vertices[i](0), vertices[i](1), vertices[i](2));
                glVertex3d(vertices[i](0)-vertices[i](3)*config.normalsScale, vertices[i](1)-vertices[i](4)*config.normalsScale, vertices[i](2)-vertices[i](5)*config.normalsScale);
            }
        glEnd();
    }
}

/// Create point cloud List
GLuint QGLVisualizer::createCloudList(hop3d::PointCloud& pointCloud, Vec3& normal){
    // create one display list
    GLuint index = glGenLists(1);
    // compile the display list, store a triangle in it
    glNewList(index, GL_COMPILE);
        glColor3ub(200,200,200);
        if (config.drawPointClouds){
            glBegin(GL_POINTS);
            for (size_t n = 0; n < pointCloud.size(); n++){
                //glColor3ub(pointCloud.second.pointCloudNormal[n].r, pointCloud.second.pointCloudNormal[n].g, pointCloud.second.pointCloudNormal[n].b);
                if (config.useNormalCloud)
                    glNormal3d(-normal(0), -normal(1), -normal(2));
                glVertex3d(pointCloud[n].position(0),
                          pointCloud[n].position(1),
                          pointCloud[n].position(2));
            }
            glEnd();
        }
        drawPatch(normal);
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
                            if (it->partIds[n][m][l]>=0)
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

/// draw Part Second Layer
void QGLVisualizer::drawPartSecondLayer(void) const{
    for (auto &index : secondLayerPartList)
        glCallList(index);
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
    if (config.drawPartObjects)
        drawPartObjects();
    drawPointClouds();
    drawCoords();
    //drawPartSecondLayer();
    updateHierarchy();
    update3Dobjects();
    updatePartsObjects();
    if (updateCoordinates)
        createCoordsList();
    //updateSecondLayerPart();
}

/// draw objects
void QGLVisualizer::animate(){
}

/// initialize visualizer
void QGLVisualizer::init(){
    // Light setup
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 50.0);
    GLfloat specular_color[4] = { 0.8f, 0.8f, 0.8f, 1.0 };
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR,  specular_color);

    GLfloat ambientLight0[] = {0.2f, 0.2f, 0.2f, 0.0f};
    GLfloat diffuseLight0[] = {0.6f, 0.6f, 0.6f, 0.0f};
    GLfloat specularLight0[] = {1.0f, 1.0f, 1.0f, 1.0f};

    GLfloat positionLight0[] = {0,0,0, 1.0f};
    GLfloat directionLight0[] = {0,0,1};

    GLfloat lm_ambient[] = {(float)0.3, (float)0.3, (float)0.3, (float)1.0};

    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, lm_ambient);
    glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);

    glLightfv(GL_LIGHT0, GL_AMBIENT, ambientLight0);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, diffuseLight0);
    glLightfv(GL_LIGHT0, GL_SPECULAR, specularLight0);

    glLightfv(GL_LIGHT0, GL_POSITION, positionLight0);
    glLightfv(GL_LIGHT0, GL_SPOT_DIRECTION, directionLight0);

    glEnable (GL_LIGHTING);
    glEnable (GL_LIGHT0);
//    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, GL_TRUE);

    glEnable(GL_AUTO_NORMAL);
    glEnable(GL_NORMALIZE);
    // Restore previous viewer state.
    restoreStateFromFile();

    camera()->setZNearCoefficient((float)0.00001);
    camera()->setZClippingCoefficient(100.0);

    //qglviewer::Quaternion q(-0.5,0.5,0.5,0.5);
    //camera()->setOrientation(q);

    setBackgroundColor(config.backgroundColor);

    glEnable(GL_LINE_SMOOTH);
    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Opens help window
    help();

    // Add custom key description (see keyPressEvent).
    setKeyDescription(Qt::Key_1, "Display parts from layer 1");
    setKeyDescription(Qt::Key_2, "Display parts from layer 2");
    setKeyDescription(Qt::Key_3, "Display parts from layer 3");
    setKeyDescription(Qt::Key_4, "Display parts from layer 4");
    setKeyDescription(Qt::Key_5, "Display parts from layer 5");
    setKeyDescription(Qt::Key_6, "Display parts from layer 6");

    startAnimation();
}

/// keyboard events
void QGLVisualizer::keyPressEvent(QKeyEvent *e) {
  // Get event modifiers key
  const Qt::KeyboardModifiers modifiers = e->modifiers();

  // A simple switch on e->key() is not sufficient if we want to take state key into account.
  // With a switch, it would have been impossible to separate 'F' from 'CTRL+F'.
  // That's why we use imbricated if...else and a "handled" boolean.
  bool handled = false;
  if ((e->key()==Qt::Key_1) && (modifiers==Qt::NoButton)){
      activeLayer=0;
      handled = true;
  }
  else if ((e->key()==Qt::Key_2) && (modifiers==Qt::NoButton)){
      activeLayer=1;
      handled = true;
  }
  else if ((e->key()==Qt::Key_3) && (modifiers==Qt::NoButton)){
      activeLayer=2;
      handled = true;
  }
  else if ((e->key()==Qt::Key_4) && (modifiers==Qt::NoButton)){
      activeLayer=3;
      handled = true;
  }
  else if ((e->key()==Qt::Key_5) && (modifiers==Qt::NoButton)){
      activeLayer=4;
      handled = true;
  }
  else if ((e->key()==Qt::Key_6) && (modifiers==Qt::NoButton)){
      activeLayer=5;
      handled = true;
  }
  else if ((e->key()==Qt::Key_Q) && (modifiers==Qt::NoButton)){
      activeOverlapNo=0;
      handled = true;
  }
  else if ((e->key()==Qt::Key_W) && (modifiers==Qt::NoButton)){
      activeOverlapNo=1;
      handled = true;
  }
  else if ((e->key()==Qt::Key_E) && (modifiers==Qt::NoButton)){
      activeOverlapNo=2;
      handled = true;
  }
  else if ((e->key()==Qt::Key_R) && (modifiers==Qt::NoButton)){
      activeOverlapNo=3;
      handled = true;
  }
  // ... and so on with other else/if blocks.

  if (!handled)
    QGLViewer::keyPressEvent(e);
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

