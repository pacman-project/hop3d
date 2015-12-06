#include "Utilities/depthSensorModel.h"

/// Construction
DepthSensorModel::DepthSensorModel(std::string configFile) : config(configFile){
    PHCPModel << 1/config.focalLength[0],0,-config.focalAxis[0]/config.focalLength[0],
                  0,1/config.focalLength[1], -config.focalAxis[1]/config.focalLength[1],
                  0,0,1;
    Ruvd << config.varU, 0, 0,
            0, config.varV, 0,
            0, 0, 0;
}

/// compute 3D point from image coordinates
void DepthSensorModel::getPoint(int u, int v, double depth, Eigen::Vector3d& point3D) const{
    if (depth>6.0||depth<0.4)//out of range
        point3D=Eigen::Vector3d(NAN,NAN,NAN);
    else {
        Eigen::Vector3d point(u, v, 1);
        point3D = depth*PHCPModel*point;
    }
}

/// compute 3D point from image coordinates
void DepthSensorModel::getPoint(const Eigen::Vector3d& depthImageCoord, Eigen::Vector3d& point3D) const{
    Eigen::Vector3d point(depthImageCoord(0), depthImageCoord(1), 1);
    point3D = depthImageCoord(2)*PHCPModel*point;
}

Eigen::Vector3d DepthSensorModel::inverseModel(double x, double y, double z) const {
    Eigen::Vector3d point(((config.focalLength[0]*x)/z)+config.focalAxis[0], ((config.focalLength[1]*y)/z)+config.focalAxis[1], z);
    if (point(0)<0||point(0)>config.imageSize[0]||point(1)<0||point(1)>config.imageSize[1]||z<0.4||z>6.0){
        point(0) = NAN; point(1) = NAN; point(2) = NAN;
    }
    return point;
}

/// u,v [px], depth [m]
void DepthSensorModel::computeCov(int u, int v, double depth, hop3d::Mat33& cov) {
    //double dispDer = config.k3 * 1/(config.k2*pow(cos((disparity/config.k2) + config.k1),2.0));
    hop3d::Mat33 J;
    J << depth/config.focalLength[0], 0, ((u/config.focalLength[0])-(config.focalAxis[0]/config.focalLength[0])),
         0, depth/config.focalLength[1], ((v/config.focalLength[1])-(config.focalAxis[1]/config.focalLength[1])),
         0, 0, 1;
    Ruvd(2,2) = config.distVarCoefs[0]*pow(depth,3.0) + config.distVarCoefs[1]*pow(depth,2.0) + config.distVarCoefs[2]*depth + config.distVarCoefs[3];
    cov=J*Ruvd*J.transpose();
}

/// point xyz in camera frame
void DepthSensorModel::computeCov(hop3d::Vec3 point, hop3d::Mat33& cov){
    hop3d::Vec3 imageCoordinates = inverseModel(point.x(), point.y(), point.z());
    if (imageCoordinates.x()==-1)
        cov.setZero();
    else
        computeCov((int)imageCoordinates.x(), (int)imageCoordinates.y(), (int)imageCoordinates.z(),cov);
}

/// compute information matrix
hop3d::Mat33 DepthSensorModel::informationMatrix(double x, double y, double z){
    hop3d::Mat33 info;
	Eigen::Vector3d cam3D = inverseModel(x, y, z);
    computeCov((int)cam3D(0), (int)cam3D(1), (int)cam3D(2), info);
    return info.inverse();
}

hop3d::Mat33 DepthSensorModel::informationMatrixFromImageCoordinates(double u, double v, double z) {
    hop3d::Mat33 cov;
    computeCov((int)u, (int)v, z, cov);
    return cov.inverse();
}

/// normalize vector
/*void DepthSensorModel::normalizeVector(hop3d::Vec3& normal){
    double norm = normal.norm();
    normal.x() /= norm;    normal.y() /= norm;    normal.z() /= norm;
}*/
