/** @file depthSensorModel.h
 *
 * implementation - Kinect/Xtion model
 *
 */

#ifndef DEPTH_SENSOR_MODEL_H_INCLUDED
#define DEPTH_SENSOR_MODEL_H_INCLUDED

#include "Data/Defs.h"
#include "../external/tinyXML/tinyxml2.h"
#include <iostream>
#include <memory>

class DepthSensorModel {
  public:
    /// Construction
    DepthSensorModel() {};

    /// Construction
    DepthSensorModel(std::string configFile);

    /// compute 3D point from image coordinates
    void getPoint(int u, int v, double depth, Eigen::Vector3d& point3D) const;

    /// compute 3D point from image coordinates
    void getPoint(const Eigen::Vector3d& depthImageCoord, Eigen::Vector3d& point3D) const;

    /// inverse model of the sensor
    Eigen::Vector3d inverseModel(double x, double y, double z) const;

    /// u,v [px], depth [m]
    void computeCov(int u, int v, double depth, hop3d::Mat33& cov);

    /// point xyz in camera frame
    void computeCov(hop3d::Vec3 point, hop3d::Mat33& cov);

    /// compute information matrix
    hop3d::Mat33 informationMatrix(double x, double y, double z);

    /// compute information matrix using image coordinates
    hop3d::Mat33 informationMatrixFromImageCoordinates(double u, double v, double z);

    /// normalize vector
    static void normalizeVector(hop3d::Vec3& normal);

    class Config{
      public:
        Config() :
            focalLength{582.64, 586.97},
            focalAxis{320.17, 260.0},
            varU(1.1046), varV(0.64160),
            distVarCoefs{-8.9997e-06, 3.069e-003, 3.6512e-006, -0.0017512e-3}{
        }
        Config(std::string configFilename){
            tinyxml2::XMLDocument config;
            std::string filename = "../../resources/" + configFilename;
            std::cout<<"CONFIG FILENAME: " << configFilename << std::endl;
            config.LoadFile(filename.c_str());
            if (config.ErrorID())
                std::cout << "unable to load sensor config file: error = " << config.ErrorID() << std::endl;;

            tinyxml2::XMLElement * model = config.FirstChildElement( "Model" );
            model->FirstChildElement( "focalLength" )->QueryDoubleAttribute("fu", &focalLength[0]);
            model->FirstChildElement( "focalLength" )->QueryDoubleAttribute("fv", &focalLength[1]);
            model->FirstChildElement( "focalAxis" )->QueryDoubleAttribute("Cu", &focalAxis[0]);
            model->FirstChildElement( "focalAxis" )->QueryDoubleAttribute("Cv", &focalAxis[1]);
            model->FirstChildElement( "variance" )->QueryDoubleAttribute("sigmaU", &varU);
            model->FirstChildElement( "variance" )->QueryDoubleAttribute("sigmaV", &varV);
            model->FirstChildElement( "varianceDepth" )->QueryDoubleAttribute("c3", &distVarCoefs[0]);
            model->FirstChildElement( "varianceDepth" )->QueryDoubleAttribute("c2", &distVarCoefs[1]);
            model->FirstChildElement( "imageSize" )->QueryIntAttribute("sizeU", &imageSize[0]);
            model->FirstChildElement( "imageSize" )->QueryIntAttribute("sizeV", &imageSize[1]);
            model->FirstChildElement( "alternateModel" )->QueryDoubleAttribute("scaleUncertaintyNormal", &scaleUncertaintyNormal);
            model->FirstChildElement( "alternateModel" )->QueryDoubleAttribute("scaleUncertaintyGradient", &scaleUncertaintyGradient);
            tinyxml2::XMLElement * posXML = config.FirstChildElement( "pose" );
            double query[4];
            posXML->QueryDoubleAttribute("qw", &query[0]); posXML->QueryDoubleAttribute("qx", &query[1]); posXML->QueryDoubleAttribute("qy", &query[2]); posXML->QueryDoubleAttribute("qz", &query[3]);
            double queryPos[4];
            posXML->QueryDoubleAttribute("x", &queryPos[0]); posXML->QueryDoubleAttribute("y", &queryPos[1]); posXML->QueryDoubleAttribute("z", &queryPos[2]);
            //pose = hop3d::Quaternion (query[0], query[1], query[2], query[3])*hop3d::Vec3(queryPos[0], queryPos[1], queryPos[2]);
        }
        public:
            double focalLength[2];
            double focalAxis[2];
            double varU, varV;// variance u,v
            double distVarCoefs[4];
            int imageSize[2];//[sizeU, sizeV]
            hop3d::Mat34 pose; // kinect pose in robot's coordination frame
            double scaleUncertaintyNormal;
            double scaleUncertaintyGradient;
    };

    Config config;

    protected:
        hop3d::Mat33 PHCPModel;//pin-hole camera projection model
        hop3d::Mat33 Ruvd; //covariance matrix for [u,v,disp]
};

#endif // DEPTH_SENSOR_MODEL_H_INCLUDED
