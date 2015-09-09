#include "Utilities/Writer.h"


int hop3d::Writer::writePlyToFile(std::string fileName, const hop3d::PointCloud& inputPointCloud, const std::vector<Eigen::Vector4i> &inputFaces){
    std::ofstream myfile;
    myfile.open(fileName);
    myfile << "ply\r\n";
    myfile << "format ascii 1.0\r\n";
    myfile << "comment file created with HOP3D\r\n";
    myfile << "element vertex " << inputPointCloud.pointCloudNormal.size() << "\r\n";
    myfile << "property double x\r\n";
    myfile << "property double y\r\n";
    myfile << "property double z\r\n";
    myfile << "property double nx\r\n";
    myfile << "property double ny\r\n";
    myfile << "property double nz\r\n";
    myfile << "element face " << inputFaces.size() << "\r\n";
    myfile << "property list uchar uint vertex_indices\r\n";
    myfile << "end_header\r\n";

    for(unsigned long  i=0; i < inputPointCloud.pointCloudNormal.size(); i++ ){
        myfile << std::fixed << std::setprecision(6) << inputPointCloud.pointCloudNormal[i].position(0) <<" "<< inputPointCloud.pointCloudNormal[i].position(1) <<" " << inputPointCloud.pointCloudNormal[i].position(2) << " " <<
                  inputPointCloud.pointCloudNormal[i].normal(0) <<" "<< inputPointCloud.pointCloudNormal[i].normal(1) <<" " << inputPointCloud.pointCloudNormal[i].normal(2) << "\r\n";
    }
    for(unsigned long  i=0; i < inputFaces.size(); i++ ){
        myfile << "4" <<" "<< inputFaces[i](0) <<" "<< inputFaces[i](1) <<" " << inputFaces[i](2)<<" "<< inputFaces[i](3) <<"\r\n";
    }
    myfile.close();
    return 0;

}
