#include "Utilities/Writer.h"


int Writer::writePlyToFile(std::string fileName, const PointCloud& inputPointCloud, const std::vector<Eigen::Vector4i> &inputFaces){
    std::ofstream myfile;
    myfile.open(fileName);
    myfile << "ply\r\n";
    myfile << "format ascii 1.0\r\n";
    myfile << "comment file created with HOP3D\r\n";
    myfile << "element vertex " << inputPointCloud.PointCloudNormal.size() << "\r\n";
    myfile << "property float x\r\n";
    myfile << "property float y\r\n";
    myfile << "property float z\r\n";
    myfile << "property float nx\r\n";
    myfile << "property float ny\r\n";
    myfile << "property float nz\r\n";
    myfile << "element face " << inputFaces.size() << "\r\n";
    myfile << "property list uchar uint vertex_indices\r\n";
    myfile << "end_header\r\n";

    for(unsigned long  i=0; i < inputPointCloud.PointCloudNormal.size(); i++ ){
        myfile << std::fixed << std::setprecision(6) << inputPointCloud.PointCloudNormal[i].position(0) <<" "<< inputPointCloud.PointCloudNormal[i].position(1) <<" " << inputPointCloud.PointCloudNormal[i].position(2) << " " <<
                  inputPointCloud.PointCloudNormal[i].normal(0) <<" "<< inputPointCloud.PointCloudNormal[i].normal(1) <<" " << inputPointCloud.PointCloudNormal[i].normal(2) << "\r\n";
    }
    for(unsigned long  i=0; i < inputFaces.size(); i++ ){
        myfile << "4" <<" "<< inputFaces[i](0) <<" "<< inputFaces[i](1) <<" " << inputFaces[i](2)<<" "<< inputFaces[i](3) <<"\r\n";
    }
    myfile.close();
    return 0;

}
