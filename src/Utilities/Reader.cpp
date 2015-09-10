#include "Utilities/Reader.h"



void hop3d::Reader::split(const std::string& s, char c,std::vector<std::string>& v) {
   std::size_t i = 0;
   std::size_t j = s.find(c);

   while (j != std::string::npos) {
      v.push_back(s.substr(i, j-i));
      i = ++j;
      j = s.find(c, j);

      if (j == std::string::npos) {
         v.push_back(s.substr(i, s.length()));
      }
   }
}

int hop3d::Reader::readPlyFile(std::string fileName, PointCloud& outputPointCloud, std::vector<Eigen::Vector4i> &outputFaces){
    std::cout << "Loading file: " << fileName << std::endl;
    std::string line;
    std::ifstream myfile (fileName.c_str());
    unsigned long vertexElements;
    unsigned long faceElements;
    if (myfile.is_open()){
        while ( getline (myfile,line) ){
            std::vector<std::string> lineSeq;
            split(line,' ',lineSeq);
           if(lineSeq.size() > 0){
                if(lineSeq[0] == "element"){
                    if(lineSeq[1] == "vertex"){
                        vertexElements = std::atoi(lineSeq[2].c_str());
                        std::cout << "Number of vertices: " << vertexElements <<std::endl;
                    }
                    if(lineSeq[1] == "face"){
                        faceElements = std::stol(lineSeq[2].c_str());
                        std::cout << "Number of faces: " << faceElements <<std::endl;
                    }

                }
            }
            if(line == "end_header"){
                break;
            }
        }
        for(unsigned long  i=0; i < vertexElements; i++ ){
        getline (myfile,line);
        line.erase(std::remove(line.begin(), line.end(), '\n'), line.end());
        line.erase(std::remove(line.begin(), line.end(), '\r'), line.end());
        line.erase(std::remove(line.begin(), line.end(), '\t'), line.end());
            std::vector<std::string> lineSeq;
            split(line,' ',lineSeq);
            Eigen::Vector3d vecPoint;
            Eigen::Vector3d vecNorm;
            for(int i = 0; i< 3; i++) vecPoint(i) = std::stof(lineSeq[i].c_str());
            for(int i = 3; i< 6; i++) vecNorm(i-3) = std::stof(lineSeq[i].c_str());
            hop3d::PointNormal tempPointNormal;
            tempPointNormal.position = vecPoint;
            tempPointNormal.normal = vecNorm;
            outputPointCloud.pointCloudNormal.push_back(tempPointNormal);
        }
        std::cout << "Points and normals successfuly loaded from " << fileName << std::endl;
        std::cout << "The data occupies " << (2*(sizeof(std::vector<Eigen::Vector3d>) + (sizeof(Eigen::Vector3d) * outputPointCloud.pointCloudNormal.size())))/1024/1024 << " MB" << std::endl;
        for(unsigned long  i=0; i < faceElements; i++ ){
        getline (myfile,line);
        line.erase(std::remove(line.begin(), line.end(), '\n'), line.end());
        line.erase(std::remove(line.begin(), line.end(), '\r'), line.end());
        line.erase(std::remove(line.begin(), line.end(), '\t'), line.end());
            std::vector<std::string> lineSeq;
            split(line,' ',lineSeq);
            Eigen::Vector4i vecFace;
            for(int i = 1; i< 5; i++) vecFace(i-1) = std::stoi(lineSeq[i].c_str());
            outputFaces.push_back(vecFace);
        }
        myfile.close();

    }
    else{
        std::cout << "Unable to open ply file";
        return 1;
    }
    return 0;
}

int hop3d::Reader::readFirstLayerVoc(std::string fileName, hop3d::FirstLayerPart::Seq &parts){
    std::vector<hop3d::Vec3> discretizedNormals;
    std::cout << "Loading file: " << fileName << std::endl;
    std::string line;
    std::ifstream myfile (fileName.c_str());
    hop3d::U64 id = 0;
    if (myfile.is_open()){
        while ( getline (myfile,line) ){

        line.erase(std::remove(line.begin(), line.end(), '\n'), line.end());
        line.erase(std::remove(line.begin(), line.end(), '\r'), line.end());
        line.erase(std::remove(line.begin(), line.end(), '\t'), line.end());
            std::vector<std::string> lineSeq;
            split(line,',',lineSeq);
            Eigen::Vector3d vecNorm;
            for(int i = 0; i< 3; i++) vecNorm(i) = ::atof(lineSeq[i].c_str());
            hop3d::FirstLayerPart part;
            //part.setPartId(id);
            part.setNormal(vecNorm);
            parts.push_back(part);
            id++;
            //discretizedNormals.push_back(vecNorm);
        }
        std::cout << "Normals successfuly loaded from " << fileName << std::endl;
        std::cout << "The data occupies " << (sizeof(std::vector<Eigen::Vector3d>) + (sizeof(Eigen::Vector3d) * discretizedNormals.size()))/1024 << " kB" << std::endl;
        myfile.close();
    }
    else{
        std::cout << "Unable to first layer vocabulary file.";
        return 1;
    }
//    for(int i = 0; i < discretizedNormals.size(); i++){
//        double dist;
//        double dot;
//        dot = discretizedNormals[0].dot(discretizedNormals[i]);
//        if(dot > 1) dot = 1;
//        if(dot < -1) dot = -1;
//        dist = acos(dot);
//        cout << discretizedNormals[0].dot(discretizedNormals[i]) << std::endl;

//        _normalsDiscreteMap[dist] = discretizedNormals[i];
//    }
//    std::cout << "mymap contains:";
//      for ( auto it = _normalsDiscreteMap.begin(); it != _normalsDiscreteMap.end(); ++it )
//        std::cout << " " << it->first << ":" << std::endl << it->second << std::endl;
//        std::cout << std::endl;
    return 0;
}


/// reading first layer filters defined in Octave -- reading from xml file using tinyXML

int hop3d::Reader::readFilters(std::string patchesFileName, std::string normalsFileName, hop3d::Filter::Seq &filters)
{
    tinyxml2::XMLDocument patchesFile;
    std::string filenamePatches = "../../resources/" + patchesFileName;
    config.LoadFile(filenamePatches.c_str());
    if (patchesFile.ErrorID())
        std::cout << "unable to load depth filter config file.\n";
    tinyxml2::XMLDocument normalsFile;
    std::string filenameNormals = "../../resources/" + normalsFileName;
    config.LoadFile(filenameNormals.c_str());
    if (normalsFile.ErrorID())
        std::cout << "unable to load depth filter config file.\n";


}
