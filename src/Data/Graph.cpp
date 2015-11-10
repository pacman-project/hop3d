#include "Data/Graph.h"
#include "../../external/tinyXML/tinyxml2.h"

namespace hop3d {

/// Construction
Hierarchy::Hierarchy(std::string configFilename) {
    tinyxml2::XMLDocument config;
    std::string filename = "../../resources/" + configFilename;
    config.LoadFile(filename.c_str());
    if (config.ErrorID()){
        std::cout << "unable to load hierarchy config file.\n";
    }
    tinyxml2::XMLElement *params = config.FirstChildElement("Hierarchy")->FirstChildElement("parameters");
    int VDLayersNo, VIndLayersNo;
    params->QueryIntAttribute("viewDependentLayersNo", &VDLayersNo);
    params->QueryIntAttribute("viewIndependentLayersNo", &VIndLayersNo);

    this->viewDependentLayers.resize(VDLayersNo);
    this->viewIndependentLayers.resize(VIndLayersNo);
}

// Insertion operator
std::ostream& operator<<(std::ostream& os, const Hierarchy& hierarchy){
    //save filters no
    os << hierarchy.firstLayer.size() << "\n";
    for (auto& filter : hierarchy.firstLayer){
        os << filter.id;
        os << " " << filter.normal(0) << " " << filter.normal(1) << " " << filter.normal(2) << " ";
        os << filter.mask.rows << " " << filter.mask.cols << " ";
        for (int i=0;i<filter.mask.rows;i++){
            for (int j=0;j<filter.mask.cols;j++){
                os << filter.mask.at<double>(i,j) << " ";
            }
        }
        os << filter.patch.rows << " " << filter.patch.cols << " ";
        for (int i=0;i<filter.patch.rows;i++){
            for (int j=0;j<filter.patch.cols;j++){
                os << filter.patch.at<double>(i,j) << " ";
            }
        }
        os << "\n";
    }
    return os;
}

// Extraction operator
std::istream& operator>>(std::istream& is, Hierarchy& hierarchy){
    // read filters no
    int filtersNo;
    is >> filtersNo;
    hierarchy.firstLayer.clear();
    hierarchy.firstLayer.reserve(filtersNo);
    for (int i=0;i<filtersNo;i++){
        Filter filterTmp;
        is >> filterTmp.id;
        is >> filterTmp.normal(0) >> filterTmp.normal(1) >> filterTmp.normal(2);
        int cols, rows;
        is >> rows >> cols;
        cv::Mat maskTmp(cols,rows, cv::DataType<double>::type,1);
        for (int i=0;i<rows;i++){
            for (int j=0;j<cols;j++){
                is >> maskTmp.at<double>(i,j);
            }
        }
        filterTmp.mask = maskTmp.clone();
        is >> rows >> cols;
        cv::Mat patchTmp(cols,rows, cv::DataType<double>::type,1);
        for (int i=0;i<rows;i++){
            for (int j=0;j<cols;j++){
                is >> patchTmp.at<double>(i,j);
            }
        }
        filterTmp.patch = patchTmp.clone();
        hierarchy.firstLayer.push_back(filterTmp);
    }
    //cv::Mat patchTmp(config.filterSize,config.filterSize, cv::DataType<double>::type);
    return is;
}

/// get normal vector related to the part
void Hierarchy::getNormal(const ViewDependentPart& part, Vec3& normal) const{
    int layerId = part.layerId;
    if (layerId==2) {
        int filterId = part.partIds[1][1];
        normal = firstLayer[filterId].normal;
    }
    else if (layerId==3) {
        int filterId = viewDependentLayers[0][part.partIds[1][1]].partIds[1][1];
        normal = firstLayer[filterId].normal;
    }
}

/// print ids
void Hierarchy::printIds(const ViewDependentPart& part){
    if (part.layerId==2){
        for (int i=0;i<3;i++){
            for (int j=0;j<3;j++){
                std::cout << part.partIds[0][0] << ", ";
            }
            std::cout << "\n";
        }
    }
    else if (part.layerId==3){
        for (int i=0;i<3;i++){
            for (int j=0;j<3;j++){
                for (int k=0;k<3;k++){
                    for (int l=0;l<3;l++){
                        if (part.partIds[i][k]==-1)
                            std::cout << "-1, ";
                        else
                            std::cout << viewDependentLayers[0][part.partIds[i][k]].partIds[j][l] << ", ";
                    }
                }
                std::cout << "\n";
                //(0,0,0,0) (0,0,0,1) (0,0,0,2) (0,1,0,0) (0,1,0,1) (0,1,0,2) (0,2,0,0) (0,2,0,1) (0,2,0,2)
                //(0,0,1,0) (0,0,1,1) (0,0,1,2) (0,1,1,0) (0,1,1,1) (0,1,1,2) (0,2,1,0) (0,2,1,1) (0,2,1,2)
                //(0,0,2,0) (0,0,2,1) (0,0,2,2) (0,1,2,0) (0,1,2,1) (0,1,2,2) (0,2,2,0) (0,2,2,1) (0,2,2,2)
            }
        }
    }
}

}
