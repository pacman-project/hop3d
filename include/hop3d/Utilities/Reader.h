#ifndef UTILITIES_READER_H
#define UTILITIES_READER_H
#include "hop3d/Data/Cloud.h"
#include "hop3d/Data/Defs.h"
#include "hop3d/Data/Part.h"
#include "hop3d/Data/Vocabulary.h"
#include "tinyXML/tinyxml2.h"
#include <boost/filesystem.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <memory>
#include <fstream>
#include <string>

namespace hop3d {


class Reader{

public:
/// Pointer
    typedef std::unique_ptr<Reader> Ptr;

    int readPlyFile(std::string fileName, PointCloud& outputPointCloud, std::vector<Eigen::Vector4i> &outputFaces);
/// reading first layer filters defined in Octave -- reading from xml file using tinyXML
    int readFilters(std::string patchesFileName, std::string normalsFileName, std::string masksFileName, Filter::Seq &filters);
///reading depth images from selected directory
    int readMultipleImages(boost::filesystem::path directoryPath,std::vector<cv::Mat> &output);



protected:
void split(const std::string& s, char c,std::vector<std::string>& v);

private:

};

}

#endif /* UTILITIES_READER_H */
