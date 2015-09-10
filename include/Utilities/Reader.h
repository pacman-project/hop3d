#ifndef UTILITIES_READER_H
#define UTILITIES_READER_H
#include "Data/Cloud.h"
#include "Data/Defs.h"
#include "Data/Part.h"
#include "Data/Vocabulary.h"
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
    int readFirstLayerVoc(std::string fileName, Filter::Seq &parts);



protected:
void split(const std::string& s, char c,std::vector<std::string>& v);

private:

};

}

#endif /* UTILITIES_READER_H */
