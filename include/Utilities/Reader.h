#ifndef CORE_READER_H
#define CORE_READER_H
#include "Data/Cloud.h"
#include <iostream>
#include <memory>
#include <fstream>
#include <string>

class Reader{

public:
/// Pointer
    typedef std::unique_ptr<Reader> Ptr;
    int readPlyFile(std::string fileName, PointCloud& outputPointCloud, std::vector<Eigen::Vector4i> &outputFaces);



protected:
void split(const std::string& s, char c,std::vector<std::string>& v);

private:

};


#endif /* CORE_READER_H */
