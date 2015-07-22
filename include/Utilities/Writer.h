#ifndef UTILITIES_WRITER_H
#define UTILITIES_WRITER_H
#include "Data/Cloud.h"

#include <iostream>
#include <memory>
#include <fstream>
#include <string>
#include <iomanip>

namespace hop3d{

class Writer{

public:
    /// Pointer
    typedef std::unique_ptr<Writer> Ptr;
    int writePlyToFile(std::string fileName, const hop3d::PointCloud& inputPointCloud, const std::vector<Eigen::Vector4i> &inputFaces);

protected:

private:

};

}
#endif /* UTILITIES_WRITER_H */
