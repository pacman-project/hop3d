#ifndef CORE_WRITER_H
#define CORE_WRITER_H
#include "Data/Cloud.h"

#include <iostream>
#include <memory>
#include <fstream>
#include <string>
#include <iomanip>

class Writer{

public:
    /// Pointer
    typedef std::unique_ptr<Writer> Ptr;
    int writePlyToFile(std::string fileName, const PointCloud& inputPointCloud, const std::vector<Eigen::Vector4i> &inputFaces);

protected:

private:

};


#endif /* CORE_WRITER_H */
