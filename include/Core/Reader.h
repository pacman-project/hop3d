#ifndef CORE_READER_H
#define CORE_READER_H
#include "Data/Cloud.h"
#include <iostream>
#include <fstream>
#include <string>

class Reader{

public:
int readPlyFile(std::string fileName, PointCloud& pointCloud, std::vector<Eigen::Vector4i> &outputFaces);

protected:
void split(const std::string& s, char c,std::vector<std::string>& v);

private:

};


#endif /* CORE_READER_H */
