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
    unsigned long vertexElements=0;
    unsigned long faceElements=0;
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


/// reading first layer filters defined in Octave -- reading from xml file using tinyXML

int hop3d::Reader::readFilters(std::string patchesFileName, std::string normalsFileName, std::string masksFileName, hop3d::Filter::Seq &filters)
{
    std::string filenamePatches = "../../resources/" + patchesFileName;
    tinyxml2::XMLDocument patchesFile;
    patchesFile.LoadFile(filenamePatches.c_str());
    if (patchesFile.ErrorID())
        std::cout << "unable to load depth filter data.\n";

    int filtersNumber = 0;
    int arraySize = 0;
    tinyxml2::XMLElement * patchParse = patchesFile.FirstChildElement( "octave" );
    patchParse->FirstChildElement( "matrix" )->QueryIntAttribute("rows", &filtersNumber);
    patchParse->FirstChildElement( "matrix" )->QueryIntAttribute("columns", &arraySize);
    int filterSize = int(sqrt(double(arraySize)));
    std::cout << "Loading filters and normals...\n";
    std::cout << "Filters no.: " << filtersNumber << "\n";
    std::cout << "Filter size: " << filterSize << "\n";

    int fNumber = 0;
    cv::Mat filterTemp(filterSize,filterSize, cv::DataType<double>::type);
    std::vector<cv::Mat> filtersTemps;
    for(tinyxml2::XMLElement* e = patchParse->FirstChildElement("matrix")->FirstChildElement("scalar"); e != NULL; e = e->NextSiblingElement("scalar"))
    {
        int arrayElem = fNumber%arraySize;

        tinyxml2::XMLText* text = e->FirstChild()->ToText();
        if(text == NULL){
            continue;
        }
        std::string t = text->Value();
        int xCol = arrayElem/filterSize;
        int yCol = arrayElem%filterSize;
        filterTemp.at<double>(xCol,yCol) = std::stod(t);
        fNumber++;
        arrayElem = fNumber%arraySize;
        if (!arrayElem){
            //Done like this because just pushing filterTemp ends up with optimisation where all the elements of the vector are the same
            //have value of last element
            cv::Mat imageRoi;
            imageRoi = filterTemp.clone();
            filtersTemps.push_back(imageRoi);
            imageRoi.release();
        }
    }

    std::string filenameMasks = "../../resources/" + masksFileName;
    tinyxml2::XMLDocument masksFile;
    masksFile.LoadFile(filenameMasks.c_str());
    if (masksFile.ErrorID())
        std::cout << "unable to load depth filter data.\n";

    int masksNumber = 0;
    int arrayMSize = 0;
    tinyxml2::XMLElement * maskParse = masksFile.FirstChildElement( "octave" );
    maskParse->FirstChildElement( "matrix" )->QueryIntAttribute("rows", &masksNumber);
    maskParse->FirstChildElement( "matrix" )->QueryIntAttribute("columns", &arrayMSize);
    int maskSize = int(sqrt(double(arrayMSize)));
    std::cout << "Loading filters and normals...\n";
    std::cout << "Filters no.: " << masksNumber << "\n";
    std::cout << "Filter size: " << maskSize << "\n";

    int mNumber = 0;
    cv::Mat maskTemp(maskSize,maskSize, cv::DataType<double>::type);
    std::vector<cv::Mat> masksTemps;
    for(tinyxml2::XMLElement* e = maskParse->FirstChildElement("matrix")->FirstChildElement("scalar"); e != NULL; e = e->NextSiblingElement("scalar"))
    {
        int arrayElem = mNumber%arrayMSize;

        tinyxml2::XMLText* text = e->FirstChild()->ToText();
        if(text == NULL){
            continue;
        }
        std::string t = text->Value();
        int xCol = arrayElem/filterSize;
        int yCol = arrayElem%filterSize;
        maskTemp.at<double>(xCol,yCol) = std::stod(t);
        mNumber++;
        arrayElem = mNumber%arrayMSize;
        if (!arrayElem){
            //Done like this because just pushing filterTemp ends up with optimisation where all the elements of the vector are the same
            //have value of last element
            cv::Mat imageRoi;
            imageRoi = maskTemp.clone();
            masksTemps.push_back(imageRoi);
            imageRoi.release();
        }
    }









    tinyxml2::XMLDocument normalsFile;
    std::string filenameNormals = "../../resources/" + normalsFileName;
    normalsFile.LoadFile(filenameNormals.c_str());
    if (normalsFile.ErrorID())
        std::cout << "unable to load depth filter data.\n";
    int normalsNumber = 0;
    int normalSize = 0;
    tinyxml2::XMLElement * normalParse = normalsFile.FirstChildElement( "octave" );
    normalParse->FirstChildElement( "matrix" )->QueryIntAttribute("rows", &normalsNumber);
    normalParse->FirstChildElement( "matrix" )->QueryIntAttribute("columns", &normalSize);
    std::cout << "Load filter parameters...\n";
    std::cout << "Filters no.: " << normalsNumber << "\n";
    std::cout << "Filters size: " << normalSize << "\n";

    int nNumber = 0;
    hop3d::Vec3 normalTemp;
    std::vector<hop3d::Vec3> normalsTemps;
    for(tinyxml2::XMLElement* e = normalParse->FirstChildElement("matrix")->FirstChildElement("scalar"); e != NULL; e = e->NextSiblingElement("scalar"))
    {
        int arrayElem = nNumber%normalSize;
        tinyxml2::XMLText* text = e->FirstChild()->ToText();
        if(text == NULL){
            continue;
        }
        std::string t = text->Value();
        normalTemp(arrayElem,0) = std::stod(t);
        nNumber++;
        arrayElem = nNumber%normalSize;
        if (!arrayElem){
            //std::cout << "normalTemp "<< nNumber/normalSize <<" = " << std::endl <<        normalTemp           << std::endl << std::endl;
            normalsTemps.push_back(normalTemp);

        }
    }
    std::cout << normalsTemps.size() << std::endl;
    for(unsigned int i = 0; i < normalsTemps.size();i++){
        hop3d::Filter tempFilter;
        tempFilter.id = i;
        tempFilter.mask = masksTemps[i];
        tempFilter.patch = filtersTemps[i];
        tempFilter.normal = normalsTemps[i];
        filters.push_back(tempFilter);
    }
    return 0;
}

int hop3d::Reader::readMultipleImages(boost::filesystem::path directoryPath, std::vector<cv::Mat> &output)
{
    try {
            if (exists(directoryPath))    // does directoryPath actually exist?
            {
              if (is_regular_file(directoryPath))        // is directoryPath a regular file?
                std::cout << directoryPath << " size is " << file_size(directoryPath) << '\n';

              else if (is_directory(directoryPath))      // is directoryPath a directory?
              {
                std::cout << directoryPath << " is a directory containing:\n";

                typedef std::vector<boost::filesystem::path> vec;             // store paths,
                vec v;                                // so we can sort them later

                copy(boost::filesystem::directory_iterator(directoryPath), boost::filesystem::directory_iterator(), back_inserter(v));

                sort(v.begin(), v.end());             // sort, since directory iteration
                                                      // is not ordered on some file systems
                int counter = 0;
                for (vec::const_iterator it(v.begin()), it_end(v.end()); it != it_end; ++it)
                {
                  std::cout << "   " << *it << '\n';
                  cv::Mat image = cv::imread(it->string(), CV_LOAD_IMAGE_UNCHANGED);
                  output.push_back(image);
                  counter++;
                }
              }
              else
                std::cout << directoryPath << " exists, but is neither a regular file nor a directory\n";
            }
            else
              std::cout << directoryPath << " does not exist\n";
          }
    catch (const boost::filesystem::filesystem_error& ex){
        std::cout << ex.what() << '\n';
    }
    return 0;
}

