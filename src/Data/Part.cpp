#include "hop3d/Data/Part.h"
#include "hop3d/Data/Vocabulary.h"
#include "hop3d/ImageFilter/normalImageFilter.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/gicp.h>
#include <chrono>
#include <random>

namespace hop3d {

/// Print
void ViewDependentPart::print() const{
    std::cout << "Id: " << id << "\n";
    std::cout << "Layer id: " << layerId << "\n";
    std::cout << "Image coords: " << location.u << ", " << location.v << ", " << location.depth << "\n";
    std::cout << "Location Euclidean: " << locationEucl(0) << ", " << locationEucl(1) << ", " << locationEucl(2) << "\n";
    std::cout << "Group size: " << group.size() << "\n";
    std::cout << "ids: ";
    for (int i =0; i<3; i++){
        for (int j =0; j<3; j++){
            std::cout << this->partIds[i][j] << ", ";
        }
        std::cout << "\n";
    }

    /*std::cout << "Gaussians:\n";
    for (size_t i=0; i<gaussians.size();i++){
        for (size_t j=0; j<gaussians.size();j++){
            std::cout << "mean(" << i << ", " << j << "): ";
            std::cout << gaussians[i][j].mean.transpose() << "\n";
            //std::cout << "covariance(" << i << ", " << j << "):\n";
            //std::cout << gaussians[i][j].covariance << "\n";
        }
    }*/
    std::cout << "offsets:\n";
        for (size_t i=0; i<offsets.size();i++){
            for (size_t j=0; j<offsets.size();j++){
                if (partIds[i][j]>=0){
                    std::cout << "offset(" << i << ", " << j << "): \n";
                    std::cout << offsets[i][j].matrix() << "\n";
                }
            }
        }
    std::cout << "Pos norm SE3:\n";
    for (size_t i=0; i<partsPosNorm.size();i++){
        for (size_t j=0; j<partsPosNorm.size();j++){
            std::cout << "mean(" << i << ", " << j << "): ";
            std::cout << partsPosNorm[i][j].mean.transpose() << "\n";
            //std::cout << "covariance(" << i << ", " << j << "):\n";
            //std::cout << gaussians[i][j].covariance << "\n";
        }
    }
    std::cout << "\nParts in group: ";
    for (auto it = group.begin();it!=group.end();it++){
        it->print();
        std::cout << it->id << ", ";
    }
    std::cout << "\n";
}

/// check if part is background
bool ViewDependentPart::isBackground(void) const{
    for (size_t i=0; i<partIds.size();i++){
        for (size_t j=0; j<partIds[0].size();j++){
            if (partIds[i][j]>=0)
                return false;
        }
    }
    return true;
}

/// Print
void ViewIndependentPart::print() const{
    std::cout << "Id: " << id << "\n";
    std::cout << "Layer id: " << layerId << "\n";
    std::cout << "Pose: \n" << pose.matrix() << "\n";
    std::cout << "ids: ";
    for (int i =0; i<3; i++){
        for (int j =0; j<3; j++){
            for (int k =0; k<3; k++){
                std::cout << this->partIds[i][j][k] << ", ";
            }
        }
    }
    std::cout << "\nGroup size: " << group.size() << "\n";
    std::cout << "Parts in group: ";
    for (auto it = group.begin();it!=group.end();it++)
        std::cout << it->id << ", ";
    std::cout << "\n Gaussians:\n";
    for (size_t i=0; i<gaussians.size();i++){
        for (size_t j=0; j<gaussians.size();j++){
            for (size_t k=0; k<gaussians.size();k++){
                if (partIds[i][j][k]>=0){
                    std::cout << "mean(" << i << ", " << j << ", " << k << "): ";
                    std::cout << gaussians[i][j][k].mean.transpose() << ", ";
                    std::cout << "covariance(" << i << ", " << j << ", " << k << "): ";
                    std::cout << gaussians[i][j][k].covariance << "\n";
                }
            }
        }
    }
    std::cout << "\n neighbour pose\n";
    for (size_t i=0; i<neighbourPoses.size();i++){
        for (size_t j=0; j<neighbourPoses.size();j++){
            for (size_t k=0; k<neighbourPoses.size();k++){
                if (partIds[i][j][k]>=0){
                    std::cout << "neigbour pose(" << i << ", " << j << ", " << k << "): ";
                    std::cout << neighbourPoses[i][j][k].matrix() << "\n";
                }
            }
        }
    }
}

/// compute distance between view-independent parts
double ViewIndependentPart::distance(const ViewIndependentPart& partA, const ViewIndependentPart& partB, Mat34& offset){
    if (partA.partIds==partB.partIds){
        offset = Mat34::Identity();
        return 0;
    }
    double sum=0;
    std::vector<std::pair<Mat34, int>> partASeq;
    std::vector<std::pair<Mat34, int>> partBSeq;
    Vec3 normA(0,0,0), normB(0,0,0);
    Vec3 meanPosA(0,0,0); Vec3 meanPosB(0,0,0);
    getPoints(partA,partB, partASeq, partBSeq, meanPosA, normA, meanPosB, normB);
    offset = computeOffset(partASeq, partBSeq, meanPosA, normA, meanPosB, normB);

    if (partASeq.size()<partBSeq.size()){
        for (auto & part : partASeq){
            int neighbourId=0;
            sum+=nearestNeighbour(part.first*offset,partBSeq, neighbourId);
            if (part.second!=partBSeq[neighbourId].second)
                sum+=1;
        }
        sum/=double(partASeq.size());//divide by the number of matched elements
    }
    else{
        for (auto & part : partBSeq){
            int neighbourId=0;
            sum+=nearestNeighbour(part.first*(offset.inverse()),partASeq, neighbourId);
            if (part.second!=partASeq[neighbourId].second)
                sum+=1;
        }
        sum/=double(partBSeq.size());//divide by the number of matched elements
    }
    sum+=fabs((double)partBSeq.size()-(double)partASeq.size())*81.0;//on 5th layer each part contain 81 points
    return sum;
}

///get central points from parts, mean and mean normal vector
void ViewIndependentPart::getPoints(const ViewIndependentPart& partA, const ViewIndependentPart& partB, std::vector<std::pair<Mat34, int>>& partASeq, std::vector<std::pair<Mat34, int>>& partBSeq, Vec3& meanPosA, Vec3& normA, Vec3& meanPosB, Vec3& normB){
    normA = Vec3(0,0,0); normB = Vec3(0,0,0);
    meanPosA = Vec3(0,0,0); meanPosB = Vec3(0,0,0);
    for (int i=0;i<3;i++){
        for (int j=0;j<3;j++){
            for (int k=0;k<3;k++){
                //Mat34 rel(Eigen::Translation<double, 3>(double(i),double(j),double(k))*Quaternion(1,0,0,0));
                if(partA.partIds[i][j][k]>=0){
                    normA+=partA.neighbourPoses[i][j][k].matrix().block<3,1>(0,2);
      //              meanA+=partA.neighbourPoses[i][j][k].matrix().block<3,1>(0,3);
                    partASeq.push_back(std::make_pair(partA.neighbourPoses[i][j][k],partA.partIds[i][j][k]));
                    meanPosA+=partA.neighbourPoses[i][j][k].matrix().block<3,1>(0,3);
                }
                if(partB.partIds[i][j][k]>=0){
                    normB+=partB.neighbourPoses[i][j][k].matrix().block<3,1>(0,2);
        //            meanB+=partB.neighbourPoses[i][j][k].matrix().block<3,1>(0,3);
                    partBSeq.push_back(std::make_pair(partB.neighbourPoses[i][j][k],partB.partIds[i][j][k]));
                    meanPosB+=partB.neighbourPoses[i][j][k].matrix().block<3,1>(0,3);
                }
            }
        }
    }
}

///compute offset for two sets of points
Mat34 ViewIndependentPart::computeOffset(const std::vector<std::pair<Mat34, int>>& partASeq, const std::vector<std::pair<Mat34, int>>&partBSeq, Vec3& meanPosA, Vec3& normA, Vec3& meanPosB, Vec3& normB){
    meanPosA/=(double)partASeq.size(); meanPosB/=(double)partBSeq.size();
    normA.normalize(); normB.normalize();
    Mat33 coordPartA = coordinateFromNormal(normA);
    Mat33 coordPartB = coordinateFromNormal(normB);
    Mat34 partApose(Eigen::Translation<double,3>(meanPosA(0),meanPosA(1),meanPosA(2))*coordPartA);
    Mat34 partBpose(Eigen::Translation<double,3>(meanPosB(0),meanPosB(1),meanPosB(2))*coordPartB);
    return partApose.inverse()*partBpose;
}

/// compute distance between view-independent parts (7th layer)
double ViewIndependentPart::distance(const ViewIndependentPart& partA, const ViewIndependentPart& partB, const ViewIndependentPart::Seq vocabulary1, const ViewIndependentPart::Seq vocabulary2, Mat34& offset){
    if (partA.partIds==partB.partIds){
        offset = Mat34::Identity();
        return 0;
    }
    std::vector<std::pair<Mat34, int>> partASeq;
    std::vector<std::pair<Mat34, int>> partBSeq;
    Vec3 normA(0,0,0), normB(0,0,0);
    Vec3 meanPosA(0,0,0); Vec3 meanPosB(0,0,0);
    for (int i=0;i<3;i++){
        for (int j=0;j<3;j++){
            for (int k=0;k<3;k++){
                ViewIndependentPart pA, pB;
                if (partA.partIds[i][j][k]>=0)
                    pA= vocabulary2[partA.partIds[i][j][k]];
                if (partB.partIds[i][j][k]>=0)
                    pB= vocabulary2[partB.partIds[i][j][k]];
                if ((partA.partIds[i][j][k]>=0)&&(partA.partIds[i][j][k]>=0)){
                    for (int l=0;l<3;l++){
                        for (int m=0;m<3;m++){
                            for (int n=0;n<3;n++){
                                ViewIndependentPart pAA, pBB;
                                if (pA.partIds[i][j][k]>=0)
                                    pAA= vocabulary1[pA.partIds[i][j][k]];
                                if (pB.partIds[i][j][k]>=0)
                                    pBB= vocabulary1[pB.partIds[i][j][k]];
                                Vec3 nA(0,0,0), nB(0,0,0);
                                Vec3 meanA(0,0,0); Vec3 meanB(0,0,0);
                                getPoints(pAA, pBB, partASeq, partBSeq, meanA, nA, meanB, nB);
                                if (pA.partIds[i][j][k]>=0){
                                    meanPosA+=meanA; normA+=nA;
                                }
                                if (pB.partIds[i][j][k]>=0){
                                    meanPosB+=meanB; normB+=nB;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
    double sum=0;
    offset = computeOffset(partASeq, partBSeq, meanPosA, normA, meanPosB, normB);

    if (partASeq.size()<partBSeq.size()){
        for (auto & part : partASeq){
            int neighbourId=0;
            sum+=nearestNeighbour(part.first*offset,partBSeq, neighbourId);
            if (part.second!=partBSeq[neighbourId].second)
                sum+=1;
        }
        sum/=double(partASeq.size());//divide by the number of matched elements
    }
    else{
        for (auto & part : partBSeq){
            int neighbourId=0;
            sum+=nearestNeighbour(part.first*(offset.inverse()),partASeq, neighbourId);
            if (part.second!=partASeq[neighbourId].second)
                sum+=1;
        }
        sum/=double(partBSeq.size());//divide by the number of matched elements
    }
    sum+=fabs((double)partBSeq.size()-(double)partASeq.size())*81.0;//on 5th layer each part contain 81 points
    return sum;
}

/// compute coordinate system from normal vector
Mat33 ViewIndependentPart::coordinateFromNormal(const Vec3& _normal){
    Vec3 x(1,0,0); Vec3 y;
    Vec3 normal(_normal);
    y = normal.cross(x);
    y.normalize();
    x = y.cross(normal);
    x.normalize();
    Mat33 R;
    R.block(0,0,3,1) = x;
    R.block(0,1,3,1) = y;
    R.block(0,2,3,1) = normal;
    return R;
}

/// compute distance between view-independent parts
double ViewIndependentPart::distanceGICP(const ViewIndependentPart& partA, const ViewIndependentPart& partB, const ConfigGICP& configGICP, Mat34& offset){
    //pcl::PointCloud<pcl::PointNormal> sourceCloud;
    if (!configGICP.verbose){
        pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
    }
    auto startTime = std::chrono::high_resolution_clock::now();
    pcl::PointCloud<pcl::PointNormal>::Ptr sourceCloud (new pcl::PointCloud<pcl::PointNormal>);
    //pcl::IterativeClosestPointWithNormals sourceCovariance (new pcl::PointCloud<pcl::PointNormal>);
    for (auto& point : partA.cloud){
        pcl::PointNormal pointPCL;
        pointPCL.x = (float)point.position(0);    pointPCL.y = (float)point.position(1);    pointPCL.z = (float)(float)point.position(2);
        pointPCL.normal_x=(float)point.normal(0); pointPCL.normal_y=(float)point.normal(1);   pointPCL.normal_z=(float)point.normal(2);
        sourceCloud->push_back(pointPCL);
        /*Mat33 rot = coordinateFromNormal(point.normal);
        Mat33 S(Mat33::Identity()); S(0,0)=0.2; S(1,1)=0.2;
        Mat33 sigma = rot*S*S*rot.inverse();*/
    }
    //std::cout << "courceCloud.size() " << sourceCloud->size() <<"\n";
    //pcl::PointCloud<pcl::PointNormal> targetCloud;
    pcl::PointCloud<pcl::PointNormal>::Ptr targetCloud (new pcl::PointCloud<pcl::PointNormal>);
    for (auto& point : partB.cloud){
        pcl::PointNormal pointPCL;
        pointPCL.x = (float)point.position(0);    pointPCL.y = (float)point.position(1);    pointPCL.z = (float)point.position(2);
        pointPCL.normal_x=(float)point.normal(0); pointPCL.normal_y=(float)point.normal(1);   pointPCL.normal_z=(float)point.normal(2);
        targetCloud->push_back(pointPCL);
    }
    //std::cout << "targetCloud.size() " << targetCloud->size() <<"\n";
    // setup Generalized-ICP
    pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> gicp;
    gicp.setMaxCorrespondenceDistance(configGICP.correspondenceDist);
    gicp.setInputSource(sourceCloud);
    gicp.setInputTarget(targetCloud);
    Eigen::Matrix4f transform(Eigen::Matrix4f::Identity());
    double errorMin=1;
    std::chrono::high_resolution_clock::duration d = std::chrono::high_resolution_clock::now() - startTime;
    unsigned seed2 = (unsigned)d.count();
    std::default_random_engine generator;
    generator.seed(seed2);
    std::uniform_real_distribution<double> distributionAlpha(configGICP.alpha.first,configGICP.alpha.second);
    std::uniform_real_distribution<double> distributionBeta(configGICP.beta.first,configGICP.beta.second);
    std::uniform_real_distribution<double> distributionGamma(configGICP.gamma.first,configGICP.gamma.second);
    for (int iter=0;iter<configGICP.guessesNo;iter++){
        double rot[3]={distributionAlpha(generator),distributionBeta(generator),distributionGamma(generator)};
        if (iter==0)
            std::fill(rot, rot+3, 0);
        Eigen::Matrix3d m;
        m = Eigen::AngleAxisd(rot[0], Eigen::Vector3d::UnitZ())* Eigen::AngleAxisd(rot[1], Eigen::Vector3d::UnitY())* Eigen::AngleAxisd(rot[2], Eigen::Vector3d::UnitZ());
        Eigen::Matrix4f initEst(Eigen::Matrix4f::Identity());
        initEst.block<3,3>(0,0)=m.cast<float>();
        //gicp.transformation_ = initEst;
        //gicp.setSourceCovariances(sourceCovariances);
        //gicp.setTargetCovariances(targetCovariances);
        // run registration and get transformation
        pcl::PointCloud<pcl::PointNormal> output;
        Eigen::Matrix<float, 4, 4> estM = initEst;
        gicp.align(output, estM);
        if (gicp.hasConverged()){
            if (gicp.getFitnessScore()<errorMin&&fabs(gicp.getFinalTransformation()(0,3))<0.05&&fabs(gicp.getFinalTransformation()(1,3))<0.05&&fabs(gicp.getFinalTransformation()(2,3))<0.05){
                transform = gicp.getFinalTransformation();
                errorMin = gicp.getFitnessScore();
            }
        }
    }
    offset.matrix() = transform.cast<double>();
    if (configGICP.verbose){
        auto endTime = std::chrono::high_resolution_clock::now();
        auto time = endTime - startTime;
        std::cout << "It took " << std::chrono::duration_cast<std::chrono::microseconds>(time).count() << " us to run.\n";
    }
    return errorMin;
}

/// compute distance between view-independent parts
double ViewIndependentPart::distance(const ViewIndependentPart& partA, const ViewIndependentPart& partB, const ViewIndependentPart::Seq vocabulary, Mat34& offset){
    if (partA.partIds==partB.partIds){
        offset = Mat34::Identity();
        return 0;
    }
    std::vector<std::pair<Mat34, int>> partASeq;
    std::vector<std::pair<Mat34, int>> partBSeq;
    Vec3 normA(0,0,0), normB(0,0,0);
    Vec3 meanPosA(0,0,0); Vec3 meanPosB(0,0,0);
    for (int i=0;i<3;i++){
        for (int j=0;j<3;j++){
            for (int k=0;k<3;k++){
                ViewIndependentPart pA, pB;
                if (partA.partIds[i][j][k]>=0)
                    pA= vocabulary[partA.partIds[i][j][k]];
                if (partB.partIds[i][j][k]>=0)
                    pB= vocabulary[partB.partIds[i][j][k]];
                Vec3 nA(0,0,0), nB(0,0,0);
                Vec3 meanA(0,0,0); Vec3 meanB(0,0,0);
                getPoints(pA, pB, partASeq, partBSeq, meanA, nA, meanB, nB);
                if (partA.partIds[i][j][k]>=0){
                    meanPosA+=meanA; normA+=nA;
                }
                if (partB.partIds[i][j][k]>=0){
                    meanPosB+=meanB; normB+=nB;
                }
            }
        }
    }
    double sum=0;
    offset = computeOffset(partASeq, partBSeq, meanPosA, normA, meanPosB, normB);

    if (partASeq.size()<partBSeq.size()){
        for (auto & part : partASeq){
            int neighbourId=0;
            sum+=nearestNeighbour(part.first*offset,partBSeq, neighbourId);
            if (part.second!=partBSeq[neighbourId].second)
                sum+=1;
        }
        sum/=double(partASeq.size());//divide by the number of matched elements
    }
    else{
        for (auto & part : partBSeq){
            int neighbourId=0;
            sum+=nearestNeighbour(part.first*(offset.inverse()),partASeq, neighbourId);
            if (part.second!=partASeq[neighbourId].second)
                sum+=1;
        }
        sum/=double(partBSeq.size());//divide by the number of matched elements
    }
    sum+=fabs((double)partBSeq.size()-(double)partASeq.size())*81.0;//on 5th layer each part contain 81 points
    return sum;
}

/// compute the min distance to the set of parts
double ViewIndependentPart::nearestNeighbour(const Mat34& pose, std::vector<std::pair<Mat34, int>> parts, int& neighbourId){
    double minDist=std::numeric_limits<double>::max();
    int partId=0;
    for (auto part : parts){
        double distance = sqrt(pow(pose(0,3)-part.first(0,3),2.0)+pow(pose(1,3)-part.first(1,3),2.0)+pow(pose(2,3)-part.first(2,3),2.0));
        if (distance<minDist){
            minDist = distance;
            neighbourId = partId;
        }
        partId++;
    }
    return minDist;
}

/// remove elements which belong to "second surface"
bool ViewDependentPart::removeSecondSurface(ViewDependentPart& part, double distThreshold) {
    bool has2surfs(false);
    //part.print();
    std::vector<double> depth;
    for (int i=0;i<3;i++){//detect two surfaces
        for (int j=0;j<3;j++){
            if (part.partIds[i][j]>=0){
                if (i==1&&j==1)
                    depth.push_back(0);
                else
                    depth.push_back(part.partsPosNorm[i][j].mean(2));
            }
        }
    }
    std::sort(depth.begin(),depth.end());
    double distBorder;
    //size_t groupSize=0;
    bool removeBack=false;
    for (size_t i=0;i<depth.size()-1;i++){
        if (depth[i+1]-depth[i]>distThreshold){
            if (i+1>=depth.size()-i){
                //groupSize = i+1;
                removeBack=true;
            }
            //else
            //    groupSize = depth.size()-i;
            distBorder = depth[i]+std::numeric_limits<double>::epsilon();
            has2surfs = true;
        }
    }
    if (!has2surfs)
        return false;
    for (int i=0;i<3;i++){// remove smaller surface
        for (int j=0;j<3;j++){
            if (removeBack) {//remove back surface
                if (i==1&&j==1){
                    if (0>distBorder)
                        part.partIds[i][j]=-1;
                }
                else {
                    if (part.partsPosNorm[i][j].mean(2)>distBorder)
                        part.partIds[i][j]=-1;
                }
            }
            else {//remove front surface
                if (i==1&&j==1){
                    if (0<distBorder)
                        part.partIds[i][j]=-1;
                }
                else {
                    if (part.partsPosNorm[i][j].mean(2)<distBorder)
                        part.partIds[i][j]=-1;
                }
            }
        }
    }
    /*if (groupSize<3){
        std::cout << "small number of points in group";
        part.print();
        getchar();
    }*/
    return true;
}

///find optimal transformation between normals
double ViewDependentPart::findOptimalTransformation(const ViewDependentPart& partA, const ViewDependentPart& partB, int distanceMetric, Mat34& transOpt){
    std::vector<std::pair<int, int>> pointCorrespondence = {{0,0}, {0,1}, {0,2}, {1,2}, {2,2}, {2,1}, {2,0}, {1,0}};
    double minDist = std::numeric_limits<double>::max();
    ViewDependentPart partC(partA);
    ViewDependentPart partD(partB);
    //if(removeSecondSurface(partC)||removeSecondSurface(partD)){
        /*std::cout << "partC przed\n";
        partA.print();
        std::cout << "partC po\n";
        partC.print();
        std::cout << "partD przed\n";
        partB.print();
        std::cout << "partD po\n";
        partD.print();
        getchar();*/
    //}
    Mat34 trans;
    for (size_t i=0;i<pointCorrespondence.size();i++){
        int pairsNo=0;
        std::vector<Vec3> setA; std::vector<Vec3> setB;
        size_t idx=i;
        for (size_t j=0;j<pointCorrespondence.size();j++){
            int coordA[2]={pointCorrespondence[j].first, pointCorrespondence[j].second};//partA is not rotated
            int coordB[2]={pointCorrespondence[idx%(pointCorrespondence.size())].first, pointCorrespondence[idx%(pointCorrespondence.size())].second};//partA is not rotated
            //std::cout << coordA[0] << " -> " << coordA[1] << "\n";
            //std::cout << coordB[0] << " -> " << coordB[1] << "\n";
            if ((partC.partIds[coordA[0]][coordA[1]]>=0)&&(partD.partIds[coordB[0]][coordB[1]]>=0)){
                pairsNo++;
                Vec3 posC = partC.partsPosNorm[coordA[0]][coordA[1]].mean.block<3,1>(0,0);
                Vec4 posc4(posC(0),posC(1),posC(2),1.0);
                posc4 = partC.offsets[coordA[0]][coordA[1]]*posc4;
                setA.push_back(posc4.block<3,1>(0,0));
                Vec3 posD = partD.partsPosNorm[coordB[0]][coordB[1]].mean.block<3,1>(0,0);
                Vec4 posd4(posD(0),posD(1),posD(2),1.0);
                posd4 = partD.offsets[coordB[0]][coordB[1]]*posd4;
                setB.push_back(posd4.block<3,1>(0,0));
            }
            idx++;
        }
        if ((partA.partIds[1][1]>=0)&&(partB.partIds[1][1]>=0)){
            pairsNo++;
            setA.push_back(Vec3(0,0,0));
            setB.push_back(Vec3(0,0,0));
        }
        if (pairsNo>=3){ // it's possible to find SE3 transformation
            Eigen::MatrixXd pointsA(3,pairsNo);
            Eigen::MatrixXd pointsB(3,pairsNo);
            for (int pairNo=0;pairNo<pairsNo;pairNo++){
                for (int col=0;col<3;col++){
                    pointsA(col,pairNo) = setA[pairNo](col);
                    pointsB(col,pairNo) = setB[pairNo](col);
                }
            }
            Eigen::Matrix4d trans1 = Eigen::umeyama(pointsA,pointsB,false);
            trans.matrix() = trans1;
            trans(2,3)=0;
            //trans = putslam::KabschEst::computeTrans(pointsA, pointsB);
            //std::cout << "transss\n" << trans.matrix() << "\n";

            ViewDependentPart partD(partB);// rotate part
            idx=i;
            for (size_t j=0;j<pointCorrespondence.size();j++){
                int coordA[2]={pointCorrespondence[j].first, pointCorrespondence[j].second};//partA is not rotated
                int coordB[2]={pointCorrespondence[idx%(pointCorrespondence.size())].first, pointCorrespondence[idx%(pointCorrespondence.size())].second};//partA is not rotated
                partD.partIds[coordA[0]][coordA[1]] = partB.partIds[coordB[0]][coordB[1]];
                partD.partsPosNorm[coordA[0]][coordA[1]] = partB.partsPosNorm[coordB[0]][coordB[1]];
                partD.offsets[coordA[0]][coordA[1]] = partB.offsets[coordB[0]][coordB[1]];
                idx++;
            }

            double error = computeError(partA, partD, trans, distanceMetric, 0.1);
            trans(2,3)=trans1(2,3);
            if (error<minDist){
                minDist=error;
                transOpt = trans;
            }
        }
    }
    return minDist;
}

/// create point cloud from second layer part
int ViewDependentPart::createPointsMatrix(const ViewDependentPart& part, const ViewDependentPart::Seq& vocabulary, int rotIndex, PointsSecondLayer& points){
    typedef std::vector<std::pair<int, int>> PointCorrespondence;
    PointCorrespondence pointCorrespondence = {{0,0}, {0,1}, {0,2}, {1,2}, {2,2}, {2,1}, {2,0}, {1,0}};
    std::vector<std::vector<PointCorrespondence>> ids(3,std::vector<PointCorrespondence>(3));
    int coordsNo=0;
    for (auto & coords : pointCorrespondence){
        for (int corr=0;corr<8;corr++){
            ids[coords.first][coords.second].push_back(pointCorrespondence[(coordsNo+corr)%(pointCorrespondence.size())]);
        }
        coordsNo++;
    }
    for (int corr=0;corr<8;corr++){
        ids[1][1].push_back(std::make_pair(1,1));
    }
    /*for (int id=0;id<8;id++){
        std::cout << id << ": \n";
        for (int i=0;i<3;i++){
            for (int j=0;j<3;j++){
                std::cout << "(" << ids[i][j][id].first << "," << ids[i][j][id].second << "), ";
            }
            std::cout << "\n";
        }
        std::cout << "\n";
    }
    getchar();*/
    int pointsNo=0;
    for (int rowId=0; rowId<3; rowId++){
        for (int partId=0; partId<3; partId++){
            if (part.partIds[rowId][partId]>=0){
                Vec3 posMiddle;
                if (rowId==1&&partId==1)
                    posMiddle = Vec3(0,0,0);
                else
                    posMiddle = part.partsPosNorm[rowId][partId].mean.block<3,1>(0,0);
                //std::cout << "pos middle " << posMiddle.transpose() << "\n";
                for (int i=0;i<3;i++){
                    for (int j=0;j<3;j++){
                        int newCoords[2]={ids[rowId][partId][rotIndex].first*3+ids[i][j][rotIndex].first, ids[rowId][partId][rotIndex].second*3+ids[i][j][rotIndex].second};
                        if (vocabulary[part.partIds[rowId][partId]].partIds[i][j]>=0){
                            Vec3 posElement;
                            if (i==1&&j==1)
                                posElement=Vec3(0,0,0);
                            else {
                                posElement=vocabulary[part.partIds[rowId][partId]].partsPosNorm[i][j].mean.block<3,1>(0,0);
                                //Mat34 offset = vocabulary[part.partIds[rowId][partId]].offsets[i][j];
                                Vec4 pos(posElement(0),posElement(1),posElement(2),1.0);
                                pos = part.offsets[rowId][partId]*pos;
                                posElement=pos.block<3,1>(0,0);
                            }
                            points[newCoords[0]][newCoords[1]].mean.block<3,1>(0,0)=posMiddle+posElement;
                            //Mat34 offset = vocabulary[part.partIds[rowId][partId]].offsets[i][j];
                            points[newCoords[0]][newCoords[1]].mean.block<3,1>(3,0) = part.offsets[rowId][partId].rotation()*vocabulary[part.partIds[rowId][partId]].partsPosNorm[i][j].mean.block<3,1>(3,0);
                            pointsNo++;
                            //std::cout << points[newCoords[0]][newCoords[1]].mean.transpose() << " feee\n";
                            //getchar();
                            if (points[newCoords[0]][newCoords[1]].mean.block<3,1>(3,0)(0)==0&&points[newCoords[0]][newCoords[1]].mean.block<3,1>(3,0)(1)==0&&points[newCoords[0]][newCoords[1]].mean.block<3,1>(3,0)(2)==0){
                                std::cout << "ij "<< i << " " << j << "\n";
                                vocabulary[part.partIds[rowId][partId]].print();
                                getchar();
                            }
                        }
                        else{
                            points[newCoords[0]][newCoords[1]].mean.block<3,1>(0,0)=Vec3(NAN,NAN,NAN);
                            points[newCoords[0]][newCoords[1]].mean.block<3,1>(3,0)=Vec3(0,0,1);
                            //std::cout << points[newCoords[0]][newCoords[1]].mean.transpose() << " fgggfgg\n";
                            //getchar();
                        }
                    }
                }
            }
            else {// fill matrix with nans
                for (int i=0;i<3;i++){
                    for (int j=0;j<3;j++){
                        int newCoords[2]={ids[rowId][partId][rotIndex].first*3+ids[i][j][rotIndex].first, ids[rowId][partId][rotIndex].second*3+ids[i][j][rotIndex].second};
                        points[newCoords[0]][newCoords[1]].mean.block<3,1>(0,0)=Vec3(NAN,NAN,NAN);
                        points[newCoords[0]][newCoords[1]].mean.block<3,1>(3,0)=Vec3(0,0,1);
                    }
                }
            }
        }
    }
    return pointsNo;
}

/// find SE3 transformation
bool ViewDependentPart::findSE3Transformation(const PointsSecondLayer& pointsA, const PointsSecondLayer& pointsB, Mat34& trans){
    std::vector<Vec3> setA; std::vector<Vec3> setB;
    int pairsNo=0;
    for (int i=0;i<9;i++){
        for (int j=0;j<9;j++){
            if (!std::isnan(pointsA[i][j].mean(0))&&!std::isnan(pointsB[i][j].mean(0))){
                setA.push_back(pointsA[i][j].mean.block<3,1>(0,0));
                setB.push_back(pointsB[i][j].mean.block<3,1>(0,0));
                pairsNo++;
            }
        }
    }
    if (pairsNo>3){
        Eigen::MatrixXd psA(3,pairsNo);
        Eigen::MatrixXd psB(3,pairsNo);
        for (int pairNo=0;pairNo<pairsNo;pairNo++){
            for (int col=0;col<3;col++){
                psA(col,pairNo) = setA[pairNo](col);
                psB(col,pairNo) = setB[pairNo](col);
            }
        }
        Eigen::Matrix4d transform = Eigen::umeyama(psA,psB,false);
        trans.matrix() = transform;
        return true;
    }
    else {
        trans = Mat34::Identity();
        return false;
    }
}

///find optimal transformation between normals
double ViewDependentPart::findOptimalTransformation(const ViewDependentPart& partA, const ViewDependentPart& partB, const ViewDependentPart::Seq& vocabulary, int distanceMetric, Mat34& transOpt){
    double minDist = std::numeric_limits<double>::max();
    for (size_t i=0;i<8;i++){
        PointsSecondLayer pointsA, pointsB; //9x9
        int pointsNoA = createPointsMatrix(partA, vocabulary, 0, pointsA);
        /*for (auto &row : pointsA){
            for (auto &el : row){
                std::cout <<  el.mean.block<6,1>(0,0).transpose() << ", ";
            }
            std::cout << "\n";
        }
        ViewDependentPart::Seq voc;
        for (int j=0;j<9;j++){
            ViewDependentPart p1;
            for (int k=0;k<3;k++){
                for (int l=0;l<3;l++){
                    p1.partIds[k][l]=1;
                    p1.partsPosNorm[k][l].mean.block<3,1>(0,0)=Vec3(k,l,0);
                }
            }
            p1.partsPosNorm[1][1].mean.block<3,1>(0,0)=Vec3(0,0,-double(j));
            voc.push_back(p1);
            //p1.print();
        }
        ViewDependentPart p1;
        int idx=0;
        for (int k=0;k<3;k++){
            for (int l=0;l<3;l++){
                p1.partIds[k][l]=idx;
                p1.partsPosNorm[k][l].mean.block<3,1>(0,0)=Vec3(0,0,idx);
                idx++;
            }
        }
        p1.partsPosNorm[1][1].mean.block<3,1>(0,0)=Vec3(-1,-2,-3);
        //p1.print();
        */
        //int pointsNoB = createPointsMatrix(p1, voc, (int)i, pointsB);
        int pointsNoB = createPointsMatrix(partB, vocabulary, (int)i, pointsB);
        //std::cout << "another part\n";
        /*for (auto &row : pointsB){
            for (auto &el : row){
                std::cout <<  el.mean.block<6,1>(0,0).transpose() << ", ";
            }
            std::cout << "\n";
        }*/
        //getchar();
        if (pointsNoA>4&&pointsNoB>4){
            Mat34 trans;
            if (findSE3Transformation(pointsA, pointsB,trans)){
                Mat34 trans1(trans);
                trans(2,3)=0;
                double error = computeError(pointsA, pointsB, trans, distanceMetric, 0.5);
                trans(2,3)=trans1(2,3);
                if (error<minDist){
                    minDist=error;
                    transOpt = trans;
                }
            }
        }
    }
    return minDist;
}

/// view invariant error for two parts with known SE3 transformation
double ViewDependentPart::computeError(const ViewDependentPart& partA, const ViewDependentPart& partB, const Mat34& transformation, int type, double coeff){
    double errorRot=0;
    double errorPos=0;
    //partA.print();
    //partB.print();
    //std::cout << "transformation \n" << transformation.matrix() << "\n";
    for (size_t i=0; i<partA.partIds.size();i++){
        for (size_t j=0; j<partA.partIds.size();j++){
            if ((partA.partIds[i][j]>=0)&&(partB.partIds[i][j]>=0)){
                if ((type == 1)||((type == 3))){
                    Eigen::Matrix<double, 4, 1> posPrim;
                    posPrim(0) = partA.partsPosNorm[i][j].mean(0); posPrim(1) = partA.partsPosNorm[i][j].mean(1); posPrim(2) = partA.partsPosNorm[i][j].mean(2); posPrim(3) = 1;
                    posPrim = transformation * posPrim;
                    errorPos+= sqrt((posPrim.block<3,1>(0,0)-partB.partsPosNorm[i][j].mean.block<3,1>(0,0)).transpose()*(posPrim.block<3,1>(0,0)-partB.partsPosNorm[i][j].mean.block<3,1>(0,0)));
                    //std::cout << "error pos " << errorPos << "\n";
                }
                if ((type == 2)||((type == 3))){
                    double dotprodA = (transformation.rotation()*partA.partsPosNorm[i][j].mean.block<3,1>(3,0)).adjoint()*partB.partsPosNorm[i][j].mean.block<3,1>(3,0);
                    if (dotprodA>1.0) dotprodA=1.0;
                    if (dotprodA<-1.0) dotprodA=-1.0;
                    double anglePart = acos(dotprodA);
                    double dotprodB = (transformation.rotation()*partA.partsPosNorm[1][1].mean.block<3,1>(3,0)).adjoint()*partB.partsPosNorm[1][1].mean.block<3,1>(3,0);
                    if (dotprodB>1.0) dotprodB=1.0;
                    if (dotprodB<-1.0) dotprodB=-1.0;
                    double angleCenter = acos(dotprodB);
                    errorRot+=fabs(anglePart-angleCenter);
                    //std::cout << "error tmp " << errorRot << "\n";
                }
            }
            else if ((partA.partIds[i][j]<0)&&(partB.partIds[i][j]<0)){
                errorPos+=0.0;
                errorRot+=0.0;
                //std::cout << "error bckgr rot " << errorRot << "\n";
                //std::cout << "error bckgr pos " << errorPos << "\n";
            }
            else if ((partA.partIds[i][j]<0&&partB.partIds[i][j]>=0)||(partA.partIds[i][j]>=0&&partB.partIds[i][j]<0)){
                errorPos+=0.1;
                errorRot+=0.1;
                //std::cout << "error bckgr diff rot " << errorRot << "\n";
                //std::cout << "error bckgr diff pos " << errorPos << "\n";
            }
        }
    }
    //std::cout << "distpos " << errorPos << " dist rot: " << errorRot << "\n";
    //std::cout << transformation.matrix() << "\n";
    //std::cout << "error final " << errorPos+coeff*errorRot << "\n";
    //getchar();
    return errorPos+coeff*errorRot;
}

/// view invariant error for two second layer parts with known SE3 transformation
double ViewDependentPart::computeError(const PointsSecondLayer& partA, const PointsSecondLayer& partB, const Mat34& transformation, int type, double coeff){
    double errorRot=0;
    double errorPos=0;
    int correspondencesNo=0;
    for (size_t i=0; i<partA.size();i++){
        for (size_t j=0; j<partA[i].size();j++){
            if (!isnan(partA[i][j].mean(0))&&!isnan(partB[i][j].mean(0))){
                correspondencesNo++;
                if ((type == 1)||((type == 3))){
                    Eigen::Matrix<double, 4, 1> posPrim;
                    posPrim(0) = partA[i][j].mean(0); posPrim(1) = partA[i][j].mean(1); posPrim(2) = partA[i][j].mean(2); posPrim(3) = 1;
                    posPrim = transformation * posPrim;
                    errorPos+= sqrt((posPrim.block<3,1>(0,0)-partB[i][j].mean.block<3,1>(0,0)).transpose()*(posPrim.block<3,1>(0,0)-partB[i][j].mean.block<3,1>(0,0)));
                    //std::cout << "error pos " << errorPos << "\n";
                }
                if ((type == 2)||((type == 3))){
                    Vec3 rotPrim;
                    rotPrim = transformation.rotation()*partA[i][j].mean.block<3,1>(3,0);
                    double dotprodA = rotPrim.adjoint()*partB[i][j].mean.block<3,1>(3,0);
                    if (dotprodA>1.0) dotprodA=1.0;
                    if (dotprodA<-1.0) dotprodA=-1.0;
                    double anglePart = acos(dotprodA);
                    /*double dotprodB = (transformation.rotation()*partA[4][4].mean.block<3,1>(3,0)).adjoint()*partB[4][4].mean.block<3,1>(3,0);
                    if (dotprodB>1.0) dotprodB=1.0;
                    if (dotprodB<-1.0) dotprodB=-1.0;
                    double angleCenter = acos(dotprodB);
                    errorRot+=fabs(anglePart-angleCenter);*/
                    errorRot+=fabs(anglePart);
                    if ((rotPrim(2)>0&&partB[i][j].mean(5)<0)||(rotPrim(2)<0&&partB[i][j].mean(5)>0))
                        errorRot+=10;
                    //std::cout << "error rot " << errorRot << "\n";
                }
            }
            else if (isnan(partA[i][j].mean(0))&&isnan(partB[i][j].mean(0))){
                errorPos+=0.0;
                errorRot+=0.0;
            }
            else if ((isnan(partA[i][j].mean(0))&&!isnan(partB[i][j].mean(0)))||(!isnan(partA[i][j].mean(0))&&isnan(partB[i][j].mean(0)))){
                errorPos+=0.1;
                errorRot+=0.1;
                //std::cout << "error pos1 " << errorPos << "\n";
                //std::cout << "error rot1 " << errorRot << "\n";
            }
        }
    }
    return (errorPos+coeff*errorRot)/double(correspondencesNo);
}

/// compute distance between view dependent parts (invariant version)
double ViewDependentPart::distanceInvariant(const ViewDependentPart& partA, const ViewDependentPart& partB, int distanceMetric, Mat34& estimatedTransform){
    /*if (partA.partIds==partB.partIds){//fast
        estimatedTransform=Mat34::Identity();
        return 0;
    }*/
    estimatedTransform=Mat34::Identity();
    double sum = 9;
    /*ViewDependentPart partC(partA);
    partC.partIds[0][0]=-1; partC.partIds[0][1]=-1; partC.partIds[0][2]=-1;
    partC.partIds[1][0]=1; partC.partIds[1][1]=1; partC.partIds[1][2]=1;
    partC.partIds[2][0]=1; partC.partIds[2][1]=1; partC.partIds[2][2]=1;

    partC.partsPosNorm[0][0].mean.block<3,1>(0,0)=Vec3(-0.05,-0.05, 0);
    partC.partsPosNorm[0][1].mean.block<3,1>(0,0)=Vec3(0.0,-0.05, 0);
    partC.partsPosNorm[0][2].mean.block<3,1>(0,0)=Vec3(0.05,-0.05, 0);

    partC.partsPosNorm[1][0].mean.block<3,1>(0,0)=Vec3(-0.05,0.0, 0);
    partC.partsPosNorm[1][1].mean.block<3,1>(0,0)=Vec3(0.0,0.0, 0);
    partC.partsPosNorm[1][2].mean.block<3,1>(0,0)=Vec3(0.05,0.0, 0);

    partC.partsPosNorm[2][0].mean.block<3,1>(0,0)=Vec3(-0.05,0.05, 0);
    partC.partsPosNorm[2][1].mean.block<3,1>(0,0)=Vec3(0.0,0.05, 0);
    partC.partsPosNorm[2][2].mean.block<3,1>(0,0)=Vec3(0.05,0.05, 0);

    ViewDependentPart partD(partA);
    partD.partIds[0][0]=1; partD.partIds[0][1]=1; partD.partIds[0][2]=-1;
    partD.partIds[1][0]=1; partD.partIds[1][1]=1; partD.partIds[1][2]=-1;
    partD.partIds[2][0]=1; partD.partIds[2][1]=1; partD.partIds[2][2]=-1;

    partD.partsPosNorm[0][0].mean.block<3,1>(0,0)=Vec3(-0.05,-0.05, 0);
    partD.partsPosNorm[0][1].mean.block<3,1>(0,0)=Vec3(0.0,-0.05, 0);
    partD.partsPosNorm[0][2].mean.block<3,1>(0,0)=Vec3(0.05,-0.05, 0);

    partD.partsPosNorm[1][0].mean.block<3,1>(0,0)=Vec3(-0.05,0.0, 0);
    partD.partsPosNorm[1][1].mean.block<3,1>(0,0)=Vec3(0.0,0.0, 0);
    partD.partsPosNorm[1][2].mean.block<3,1>(0,0)=Vec3(0.05,0.0, 0);

    partD.partsPosNorm[2][0].mean.block<3,1>(0,0)=Vec3(-0.05,0.05, 0);
    partD.partsPosNorm[2][1].mean.block<3,1>(0,0)=Vec3(0.0,0.05, 0);
    partD.partsPosNorm[2][2].mean.block<3,1>(0,0)=Vec3(0.05,0.05, 0);

    for (int i=0;i<3;i++){
        for (int j=0;j<3;j++){
            partC.partsPosNorm[i][j].mean.block<3,1>(3,0)=Vec3(0,0,1.0);
            partD.partsPosNorm[i][j].mean.block<3,1>(3,0)=Vec3(0,0,1.0);
        }
    }*/

    sum = findOptimalTransformation(partA, partB, distanceMetric, estimatedTransform);
    //std::cout << "sum " << sum << "n\n";
    //std::cout << "est trans\n" << estimatedTransform.matrix() << "\n";
    //getchar();
    return sum;
}

/// compute distance between view dependent parts (invariant version)
double ViewDependentPart::distanceInvariant(const ViewDependentPart& partA, const ViewDependentPart& partB, int distanceMetric, const ViewDependentPart::Seq& vocabulary, Mat34& estimatedTransform){
    estimatedTransform=Mat34::Identity();
    double sum = 9;
    sum = findOptimalTransformation(partA, partB, vocabulary, distanceMetric, estimatedTransform);
    if (estimatedTransform(2,2)<0){
        sum+=0.01;
    }
    //std::cout << "sum : " << sum << "\n";
    //std::cout << "est trans\n" << estimatedTransform.matrix() << "\n";
    //getchar();
    return sum;
}

/// compute distance between view dependent parts
double ViewDependentPart::distance(const ViewDependentPart& partA, const ViewDependentPart& partB, const Filter::Seq& filters, int distanceMetric){
    if (partA.partIds==partB.partIds)//fast
        return 0;
    double sum=0;
    for (size_t i=0; i<partA.partIds.size();i++){
        for (size_t j=0; j<partA.partIds.size();j++){
            double dotprod=0;
            if ((partA.partIds[i][j]<0)&&(partB.partIds[i][j]<0)){
                dotprod=0;
                sum+=dotprod;
            }
            else if ((partA.partIds[i][j]<0)||(partB.partIds[i][j]<0)){
                dotprod=1;
                sum+=dotprod;
            }
            else{
                dotprod=Filter::distance(filters[partA.partIds[i][j]], filters[partB.partIds[i][j]]);
                if (partA.partIds[1][1]>=0&&partB.partIds[1][1]>=0) {
                    dotprod=0.01+Filter::distance(filters[partA.partIds[1][1]], filters[partB.partIds[1][1]]);
                }
                if (i!=1&&j!=1){
                    sum+=dotprod;
                }
                // compute mahalanobis distance
                if (i!=1&&j!=1){
                    if (distanceMetric==0){//dotproduct
                        sum+=dotprod;
                    }
                    else {
                        double mahalanobisDist;
                        double euclDist;
                        if (partA.gaussians[i][j].covariance(0,0)>0&&partB.gaussians[i][j].covariance(0,0)>0){
                            // compute Mahalanobis distance
                            mahalanobisDist = sqrt((partA.gaussians[i][j].mean-partB.gaussians[i][j].mean).transpose()*((partA.gaussians[i][j].covariance+partB.gaussians[i][j].covariance)/2.0).inverse()*(partA.gaussians[i][j].mean-partB.gaussians[i][j].mean));
                            if (distanceMetric==2){//mahalanobis
                                sum+=dotprod*exp(mahalanobisDist);
                            }
                        }
                        else {// compute Euclidean distance
                            euclDist = sqrt((partA.gaussians[i][j].mean-partB.gaussians[i][j].mean).transpose()*(partA.gaussians[i][j].mean-partB.gaussians[i][j].mean));
                            if (distanceMetric==1||distanceMetric==2){// Euclidean or Mahalanobis
                                sum+=dotprod*exp(euclDist);
                            }
                        }
                    }
                }
                else
                    sum+=dotprod;
            }
        }
    }
    return sum;
}

/// compute distance between view dependent parts
double ViewDependentPart::distance(const ViewDependentPart& partA, const ViewDependentPart& partB, const ViewDependentPart::Seq& layer2vocabulary, const Filter::Seq& filters, int distanceMetric){
    if (partA.partIds==partB.partIds)//fast
        return 0;
    double sum=0;
    for (size_t i=0; i<partA.partIds.size();i++)
        for (size_t j=0; j<partA.partIds.size();j++)
            if (partA.layerId==3){
                if ((partA.partIds[i][j]<0)&&(partB.partIds[i][j]<0)){
                    sum+=0;
                }
                else if ((partA.partIds[i][j]<0)||(partB.partIds[i][j]<0)){
                    sum+=9;
                }
                else{
                    sum+=distance(layer2vocabulary[partA.partIds[i][j]], layer2vocabulary[partB.partIds[i][j]], filters, distanceMetric);
                }
            }
    return sum;
}

///get normal vector related to that part
void ViewDependentPart::getNormal(Vec3& normal, const ViewDependentPart::Seq& layer2vocabulary, const Filter::Seq& filters) const{
    if (layerId==2){
        normal = filters[partIds[1][1]].normal;
    }
    else if (layerId==3){
        normal = filters[layer2vocabulary[partIds[1][1]].partIds[1][1]].normal;
    }
}

// Insertion operator
std::ostream& operator<<(std::ostream& os, const ViewDependentPart& part){
    os << part.id << " " << static_cast<unsigned int>(part.type) << " " << part.realisationId << " " << part.layerId << " " << part.location << " " << part.offset;
    for (int i=0;i<3;i++){
        for (int j=0;j<3;j++){
            os << part.partIds[i][j] << " ";
        }
    }
    for (int i=0;i<3;i++){
        for (int j=0;j<3;j++){
            os << part.gaussians[i][j];
        }
    }
    for (int i=0;i<3;i++){
        for (int j=0;j<3;j++){
            os << part.partsPosNorm[i][j];
        }
    }
    for (int i=0;i<3;i++){
        for (int j=0;j<3;j++){
            os << part.offsets[i][j];
        }
    }
    os << part.group.size() << "\n";
    for (size_t i=0;i<part.group.size();i++){
        os << part.group[i];
    }
    os << "\n";
    if (part.secondVDPart.size()>0){
        os << 1 << " ";
        os << part.secondVDPart[0];
    }
    else{
        os << 0 << " ";
    }
    os << "\n";
    return os;
}

// Extraction operator
std::istream& operator>>(std::istream& is, ViewDependentPart& part){
    unsigned int type;
    is >> part.id >> type >> part.realisationId >> part.layerId >> part.location >> part.offset;
    part.type = static_cast<Part::Type>(type);
    for (int i=0;i<3;i++){
        for (int j=0;j<3;j++){
            is >> part.partIds[i][j];
        }
    }
    for (int i=0;i<3;i++){
        for (int j=0;j<3;j++){
            is >> part.gaussians[i][j];
        }
    }
    for (int i=0;i<3;i++){
        for (int j=0;j<3;j++){
            is >> part.partsPosNorm[i][j];
        }
    }
    for (int i=0;i<3;i++){
        for (int j=0;j<3;j++){
            is >> part.offsets[i][j];
        }
    }
    int groupSize;
    is >> groupSize;
    part.group.resize(groupSize);
    for (int i=0;i<groupSize;i++){
        is >> part.group[i];
    }
    int secondVD;
    is >> secondVD;
    if (secondVD){
        ViewDependentPart secondP;
        is >> secondP;
        part.secondVDPart.push_back(secondP);
    }
    return is;
}

/// Insertion operator
std::ostream& operator<<(std::ostream& os, const ViewIndependentPart::Part3D& part){
    os << part.id << " ";
    os << part.pose << "\n";
    return os;
}

/// Extraction operator
std::istream& operator>>(std::istream& is, ViewIndependentPart::Part3D& part){
    is >> part.id;
    is >> part.pose;
    return is;
}

/// Insertion operator
std::ostream& operator<<(std::ostream& os, const ViewIndependentPart& part){
    os << part.id << " " << part.layerId << " ";
    // part Ids
    for (int i=0;i<3;i++){
        for (int j=0;j<3;j++){
            for (int k=0;k<3;k++){
				if (std::isnan(double(part.partIds[i][j][k])))
                    os << -2 << " ";
                else
                    os << part.partIds[i][j][k] << " ";
            }
        }
    }
    os << part.incomingIds.size() << " ";
    for (const auto &id : part.incomingIds){
        os << id << " ";
    }
    // gaussians
    for (int i=0;i<3;i++){
        for (int j=0;j<3;j++){
            for (int k=0;k<3;k++){
                os << part.gaussians[i][j][k];
            }
        }
    }
    os << part.group.size() << "\n";
    for (size_t i=0;i<part.group.size();i++){
        os << part.group[i];
    }
    os << part.offset;
    os << part.pose;
    for (int i=0;i<3;i++){
        for (int j=0;j<3;j++){
            for (int k=0;k<3;k++){
                os << part.neighbourPoses[i][j][k];
            }
        }
    }
    os << part.parts.size() << " ";
    for (auto& p3d : part.parts){
        os << p3d;
    }
    os << "\n";
    return os;
}

/// Extraction operator
std::istream& operator>>(std::istream& is, ViewIndependentPart& part){
    is >> part.id >> part.layerId;
    for (int i=0;i<3;i++){
        for (int j=0;j<3;j++){
            for (int k=0;k<3;k++){
                is >> part.partIds[i][j][k];
            }
        }
    }
    int incomingIdsSize;
    is >> incomingIdsSize;
    for (int i=0;i<incomingIdsSize;i++){
        int id;        is >> id;
        part.incomingIds.insert(id);
    }
    for (int i=0;i<3;i++){
        for (int j=0;j<3;j++){
            for (int k=0;k<3;k++){
                is >> part.gaussians[i][j][k];
            }
        }
    }
    int groupSize;
    is >> groupSize;
    part.group.resize(groupSize);
    for (int i=0;i<groupSize;i++){
        is >> part.group[i];
    }
    is >> part.offset;
    is >> part.pose;
    for (int i=0;i<3;i++){
        for (int j=0;j<3;j++){
            for (int k=0;k<3;k++){
                is >> part.neighbourPoses[i][j][k];
            }
        }
    }
    int partsSize;
    is >> partsSize;
    part.parts.resize(partsSize);
    for (int i=0;i<partsSize;i++){
        is >> part.parts[i];
    }
    return is;
}

}
