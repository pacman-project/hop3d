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
    std::cout << "\nParts in group: ";
    for (auto it = group.begin();it!=group.end();it++)
        std::cout << it->id << ", ";
    std::cout << "\n";
    /*std::cout << "Gaussians:\n";
    for (size_t i=0; i<gaussians.size();i++){
        for (size_t j=0; j<gaussians.size();j++){
            std::cout << "mean(" << i << ", " << j << "): ";
            std::cout << gaussians[i][j].mean.transpose() << "\n";
            //std::cout << "covariance(" << i << ", " << j << "):\n";
            //std::cout << gaussians[i][j].covariance << "\n";
        }
    }*/
    std::cout << "Pos norm SE3:\n";
    for (size_t i=0; i<partsPosNorm.size();i++){
        for (size_t j=0; j<partsPosNorm.size();j++){
            std::cout << "mean(" << i << ", " << j << "): ";
            std::cout << partsPosNorm[i][j].mean.transpose() << "\n";
            //std::cout << "covariance(" << i << ", " << j << "):\n";
            //std::cout << gaussians[i][j].covariance << "\n";
        }
    }
}

/// check if part is background
bool ViewDependentPart::isBackground(void) const{
    for (size_t i=0; i<partIds.size();i++){
        for (size_t j=0; j<partIds[0].size();j++){
            if (partIds[i][j]!=-1)
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
                if (partIds[i][j][k]!=-1){
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
                if (partIds[i][j][k]!=-1){
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
                if(partA.partIds[i][j][k]!=-1){
                    normA+=partA.neighbourPoses[i][j][k].matrix().block<3,1>(0,2);
      //              meanA+=partA.neighbourPoses[i][j][k].matrix().block<3,1>(0,3);
                    partASeq.push_back(std::make_pair(partA.neighbourPoses[i][j][k],partA.partIds[i][j][k]));
                    meanPosA+=partA.neighbourPoses[i][j][k].matrix().block<3,1>(0,3);
                }
                if(partB.partIds[i][j][k]!=-1){
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
                if (partA.partIds[i][j][k]!=-1)
                    pA= vocabulary2[partA.partIds[i][j][k]];
                if (partB.partIds[i][j][k]!=-1)
                    pB= vocabulary2[partB.partIds[i][j][k]];
                if ((partA.partIds[i][j][k]!=-1)&&(partA.partIds[i][j][k]!=-1)){
                    for (int l=0;l<3;l++){
                        for (int m=0;m<3;m++){
                            for (int n=0;n<3;n++){
                                ViewIndependentPart pAA, pBB;
                                if (pA.partIds[i][j][k]!=-1)
                                    pAA= vocabulary1[pA.partIds[i][j][k]];
                                if (pB.partIds[i][j][k]!=-1)
                                    pBB= vocabulary1[pB.partIds[i][j][k]];
                                Vec3 nA(0,0,0), nB(0,0,0);
                                Vec3 meanA(0,0,0); Vec3 meanB(0,0,0);
                                getPoints(pAA, pBB, partASeq, partBSeq, meanA, nA, meanB, nB);
                                if (pA.partIds[i][j][k]!=-1){
                                    meanPosA+=meanA; normA+=nA;
                                }
                                if (pB.partIds[i][j][k]!=-1){
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
                if (partA.partIds[i][j][k]!=-1)
                    pA= vocabulary[partA.partIds[i][j][k]];
                if (partB.partIds[i][j][k]!=-1)
                    pB= vocabulary[partB.partIds[i][j][k]];
                Vec3 nA(0,0,0), nB(0,0,0);
                Vec3 meanA(0,0,0); Vec3 meanB(0,0,0);
                getPoints(pA, pB, partASeq, partBSeq, meanA, nA, meanB, nB);
                if (partA.partIds[i][j][k]!=-1){
                    meanPosA+=meanA; normA+=nA;
                }
                if (partB.partIds[i][j][k]!=-1){
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
bool ViewDependentPart::removeSecondSurface(ViewDependentPart& part) {
    bool has2surfs(false);
    //part.print();
    double distThreshold = 0.015;
    std::vector<double> depth;
    for (int i=0;i<3;i++){//detect two surfaces
        for (int j=0;j<3;j++){
            if (part.partIds[i][j]!=-1){
                if (i==1&&j==1)
                    depth.push_back(0);
                else
                    depth.push_back(part.partsPosNorm[i][j].mean(2));
            }
        }
    }
    std::sort(depth.begin(),depth.end());
    double distBorder;
    size_t groupSize;
    bool removeBack=false;
    for (size_t i=0;i<depth.size()-1;i++){
        if (depth[i+1]-depth[i]>distThreshold){
            if (i+1>=depth.size()-i){
                groupSize = i+1;
                removeBack=true;
            }
            else
                groupSize = depth.size()-i;
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
    if (groupSize<3){
        std::cout << "small number of points in group";
        getchar();
    }
    return true;
}

///find optimal transformation between normals
double ViewDependentPart::findOptimalTransformation(const ViewDependentPart& partA, const ViewDependentPart& partB, int distanceMetric, Mat34& transOpt){
    std::vector<std::pair<int, int>> pointCorrespondence = {{0,0}, {0,1}, {0,2}, {1,2}, {2,2}, {2,1}, {2,0}, {1,0}};
    double minDist = std::numeric_limits<double>::max();
    ViewDependentPart partC(partA);
    ViewDependentPart partD(partB);
    if(removeSecondSurface(partC)||removeSecondSurface(partD)){
        /*std::cout << "partC przed\n";
        partA.print();
        std::cout << "partC po\n";
        partC.print();
        std::cout << "partD przed\n";
        partB.print();
        std::cout << "partD po\n";
        partD.print();
        getchar();*/
    }
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
            if ((partC.partIds[coordA[0]][coordA[1]]!=-1)&&(partD.partIds[coordB[0]][coordB[1]]!=-1)){
                pairsNo++;
                setA.push_back(partC.partsPosNorm[coordA[0]][coordA[1]].mean.block<3,1>(0,0));
                setB.push_back(partD.partsPosNorm[coordB[0]][coordB[1]].mean.block<3,1>(0,0));
            }
            idx++;
        }
        if ((partA.partIds[1][1]!=-1)&&(partB.partIds[1][1]!=-1)){
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
            trans.matrix()=trans1;
            trans(2,3)=0;
            //trans = putslam::KabschEst::computeTrans(pointsA, pointsB);
            //std::cout << "transss\n" << trans.matrix() << "\n";
            double error = computeError(partA, partB, trans, distanceMetric, 0.05);
            trans(2,3)=trans1(2,3);
            if (error<minDist){
                minDist=error;
                transOpt = trans;
            }
        }
    }
    return minDist;
}

/// view invariant error for two parts with known SE3 transformation
double ViewDependentPart::computeError(const ViewDependentPart& partA, const ViewDependentPart& partB, const Mat34& transformation, int type, double coeff){
    double errorRot=0;
    double errorPos=0;
    for (size_t i=0; i<partA.partIds.size();i++){
        for (size_t j=0; j<partA.partIds.size();j++){
            if ((partA.partIds[i][j]!=-1)&&(partB.partIds[i][j]!=-1)){
                if ((type == 1)||((type == 3))){
                    Eigen::Matrix<double, 4, 1> posPrim;
                    posPrim(0) = partA.partsPosNorm[i][j].mean(0); posPrim(1) = partA.partsPosNorm[i][j].mean(1); posPrim(2) = partA.partsPosNorm[i][j].mean(2); posPrim(3) = 1;
                    posPrim = transformation * posPrim;
                    errorPos+= sqrt((posPrim.block<3,1>(0,0)-partB.partsPosNorm[i][j].mean.block<3,1>(0,0)).transpose()*(posPrim.block<3,1>(0,0)-partB.partsPosNorm[i][j].mean.block<3,1>(0,0)));
                }
                if ((type == 2)||((type == 3))){
                    double anglePart = acos((transformation.rotation()*partA.partsPosNorm[i][j].mean.block<3,1>(3,0)).adjoint()*partB.partsPosNorm[i][j].mean.block<3,1>(3,0));
                    double angleCenter = acos((transformation.rotation()*partA.partsPosNorm[1][1].mean.block<3,1>(3,0)).adjoint()*partB.partsPosNorm[1][1].mean.block<3,1>(3,0));
                    errorRot+=fabs(anglePart-angleCenter);
                    //std::cout << "error tmp " << errorTmp << "\n";
                }
            }
            else if ((partA.partIds[i][j]==-1)||(partB.partIds[i][j]==-1)){
                errorPos+=1.0;
                errorRot+=1.0;
            }
        }
    }
    //std::cout << "distpos " << errorPos << " dist rot: " << errorRot << "\n";
    //getchar();
    /*std::cout << transformation.matrix() << "\n";
    std::cout << "error final " << errorTmp << "\n";
    getchar();*/
    return errorPos+coeff*errorRot;
}

/// compute distance between view dependent parts (invariant version)
double ViewDependentPart::distanceInvariant(const ViewDependentPart& partA, const ViewDependentPart& partB, int distanceMetric, Mat34& estimatedTransform){
    if (partA.partIds==partB.partIds){//fast
        estimatedTransform=Mat34::Identity();
        return 0;
    }
    estimatedTransform=Mat34::Identity();
    double sum = 9;
    sum = findOptimalTransformation(partA, partB, distanceMetric, estimatedTransform);
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
            if ((partA.partIds[i][j]==-1)&&(partB.partIds[i][j]==-1)){
                dotprod=0;
                sum+=dotprod;
            }
            else if ((partA.partIds[i][j]==-1)||(partB.partIds[i][j]==-1)){
                dotprod=1;
                sum+=dotprod;
            }
            else{
                dotprod=Filter::distance(filters[partA.partIds[i][j]], filters[partB.partIds[i][j]]);
                if (partA.partIds[1][1]!=-1&&partB.partIds[1][1]!=-1) {
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
                if ((partA.partIds[i][j]==-1)&&(partB.partIds[i][j]==-1)){
                    sum+=0;
                }
                else if ((partA.partIds[i][j]==-1)||(partB.partIds[i][j]==-1)){
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
