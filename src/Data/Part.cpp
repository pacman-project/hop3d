#include "hop3d/Data/Part.h"
#include "hop3d/Data/Vocabulary.h"
#include "hop3d/ImageFilter/normalImageFilter.h"
#include "hop3d/Data/hop3dmath.h"
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
    /*std::cout << "\nParts in group: ";
    for (auto it = group.begin();it!=group.end();it++){
        it->print();
        std::cout << it->id << ", ";
    }
    std::cout << "\n";*/
}

/// Is complete
bool ViewDependentPart::isComplete(void) const{
    for (size_t i=0;i<partIds.size();++i)
        for (size_t j=0;j<partIds[i].size();++j)
            if (partIds[i][j]==-3)
                return false;
    return true;
}

/// fitness subpart
double ViewDependentPart::subpartFitness(const Vec6& point, size_t row, size_t col, const Mat34& transform) const{
    /*std::cout << "point " << point.transpose() << "\n";
    std::cout << "point2 " << partsPosNorm[row][col].mean.transpose() << "\n";
    std::cout << "res " << (1/sqrt(pow(2*M_PI,6)*partsPosNorm[row][col].mean.norm()))*exp(-0.5*(point-partsPosNorm[row][col].mean).transpose()*partsPosNorm[row][col].covariance.inverse()*(point-partsPosNorm[row][col].mean)) << "\n";
    std::cout << "(1/sqrt(pow(2*M_PI,6)*partsPosNorm[row][col].mean.norm())) " << (1/sqrt(pow(2*M_PI,6)*partsPosNorm[row][col].mean.norm())) << "\n";
    std::cout << "partsPosNorm[row][col].covariance " << partsPosNorm[row][col].covariance.matrix() << "\n";
    std::cout << "partsPosNorm[row][col].covariance.inverse() " << partsPosNorm[row][col].covariance.inverse() << "\n";
    std::cout << "(-0.5*(point-partsPosNorm[row][col].mean).transpose()*partsPosNorm[row][col].covariance.inverse()*(point-partsPosNorm[row][col].mean)) " << (-0.5*(point-partsPosNorm[row][col].mean).transpose()*partsPosNorm[row][col].covariance.inverse()*(point-partsPosNorm[row][col].mean)) << "\n";
    std::cout << "exp(-0.5*(point-partsPosNorm[row][col].mean).transpose()*partsPosNorm[row][col].covariance.inverse()*(point-partsPosNorm[row][col].mean)) " << exp(-0.5*(point-partsPosNorm[row][col].mean).transpose()*partsPosNorm[row][col].covariance.inverse()*(point-partsPosNorm[row][col].mean)) << "\n";
    //getchar();*/
    //std::cout << "point subart " << partsPosNorm[row][col].mean.transpose() << "\n";
    //std::cout << "point before transformation " << point.transpose() << "\n";
    Vec6 pointTransformed(point);
    hop3d::transform(pointTransformed,transform);
    Vec6 meanOffset(partsPosNorm[row][col].mean);
    hop3d::transform(meanOffset,offsets[row][col]);
    /*std::cout << "point transformed " << pointTransformed.transpose() << "\n";
    std::cout << "dist bef trans " << (point.block<3,1>(0,0)-partsPosNorm[row][col].mean.block<3,1>(0,0)).norm() << "\n";
    std::cout << "dist af trans " << (point.block<3,1>(0,0)-pointTransformed.block<3,1>(0,0)).norm() << "\n";
    std::cout << "dist norm bef trans " << (point.block<3,1>(0,0)-partsPosNorm[row][col].mean.block<3,1>(3,0)).norm() << "\n";
    std::cout << "dist norm af trans " << (point.block<3,1>(0,0)-pointTransformed.block<3,1>(3,0)).norm() << "\n";
    */
    double res = (1/sqrt(pow(2*M_PI,6)*meanOffset.norm()))*exp(-0.5*(pointTransformed-meanOffset).transpose()*partsPosNorm[row][col].covariance.inverse()*(pointTransformed-meanOffset));
    //double res = (1/sqrt(pow(2*M_PI,6)*partsPosNorm[row][col].mean.norm()))*exp(-0.5*(pointTransformed-partsPosNorm[row][col].mean).transpose()*partsPosNorm[row][col].covariance.inverse()*(pointTransformed-partsPosNorm[row][col].mean));
    if (std::isnan(res)||std::isinf(res))
        return 0;
    else
        return res;
}

/// restore occluded subparts
double ViewDependentPart::distanceStats(const ViewDependentPart& part, const ViewDependentPart::Seq& layer1, const ViewDependentPart::Seq& layer2, int& rotId, Mat34& estimatedTransform) const{
    std::vector<std::pair<int, int>> pointCorrespondence = {{0,0}, {0,1}, {0,2}, {1,2}, {2,2}, {2,1}, {2,0}, {1,0}};
    double fit=0;
    /*std::cout << "this: \n";
    this->print();
    std::cout << "part: \n";*/
    if (part.layerId==2){//first layer is slightly different
        findOptimalTransformation(part,*this, 3, estimatedTransform, rotId);
        int idx=rotId;
        for (size_t i=0;i<pointCorrespondence.size()+1;i++){
            int coordA[2]; int coordB[2];
            if (i==pointCorrespondence.size()){
                coordA[0]=1; coordA[1]=1;
                coordB[0]=1; coordB[1]=1;
            }
            else{
                coordA[0]=pointCorrespondence[i].first; coordA[1]=pointCorrespondence[i].second;//partA is not rotated
                coordB[0]=pointCorrespondence[idx%(pointCorrespondence.size())].first; coordB[1]=pointCorrespondence[idx%(pointCorrespondence.size())].second;//partA is not rotated
            }
            //std::cout << coordA[0] << ", " << coordA[1] << "->" << coordB[0] << ", " << coordB[1] << "\n";
            //std::cout << "0part.partIds[i][j], thisids: " << part.partIds[coordA[0]][coordA[1]] << ", " << partIds[coordB[0]][coordB[1]] << "\n";
            if (part.partIds[coordA[0]][coordA[1]]==-3){
                if (partIds[coordB[0]][coordB[1]]>=0)
                    fit+=1;
                else
                    fit+=0;
                //std::cout << "0fit1+0\n";
            }
            if ((part.partIds[coordA[0]][coordA[1]]==-1||part.partIds[coordA[0]][coordA[1]]==-2)&&(partIds[coordB[0]][coordB[1]]==-1||partIds[coordB[0]][coordB[1]]==-2)){
                //std::cout << "0fit2+1\n";
                //fit+=0.0000001;
                auto it = subpartsProb[coordB[0]][coordB[1]].find(-1);
                if (it == subpartsProb[coordB[0]][coordB[1]].end()){
                    fit+=0;
                }
                else {
                    fit+=subpartsProb[coordB[0]][coordB[1]].at(-1);
                }
                it = subpartsProb[coordB[0]][coordB[1]].find(-2);
                if (it == subpartsProb[coordB[0]][coordB[1]].end()){
                    fit+=0;
                }
                else {
                    fit+=subpartsProb[coordB[0]][coordB[1]].at(-2);
                }
            }
            if (((part.partIds[coordA[0]][coordA[1]]==-1||part.partIds[coordA[0]][coordA[1]]==-2)&&partIds[coordB[0]][coordB[1]]>=0)||((partIds[coordB[0]][coordB[1]]==-1||partIds[coordB[0]][coordB[1]]==-2)&&part.partIds[coordA[0]][coordA[1]]>=0)){
                //std::cout << "0fit3+0\n";
                fit+=0;
            }
            if (part.partIds[coordA[0]][coordA[1]]>=0&&partIds[coordB[0]][coordB[1]]>=0){
            //if (partIds[i][j]>0&&part.partIds[i][j]>0){
                //std::cout << ", subpartFitness(part.partsPosNorm[i][j].mean, i,j) " << subpartFitness(part.partsPosNorm[coordA[0]][coordA[1]].mean, coordB[0],coordB[1], estimatedTransform) << "\n";
                fit+=1+subpartFitness(part.partsPosNorm[coordA[0]][coordA[1]].mean, coordB[0],coordB[1], estimatedTransform);
            }
            //getchar();
            idx++;
        }
        /*size_t idx=rotId;
        for (size_t i=0;i<pointCorrespondence.size()+1;i++){
            findOptimalTransformation()
            int coordA[2]; int coordB[2];
            if (i==pointCorrespondence.size()){
                coordA[0]=1; coordA[1]=1;
                coordB[0]=1; coordB[1]=1;
            }
            else{
                coordA[0]=pointCorrespondence[i].first; coordA[1]=pointCorrespondence[i].second;//partA is not rotated
                coordB[0]=pointCorrespondence[idx%(pointCorrespondence.size())].first; coordB[1]=pointCorrespondence[idx%(pointCorrespondence.size())].second;//partA is not rotated
            }
            //std::cout << "0part.partIds[i][j], thisids: " << part.partIds[i][j] << ", " << partIds[i][j] << "\n";
            if (part.partIds[coordA[0]][coordA[1]]==-3){
                fit+=0;
                //std::cout << "0fit1+0\n";
            }
            if ((part.partIds[coordA[0]][coordA[1]]==-1||part.partIds[coordA[0]][coordA[1]]==-2)&&(partIds[coordB[0]][coordB[1]]==-1||partIds[coordB[0]][coordB[1]]==-2)){
                //std::cout << "0fit2+1\n";
                fit+=1;
            }
            else if ((part.partIds[coordA[0]][coordA[1]]==-1||part.partIds[coordA[0]][coordA[1]]==-2)||(partIds[coordB[0]][coordB[1]]==-1||partIds[coordB[0]][coordB[1]]==-2)){
                //std::cout << "0fit3+0\n";
                fit+=0;
            }
            else{
            //if (partIds[i][j]>0&&part.partIds[i][j]>0){
                //std::cout << ", subpartFitness(part.partsPosNorm[i][j].mean, i,j) " << subpartFitness(part.partsPosNorm[i][j].mean, i,j) << "\n";
                fit+=subpartFitness(part.partsPosNorm[coordA[0]][coordA[1]].mean, coordB[0],coordB[1]);
            }
            //getchar();
            idx++;
        }*/
    }
    else{
        findOptimalTransformation(part,*this, layer1, 3, estimatedTransform, rotId);
        int idx=rotId;
        for (size_t i=0;i<pointCorrespondence.size()+1;i++){
            int coordA[2]; int coordB[2];
            if (i==pointCorrespondence.size()){
                coordA[0]=1; coordA[1]=1;
                coordB[0]=1; coordB[1]=1;
            }
            else{
                coordA[0]=pointCorrespondence[i].first; coordA[1]=pointCorrespondence[i].second;//partA is not rotated
                coordB[0]=pointCorrespondence[idx%(pointCorrespondence.size())].first; coordB[1]=pointCorrespondence[idx%(pointCorrespondence.size())].second;//partA is not rotated
            }
            //std::cout << coordA[0] << ", " << coordA[1] << "->" << coordB[0] << ", " << coordB[1] << "\n";
            //std::cout << "0part.partIds[i][j], thisids: " << part.partIds[coordA[0]][coordA[1]] << ", " << partIds[coordB[0]][coordB[1]] << "\n";
            if (part.partIds[coordA[0]][coordA[1]]==-3){
                if (partIds[coordB[0]][coordB[1]]>=0)
                    fit+=1;
                else
                    fit+=0;
                //std::cout << "0fit1+0\n";
            }
            if ((part.partIds[coordA[0]][coordA[1]]==-1||part.partIds[coordA[0]][coordA[1]]==-2)&&(partIds[coordB[0]][coordB[1]]==-1||partIds[coordB[0]][coordB[1]]==-2)){
                //std::cout << "0fit2+1\n";
                auto it = subpartsProb[coordB[0]][coordB[1]].find(part.partIds[coordA[0]][coordA[1]]);
                if (it == subpartsProb[coordB[0]][coordB[1]].end()){
                    //std::cout << "fit+0\n";
                    fit+=0;
                }
                else {
                    //std::cout << "subpartsProb[i][j].at(part.partIds[i][j]): " << subpartsProb[i][j].at(part.partIds[i][j]) << "\n";
                    fit+=subpartsProb[coordB[0]][coordB[1]].at(part.partIds[coordA[0]][coordA[1]]);
                }
            }
            if (((part.partIds[coordA[0]][coordA[1]]==-1||part.partIds[coordA[0]][coordA[1]]==-2)&&partIds[coordB[0]][coordB[1]]>=0)||((partIds[coordB[0]][coordB[1]]==-1||partIds[coordB[0]][coordB[1]]==-2)&&part.partIds[coordA[0]][coordA[1]]>=0)){
                //std::cout << "0fit3+0\n";
                fit+=0;
            }
            if (part.partIds[coordA[0]][coordA[1]]>=0&&partIds[coordB[0]][coordB[1]]>=0){
                auto it = subpartsProb[coordB[0]][coordB[1]].find(part.partIds[coordA[0]][coordA[1]]);
                if (it == subpartsProb[coordB[0]][coordB[1]].end()){
                    //std::cout << "fit1+0\n";
                    fit+=0;
                }
                else{
                    //std::cout << "subpartsProb[i][j].at(part.partIds[i][j]) " << subpartsProb[i][j].at(part.partIds[i][j]) << ", subpartFitness(part.partsPosNorm[i][j].mean, i,j) " << subpartFitness(part.partsPosNorm[i][j].mean, i,j) << "\n";
                    //std::cout << "fit+ " << subpartsProb[i][j].at(part.partIds[i][j])*subpartFitness(part.partsPosNorm[i][j].mean, i,j) << "\n";
                    Vec6 meanOffset(part.partsPosNorm[coordA[0]][coordA[1]].mean);
                    hop3d::transform(meanOffset,part.offsets[coordA[0]][coordA[1]]);
                    fit+=1+subpartsProb[coordB[0]][coordB[1]].at(part.partIds[coordA[0]][coordA[1]])*subpartFitness(meanOffset, coordB[0],coordB[1], estimatedTransform);
                    //fit+=1+subpartsProb[coordB[0]][coordB[1]].at(part.partIds[coordA[0]][coordA[1]])*subpartFitness(part.partsPosNorm[coordA[0]][coordA[1]].mean, coordB[0],coordB[1], estimatedTransform);
                }
            }
            //getchar();
            idx++;
        }
        /*for (size_t i=0;i<part.partIds.size();++i){
            for (size_t j=0;j<part.partIds[i].size();++j){
                //std::cout << "part.partIds[i][j], thisids: " << part.partIds[i][j] << ", " << partIds[i][j] << "\n";
                if ((part.partIds[i][j]==-1||part.partIds[i][j]==-2)&&(partIds[i][j]==-1||partIds[i][j]==-2)){
                    auto it = subpartsProb[i][j].find(part.partIds[i][j]);
                    if (it == subpartsProb[i][j].end()){
                        //std::cout << "fit+0\n";
                        fit+=0;
                    }
                    else {
                        //std::cout << "subpartsProb[i][j].at(part.partIds[i][j]): " << subpartsProb[i][j].at(part.partIds[i][j]) << "\n";
                        fit+=subpartsProb[i][j].at(part.partIds[i][j]);
                    }
                }
                else{
                //if (partIds[i][j]>0&&part.partIds[i][j]>0){
                    auto it = subpartsProb[i][j].find(part.partIds[i][j]);
                    if (it == subpartsProb[i][j].end()){
                        //std::cout << "fit1+0\n";
                        fit+=0;
                    }
                    else{
                        //std::cout << "subpartsProb[i][j].at(part.partIds[i][j]) " << subpartsProb[i][j].at(part.partIds[i][j]) << ", subpartFitness(part.partsPosNorm[i][j].mean, i,j) " << subpartFitness(part.partsPosNorm[i][j].mean, i,j) << "\n";
                        //std::cout << "fit+ " << subpartsProb[i][j].at(part.partIds[i][j])*subpartFitness(part.partsPosNorm[i][j].mean, i,j) << "\n";
                        fit+=subpartsProb[i][j].at(part.partIds[i][j])*subpartFitness(part.partsPosNorm[i][j].mean, i,j,Mat34::Identity());
                    }
                }
                //getchar();
            }
        }*/
    }
    return fit;
}

/// restore occluded subparts
void ViewDependentPart::restoreOccluded(const ViewDependentPart& fullPart, int rotId, const Mat34& transform){
    std::vector<std::pair<int, int>> pointCorrespondence = {{0,0}, {0,1}, {0,2}, {1,2}, {2,2}, {2,1}, {2,0}, {1,0}};
    //std::cout << "before\n";
    //this->print();
    size_t idx=rotId;
    for (size_t j=0;j<pointCorrespondence.size();j++){
        int coordA[2]={pointCorrespondence[j].first, pointCorrespondence[j].second};//partA is not rotated
        int coordB[2]={pointCorrespondence[idx%(pointCorrespondence.size())].first, pointCorrespondence[idx%(pointCorrespondence.size())].second};//partA is not rotated
        if (partIds[coordA[0]][coordA[1]]==-3){
            //std::cout << "restore subpart\n";
            Vec6 posNorm(fullPart.partsPosNorm[coordB[0]][coordB[1]].mean);
            hop3d::transform(posNorm,transform);
            partsPosNorm[coordA[0]][coordA[1]].mean=posNorm;
            partIds[coordA[0]][coordA[1]]=fullPart.partIds[coordB[0]][coordB[1]];
            gaussians[coordA[0]][coordA[1]]=fullPart.gaussians[coordB[0]][coordB[1]];
            subpartsProb[coordA[0]][coordA[1]]=fullPart.subpartsProb[coordB[0]][coordB[1]];
            //offsets[coordA[0]][coordA[1]]=fullPart.offsets[coordB[0]][coordB[1]];
            Mat34 transformRot(transform);//we need rotation only because translation is in mean value
            transformRot.matrix().block<3,1>(0,3)=Vec3(0,0,0);
            offsets[coordA[0]][coordA[1]]=transformRot*fullPart.offsets[coordB[0]][coordB[1]];
            restored[coordA[0]][coordA[1]]=true;
        }
        idx++;
    }
    //std::cout << "after\n";
    //this->print();
    //getchar();
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
double ViewIndependentPart::distanceUmeyama(const ViewIndependentPart& partA, const ViewIndependentPart& partB, int distanceMetric, const ViewIndependentPart::Seq& vocabulary, Mat34& offset){
    /*bool theSame=true;
    for (int i=0;i<3;i++){
        for (int j=0;j<3;j++){
            for (int k=0;k<3;k++){
                if (partA.partIds[i][j][k]!=partB.partIds[i][j][k]){
                    theSame=false;
                    break;
                }
            }
            if (!theSame) break;
        }
        if (!theSame) break;
    }
    if (theSame){
        offset=Mat34::Identity();
        return 0;
    }*/
    /// compute error
    return findOptimalTransformation(partA, partB, vocabulary, distanceMetric, offset);
}

/// compute distance between view-independent parts
double ViewIndependentPart::distanceUmeyama(const ViewIndependentPart& partA, const ViewIndependentPart& partB, int distanceMetric, Mat34& offset){
    /*bool theSame=true;
    for (int i=0;i<3;i++){
        for (int j=0;j<3;j++){
            for (int k=0;k<3;k++){
                if (partA.partIds[i][j][k]!=partB.partIds[i][j][k]){
                    theSame=false;
                    break;
                }
            }
            if (!theSame) break;
        }
        if (!theSame) break;
    }
    if (theSame){
        offset=Mat34::Identity();
        return 0;
    }*/
    /// compute error
    return findOptimalTransformation(partA, partB, distanceMetric, offset);
}

/// compute distance between view-independent parts
double ViewIndependentPart::distanceGICP(const ViewIndependentPart& partA, const ViewIndependentPart& partB, const ConfigGICP& configGICP, Mat34& offset){
    //pcl::PointCloud<pcl::PointNormal> sourceCloud;
    if (!configGICP.verbose){
        pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
    }
    auto startTime = std::chrono::high_resolution_clock::now();
    pcl::PointCloud<pcl::PointNormal>::Ptr sourceCloud (new pcl::PointCloud<pcl::PointNormal>);
    if (partA.cloud.size()==partB.cloud.size()){
        bool thesame=true;
        for (size_t i=0;i<partA.cloud.size();i++){
            if (partA.cloud[i].position!=partB.cloud[i].position){
                thesame=false;
                break;
            }
        }
        if (thesame){
            offset=Mat34::Identity();
            return 0;
        }
    }
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
    // Set the maximum number of iterations (criterion 1)
    gicp.setMaximumIterations (configGICP.maxIterations);
    // Set the transformation epsilon (criterion 2)
    gicp.setTransformationEpsilon (configGICP.transformationEpsilon);
    // Set the euclidean distance difference epsilon (criterion 3)
    gicp.setEuclideanFitnessEpsilon (configGICP.EuclideanFitnessEpsilon);
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
            Eigen::Matrix4f trans = gicp.getFinalTransformation();
            if ((trans(0,3)<0.1)&&(trans(1,3)<0.1)&&(trans(2,3)<0.1)){
                //Mat34 trans34; trans34.matrix() = trans.cast<double>();
                //double error = computeError(partA.cloud, partB.cloud, trans34);
                //std::cout << "found transform\n" << trans34.matrix() << "\n";
                //std::cout << "error " << error << "\n";
                //getchar();
                if (gicp.getFitnessScore()<errorMin&&fabs(gicp.getFinalTransformation()(0,3))<0.05&&fabs(gicp.getFinalTransformation()(1,3))<0.05&&fabs(gicp.getFinalTransformation()(2,3))<0.05){
                    transform = trans;
                    //errorMin = error;
                    errorMin = gicp.getFitnessScore();
                }
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

/// compute error between tow point clous
double ViewIndependentPart::computeError(const ViewIndependentPart& partA, const ViewIndependentPart& partB, hop3d::Mat34& trans, int type, double coeff){
    double errorRot=0;
    double errorPos=0;
    //partA.print();
    //partB.print();
    //std::cout << "transformation \n" << transformation.matrix() << "\n";
    for (size_t i=0; i<partA.partIds.size();i++){
        for (size_t j=0; j<partA.partIds.size();j++){
            for (size_t k=0; k<partA.partIds.size();k++){
                if ((partA.partIds[i][j][k]>=0)&&(partB.partIds[i][j][k]>=0)){
                    if ((type == 1)||((type == 3))){
                        Eigen::Matrix<double, 4, 1> posPrim;
                        posPrim(0) = partA.clouds[i][j][k][0].position(0); posPrim(1) = partA.clouds[i][j][k][0].position(1); posPrim(2) = partA.clouds[i][j][k][0].position(2); posPrim(3) = 1;
                        posPrim = trans * posPrim;
                        errorPos+= sqrt((posPrim.block<3,1>(0,0)-partB.clouds[i][j][k][0].position).transpose()*(posPrim.block<3,1>(0,0)-partB.clouds[i][j][k][0].position));
                        //std::cout << "error pos " << errorPos << "\n";
                    }
                    if ((type == 2)||((type == 3))){
                        double dotprodA = (trans.rotation()*partA.clouds[i][j][k][0].normal).adjoint()*partB.clouds[i][j][k][0].normal;
                        if (dotprodA>1.0) dotprodA=1.0;
                        if (dotprodA<-1.0) dotprodA=-1.0;
                        double anglePart = acos(dotprodA);
                        double dotprodB = (trans.rotation()*partA.clouds[i][j][k][0].normal).adjoint()*partB.clouds[i][j][k][0].normal;
                        if (dotprodB>1.0) dotprodB=1.0;
                        if (dotprodB<-1.0) dotprodB=-1.0;
                        double angleCenter = acos(dotprodB);
                        errorRot+=fabs(anglePart-angleCenter);
                        //std::cout << "error tmp " << errorRot << "\n";
                    }
                }
                else if ((partA.partIds[i][j][k]<0)&&(partB.partIds[i][j][k]<0)){
                    errorPos+=0.0;
                    errorRot+=0.0;
                    //std::cout << "error bckgr rot " << errorRot << "\n";
                    //std::cout << "error bckgr pos " << errorPos << "\n";
                }
                else if ((partA.partIds[i][j][k]<0&&partB.partIds[i][j][k]>=0)||(partA.partIds[i][j][k]>=0&&partB.partIds[i][j][k]<0)){
                    errorPos+=0.5;
                    errorRot+=0.5;
                    //std::cout << "error bckgr diff rot " << errorRot << "\n";
                    //std::cout << "error bckgr diff pos " << errorPos << "\n";
                }
            }
        }
    }
    //std::cout << "distpos " << errorPos << " dist rot: " << errorRot << "\n";
    //std::cout << transformation.matrix() << "\n";
    //std::cout << "error final " << errorPos+coeff*errorRot << "\n";
    //getchar();
    return errorPos+coeff*errorRot;
}

/// compute error between tow point clous
double ViewIndependentPart::computeError(const hop3d::PointCloud& cloudA, const hop3d::PointCloud& cloudB, hop3d::Mat34& trans){
    hop3d::PointCloud cloudD;
    // transform point
    trans= trans.inverse();
    for (auto &point : cloudB){
        PointNormal pointD;
        //std::cout << "point.position " << point.position.transpose() << "\n";
        Vec4 pos4d(point.position(0), point.position(1), point.position(2),1.0);
        //std::cout << (trans*pos4d).matrix() << "\n";
        pointD.position = (trans*pos4d).block<3,1>(0,0);
        //std::cout << "pointD.position " << pointD.position.transpose() << "\n";
        pointD.normal = trans.rotation()*point.normal;
        //std::cout << "pointD.normal " << pointD.normal.transpose() << "\n";
        cloudD.push_back(pointD);
    }
    //std::cout << "trans \n" << trans.matrix() << "\n";
    ///compute distance between neighbours
    double error = 0;
    double coeff = 0.05;
    hop3d::PointCloud biggerCloud;
    hop3d::PointCloud smallerCloud;
    if (cloudA.size()>cloudD.size()){
        biggerCloud = cloudA;
        smallerCloud = cloudD;
    }
    else{
        biggerCloud = cloudD;
        smallerCloud = cloudA;
    }
    for (auto &pointA : biggerCloud){
        double minDist = std::numeric_limits<double>::max();
        double minError = std::numeric_limits<double>::max();
        for (auto &pointD : smallerCloud){
            double dist = sqrt((pointA.position-pointD.position).transpose()*(pointA.position-pointD.position));
            if (dist<minDist){
                minDist=dist;
                double dotprod = pointA.normal.adjoint()*pointD.normal;
                if (dotprod>1.0) dotprod=1.0;
                if (dotprod<-1.0) dotprod=-1.0;
                double angle = acos(dotprod);
                minError = dist + coeff*angle;
            }
        }
        error+=minError;
    }
    return error/double(biggerCloud.size());
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

// The input 3D points are stored as columns.
Eigen::Affine3d ViewDependentPart::Find3DAffineTransform(Eigen::Matrix3Xd in, Eigen::Matrix3Xd out) {

  // Default output
  Eigen::Affine3d A;
  A.linear() = Eigen::Matrix3d::Identity(3, 3);
  A.translation() = Eigen::Vector3d::Zero();

  if (in.cols() != out.cols())
    throw "Find3DAffineTransform(): input data mis-match";

  // First find the scale, by finding the ratio of sums of some distances,
  // then bring the datasets to the same scale.
  double dist_in = 0, dist_out = 0;
  for (int col = 0; col < in.cols()-1; col++) {
    dist_in  += (in.col(col+1) - in.col(col)).norm();
    dist_out += (out.col(col+1) - out.col(col)).norm();
  }
  if (dist_in <= 0 || dist_out <= 0)
    return A;
  double scale = dist_out/dist_in;
  out /= scale;

  // Find the centroids then shift to the origin
  Eigen::Vector3d in_ctr = Eigen::Vector3d::Zero();
  Eigen::Vector3d out_ctr = Eigen::Vector3d::Zero();
  for (int col = 0; col < in.cols(); col++) {
    in_ctr  += in.col(col);
    out_ctr += out.col(col);
  }
  in_ctr /= in.cols();
  out_ctr /= out.cols();
  for (int col = 0; col < in.cols(); col++) {
    in.col(col)  -= in_ctr;
    out.col(col) -= out_ctr;
  }

  // SVD
  Eigen::MatrixXd Cov = in * out.transpose();
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(Cov, Eigen::ComputeThinU | Eigen::ComputeThinV);

  // Find the rotation
  double d = (svd.matrixV() * svd.matrixU().transpose()).determinant();
  if (d > 0)
    d = 1.0;
  else
    d = -1.0;
  Eigen::Matrix3d I = Eigen::Matrix3d::Identity(3, 3);
  I(2, 2) = d;
  Eigen::Matrix3d R = svd.matrixV() * I * svd.matrixU().transpose();

  // The final transform
  A.linear() = scale * R;
  A.translation() = scale*(out_ctr - R*in_ctr);

  return A;
}

///find optimal transformation between normals
double ViewDependentPart::findOptimalTransformation(const ViewDependentPart& partA, const ViewDependentPart& partB, int distanceMetric, Mat34& transOpt, int& rotId){
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
            //std::cout << "pointsA \n" << pointsA << "\n";
            //std::cout << "pointsB \n" << pointsB << "\n";
            Eigen::Matrix4d transUmeyama = Eigen::umeyama(pointsA,pointsB,false);
            //std::cout << "trans without umeyama\n" << trans1 << "\n";
            Eigen::Matrix4d transUmeyamaScale = Eigen::umeyama(pointsA,pointsB,true);
            //std::cout << "trans with umeyama\n" << trans2 << "\n";
            Eigen::Affine3d transNew = Find3DAffineTransform(pointsA,pointsB);
            //std::cout << "new method \n" << A.matrix() << "\n";
            //trans(2,3)=0;
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
            trans.matrix() = transUmeyama;
            trans(2,3)=0;
            double errorUmeyama = computeError(partA, partD, trans, distanceMetric, 0.1);
            //std::cout << "error without umeyama: " << errorUmeyama <<" \n";
            trans.matrix() = transUmeyamaScale;
            trans(2,3)=0;
            double errorUmeyamaScale = computeError(partA, partD, trans, distanceMetric, 0.1);
            //std::cout << "error with umeyama: " << errorUmeyamaScale <<" \n";
            trans.matrix() = transNew.matrix();
            trans(2,3)=0;
            double errorNew = computeError(partA, partD, trans, distanceMetric, 0.1);
            //std::cout << "error with new method: " << errorNew <<" \n";
            double error;
            if (errorUmeyama<errorUmeyamaScale){
                error=errorUmeyama;
                trans.matrix()=transUmeyama;
            }
            else{
                error=errorUmeyamaScale;
                trans.matrix()=transUmeyamaScale;
            }
            if (errorNew<error){
                error = errorNew;
                trans.matrix() = transNew.matrix();
            }
            //std::cout << "min error " << error <<"\n";
            //getchar();
            //trans(2,3)=trans1(2,3);
            if (error<minDist){
                minDist=error;
                transOpt = trans;
                rotId = (int)i;
            }
        }
    }
    return minDist;
}

/// create point cloud from second layer part
int ViewDependentPart::createPointsMatrix(const ViewDependentPart& part, const ViewDependentPart::Seq& vocabulary, int rotIndex, PointsSecondLayer& points, Eigen::MatrixXd& partIds){
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
                        partIds(newCoords[0],newCoords[1])=vocabulary[part.partIds[rowId][partId]].partIds[i][j];
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
                            /*if (points[newCoords[0]][newCoords[1]].mean.block<3,1>(3,0)(0)==0&&points[newCoords[0]][newCoords[1]].mean.block<3,1>(3,0)(1)==0&&points[newCoords[0]][newCoords[1]].mean.block<3,1>(3,0)(2)==0){
                                std::cout << "ij "<< i << " " << j << "\n";
                                vocabulary[part.partIds[rowId][partId]].print();
                                getchar();
                            }*/
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
                        partIds(newCoords[0],newCoords[1])=part.partIds[rowId][partId];
                    }
                }
            }
        }
    }
    return pointsNo;
}

/// create point cloud from second layer part
int ViewDependentPart::createPointsMatrix(const ViewDependentPart& part, const ViewDependentPart::Seq& vocabularyLayer2, const ViewDependentPart::Seq& vocabularyLayer3, int rotIndex, PointsThirdLayer& points, Eigen::MatrixXd& partIds){
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
    int pointsNo=0;
    for (int rowId=0; rowId<3; rowId++){
        for (int partId=0; partId<3; partId++){
            if (part.partIds[rowId][partId]>=0){
                Vec3 posMiddle;
                if (rowId==1&&partId==1)
                    posMiddle = Vec3(0,0,0);
                else
                    posMiddle = part.partsPosNorm[rowId][partId].mean.block<3,1>(0,0);
                PointsSecondLayer pointsPrevLay; //9x9
                Eigen::MatrixXd partPrevids(9,9);
                pointsNo += createPointsMatrix(vocabularyLayer3[part.partIds[rowId][partId]], vocabularyLayer2, (int)rotIndex, pointsPrevLay, partPrevids);
                int newCoords[2]={ids[rowId][partId][rotIndex].first, ids[rowId][partId][rotIndex].second};
                for (int i=0;i<9;i++){
                    for (int j=0;j<9;j++){
                        points[newCoords[0]*9+i][newCoords[1]*9+j].mean=pointsPrevLay[i][j].mean;
                        partIds(newCoords[0]*9+i,newCoords[1]*9+j)=partPrevids(i,j);
                    }
                }
            }
            else {// fill matrix with nans
                for (int i=0;i<3;i++){
                    for (int j=0;j<3;j++){
                        int newCoords[2]={ids[rowId][partId][rotIndex].first*3+ids[i][j][rotIndex].first, ids[rowId][partId][rotIndex].second*3+ids[i][j][rotIndex].second};
                        for (int k=0;k<3;k++){
                            for (int l=0;l<3;l++){
                                int newCoords2[2]={newCoords[0]*3+ids[k][l][rotIndex].first, newCoords[1]*3+ids[k][l][rotIndex].second};
                                points[newCoords2[0]][newCoords2[1]].mean.block<3,1>(0,0)=Vec3(NAN,NAN,NAN);
                                points[newCoords2[0]][newCoords2[1]].mean.block<3,1>(3,0)=Vec3(0,0,1);
                                partIds(newCoords2[0],newCoords2[1])=part.partIds[rowId][partId];
                            }
                        }
                    }
                }
            }
        }
    }
    return pointsNo;
}

/// find SE3 transformation
bool ViewDependentPart::findSE3Transformation(const PointsSecondLayer& pointsA, const PointsSecondLayer& pointsB, Mat34& trans, int estType){
    std::vector<Vec3> setA; std::vector<Vec3> setB;
    int pairsNo=0;
    for (int i=0;i<(int)pointsA.size();i++){
        for (int j=0;j<(int)pointsA[i].size();j++){
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
        Eigen::Matrix4d transform;
        if (estType==0)
            transform= Eigen::umeyama(psA,psB,false);
        else if (estType==1)
            transform = Eigen::umeyama(psA,psB,true);
        else if (estType==2){
            Eigen::Affine3d transNew = Find3DAffineTransform(psA,psB);
            transform = transNew.matrix();
        }

        trans.matrix() = transform;
        return true;
    }
    else {
        trans = Mat34::Identity();
        return false;
    }
}

/// find SE3 transformation
bool ViewDependentPart::findSE3Transformation(const PointsThirdLayer& pointsA, const PointsThirdLayer& pointsB, Mat34& trans, int estType){
    std::vector<Vec3> setA; std::vector<Vec3> setB;
    int pairsNo=0;
    for (int i=0;i<(int)pointsA.size();i++){
        for (int j=0;j<(int)pointsA[i].size();j++){
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
        Eigen::Matrix4d transform;
        if (estType==0)
            transform= Eigen::umeyama(psA,psB,false);
        else if (estType==1)
            transform = Eigen::umeyama(psA,psB,true);
        else if (estType==2){
            Eigen::Affine3d transNew = Find3DAffineTransform(psA,psB);
            transform = transNew.matrix();
        }
        trans.matrix() = transform;
        return true;
    }
    else {
        trans = Mat34::Identity();
        return false;
    }
}

///find optimal transformation for view independent parts
double ViewIndependentPart::findOptimalTransformation(const ViewIndependentPart& partA, const ViewIndependentPart& partB, const ViewIndependentPart::Seq& vocabulary, int distanceMetric, Mat34& transOpt){
    double minDist = std::numeric_limits<double>::max();
    for (int rotX=0; rotX<8;rotX++){//for each
        for (int rotY=0; rotY<8;rotY++){
            //std::cout << "rotNo " << rotNo << "\n";
            Mat34 trans;
            double error = computeVIPartsDistance(partA, partB,rotX, rotY, vocabulary, distanceMetric, trans);
            if (error<minDist){
                minDist=error;
                transOpt = trans;
            }
        }
    }
    //std::cout << minDist << ", ";
    return minDist;
}

/// find rotated coordinates
void ViewIndependentPart::findCorrespondence(int idX, int idY, int idZ, int rotX, int rotY, int rotZ, std::array<int,3>& newCoords){
    hop3d::Quaternion quat;
    Eigen::AngleAxis<double> aaX(rotX*(M_PI/4.0), Eigen::Vector3d::UnitX());
    Eigen::AngleAxis<double> aaY(rotY*(M_PI/4.0), Eigen::Vector3d::UnitY());
    Eigen::AngleAxis<double> aaZ(rotZ*(M_PI/4.0), Eigen::Vector3d::UnitZ());
    quat = aaX * aaY * aaZ;
    Mat33 rot = quat.matrix();
    Vec3 initPos(idX-1, idY-1, idZ-1);
    initPos.normalize();
    Vec3 newPos(rot*initPos);
    for (int i=0;i<3;i++){
        newCoords[i]=int(newPos(i)+1.5);
    }
}

/// Compute distance between two parts
double ViewIndependentPart::computeVIPartsDistance(const ViewIndependentPart& partA, const ViewIndependentPart& partB, int rotX, int rotY, int distanceMetric, Mat34& transOpt){
    std::vector<Vec3> setA; std::vector<Vec3> setB;
    getCorrespondingPoints(partA, partB, rotX, rotY, setA, setB);
    if (setA.size()>=3){ // it's possible to find SE3 transformation
        Eigen::MatrixXd pointsA(3,setA.size());
        Eigen::MatrixXd pointsB(3,setA.size());
        for (int pairNo=0;pairNo<(int)setA.size();pairNo++){
            for (int col=0;col<3;col++){
                pointsA(col,pairNo) = setA[pairNo](col);
                pointsB(col,pairNo) = setB[pairNo](col);
            }
        }
        Eigen::Matrix4d trans = Eigen::umeyama(pointsA,pointsB,false);
        transOpt.matrix() = trans;
        return computeError(partA, partB, transOpt, distanceMetric, 0.1);
    }
    else
        return std::numeric_limits<double>::max();
}

/// Get corresponding points from parts
void ViewIndependentPart::getCorrespondingPoints(const ViewIndependentPart& partA, const ViewIndependentPart& partB, int rotX, int rotY, std::vector<Vec3>& setA, std::vector<Vec3>& setB){
    for (int idX=0;idX<3;idX++){// for each subpart
        for (int idY=0;idY<3;idY++){
            for (int idZ=0;idZ<3;idZ++){
                std::array<int,3> newCoords = {0,0,0};
                if (!(idX==1&&idY==1&&idZ==1))
                    findCorrespondence(idX, idY, idZ, rotX, rotY, 0, newCoords);
                if (partA.partIds[idX][idY][idZ]>=0&&partB.partIds[newCoords[0]][newCoords[1]][newCoords[2]]>=0){
                    for (size_t pointNo=0; pointNo<partA.clouds[idX][idY][idZ].size(); pointNo++){//do not takes into accoun normal vectors
                        setA.push_back(partA.clouds[idX][idY][idZ][pointNo].position);
                        setB.push_back(partB.clouds[newCoords[0]][newCoords[1]][newCoords[2]][pointNo].position);
                    }
                }
                //std::cout << "old " << idX << ", " << idY << ", " << idZ << " rot by " << rotX*(M_PI/4.0)*180/3.14 << ", " << rotY*(M_PI/4.0)*180/3.14 << ", " << " new coords " << newCoords[0] << ", " << newCoords[1] << ", " << newCoords[2] << "\n";
            }
        }
    }
}

/// Compute distance between two parts
double ViewIndependentPart::computeVIPartsDistance(const ViewIndependentPart& partA, const ViewIndependentPart& partB, int rotX, int rotY, const ViewIndependentPart::Seq vocabulary, int distanceMetric, Mat34& transOpt){
    std::vector<Vec3> setA; std::vector<Vec3> setB;
    for (int idX=0;idX<3;idX++){// for each subpart
        for (int idY=0;idY<3;idY++){
            for (int idZ=0;idZ<3;idZ++){
                int idA = partA.partIds[idX][idY][idZ];
                int idB = partB.partIds[idX][idY][idZ];
                if (idA>0&&idB>0){
                    std::vector<Vec3> setAtmp; std::vector<Vec3> setBtmp;
                    getCorrespondingPoints(vocabulary[idA], vocabulary[idB], rotX, rotY, setAtmp, setBtmp);
                    /// transform parts

                    setA.insert(setA.end(), setAtmp.begin(), setAtmp.end());
                    setB.insert(setB.end(), setBtmp.begin(), setBtmp.end());
                }
            }
        }
    }
    if (setA.size()>=3){ // it's possible to find SE3 transformation
        Eigen::MatrixXd pointsA(3,setA.size());
        Eigen::MatrixXd pointsB(3,setA.size());
        for (int pairNo=0;pairNo<(int)setA.size();pairNo++){
            for (int col=0;col<3;col++){
                pointsA(col,pairNo) = setA[pairNo](col);
                pointsB(col,pairNo) = setB[pairNo](col);
            }
        }
        Eigen::Matrix4d trans = Eigen::umeyama(pointsA,pointsB,false);
        transOpt.matrix() = trans;
        return computeError(partA, partB, transOpt, distanceMetric, 0.1);
    }
    else
        return std::numeric_limits<double>::max();
}

///find optimal transformation for view independent parts
double ViewIndependentPart::findOptimalTransformation(const ViewIndependentPart& partA, const ViewIndependentPart& partB, int distanceMetric, Mat34& transOpt){
    double minDist = std::numeric_limits<double>::max();
    for (int rotX=0; rotX<8;rotX++){//for each
        for (int rotY=0; rotY<8;rotY++){
            //std::cout << "rotNo " << rotNo << "\n";
            Mat34 trans;
            double error = computeVIPartsDistance(partA, partB,rotX, rotY, distanceMetric, trans);
            if (error<minDist){
                minDist=error;
                transOpt = trans;
            }
        }
    }
    //std::cout << minDist << ", ";
    return minDist;
}

///find optimal transformation between normals
double ViewDependentPart::findOptimalTransformation(const ViewDependentPart& partA, const ViewDependentPart& partB, const ViewDependentPart::Seq& vocabulary, int distanceMetric, Mat34& transOpt, int& rotId){
    double minDist = std::numeric_limits<double>::max();
    for (size_t i=0;i<8;i++){
        PointsSecondLayer pointsA, pointsB; //9x9
        Eigen::MatrixXd partAids(9,9);
        Eigen::MatrixXd partBids(9,9);
        int pointsNoA = createPointsMatrix(partA, vocabulary, 0, pointsA, partAids);
        int pointsNoB = createPointsMatrix(partB, vocabulary, (int)i, pointsB, partBids);
        if (pointsNoA>4&&pointsNoB>4){
            Mat34 trans;
            if (findSE3Transformation(pointsA, pointsB,trans,0)){
                Mat34 trans1(trans);
                trans(2,3)=0;
                double error = computeError(pointsA, pointsB, partAids, partBids, trans, distanceMetric, 1.0);
                trans(2,3)=trans1(2,3);
                if (error<minDist){
                    minDist=error;
                    transOpt = trans;
                    rotId = (int)i;
                }
            }
            if (findSE3Transformation(pointsA, pointsB,trans,1)){
                Mat34 trans1(trans);
                trans(2,3)=0;
                double error = computeError(pointsA, pointsB, partAids, partBids, trans, distanceMetric, 1.0);
                trans(2,3)=trans1(2,3);
                if (error<minDist){
                    minDist=error;
                    transOpt = trans;
                    rotId = (int)i;
                }
            }
            if (findSE3Transformation(pointsA, pointsB,trans,2)){
                Mat34 trans1(trans);
                trans(2,3)=0;
                double error = computeError(pointsA, pointsB, partAids, partBids, trans, distanceMetric, 1.0);
                trans(2,3)=trans1(2,3);
                if (error<minDist){
                    minDist=error;
                    transOpt = trans;
                    rotId = (int)i;
                }
            }
        }
    }
    return minDist;
}

///find optimal transformation between normals
double ViewDependentPart::findOptimalTransformation(const ViewDependentPart& partA, const ViewDependentPart& partB, const ViewDependentPart::Seq& vocabularyLayer2, const ViewDependentPart::Seq& vocabularyLayer3, int distanceMetric, Mat34& transOpt, int& rotId){
    double minDist = std::numeric_limits<double>::max();
    for (size_t i=0;i<8;i++){
        PointsThirdLayer pointsA, pointsB; //27x27
        Eigen::MatrixXd partAids(27,27);
        Eigen::MatrixXd partBids(27,27);
        int pointsNoA = createPointsMatrix(partA, vocabularyLayer2, vocabularyLayer3, 0, pointsA, partAids);
        int pointsNoB = createPointsMatrix(partB, vocabularyLayer2, vocabularyLayer3, (int)i, pointsB, partBids);
        if (pointsNoA>4&&pointsNoB>4){
            Mat34 trans;
            if (findSE3Transformation(pointsA, pointsB,trans,0)){
                Mat34 trans1(trans);
                trans(2,3)=0;
                double error = computeError(pointsA, pointsB, partAids, partBids, trans, distanceMetric, 1.0);
                trans(2,3)=trans1(2,3);
                if (error<minDist){
                    minDist=error;
                    transOpt = trans;
                    rotId = (int)i;
                }
            }
            if (findSE3Transformation(pointsA, pointsB,trans,1)){
                Mat34 trans1(trans);
                trans(2,3)=0;
                double error = computeError(pointsA, pointsB, partAids, partBids, trans, distanceMetric, 1.0);
                trans(2,3)=trans1(2,3);
                if (error<minDist){
                    minDist=error;
                    transOpt = trans;
                    rotId = (int)i;
                }
            }
            if (findSE3Transformation(pointsA, pointsB,trans,2)){
                Mat34 trans1(trans);
                trans(2,3)=0;
                double error = computeError(pointsA, pointsB, partAids, partBids, trans, distanceMetric, 1.0);
                trans(2,3)=trans1(2,3);
                if (error<minDist){
                    minDist=error;
                    transOpt = trans;
                    rotId = (int)i;
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
    double coeffC=1.0;
    for (size_t i=0; i<partA.partIds.size();i++){
        for (size_t j=0; j<partA.partIds.size();j++){
            if ((partA.partIds[i][j]>=0)&&(partB.partIds[i][j]>=0)){
                if ((type == 1)||((type == 3))){
                    Eigen::Matrix<double, 4, 1> posPrim;
                    posPrim(0) = partA.partsPosNorm[i][j].mean(0); posPrim(1) = partA.partsPosNorm[i][j].mean(1); posPrim(2) = partA.partsPosNorm[i][j].mean(2); posPrim(3) = 1;
                    //std::cout << "point before " << posPrim << "\n";
                    posPrim = transformation * posPrim;
                    //std::cout << " point transformed " << posPrim << "\n";
                    errorPos+= sqrt((posPrim.block<3,1>(0,0)-partB.partsPosNorm[i][j].mean.block<3,1>(0,0)).transpose()*(posPrim.block<3,1>(0,0)-partB.partsPosNorm[i][j].mean.block<3,1>(0,0)));
                    //std::cout << "should be " << partB.partsPosNorm[i][j].mean.block<3,1>(0,0) << "\n";
                    //std::cout << "error pos " << sqrt((posPrim.block<3,1>(0,0)-partB.partsPosNorm[i][j].mean.block<3,1>(0,0)).transpose()*(posPrim.block<3,1>(0,0)-partB.partsPosNorm[i][j].mean.block<3,1>(0,0))) << "\n";
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
            else if ((partA.partIds[i][j]==-1||partA.partIds[i][j]==-2)&&(partB.partIds[i][j]==-1||partB.partIds[i][j]==-2)){
                errorPos+=0.0;
                errorRot+=0.0;
            }
            else if ((partA.partIds[i][j]==-3)&&(partB.partIds[i][j]==-3)){
                errorPos+=0.0;
                errorRot+=0.0;
            }
            else if (((partA.partIds[i][j]==-1||partA.partIds[i][j]==-2)&&partB.partIds[i][j]>=0)||(partA.partIds[i][j]>=0&&(partB.partIds[i][j]==-1||partB.partIds[i][j]==-2))){
                errorPos+=coeffC;
                errorRot+=coeffC;
            }
            else if ((partA.partIds[i][j]==-3&&partB.partIds[i][j]!=-3)||(partA.partIds[i][j]!=-3&&partB.partIds[i][j]==-3)){
                errorPos+=coeffC;
                errorRot+=coeffC;
            }
            else if ((partA.partIds[i][j]==-3&&(partB.partIds[i][j]==-1||partB.partIds[i][j]==-2))||((partA.partIds[i][j]==-1||partA.partIds[i][j]==-2)&&partB.partIds[i][j]==-3)){
                errorPos+=coeffC;
                errorRot+=coeffC;
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
double ViewDependentPart::computeError(const PointsSecondLayer& partA, const PointsSecondLayer& partB, const Eigen::MatrixXd& partAids, const Eigen::MatrixXd& partBids, const Mat34& transformation, int type, double coeff){
    double errorRot=0;
    double errorPos=0;
    int correspondencesNo=0;
    double coeffC=1.0;
    for (size_t i=0; i<partA.size();i++){
        for (size_t j=0; j<partA[i].size();j++){
            if ((partAids(i,j)>=0)&&(partBids(i,j)>=0)){
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
            else if ((partAids(i,j)==-1||partAids(i,j)==-2)&&(partBids(i,j)==-1||partBids(i,j)==-2)){
                errorPos+=0.0;
                errorRot+=0.0;
            }
            else if ((partAids(i,j)==-3)&&(partBids(i,j)==-3)){
                errorPos+=0.0;
                errorRot+=0.0;
            }
            else if (((partAids(i,j)==-1||partAids(i,j)==-2)&&partBids(i,j)>=0)||(partAids(i,j)>=0&&(partBids(i,j)==-1||partBids(i,j)==-2))){
                errorPos+=coeffC;
                errorRot+=coeffC;
            }
            else if ((partAids(i,j)==-3&&partBids(i,j)!=-3)||(partAids(i,j)!=-3&&partBids(i,j)==-3)){
                errorPos+=coeffC;
                errorRot+=coeffC;
            }
            else if ((partAids(i,j)==-3&&(partBids(i,j)==-1||partBids(i,j)==-2))||((partAids(i,j)==-1||partAids(i,j)==-2)&&partBids(i,j)==-3)){
                errorPos+=coeffC;
                errorRot+=coeffC;
            }
        }
    }
    return (errorPos+coeff*errorRot)/double(correspondencesNo);
}

/// view invariant error for two second layer parts with known SE3 transformation
double ViewDependentPart::computeError(const PointsThirdLayer& partA, const PointsThirdLayer& partB, const Eigen::MatrixXd& partAids, const Eigen::MatrixXd& partBids, const Mat34& transformation, int type, double coeff){
    double errorRot=0;
    double errorPos=0;
    int correspondencesNo=0;
    double coeffC =1.0;
    for (size_t i=0; i<partA.size();i++){
        for (size_t j=0; j<partA[i].size();j++){
            if ((partAids(i,j)>=0)&&(partBids(i,j)>=0)){
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
            else if ((partAids(i,j)==-1||partAids(i,j)==-2)&&(partBids(i,j)==-1||partBids(i,j)==-2)){
                errorPos+=0.0;
                errorRot+=0.0;
            }
            else if ((partAids(i,j)==-3)&&(partBids(i,j)==-3)){
                errorPos+=0.0;
                errorRot+=0.0;
            }
            else if (((partAids(i,j)==-1||partAids(i,j)==-2)&&partBids(i,j)>=0)||(partAids(i,j)>=0&&(partBids(i,j)==-1||partBids(i,j)==-2))){
                errorPos+=coeffC;
                errorRot+=coeffC;
            }
            else if ((partAids(i,j)==-3&&partBids(i,j)!=-3)||(partAids(i,j)!=-3&&partBids(i,j)==-3)){
                errorPos+=coeffC;
                errorRot+=coeffC;
            }
            else if ((partAids(i,j)==-3&&(partBids(i,j)==-1||partBids(i,j)==-2))||((partAids(i,j)==-1||partAids(i,j)==-2)&&partBids(i,j)==-3)){
                errorPos+=coeffC;
                errorRot+=coeffC;
            }
        }
    }
    return (errorPos+coeff*errorRot)/double(correspondencesNo);
}

/// compute distance between view dependent parts (invariant version)
double ViewDependentPart::distanceInvariant(const ViewDependentPart& partA, const ViewDependentPart& partB, int distanceMetric, Mat34& estimatedTransform, int& rotId){
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

    sum = findOptimalTransformation(partA, partB, distanceMetric, estimatedTransform, rotId);
    //std::cout << "sum " << sum << "n\n";
    //std::cout << "est trans\n" << estimatedTransform.matrix() << "\n";
    //getchar();
    return sum;
}

/// compute distance between view dependent parts (invariant version)
double ViewDependentPart::distanceInvariant(const ViewDependentPart& partA, const ViewDependentPart& partB, int distanceMetric, const ViewDependentPart::Seq& vocabulary, Mat34& estimatedTransform, int& rotId){
    estimatedTransform=Mat34::Identity();
    double sum = 9;
    sum = findOptimalTransformation(partA, partB, vocabulary, distanceMetric, estimatedTransform, rotId);
    if (estimatedTransform(2,2)<0){
        sum+=0.01;
    }
    //std::cout << "sum : " << sum << "\n";
    //std::cout << "est trans\n" << estimatedTransform.matrix() << "\n";
    //getchar();
    return sum;
}

/// compute distance between view dependent parts (invariant version) for fourth layer
double ViewDependentPart::distanceInvariant(const ViewDependentPart& partA, const ViewDependentPart& partB, int distanceMetric, const ViewDependentPart::Seq& vocabularyLayer2, const ViewDependentPart::Seq& vocabularyLayer3, Mat34& estimatedTransform, int& rotId){
    estimatedTransform=Mat34::Identity();
    double sum = 9;
    sum = findOptimalTransformation(partA, partB, vocabularyLayer2, vocabularyLayer3, distanceMetric, estimatedTransform, rotId);
    if (estimatedTransform(2,2)<0){
        sum+=0.01;
    }
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
    os << part.id << " " << " " << part.realisationId << " " << part.layerId << " " << part.locationEucl << part.offset;
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
    os << part.group.size() << " ";
    for (size_t i=0;i<part.group.size();i++){
        os << part.group[i];
    }
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
    is >> part.id >> part.realisationId >> part.layerId >> part.locationEucl >> part.offset;
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
    os << part.pose << " ";
    os << part.realisationId << "\n";
    return os;
}

/// Extraction operator
std::istream& operator>>(std::istream& is, ViewIndependentPart::Part3D& part){
    is >> part.id;
    is >> part.pose;
    is >> part.realisationId;
    return is;
}

/// Insertion operator
std::ostream& operator<<(std::ostream& os, const ViewIndependentPart& part){
    os << part.id << " " << part.realisationId << " " << part.layerId << " ";
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
    os << part.cloud.size() << " ";
    for (auto &point : part.cloud){
        os << point;
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
    for (int i=0;i<3;i++){
        for (int j=0;j<3;j++){
            for (int k=0;k<3;k++){
                os << part.offsets[i][j][k];
            }
        }
    }
    for (int i=0;i<3;i++){
        for (int j=0;j<3;j++){
            for (int k=0;k<3;k++){
                os << part.clouds[i][j][k].size() << " ";
                for (auto &point : part.clouds[i][j][k]){
                    os << point;
                }
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
    is >> part.id >> part.realisationId >> part.layerId;
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
    int cloudSize;
    is >> cloudSize;
    part.cloud.clear();
    part.cloud.reserve(cloudSize);
    for (int i=0;i<cloudSize;i++){
        PointNormal point;
        is >> point;
        part.cloud.push_back(point);
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
    for (int i=0;i<3;i++){
        for (int j=0;j<3;j++){
            for (int k=0;k<3;k++){
                is >> part.offsets[i][j][k];
            }
        }
    }
    for (int i=0;i<3;i++){
        for (int j=0;j<3;j++){
            for (int k=0;k<3;k++){
                is >> cloudSize;
                part.clouds[i][j][k].clear();
                part.clouds[i][j][k].reserve(cloudSize);
                for (int pointNo=0;pointNo<cloudSize;pointNo++){
                    PointNormal point;
                    is >> point;
                    part.clouds[i][j][k].push_back(point);
                }
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
