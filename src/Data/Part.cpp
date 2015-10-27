#include "Data/Part.h"
#include "Data/Vocabulary.h"

namespace hop3d {

/// Print
void ViewDependentPart::print() const{
    std::cout << "Id: " << id << "\n";
    std::cout << "Layer id: " << layerId << "\n";
    std::cout << "Image coords: " << location.u << ", " << location.v << ", " << location.depth << "\n";
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
    std::cout << "Gaussians:\n";
    for (size_t i=0; i<gaussians.size();i++){
        for (size_t j=0; j<gaussians.size();j++){
            std::cout << "mean(" << i << ", " << j << "): ";
            std::cout << gaussians[i][j].mean.transpose() << "\n";
            //std::cout << "covariance(" << i-1 << ", " << j-1 << "):\n";
            //std::cout << gaussians[i][j].covariance << "\n";
        }
    }
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
double ViewIndependentPart::distance(const ViewIndependentPart& partA, const ViewIndependentPart& partB, Mat34& offset, int verbose){
    if (partA.partIds==partB.partIds){
        offset = Mat34::Identity();
        return 0;
    }
    double sum=0;
    std::vector<std::pair<Mat34, int>> partASeq;
    std::vector<std::pair<Mat34, int>> partBSeq;
    Vec3 normA(0,0,0), normB(0,0,0);
    //Vec3 meanA(0,0,0), meanB(0,0,0);
    int partAElementsNo=0; int partBElementsNo=0;
    Vec3 meanPosA(0,0,0); Vec3 meanPosB(0,0,0);
    for (int i=0;i<3;i++){
        for (int j=0;j<3;j++){
            for (int k=0;k<3;k++){
                //Mat34 rel(Eigen::Translation<double, 3>(double(i),double(j),double(k))*Quaternion(1,0,0,0));
                if(partA.partIds[i][j][k]!=-1){
                    if (verbose){
                        std::cout << "pA:\n" << partA.neighbourPoses[i][j][k].matrix() << "\n";
                    }
                    normA+=partA.neighbourPoses[i][j][k].matrix().block<3,1>(0,2);
      //              meanA+=partA.neighbourPoses[i][j][k].matrix().block<3,1>(0,3);
                    partAElementsNo++;
                    partASeq.push_back(std::make_pair(partA.neighbourPoses[i][j][k],partA.partIds[i][j][k]));
                    meanPosA+=partA.neighbourPoses[i][j][k].matrix().block<3,1>(0,3);
                }
                if(partB.partIds[i][j][k]!=-1){
                    if (verbose){
                        std::cout << "pB:\n" << partB.neighbourPoses[i][j][k].matrix() << "\n";
                    }
                    normB+=partB.neighbourPoses[i][j][k].matrix().block<3,1>(0,2);
        //            meanB+=partB.neighbourPoses[i][j][k].matrix().block<3,1>(0,3);
                    partBElementsNo++;
                    partBSeq.push_back(std::make_pair(partB.neighbourPoses[i][j][k],partB.partIds[i][j][k]));
                    meanPosB+=partB.neighbourPoses[i][j][k].matrix().block<3,1>(0,3);
                }
            }
        }
    }
    meanPosA/=(double)partASeq.size(); meanPosB/=(double)partBSeq.size();
    if (verbose){
        std::cout << "mean A " << meanPosA.transpose() << "\n";
        std::cout << "mean B " << meanPosB.transpose() << "\n";
    }
    normalizeVector(normA); normalizeVector(normB);
    //meanA/=double(partAElementsNo); meanB/=double(partBElementsNo);
    Mat33 coordPartA = coordinateFromNormal(normA);
    Mat33 coordPartB = coordinateFromNormal(normB);
    Mat34 partApose(Eigen::Translation<double,3>(meanPosA(0),meanPosA(1),meanPosA(2))*coordPartA);
//    Mat34 partApose(Eigen::Translation<double,3>(0,0,0)*coordPartA);
    Mat34 partBpose(Eigen::Translation<double,3>(meanPosB(0),meanPosB(1),meanPosB(2))*coordPartB);
//    Mat34 partBpose(Eigen::Translation<double,3>(0,0,0)*coordPartB);
    if (verbose){
        std::cout << "partApose:\n" << partApose.matrix() << "\n";
        std::cout << "partBpose:\n" << partBpose.matrix() << "\n";
    }
    offset = partApose.inverse()*partBpose;
    /*offset(0,3) = meanPosA(0)-meanPosB(0);
    offset(1,3) = meanPosA(1)-meanPosB(1);
    offset(2,3) = meanPosA(2)-meanPosB(2);*/
    if (verbose){
        std::cout << "offset:\n" << offset.matrix() << "\n";
        getchar();
    }

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
double ViewIndependentPart::nearestNeighbour(Mat34 pose, std::vector<std::pair<Mat34, int>> parts, int& neighbourId){
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

/// compute coordinate system from normal vector
Mat33 ViewIndependentPart::coordinateFromNormal(const Vec3& _normal){
    Vec3 x(1,0,0); Vec3 y;
    Vec3 normal(_normal);
    y = normal.cross(x);
    normalizeVector(y);
    x = y.cross(normal);
    normalizeVector(x);
    Mat33 R;
    R.block(0,0,3,1) = x;
    R.block(0,1,3,1) = y;
    R.block(0,2,3,1) = normal;
    return R;
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
            }
            else if ((partA.partIds[i][j]==-1)||(partB.partIds[i][j]==-1)){
                dotprod=1;
            }
            else{
                dotprod=Filter::distance(filters[partA.partIds[i][j]], filters[partB.partIds[i][j]]);
            }
            ///compute mahalanobis distance
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
                            sum+=dotprod*mahalanobisDist;
                        }
                    }
                    else {// compute Euclidean distance
                        euclDist = sqrt((partA.gaussians[i][j].mean-partB.gaussians[i][j].mean).transpose()*(partA.gaussians[i][j].mean-partB.gaussians[i][j].mean));
                        if (distanceMetric==1||distanceMetric==2){// Euclidean or Mahalanobis
                            sum+=dotprod*euclDist;
                        }
                    }
                }
                /*if (partA.gaussians[i][j].covariance(0,0)>0&&partB.gaussians[i][j].covariance(0,0)>0){
                                std::cout << "meanA: " << partA.gaussians[i][j].mean.transpose() << "\n";
                                std::cout << "meanB: " << partB.gaussians[i][j].mean.transpose() << "\n";
                                std::cout << "covA: " << partA.gaussians[i][j].covariance << "\n";
                                std::cout << "covB: " << partB.gaussians[i][j].covariance << "\n";
                                double mahalanobisDist = sqrt((partA.gaussians[i][j].mean-partB.gaussians[i][j].mean).transpose()*((partA.gaussians[i][j].covariance+partB.gaussians[i][j].covariance)/2.0).inverse()*(partA.gaussians[i][j].mean-partB.gaussians[i][j].mean));
                                std::cout << "mahalanobisDist " << mahalanobisDist << "\n";
                                std::cout << "dist " << sqrt(pow(partA.gaussians[i][j].mean(0)-partB.gaussians[i][j].mean(0),2.0)+pow(partA.gaussians[i][j].mean(1)-partB.gaussians[i][j].mean(1),2.0)+pow(partA.gaussians[i][j].mean(2)-partB.gaussians[i][j].mean(2),2.0)) << "\n";
                                getchar();
                            }*/
            }
            else
                sum+=dotprod;
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

}
