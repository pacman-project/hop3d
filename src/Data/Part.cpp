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
    //std::cout << "Group size: " << group.size() << "\n";
    std::cout << "ids: ";
    for (int i =0; i<3; i++){
        for (int j =0; j<3; j++){
            for (int k =0; k<3; k++){
                std::cout << this->partIds[i][j][k] << ", ";
            }
        }
    }
    std::cout << "\nParts in group: ";
    for (auto it = group.begin();it!=group.end();it++)
        std::cout << it->id << ", ";
    std::cout << "\n";
    for (size_t i=0; i<gaussians.size();i++){
        for (size_t j=0; j<gaussians.size();j++){
            for (size_t k=0; k<gaussians.size();k++){
                if (partIds[i][j][k]!=-1){
                    std::cout << "mean(" << i << ", " << j << "): ";
                    std::cout << gaussians[i][j][k].mean.transpose() << "\n";
                    std::cout << "covariance(" << i << ", " << j << "):\n";
                    std::cout << gaussians[i][j][k].covariance << "\n";
                }
            }
        }
    }
}

/// compute distance between view-independent parts
double ViewIndependentPart::distance(const ViewIndependentPart& partA, const ViewIndependentPart& partB){
    //find relative position between part A and B
    Mat34 relA2B = partA.pose.inverse()*partB.pose;
    relA2B(0,3)=0; relA2B(1,3)=0; relA2B(2,3)=0;
    double sum=0;
    std::vector<std::pair<Mat34, int>> partASeq;
    std::vector<std::pair<Mat34, int>> partBSeq;
    Vec3 normA(0,0,0), normB(0,0,0);
    //Vec3 meanA(0,0,0), meanB(0,0,0);
    int partAElementsNo=0; int partBElementsNo=0;
    for (int i=0;i<3;i++){
        for (int j=0;j<3;j++){
            for (int k=0;k<3;k++){
                //Mat34 rel(Eigen::Translation<double, 3>(double(i),double(j),double(k))*Quaternion(1,0,0,0));
                if(partA.partIds[i][j][k]!=-1){
                    normA+=partA.neighbourPoses[i][j][k].matrix().block<3,1>(0,2);
      //              meanA+=partA.neighbourPoses[i][j][k].matrix().block<3,1>(0,3);
                    partAElementsNo++;
                    partASeq.push_back(std::make_pair(partA.neighbourPoses[i][j][k],partA.partIds[i][j][k]));
                }
                if(partB.partIds[i][j][k]!=-1){
                    normB+=partB.neighbourPoses[i][j][k].matrix().block<3,1>(0,2);
        //            meanB+=partB.neighbourPoses[i][j][k].matrix().block<3,1>(0,3);
                    partBElementsNo++;
                    partBSeq.push_back(std::make_pair(partB.neighbourPoses[i][j][k],partA.partIds[i][j][k]));
                }
            }
        }
    }
    normalizeVector(normA); normalizeVector(normB);
    //meanA/=double(partAElementsNo); meanB/=double(partBElementsNo);
    Mat33 coordPartA = coordinateFromNormal(normA);
    Mat33 coordPartB = coordinateFromNormal(normB);
    Mat34 partApose(coordPartA*Eigen::Translation<double,3>(0,0,0));
    Mat34 partBpose(coordPartB*Eigen::Translation<double,3>(0,0,0));
    Mat34 transform = partApose.inverse()*partBpose;
    /*std::cout << "setA:\n";
    for (auto & part : partASeq){
        std::cout << "(" << part.first(0,3) << ", " << part.first(1,3) << ", " << part.first(2,3) << "), ";
    }
    std::cout << "\nsetB:\n";
    for (auto & part : partBSeq){
        std::cout << "(" << part.first(0,3) << ", " << part.first(1,3) << ", " << part.first(2,3) << "), ";
    }*/
    if (partASeq.size()<partBSeq.size()){
        for (auto & part : partASeq){
            sum+=nearestNeighbour(part.first*transform,partBSeq);
            //std::cout << "sumB : " << sum << "\n";
        }
        sum/=double(partASeq.size());//divide by the number of matched elements
    }
    else{
        for (auto & part : partBSeq){
            sum+=nearestNeighbour(part.first*(transform.inverse()),partASeq);
            //std::cout << "sumA : "<< sum << "\n";
        }
        sum/=double(partBSeq.size());//divide by the number of matched elements
    }
    return sum;
}

/// compute the min distance to the set of parts
double ViewIndependentPart::nearestNeighbour(Mat34 pose, std::vector<std::pair<Mat34, int>> parts){
    double minDist=std::numeric_limits<double>::max();
    for (auto part : parts){
        double distance = sqrt(pow(pose(0,3)-part.first(0,3),2.0)+pow(pose(1,3)-part.first(1,3),2.0)+pow(pose(2,3)-part.first(2,3),2.0));
        if (distance<minDist)
            minDist = distance;
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
double ViewDependentPart::distance(const ViewDependentPart& partA, const ViewDependentPart& partB, const Filter::Seq& filters){
    if (partA.partIds==partB.partIds)//fast
        return 0;
    double sum=0;
    for (size_t i=0; i<partA.partIds.size();i++){
        for (size_t j=0; j<partA.partIds.size();j++){
            if ((partA.partIds[i][j]==-1)&&(partB.partIds[i][j]==-1)){
                sum+=0;
            }
            else if ((partA.partIds[i][j]==-1)||(partB.partIds[i][j]==-1)){
                sum+=1;
            }
            else{
                sum+=Filter::distance(filters[partA.partIds[i][j]], filters[partB.partIds[i][j]]);
            }
        }
    }
    return sum;
}

/// compute distance between view dependent parts
double ViewDependentPart::distance(const ViewDependentPart& partA, const ViewDependentPart& partB, const ViewDependentPart::Seq& layer2vocabulary, const Filter::Seq& filters){
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
                    sum+=distance(layer2vocabulary[partA.partIds[i][j]], layer2vocabulary[partB.partIds[i][j]], filters);
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
