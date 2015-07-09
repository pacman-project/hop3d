#include "Core/LayerFilterer.h"


int LayerFilterer::nearestNeighbour(const PointCloud &inputPointCloud)
{
    int nn = 4;

       flann::Matrix<float> dataset;
       flann::Matrix<float> query;
       loadPointCloud(dataset,inputPointCloud);
       loadPointCloud(query,inputPointCloud);

       flann::Matrix<int> indices(new int[query.rows*nn], query.rows, nn);
       flann::Matrix<float> dists(new float[query.rows*nn], query.rows, nn);

       // construct an randomized kd-tree index using 4 kd-trees
       flann::Index<flann::L2<float> > index(dataset, flann::KDTreeIndexParams(4));
       index.buildIndex();

       // do a knn search, using 128 checks
       index.knnSearch(query, indices, dists, nn, flann::SearchParams(128));

       std::cout << dataset[0][0] << "; "<< dataset[0][1] << "; "<< dataset[0][2] << std::endl;
       std::cout << dists[0][0] << "; "<< dists[0][1] << "; "<< dists[0][2] << "; "<< dists[0][3] << std::endl;


       delete[] dataset.ptr();
       delete[] query.ptr();
       delete[] indices.ptr();
       delete[] dists.ptr();

       return 0;

}

template<typename T>
void LayerFilterer::loadPointCloud(flann::Matrix<T>& dataset, const PointCloud &inputPointCloud)
{


    unsigned long long dimsOut[2];
    dimsOut[0] = inputPointCloud.PointCloudNormal.size();
    dimsOut[1] = inputPointCloud.PointCloudNormal[0].position.size();
    std::cout << dimsOut[0] << "; "<< dimsOut[1] << std::endl;

    dataset = flann::Matrix<T>(new T[dimsOut[0]*dimsOut[1]], dimsOut[0], dimsOut[1]);

    for(unsigned long  i=0; i < inputPointCloud.PointCloudNormal.size(); i++ ){
        dataset[i][0] = inputPointCloud.PointCloudNormal[i].position(0);
        dataset[i][1] = inputPointCloud.PointCloudNormal[i].position(1);
        dataset[i][2] = inputPointCloud.PointCloudNormal[i].position(2);
    }

//    status = H5Dread(dataset_id, get_hdf5_type<T>(), H5S_ALL, H5S_ALL, H5P_DEFAULT, dataset[0]);
//    CHECK_ERROR(status, "Error reading dataset");

//    H5Sclose(space_id);
//    H5Dclose(dataset_id);
//    H5Fclose(file_id);
}
