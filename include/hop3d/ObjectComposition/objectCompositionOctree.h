/** @file objectCompositionOctree.h
 *
 * implementation - object Composition Octree
 *
 */

#ifndef OBJECT_COMPOSITION_OCTREE_H_INCLUDED
#define OBJECT_COMPOSITION_OCTREE_H_INCLUDED

#include "hop3d/ObjectComposition/objectComposition.h"
#include "tinyXML/tinyxml2.h"
#include "octree/octree.h"
#include "hop3d/Data/Cloud.h"

namespace hop3d {
    /// create a single part selector
    ObjectComposition* createObjectCompositionOctree(void);
    /// create a single part selector
    ObjectComposition* createObjectCompositionOctree(std::string config);

/// Object Composition implementation
class ObjectCompositionOctree: public ObjectComposition {
public:
    /// Pointer
    typedef std::unique_ptr<ObjectCompositionOctree> Ptr;
    typedef std::shared_ptr< Octree<ViewIndependentPart> > OctreePtr;
    typedef std::shared_ptr< Octree<hop3d::PointCloud> > OctreeCloudPtr;

    /// Construction
    ObjectCompositionOctree(void);

    /// Construction
    ObjectCompositionOctree(std::string config);

    /// update composition from octets (words from last view-independent layer's vocabulary)
    //void update(int layerNo, const std::vector<ViewDependentPart>& parts, const Mat34& cameraPose);

    /// update voxel grid which contains point and normals
    void updatePCLGrid(const std::vector<ViewDependentPart>& parts, const Mat34& cameraPose);

    /// get clusters of parts id stored in octree (one cluster per voxel)
    void getClusters(int layerNo, std::vector< std::set<int>>& clusters);

    /// create next layer vocabulary
    void createNextLayerVocabulary(int destLayerNo, std::vector<ViewIndependentPart>& vocabulary);

    /// get octree in layer layerNo
    void getParts(int layerNo, std::vector<ViewIndependentPart>& parts);

    /// update ids in the octree using new vocabulary
    //void updateIds(int layerNo, const std::vector<ViewIndependentPart>& vocabulary, Hierarchy& hierarchy);

    /// get set of ids for the given input point
    void getPartsIds(const Vec3& point, int overlapNo, std::vector<int>& ids) const;

    /// get realisations ids
    void getRealisationsIds(const Vec3& point, int overlapNo, std::vector<int>& ids) const;

    /// upodate voxel poses using new vocabulary
    void updateVoxelsPose(int layerNo, const std::vector<ViewIndependentPart>& vocabulary, const Hierarchy& hierarchy);

    /// find new parts in composition
    void identifyParts(int layerNo, const std::vector<ViewIndependentPart>& vocabulary, const Hierarchy& hierarchy, double distThreshold, std::vector<ViewIndependentPart>& oldParts, std::vector<ViewIndependentPart>& newParts);

    /// get parts realisations
    void getPartsRealisation(int layerNo, int overlapNo, std::vector<ViewIndependentPart::Part3D>& partsViewTmp) const;

    /// set realisation counter
    static void setRealisationCounter(int startRealisationId);

    /// Insertion operator
    friend std::ostream& operator<<(std::ostream& os, const ObjectCompositionOctree& object){
        os << static_cast<unsigned int>(object.type) << " ";
        os << object.octrees.size() << " ";//layers no
        for (auto& layer : object.octrees){
            os << layer.size() << " ";// overlaps no
            for (auto& octree : layer){
                int voxelsNo=0;
                for (int idX=0; idX<octree->size(); idX++)///to do z-slicing
                    for (int idY=0; idY<octree->size(); idY++)
                        for (int idZ=0; idZ<octree->size(); idZ++)
                            if (octree->at(idX,idY,idZ).parts.size()>0||octree->at(idX,idY,idZ).id!=-1)
                                voxelsNo++;
                os << voxelsNo << " ";
                for (int idX=0; idX<octree->size(); idX++){///to do z-slicing
                    for (int idY=0; idY<octree->size(); idY++){
                        for (int idZ=0; idZ<octree->size(); idZ++){
                            if (octree->at(idX,idY,idZ).parts.size()>0||octree->at(idX,idY,idZ).id!=-1){
                                os << idX << " " << idY << " " << idZ << " ";
                                os << octree->at(idX,idY,idZ) << "\n";
                            }
                        }
                    }
                }
            }
        }
        return os;
    }

    // Extraction operator
    friend std::istream& operator>>(std::istream& is, ObjectCompositionOctree& object){
        unsigned int type;
        is >> type;
        object.type = static_cast<ObjectComposition::Type>(type);
        int layersNo;
        is >> layersNo;
        for (int layNo=0;layNo<layersNo;layNo++){
            int overlapsNo;
            is >> overlapsNo;
            for (int ovNo=0;ovNo<overlapsNo;ovNo++){
                int voxelsNo;
                is >> voxelsNo;
                for (int j=0;j<voxelsNo;j++){
                    int x,y,z;
                    is >> x >> y >> z;
                    is >> (*object.octrees[layNo][ovNo])(x,y,z);
                }
            }
        }
        return is;
    }

    /// Destruction
    ~ObjectCompositionOctree(void);

    class Config{
      public:
        Config() : verbose(0){
        }

        Config(std::string configFilename);
        public:
            /// Verbose
            int verbose;
            /// voxel size
            double voxelSize;
            /// Clusters no -- second layer
            int voxelsNo;
            /// max angle between two parts (normals), if current angle is bigger than max second cluster is created
            double maxAngleGrid;
            /// Point cloud grid -- size
            double voxelSizeGrid;
            /// Point cloud grid -- voxelsNo
            int voxelsNoGrid;
            /// min number of patches in the cloud
            int minPatchesNo;
            /// config GICP
            ConfigGICP configGICP;
    };

private:
    ///Configuration of the module
    Config config;
    /// Octree (layerNo->overlapNo)
    std::vector<std::vector<OctreePtr>> octrees;
    /// points with normals grid
    OctreeCloudPtr octreeGrid;
    /// part realisation counter
    static int partRealisationsCounter;

    /// compute rotation matrix from normal vector ('y' axis is vetical)
    //void normal2rot(const Vec3& normal, Mat33& rot);

    /// assign neighbouring parts to new part
    int assignPartNeighbours(ViewIndependentPart& partVoxel, const Hierarchy& hierarchy, int layerNo, int overlapNo, int x, int y, int z);

    /// assign neighbouring parts to new part
    int createFirstLayerPart(ViewIndependentPart& newPart, int overlapNo, int x, int y, int z);

    /// assign neighbouring parts to new part
    int createNextLayerPart(const Hierarchy& hierarchy, int destLayerNo, int overlapNo, ViewIndependentPart& newPart, int x, int y, int z);

    /// assign neighbouring parts to new part
    int createNextLayerPart(ViewIndependentPart& newPart, int layerNo, int overlapNo, int x, int y, int z);

    /// find part in the vocabulary and return new id
    int findIdInVocabulary(const ViewIndependentPart& part, const std::vector<ViewIndependentPart>& vocabulary);

    /// convert global coordinates to octree coordinates
    void toCoordinate(double pos, int& coord, int layerNo) const;

    /// convert global coordinates to octree coordinates
    void toCoordinate(int coordPrevLayer, int& coord, int overlapNo) const;

    /// convert global coordinates to octree coordinates
    void toCoordinate(double pos, int& coord, int layerNo, int overlapNo) const;

    /// convert global coordinates to octree coordinates
    void toCoordinatePCLGrid(double pos, int& coord) const;

    /// convert octree coordinates to global coordinates
    void fromCoordinate(int coord, double& pos, int layerNo) const;

    /// convert octree coordinates to global coordinates
    void fromCoordinate(int coord, double& pos, int layerNo, int overlapNo) const;

    /// convert octree coordinates to global coordinates
    void fromCoordinatePCLGrid(int coord, double& pos) const;

    /// filter voxel grid
    void filterPCLGrid(int destLayerNo);

    /// compute mean value of normal and position
    PointNormal computeMeanPosNorm(PointCloud cloud);

    /// create first layer vocabulary from voxel grid
    void createFirstLayer(std::vector<ViewIndependentPart>& vocabulary);

    /// get all patches from the voxel grid
    int getCorrespondingPatches(hop3d::PointCloud& patches, int centerX, int centerY, int centerZ, int coeff);
};
}
#endif // OBJECT_COMPOSITION_OCTREE_H_INCLUDED
