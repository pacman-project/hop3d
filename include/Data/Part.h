#ifndef DATA_PART_H
#define DATA_PART_H
#include <iostream>
#include <memory>
#include <vector>

#include "Data/Defs.h"



namespace hop3d {

class Part{
    friend class LayerVocabulary;

public:
    typedef std::vector<Part> Seq;
    int setPartId(hop3d::U64 id);
    hop3d::U64 getPartId() const;
protected:
    /// Pointer
    typedef std::unique_ptr<Part> Ptr;
    hop3d::U64 Id;
    hop3d::U8 LayerId;
    hop3d::U64 Central;
    std::vector<hop3d::U64> Members;
private:

};

class FirstLayerPart : public Part{
public:
    typedef std::vector<FirstLayerPart> Seq;
    int setNormal(hop3d::Vec3 normal);
    hop3d::Vec3 getNormal() const;
protected:
    hop3d::Vec3 Normals;
private:

};

class PartRealization : public Part {

public:
    /// Pointer
    typedef std::unique_ptr<PartRealization> Ptr;

    hop3d::Vec3 position;
    hop3d::ReferenceFrame referenceFrame;
    hop3d::F32 activation;
    hop3d::I8 scale;
    typedef std::vector<PartRealization>  Seq;

protected:

private:

};

}
#endif /* DATA_PART_H */
