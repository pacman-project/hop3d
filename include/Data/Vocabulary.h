#ifndef DATA_VOCABULARY_H
#define DATA_VOCABULARY_H
#include <iostream>

#include "Data/Defs.h"
#include "Data/Part.h"

namespace hop3d {

/// Vocaulary of a view dependent layer
typedef std::vector<ViewDependentPart> VDLayerVocabulary;
/// Vocaulary of a view independent layer
typedef std::vector<ViewIndependentPart> VILayerVocabulary;

}

#endif /* DATA_VOCABULARY_H */
