#ifndef DATA_VOCABULARY_H
#define DATA_VOCABULARY_H
#include <iostream>
#include <memory>

class Vocabulary{

public:
    /// Pointer
    typedef std::unique_ptr<Vocabulary> Ptr;

protected:

private:

};

class LayerVocabulary : public Vocabulary {

public:
    /// Pointer
    typedef std::unique_ptr<LayerVocabulary> Ptr;

protected:

private:

};

#endif /* DATA_VOCABULARY_H */
