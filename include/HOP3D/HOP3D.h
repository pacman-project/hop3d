/** @file HOP3D.h
 *
 * HOP3D interface
 *
 */

#ifndef _HOP3D_H_
#define _HOP3D_H_

#include "../Data/Defs.h"
#include "../Data/Vocabulary.h"
#include "Utilities/observer.h"

namespace hop3d {

/// Statistics builder interface
class HOP3D : public Subject{
public:

    /// HOP3d type
    enum Type {
        /// HOP3D Bham implementation
        HOP3D_BHAM,
    };

    /// overloaded constructor
    HOP3D(const std::string _name, Type _type) :
            name(_name), type(_type) {
    };

    /// Name of the hop3d
    virtual const std::string& getName() const {return name;}

    /// learining from the dataset
    virtual void learn(void) = 0;

    /// Virtual descrutor
    virtual ~HOP3D() {
    }

protected:
    /// hop3d name
    const std::string name;

    /// Map type
    Type type;
};
};

#endif // _HOP3D_H_
