#ifndef PROPERTY_SET_H_
#define PROPERTY_SET_H_

#include "wire/core/datatypes.h"
#include "wire/core/IStateEstimator.h"

#include "problib/pdfs/PDF.h"

namespace mhf {

class Property;

class PropertySet : public IStateEstimator {
public:

    static int N_PROPERTY_SET;

    PropertySet(Time timestamp = 0);

    PropertySet(const PropertySet& orig);

    virtual ~PropertySet();

    PropertySet* clone() const;

    void addProperty(const Attribute& attribute, const pbl::PDF& value);

    void addProperty(const std::string& attribute, const pbl::PDF& value);

    void addProperty(const Attribute& attribute, const IStateEstimator& estimator);    

    const Property* getProperty(const Attribute& attribute) const;

    const Property* getProperty(const std::string& attribute) const;

    void propagate(const Time& time);

    void update(const pbl::PDF& z, const Time& time);

    void reset();

    const pbl::PDF& getValue() const;

    virtual double getLikelihood(const PropertySet& P) const;

    const std::map<Attribute, Property*>& getPropertyMap() const;

    Time getTimestamp() const;

    std::string toString() const;

protected:

    Time timestamp_;

    void addProperty(Property* property);

    Property* getProperty(const Attribute& attribute);

private:

    std::map<Attribute, Property*> properties_;

};

}

#endif /* PROPERTY_SET_H_ */
