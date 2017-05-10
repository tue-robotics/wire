/*
 * PropertySet.cpp
 *
 *  Created on: March, 2011
 *  Author: Jos Elfring, Sjoerd van den Dries
 *  Affiliation: Eindhoven University of Technology
 */

#include "wire/core/PropertySet.h"
#include "wire/core/Property.h"
#include "wire/models/FixedState.h"

using namespace std;

namespace mhf {

int PropertySet::N_PROPERTY_SET = 0;

PropertySet::PropertySet(Time timestamp) : timestamp_(timestamp) {
    ++N_PROPERTY_SET;
}

PropertySet::~PropertySet() {
    for(map<Attribute, Property*>::iterator it = properties_.begin(); it != properties_.end(); ++it) {
        delete it->second;
    }

    --N_PROPERTY_SET;
}

PropertySet::PropertySet(const PropertySet& orig) : timestamp_(orig.timestamp_)  {
    for(map<Attribute, Property*>::const_iterator it = orig.properties_.begin(); it != orig.properties_.end(); ++it) {
        properties_[it->first] = it->second->clone();
    }
}

PropertySet* PropertySet::clone() const {
    return new PropertySet(*this);
}

void PropertySet::addProperty(const Attribute& att, const pbl::PDF& value) {
    map<Attribute, Property*>::iterator it = properties_.find(att);
    if (it == properties_.end()) {
        properties_[att] = new Property(att, FixedState(value));
    } else {
        delete it->second;
        it->second = new Property(att, FixedState(value));
    }
}

void PropertySet::addProperty(const string& att, const pbl::PDF& value) {
    addProperty(AttributeConv::attribute(att), value);
}

void PropertySet::addProperty(const Attribute& att, const IStateEstimator& estimator) {
    map<Attribute, Property*>::iterator it = properties_.find(att);
    if (it == properties_.end()) {
        properties_[att] = new Property(att, estimator);
    } else {
        delete it->second;
        it->second = new Property(att, estimator);
    }
}

void PropertySet::addProperty(Property* property) {
    map<Attribute, Property*>::iterator it = properties_.find(property->getAttribute());
    if (it == properties_.end()) {
        properties_[property->getAttribute()] = property;
    } else {
        delete it->second;
        it->second = property;
    }
}

const Property* PropertySet::getProperty(const Attribute& attribute) const {
    map<Attribute, Property*>::const_iterator it = properties_.find(attribute);
    if (it != properties_.end()) {
        return it->second;
    }
    return 0;
}

Property* PropertySet::getProperty(const Attribute& attribute) {
    map<Attribute, Property*>::iterator it = properties_.find(attribute);
    if (it != properties_.end()) {
        return it->second;
    }
    return 0;
}

const Property* PropertySet::getProperty(const std::string& attribute) const {
    return getProperty(AttributeConv::attribute(attribute));
}

void PropertySet::propagate(const Time& time) {
    if (fabs(time - timestamp_) < 0.001) {
        return;
    }

    for(map<Attribute, Property*>::iterator it = properties_.begin(); it != properties_.end(); ++it) {
        it->second->propagate(time);
    }

    timestamp_ = time;
}

void PropertySet::update(const pbl::PDF& z, const Time& time) {
    assert(false);
}

void PropertySet::reset() {
    for(map<Attribute, Property*>::iterator it = properties_.begin(); it != properties_.end(); ++it) {
        it->second->reset();
    }
}

const pbl::PDF& PropertySet::getValue() const {
    assert(false);
}

double PropertySet::getLikelihood(const PropertySet& P) const {
    double likelihood = 1;

    const map<Attribute, Property*>& other_props = P.properties_;

    for(map<Attribute, Property*>::const_iterator it = other_props.begin(); it != other_props.end(); ++it) {

        const Attribute& attribute = it->first;
        const Property* other_prop = it->second;

        const Property* this_prop = getProperty(attribute);

        if (this_prop) {

            /*
            std::cout << "Attribute: " << AttributeConv::attribute_str(it->first) << std::endl;
            std::cout << "PDF mine:  " << it_prop->second->getValue().toString() << std::endl;
            std::cout << "PDF other: " << it->second->getValue().toString() << std::endl;
            std::cout << "Likelihood: " << it_prop->second->getLikelihood(it->second->getValue()) << std::endl;
            */

            likelihood *= this_prop->getLikelihood(other_prop->getValue());
        } else {
            printf("Error during likelihood calculation: property '%s' is not in property set.\n", AttributeConv::attribute_str(attribute).c_str());

            printf("This (%p) constains:\n", this);
            for(map<Attribute, Property*>::const_iterator it = properties_.begin(); it != properties_.end(); ++it) {
                printf(" - %s\n", AttributeConv::attribute_str(it->first).c_str());
            }

            printf("Other (%p) constains:\n", &P);
            for(map<Attribute, Property*>::const_iterator it = other_props.begin(); it != other_props.end(); ++it) {
                printf(" - %s\n", AttributeConv::attribute_str(it->first).c_str());
            }
        }

    }

    return likelihood;
}

const std::map<Attribute, Property*>& PropertySet::getPropertyMap() const {
    return properties_;
}

Time PropertySet::getTimestamp() const {
    return timestamp_;
}

string PropertySet::toString() const {
    stringstream s;
    for(map<Attribute, Property*>::const_iterator it = properties_.begin(); it != properties_.end(); ++it) {
        s << " - " << AttributeConv::attribute_str(it->first) << endl;
    }
    return s.str();
}

}
