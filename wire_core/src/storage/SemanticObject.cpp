/*
 * MyObject.cpp
 *
 *  Created on: Apr 27, 2011
 *      Author: sdries
 */

#include "wire/storage/SemanticObject.h"
#include "wire/storage/KnowledgeDatabase.h"

#include "wire/core/Evidence.h"
#include "wire/core/ClassModel.h"
#include "wire/core/Property.h"
#include "wire/logic/Assignment.h"
#include "wire/logic/Hypothesis.h"

#include <problib/conversions.h>

using namespace std;

namespace mhf {

int SemanticObject::N_SEMANTICOBJECT = 0;

SemanticObject::SemanticObject(long ID) : ID_(ID), expected_class_("") {
    ++N_SEMANTICOBJECT;
}

SemanticObject::SemanticObject(const SemanticObject& orig)
    : PropertySet(orig), ID_(orig.ID_), expected_class_(orig.expected_class_) {
    ++N_SEMANTICOBJECT;
}

SemanticObject::~SemanticObject() {
    --N_SEMANTICOBJECT;
}

void SemanticObject::init(const Evidence& z) {
    update(z);
}

double SemanticObject::getLikelihood(const PropertySet& ev) const {

    //propagate(ev.getTimeStamp());

    double likelihood = 1;

    vector<Attribute> need_to_deduce;

    const map<Attribute, Property*>& ev_props = ev.getPropertyMap();
    for(map<Attribute, Property*>::const_iterator it = ev_props.begin(); it != ev_props.end(); ++it) {
        const Attribute& attribute = it->first;
        const Property* ev_prop = it->second;

        const Property* this_prop = getProperty(attribute);

        if (this_prop) {
             likelihood *= this_prop->getLikelihood(ev_prop->getValue());
        } else {
            need_to_deduce.push_back(attribute);
        }
    }

    vector<Property> deduced_props = KnowledgeDatabase::getInstance().inferProperties(*this, need_to_deduce);

    for(vector<Property>::iterator it_prop = deduced_props.begin(); it_prop != deduced_props.end(); ++it_prop) {
        const Property* ev_prop = ev.getProperty(it_prop->getAttribute());
        assert(ev_prop);
        likelihood *= it_prop->getLikelihood(ev_prop->getValue());
    }

    // cout << "    Likelihood existing = " << likelihood << endl;

    return likelihood;
}

void SemanticObject::update(const Evidence& ev) {

    propagate(ev.getTimestamp());

    // first update class property

    Attribute class_att = AttributeConv::attribute("class_label");
    const Property* ev_class = ev.getProperty(class_att);

    if (ev_class) {
        Property* my_class = getProperty(class_att);

        if (my_class) {
            my_class->update(ev_class->getValue(), ev.getTimestamp());
        } else {
            const IStateEstimator* prototype = getExpectedObjectModel().getEstimator(class_att);
            if (prototype) {
                Property* new_prop = new Property(class_att, *prototype);
                new_prop->update(ev_class->getValue(), ev.getTimestamp());
                addProperty(new_prop);
            } else {
                printf("Could not find prototype estimator for attribute '%s'\n", AttributeConv::attribute_str(class_att).c_str());
            }
        }
    }

    bool class_changed = false;
    const ClassModel& class_model = getExpectedObjectModel();

    if (expected_class_ != class_model.getModelName()) {
        expected_class_ = class_model.getModelName();
        class_changed = true;
    }

    // then update rest

    for(map<Attribute, Property*>::const_iterator it = ev.getPropertyMap().begin(); it != ev.getPropertyMap().end(); ++it) {

        const Attribute& attribute = it->first;
        const Property* ev_prop = it->second;

        if (attribute != class_att) {
            Property* my_prop = getProperty(attribute);

            if (my_prop && !class_changed) {
                my_prop->update(ev_prop->getValue(), ev.getTimestamp());
                //cout << "Updating " << AttributeConv::attribute_str(attribute) << " with " << ev_prop->getValue().toString() << endl;
                //cout << "Result: " << my_prop->toString() << endl;
            } else {
                const IStateEstimator* prototype = getExpectedObjectModel().getEstimator(attribute);
                if (prototype) {
                    Property* new_prop = new Property(attribute, *prototype);
                    new_prop->update(ev_prop->getValue(), ev.getTimestamp());
                    addProperty(new_prop);
                } else {
                    printf("Could not find prototype estimator for attribute '%s'\n", AttributeConv::attribute_str(it->first).c_str());
                }
            }
        }
    }

}

SemanticObject* SemanticObject::clone() const {
    return new SemanticObject(*this);
}

const ClassModel& SemanticObject::getExpectedObjectModel() const {
    const Property* class_prop = getProperty("class_label");

    string class_name;

    if (class_prop) {
        class_prop->getValue().getExpectedValue(class_name);
    } else {
        class_name = "object";
    }

    return *KnowledgeDatabase::getInstance().getClassModel(class_name);
}

void SemanticObject::addPotentialAssignment(const Evidence& ev, double probability) {
    Assignment* assignment = new Assignment(Assignment::EXISTING, &ev, this, probability);

    for(set<Hypothesis*>::iterator it_hyp = parent_hypotheses_.begin(); it_hyp != parent_hypotheses_.end(); ++it_hyp) {
        Hypothesis& hyp = **it_hyp;
        hyp.addPotentialAssignment(assignment);
    }

    // TODO: check if object now has potential assignments between hypotheses of different clusters. If so, those trees need to merge.
}

ObjectID SemanticObject::getID() const {
    return ID_;
}

void SemanticObject::addToHypothesis(Hypothesis* hyp) {
    parent_hypotheses_.insert(hyp);
}

void SemanticObject::removeFromHypothesis(Hypothesis* hyp) {
    parent_hypotheses_.erase(hyp);
}

unsigned int SemanticObject::getNumParentHypotheses() const {
    return parent_hypotheses_.size();
}

}
