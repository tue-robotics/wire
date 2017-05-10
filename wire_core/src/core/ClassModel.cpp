/*
 * ObjectModel.cpp
 *
 *  Created on: Mar 18, 2011
 *      Author: sdries
 */

#include "wire/core/ClassModel.h"
#include "wire/core/Property.h"

using namespace std;

namespace mhf {

ClassModel::ClassModel(const std::string& model_name) : model_name_(model_name) {

}

ClassModel::ClassModel(const ClassModel& orig)
    : new_pdfs_(orig.new_pdfs_), clutter_pdfs_(orig.clutter_pdfs_), estimators_(orig.estimators_),
      model_name_(orig.model_name_) {
}

ClassModel::~ClassModel() {

}

void ClassModel::setNewPDF(const Attribute& attribute, const pbl::PDF& pdf) {
    new_pdfs_.addProperty(attribute, pdf);
}

void ClassModel::setClutterPDF(const Attribute& attribute, const pbl::PDF& pdf) {
    clutter_pdfs_.addProperty(attribute, pdf);
}

void ClassModel::setEstimator(const Attribute& attribute, const IStateEstimator& estimator) {
    estimators_.addProperty(attribute, estimator);
}

void ClassModel::setModelName(const std::string& name)  {
    model_name_ = name;
}

const std::string& ClassModel::getModelName() const {
    return model_name_;
}

const IStateEstimator* ClassModel::getEstimator(const Attribute& attribute) const {
    const Property* prop = estimators_.getProperty(attribute);
    if (prop) {
        return &prop->getEstimator();
    }
    return 0;
}

const PropertySet& ClassModel::getNewPDFs() const {
    return new_pdfs_;
}

const PropertySet& ClassModel::getClutterPDFs() const {
    return clutter_pdfs_;
}

}
