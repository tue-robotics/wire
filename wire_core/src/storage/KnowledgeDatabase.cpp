#include "wire/storage/KnowledgeDatabase.h"

#include "wire/core/Property.h"
#include "wire/core/ClassModel.h"
#include "wire/core/Evidence.h"
#include "wire/storage/SemanticObject.h"

#include "problib/conversions.h"

using namespace std;

namespace mhf {

KnowledgeDatabase* KnowledgeDatabase::instance_ = 0;

KnowledgeDatabase& KnowledgeDatabase::getInstance() {
    if (instance_) {
        return *instance_;
    }
    instance_ = new KnowledgeDatabase();
    return *instance_;
}

KnowledgeDatabase::KnowledgeDatabase() : prior_new_(0), prior_existing_(0), prior_clutter_(0) {

}

KnowledgeDatabase::~KnowledgeDatabase() {
    for(map<string, ClassModel*>::iterator it_model = class_models_.begin(); it_model != class_models_.end(); ++it_model) {
        delete it_model->second;
    }
}

void KnowledgeDatabase::addClassModel(const string& class_name, ClassModel* model) {
    class_models_[class_name] = model;
}

const PropertySet& KnowledgeDatabase::getNewPDFs(const std::string& class_name) const {
    return getClassModel(class_name)->getNewPDFs();
}

const PropertySet& KnowledgeDatabase::getClutterPDFs(const std::string& class_name) const {
    return getClassModel(class_name)->getClutterPDFs();
}

const IStateEstimator* KnowledgeDatabase::getEstimator(const std::string& class_name, const Attribute& attribute) const {
    return getClassModel(class_name)->getEstimator(attribute);
}

const pbl::PMF& KnowledgeDatabase::getClassDistribution() const {
    return class_pmf_;
}

void KnowledgeDatabase::setPriorNew(double prior_new) {
    prior_new_ = prior_new;
}

void KnowledgeDatabase::setPriorExisting(double prior_existing) {
    prior_existing_ = prior_existing;
}

void KnowledgeDatabase::setPriorClutter(double prior_clutter) {
    prior_clutter_ = prior_clutter;
}

double KnowledgeDatabase::getPriorNew() const {
    return prior_new_;
}

double KnowledgeDatabase::getPriorExisting() const {
    return prior_existing_;
}

double KnowledgeDatabase::getPriorClutter() const {
    return prior_clutter_;
}

const std::map<std::string, ClassModel*>& KnowledgeDatabase::getClassModels() const {
    return class_models_;
}

const ClassModel* KnowledgeDatabase::getClassModel(const std::string& class_name) const {
    map<string, ClassModel*>::const_iterator it_model = class_models_.find(class_name);
    if (it_model != class_models_.end()) {
        return it_model->second;
    }

    it_model = class_models_.find("object");
    assert(it_model != class_models_.end());

    return it_model->second;
}

double KnowledgeDatabase::getProbabilityNew(const Evidence& z) {

    const Property* class_prop = z.getProperty("class_label");

    double likelihood = 0;
    double total_prob = 0;

    if (class_prop) {
        // we have information about the class distribution
        const pbl::PMF* class_pmf = pbl::PDFtoPMF(class_prop->getValue());

        vector<double> class_probs;
        class_pmf->getProbabilities(class_probs);

        vector<string> class_names;
        class_pmf->getValues(class_names);

        likelihood = 0;
        for(unsigned int i = 0; i < class_probs.size(); ++i) {
            const ClassModel* class_model = getClassModel(class_names[i]);

            if (class_model) {
                likelihood += class_probs[i] * class_model->getNewPDFs().getLikelihood(z);
                total_prob += class_probs[i];
            }
        }
    }

    const ClassModel* default_model = getClassModel("object");
    assert(default_model != 0);

    likelihood += (1 - total_prob) * default_model->getNewPDFs().getLikelihood(z);

    double p_new = getPriorNew() * likelihood;

    //cout << "p_new = " << getPriorNew() << " * " << likelihood << " = " << p_new << endl;

    return p_new;
}

double KnowledgeDatabase::getProbabilityClutter(const Evidence& z) {

    const Property* class_prop = z.getProperty("class_label");

    double likelihood = 0;
    double total_prob = 0;

    if (class_prop) {
        // we have information about the class distribution
        const pbl::PMF* class_pmf = pbl::PDFtoPMF(class_prop->getValue());

        vector<double> class_probs;
        class_pmf->getProbabilities(class_probs);

        vector<string> class_names;
        class_pmf->getValues(class_names);

        likelihood = 0;
        for(unsigned int i = 0; i < class_probs.size(); ++i) {
            const ClassModel* class_model = getClassModel(class_names[i]);

            if (class_model) {
                likelihood += class_probs[i] * class_model->getClutterPDFs().getLikelihood(z);
                total_prob += class_probs[i];
            }
        }
    }

    const ClassModel* default_model = getClassModel("object");
    assert(default_model != 0);

    likelihood += (1 - total_prob) * default_model->getClutterPDFs().getLikelihood(z);

    double p_clutter = getPriorClutter() * likelihood;

    //cout << "p_new = " << getPriorNew() << " * " << likelihood << " = " << p_new << endl;

    return p_clutter;
}

double KnowledgeDatabase::getProbabilityExisting(const Evidence& z, const SemanticObject& obj) {
    // calculate prior (prior probability that target generates a detection)
    double prior = getPriorExisting();

    // calculate likelihood (likelihood that measurements originates from the target)
    double likelihood = obj.getLikelihood(z);

    //cout << "p_existing = " << prior << " * " << likelihood << " = " << prior * likelihood << endl;

    return prior * likelihood;
}

vector<Property> KnowledgeDatabase::inferProperties(const PropertySet& prop_set, vector<Attribute> attribs) const {
    const Property* class_prop = prop_set.getProperty("class_label");

    const ClassModel* most_prob_class_model = 0;
    if (class_prop) {
        string most_prob_class;
        class_prop->getValue().getExpectedValue(most_prob_class);
        most_prob_class_model = getClassModel(most_prob_class);
    } else {
        most_prob_class_model = getClassModel("object");
    }

    vector<Property> inferred_props;
    for(vector<Attribute>::iterator it_att = attribs.begin(); it_att != attribs.end(); ++it_att) {
        const Property* prop = most_prob_class_model->getNewPDFs().getProperty(*it_att);
        assert(prop);
        inferred_props.push_back(*prop);
    }

    return inferred_props;
}

}
