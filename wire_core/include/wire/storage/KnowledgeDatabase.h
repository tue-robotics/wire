/*
 * KnowledgeDatabase.h
 *
 *  Created on: September, 2012
 *      Author: sdries
 */

#ifndef WM_KNOWLEDGEDATABASE_H_
#define WM_KNOWLEDGEDATABASE_H_

#include <problib/pdfs/PMF.h>
#include "wire/core/ClassModel.h"

#include <map>

namespace mhf {

class Evidence;
class SemanticObject;

class KnowledgeDatabase {

public:

    static KnowledgeDatabase& getInstance();

    virtual ~KnowledgeDatabase();

    void addClassModel(const std::string& class_name, ClassModel* model);

    const PropertySet& getNewPDFs(const std::string& class_name) const;

    const PropertySet& getClutterPDFs(const std::string& class_name) const;

    const IStateEstimator* getEstimator(const std::string& class_name, const Attribute& attribute) const;

    const pbl::PMF& getClassDistribution() const;

    const std::map<std::string, ClassModel*>& getClassModels() const;

    const ClassModel* getClassModel(const std::string& class_name) const;

    void setPriorNew(double prior_new);

    void setPriorExisting(double prior_existing);

    void setPriorClutter(double prior_clutter);

    double getPriorNew() const;

    double getPriorExisting() const;

    double getPriorClutter() const;

    double getProbabilityNew(const Evidence& z);

    double getProbabilityClutter(const Evidence& z);

    double getProbabilityExisting(const Evidence& z, const SemanticObject& obj);

    std::vector<Property> inferProperties(const PropertySet& prop_set, std::vector<Attribute>) const;

protected:

    KnowledgeDatabase();

    static KnowledgeDatabase* instance_;

    double prior_new_;

    double prior_existing_;

    double prior_clutter_;

    pbl::PMF class_pmf_;

    std::map<std::string, ClassModel*> class_models_;

};

}

#endif /* PROPERTYDATABASE_H_ */
