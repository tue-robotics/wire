/*
 * ObjectModelParser.h
 *
 *  Created on: Mar 6, 2012
 *      Author: sdries
 */

#ifndef OBJECTMODELPARSER_H_
#define OBJECTMODELPARSER_H_

#include "wire/core/IStateEstimator.h"

// xml parser
#include <tinyxml.h>

#include <pluginlib/class_loader.h>

#include <map>

namespace mhf {

class ClassModel;
class KnowledgeDatabase;

class ObjectModelParser {

public:

    ObjectModelParser(const std::string& filename);

    virtual ~ObjectModelParser();

    bool parse(KnowledgeDatabase& obj_models);

    std::string getErrorMessage() const;

protected:

    std::string filename_;

    std::stringstream parse_errors_;

    pluginlib::ClassLoader<IStateEstimator>* object_model_loader_;

    std::string getPropertyValue(const TiXmlElement* elem, std::string prop_name, double& value, std::stringstream& error, bool optional = false);

    bool getAttributeValue(const TiXmlElement* elem, std::string att_name, std::string& att_value, std::stringstream& error);

    bool getAttributeValue(const TiXmlElement* elem, std::string att_name, double& att_value, std::stringstream& error);

    bool hasAttributeValue(const TiXmlElement* elem, std::string att_name, std::string att_value);

    bool parseStateEstimator(ClassModel* obj_model, const TiXmlElement* elem, std::stringstream& error);

    pbl::PDF* parsePDF(const TiXmlElement* elem, std::stringstream& error);

    bool getStateEstimatorParameter(const TiXmlElement* elem, const std::string& param_name, double& value);

};

}

#endif /* OBJECTMODELPARSER_H_ */
