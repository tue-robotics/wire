/*
 * ObjectModelParser.cpp
 *
 *  Created on: Mar 6, 2012
 *      Author: sdries
 */

#include "wire/util/ObjectModelParser.h"
#include "wire/core/ClassModel.h"
#include "wire/storage/KnowledgeDatabase.h"

#include <problib/pdfs/Uniform.h>
#include <problib/pdfs/PMF.h>

namespace mhf {

ObjectModelParser::ObjectModelParser(const std::string& filename, const std::string& plugin_attr) : filename_(filename),
    object_model_loader_(new pluginlib::ClassLoader<IStateEstimator>("wire_core", "IStateEstimator", plugin_attr)) {

}

ObjectModelParser::~ObjectModelParser() {
    // TODO: change ownership of the class_loader, as it must persist throughout the whole lifetime of the world model
    //delete object_model_loader_;
}

std::string ObjectModelParser::getErrorMessage() const {
    return parse_errors_.str();
}

std::string ObjectModelParser::getPropertyValue(const tinyxml2::XMLElement* elem, std::string prop_name, double& value, std::stringstream& error, bool optional) {
    const tinyxml2::XMLElement* p = elem->FirstChildElement(prop_name.c_str());
    if (p) {
        return p->Attribute("value", std::to_string(value).c_str());
    }
    if (!optional) {
        error << "Could not find property '" << prop_name << "' of element '" << elem->Value() << "'" << std::endl;
    }
    return "";
}

bool ObjectModelParser::getAttributeValue(const tinyxml2::XMLElement* elem, std::string att_name, std::string& att_value, std::stringstream& error) {
    if (!elem) return false;

    const char* value = elem->Attribute(att_name.c_str());
    if (!value) {
        error << "Could not find attribute '" << att_name << "' of element '" << elem->Value() << "'" << std::endl;
        return false;
    }

    att_value = value;
    return true;
}

bool ObjectModelParser::getAttributeValue(const tinyxml2::XMLElement* elem, std::string att_name, double& att_value, std::stringstream& error) {
    if (!elem) return false;

    const char* value = elem->Attribute(att_name.c_str());
    if (!value) {
        error << "Could not find attribute '" << att_name << "' of element '" << elem->Value() << "'" << std::endl;
        return false;
    }

    att_value = std::atof(value);
    return true;
}

bool ObjectModelParser::hasAttributeValue(const tinyxml2::XMLElement* elem, std::string att_name, std::string att_value) {
    if (!elem) return false;

    const char* value = elem->Attribute(att_name.c_str());
    if (!value) return false;

    std::string valueStr = value;
    return (att_value == valueStr);
}

pbl::PDF* ObjectModelParser::parsePDF(const tinyxml2::XMLElement* pdf_elem, std::stringstream& error) {
    const char* pdf_type = pdf_elem->Attribute("type");
    if (pdf_type) {
        if (std::string(pdf_type) == "uniform") {
            double dim = 0;
            double density = 0;
            if (getAttributeValue(pdf_elem, "dimensions", dim, error)
                    && getAttributeValue(pdf_elem, "density", density, error)) {
                return new pbl::Uniform((int)dim, density);
            }
        } else if (std::string(pdf_type) == "discrete") {
            double domain_size;
            if (getAttributeValue(pdf_elem, "domain_size", domain_size, error)) {
                return new pbl::PMF((int)domain_size);
            }
        } else {
            error << "Unknown pdf type: " << pdf_type << std::endl;
        }
    } else {
        error << "PDF specification should contain 'type' attribute" << std::endl;
    }
    return 0;
}

bool ObjectModelParser::parseStateEstimator(ClassModel* obj_model, const tinyxml2::XMLElement* elem, std::stringstream& error) {

    // check behavior model's attribute and model type
    std::string attribute_name, model_type;
    if (!getAttributeValue(elem, "attribute", attribute_name, error)
            | !getAttributeValue(elem, "model", model_type, error)) {
        return false;
    }

    Attribute attribute = AttributeConv::attribute(attribute_name);

    if (!object_model_loader_->isClassAvailable(model_type)){
        std::vector<std::string> classes = object_model_loader_->getDeclaredClasses();
        for(unsigned int i = 0; i < classes.size(); ++i){
            if(model_type == object_model_loader_->getName(classes[i])){
                //if we've found a match... we'll get the fully qualified name and break out of the loop
                ROS_WARN("Planner specifications should now include the package name. You are using a deprecated API. Please switch from %s to %s in your yaml file.",
                        model_type.c_str(), classes[i].c_str());
                model_type = classes[i];
                break;
            }
        }
    }

    IStateEstimator* estimator;

    if (object_model_loader_->isClassAvailable(model_type)) {
        estimator = object_model_loader_->createUnmanagedInstance(model_type)->clone();
    } else {
        error << "Unknown model: " << model_type << std::endl;
        return false;
    }

    // set estimator parameters
    const tinyxml2::XMLElement* param = elem->FirstChildElement("param");
    while (param) {
        const char* param_name = param->Attribute("name");

        if (param_name) {
            bool v_bool;
            int v_int;
            double v_double;

            bool set_param_ok = true;
            if (param->QueryDoubleAttribute("value", &v_double) == tinyxml2::XML_SUCCESS) {
                set_param_ok = estimator->setParameter(std::string(param_name), v_double);
            } else if (param->QueryIntAttribute("value", &v_int) == tinyxml2::XML_SUCCESS) {
                set_param_ok = estimator->setParameter(std::string(param_name), (double)v_int);
            } else if (param->QueryBoolAttribute("value", &v_bool) == tinyxml2::XML_SUCCESS) {
                set_param_ok = estimator->setParameter(std::string(param_name), v_bool);
            } else {
                const char* v_str = param->Attribute("value");
                if (v_str) {
                    set_param_ok = estimator->setParameter(std::string(param_name), std::string(v_str));
                } else {
                    error << "State estimator parameters should always have a 'name' and 'value' attribute." << std::endl;
                }
            }

            if (!set_param_ok) {
                error << "Unknown parameter for estimator '" << model_type << "': " << param_name << std::endl;
            }

        } else {
            error << "State estimator parameters should always have a 'name' and 'value' attribute." << std::endl;
        }

        param = param->NextSiblingElement("param");
    }

    const tinyxml2::XMLElement* pnew = elem->FirstChildElement("pnew");
    if (pnew) {
        pbl::PDF* pdf_new = parsePDF(pnew, error);
        if (pdf_new) {
            obj_model->setNewPDF(attribute, *pdf_new);

            estimator->update(*pdf_new, 0);

            delete pdf_new;
        } else {
            return false;
        }
    } else {
        error << "Estimator specification does not contain 'pnew'." << std::endl;
        return false;
    }

    const tinyxml2::XMLElement* pclutter = elem->FirstChildElement("pclutter");
    if (pnew) {
        pbl::PDF* pdf_clutter = parsePDF(pclutter, error);
        if (pdf_clutter) {
            obj_model->setClutterPDF(attribute, *pdf_clutter);

            delete pdf_clutter;
        } else {
            return false;
        }
    } else {
        error << "Estimator specification does not contain 'pclutter'." << std::endl;
        return false;
    }

    obj_model->setEstimator(attribute, *estimator);

    return true;
}

bool ObjectModelParser::getStateEstimatorParameter(const tinyxml2::XMLElement* elem, const std::string& param_name, double& value) {

    const tinyxml2::XMLElement* param = elem->FirstChildElement("param");
    while (param) {
        const char* v = param->Attribute("name");
        if (v && (std::string)v == param_name) {
            param->Attribute("value", std::to_string(value).c_str());
            return true;
        }
        param = param->NextSiblingElement("param");
    }

    return false;
}

bool ObjectModelParser::parse(KnowledgeDatabase& knowledge_db) {

    tinyxml2::XMLDocument doc;
    doc.LoadFile(filename_.c_str());

    if (doc.Error()) {
        ROS_ERROR_STREAM("While parsing '" << filename_ << "': " << std::endl << std::endl << doc.ErrorStr() << " at line " << doc.ErrorLineNum());
        return false;
    }

    const tinyxml2::XMLElement* root = doc.RootElement();

    double prior_new;
    const tinyxml2::XMLElement* prior_new_elem = root->FirstChildElement("prior_new");
    if (prior_new_elem) {
        if (getAttributeValue(prior_new_elem, "value", prior_new, parse_errors_)) {
            knowledge_db.setPriorNew(prior_new);
        }
    } else {
        parse_errors_ << "Knowledge file does not contain 'prior_new'" << std::endl;
    }

    double prior_existing;
    const tinyxml2::XMLElement* prior_existing_elem = root->FirstChildElement("prior_existing");
    if (prior_existing_elem) {
        if (getAttributeValue(prior_existing_elem, "value", prior_existing, parse_errors_)) {
            knowledge_db.setPriorExisting(prior_existing);
        }
    } else {
        parse_errors_ << "Knowledge file does not contain 'prior_existing'" << std::endl;
    }

    double prior_clutter;
    const tinyxml2::XMLElement* prior_clutter_elem = root->FirstChildElement("prior_clutter");
    if (prior_clutter_elem) {
        if (getAttributeValue(prior_clutter_elem, "value", prior_clutter, parse_errors_)) {
            knowledge_db.setPriorClutter(prior_clutter);
        }
    } else {
        parse_errors_ << "Knowledge file does not contain 'prior_clutter'" << std::endl;
    }


    const tinyxml2::XMLElement* class_element = root->FirstChildElement("object_class");

    /* PARSE ALL OBJECT MODELS */

    while(class_element) {
        ClassModel* class_model = 0;

        std::string model_name;
        getAttributeValue(class_element, "name", model_name, parse_errors_);

        std::cout << "Parsing model for class " << model_name << std::endl;

        std::string base_class = "";
        const char* value = class_element->Attribute("base");

        if (value) {
            // class derives from base class
            base_class = value;

            const ClassModel* base_model = knowledge_db.getClassModel(base_class);
            if (base_model) {
                class_model = new ClassModel(*base_model);
                class_model->setModelName(model_name);
            } else {
                parse_errors_ << "Error in class definition of '" << model_name << "': unknown base class '" << base_class << "'." << std::endl;
                class_model = new ClassModel(model_name);
            }
        } else {
            class_model = new ClassModel(model_name);
        }

        // parse properties
        const tinyxml2::XMLElement* prop = class_element->FirstChildElement();
        while(prop) {
            std::string prop_name = prop->Value();
            if (prop_name == "behavior_model") {
                std::stringstream bh_errors;
                parseStateEstimator(class_model, prop, bh_errors);
                if (bh_errors.str() != "") {
                    parse_errors_ << "In class description for '" << class_model->getModelName() << "': " << bh_errors.str() << std::endl;
                }
            } else {
                parse_errors_ << "In class description for '" << class_model->getModelName() << "': Unknown class property: '" << prop_name << "'" << std::endl;
            }
            prop = prop->NextSiblingElement();
        }

        knowledge_db.addClassModel(class_model->getModelName(), class_model);

        class_element = class_element->NextSiblingElement("object_class");
    }

    if (parse_errors_.str() != "") {
        return false;
    }

    return true;
}

}
