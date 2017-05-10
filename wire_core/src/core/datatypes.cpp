/*
 * datatypes.cpp
 *
 *  Created on: Jul 3, 2012
 *      Author: sdries
 */

#include "wire/core/datatypes.h"

namespace mhf {

std::map<Attribute, std::string> AttributeConv::ATTR_TO_STR;

std::map<std::string, Attribute> AttributeConv::STR_TO_ATTR;

Attribute AttributeConv::attribute(const std::string& attribute_str) {
    std::map<std::string, Attribute>::iterator it = STR_TO_ATTR.find(attribute_str);
    if (it == STR_TO_ATTR.end()) {
        Attribute attribute_id = STR_TO_ATTR.size();
        STR_TO_ATTR[attribute_str] = attribute_id;
        ATTR_TO_STR[attribute_id] = attribute_str;
        return attribute_id;
    }
    return it->second;
}

std::string AttributeConv::attribute_str(const Attribute& attribute) {
    std::map<Attribute, std::string>::iterator it = ATTR_TO_STR.find(attribute);
    if (it == ATTR_TO_STR.end()) {
        return "";
    }
    return it->second;
}

}



