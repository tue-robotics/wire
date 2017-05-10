/************************************************************************
 *  Copyright (C) 2012 Eindhoven University of Technology (TU/e).       *
 *  All rights reserved.                                                *
 ************************************************************************
 *  Redistribution and use in source and binary forms, with or without  *
 *  modification, are permitted provided that the following conditions  *
 *  are met:                                                            *
 *                                                                      *
 *      1.  Redistributions of source code must retain the above        *
 *          copyright notice, this list of conditions and the following *
 *          disclaimer.                                                 *
 *                                                                      *
 *      2.  Redistributions in binary form must reproduce the above     *
 *          copyright notice, this list of conditions and the following *
 *          disclaimer in the documentation and/or other materials      *
 *          provided with the distribution.                             *
 *                                                                      *
 *  THIS SOFTWARE IS PROVIDED BY TU/e "AS IS" AND ANY EXPRESS OR        *
 *  IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED      *
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE  *
 *  ARE DISCLAIMED. IN NO EVENT SHALL TU/e OR CONTRIBUTORS BE LIABLE    *
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR        *
 *  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT   *
 *  OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;     *
 *  OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF       *
 *  LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT           *
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE   *
 *  USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH    *
 *  DAMAGE.                                                             *
 *                                                                      *
 *  The views and conclusions contained in the software and             *
 *  documentation are those of the authors and should not be            *
 *  interpreted as representing official policies, either expressed or  *
 *  implied, of TU/e.                                                   *
 ************************************************************************/

#ifndef WM_DATATYPES_H_
#define WM_DATATYPES_H_

#include <map>
#include <string>

namespace mhf {

/* Type for object identifier */
typedef long ObjectID;

/* Type for object attribute */
typedef int Attribute;

/* Type for time */
typedef double Time;

/* Type for duration */
typedef double Duration;

/**
 * @brief Class for converting between attribute names (strings) and more effecient attribute types
 */
class AttributeConv {

public:

    /// map from attribute types to strings
    static std::map<Attribute, std::string> ATTR_TO_STR;

    /// map from attribute string to types
    static std::map<std::string, Attribute> STR_TO_ATTR;

public:

    /**
     * @brief attribute
     * @param attribute_str The attribute name as string
     * @return The attribute type
     */
    static Attribute attribute(const std::string& attribute_str);

    /**
     * @brief attribute_str
     * @param attribute The attribute type
     * @return The attribute name as string
     */
    static std::string attribute_str(const Attribute& attribute);

};

}

#endif /* WM_DATATYPES_H_ */
