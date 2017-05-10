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

#ifndef PROPERTY_H_
#define PROPERTY_H_

#include "wire/core/datatypes.h"
#include "problib/pdfs/PDF.h"

#include <string>

namespace mhf {

class IStateEstimator;

class Property {

public:

    Property(const Attribute& attribute, const IStateEstimator& bm, const ObjectID& object_id = -1);

    Property(const Property& orig);

    virtual ~Property();

    Property* clone() const;

    Property& operator=(const Property& other);

    const Attribute& getAttribute() const;

    const IStateEstimator& getEstimator() const;

    const pbl::PDF& getValue() const;

    //void setObjectID(const ObjectID& id);

    const ObjectID& getObjectID() const;

    void update(const pbl::PDF& z, const Time& time);

    void propagate(const Time& time);

    void reset();

    virtual double getLikelihood(const pbl::PDF& pdf) const;

    std::string toString(const std::string& prefix = "") const;

protected:

    Time time_;

    //ObjectID object_id_;

    Attribute attribute_;

    IStateEstimator* estimator_;

};

}

#endif /* PREDICATE_H_ */
