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

#ifndef WM_CLASS_MODEL_H_
#define WM_CLASS_MODEL_H_

#include "wire/core/PropertySet.h"
#include "wire/core/datatypes.h"
#include <string>

namespace mhf {

class IStateEstimator;

/**
 * @author Sjoerd van den Dries
 * @date December, 2012
 *
 * @brief Contains knowledge about a specific object class on where to expect new objects
 * of that class (new) and where not to expect them (clutter). Furthermore, the ClassModel
 * constains prototype state estimators for all possible attributes of the class, which
 * are used for the initialization of object instances of the class.
 */
class ClassModel {

public:

    ClassModel(const std::string& model_name);

    ClassModel(const ClassModel& orig);

    virtual ~ClassModel();

    void setNewPDF(const Attribute& attribute, const pbl::PDF& pdf);

    void setClutterPDF(const Attribute& attribute, const pbl::PDF& pdf);

    void setEstimator(const Attribute& attribute, const IStateEstimator& estimator);

    void setModelName(const std::string& name);

    const std::string& getModelName() const;

    const PropertySet& getNewPDFs() const;

    const PropertySet& getClutterPDFs() const;

    const IStateEstimator* getEstimator(const Attribute& attribute) const;

    //void setProbDetectedGivenVisible(double prob);

    //double getProbDetectedGivenVisible() const;

protected:

    PropertySet new_pdfs_;

    PropertySet clutter_pdfs_;

    PropertySet estimators_;

    std::string model_name_;

    //double p_detected_given_visible_;

};

}

#endif /* WM_CLASS_MODEL_H_ */
