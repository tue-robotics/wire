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

#ifndef WM_I_STATE_ESTIMATOR_H_
#define WM_I_STATE_ESTIMATOR_H_

#include "wire/core/datatypes.h"
#include "problib/pdfs/PDF.h"

namespace mhf {

/**
 * @author Sjoerd van den Dries
 * @date December, 2012
 *
 * @brief Base class for all state estimators used by the world model.
 *
 * A state estimator estimates the value of one specific attribute
 * of one specific object in the world. The attribute value is represented
 * by a probability density over the domain of the attribute. For example,
 * a position can be represented by a PDF over Cartesian space. A state
 * estimator should implement three methods: propagate(Time t), which
 * changes the internal state of the estimator to the estimated value at
 * Time t; update(PDF z, Time time) which updates the internal state
 * based on measurement z at Time t; and getValue() which returns the
 * current state.
 */
class IStateEstimator {

public:

    virtual ~IStateEstimator() {}

    virtual IStateEstimator* clone() const = 0;

    /**
     * @brief Propagates the internal state to Time time
     * @param time The time to which the internal state is propagated
     */
    virtual void propagate(const Time& time) = 0;

    /**
     * @brief Updates the internal state based on measurement z
     * @param z The measurement with which to update, represented as a probability density function
     * @param time The time to which the internal state is propagated before updating
     */
    virtual void update(const pbl::PDF& z, const Time& time) = 0;

    /**
     * @brief Resets the internal state of the estimator to its initial value
     */
    virtual void reset() = 0;

    /**
     * @brief Returns the current estimated state value
     * @return The current state, i.e., the current attribute value represented as probability density function
     */
    virtual const pbl::PDF& getValue() const = 0;

    /**
     * @brief Resets the internal state of the estimator to the given PDF
     * @param pdf The value to which the internal state is set
     */
    //virtual void setValue(const pbl::PDF& pdf) {
    //}

    /**
     * @brief Set a boolean parameter of this state estimator
     * @param param The parameter name
     * @param b The boolean value
     * @return Returns true if the parameter was known to the estimator; false otherwise
     */
    virtual bool setParameter(const std::string& param, bool b) {
        return false;
    }

    /**
     * @brief Set a real-valued parameter of this state estimator
     * @param param The parameter name
     * @param v The float value
     * @return Returns true if the parameter was known to the estimator; false otherwise
     */
    virtual bool setParameter(const std::string& param, double v) {
        return false;
    }

    /**
     * @brief Set a string parameter of this state estimator
     * @param param The parameter name
     * @param s The string value
     * @return Returns true if the parameter was known to the estimator; false otherwise
     */
    virtual bool setParameter(const std::string& param, const std::string& s) {
        return false;
    }
};

}

#endif /* WM_I_STATE_ESTIMATOR_H_ */
