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

#ifndef SE_POSITION_FILTER_H_
#define SE_POSITION_FILTER_H_

#include "wire/core/IStateEstimator.h"
#include "problib/conversions.h"

class KalmanFilter;

/**
 * @brief Estimator specialized in estimating the position of a target. The estimator
 * uses a Kalman filter with constant velocity process model. If no updates are received
 * for a given amount of time, the PositionFilter replaces the Kalman filter output by
 * a predefined Gaussian distribution over the current state of the filter (to avoid
 * unrealistic propagation of the state).
 */
class PositionFilter : public mhf::IStateEstimator {

public:

    PositionFilter();

    PositionFilter(const PositionFilter& orig);

    virtual PositionFilter* clone() const;

    virtual ~PositionFilter();

    /**
     * @brief Propagates the internal state to Time time
     * @param time The time to which the internal state is propagated
     */
    virtual void propagate(const mhf::Time& time);

    /**
     * @brief Updates the internal state based on measurement z
     * @param z The measurement with which to update, represented as a probability density function
     * @param time The time to which the internal state is propagated before updating
     */
    void update(const pbl::PDF& z, const mhf::Time& time);

    /**
     * @brief Resets the internal state of the estimator to its initial value
     */
    virtual void reset();

    /**
     * @brief Returns the current estimated state value
     * @return The current state, i.e., the current attribute value represented as probability density function
     */
    const pbl::PDF& getValue() const;

    void setValue(const pbl::PDF& pdf);

    /**
     * @brief Set a boolean parameter of this state estimator
     * @param param The parameter name
     * @param b The boolean value
     * @return Returns true if the parameter was known to the estimator; false otherwise
     */
    bool setParameter(const std::string& param, bool b);

    /**
     * @brief Set a real-valued parameter of this state estimator
     * @param param The parameter name
     * @param v The float value
     * @return Returns true if the parameter was known to the estimator; false otherwise
     */
    bool setParameter(const std::string& param, double v);

protected:

    mhf::Time t_last_update_;

    mhf::Time t_last_propagation_;

    KalmanFilter* kalman_filter_;

    pbl::Gaussian* fixed_pdf_;

    // ********* filter parameters *********

    double max_acceleration_;

    double fixed_pdf_cov_;

    double kalman_timeout_;

};

#endif
