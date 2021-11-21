/************************************************************************
 *  Copyright (C) 2021 Eindhoven University of Technology (TU/e).       *
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

#ifndef TEST_TEST_PLUGIN_H_
#define TEST_TEST_PLUGIN_H_

#include "wire/core/IStateEstimator.h"

/**
 * @brief TestPlugin is a simple wrapper such that probability density function can
 * be threaded as state estimator, i.e., TestPlugin reflects a state but update,
 * propagation and reset do not influence the state.
 */
class TestPlugin : public mhf::IStateEstimator {

public:

    TestPlugin();

    TestPlugin(const pbl::PDF& pdf);

    TestPlugin(const TestPlugin& orig);

    virtual ~TestPlugin();

    virtual TestPlugin* clone() const;

    /**
     * @brief Performs an update, but since the state is fixed, update will do nothing.
     */
    virtual void update(const pbl::PDF& z, const mhf::Time& time);

    /**
     * @brief Propagates the state, but since the state is fixed, propagate will do nothing.
     */

    virtual void propagate(const mhf::Time& time);

    /**
     * @brief Resets the state, but since the state is fixed, reset will do nothing.
     */
    virtual void reset();

    const pbl::PDF& getValue() const;

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

    /**
     * @brief Set a string parameter of this state estimator
     * @param param The parameter name
     * @param s The string value
     * @return Returns true if the parameter was known to the estimator; false otherwise
     */
    bool setParameter(const std::string& param, const std::string& s);

protected:

    pbl::PDF* pdf_;

    bool bool_param_;
    double double_param_;
    std::string string_param_;

};

#endif /* TEST_TEST_PLUGIN_H_ */
