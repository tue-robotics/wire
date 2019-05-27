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

#ifndef FIXEDSTATE_H_
#define FIXEDSTATE_H_

#include "wire/core/IStateEstimator.h"

/**
 * @brief FixedState is a simple wrapper such that probability density function can
 * be threaded as state estimator, i.e., FixedState reflects a state but update,
 * propagation and reset do not influence the state.
 */
class FixedState : public mhf::IStateEstimator {

public:

    FixedState();

    FixedState(const pbl::PDF& pdf);

    FixedState(const FixedState& orig);

    virtual ~FixedState();

    FixedState* clone() const;

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

protected:

    pbl::PDF* pdf_;

};

#endif /* FIXEDSTATE_H_ */
