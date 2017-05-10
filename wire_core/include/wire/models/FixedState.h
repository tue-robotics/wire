/*
 * FixedState.h
 *
 *  Created on: Jul 3, 2012
 *      Author: sdries
 */

#ifndef FIXEDSTATE_H_
#define FIXEDSTATE_H_

#include "wire/core/IStateEstimator.h"

namespace mhf {

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

}

#endif /* FIXEDSTATE_H_ */
