/*
 * FixedState.cpp
 *
 *  Created on: Jul 3, 2012
 *      Author: sdries
 */

#include "wire/models/FixedState.h"

namespace mhf {

FixedState::FixedState() {

}

FixedState::FixedState(const pbl::PDF& pdf) : pdf_(pdf.clone()) {
}

FixedState::FixedState(const FixedState& orig) : IStateEstimator(orig), pdf_(orig.pdf_->clone()) {
}

FixedState::~FixedState() {
    delete pdf_;
}

FixedState* FixedState::clone() const {
    return new FixedState(*this);
}

void FixedState::update(const pbl::PDF& z, const mhf::Time& time) {
}

void FixedState::propagate(const mhf::Time& time) {
}

void FixedState::reset() {
}

const pbl::PDF& FixedState::getValue() const {
    return *pdf_;
}

}
