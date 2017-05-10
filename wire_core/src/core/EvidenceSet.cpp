/*
 * EvidenceSet.cpp
 *
 *  Created on: May 24, 2012
 *      Author: sdries
 */

#include "wire/core/EvidenceSet.h"
#include "wire/core/Evidence.h"

namespace mhf {

EvidenceSet::EvidenceSet() : timestamp_(-1) {
}

EvidenceSet::~EvidenceSet() {
}


void EvidenceSet::add(Evidence* ev) {
    evidence_.push_back(ev);

    // all evidence added to the evidence set should have same timestamp
    assert(timestamp_ < 0 || fabs(ev->getTimestamp() - timestamp_) < 1e-10);

    timestamp_ = ev->getTimestamp();
}

unsigned int EvidenceSet::size() const {
    return evidence_.size();
}

const Time& EvidenceSet::getTimestamp() const {
    return timestamp_;
}

std::vector<Evidence*>::const_iterator EvidenceSet::begin() const {
    return evidence_.begin();
}

std::vector<Evidence*>::const_iterator EvidenceSet::end() const {
    return evidence_.end();
}

}
