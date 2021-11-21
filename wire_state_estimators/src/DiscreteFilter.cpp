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

#include "DiscreteFilter.h"

int DiscreteFilter::N_DISCRETEKALMAN_FILTER = 0;

DiscreteFilter::DiscreteFilter() {
	++N_DISCRETEKALMAN_FILTER;
}

DiscreteFilter::DiscreteFilter(const DiscreteFilter& orig) : mhf::IStateEstimator(orig), pmf_(orig.pmf_) {
	++N_DISCRETEKALMAN_FILTER;
}

DiscreteFilter::~DiscreteFilter() {
    --N_DISCRETEKALMAN_FILTER;
}

DiscreteFilter* DiscreteFilter::clone() const {
	return new DiscreteFilter(*this);
}

void DiscreteFilter::propagate(const mhf::Time& /*time*/) {
}

void DiscreteFilter::update(const pbl::PDF& z, const mhf::Time& /*time*/) {
	assert(z.type() == pbl::PDF::DISCRETE);
	const pbl::PMF* pmf = pbl::PDFtoPMF(z);
	pmf_.update(*pmf);
}

void DiscreteFilter::reset() {
    pmf_ = pbl::PMF();
}

const pbl::PDF& DiscreteFilter::getValue() const {
	return pmf_;
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( DiscreteFilter, mhf::IStateEstimator )
