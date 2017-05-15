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

#include "MultiModelFilter.h"
#include "KalmanFilter.h"

#include "PositionFilter.h"

MultiModelFilter::MultiModelFilter() : initialized_(false) {
    PositionFilter* e1 = new PositionFilter();
    e1->setParameter("max_acceleration", 0.0);

    PositionFilter* e2 = new PositionFilter();
    e2->setParameter("max_acceleration", 8.0);

    addEstimator(e1);
    addEstimator(e2);
}

MultiModelFilter::MultiModelFilter(const MultiModelFilter& orig) : mhf::IStateEstimator(orig), initialized_(orig.initialized_),
        mixture_(orig.mixture_) {
    for(unsigned int i = 0; i < orig.estimators_.size(); ++i) {
        estimators_.push_back(orig.estimators_[i]->clone());
        weights_.push_back(orig.weights_[i]);
    }
}

MultiModelFilter::~MultiModelFilter() {
    for(unsigned int i = 0; i < estimators_.size(); ++i) {
        delete estimators_[i];
    }
}

MultiModelFilter* MultiModelFilter::clone() const {
    return new MultiModelFilter(*this);
}

void MultiModelFilter::addEstimator(mhf::IStateEstimator* estimator) {
    estimators_.push_back(estimator);

    weights_.resize(estimators_.size());
    for(unsigned int i = 0; i < estimators_.size(); ++i) {
        weights_[i] = 1 / (double)estimators_.size();
    }
}

void MultiModelFilter::propagate(const mhf::Time& time) {
    for(unsigned int i = 0; i < estimators_.size(); ++i) {
        estimators_[i]->propagate(time);
    }

    // propagate weights (TODO: use transition matrix here)
    double w0 = 0.5 * weights_[0] + 0.5 * weights_[1];
    double w1 = 0.5 * weights_[0] + 0.5 * weights_[1];

    weights_[0] = w0;
    weights_[1] = w1;
}

void MultiModelFilter::update(const pbl::PDF& z, const mhf::Time& time) {
    if (!initialized_) {
        for(unsigned int i = 0; i < estimators_.size(); ++i) {
            estimators_[i]->update(z, time);
        }
        initialized_ = true;
        return;
    }

    // update weights based on likelihoods
    double total_weight = 0;
    for(unsigned int i = 0; i < estimators_.size(); ++i) {
        weights_[i] = weights_[i] * estimators_[i]->getValue().getLikelihood(z);
        total_weight += weights_[i];
    }

    // normalize weights and find best estimator
    unsigned int i_best = 0;
    for(unsigned int i = 0; i < estimators_.size(); ++i) {
        weights_[i] /= total_weight;
        if (weights_[i] > weights_[i_best]) {
            i_best = i;
        }
    }

    // update most probable estimator with the evidence, set others to 0 weight
    for(unsigned int i = 0; i < estimators_.size(); ++i) {
        if (i == i_best) {
            weights_[i] = 1;            
        } else {
            weights_[i] = 0;
            estimators_[i]->reset();
        }
        estimators_[i]->update(z, time);
    }
}

void MultiModelFilter::reset() {
    for(unsigned int i = 0; i < estimators_.size(); ++i) {
        estimators_[i]->reset();
    }
}

const pbl::PDF& MultiModelFilter::getValue() const {
    mixture_.clear();
    for(unsigned int i = 0; i < estimators_.size(); ++i) {
        mixture_.addComponent(estimators_[i]->getValue(), weights_[i]);
    }
    mixture_.normalizeWeights();
    return mixture_;
}

void MultiModelFilter::setValue(const pbl::PDF& pdf) {

}

bool MultiModelFilter::setParameter(const std::string& param, bool b) {
    return false;
}

bool MultiModelFilter::setParameter(const std::string &param, double v) {
    return true;
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( MultiModelFilter, mhf::IStateEstimator )
