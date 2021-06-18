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

#include "PositionFilter.h"
#include "KalmanFilter.h"

PositionFilter::PositionFilter() : t_last_update_(0), t_last_propagation_(0), kalman_filter_(nullptr),
    fixed_pdf_(nullptr), max_acceleration_(0), fixed_pdf_cov_(0), kalman_timeout_(0) {
}

PositionFilter::PositionFilter(const PositionFilter& orig) : mhf::IStateEstimator(orig), t_last_update_(orig.t_last_update_),
    t_last_propagation_(orig.t_last_propagation_), kalman_filter_(nullptr), fixed_pdf_(nullptr), max_acceleration_(orig.max_acceleration_),
    fixed_pdf_cov_(orig.fixed_pdf_cov_ ), kalman_timeout_(orig.kalman_timeout_) {

    if (orig.fixed_pdf_) {
        fixed_pdf_ = orig.fixed_pdf_->clone();
    }

    if (orig.kalman_filter_) {
        kalman_filter_ = new KalmanFilter(*orig.kalman_filter_);
    }
}

PositionFilter::~PositionFilter() {
    if (kalman_filter_)
        delete kalman_filter_;
    if (fixed_pdf_)
        delete fixed_pdf_;
}

PositionFilter* PositionFilter::clone() const {
    return new PositionFilter(*this);
}

void PositionFilter::propagate(const mhf::Time& time) {
    if (t_last_propagation_ == 0) {
        t_last_propagation_ = time;
        return;
    }

    mhf::Duration dt = time - t_last_propagation_;
    t_last_propagation_ = time;

    assert(dt >= 0);

    if (!kalman_filter_) {
        return;
    }

    if ((time - t_last_update_) > kalman_timeout_ && kalman_timeout_ > 0) {
    //if ((time - t_last_update_) > 10.0) {
        if (!fixed_pdf_) {
            int dimensions = kalman_filter_->getGaussian().getMean().n_rows;
            pbl::Matrix cov = arma::eye(dimensions, dimensions) * fixed_pdf_cov_;
            fixed_pdf_ = new pbl::Gaussian(kalman_filter_->getGaussian().getMean(), cov);
        } else {
            fixed_pdf_->setMean(kalman_filter_->getGaussian().getMean());
        }

        // kalman_filter_ has been checked before, so it isn't a nullptr
        delete kalman_filter_;
        kalman_filter_ = 0;
        return;
    }

    // TODO: fix the kalman filter update (we shouldn't need a loop here...)
    mhf::Duration small_dt = 0.05;
    if (dt < small_dt) {
        kalman_filter_->propagate(dt);
    } else {
        double total_dt = 0;
        for(; total_dt < dt; total_dt += small_dt) {
            kalman_filter_->propagate(small_dt);
        }
        if (total_dt < dt) {
            kalman_filter_->propagate(dt - total_dt);
        }
    }
}

void PositionFilter::update(const pbl::PDF& z, const mhf::Time& time) {
    t_last_update_ = time;

    if (z.type() == pbl::PDF::GAUSSIAN) {
        const pbl::Gaussian* G = pbl::PDFtoGaussian(z);

        if (!kalman_filter_) {
            kalman_filter_ = new KalmanFilter(z.dimensions());
            kalman_filter_->setMaxAcceleration(max_acceleration_);
            kalman_filter_->init(*G);
        } else {
            kalman_filter_->update(*G);
        }
    } else {
        std::cout << "PositionFilter can only be updated with Gaussians, not with " << z.type() << std::endl;
    }
}

void PositionFilter::reset() {
    if (kalman_filter_) {
        delete kalman_filter_;
        kalman_filter_ = nullptr;
    }

    if (fixed_pdf_) {
        delete fixed_pdf_;
        fixed_pdf_ = nullptr;
    }
}

const pbl::PDF& PositionFilter::getValue() const {
    if (kalman_filter_) {
        return kalman_filter_->getGaussian();
    } else if (fixed_pdf_) {
        return *fixed_pdf_;
    }

    throw std::runtime_error("Couldn't get a value");
}

bool PositionFilter::setParameter(const std::string& /*param*/, bool /*b*/) {
    return false;
}

bool PositionFilter::setParameter(const std::string &param, double v) {
    if (param == "max_acceleration") {
        max_acceleration_ = v;
        if (kalman_filter_) {
            kalman_filter_->setMaxAcceleration(max_acceleration_);
        }
    } else if (param == "fixed_pdf_cov") {
        fixed_pdf_cov_ = v;
    } else if (param == "kalman_timeout") {
        kalman_timeout_ = v;
    } else {
        return false;
    }
    return true;
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( PositionFilter, mhf::IStateEstimator )
