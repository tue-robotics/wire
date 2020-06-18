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

#include "PositionFilterTW_Unif.h"
#include "KalmanFilter.h"

PositionFilterTW_Unif::PositionFilterTW_Unif() : t_last_update_(0), t_last_propagation_(0), kalman_filter_(0),
    fixed_pdf_gaus_(0), fixed_pdf_unif_(0), max_acceleration_(0), fixed_pdf_cov_(0), kalman_timeout_(0), OOS_min_x_(0),
    OOS_max_x_(0), OOS_min_y_(0), OOS_max_y_(0), OOS_mean_x_(0), OOS_mean_y_(0) {
}

PositionFilterTW_Unif::PositionFilterTW_Unif(const PositionFilterTW_Unif& orig) : mhf::IStateEstimator(orig), t_last_update_(orig.t_last_update_),
    t_last_propagation_(orig.t_last_propagation_), kalman_filter_(0), fixed_pdf_gaus_(0), fixed_pdf_unif_(0), max_acceleration_(orig.max_acceleration_),
    fixed_pdf_cov_(orig.fixed_pdf_cov_ ), kalman_timeout_(orig.kalman_timeout_), OOS_min_x_(orig.OOS_min_x_), OOS_max_x_(orig.OOS_max_x_),
    OOS_min_y_(orig.OOS_min_y_), OOS_max_y_(orig.OOS_max_y_), OOS_mean_x_(0), OOS_mean_y_(0) {

    if (orig.fixed_pdf_gaus_) {
        fixed_pdf_gaus_ = orig.fixed_pdf_gaus_->clone();
    }

    if (orig.fixed_pdf_unif_) {
        fixed_pdf_unif_ = orig.fixed_pdf_unif_->clone();
    }

    if (orig.kalman_filter_) {
        kalman_filter_ = new KalmanFilter(*orig.kalman_filter_);
    }
}

PositionFilterTW_Unif::~PositionFilterTW_Unif() {
    delete kalman_filter_;
    delete fixed_pdf_gaus_;
    delete fixed_pdf_unif_;
}

PositionFilterTW_Unif* PositionFilterTW_Unif::clone() const {
    return new PositionFilterTW_Unif(*this);
}

void PositionFilterTW_Unif::propagate(const mhf::Time& time) {
    if (t_last_propagation_ == 0) {
        t_last_propagation_ = time;
        return;
    }

    mhf::Duration dt = time - t_last_propagation_;
    t_last_propagation_ = time;

    assert(dt >= 0);

    if (!kalman_filter_) {
        if (fixed_pdf_unif_) {
            // Compute mean position
            OOS_mean_x_ = (OOS_max_x_ + OOS_min_x_) / 2;
            OOS_mean_y_ = (OOS_max_y_ + OOS_min_y_) / 2;

            fixed_pdf_unif_->setMean({OOS_mean_x_, OOS_mean_y_, 1.0});
        }
        return;
    }

    // Get the current propagated position
    double x_prop_ = kalman_filter_->getGaussian().getMean()[0];
    double y_prop_ = kalman_filter_->getGaussian().getMean()[1];

    // If no new updates for kalman_timeout_, set position to fixed state
    if ((time - t_last_update_) > kalman_timeout_) {

        // If no updates due to known occlusion, fix position in middle of occlusion
        if ((x_prop_ > OOS_min_x_) && (x_prop_ < OOS_max_x_) && (y_prop_ > OOS_min_y_) && (y_prop_ < OOS_max_y_)) {

            // Compute mean position
            OOS_mean_x_ = (OOS_max_x_ + OOS_min_x_) / 2;
            OOS_mean_y_ = (OOS_max_y_ + OOS_min_y_) / 2;

            if (!fixed_pdf_unif_) {
                // Create uniform distribution centered at (x,y,z) OOS_mean_x_, OOS_mean_y_, 1.0 and width
                fixed_pdf_unif_ = new pbl::Uniform({OOS_mean_x_, OOS_mean_y_, 1.0}, {OOS_max_x_ - OOS_min_x_, OOS_max_y_ - OOS_min_y_ + 0.2, 2.0});
            } else {
                fixed_pdf_unif_->setMean({OOS_mean_x_, OOS_mean_y_, 1.0});
            }
            delete kalman_filter_, fixed_pdf_gaus_;
            kalman_filter_ = 0;
            fixed_pdf_gaus_ = 0;
            return;
        }

        // If no updates for kalman_timeout_ > 0, set to last known location
        else if (kalman_timeout_ > 0) {
            if (!fixed_pdf_gaus_) {
                int dimensions = kalman_filter_->getGaussian().getMean().n_rows;
                pbl::Matrix cov = arma::eye(dimensions, dimensions) * fixed_pdf_cov_;
                fixed_pdf_gaus_ = new pbl::Gaussian(kalman_filter_->getGaussian().getMean(), cov);
            } else {
                fixed_pdf_gaus_->setMean(kalman_filter_->getGaussian().getMean());
            }

            delete kalman_filter_, fixed_pdf_unif_;
            kalman_filter_ = 0;
            fixed_pdf_unif_ = 0;
            return;
        }
    }

    // propogate with Kalman filter
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

void PositionFilterTW_Unif::update(const pbl::PDF& z, const mhf::Time& time) {
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
        printf("PositionFilter can only be updated with Gaussians.\n");
    }
}

void PositionFilterTW_Unif::reset() {
    delete kalman_filter_;
    kalman_filter_ = 0;

    delete fixed_pdf_gaus_;
    fixed_pdf_gaus_ = 0;

    delete fixed_pdf_unif_;
    fixed_pdf_unif_ = 0;
}

const pbl::PDF& PositionFilterTW_Unif::getValue() const {
    if (kalman_filter_) {
        return kalman_filter_->getGaussian();
    } else if (fixed_pdf_unif_) {
        return *fixed_pdf_unif_;
    } else if (fixed_pdf_gaus_) {
        return *fixed_pdf_gaus_;
    }

    std::cout << "SOMETHINGS WRONG" << std::endl;
}

bool PositionFilterTW_Unif::setParameter(const std::string& param, bool b) {
    return false;
}

bool PositionFilterTW_Unif::setParameter(const std::string &param, double v) {
    if (param == "max_acceleration") {
        max_acceleration_ = v;
        if (kalman_filter_) {
            kalman_filter_->setMaxAcceleration(max_acceleration_);
        }
    } else if (param == "fixed_pdf_cov") {
        fixed_pdf_cov_ = v;
    } else if (param == "kalman_timeout") {
        kalman_timeout_ = v;
    } else if (param == "OOS_min_x") {
        OOS_min_x_ = v;
    } else if (param == "OOS_max_x") {
        OOS_max_x_ = v;
    } else if (param == "OOS_min_y") {
        OOS_min_y_ = v;
    } else if (param == "OOS_max_y") {
        OOS_max_y_ = v;
    } else {
        return false;
    }
    return true;
}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( PositionFilterTW_Unif, mhf::IStateEstimator )
