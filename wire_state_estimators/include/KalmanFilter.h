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

#ifndef SE_KALMANFILTER_H_
#define SE_KALMANFILTER_H_

#include "problib/pdfs/Gaussian.h"

/**
 * @brief Kalman filter with constant-velocity system model. The system noise is automatically
 * calculated from the maximum expected acceleration which can be set as a parameter.
 */
class KalmanFilter {
public:

    /**
     * @brief Constructs a Kalman filter with specified state dimensionality
     * @param dim Dimensionality of the state
     */
	KalmanFilter(int dim);

    /**
     * @brief Copy constructor
     */
	KalmanFilter(const KalmanFilter& orig);

    /**
     * @brief Clone operator
     */
	virtual KalmanFilter* clone() const;

    /**
     * @brief Destructor
     */
	virtual ~KalmanFilter();

    /**
     * @brief Initializes the Kalman filter according to the given Gaussian, i.e., the
     * initial state and state covariance is set to the mean and covariance of the
     * given Gaussian.
     * @param G The Gaussian to initialize with
     */
	virtual void init(const pbl::Gaussian& G);

    /**
     * @brief Updates the Kalman filter with the given (Gaussian) measurement
     * @param z The Gaussian measurement
     */
	void update(const pbl::Gaussian& z);

    /**
     * @brief Propagates the state of the Kalman filter according to the (constant-
     * velocity) system model.
     * @param dt The time increment with which to propagate
     */
    virtual void propagate(const double& dt);

    /**
     * @brief Calculates the likelihood that the given measurement originates from the
     * estimated Kalman state.
     * @param z The Gaussian measurement
     * @return
     */
	double getLikelihood(const pbl::Gaussian& z) const;

    /**
     * @brief Returns the Kalman state and covariance as Gaussian
     * @return The Kalman state and covariance as Gaussian
     */
	const pbl::Gaussian& getGaussian() const;

    /**
     * @brief Returns the Kalman state (mean of the estimated Gaussian)
     * @return The Kalman state
     */
	const pbl::Vector& getState() const;

    /**
     * @brief Returns the covariance of the Kalman state
     * @return The covariance of the Kalman state
     */
	const pbl::Matrix& getStateCovariance() const;

    /**
     * @brief Set the maximum expected acceleration. This parameter is used to determine
     * the system noise in the propagate phase: a constant velocity is assumed, but the
     * system noise takes into account that the the target may accelerate or deccelerate
     * with the specified maximum acceleration.
     * @param a_max The maximum expected acceleration
     */
	void setMaxAcceleration(double a_max);

protected:

    /** Dimensionality of the measurements **/
	int meas_dim_;

    /** Dimensionality of the full state **/
	int state_dim_;

    /** The estimated Kalman state as Gaussian (state mean and covariance) **/
	pbl::Gaussian G_;

    /** Portion of the estimated Kalman state that contains only the measurement dimensions **/
	pbl::Gaussian G_small_;

    /** Observation model **/
    pbl::Matrix H_;

    /** Maximum expected acceleration **/
	double a_max_;
};

#endif /* KALMANTRACKER_H_ */
