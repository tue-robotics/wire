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

#ifndef PROBLIB_GAUSSIAN_H_
#define PROBLIB_GAUSSIAN_H_

#include "PDF.h"

namespace pbl {

/**
 * @author Sjoerd van den Dries
 * @date December, 2012
 * @version 1.0
 *
 * @brief This class represents a multi-variate Gaussian (Normal) distribution.
 */
class Gaussian: public PDF {

public:

    /**
     * @brief Constructs a (multi-variate) Gaussian with specific dimensionality
     * but leaves mean and covariance unspecified.
     * @param dim The dimensionality of the Gaussian
     */
	Gaussian(int dim);

    /**
     * @brief Constructs a (multi-variate) Gaussian with specified mean and
     * covariance.
     * @param mean The mean vector of the Gaussian
     * @param cov The covariance matrix of the Gaussian
     */
	Gaussian(const arma::vec& mean, const arma::mat& cov);

    /**
     * @brief Copy constructor
     */
	Gaussian(const Gaussian& orig);

    /**
     * @brief Destructor
     */
	virtual ~Gaussian();

    /**
     * @brief Assignment operator. The operation is cheap since it only
     * copies a pointer. A deep clone will only be created if the original
     * object is modified.
     */
	Gaussian& operator=(const Gaussian& other);

    /**
     * @brief Creates a clone of the object. The clone method is cheap since it only
     * copies a pointer. A deep clone will only be created if the original
     * object is modified.
     */
	Gaussian* clone() const;

	double getLikelihood(const PDF& pdf) const;

    /**
     * @brief Calculates the density of the Gaussian at point v.
     * @param v The point to calculate the density for
     * @param max_mah_dist
     * @return The density of the Gaussian at point v.
     */
	double getDensity(const arma::vec& v, double max_mah_dist = 0) const;

	double getDensity(const Gaussian& npdf, double max_mah_dist = 0) const;

    /**
     * @brief Calculates the maximum density of the Gaussian, i.e., the density at
     * the mean.
     * @return The maximum density of the Gaussian.
     */
	double getMaxDensity() const;

    /**
     * @brief Returns the expected value E[x] of the Gaussian, which corresponds to
     * its mean.
     * @param v The returned expected value
     * @return Always true
     */
	bool getExpectedValue(arma::vec& v) const;

    /**
     * @brief Sets the mean of the Gaussian
     * @param mu The mean of the Gaussian
     */
	void setMean(const arma::vec& mu);

    /**
     * @brief Sets the covariance of the Gaussian
     * @param cov The covariance matrix of the Gaussian
     */
	void setCovariance(const arma::mat& cov);

    /**
     * @brief Returns the mean of the Gaussian
     * @return The mean of the Gaussian
     */
	const arma::vec& getMean() const;

    /**
     * @brief Returns the covariance matrix of the Gaussian
     * @return The covariance matrix of the Gaussian
     */
	const arma::mat& getCovariance() const;

    /**
     * @brief Represents the Gaussian as a string for easier console output
     * @note Should be changed into stream operator <<
     * @return The Gaussian as string
     */
	std::string toString(const std::string& indent = "") const;

protected:

	struct GaussianStruct {

		arma::vec mu_;

		arma::mat cov_;

		int n_ptrs_;

		GaussianStruct(const arma::vec& mu, const arma::mat& cov) : mu_(mu) , cov_(cov), n_ptrs_(1) { }

		GaussianStruct(const GaussianStruct& orig) : mu_(orig.mu_), cov_(orig.cov_), n_ptrs_(1) { }
	};

	GaussianStruct* ptr_;

	void cloneStruct();

	double getDensity(const arma::vec& v1, const arma::vec& v2, const arma::mat& S, double max_mah_dist = 0) const;

#define CHECK_INITIALIZED assert_msg(ptr_, "Gaussian was not yet initialized.")

};

}

#endif /* NORMALPDF_H_ */
