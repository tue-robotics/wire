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

#ifndef PROBLIB_UNIFORMPDF_H_
#define PROBLIB_UNIFORMPDF_H_

#include "PDF.h"

namespace pbl {

/**
 * @author Sjoerd van den Dries
 * @date December, 2012
 * @version 1.0
 *
 * @brief This class represents a hyper-cube shaped uniform distribution
 */
class Uniform: public PDF {

public:

    /**
     * @brief Constructs a uniform distribution with known dimensionality but unknown
     * size and density.
     * @param dim The dimensionality of the uniform distribution
     */
	Uniform(int dim);


    /**
     * @brief Constructs a uniform distribution with known dimensionality and density,
     * but unknown shape. Will become obsolete in the future
     * @param dim The dimensionality of the uniform distribution
     * @param density The density of the uniform distribution
     */
	Uniform(int dim, double density);

    /**
     * @brief Constructs a uniform distribution with given mean value and size. The density
     * of the distribution equals 1 / Volume, with the volume being determined by the size
     * vector.
     * @param mean The mean of the uniform distribution
     * @param size The size of the uniform distribution
     */
    Uniform(pbl::Vector mean, pbl::Vector size);

    /**
     * @brief Copy constructor
     */
	Uniform(const Uniform& pdf);

    /**
     * @brief Destructor
     */
	virtual ~Uniform();

    /**
     * @brief Assignment operator. The operation is cheap since it only
     * copies a pointer. A deep clone will only be created if the original
     * object is modified.
     */
	Uniform& operator=(const Uniform& other);

    /**
     * @brief Creates a clone of the object. The clone method is cheap since it only
     * copies a pointer. A deep clone will only be created if the original
     * object is modified.
     */
	Uniform* clone() const;

	double getLikelihood(const PDF& pdf) const;

    /**
     * @brief Sets the density of the uniform distribution
     * @note Only makes sense if the size of the volume is not set
     * @param density The uniform density of the distribution
     */
	void setDensity(const double& density);

    /**
     * @brief Calculates the density of the distribution at point v. Will always return
     * the same density if v is inside the volume, and 0 otherwise.
     * @param v The point to calculate the density for
     * @return The density of the distribution at point v
     */
	double getDensity(const arma::vec& vec) const;

    /**
     * @brief Returns the maximum density of this distribution, which always equals 1 / volume
     * @return The maximum density of this distribution
     */
    double getMaxDensity() const;

    /**
     * @brief Set the mean of the volume representing this uniform distribution
     * @param mean The mean of the volumne
     */
    void setMean(const pbl::Vector mean);

    /**
     * @brief Set the size of the volume representing this uniform distribution
     * @param size The size of the volumne
     */
    void setSize(const pbl::Vector size);

    /**
     * @brief Represents the uniform distribution as a string for easier console output
     * @note Should be changed into stream operator <<
     * @return The Gaussian as string
     */
	std::string toString(const std::string& indent = "") const;

protected:

    pbl::Vector mean_;

    pbl::Vector size_;

    double uniform_probability_;

    bool size_is_set_;

    void calculateUniformDensity();

};

}

#endif /* UNIFORMPDF_H_ */
