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

#ifndef PROBLIB_PMF_H_
#define PROBLIB_PMF_H_

#include "PDF.h"

namespace pbl {

/**
 * @author Sjoerd van den Dries
 * @date December, 2012
 * @version 1.0
 *
 * @brief This class represents a discrete probability distribution (or probability
 * mass function). Currently, this PMF can only take strings as values.
 */
class PMF : public PDF {

public:

    /**
     * @brief Constructs a discrete probability distribution. The optional parameter
     * domain size states the number of possible values of the random variable
     * underlying this distribution, and determines the probability of unknown values
     * if the probabilities of all unknown values do not sum up to one.
     * @param domain_size The number of values the random variable of this pmf can take
     */
	PMF(int domain_size = -1);

    /**
     * @brief Copy constructor
     */
	PMF(const PMF& pmf);

    /**
     * @brief Destructor
     */
	virtual ~PMF();

    /**
     * @brief Assignment operator. The operation is cheap since it only
     * copies a pointer. A deep clone will only be created if the original
     * object is modified.
     */
	PMF& operator=(const PMF& other);

    /**
     * @brief Creates a clone of the object. The clone method is cheap since it only
     * copies a pointer. A deep clone will only be created if the original
     * object is modified.
     */
	PMF* clone() const;

    /**
     * @brief Returns the probability of the given value
     * @return The probability of the given value
     */
	double getProbability(const std::string& value) const;

    /**
     * @brief Returns the probability of the given value, for a given domain_size. The domain
     * size determines the probability if the value is unknown, in which case a uniform
     * distribution over all unknown values is assumed.
     * @return The probability of the given value
     */
	double getProbability(const std::string& value, int domain_size) const;

    /**
     * @brief Set the probability of a given value
     */
	void setProbability(const std::string& value, double p);

    /**
     * @brief Set the probability of the given value to 1. All other values are given a
     * probability of 0
     */
	void setExact(const std::string& value);

    /**
     * @brief Returns a vector of values for which a probability is specified
     */
	void getValues(std::vector<std::string>& values) const;

    /**
     * @brief Returns all probabilities of the known values
     */
	void getProbabilities(std::vector<double>& probabilities) const;

	double getLikelihood(const PDF& pdf) const;

	double getLikelihood(const PMF& pmf) const;

    /**
     * @brief Returns in parameter v the expected value for this distribution, i.e., the
     * value with the highest probability.
     * @return Whether an expected value was found
     */
	bool getExpectedValue(std::string& v) const;

	void update(const pbl::PMF& pmf);

    /**
     * @brief Sets the domain size of this discrete distribution
     */
	void setDomainSize(int domain_size);

    /**
     * @brief Returns the domain size of this distribution
     */
	int getDomainSize() const;

	void normalize();

    /**
     * @brief Represents the PMF as a string for easier console output
     * @note Should be changed into stream operator <<
     * @return The Gaussian as string
     */
	std::string toString(const std::string& indent = "") const;

	double getDensity(const arma::vec& v) const;

	double getMaxDensity() const;

	double getProbabilityUnknown() const;

	double getProbabilityUnknown(int domain_size) const;

	// obsolete

	std::string getMostProbableValue() const;


protected:

	struct PMFStruct {

		int domain_size_;

		double total_prob_;

		std::map<std::string, double> pmf_;

		int n_ptrs_;

		PMFStruct(int domain_size) : domain_size_(domain_size), total_prob_(0), n_ptrs_(1) { }

		PMFStruct(const PMFStruct& orig) : domain_size_(orig.domain_size_), total_prob_(orig.total_prob_), pmf_(orig.pmf_), n_ptrs_(1) { }
	};

	PMFStruct* ptr_;

	void cloneStruct();

};

}

#endif /* PMF_H_ */
