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

#include "problib/pdfs/PMF.h"

using namespace pbl;

PMF::PMF(int domain_size) : PDF(1, PDF::DISCRETE), ptr_(new PMFStruct(domain_size)) {
}

PMF::PMF(const PMF& pmf) : PDF(1, PDF::DISCRETE), ptr_(pmf.ptr_) {
    if (ptr_) {
        ++ptr_->n_ptrs_;
    }
}

PMF::~PMF() {
	--ptr_->n_ptrs_;

	if (ptr_->n_ptrs_ == 0) {
		delete ptr_;
	}
}

PMF& PMF::operator=(const PMF& other)  {
    if (this != &other)  {
    	if (ptr_) {
			--ptr_->n_ptrs_;
			if (ptr_->n_ptrs_ == 0) {
				delete ptr_;
			}
    	}

    	ptr_ = other.ptr_;
    	++ptr_->n_ptrs_;

    	dimensions_ = other.dimensions_;
    }
    return *this;
}

PMF* PMF::clone() const {
	return new PMF(*this);
}

void PMF::cloneStruct() {
	if (ptr_->n_ptrs_ > 1) {
		--ptr_->n_ptrs_;
		ptr_ = new PMFStruct(*ptr_);
	}
}

double PMF::getProbability(const std::string& value) const {
	return getProbability(value, ptr_->domain_size_);
}

double PMF::getProbability(const std::string& value, int domain_size) const {
	std::map<std::string, double>::const_iterator it = ptr_->pmf_.find(value);
	if (it != ptr_->pmf_.end()) {
		return (*it).second;
	}
	// if now probability is known for this value, calculate its probability
	// based on a uniform distribution over all unknown values.
	return getProbabilityUnknown(domain_size);
}

void PMF::setProbability(const std::string& value, double p) {
	cloneStruct();

	std::map<std::string, double>::iterator it = ptr_->pmf_.find(value);
	if (it != ptr_->pmf_.end()) {
		ptr_->total_prob_ -= (*it).second;
		(*it).second = p;
	} else {
		ptr_->pmf_[value] = p;
	}
	ptr_->total_prob_ += p;
}

void PMF::setExact(const std::string& value) {
	if (ptr_->n_ptrs_ > 1) {
		--ptr_->n_ptrs_;
		ptr_ = new PMFStruct(ptr_->domain_size_);
	} else {
		ptr_->pmf_.clear();
	}
	ptr_->pmf_[value] = 1.0;
	ptr_->total_prob_ = 1.0;
}

void PMF::getValues(std::vector<std::string>& values) const {
	for(std::map<std::string, double>::const_iterator it = ptr_->pmf_.begin(); it != ptr_->pmf_.end(); ++it) {
		values.push_back((*it).first);
	}
}

void PMF::getProbabilities(std::vector<double>& probabilities) const {
	for(std::map<std::string, double>::const_iterator it = ptr_->pmf_.begin(); it != ptr_->pmf_.end(); ++it) {
		probabilities.push_back((*it).second);
	}
}

bool PMF::getExpectedValue(std::string& v) const {
	double p_max = 0;
	for(std::map<std::string, double>::const_iterator it = ptr_->pmf_.begin(); it != ptr_->pmf_.end(); ++it) {
		if ((*it).second > p_max) {
			v = (*it).first;
			p_max = (*it).second;
		}
	}

	if (ptr_->domain_size_ > 0 && p_max < getProbabilityUnknown(ptr_->domain_size_)) {
        v = "";
		return false;
	}

	return true;
}

double PMF::getLikelihood(const PDF& pdf) const {
	assert_msg(pdf.type() == PDF::DISCRETE, "PMF: Likelihood can only be calculated with another PMF.");

	const PMF* pmf = static_cast<const pbl::PMF*>(&pdf);
	return getLikelihood(*pmf);
}

double PMF::getLikelihood(const PMF& other) const {
	int my_domain_size = ptr_->domain_size_;
	int other_domain_size = other.ptr_->domain_size_;

	assert(my_domain_size == -1 || other_domain_size == -1 || my_domain_size == other_domain_size);
	int domain_size = std::max(my_domain_size, other_domain_size);

	// determine which pmf has the most determined values, and which one less
	const PMF* small_pmf = this;
	const PMF* big_pmf = &other;
	if (this->ptr_->pmf_.size() > other.ptr_->pmf_.size()) {
		small_pmf = &other;
		big_pmf = this;
	}

	// determine the likelihood based on all determined values in small_pmf
	double likelihood = 0;
	double big_p_total_match = 0;  // keeps track of the total probability of all values in big_pmf
								   // that are matched to small_pmf
	for(std::map<std::string, double>::const_iterator it = small_pmf->ptr_->pmf_.begin(); it != small_pmf->ptr_->pmf_.end(); ++it) {
		double big_p_v = big_pmf->getProbability((*it).first, domain_size);
		likelihood += big_p_v * (*it).second;
		big_p_total_match += big_p_v;
	}

	// if the determined values in small_pdf do NOT add up to probability 1, AND
	// we have not yet matched all values in big_pmf with a total probability of 1, it means
	// we still need to match the undetermined values in small_pmf and big_pmf. ASSUME a
	// UNIFORM DISTRIBUTION over the undetermined values in both small_pmf and big_pmf
	if (small_pmf->ptr_->total_prob_ < 1 && big_p_total_match < 1) {
		// determine the number of unmatched values. Since we used small_pmf as a basis for
		// matching above (we matched all its determined values), the number of unmatched values
		// equals the domain size MINUS the number of determined values
		int num_unmatched_values = (domain_size - small_pmf->ptr_->pmf_.size());
		assert(num_unmatched_values > 0);

		// determine the average probability of an unknown value in big_pmf, assuming
		// a uniform distribution over all unknown values.
		double big_p_unknown   = (1 - big_p_total_match)      / num_unmatched_values;
		// determine the probability of an unknown value in small_pmf, assuming
		// a uniform distribution over all unknown values
		double small_p_unknown = (1 - small_pmf->ptr_->total_prob_) / num_unmatched_values;

		// add to the likelihood the SUM of big_p_unknown * small_p_unknown over all
		// unmatches values, which equals num_unmatched_values * big_p_unknown * small_p_unknown
		likelihood += num_unmatched_values * big_p_unknown * small_p_unknown;
	}

	return likelihood;
}

/**
 * @todo: make this implementation more efficient (no need for O(log n) look-ups)
 */
void PMF::update(const pbl::PMF& other) {
    assert(this->ptr_->domain_size_ == -1 || other.ptr_->domain_size_ == -1
    		|| this->ptr_->domain_size_ == other.ptr_->domain_size_);

    cloneStruct();

    this->ptr_->domain_size_ = std::max(this->ptr_->domain_size_, other.ptr_->domain_size_);

	// calculate likelihood
	double likelihood = getLikelihood(other);

	// calculate the current probability of unknown values
	double p_unknown = getProbabilityUnknown(this->ptr_->domain_size_);

	std::set<std::string> updated_values;

	ptr_->total_prob_ = 0;
	for(std::map<std::string, double>::iterator it = ptr_->pmf_.begin(); it != ptr_->pmf_.end(); ++it) {
		double new_prob = (*it).second * other.getProbability((*it).first, this->ptr_->domain_size_) / likelihood;
		(*it).second = new_prob;
		ptr_->total_prob_ += new_prob;
		updated_values.insert((*it).first);
	}

	for(std::map<std::string, double>::const_iterator it = other.ptr_->pmf_.begin(); it != other.ptr_->pmf_.end(); ++it) {
		if (updated_values.find((*it).first) == updated_values.end()) {
			double new_prob = p_unknown * (*it).second / likelihood;
			ptr_->pmf_[(*it).first] = new_prob;
			ptr_->total_prob_ += new_prob;
		}
	}
}

void PMF::setDomainSize(int domain_size) {
	cloneStruct();
	ptr_->domain_size_ = domain_size;
}

int PMF::getDomainSize() const {
	return ptr_->domain_size_;
}

double PMF::getProbabilityUnknown() const {
	return getProbabilityUnknown(ptr_->domain_size_);
}

double PMF::getProbabilityUnknown(int domain_size) const {
	if (ptr_->total_prob_ == 1) return 0;
	assert(domain_size > 0);
	if (domain_size == (int)ptr_->pmf_.size()) return 0;
	return (1 - ptr_->total_prob_) / (domain_size - ptr_->pmf_.size());
}

void PMF::normalize() {
	if (ptr_->total_prob_ == 1) return;

	assert(ptr_->total_prob_ > 0);

	cloneStruct();

	for(std::map<std::string, double>::iterator it = ptr_->pmf_.begin(); it != ptr_->pmf_.end(); ++it) {
		(*it).second /= ptr_->total_prob_;
	}

	ptr_->domain_size_ = ptr_->pmf_.size();

	ptr_->total_prob_ = 1;
}

double PMF::getDensity(const arma::vec& v) const {
	assert_msg(false, "Cannot get density of a PMF");
	return 0;
}

double PMF::getMaxDensity() const {
	assert_msg(false, "Cannot get max density of a PMF");
	return 0;
}

std::string PMF::toString(const std::string& indent) const {
	std::stringstream ss;
	ss << indent << "PMF(" << ptr_->domain_size_ << ")[";

	std::map<std::string, double>::const_iterator it = ptr_->pmf_.begin();
	if (it != ptr_->pmf_.end()) {
		ss << " " << (*it).first << " : " << (*it).second;
		++it;
		for(; it != ptr_->pmf_.end(); ++it) {
			ss << ", " << (*it).first << " : " << (*it).second;
		}
	}
	ss << " ]";
	return ss.str();
}

/* * * * * * * * OBSOLETE * * * * * * * * */

std::string PMF::getMostProbableValue() const {
	std::string v;
	getExpectedValue(v);
	return v;
}
