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

#include "problib/pdfs/Mixture.h"

using namespace pbl;

Mixture::Mixture() : PDF(-1, PDF::MIXTURE), ptr_(0) {
}

Mixture::Mixture(const Mixture& orig) : PDF(orig), ptr_(orig.ptr_){
    if (ptr_) {
        ++ptr_->n_ptrs_;
    }
}

Mixture::~Mixture() {
	if (ptr_) {
		--ptr_->n_ptrs_;

		if (ptr_->n_ptrs_ == 0) {
			delete ptr_;
		}
	}
}

Mixture& Mixture::operator=(const Mixture& other)  {
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

Mixture* Mixture::clone() const {
	return new Mixture(*this);
}

void Mixture::cloneStruct() {
	if (ptr_->n_ptrs_ > 1) {
		--ptr_->n_ptrs_;
		ptr_ = new MixtureStruct(*ptr_);
	}
}

double Mixture::getLikelihood(const PDF& pdf) const {
	assert_msg(ptr_, "Mixture does not contain components.");
	assert(ptr_->num_components_ > 0);
	assert(ptr_->weights_total_ == 1);

	double likelihood = 0;
	std::vector<double>::const_iterator it_w =ptr_-> weights_.begin();
	for (std::vector<PDF*>::const_iterator it_pdf = ptr_->components_.begin(); it_pdf != ptr_->components_.end(); ++it_pdf) {
		likelihood += (*it_w) * (*it_pdf)->getLikelihood(pdf);
		++it_w;
	}
	return likelihood;
}

void Mixture::clear() {
	if (ptr_) {
		--ptr_->n_ptrs_;
		if (ptr_->n_ptrs_ == 0) {
			delete ptr_;
		}
		ptr_ = 0;
	}
}

double Mixture::getMaxDensity() const {
	assert_msg(false, "Mixture does not contain components.");
	return 0;
}

int Mixture::components() const {
	return ptr_->num_components_;
}

void Mixture::addComponent(const PDF& pdf, double w) {
	if (dimensions_ < 0) {
		dimensions_ = pdf.dimensions();
	} else {
		assert(dimensions_ == pdf.dimensions());
	}

	if (!ptr_) {
		ptr_ = new MixtureStruct();
	} else {
		cloneStruct();
	}

	ptr_->components_.push_back(pdf.clone());
	ptr_->weights_.push_back(w);
	ptr_->weights_total_ += w;
	++ptr_->num_components_;
}

const PDF& Mixture::getComponent(int i) const {
	assert_msg(ptr_, "Mixture does not contain components.");
	return *(ptr_->components_[i]);
}

double Mixture::getWeight(int i) const {
	assert_msg(ptr_, "Mixture does not contain components.");
	return ptr_->weights_[i];
}

void Mixture::normalizeWeights() {
	assert_msg(ptr_, "Mixture does not contain components.");

	if (ptr_->weights_total_ == 1) return;

	assert(ptr_->weights_total_ > 0);

	for (std::vector<double>::iterator it_w = ptr_->weights_.begin(); it_w != ptr_->weights_.end(); ++it_w) {
		(*it_w) /= ptr_->weights_total_;
	}
	ptr_->weights_total_ = 1;
}

std::string Mixture::toString(const std::string& indent) const {
	if (!ptr_) {
		return "MIX(-)";
	}

	std::string new_indent = indent + "  ";

	std::stringstream ss;
	ss << "MIX{\n";
	std::vector<double>::const_iterator it_w = ptr_->weights_.begin();
	for (std::vector<PDF*>::const_iterator it_pdf = ptr_->components_.begin(); it_pdf != ptr_->components_.end(); ++it_pdf) {
		ss << new_indent << (*it_w) << " : " << (*it_pdf)->toString(new_indent) << "\n";
		++it_w;
	}
	ss << indent << "}";
	return ss.str();
}
