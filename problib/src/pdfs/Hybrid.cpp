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

#include "problib/pdfs/Hybrid.h"

using namespace pbl;

Hybrid::Hybrid() : PDF(-1, PDF::HYBRID), ptr_(0) {
}

Hybrid::Hybrid(const Hybrid& orig) : PDF(orig), ptr_(orig.ptr_) {
    if (ptr_) {
        ++ptr_->n_ptrs_;
    }
}

Hybrid::~Hybrid() {
	if (ptr_) {
		--ptr_->n_ptrs_;

		if (ptr_->n_ptrs_ == 0) {
			delete ptr_;
		}
	}
}

Hybrid& Hybrid::operator=(const Hybrid& other)  {
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

Hybrid* Hybrid::clone() const {
    return new Hybrid(*this);
}

void Hybrid::cloneStruct() {
	if (ptr_->n_ptrs_ > 1) {
		--ptr_->n_ptrs_;
        ptr_ = new HybridStruct(*ptr_);
	}
}

double Hybrid::getLikelihood(const PDF& pdf) const {
    assert_msg(false, "Likelihood method not implemented. Please create a subclass of Hybrid and implement your own method.");
}

void Hybrid::clear() {
	if (ptr_) {
		--ptr_->n_ptrs_;
		if (ptr_->n_ptrs_ == 0) {
			delete ptr_;
		}
		ptr_ = 0;
	}
}

double Hybrid::getMaxDensity() const {
    assert_msg(false, "Cannot calculate MaxDensity of Hybrid.");
	return 0;
}

void Hybrid::addPDF(const PDF& pdf, double priority) {
	if (dimensions_ < 0) {
		dimensions_ = pdf.dimensions();
	} else {
        assert(dimensions_ == pdf.dimensions() || pdf.type() == PDF::DISCRETE);
	}

	if (!ptr_) {
        ptr_ = new HybridStruct();
	} else {
		cloneStruct();
	}

    ptr_->pdfs_.push_back(pdf.clone());
}

const std::vector<PDF*>& Hybrid::getPDFS() const {
    assert_msg(ptr_, "Hybrid does not contain pdfs.");
    return ptr_->pdfs_;
}

std::string Hybrid::toString(const std::string& indent) const {
	if (!ptr_) {
        return "HYBRID(-)";
	}

	std::string new_indent = indent + "  ";

	std::stringstream ss;
    ss << "HYBRID{\n";
    for (std::vector<PDF*>::const_iterator it_pdf = ptr_->pdfs_.begin(); it_pdf != ptr_->pdfs_.end(); ++it_pdf) {
        ss << new_indent << (*it_pdf)->toString(new_indent) << "\n";
	}
	ss << indent << "}";
	return ss.str();
}
