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

#ifndef PROBLIB_DATATYPES_H_
#define PROBLIB_DATATYPES_H_

#include <armadillo>

namespace pbl {

typedef arma::vec Vector;

typedef arma::mat Matrix;

class Scalar : public arma::vec::fixed<1> {

public:
    Scalar(double x) {
        (*this)(0) = x;
    }
};

class Vector3 : public arma::vec3 {

public:
	Vector3(double v0, double v1, double v2) {
		(*this)(0) = v0;
		(*this)(1) = v1;
		(*this)(2) = v2;
	}

};

//class Vector3 : public Eigen::Vector3d {
//
//public:
//	Vector3(double v0, double v1, double v2) {
//		(*this)(0) = v0;
//		(*this)(1) = v1;
//		(*this)(2) = v2;
//	}
//
//};

class Vector4 : public arma::vec4 {

public:
	Vector4(double v0, double v1, double v2, double v3) {
		(*this)(0) = v0;
		(*this)(1) = v1;
		(*this)(2) = v2;
		(*this)(3) = v3;
	}

};

class Matrix3 : public arma::mat33 {

public:
	Matrix3(double m00, double m11, double m22) {
		this->zeros();
		(*this)(0, 0) = m00;
		(*this)(1, 1) = m11;
		(*this)(2, 2) = m22;
	}
};

class Matrix4 : public arma::mat44 {

public:
	Matrix4(double m00, double m11, double m22, double m33) {
		this->zeros();
		(*this)(0, 0) = m00;
		(*this)(1, 1) = m11;
		(*this)(2, 2) = m22;
		(*this)(3, 3) = m33;
	}
};


}

#endif /* DATATYPES_H_ */
