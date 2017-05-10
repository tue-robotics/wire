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

#ifndef PROBLIB_MIXTUREPDF_H_
#define PROBLIB_MIXTUREPDF_H_

#include "PDF.h"
#include "problib/pdfs/Gaussian.h"
#include "problib/pdfs/Uniform.h"

namespace pbl {

/**
 * @author Sjoerd van den Dries
 * @date December, 2012
 * @version 1.0
 *
 * @brief This class represents the weighted sum of a finite set of probability
 * density functions.
 */
class Mixture: public PDF {

public:

    /**
     * @brief Constructs a mixture pdf with no initial components.
     */
	Mixture();

    /*
	Mixture(const PDF& pdf, double w = 1);
    */

    /**
     * @brief Copy constructor
     */
	Mixture(const Mixture& orig);

    /**
     * @brief Destructor
     */
	virtual ~Mixture();

    /**
     * @brief Assignment operator. The operation is cheap since it only
     * copies a pointer. A deep clone will only be created if the original
     * object is modified.
     */
	Mixture& operator=(const Mixture& other);

    /**
     * @brief Creates a clone of the object. The clone method is cheap since it only
     * copies a pointer. A deep clone will only be created if the original
     * object is modified.
     */
	Mixture* clone() const;

	double getLikelihood(const PDF& pdf) const;

    /**
     * @brief Removes all components
     */
    void clear();

    /**
     * @brief Returns the number of components
     * @return The number of components
     */
	int components() const;

    /**
     * @brief NOT IMPLEMENTED FOR MIXTURE
     */
	double getMaxDensity() const;

    /**
     * @brief Adds a component pdf with given weight
     * @param pdf The component pdf
     * @param w Weight of the component
     */
	void addComponent(const PDF& pdf, double w);

    /**
     * @brief Returns a reference to the i'th component
     * @param i Index of the component
     * @return A reference to the i'th component
     */
	const PDF& getComponent(int i) const;

    /**
     * @brief Returns the weight of the i'th component
     * @param i Index of the component
     * @return The weight of the i'th component
     */
	double getWeight(int i) const;

    /**
     * @brief Normalizes the weights of all components
     */
	void normalizeWeights();

    /**
     * @brief Represents the Mixture as a string for easier console output
     * @note Should be changed into stream operator <<
     * @return The Gaussian as string
     */
	std::string toString(const std::string& indent = "") const;

protected:

	struct MixtureStruct {

		int num_components_;

		std::vector<PDF*> components_;

		std::vector<double> weights_;

		double weights_total_;

		int n_ptrs_;

		MixtureStruct() : num_components_(0) , weights_total_(0), n_ptrs_(1) { }

		MixtureStruct(const MixtureStruct& orig) : num_components_(orig.num_components_), weights_(orig.weights_),
				weights_total_(orig.weights_total_), n_ptrs_(1) {

			for (std::vector<PDF*>::const_iterator it_pdf = orig.components_.begin(); it_pdf != orig.components_.end(); ++it_pdf) {
				components_.push_back((*it_pdf)->clone());
			}
		}

		~MixtureStruct() {
			for (std::vector<PDF*>::const_iterator it_pdf = components_.begin(); it_pdf != components_.end(); ++it_pdf) {
				delete *it_pdf;
			}
		}
	};

	MixtureStruct* ptr_;

	void cloneStruct();

};

}

#endif /* MIXTUREPDF_H_ */
