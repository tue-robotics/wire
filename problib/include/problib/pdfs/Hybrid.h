/*
 * Hybrid.h
 *
 *  Created on: Sep 20, 2011
 *      Author: sdries
 */

#ifndef PROBLIB_HYBRIDPDF_H_
#define PROBLIB_HYBRIDPDF_H_

#include "PDF.h"

namespace pbl {

class Hybrid: public PDF {

public:

    Hybrid();

    Hybrid(const Hybrid& orig);

    virtual ~Hybrid();

    Hybrid& operator=(const Hybrid& other);

    Hybrid* clone() const;

    virtual double getLikelihood(const PDF& pdf) const;

	void clear();

	double getMaxDensity() const;

    void addPDF(const PDF& pdf, double priority);

    const std::vector<PDF*>& getPDFS() const;

	std::string toString(const std::string& indent = "") const;

protected:

    struct HybridStruct {

        std::vector<PDF*> pdfs_;

		int n_ptrs_;

        HybridStruct() : n_ptrs_(1) { }

        HybridStruct(const HybridStruct& orig) : n_ptrs_(1) {

            for (std::vector<PDF*>::const_iterator it_pdf = orig.pdfs_.begin(); it_pdf != orig.pdfs_.end(); ++it_pdf) {
                pdfs_.push_back((*it_pdf)->clone());
			}
		}

        ~HybridStruct() {
            for (std::vector<PDF*>::const_iterator it_pdf = pdfs_.begin(); it_pdf != pdfs_.end(); ++it_pdf) {
				delete *it_pdf;
			}
		}
	};

    HybridStruct* ptr_;

	void cloneStruct();

};

}

#endif /* HYBRIDPDF_H_ */
