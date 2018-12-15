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

#include "ros/ros.h"
#include "problib/conversions.h"
#include "problib/datatypes.h"

#include <iostream>
#include <armadillo>

#include <time.h>

using namespace std;
using namespace arma;

list<timespec> TIMERS;

stringstream TIMER_LOG;
stringstream OUTPUT_LOG;

inline void startTimer(int ID = 0) {
	timespec t_start;
	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &t_start);
	TIMERS.push_back(t_start);
}

inline void stopTimer(string msg, int ID = 0, double factor = 1) {
	timespec t_end;
	clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &t_end);

	timespec& t_start = TIMERS.back();
	TIMER_LOG << msg << ": " << factor * ((t_end.tv_sec - t_start.tv_sec) + double(t_end.tv_nsec - t_start.tv_nsec) / 1e9) << " secs." << endl;
	TIMERS.pop_back();
}

void showTimerLog() {
	cout << endl;
	cout << "------------- TIME -----------" << endl;
	cout << TIMER_LOG.str();
	cout << "------------------------------" << endl;
}

void testOutput(string msg, double out, double actual, double epsilon = 1e-9) {
	double diff = fabs(out - actual);
	if (diff < epsilon) {
		OUTPUT_LOG << "OK   - " << msg << ": " << out << endl;
	} else {
		OUTPUT_LOG << "DIFF - " << msg << ": " << out << " - " << actual << " = " << diff << endl;
	}
}

void showOutputLog() {
	cout << endl;
	cout << "----------- OUTPUT -----------" << endl;
	cout << OUTPUT_LOG.str();
	cout << "------------------------------" << endl;
}

void test() {

	using namespace pbl;

    Uniform U1(pbl::Vector3(1, 2, 0.8), pbl::Vector3(2, 1, 0.1));
    Uniform U2(pbl::Vector3(1.2, 2.3, 0.85), pbl::Vector3(2, 1, 0.1));


    cout << U1.toString() << endl;
    cout << U2.toString() << endl;
    cout << U1.getLikelihood(U2) << endl;
    cout << U2.getLikelihood(U1) << endl;

    Hybrid H;

    PMF pmf1;
    pmf1.setProbability("test", 0.8);
    H.addPDF(pmf1, 1);

    problib::PDF H_msg;
    pbl::PDFtoMsg(H, H_msg);

    cout << H_msg << endl;

    PDF* H_received = pbl::msgToPDF(H_msg);

    cout << H_received->toString() << endl;

	Vector3 mean1(0, 0, 0);
	Matrix3 var1(1, 1, 1);
	Gaussian pdf1(mean1, var1);

	Vector3 mean2(1, 1, 1);
	Matrix3 var2(0.1, 0.1, 0.1);
	Gaussian pdf2(mean2, var2);

	Vector3 mean3(0.3, 2, -0.7);
	Matrix3 var3(0.2, 0.1, 0.4);
	Gaussian pdf3(mean3, var3);

	Mixture mix;
	mix.addComponent(pdf1, 0.7);
	mix.addComponent(pdf2, 0.3);

	Mixture mix2;
	mix2.addComponent(mix, 0.1);
	mix2.addComponent(pdf3, 0.9);

	Gaussian exact2(3);
	cout << "Before initialization: " << exact2.toString() << endl;
	exact2.setMean(mean2);

	cout << "Mixture: " << endl << mix2.toString() << endl;

	double d;
	for (int i = 0; i < 1000; ++i) {
		d = mix2.getLikelihood(exact2);
	}
	cout << "Density of mixture at " << mean2 << " = " << d << endl << endl;

	cout << "Converting to msg ..." << endl << endl;
	problib::PDF pdf_msg;
	pbl::PDFtoMsg(mix2, pdf_msg);
	cout << "Result:" << endl << pdf_msg << endl;

	cout << "Converting back to pdf ..." << endl << endl;
	PDF* received_pdf = pbl::msgToPDF(pdf_msg);
	cout << "Result:" << endl << received_pdf->toString() << endl << endl;

	cout << "Density of mixture at " << mean2 << " = " << received_pdf->getLikelihood(exact2) << endl << endl;

	delete received_pdf;

	cout << "Creating pmf ..." << endl;
	PMF pmf;
	pmf.setDomainSize(3);

	PMF pmf2;
	pmf2.setDomainSize(3);
	pmf2.setProbability("A", 0.25);
	pmf2.setProbability("B", 0.5);
	//pmf2.setProbability("C", 0.25);

	cout << pmf.toString() << endl << endl;

	cout << "Updating pmf ..." << endl;
	pmf.update(pmf2);
	cout << pmf.toString() << endl << endl;

	PMF pmf3;
	pmf3.setDomainSize(3);
	pmf3.setProbability("C", 0.999);

	cout << "Updating pmf ..." << endl;
	pmf.update(pmf3);
	cout << pmf.toString() << endl;


	PMF pmf_copy;
	pmf_copy = pmf;

	for(int i = 0; i < 1000; ++i) {
		pmf_copy = pmf;
	}

	pmf_copy.update(pmf3);
	cout << pmf_copy.toString() << endl;
	cout << pmf.toString() << endl;

	cout << "Converting to msg ..." << endl << endl;
	problib::PDF pmf_msg;
	pbl::PDFtoMsg(pmf, pmf_msg);
	cout << "Result:" << endl << pmf_msg << endl;

	cout << "Converting back to pdf ..." << endl << endl;
	PDF* received_pdf2 = pbl::msgToPDF(pmf_msg);
	cout << "Result:" << endl << received_pdf2->toString() << endl << endl;

	delete received_pdf2;

	cout << "Testing simple population of msg for exact (string) value ..." << endl;
	problib::PDF exact_str;
	exact_str.exact_value_str = "test";
	PDF* pdf_exact_str = pbl::msgToPDF(exact_str);
	cout << "exact_str:" << endl << pdf_exact_str->toString("    ") << endl << endl;
	delete pdf_exact_str;

	cout << "Testing simple population of msg for exact (real) value ..." << endl;
	problib::PDF exact_real;
	exact_real.exact_value_vec.push_back(1);
	exact_real.exact_value_vec.push_back(1);
	exact_real.exact_value_vec.push_back(1);
	PDF* pdf_exact_real = pbl::msgToPDF(exact_real);
	cout << "exact_real:" << endl << pdf_exact_real->toString("    ") << endl << endl;

	startTimer();

	double d2;
	for(int i = 0; i < 1000000; ++i) {
		d2 = mix2.getLikelihood(*pdf_exact_real);
	}

	stopTimer("Likelihood on mixture", (double)1 / 1000000);

	cout << "Likelihood with mixture = " << mix2.getLikelihood(*pdf_exact_real) << endl << endl;

	testOutput("Likelihood with mixture", d2, 0.0612611897752479);

	printf("%.16f\n", d2);

	//cout << "Likelihood with itself = " << pdf_exact_real->getLikelihood(*pdf_exact_real) << endl << endl;

	Mixture mix_copy = mix2;
	mix2.clear();

	cout << mix_copy.toString() << endl;

	double d3 = mix_copy.getLikelihood(*pdf_exact_real);
	testOutput("Likelihood with mixture (copy)", d3, d2);

	delete pdf_exact_real;
}

int main(int argc, char **argv) {

	startTimer();
	test();
	stopTimer("TOTAL");

	showTimerLog();
	showOutputLog();
}


