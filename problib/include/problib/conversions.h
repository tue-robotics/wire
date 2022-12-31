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

#ifndef PROBLIB_CONVERSIONS_H_
#define PROBLIB_CONVERSIONS_H_

#include <problib_msgs/PDF.h>

#include "problib/pdfs/Gaussian.h"
#include "problib/pdfs/Mixture.h"
#include "problib/pdfs/Hybrid.h"
#include "problib/pdfs/PMF.h"

#include "problib/datatypes.h"

namespace pbl {

/* * * * * * * CONVERSION * * * * * * */

/**
 * @brief Converts a PDF object to a ROS message
 * @param pdf The PDF to be converter
 * @param msg Reference to the message that will be filled with the PDF information
 */
void PDFtoMsg(const PDF& pdf, problib_msgs::PDF& msg);

/**
 * @brief Converts a PDF object to a ROS message
 * @param pdf The PDF to be converter
 * @return The PDF ROS Message
 */
problib_msgs::PDF PDFtoMsg(const PDF& pdf);

/**
 * @brief Converts a PDF ROS message to a PDF object
 * @param msg The PDF ROS message
 * @return Pointer to the PDF object if conversion was successful; 0 otherwise
 */
PDF* msgToPDF(const problib_msgs::PDF& msg);

/**
 * @brief Converts a PDF ROS message to a Gaussian object if the message represents a Gaussian
 * @param msg The PDF ROS message
 * @return Pointer to the PDF object if conversion was successful
 * (only if the message represents a Gaussian); 0 otherwise
 */
Gaussian* msgToGaussian(const problib_msgs::PDF& msg);

/**
 * @brief Converts a PDF ROS message to a Mixture object if the message represents a Mixture
 * @param msg The PDF ROS message
 * @return Pointer to the PDF object if conversion was successful
 * (only if the message represents a Mixture); 0 otherwise
 */
Mixture* msgToMixture(const problib_msgs::PDF& msg);

/**
 * @brief Converts a PDF ROS message to a PMF object if the message represents a PMF
 * @param msg The PDF ROS message
 * @return Pointer to the PDF object if conversion was successful
 * (only if the message represents a PMF); 0 otherwise
 */
PMF* msgToPMF(const problib_msgs::PDF& msg);

/**
 * @brief Casts a PDF object to a Gaussian if the PDF represents a Gaussian
 * @param pdf The PDF object
 * @return Pointer to the PDF object if casting was successful; 0 otherwise
 */
const Gaussian* PDFtoGaussian(const PDF& pdf);

/**
 * @brief Casts a PDF object to a Mixture if the PDF represents a Mixture
 * @param pdf The PDF object
 * @return Pointer to the PDF object if casting was successful; 0 otherwise
 */
const Mixture* PDFtoMixture(const PDF& pdf);

/**
 * @brief Casts a PDF object to a Uniform distribution if the PDF represents a Uniform distribution
 * @param pdf The PDF object
 * @return Pointer to the PDF object if casting was successful; 0 otherwise
 */
const Uniform* PDFtoUniform(const PDF& pdf);


/**
 * @brief Casts a PDF object to a PMF if the PDF represents a PMF
 * @param pdf The PDF object
 * @return Pointer to the PDF object if casting was successful; 0 otherwise
 */
const PMF* PDFtoPMF(const PDF& pdf);

/* * * * * * * OTHER * * * * * * */

/**
 * @brief Get the type of PDF as string
 * @param type PDFType as enum
 * @return String of the PDFType
 */
std::string typeToName(PDF::PDFType type);

/* * * * * * * SERIALIZATION AND DESERIALIZATION * * * * * * */

/**
 * @brief Serialize a PDF object to a PDF message
 * @param[in] pdf The PDF object
 * @param[out] msg The filled PDF message
 */
void serialize(const PDF& pdf, problib_msgs::PDF& msg);

/**
 * @brief Deserialize a PDF ROS message to a PDF object
 * @param msg The PDF ROS message
 * @param type The type of the PDF
 * @param i_data The positon of the data to be read
 * @return Pointer to the PDF object if deserialization was successful; 0 otherwise
 */
PDF* deserialize(const problib_msgs::PDF& msg, int type, int& i_data);

/**
 * @brief Serialize a Gaussian PDF object to a PDF ROS message
 * @param[in] gauss The Gaussian PDF object
 * @param[out] msg The filled PDF message
 */
void serialize_gaussian(const Gaussian& gauss, problib_msgs::PDF& msg);

/**
 * @brief Deserialize a Gaussian PDF ROS message to a Gaussian PDF object
 * @param msg The Gaussian PDF ROS message
 * @param i_data The positon of the data to be read
 * @return Pointer to the Guassian PDF object if deserialization was successful; 0 otherwise
 */
Gaussian* deserialize_gaussian(const problib_msgs::PDF& msg, int& i_data);

/**
 * @brief Serialize a Mixture PDF object to a PDF ROS message
 * @param[in] mix The Mixture PDF object
 * @param[out] msg The filled PDF ROS message
 */
void serialize_mixture(const Mixture& mix, problib_msgs::PDF& msg);

/**
 * @brief Deserialize a Mixture PDF ROS message to a Mixture PDF object
 * @param msg The Mixture PDF ROS message
 * @param i_data The positon of the data to be read
 * @return Pointer to the Mixture PDF object if deserialization was successful; 0 otherwise
 */
Mixture* deserialize_mixture(const problib_msgs::PDF& msg, int& i_data);

/**
 * @brief Serialize an Uniform PDF object to a PDF ROS message
 * @param[in] uniform The Uniform PDF object
 * @param[out] msg The filled PDF ROS message
 */
void serialize_uniform(const Uniform& uniform, problib_msgs::PDF& msg);

/**
 * @brief Deserialize an Uniform PDF ROS message to an Uniform PDF object
 * @param msg The Uniform PDF ROS message
 * @param i_data The positon of the data to be read
 * @return Pointer to the Uniform PDF object if deserialization was successful; 0 otherwise
 */
Uniform* deserialize_uniform(const problib_msgs::PDF& msg, int& i_data);

/**
 * @brief Serialize a Hybrid PDF object to a PDF ROS message
 * @param[in] hybrid The Hybrid PDF object
 * @param[out] msg The filled PDF ROS message
 */
void serialize_hybrid(const Hybrid& hybrid, problib_msgs::PDF& msg);

/**
 * @brief Deserialize a Hybrid PDF ROS message to a Hybrid PDF object
 * @param msg The Hybrid PDF ROS message
 * @param i_data The positon of the data to be read
 * @return Pointer to the Hybrid PDF object if deserialization was successful; 0 otherwise
 */
Hybrid *deserialize_hybrid(const problib_msgs::PDF& msg, int& i_data);

/**
 * @brief Serialize a PMF object to a PDF ROS message
 * @param[in] pmf The PMF object
 * @param[out] msg The filled PDF ROS message
 */
void serialize_discrete(const PMF& pmf, problib_msgs::PDF& msg);

/**
 * @brief Deserialize a Discrete PDF ROS message to a PMF object
 * @param msg The Discrete PDF ROS message
 * @return Pointer to the PMF object if deserialization was successful; 0 otherwise
 */
PMF* deserialize_discrete(const problib_msgs::PDF& msg);

/**
 * @brief Deserialize an Exact PDF ROS message to an Exact PDF object
 * @param msg The Exact PDF ROS message
 * @return Pointer to the Exact PDF object if deserialization was successful; 0 otherwise
 */
PDF* deserialize_exact(const problib_msgs::PDF& msg);

}

#endif /* CONVERSIONS_H_ */
