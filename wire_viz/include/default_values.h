/*
 * default_values.h
 *
 *  Created on: Oct 17, 2012
 *      Author: jelfring
 */

#ifndef DEFAULT_VALUES_H_
#define DEFAULT_VALUES_H_


// Default values used in case of incomplete default specification in YAML file
namespace visualize {

bool DEFAULT_SHOW_TEXT = true;
bool DEFAULT_SHOW_COV = false;
bool DEFAULT_SHOW_ID = true;
double DEFAULT_COLOR_R = 1.0;
double DEFAULT_COLOR_G = 1.0;
double DEFAULT_COLOR_B = 1.0;
double DEFAULT_COLOR_ALPHA = 1.0;
double DEFAULT_SCALE_X = 0.1;
double DEFAULT_SCALE_Y = 0.1;
double DEFAULT_SCALE_Z = 0.1;
double DEFAULT_OFFSET_X_POS = 0;
double DEFAULT_OFFSET_Y_POS = 0;
double DEFAULT_OFFSET_Z_POS = 0;
double DEFAULT_OFFSET_X_ROT = 0;
double DEFAULT_OFFSET_Y_ROT = 0;
double DEFAULT_OFFSET_Z_ROT = 0;
double DEFAULT_OFFSET_W_ROT = 1.0;
std::string DEFAULT_TYPE = "sphere";
double DEFAULT_MARKER_LIFETIME = 1.0;

}

#endif /* DEFAULT_VALUES_H_ */
