/*
 * Measurement.cpp
 *
 *  Created on: March, 2011
 *  Author: Jos Elfring, Sjoerd van den Dries
 *  Affiliation: Eindhoven University of Technology
 */

#include "wire/core/Evidence.h"

using namespace std;

namespace mhf {

int Evidence::N_EVIDENCE = 0;

Evidence::Evidence(Time timestamp) : PropertySet(timestamp) {
    ++N_EVIDENCE;
}

Evidence::~Evidence() {
    --N_EVIDENCE;
}

}
