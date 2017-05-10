/*
 * Assignment.h
 *
 *  Created on: Jul 28, 2011
 *      Author: sdries
 */

#include "wire/logic/AssignmentMatrix.h"
#include "wire/logic/Assignment.h"

#include <algorithm>
#include <stdio.h>

using namespace std;

namespace mhf {

bool compareAssignments(const Assignment* ass1, const Assignment* ass2) {
    return ass1->getProbability() > ass2->getProbability();
}

AssignmentMatrix::AssignmentMatrix() {

}

AssignmentMatrix::~AssignmentMatrix() {

}

void AssignmentMatrix::addPotentialAssignment(const Assignment& assignment) {
    map<const Evidence*, unsigned int>::iterator it_ev = evidence_to_index_.find(assignment.getEvidence());

    int ev_index;
    if (it_ev == evidence_to_index_.end()) {
        ev_index = assignments_.size();
        evidence_to_index_[assignment.getEvidence()] = ev_index;
        assignments_.resize(ev_index + 1);
    } else {
        ev_index = it_ev->second;
    }

    assignments_[ev_index].push_back(&assignment);
}


void AssignmentMatrix::sortAssignments() {
    for(vector<vector<const Assignment*> >::iterator it_ev = assignments_.begin(); it_ev != assignments_.end(); ++it_ev) {
        sort(it_ev->begin(), it_ev->end(), compareAssignments);
    }
}

const Assignment& AssignmentMatrix::getAssignment(unsigned int i_ev, int i_assignment) {
    return *assignments_[i_ev][i_assignment];
}

unsigned int AssignmentMatrix::getNumAssignments(unsigned int i_ev) {
    return assignments_[i_ev].size();
}

unsigned int AssignmentMatrix::getNumMeasurements() const {
    return assignments_.size();
}

}
