/*
 * AssignmentSet.h
 *
 *  Created on: Jul 29, 2011
 *      Author: sdries
 */

#include "wire/logic/AssignmentSet.h"
#include "wire/logic/Hypothesis.h"
#include "wire/logic/Assignment.h"
#include "wire/logic/AssignmentMatrix.h"

#include <iostream>
#include <cassert>
#include <stdio.h>
#include <set>

using namespace std;

namespace mhf {

AssignmentSet::AssignmentSet(Hypothesis* hyp, AssignmentMatrix* assignment_matrix) :
    hyp_(hyp), assignment_matrix_(assignment_matrix), probability_(hyp_->getProbability()),
    evidence_assignments_(assignment_matrix_->getNumMeasurements()), n_blocked_(0) {

    for(unsigned int i = 0; i < evidence_assignments_.size(); ++i) {
        evidence_assignments_[i] = 0;
        probability_ *= assignment_matrix_->getAssignment(i, 0).getProbability();
    }

    assert(probability_ > 0);
}

AssignmentSet::AssignmentSet(const AssignmentSet& orig) :
    hyp_(orig.hyp_), assignment_matrix_(orig.assignment_matrix_), probability_(orig.probability_),
    evidence_assignments_(orig.evidence_assignments_), n_blocked_(orig.n_blocked_) {
}

AssignmentSet::~AssignmentSet() {
}

void AssignmentSet::expand(list<AssignmentSet*>& children) const {
    for(unsigned int i = n_blocked_; i < evidence_assignments_.size(); ++i) {
        if (evidence_assignments_[i] + 1 < assignment_matrix_->getNumAssignments(i)) {
            AssignmentSet* child = new AssignmentSet(*this);
            child->evidence_assignments_[i]++;
            child->n_blocked_ = i;
            child->probability_ *= assignment_matrix_->getAssignment(i, child->evidence_assignments_[i]).getProbability()
                    / assignment_matrix_->getAssignment(i, this->evidence_assignments_[i]).getProbability();

            children.push_back(child);
        }
    }
}

const Assignment& AssignmentSet::getMeasurementAssignment(unsigned int i_ev) const {
    return assignment_matrix_->getAssignment(i_ev, evidence_assignments_[i_ev]);
}

void AssignmentSet::getAllAssignments(list<const Assignment*>& assignments) const {
    for(unsigned int i = 0; i < evidence_assignments_.size(); ++i) {
        assignments.push_back(&getMeasurementAssignment(i));
    }
}

double AssignmentSet::getProbability() const {
    return probability_;
}

Hypothesis* AssignmentSet::getHypothesis() const {
    return hyp_;
}

int AssignmentSet::getNumMeasurements() const {
    return evidence_assignments_.size();
}

bool AssignmentSet::isValid() const {
    return true;

    /*
    set<const SemanticObject*> assigned_objects;

    for(unsigned int i = 0; i < evidence_assignments_.size(); ++i) {
        const SemanticObject* target = assignment_matrix_->getAssignment(i, evidence_assignments_[i]).getTarget();
        if (target) {
            if (assigned_objects.find(target) != assigned_objects.end()) {
                return false;
            }
            assigned_objects.insert(target);
        }
    }
    return true;
    */
}

void AssignmentSet::print() const {
    cout << endl << "===== P = " << getProbability() << " ====" << endl;

    for(unsigned int i = 0; i < evidence_assignments_.size(); ++i) {
        for(unsigned int j = 0; j <  assignment_matrix_->getNumAssignments(i); ++j) {
            double prob = assignment_matrix_->getAssignment(i, j).getProbability();
            if (j == evidence_assignments_[i]) {
                std::cout << " (" << prob << ")";
            } else {
                std::cout << "  " << prob << " ";
            }
        }
        printf("\n");
    }

    for(unsigned int i = 0; i < evidence_assignments_.size(); ++i) {
        cout << assignment_matrix_->getAssignment(i, evidence_assignments_[i]).toString() << endl;
    }

    isValid();

    printf("\n");
}

}

