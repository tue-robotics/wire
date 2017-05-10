/*
 * Hypothesis.cpp
 *
 *  Created on: March, 2011
 *  Author: Jos Elfring, Sjoerd van den Dries
 *  Affiliation: Eindhoven University of Technology
 */

#include "wire/logic/Hypothesis.h"

#include "wire/logic/AssignmentSet.h"
#include "wire/logic/Assignment.h"
#include "wire/logic/AssignmentMatrix.h"
#include "wire/storage/SemanticObject.h"
#include "wire/storage/ObjectStorage.h"

using namespace std;

namespace mhf {

/* ****************************************************************************** */
/* *                        CONSTRUCTOR / DESTRUCTOR                            * */
/* ****************************************************************************** */

Hypothesis::Hypothesis(const double& timestamp, double probability) : probability_(probability), timestamp_(timestamp),
    parent_(0), assignment_set_(0), assignment_matrix_(0), height_(0), is_active_leaf_(true) {
}

Hypothesis::~Hypothesis() {
    clear();
    delete assignment_matrix_;
}

/* ****************************************************************************** */
/* *                                 GETTERS                                    * */
/* ****************************************************************************** */


const AssignmentSet* Hypothesis::getAssignments() const {
    return assignment_set_;
}

const Hypothesis* Hypothesis::getBestLeaf() const {
    return best_leaf_;
}


list<Hypothesis*>& Hypothesis::getChildHypotheses() {
    return children_;
}

int Hypothesis::getHeight() const {
    return height_;
}

int Hypothesis::getNumObjects() const {
    return objects_.size();
}

const list<SemanticObject*>& Hypothesis::getObjects() const {
    return objects_;
}


const Hypothesis* Hypothesis::getParent() const {
    return parent_;
}

double Hypothesis::getProbability() const {
    return probability_;
}


double Hypothesis::getTimestamp() const {
    return timestamp_;
}

AssignmentMatrix* Hypothesis::getAssignmentMatrix() const {
    return assignment_matrix_;
}

/* ****************************************************************************** */
/* *                                 SETTERS                                    * */
/* ****************************************************************************** */


void Hypothesis::setAssignments(AssignmentSet* assignments) {
    if (assignment_set_) {
        delete assignment_set_;
    }
    assignment_set_ = assignments;
}


void Hypothesis::setInactive() {
    is_active_leaf_ = false;
}

void Hypothesis::setProbability(double prob) {
    probability_ = prob;
}

/* ****************************************************************************** */
/* *                           HYPOTHESIS MODIFIERS                             * */
/* ****************************************************************************** */


void Hypothesis::addChildHypothesis(Hypothesis* h) {
    h->parent_ = this;
    this->is_active_leaf_ = false;
    children_.push_back(h);
}

void Hypothesis::addObject(SemanticObject* obj) {
    objects_.push_back(obj);
    obj->addToHypothesis(this);
}


void Hypothesis::clearAssignmentSet() {
    delete assignment_set_;
    assignment_set_ = 0;
}

void Hypothesis::addPotentialAssignment(Assignment* assignment) {
    if (!assignment_matrix_) {
        assignment_matrix_ = new AssignmentMatrix();
    }
    assignment_matrix_->addPotentialAssignment(*assignment);
}

void Hypothesis::applyAssignments() {

    list<const Assignment*> all_assignments;
    assignment_set_->getAllAssignments(all_assignments);

    // apply cases without target
    for(list<const Assignment*>::iterator it_ass = all_assignments.begin(); it_ass != all_assignments.end();) {
        const Assignment* ass = *it_ass;

        if (ass->getType() == Assignment::CLUTTER) {
            // remove assignment from list
            it_ass = all_assignments.erase(it_ass);
        } else if (ass->getType() == Assignment::NEW) {
            SemanticObject* new_obj = ass->getNewObject();
            addObject(new_obj);

            // remove assignment from list
            it_ass = all_assignments.erase(it_ass);
        } else {
            ++it_ass;
        }
    }

    // apply cases with target
    const list<SemanticObject*>& hyp_parent_objs = parent_->getObjects();
    for (list<SemanticObject*>::const_iterator it_obj = hyp_parent_objs.begin(); it_obj != hyp_parent_objs.end(); ++it_obj) {
        SemanticObject* obj = *it_obj;

        const Assignment* update_ass = 0;

        for(list<const Assignment*>::iterator it_ass = all_assignments.begin(); it_ass != all_assignments.end();) {
            const Assignment* ass = *it_ass;

            if (obj == ass->getTarget()) {
                update_ass = ass;
                it_ass = all_assignments.erase(it_ass);
            } else {
                ++it_ass;
            }
        }

        if (update_ass) {
            SemanticObject* updated_obj = update_ass->getUpdatedObject();
            addObject(updated_obj);
        } else {
            addObject(obj);
        }
    }

    assert(all_assignments.empty());
    clearAssignmentSet();
}

/* ****************************************************************************** */
/* *                           TREE UPDATE METHODS                              * */
/* ****************************************************************************** */

double Hypothesis::calculateBranchProbabilities() {
    if (is_active_leaf_) {
        return probability_;
    }

    probability_ = 0;

    for (list<Hypothesis*>::const_iterator it = children_.begin(); it != children_.end(); ++it) {
        probability_ += (*it)->calculateBranchProbabilities();
    }
    return probability_;
}


int Hypothesis::calculateHeigth() {
    height_ = 0;
    for (list<Hypothesis*>::const_iterator it = children_.begin(); it != children_.end(); ++it) {
        height_ = max(height_, (*it)->calculateHeigth() + 1);
    }
    return height_;
}


Hypothesis* Hypothesis::determineBestLeaf() {
    if (is_active_leaf_) {
        return this;
    }
    best_leaf_ = 0;
    for (list<Hypothesis*>::const_iterator it = children_.begin(); it != children_.end(); ++it) {
        Hypothesis* child_best_leaf_ = (*it)->determineBestLeaf();
        if (best_leaf_ == 0 || (child_best_leaf_ != 0 && child_best_leaf_->getProbability() > best_leaf_->getProbability())) {
            best_leaf_ = child_best_leaf_;
        }
    }
    return best_leaf_;

}


void Hypothesis::findActiveLeafs(list<Hypothesis*>& active_leafs) {
    if (is_active_leaf_) {
        active_leafs.push_back(this);
        return;
    }

    for (list<Hypothesis*>::const_iterator it = children_.begin(); it != children_.end(); ++it) {
        (*it)->findActiveLeafs(active_leafs);
    }

}

/* ****************************************************************************** */
/* *                       TREE CLEAR / DELETE METHODS                          * */
/* ****************************************************************************** */


void Hypothesis::clear() {
    // remove this hypothesis from the hypothesis list of all objects contained in this hypothesis
    for (list<SemanticObject*>::iterator it_obj = objects_.begin(); it_obj != objects_.end(); ++it_obj) {
        SemanticObject* obj = *it_obj;
        obj->removeFromHypothesis(this);
        if (obj->getNumParentHypotheses() == 0) {
            ObjectStorage::getInstance().removeObject(*obj);
            delete obj;
        }
    }

    objects_.clear();

    is_active_leaf_ = false;

    // remove any remaining assignments
    if (assignment_set_) {
        delete assignment_set_;
    }
}


void Hypothesis::clearInactive() {
    if (!is_active_leaf_) {
        clear();
        for (list<Hypothesis*>::const_iterator it = children_.begin(); it != children_.end(); ++it) {
            (*it)->clearInactive();
        }
    }
}


void Hypothesis::deleteChildren() {
    // delete all child hypotheses
    for (list<Hypothesis*>::iterator it = children_.begin(); it != children_.end(); ++it) {
        (*it)->deleteChildren();
        delete (*it);
    }
}


Hypothesis* Hypothesis::deleteSinglePaths() {
    for (list<Hypothesis*>::iterator it_child = children_.begin(); it_child != children_.end();) {
        Hypothesis* new_child = (*it_child)->deleteSinglePaths();
        if (new_child != *it_child) {
            it_child = children_.erase(it_child);
            children_.insert(it_child, new_child);
        } else {
            ++it_child;
        }
    }

    if (children_.size() == 1) {
        Hypothesis* hyp = *children_.begin();
        hyp->parent_ = this->parent_;
        delete this;
        return hyp;
    }

    return this;
}

}
