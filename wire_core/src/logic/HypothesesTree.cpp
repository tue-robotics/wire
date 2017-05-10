/*
 * HypothesesTree.cpp
 *
 *  Created on: March, 2011
 *  Author: Jos Elfring, Sjoerd van den Dries
 *  Affiliation: Eindhoven University of Technology
 */

#include "wire/logic/HypothesesTree.h"

#include "wire/logic/Assignment.h"
#include "wire/logic/AssignmentSet.h"
#include "wire/logic/AssignmentMatrix.h"
#include "wire/logic/Hypothesis.h"

#include "wire/storage/KnowledgeDatabase.h"
#include "wire/storage/ObjectStorage.h"
#include "wire/storage/SemanticObject.h"

#include "wire/core/Evidence.h"
#include "wire/core/EvidenceSet.h"
#include "wire/core/Property.h"
#include "wire/core/ClassModel.h"

#include <queue>
#include <cassert>
#include <float.h>

using namespace std;

#ifdef MHF_MEASURE_TIME
    #include <time.h>
#endif

//#define DEBUG_INFO(_msg, ...) printf(_msg, ##__VA_ARGS__)
#define DEBUG_INFO(_msg, ...)

namespace mhf {

/* ****************************************************************************** */
/* *                        CONSTRUCTOR / DESTRUCTOR                            * */
/* ****************************************************************************** */

HypothesisTree::HypothesisTree(int num_max_hyps, double max_min_prob_ratio) : n_updates_(0), t_last_update_(-1),
        tree_height_(0), num_max_hyps_(num_max_hyps), max_min_prob_ratio_(max_min_prob_ratio) {

    // create empty hypothesis (contains no objects) with timestep 0
    Hypothesis* empty_hyp = new Hypothesis(t_last_update_, 1.0);

    // add empty hypothesis to leaf list and set as root
    leafs_.push_back(empty_hyp);
    root_ = empty_hyp;
    MAP_hypothesis_ = empty_hyp;
}

HypothesisTree::~HypothesisTree() {
    root_->deleteChildren();
    delete root_;
}

/* ****************************************************************************** */
/* *                          PUBLIC MHT OPERATIONS                             * */
/* ****************************************************************************** */

void HypothesisTree::addEvidence(const EvidenceSet& ev_set) {
    DEBUG_INFO("HypothesesTree::processMeasurements\n");

    if (ev_set.size() == 0) {
        return;
    }

#ifdef MHF_MEASURE_TIME
    timespec t_start_total, t_end_total;
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &t_start_total);
#endif

    for(EvidenceSet::const_iterator it_ev = ev_set.begin(); it_ev != ev_set.end(); ++it_ev) {
        ObjectStorage::getInstance().match(**it_ev);
    }

    t_last_update_ = ev_set.getTimestamp();

    expandTree(ev_set);

    pruneTree(ev_set.getTimestamp());

    applyAssignments();

    // clear old hypotheses leafs
    // The hypotheses will still be there to form a tree, but do not contain any objects anymore
    root_->clearInactive();

    root_ = root_->deleteSinglePaths();

    tree_height_ = root_->calculateHeigth();

    DEBUG_INFO("*** Free memory: assignment matrices ***\n");

    ++n_updates_;

#ifdef MHF_MEASURE_TIME
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &t_end_total);
    printf("Total update took %f seconds.\n", (t_end_total.tv_sec - t_start_total.tv_sec) + double(t_end_total.tv_nsec - t_start_total.tv_nsec) / 1e9);
#endif

    DEBUG_INFO("HypothesesTree::processMeasurements - end\n");
}

/* ****************************************************************************** */
/* *                         PROTECTED MHT OPERATIONS                           * */
/* ****************************************************************************** */

struct compareAssignmentSets {
    bool operator()(const AssignmentSet* a1, const AssignmentSet* a2) const {
        return a1->getProbability() < a2->getProbability();
   }
};

void HypothesisTree::applyAssignments() {
    DEBUG_INFO("applyAssignments - begin\n");

    // iterate over all leaf hypotheses
    for (list<Hypothesis*>::iterator it = leafs_.begin(); it != leafs_.end(); ++it) {
        DEBUG_INFO("  materializing hyp %p, with parent %p\n", (*it), (*it)->getParent());
        (*it)->applyAssignments();
    }

    DEBUG_INFO("applyAssignments - end\n");
}

void HypothesisTree::expandTree(const EvidenceSet& ev_set) {

    DEBUG_INFO("expandTree - start\n");

    //** Create new objects based on measurements

    list<Assignment*> new_assignments;
    list<Assignment*> clutter_assignments;
    for(EvidenceSet::const_iterator it_ev = ev_set.begin(); it_ev != ev_set.end(); ++it_ev) {
        // new
        new_assignments.push_back(new Assignment(Assignment::NEW, *it_ev, 0, KnowledgeDatabase::getInstance().getProbabilityNew(**it_ev)));

        // clutter
        clutter_assignments.push_back(new Assignment(Assignment::CLUTTER, *it_ev, 0, KnowledgeDatabase::getInstance().getProbabilityClutter(**it_ev)));
    }

#ifdef MHF_MEASURE_TIME
    timespec t_start, t_end;
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &t_start);
#endif

    //** expand all current leaf hypotheses

    priority_queue<AssignmentSet*, vector<AssignmentSet*>, compareAssignmentSets > assignment_sets;

    DEBUG_INFO(" - Create assignment matrices and assignment sets\n");

    for (list<Hypothesis*>::iterator it_hyp = leafs_.begin(); it_hyp != leafs_.end(); ++it_hyp) {
        Hypothesis* hyp = *it_hyp;

        for(list<Assignment*>::iterator it_ass = new_assignments.begin(); it_ass != new_assignments.end(); ++it_ass) {
            hyp->addPotentialAssignment(*it_ass);
        }

        for(list<Assignment*>::iterator it_ass = clutter_assignments.begin(); it_ass != clutter_assignments.end(); ++it_ass) {
            hyp->addPotentialAssignment(*it_ass);
        }

        // sort assignment matrix
        hyp->getAssignmentMatrix()->sortAssignments();

        // create empty assignment set
        AssignmentSet* ass_set = new AssignmentSet(hyp, hyp->getAssignmentMatrix());
        assignment_sets.push(ass_set);
    }

    DEBUG_INFO(" - Generate hypotheses\n");

    double min_prob = 0;

    // set all current leafs to inactive
    for (list<Hypothesis*>::iterator it_hyp = leafs_.begin(); it_hyp != leafs_.end(); ++it_hyp) {
        (*it_hyp)->setInactive();
    }

    leafs_.clear();

    int n_iterations = 0;

    while(!assignment_sets.empty() && leafs_.size() < num_max_hyps_ && assignment_sets.top()->getProbability() > min_prob) {
        // assignment_sets.top()->print();

        ++n_iterations;
        DEBUG_INFO("  #assignment sets = %i\n", (int)assignment_sets.size());
        AssignmentSet* ass_set = assignment_sets.top();

        DEBUG_INFO("  inspecting assignment set %p with probability %.16f\n", ass_set, ass_set->getProbability());

        assignment_sets.pop();
        Hypothesis* hyp = ass_set->getHypothesis();

        if (ass_set->isValid()) {
            /* ************ assignment set is complete! Create hypothesis ************ */

            Hypothesis* hyp_child = new Hypothesis(ev_set.getTimestamp(), ass_set->getProbability());
            hyp_child->setAssignments(ass_set);
            hyp->addChildHypothesis(hyp_child);

            if (leafs_.empty()) {
                // first hypothesis found (and therefore the best one)
                min_prob = hyp_child->getProbability() * max_min_prob_ratio_;

                MAP_hypothesis_ = hyp_child;
            }

            /*
            if (leafs_.size() <= 3) {
                ass_set->print();
            }
            */

            //printf("%i: new leaf with prob %f\n", leafs_.size(), hyp_child->probability_);

            DEBUG_INFO(" NEW LEAF: %p\n", hyp_child);
            leafs_.push_back(hyp_child);
            DEBUG_INFO("  #leafs = %i, #old leafs = %i\n", (int)leafs_.size(), n_old_leafs);

            /* ************************************************************************* */
        }

        list<AssignmentSet*> child_assignment_sets;
        ass_set->expand(child_assignment_sets);
        for(list<AssignmentSet*>::iterator it_child = child_assignment_sets.begin(); it_child != child_assignment_sets.end(); ++it_child) {
            assignment_sets.push(*it_child);
        }

    }

    DEBUG_INFO(" - Free memory (remaining assignment sets)\n");

    assert(leafs_.size() > 0);

    // delete remaining assignment sets (the ones that where not used to generate hypotheses)
    while(!assignment_sets.empty()) {
        delete assignment_sets.top();
        assignment_sets.pop();
    }

    DEBUG_INFO(" ... done\n");

    ++tree_height_;

    normalizeProbabilities();

#ifdef MHF_MEASURE_TIME
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &t_end);
    printf("Expansion of hypotheses took %f seconds.\n", (t_end.tv_sec - t_start.tv_sec) + double(t_end.tv_nsec - t_start.tv_nsec) / 1e9);
#endif

    DEBUG_INFO("expandTree - done\n");
}

void HypothesisTree::normalizeProbabilities() {
    // Calculate sum of all probabilities
    double p_total = 0;
    for(list<Hypothesis* >::iterator it_hyp = leafs_.begin(); it_hyp != leafs_.end(); ++it_hyp) {
        p_total += (*it_hyp)->getProbability();
    }

    // Normalize all probabilities
    for(list<Hypothesis* >::iterator it_hyp = leafs_.begin(); it_hyp != leafs_.end(); ++it_hyp) {
        (*it_hyp)->setProbability((*it_hyp)->getProbability() / p_total);
    }

    root_->calculateBranchProbabilities();
}

void HypothesisTree::pruneTree(const Time& timestamp) {
    //return;


    DEBUG_INFO("pruneTree - begin\n");

    DEBUG_INFO("   old #leafs = %i\n", (int)leafs_.size());

    // determine best branch leaf per hypothesis

    DEBUG_INFO(" - determine best leaf per branch\n");
    root_->determineBestLeaf();
    DEBUG_INFO("   ... done\n");

    double prob_ratios[] = {1e-8, 1e-7, 1e-6, 1e-5, 1e-5, 1e-4, 1};

    list<Hypothesis*> hyp_stack;
    hyp_stack.push_back(root_);

    while(!hyp_stack.empty()) {
        Hypothesis* hyp = hyp_stack.front();
        hyp_stack.pop_front();

        list<Hypothesis*>& children = hyp->getChildHypotheses();
        if (!children.empty()) {

            // determine best branch of root hypothesis (highest sum of leaf probabilities)
            Hypothesis* best_child = *children.begin();
            for (list<Hypothesis*>::const_iterator it_child = children.begin(); it_child != children.end(); ++it_child) {
                if ((*it_child)->getProbability() > best_child->getProbability()) {
                    best_child = *it_child;
                }
                //printf(" (%p, %f)", *it_child, (*it_child)->getProbability());
            }

            double prob_ratio = 0;

            if (hyp->getHeight() > 6) {
                prob_ratio = 0;
            } else {
                prob_ratio = prob_ratios[hyp->getHeight()];
            }

            double min_prob = best_child->getProbability() * prob_ratio;

            for (list<Hypothesis*>::iterator it_child = children.begin(); it_child != children.end();) {
                bool prune_child = false;

                if (*it_child != best_child) {
                    if ((*it_child)->getProbability() == 0) {
                        prune_child = true;
                    } else if (hyp->getHeight() > 6) {
                        DEBUG_INFO(" - Determine hyp similarity between %p and %p\n", best_child->getBestLeaf(), (*it_child)->getBestLeaf());
                        double similarity = 1;
                        DEBUG_INFO("   ... done\n");

                        //printf("  similarity = %f\n", similarity);

                        prune_child = (similarity > 0.5);
                    } else if ((*it_child)->getProbability() < min_prob) {
                        prune_child = true;
                    }
                }

                if (prune_child) {
                    // prune hypothesis and all child hypothesis
                    (*it_child)->deleteChildren();
                    delete (*it_child);
                    it_child = children.erase(it_child);
                } else {
                    hyp_stack.push_front(*it_child);
                    ++it_child;
                }
            }

        }
    }

    // clear leaf list and add new leafs of tree
    leafs_.clear();
    root_->findActiveLeafs(leafs_);

    normalizeProbabilities();

    DEBUG_INFO("   #leafs after pruning = %i\n", (int)leafs_.size());

    DEBUG_INFO("pruneTree - end\n");
}

/* ****************************************************************************** */
/* *                                GETTERS                                     * */
/* ****************************************************************************** */

const list<Hypothesis*>& HypothesisTree::getHypotheses() const {
    return leafs_;
}

int HypothesisTree::getHeight() const {
    return tree_height_;
}


const Hypothesis& HypothesisTree::getMAPHypothesis() const {
    return *MAP_hypothesis_;
}

const list<SemanticObject*>& HypothesisTree::getMAPObjects() const {
    DEBUG_INFO("getMAPObjects - begin\n");
    return getMAPHypothesis().getObjects();
}


/* ****************************************************************************** */
/* *                              PRINT METHODS                                 * */
/* ****************************************************************************** */

void HypothesisTree::showStatistics() {
    cout << "   Number of hypotheses        = " << leafs_.size() << endl;
    cout << "   Max probability             = " << getMAPHypothesis().getProbability() << endl;
    cout << "   Tree height                  = " << tree_height_ << endl;
}

}
