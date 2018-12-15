/** HypothesesTree.h
 * @class HypothesesTree
 *
 * The hypothesis tree class is used to store all hypotheses. Protected members are trailed with an underscore. Each time a new measurement arrives, the tree
 * is expanded and the most hypothesis with the maximum a posteriori probability (MAP) is selected. The MAP hypothesis is believed to represent the current
 * state of the world. The number of hypotheses in a tree grows more than exponential with the number of measurement and, therefore, this class includes
 * a pruning function that after each measurement prunes the least probable hypotheses. A hypotheses tree contains:
 * - a multiset containing pointers to all hypotheses
 * - a list containing pointers to all objects that exist in all hypotheses
 * - a pointer to the MAP hypothesis
 *
 * \authors Jos Elfring, Sjoerd van den Dries
 * \date March, 2011
 * \version 1.0
 *
 * <HR>
 *
 * This file is part of the RoboEarth ROS world modeling package.
 *
 * It was originally created for <a href="http://www.roboearth.org/">RoboEarth</a>.
 * The research leading to these results has received funding from the European Union Seventh Framework Programme FP7/2007-2013 under grant agreement no248942 RoboEarth.
 *
 * Copyright (C) 2011 by <a href=" mailto:J.Elfring@tue.nl">Jos Elfring</a> and <a href=" mailto:S.v.d.Dries@tue.nl">Sjoerd van den Dries</a>, Eindhoven University of Technology
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *    <UL>
 *     <LI> Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     <LI> Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     <LI> Neither the name of Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *    </UL>
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \image html http://www.roboearth.org/sites/default/files/RoboEarth.org_logo.gif
 * \image latex http://www.roboearth.org/sites/default/files/RoboEarth.org_logo.gif
 *
 * \authors Jos Elfring, Sjoerd van den Dries
 * \date March, 2011
 * \version 1.0
 */

#ifndef HYPOTHESESTREE_H_
#define HYPOTHESESTREE_H_

#include "wire/core/datatypes.h"
#include <list>
#include <map>

namespace mhf {

class Evidence;
class SemanticObject;
class Hypothesis;
class AssignmentMatrix;
class KnowledgeDatabase;
class EvidenceSet;

class HypothesisTree {

public:

    /* CONSTRUCTORS / DESTRUCTORS */

    // Constructor
    HypothesisTree(int num_max_hyps, double max_min_prob_ratio);

    // Destructor
    virtual ~HypothesisTree();


    /* GETTERS */

    const std::list<SemanticObject*>& getMAPObjects() const;

    const std::list<SemanticObject*>& getAllObjects() const;

    const std::list<Hypothesis*>& getHypotheses() const;

    const Hypothesis& getMAPHypothesis() const;

    int getHeight() const;


    /* SETTERS */

    void addEvidence(const EvidenceSet& ev_set);


    /* PRINT METHODS */

    void showStatistics();

protected:

    Hypothesis* root_;

    std::list<Hypothesis*> leafs_;

    Hypothesis* MAP_hypothesis_;

    long n_updates_;

    double t_last_update_;

    int tree_height_;

    unsigned int num_max_hyps_;

    /** ratio between max and min hypothesis probability **/
    double max_min_prob_ratio_;

    void applyAssignments();

    //void determineMAPHypothesis();

    void expandTree(const EvidenceSet &ev_set);

    // Normalize the probabilities of all hypotheses in the tree
    void normalizeProbabilities();

    // Prune the tree
    void pruneTree(const Time& timestamp);

};

}
#endif /* HYPOTHESESTREE_H_ */
