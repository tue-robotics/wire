/** Hypothesis.h
 *
 * @class Hypothesis
 *
 * The hypothesis class is used to store information about an hypothesis. Protected members are trailed with an underscore. Each hypothesis, contains:
 * - An object list containing pointers to the object that exists according to the hypothesis
 * - Probability that the current hypothesis describes the actual state in the world
 *
 * Each hypothesis should be part of a multiple hypotheses tree. Expanding the tree requires expanding all hypotheses in the tree. Expanding an hypothesis
 * is always done on the basis of a measurement. Each measurement can:
 * - be a newly appeared object, the object should be added to the object list in the current hypothesis
 * - originate from one of the objects in the object list
 * - clutter, i.e., a false detection
 *
 * The hypothesis class contains member functions that allow expanding the current hypothesis.
 *
 * <HR>
 *
 * This file is part of the RoboEarth ROS world modeling package.
 *
 * It was originally created for <a href="http://www.roboearth.org/">RoboEarth</a>.
 * The research leading to these results has received funding from the European Union Seventh Framework Programme FP7/2007-2013 under grant agreement no248942 RoboEarth.
 *
 * Copyright (C) 2011 by <a href=" mailto:J.Elfring@tue.nl">Jos Elfring</a> and <a href=" mailto:S.v.d..Dries@tue.nl">Sjoerd van den Dries</a>, Eindhoven University of Technology
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

#ifndef HYPOTHESIS_H_
#define HYPOTHESIS_H_

#include <string>
#include <list>

namespace mhf {

class AssignmentSet;
class SemanticObject;
class Assignment;
class ProbabilityModel;
class AssignmentMatrix;

class Hypothesis {

public:
        /* CONSTRUCTORS / DESTRUCTORS */

    // Constructor
    Hypothesis(const double& global_timestamp, double probability = -1);

    // Destructor
    virtual ~Hypothesis();

    Hypothesis* clone() const;


        /* GETTERS */

    const AssignmentSet* getAssignments() const;

    const Hypothesis* getBestLeaf() const;

    std::list<Hypothesis*>& getChildHypotheses();

    int getHeight() const;

    // returns the number of objects in the hypothesis
    int getNumObjects() const;

    // returns the list of objects contained in this hypothesis
    const std::list<SemanticObject*>& getObjects() const;

    //double getObjectProbability(MHTObject* obj) const;

    const Hypothesis* getParent() const;

    // returns probability of this hypothesis
    double getProbability() const;

    double getTimestamp() const;

    AssignmentMatrix* getAssignmentMatrix() const;

        /* SETTERS */

    void setAssignments(AssignmentSet* assignments);

    void setInactive();

    void setProbability(double prob);

    //void setObjectProbability(MHTObject* obj, double p);

        /* HYPOTHESIS MODIFIERS */

    void addChildHypothesis(Hypothesis* h);

    // Add object to the object list
    void addObject(SemanticObject* obj);

    void clearAssignmentSet();

    void clear();

    void addPotentialAssignment(Assignment* assignment);

    void applyAssignments();

        /* TREE UPDATE METHODS  */

    void findActiveLeafs(std::list<Hypothesis*>& active_leafs);

    double calculateBranchProbabilities();

    int calculateHeigth();

    Hypothesis* determineBestLeaf();


        /* TREE CLEAR / DELETE METHODS */

    void clearInactive();

    void deleteChildren();

    Hypothesis* deleteSinglePaths();

protected:

    double probability_;

    double timestamp_;

    std::list<SemanticObject*> objects_;

    Hypothesis* parent_;

    std::list<Hypothesis*> children_;

    AssignmentSet* assignment_set_;

    AssignmentMatrix* assignment_matrix_;

    Hypothesis* best_leaf_;

    int height_;

    bool is_active_leaf_;

};

}
#endif /* HYPOTHESIS_H_ */
