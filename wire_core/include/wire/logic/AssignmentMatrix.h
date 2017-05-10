/*
 * Assignment.h
 *
 *  Created on: Jul 28, 2011
 *      Author: sdries
 */

#ifndef ASSIGNMENTMATRIX_H_
#define ASSIGNMENTMATRIX_H_

#include <vector>
#include <map>

namespace mhf {

class Evidence;
class SemanticObject;
class Assignment;

class AssignmentMatrix {

public:

    AssignmentMatrix();

    virtual ~AssignmentMatrix();

    void addPotentialAssignment(const Assignment& ass);

    const Assignment& getAssignment(unsigned int i_ev, int i_assignment);

    unsigned int getNumAssignments(unsigned int i_ev);

    void sortAssignments();

    unsigned int getNumMeasurements() const;

protected:

    std::map<const Evidence*, unsigned int> evidence_to_index_;

    std::vector< std::vector<const Assignment*> > assignments_;

};

}

#endif /* ASSIGNMENTMATRIX_H_ */
