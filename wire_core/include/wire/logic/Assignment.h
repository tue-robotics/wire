/*
 * Assignment.h
 *
 *  Created on: Jul 28, 2011
 *      Author: sdries
 */

#ifndef WM_ASSIGNMENT_H_
#define WM_ASSIGNMENT_H_

#include <string>

namespace mhf {

class Evidence;
class SemanticObject;

class Assignment {

public:

    enum AssignmentType {
        NEW,
        EXISTING,
        CLUTTER
    };

    virtual ~Assignment();

    Assignment(AssignmentType type, const Evidence* evidence, const SemanticObject* target, double probability);

    AssignmentType getType() const;

    const Evidence* getEvidence() const;

    const SemanticObject* getTarget() const;

    double getProbability() const;

    SemanticObject* getNewObject() const;

    SemanticObject* getUpdatedObject() const;

    std::string toString() const;

protected:

    AssignmentType type_;

    const Evidence* evidence_;

    const SemanticObject* target_;

    double probability_;

    mutable SemanticObject* new_object_;

    mutable SemanticObject* updated_object_;
};

}

#endif /* ASSIGNMENT_H_ */
