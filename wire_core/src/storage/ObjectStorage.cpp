#include "wire/storage/ObjectStorage.h"

#include "wire/storage/KnowledgeDatabase.h"
#include "wire/storage/SemanticObject.h"
#include "wire/core/Evidence.h"

using namespace std;

namespace mhf {

ObjectStorage* ObjectStorage::instance_ = 0;

ObjectStorage& ObjectStorage::getInstance() {
    if (instance_) {
        return *instance_;
    }
    instance_ = new ObjectStorage();
    return *instance_;
}

ObjectStorage::ObjectStorage() : ID_(0), knowledge_db_(KnowledgeDatabase::getInstance()) {

}

ObjectStorage::~ObjectStorage() {

}

void ObjectStorage::addObject(SemanticObject* obj) {
    objects_.push_back(obj);
    obj->it_obj_storage_ = --objects_.end();
}

void ObjectStorage::removeObject(SemanticObject& obj) {
    objects_.erase(obj.it_obj_storage_);
}

long ObjectStorage::getUniqueID() {
    return ID_++;
}

void ObjectStorage::match(const Evidence& ev) {

    //cout << endl << "ObjectStorage::match" << endl;

    for(list<SemanticObject*>::iterator it_obj = objects_.begin(); it_obj != objects_.end(); ++it_obj) {
        SemanticObject& obj = **it_obj;
        obj.propagate(ev.getTimestamp());
    }

    for(list<SemanticObject*>::iterator it_obj = objects_.begin(); it_obj != objects_.end(); ++it_obj) {
        SemanticObject& obj = **it_obj;

        double prob_existing = KnowledgeDatabase::getInstance().getProbabilityExisting(ev, obj);
        if (prob_existing > 0) {

            //cout << "Adding evidence " << &ev << " to object " << &obj << endl;

            obj.addPotentialAssignment(ev, prob_existing);
        }
    }
}

}
