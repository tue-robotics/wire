#ifndef WORLDMODELROS_H_
#define WORLDMODELROS_H_

#include "wire/core/datatypes.h"

// ros
#include "ros/ros.h"

#include "wire_msgs/WorldEvidence.h"
#include "wire_msgs/ObjectEvidence.h"

#include "wire_msgs/WorldState.h"
#include "wire_msgs/ObjectState.h"

// services
#include "std_srvs/Empty.h"  // for reset 'button'

#include "problib/conversions.h"

// transform listener
#include "tf/transform_listener.h"

#include "wire/storage/KnowledgeDatabase.h"

namespace mhf {

class HypothesisTree;
class Hypothesis;
class KnowledgeProbModel;
class PropertySet;
class ClassModel;
class SemanticObject;

class WorldModelROS {

public:

    WorldModelROS(tf::TransformListener* tf_listener = 0);

    virtual ~WorldModelROS();

    void registerEvidenceTopic(const std::string& topic_name);

    void publish() const;

    void showStatistics() const;

    void processEvidence(const ros::Duration max_duration);

    void processEvidence(const wire_msgs::WorldEvidence& world_evidence_msg);

    void start();

    void startThreaded();

    const std::list<SemanticObject*>& getMAPObjects() const;

protected:

    boost::thread processing_thread_;

    double loop_rate_;

    // evidence buffer
    std::list<wire_msgs::WorldEvidence> evidence_buffer_;

    // multiple hypothesis filter_
    mhf::HypothesisTree* world_model_;

    // transform listener
    tf::TransformListener* tf_listener_;
    bool is_tf_owner_;

    // world model publishers
    ros::Publisher pub_wm_;

    // evidence subscriber
    std::list<ros::Subscriber> subs_evidence_;

    ros::ServiceServer srv_reset_;

    // computation time needed for last tree update
    double last_update_duration;
    double max_update_duration;

    // frame in which objects are tracked and stored in world model
    std::string world_model_frame_id_;

    // frame in which objects are published
    std::string output_frame_id_;

    std::stringstream warnings_;

    int max_num_hyps_;

    double min_prob_ratio_;

    ros::Time last_update_;

    bool initialize();

    void initializeMHF();

    bool objectToMsg(const SemanticObject& obj, wire_msgs::ObjectState& msg) const;

    bool hypothesisToMsg(const mhf::Hypothesis& hyp, wire_msgs::WorldState& msg) const;

    void printWorldObjects(const mhf::Hypothesis& hyp) const;

    bool transformPosition(const pbl::PDF& pdf_in, const std::string& frame_in, pbl::Gaussian& pdf_out) const;

    bool transformOrientation(const pbl::PDF& pdf_in, const std::string& frame_in, pbl::Gaussian& pdf_out) const;

    void evidenceCallback(const wire_msgs::WorldEvidence::ConstPtr& world_evidence_msg);

    bool resetWorldModel(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);

    void shutdown();

};

}

#endif /* WORLDMODELROS_H_ */
