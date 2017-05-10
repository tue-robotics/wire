#include "wire/WorldModelROS.h"

#include "wire/core/ClassModel.h"
#include "wire/core/Property.h"
#include "wire/core/Evidence.h"
#include "wire/core/EvidenceSet.h"
#include "wire/storage/SemanticObject.h"
#include "wire/util/ObjectModelParser.h"

#include "wire/logic/Hypothesis.h"
#include "wire/logic/HypothesesTree.h"

#include "boost/thread.hpp"

using namespace std;
using namespace mhf;

WorldModelROS::WorldModelROS(tf::TransformListener* tf_listener)
    : loop_rate_(20), world_model_(0),  tf_listener_(tf_listener), is_tf_owner_(false), last_update_duration(0),
      max_update_duration(0), world_model_frame_id_("/map"), output_frame_id_("/map"), max_num_hyps_(100), min_prob_ratio_(1e-10),
      last_update_(0) {
    initialize();
}

WorldModelROS::~WorldModelROS() {

    // delete tf listener
    if (is_tf_owner_) {
        delete tf_listener_;
    }

    // delete multiple hypothesis filter_
    delete world_model_;

    // shutdown subscribers
    for (list<ros::Subscriber>::iterator it = subs_evidence_.begin(); it != subs_evidence_.end(); ++it) {
        it->shutdown();
    }
}

bool WorldModelROS::initialize() {
    // get node handle
    ros::NodeHandle n("~");

    // set parameters
    n.getParam("world_model_frame", world_model_frame_id_);
    n.getParam("output_frame", output_frame_id_);
    n.getParam("max_num_hypotheses", max_num_hyps_);
    n.getParam("min_probability_ratio", min_prob_ratio_);

    string object_models_filename = "";
    n.getParam("knowledge_filename", object_models_filename);

    if (object_models_filename == "") {
        ROS_ERROR("Parameter 'knowledge_filename' not set.");
        return false;
    }

    // Parse object models
    // ObjectModelParser parser(object_models_filename);
    ObjectModelParser parser(object_models_filename);
    if (!parser.parse(KnowledgeDatabase::getInstance())) {
        // parsing failed
        ROS_ERROR_STREAM("While parsing '" << object_models_filename << "': " << endl << endl << parser.getErrorMessage());
        return false;
    }

    // create tf listener
    if (!tf_listener_) {
        tf_listener_ = new tf::TransformListener();
        is_tf_owner_ = true;
    }

    // Service for resetting the world model
    srv_reset_ = n.advertiseService("reset", &WorldModelROS::resetWorldModel, this);

    // Publishers
    pub_wm_ = n.advertise<wire_msgs::WorldState>("/world_state", 10);

    // initialize the filter
    world_model_ = new HypothesisTree(max_num_hyps_, min_prob_ratio_);

    return true;
}

void WorldModelROS::registerEvidenceTopic(const std::string& topic_name) {
    ros::NodeHandle n;
    subs_evidence_.push_back(n.subscribe(topic_name, 100, &WorldModelROS::evidenceCallback, this));
}

void WorldModelROS::startThreaded() {
    processing_thread_ = boost::thread(boost::bind(&WorldModelROS::start, this));
}

void WorldModelROS::start() {
    ros::Rate r(loop_rate_);

    int count = 0;
    while(ros::ok()) {
        ros::spinOnce();
        processEvidence(r.expectedCycleTime());
        publish();
        ++count;
        if (count == 15) {
            //showStatistics();
            count = 0;
        }
        r.sleep();
    }

    ros::spinOnce();
}

bool WorldModelROS::objectToMsg(const SemanticObject& obj, wire_msgs::ObjectState& msg) const {
    msg.ID = obj.getID();

    const map<Attribute, Property*>& properties = obj.getPropertyMap();

    for(map<Attribute, Property*>::const_iterator it_prop = properties.begin(); it_prop != properties.end(); ++it_prop) {
        Property* prop = it_prop->second;

        wire_msgs::Property prop_msg;
        prop_msg.attribute = AttributeConv::attribute_str(it_prop->first);
        pbl::PDFtoMsg(prop->getValue(), prop_msg.pdf);
        msg.properties.push_back(prop_msg);
    }

    return true;
}

bool WorldModelROS::hypothesisToMsg(const Hypothesis& hyp, wire_msgs::WorldState& msg) const {
    ros::Time time = ros::Time::now();

    msg.header.frame_id = world_model_frame_id_;
    msg.header.stamp = time;

    for(list<SemanticObject*>::const_iterator it = hyp.getObjects().begin(); it != hyp.getObjects().end(); ++it) {

        SemanticObject* obj_clone = (*it)->clone();
        obj_clone->propagate(time.toSec());

        wire_msgs::ObjectState obj_msg;
        if (objectToMsg(*obj_clone, obj_msg)) {
            msg.objects.push_back(obj_msg);
        }

        delete obj_clone;

    }

    return true;
}

bool WorldModelROS::transformPosition(const pbl::PDF& pdf_in, const string& frame_in, pbl::Gaussian& pdf_out) const {
    const pbl::Gaussian* gauss = pbl::PDFtoGaussian(pdf_in);

    if (!gauss) {
        ROS_ERROR("Position evidence is not a gaussian!");
        return false;
    }

    const arma::vec& pos = gauss->getMean();
    tf::Stamped<tf::Point> pos_stamped(tf::Point(pos(0), pos(1), pos(2)), ros::Time(), frame_in);

    try{
        tf::Stamped<tf::Point> pos_stamped_world;
        tf_listener_->transformPoint(world_model_frame_id_, pos_stamped, pos_stamped_world);

        pbl::Vector3 pos_world(pos_stamped_world.getX(), pos_stamped_world.getY(), pos_stamped_world.getZ());
        pdf_out.setMean(pos_world);

        // todo: also transform covariance
        pdf_out.setCovariance(gauss->getCovariance());

    } catch (tf::TransformException& ex){
        ROS_ERROR("[WORLD_MODEL] %s",ex.what());
        return false;
    }
    return true;
}

bool WorldModelROS::transformOrientation(const pbl::PDF& pdf_in, const string& frame_in, pbl::Gaussian& pdf_out) const {
    const pbl::Gaussian* gauss = pbl::PDFtoGaussian(pdf_in);

    if (!gauss) {
        ROS_ERROR("Orientation evidence is not a gaussian!");
        return false;
    }

    const arma::vec& ori = gauss->getMean();
    tf::Stamped<tf::Quaternion> ori_stamped(tf::Quaternion(ori(0), ori(1), ori(2), ori(3)), ros::Time(), frame_in);

    try{
        tf::Stamped<tf::Quaternion> ori_stamped_world;
        tf_listener_->transformQuaternion(world_model_frame_id_, ori_stamped, ori_stamped_world);

        pbl::Vector4 ori_world(ori_stamped_world.getX(), ori_stamped_world.getY(), ori_stamped_world.getZ(), ori_stamped_world.getW());
        pdf_out.setMean(ori_world);

        // todo: also transform covariance
        pdf_out.setCovariance(gauss->getCovariance());

    } catch (tf::TransformException& ex){
        ROS_ERROR("[WORLD MODEL] %s",ex.what());
        return false;
    }
    return true;
}

void WorldModelROS::evidenceCallback(const wire_msgs::WorldEvidence::ConstPtr& world_evidence_msg) {
    evidence_buffer_.push_back(*world_evidence_msg);
}

void WorldModelROS::processEvidence(const ros::Duration max_duration) {

    ros::Time start_time = ros::Time::now();

    while(!evidence_buffer_.empty() && ros::Time::now() - start_time < max_duration) {

        ros::Time time_before_update = ros::Time::now();

        processEvidence(evidence_buffer_.back());

        last_update_duration = (ros::Time::now().toSec() - time_before_update.toSec());
        max_update_duration = max(max_update_duration, last_update_duration);

        evidence_buffer_.pop_back();
    }
}

void WorldModelROS::processEvidence(const wire_msgs::WorldEvidence& world_evidence_msg) {
    ros::Time current_time = ros::Time::now();

    if (current_time < last_update_) {
        ROS_WARN("Saw a negative time change of %f seconds; resetting the world model.", (current_time - last_update_).toSec());
        delete world_model_;
        world_model_ = new HypothesisTree(max_num_hyps_, min_prob_ratio_);
    }
    last_update_ = current_time;

    // reset the warnings stringstream
    warnings_.str("");

    EvidenceSet evidence_set;
    list<Evidence*> measurements_mem;

    const vector<wire_msgs::ObjectEvidence>& object_evidence = world_evidence_msg.object_evidence;
    for(vector<wire_msgs::ObjectEvidence>::const_iterator it_ev = object_evidence.begin(); it_ev != object_evidence.end(); ++it_ev) {
        const wire_msgs::ObjectEvidence& evidence = (*it_ev);

        //Evidence* meas = new Evidence(world_evidence_msg->header.stamp.toSec(), evidence.certainty, evidence.negative);
        Evidence* meas = new Evidence(current_time.toSec());

        measurements_mem.push_back(meas);

        bool position_ok = true;

        for(vector<wire_msgs::Property>::const_iterator it_prop = evidence.properties.begin(); it_prop != evidence.properties.end(); ++it_prop ) {
            const wire_msgs::Property& prop = *it_prop;

            pbl::PDF* pdf = pbl::msgToPDF(prop.pdf);

            if (pdf) {
                if (prop.attribute == "position") {
                    pbl::Gaussian pos_pdf(3);
                    if (!transformPosition(*pdf, world_evidence_msg.header.frame_id, pos_pdf)) {
                        // position is a necessary property. If the transform was not successful, abort and don't use the evidence
                        position_ok = false;
                        break;
                    } else {
                        meas->addProperty(AttributeConv::attribute(prop.attribute), pos_pdf);
                    }
                } else if (prop.attribute == "orientation") {
                    pbl::Gaussian ori_pdf(4);
                    if (!transformOrientation(*pdf, world_evidence_msg.header.frame_id, ori_pdf)) {
                        meas->addProperty(AttributeConv::attribute(prop.attribute), ori_pdf);
                    }
                } else {
                    meas->addProperty(AttributeConv::attribute(prop.attribute), *pdf);
                }
                delete pdf;
            } else {
                ROS_ERROR_STREAM("For attribute '" << prop.attribute << "': malformed pdf: " << prop.pdf);
            }
        }

        if (position_ok) {                
            evidence_set.add(meas);
        } else {
            ROS_ERROR("Unable to transform position.");
        }

    } // end iteration over object evidence list

    world_model_->addEvidence(evidence_set);

    for(list<Evidence*>::iterator it = measurements_mem.begin(); it != measurements_mem.end(); ++it) {
        delete (*it);
    }
}

bool WorldModelROS::resetWorldModel(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
    delete world_model_;
    world_model_ = new HypothesisTree(max_num_hyps_, min_prob_ratio_);
    return true;
}

void WorldModelROS::publish() const {
    wire_msgs::WorldState map_world_msg;
    hypothesisToMsg(world_model_->getMAPHypothesis(), map_world_msg);

    // Publish results
    pub_wm_.publish(map_world_msg);

}

const list<SemanticObject*>& WorldModelROS::getMAPObjects() const {
    return world_model_->getMAPObjects();
}

void WorldModelROS::showStatistics() const {
    printf("***** %f *****\n", ros::Time::now().toSec());
    world_model_->showStatistics();
    cout << "Num MAP objects:      " << world_model_->getMAPObjects().size() << endl;
    cout << "Last update:          " << last_update_duration << " seconds" << endl;
    cout << "Max update:           " << max_update_duration << " seconds" << endl;
    cout << "Evidence buffer size: " << evidence_buffer_.size() << endl;

    /*
    const list<Hypothesis*>& hyp_list = world_model_->getHypotheses();
    for(list<Hypothesis* >::const_iterator it_hyp = hyp_list.begin(); it_hyp != hyp_list.end(); ++it_hyp) {

        const list<SemanticObject*>& objs = (*it_hyp)->getObjects();

        double hyp_prob = (*it_hyp)->getProbability();

        cout << "Hyp P = " << hyp_prob << ": " << objs.size() << " object(s)" << endl;

    }
    */

    if (!warnings_.str().empty()) {
        ROS_WARN_STREAM(warnings_.str());
    }
}
