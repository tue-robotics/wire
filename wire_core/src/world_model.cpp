#include "wire/WorldModelROS.h"

using namespace std;

double loop_rate_ = 20;

int main(int argc, char **argv) {

    // Initialize node
    ros::init(argc, argv, "WorldModel");
    ros::NodeHandle nh_private("~");

    mhf::WorldModelROS wm;

    /* * * * * * * * * Load evidence topic names from parameters * * * * * * * * * */

    XmlRpc::XmlRpcValue evidence_topics;
    nh_private.getParam("evidence_topics", evidence_topics);

    if (evidence_topics.getType() != XmlRpc::XmlRpcValue::TypeArray) {
        ROS_ERROR("Parameter 'evidence_topics' should be an array.");
        return -1;
    }

    for (int i = 0; i < evidence_topics.size(); ++i) {
        XmlRpc::XmlRpcValue& evidence_topic = evidence_topics[i];

        if (evidence_topic.getType() != XmlRpc::XmlRpcValue::TypeString) {
            ROS_ERROR("Each evidence topic must be a string.");
            return -1;
        }

        wm.registerEvidenceTopic((string)evidence_topic);
    }

    /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

    wm.startThreaded();

    ros::spin();

    return 0;
}
