#include "wire/WorldModelROS.h"
#include <std_msgs/Float32.h>

using namespace std;

double loop_rate_ = 20;

int main(int argc, char **argv) {

    // Initialize node
    ros::init(argc, argv, "WorldModel");
    ros::NodeHandle nh_private("~");

    // Initialize topic that publishes number of hypotheses
//    ros::Publisher numbHypotheses_pub = nh_private.advertise<std_msgs::Float32>("numbHypotheses", 1000);
//    ros::Rate loop_rate1(10);
//    while(ros::ok())
//    {
//        std_msgs::Float32 msg16;
//        msg16.data = 0.5f;
//        numbHypotheses_pub.publish(msg16);
//        ROS_INFO("Test");
//        ros::spinOnce();
//        loop_rate1.sleep();
//    }

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
