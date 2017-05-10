/*
 * visualiser.cpp
 *
 *  Created on: May 7, 2012
 *      Author: sdries
 */


#include "ros/ros.h"
#include "tf/transform_listener.h"

// World model messages
#include "wire_msgs/WorldEvidence.h"
#include "wire_msgs/WorldState.h"

// Visualizer class
#include "Visualizer.h"

// Conversions
#include "problib/conversions.h"

using namespace std;

//// Globals

// Visualizer instances for world model input and output
Visualizer world_evidence_visualizer_;
Visualizer world_state_visualizer_;

// Marker publishers
ros::Publisher world_evidence_marker_pub_;
ros::Publisher world_state_marker_pub_;

// Marker frame
string marker_frame_ = "";
const string DEFAULT_MARKER_FRAME = "/map";


/*
 * World evidence callback. Creates a marker for each of the world model objects
 */
void worldEvidenceCallback(const wire_msgs::WorldEvidence::ConstPtr& msg) {

	// ID for marker administration
	long ID = 0;

	// Iterate over all elements of evidence
	for (vector<wire_msgs::ObjectEvidence>::const_iterator it = msg->object_evidence.begin(); it != msg->object_evidence.end(); ++it) {

		// Get current evidence
		const wire_msgs::ObjectEvidence& obj_ev = *it;

		// Create marker
		visualization_msgs::MarkerArray markers_msg;
		world_evidence_visualizer_.createMarkers(msg->header, ID++, obj_ev.properties, markers_msg.markers, marker_frame_);

		// Publish marker
		world_evidence_marker_pub_.publish(markers_msg);

	}
}


/*
 * World state callback. Creates a marker for each detection.
 */
void worldStateCallback(const wire_msgs::WorldState::ConstPtr& msg) {


	// Iterate over world state objects
	for (vector<wire_msgs::ObjectState>::const_iterator it = msg->objects.begin(); it != msg->objects.end(); ++it) {

		// Get current object
		const wire_msgs::ObjectState& obj = *it;

		// Create marker
		visualization_msgs::MarkerArray markers_msg;
		world_state_visualizer_.createMarkers(msg->header, obj.ID, obj.properties, markers_msg.markers, marker_frame_);

		// Publish marker
		world_state_marker_pub_.publish(markers_msg);

	}
}


/*
 * Loads the marker frame from the parameter server. Uses the default frame is it is not defined.
 */
bool getMarkerFrame(ros::NodeHandle& n) {

	// Get marker frame from the parameter server
	XmlRpc::XmlRpcValue marker_frame;
	if (!n.getParam("marker_frame", marker_frame)) {
		ROS_WARN("No marker_frame defined, using default frame");
		return false;
	}

	// Check if the marker frame is of type string
	if (marker_frame.getType() != XmlRpc::XmlRpcValue::TypeString) {
		ROS_ERROR("Parameter marker_frame must be of type string, using default frame");
		return false;
	}

	// Store marker frame
	marker_frame_ = (string)marker_frame;

	return true;
}



int main(int argc, char **argv) {
	ros::init(argc, argv, "wire_viz");
	ros::NodeHandle n("~");

	// Set parameters for visualizers
	if (!world_evidence_visualizer_.setParameters(n, "world_evidence")) {
		return 0;
	}
	if (!world_state_visualizer_.setParameters(n, "world_state")) {
		return 0;
	}

	// Set tf listeners
	tf::TransformListener tf_listener;
	world_evidence_visualizer_.setTFListener(&tf_listener);
	world_state_visualizer_.setTFListener(&tf_listener);

	// Get marker frame from parameter server
	if (!getMarkerFrame(n)) marker_frame_ = DEFAULT_MARKER_FRAME;

	// Subscribe to world evidence and world state topics
	ros::Subscriber sub_world_ev = n.subscribe("/world_evidence", 10, &worldEvidenceCallback);
	ros::Subscriber sub_world_st = n.subscribe("/world_state", 10, &worldStateCallback);

	// Advertise to marker topics
	world_evidence_marker_pub_ = n.advertise<visualization_msgs::MarkerArray>("/visualization_markers/world_evidence", 10);
	world_state_marker_pub_ = n.advertise<visualization_msgs::MarkerArray>("/visualization_markers/world_state", 10);

	ros::spin();
	return 0;
}
