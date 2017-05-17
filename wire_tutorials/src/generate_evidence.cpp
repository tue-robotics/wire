/*
 * generate_evidence.cpp
 *
 *  Created on: Nov 12, 2012
 *      Author: jelfring
 */

#include <ros/ros.h>

#include "wire_msgs/WorldEvidence.h"
#include "wire_msgs/ObjectEvidence.h"

#include "problib/conversions.h"

// Publisher used to send evidence to world model
ros::Publisher world_evidence_publisher_;


void addEvidence(wire_msgs::WorldEvidence& world_evidence, double x, double y, double z, const std::string& class_label, const std::string& color) {
	wire_msgs::ObjectEvidence obj_evidence;

	// Set the continuous position property
	wire_msgs::Property posProp;
	posProp.attribute = "position";

	// Set position (x,y,z), set the covariance matrix as 0.005*identity_matrix
	pbl::PDFtoMsg(pbl::Gaussian(pbl::Vector3(x, y, z), pbl::Matrix3(0.0005, 0.0005, 0.0005)), posProp.pdf);
	obj_evidence.properties.push_back(posProp);

	// Set the continuous orientation property
	wire_msgs::Property oriProp;
	oriProp.attribute = "orientation";

	// Set the orientation (0,0,0,1), with covariance matrix 0.01*identity_matrix
	pbl::PDFtoMsg(pbl::Gaussian(pbl::Vector4(0, 0, 0, 1), pbl::Matrix4(0.01, 0.01, 0.01, 0.01)), oriProp.pdf);
	obj_evidence.properties.push_back(oriProp);

	// Set the discrete class label property
	wire_msgs::Property classProp;
	classProp.attribute = "class_label";
	pbl::PMF classPMF;

	// Probability of the class label is 0.7
	classPMF.setProbability(class_label, 0.7);
	pbl::PDFtoMsg(classPMF, classProp.pdf);
	obj_evidence.properties.push_back(classProp);

	// Set the discrete color property with a probability of 0.9
	wire_msgs::Property colorProp;
	colorProp.attribute = "color";
	pbl::PMF colorPMF;

	// The probability of the detected color is 0.9
	colorPMF.setProbability(color, 0.1);
	pbl::PDFtoMsg(colorPMF, colorProp.pdf);
	obj_evidence.properties.push_back(colorProp);

	// Add all properties to the array
	world_evidence.object_evidence.push_back(obj_evidence);
}


void generateEvidence() {

	// Create world evidence message
	wire_msgs::WorldEvidence world_evidence;

	// Set header
	world_evidence.header.stamp = ros::Time::now();
	world_evidence.header.frame_id = "/map";

	// Add evidence
	addEvidence(world_evidence, 2, 2.2, 3, "mug", "red");

	// Publish results
	world_evidence_publisher_.publish(world_evidence);
	ROS_INFO("Published world evidence with size %d", world_evidence.object_evidence.size());

}

/**
 * Main
 */
int main(int argc, char **argv) {

	// Initialize ros and create node handle
	ros::init(argc,argv,"generate_evidence");
	ros::NodeHandle nh;

	// Publisher
	world_evidence_publisher_ = nh.advertise<wire_msgs::WorldEvidence>("/world_evidence", 100);

	// Publish with 3 Hz
	ros::Rate r(3.0);

	while (ros::ok()) {
		generateEvidence();
		r.sleep();
	}

	return 0;
}
