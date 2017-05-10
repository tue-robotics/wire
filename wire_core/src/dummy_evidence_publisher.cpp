/*
 * dummy_evidence_publisher.cpp
 *
 *  Created on: Mar 2, 2012
 *      Author: sdries
 */

/*
 * msg_converter.cpp
 *
 *  Created on: Dec 14, 2011
 *      Author: Jos Elfring, Sjoerd van den Dries
 */

#include <ros/ros.h>

#include "wire_msgs/WorldEvidence.h"
#include "wire_msgs/ObjectEvidence.h"

#include "problib/pdfs/PDF.h"

#include "problib/conversions.h"

#include <map>

using namespace std;

ros::Publisher EVIDENCE_PUB;                             // publisher

ros::Time time_last_cycle_;

double x_person = 0;
double x_vel_person = 0.5;

void addEvidence(wire_msgs::WorldEvidence& world_evidence, double x, double y, double z, const string& class_label) {
	wire_msgs::ObjectEvidence obj_evidence;

    obj_evidence.certainty = 1.0;

	// setting position property
    wire_msgs::Property posProp;
	posProp.attribute = "position";
	pbl::PDFtoMsg(pbl::Gaussian(pbl::Vector3(x, y, z), pbl::Matrix3(0.001, 0.001, 0.001)), posProp.pdf);
	obj_evidence.properties.push_back(posProp);

	// setting class label property
    wire_msgs::Property classProp;
	classProp.attribute = "class_label";
	pbl::PMF classPMF;
	classPMF.setProbability(class_label, 0.2);
	pbl::PDFtoMsg(classPMF, classProp.pdf);
	obj_evidence.properties.push_back(classProp);

	// Add to array
	world_evidence.object_evidence.push_back(obj_evidence);
}


void generateEvidence() {
	ros::Time time_now = ros::Time::now();
	double dt = (time_now - time_last_cycle_).toSec();

	x_person += dt * x_vel_person;

	wire_msgs::WorldEvidence world_evidence;
	world_evidence.header.stamp = ros::Time::now();
	world_evidence.header.frame_id = "/map";

	addEvidence(world_evidence, 5, 4, 3, "body");
	addEvidence(world_evidence, 5, 4, 4, "face");
	addEvidence(world_evidence, 1, 2, 3, "cup");
	addEvidence(world_evidence, 3, 2.5, 1, "cup");
	addEvidence(world_evidence, 2, 2, 3, "test_class");
	//addEvidence(world_evidence, x_person, 2, 1+x_person, "cup", "");

	// Publish results
	EVIDENCE_PUB.publish(world_evidence);

	time_last_cycle_ = time_now;

    ROS_INFO("Published evidence with size %zu", world_evidence.object_evidence.size());
}

/**
 * Main
 */
int main(int argc, char **argv) {

	// Initialize ros and create node handle
	ros::init(argc,argv,"dummy_evidence_publisher");
	ros::NodeHandle nh;

	// Subscriber/publisher
	EVIDENCE_PUB = nh.advertise<wire_msgs::WorldEvidence>("/world_evidence", 100);

	ros::Rate r(20);

	time_last_cycle_ = ros::Time::now();
	while (ros::ok()) {
		generateEvidence();
		r.sleep();
	}

	return 0;
}
