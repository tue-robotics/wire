/*
 * get_world_state.cpp
 *
 *  Created on: Nov 13, 2012
 *      Author: jelfring
 */

#include "ros/ros.h"
#include "wire_msgs/ObjectState.h"
#include "wire_msgs/WorldState.h"

#include "problib/conversions.h"

#include <vector>

using namespace std;


// Function that extract the Gaussian with the highest weight from a mixture of Gaussians and/or a uniform distribution
const pbl::Gaussian* getBestGaussianFromMixture(const pbl::Mixture& mix, double min_weight = 0) {
	const pbl::Gaussian* G_best = 0;
	double w_best = min_weight;

	// Iterate over all components
	for(int i = 0; i < mix.components(); ++i) {
		const pbl::PDF& pdf = mix.getComponent(i);
		const pbl::Gaussian* G = pbl::PDFtoGaussian(pdf);
		double w = mix.getWeight(i);

		// Get Gaussian with the highest associated weight
		if (G && w > w_best) {
			G_best = G;
			w_best = w;
		}
	}
	return G_best;
}



void worldStateCallback(const wire_msgs::WorldState::ConstPtr& msg) {

	ROS_INFO("Received world state with %d objects", msg->objects.size());


	// Iterate over world model objects
	for (vector<wire_msgs::ObjectState>::const_iterator it_obj = msg->objects.begin(); it_obj != msg->objects.end(); ++it_obj) {

		// Get object from the object array
		wire_msgs::ObjectState obj = *it_obj;
		ROS_INFO(" Object:");

		// Iterate over all properties
		for (vector<wire_msgs::Property>::const_iterator it_prop = obj.properties.begin(); it_prop != obj.properties.end(); ++it_prop) {

			// Get the pdf of the current property
			pbl::PDF* pdf = pbl::msgToPDF(it_prop->pdf);

			if (pdf) {

				//// Check the attribute

				// Object class
				if (it_prop->attribute == "class_label") {
					string label = "";
					pdf->getExpectedValue(label);

					// Get probability
					const pbl::PMF* pmf = pbl::PDFtoPMF(*pdf);
					double p = pmf->getProbability(label);
					ROS_INFO(" - class %s with probability %f", label.c_str(), p);
				}
				// Position
				else if (it_prop->attribute == "position") {
					const pbl::Mixture* pos_pdf = pbl::PDFtoMixture(*pdf);
					if (pos_pdf) {
						const pbl::Gaussian* pos_gauss = getBestGaussianFromMixture(*pos_pdf);
						if (pos_gauss) {
							const pbl::Vector& pos = pos_gauss->getMean();
							ROS_INFO(" - position: (%f,%f,%f)", pos(0), pos(1), pos(2));
							ROS_INFO(" - diagonal position cov: (%f,%f,%f)",
									pos_gauss->getCovariance()(0, 0), pos_gauss->getCovariance()(1, 1), pos_gauss->getCovariance()(2, 2));

						}
					} else {
						ROS_INFO(" - position: object position unknown (uniform distribution)");
					}
				}
				// Orientation
				else if (it_prop->attribute == "orientation") {
					const pbl::Mixture* orientation_pdf = pbl::PDFtoMixture(*pdf);
					if (orientation_pdf) {
						const pbl::Gaussian* orientation_gauss = getBestGaussianFromMixture(*orientation_pdf);
						if (orientation_gauss) {
							const pbl::Vector& ori = orientation_gauss->getMean();
							ROS_INFO(" - orientation: (%f,%f,%f,%f)", ori(0), ori(1), ori(2), ori(3));
						}
					} else {
						ROS_INFO(" - orientation: object orientation unknown (uniform distribution)");
					}
				}
				// Color
				else if (it_prop->attribute == "color") {
					string color = "";
					pdf->getExpectedValue(color);
					ROS_INFO(" - color: %s", color.c_str());
				}

				delete pdf;
			}

		}

	}

}

int main(int argc, char **argv) {
	ros::init(argc, argv, "get_world_state");
	ros::NodeHandle nh;

	// Subscribe to the world model
	ros::Subscriber sub_obj = nh.subscribe("/world_state", 1000, &worldStateCallback);

	ros::spin();
	return 0;
}
