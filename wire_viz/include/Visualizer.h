/*
 * Visualizer.h
 *
 *  Created on: May 8, 2012
 *      Author: sdries
 */

#ifndef VISUALIZER_H_
#define VISUALIZER_H_

#include "ros/ros.h"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <wire_msgs/Property.h>

#include "tf/transform_listener.h"

#include "problib/conversions.h"



#include <map>

// Color struct
struct Color {
		double r;
		double g;
		double b;

		Color() {};
		Color(double red, double green, double blue) : r(red), g(green), b(blue) {};
	};


class Visualizer {

public:

	Visualizer();

	virtual ~Visualizer();

	bool setParameters(ros::NodeHandle& n, const std::string& ns);

	bool createMarkers(const std_msgs::Header& header, long ID,
			const std::vector<wire_msgs::Property>& props,
			std::vector<visualization_msgs::Marker>& markers_out,
			const std::string& frame_id);

	void setTFListener(tf::TransformListener* tf_listener);

protected:

	// Structure storing marker information
	struct MarkerInfo {
		visualization_msgs::Marker shape_marker;
		visualization_msgs::Marker text_marker;
		visualization_msgs::Marker cov_marker;
		bool show_cov;
		bool show_text;
	};

	// Color mapping
	std::map<std::string, Color> color_mapping_;

	// Tf listener
	tf::TransformListener* tf_listener_;

	// Mapping from object class to marker
	std::map<std::string, MarkerInfo> object_class_to_marker_map_;

	// Mapping that defines per attribute if it has to be shown
	std::map<std::string, bool> attribute_map_;

	// Get marker parameters
	bool getMarkerParameters(ros::NodeHandle& n, const std::string& ns);

	// Get attribute settings
	bool getAttributeSettings(ros::NodeHandle& n, const std::string& ns);

	// Get most probable Gaussian from a pdf
	const pbl::Gaussian* getBestGaussian(const pbl::PDF& pdf, double min_weight = 0);

	// Set marker type
	void setMarkerType(XmlRpc::XmlRpcValue& v, MarkerInfo& m);

	// Overloaded get value function
	void getStructValue(XmlRpc::XmlRpcValue& s, const std::string& name, float& f, float default_value);
	void getStructValue(XmlRpc::XmlRpcValue& s, const std::string& name, double& d, double default_value);
	void getStructValue(XmlRpc::XmlRpcValue& s, const std::string& name, bool& b, bool default_value);
	void getStructValue(XmlRpc::XmlRpcValue& s, const std::string& name, std::string& str, const std::string& default_value);
};

#endif /* VISUALIZER_H_ */
