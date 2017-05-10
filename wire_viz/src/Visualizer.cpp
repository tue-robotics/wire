/*
 * Visualizer.cpp
 *
 *  Created on: May 8, 2012
 *      Author: sdries
 */

#include "Visualizer.h"
#include "default_values.h"

using namespace std;

Visualizer::Visualizer() : tf_listener_(0) {

	color_mapping_["red"] = Color(1,0,0);
	color_mapping_["green"] = Color(0,1,0);
	color_mapping_["blue"] = Color(0,0,1);
	color_mapping_["white"] = Color(1,1,1);
	color_mapping_["black"] = Color(0,0,0);
	color_mapping_["orange"] = Color(1,0.65,0);
	color_mapping_["pink"] = Color(1,0.08,0.58);
	color_mapping_["gold"] = Color(1,0.84,0);
	color_mapping_["grey"] = Color(0.41,0.41,0.41);
	color_mapping_["gray"] = Color(0.41,0.41,0.41);
	color_mapping_["cyan"] = Color(0,1,1);
	color_mapping_["yellow"] = Color(1,1,0);
	color_mapping_["magenta"] = Color(1,0,1);

}

Visualizer::~Visualizer() {

}

/*
 * Function that creates markers
 */
bool Visualizer::createMarkers(const std_msgs::Header& header, long ID,
		const vector<wire_msgs::Property>& props,
		vector<visualization_msgs::Marker>& markers_out,
		const std::string& frame_id = "" ) {

	// Pose according to evidence
	tf::Stamped<tf::Pose> ev_pose;
	ev_pose.setRotation(tf::Quaternion(0, 0, 0, 1));
	ev_pose.frame_id_ = header.frame_id;
	ev_pose.stamp_ = ros::Time();

	// Store class label and marker text
	string marker_text = "", class_label = "", color = "";

	// Variances
	double cov_xx = 0;
	double cov_yy = 0;
	double cov_zz = 0;

	// Iterate over all the propoerties
	bool pos_found = false;
	for (vector<wire_msgs::Property>::const_iterator it_prop = props.begin(); it_prop != props.end(); ++it_prop) {

		pbl::PDF* pdf = pbl::msgToPDF(it_prop->pdf);

		// Position
		if (it_prop->attribute == "position") {
			const pbl::Gaussian* gauss = getBestGaussian(*pdf);
			if (gauss) {
				const pbl::Vector& mean = gauss->getMean();

				cov_xx = gauss->getCovariance()(0, 0);
				cov_yy = gauss->getCovariance()(1, 1);
				cov_zz = gauss->getCovariance()(2, 2);

				if (gauss->dimensions() == 2) {
					ev_pose.setOrigin(tf::Point(mean(0), mean(1), 0));
					pos_found = true;
				} else if (gauss->dimensions() == 3) {
					pos_found = true;
					ev_pose.setOrigin(tf::Point(mean(0), mean(1), mean(2)));
				} else {
					ROS_WARN("World evidence: position attribute has %d dimensions, Visualizer cannot deal with this.",
							gauss->dimensions());
				}
			}
		}
		// Orientation
		else if (it_prop->attribute == "orientation") {
			if (pdf->dimensions() == 4) {
				const pbl::Gaussian* gauss = getBestGaussian(*pdf);
				if (gauss) {
					const pbl::Vector& mean = gauss->getMean();
					ev_pose.setRotation(tf::Quaternion(mean(0), mean(1), mean(2), mean(3)));
				}
			} else {
				ROS_WARN("Orientation attribute has %d dimensions, must be 4 (X Y Z W).", pdf->dimensions());
			}
		}
		// Class label
		else if (it_prop->attribute == "class_label") {
			pdf->getExpectedValue(class_label);
		}
		// Color
		else if (it_prop->attribute == "color") {
			pdf->getExpectedValue(color);
		}

		// Check if the current property represents an attribute that has to be added to the text label
		for (map<string,bool>::const_iterator it = attribute_map_.begin(); it != attribute_map_.end(); ++it) {
			if (it_prop->attribute == it->first && it->second) {
				string txt = "";
				// Only consider pmfs otherwise the getExpectedValue function fails
				if (pdf->type() == pbl::PDF::DISCRETE) {
					pdf->getExpectedValue(txt);
					// If marker text not empty add ',' for ease of reading
					if (marker_text != "") {
						txt = ", " + txt;
					}
					marker_text += txt;
				}
			}
		}


		delete pdf;

	} // End iterate over properties

	// A position is always needed
	if (!pos_found) {
		return false;
	}

	// Default marker must be defined
	ROS_ASSERT(object_class_to_marker_map_.find("default") != object_class_to_marker_map_.end());

	// Create markers belonging to the objects class
	string map_key = "default";
	visualization_msgs::Marker marker;
	visualization_msgs::Marker text_marker;
	if (object_class_to_marker_map_.find(class_label) != object_class_to_marker_map_.end()) {
		map_key = class_label;
	}
	marker = object_class_to_marker_map_[map_key].shape_marker;
	text_marker = object_class_to_marker_map_[map_key].text_marker;

	// If the object has a color, use color of the object as marker color
	if (color != "" && color_mapping_.find(color) != color_mapping_.end()) {
		marker.color.r = color_mapping_[color].r;
		marker.color.g = color_mapping_[color].g;
		marker.color.b = color_mapping_[color].b;
	}


	// Transform to frame
	tf::Stamped<tf::Pose> ev_pose_TRANSFORMED;
	if (frame_id != "" && tf_listener_ != 0) {
		try{
			tf_listener_->transformPose(frame_id, ev_pose, ev_pose_TRANSFORMED);
			marker.header.frame_id = frame_id;
			text_marker.header.frame_id = frame_id;
		} catch (tf::TransformException ex){
			ROS_ERROR("[VISUALIZER] %s",ex.what());
			ev_pose_TRANSFORMED = ev_pose;
			marker.header.frame_id = header.frame_id;
			text_marker.header.frame_id = header.frame_id;
		}
	} else {
		ev_pose_TRANSFORMED = ev_pose;
		marker.header.frame_id = header.frame_id;
		text_marker.header.frame_id = header.frame_id;
	}

	// Update shape marker position to transformed position and add marker to array
	marker.pose.position.x = ev_pose_TRANSFORMED.getOrigin().getX();
	marker.pose.position.y = ev_pose_TRANSFORMED.getOrigin().getY();
	marker.pose.position.z = ev_pose_TRANSFORMED.getOrigin().getZ();
	marker.header.stamp = header.stamp;
	marker.id = 3 * ID;
	markers_out.push_back(marker);

	// Only if text must be shown
	if (object_class_to_marker_map_[map_key].show_text) {
		// Update text marker and add to array
		text_marker.pose.position.x += ev_pose_TRANSFORMED.getOrigin().getX();
		text_marker.pose.position.y += ev_pose_TRANSFORMED.getOrigin().getY();
		text_marker.pose.position.z += ev_pose_TRANSFORMED.getOrigin().getZ();
		text_marker.header.stamp = header.stamp;
		text_marker.id = 3 * ID + 1;

		// Add ID separately since this not necessarily is an attribute
		if (attribute_map_.find("ID") != attribute_map_.end() && attribute_map_["ID"]) {
			stringstream ss;
			ss << ID;
			string id_text = " (" + ss.str() + ")";
			marker_text += id_text;
		}

		// Set the text label and add the marker
		text_marker.text = marker_text;
		markers_out.push_back(text_marker);
	}

	// Covariance marker
	if (object_class_to_marker_map_[map_key].show_cov) {
		visualization_msgs::Marker cov_marker = marker;
		cov_marker.color.a *= 0.5;
		cov_marker.scale.x *= (1 + sqrt(cov_xx) * 3);
		cov_marker.scale.y *= (1 + sqrt(cov_yy) * 3);
		cov_marker.scale.z *= (1 + sqrt(cov_zz) * 3);
		cov_marker.id = 3 * ID + 2;
		markers_out.push_back(cov_marker);
	}

	return true;
}


/*
 * Set the tf listener
 */
void Visualizer::setTFListener(tf::TransformListener* tf_listener) {
	tf_listener_ = tf_listener;
}

/*
 * Get the most probable Gaussian from a pdf
 */
const pbl::Gaussian* Visualizer::getBestGaussian(const pbl::PDF& pdf, double min_weight) {
	if (pdf.type() == pbl::PDF::GAUSSIAN) {
		return pbl::PDFtoGaussian(pdf);
	} else if (pdf.type() == pbl::PDF::MIXTURE) {
		const pbl::Mixture* mix = pbl::PDFtoMixture(pdf);

		if (mix){
			const pbl::Gaussian* G_best = 0;
			double w_best = min_weight;
			for(int i = 0; i < mix->components(); ++i) {
				const pbl::PDF& pdf = mix->getComponent(i);
				const pbl::Gaussian* G = pbl::PDFtoGaussian(pdf);
				double w = mix->getWeight(i);
				if (G && w > w_best) {
					G_best = G;
					w_best = w;
				}
			}
			return G_best;
		}
    }

	return 0;
}


/*
 * Get double
 */
void Visualizer::getStructValue(XmlRpc::XmlRpcValue& s, const std::string& name, double& d, double default_value) {
	XmlRpc::XmlRpcValue& v = s[name];

	if (v.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
		d = (double)v;
		return;
	}

	if (v.getType() == XmlRpc::XmlRpcValue::TypeInt) {
		d = (int)v;
		return;
	}

	d = default_value;
}

/*
 * Get float
 */
void Visualizer::getStructValue(XmlRpc::XmlRpcValue& s, const std::string& name, float& f, float default_value) {
	double d;
	getStructValue(s, name, d, default_value);
	f = (float)d;
}

/*
 * Get bool
 */
void Visualizer::getStructValue(XmlRpc::XmlRpcValue& s, const std::string& name, bool& b, bool default_value) {
	XmlRpc::XmlRpcValue& v = s[name];

	if (v.getType() == XmlRpc::XmlRpcValue::TypeBoolean) {
		b = (bool)v;
		return;
	}

	b = default_value;
}

/*
 * Get string
 */
void Visualizer::getStructValue(XmlRpc::XmlRpcValue& s, const std::string& name, std::string& str, const string& default_value) {
	XmlRpc::XmlRpcValue& v = s[name];

	if (v.getType() == XmlRpc::XmlRpcValue::TypeString) {
		str = (string)v;
		return;
	}

	str = default_value;
}


bool Visualizer::setParameters(ros::NodeHandle& n, const string& ns) {

	bool param = false, att = false;
	param = getMarkerParameters(n, ns);
	att = getAttributeSettings(n, ns);

	return (param && att);
}



/*
 * Get marker parameters from parameter servers
 */
bool Visualizer::getMarkerParameters(ros::NodeHandle& n, const string& ns) {

	// Get marker parameters from the parameter server
	XmlRpc::XmlRpcValue marker_params;
	if (!n.getParam(ns, marker_params)) {
		ROS_ERROR("No global marker parameters given. (namespace: %s)", n.getNamespace().c_str());
		return false;
	}

	// Check if the marker parameters are of type array
	if (marker_params.getType() != XmlRpc::XmlRpcValue::TypeArray) {
		ROS_ERROR("Malformed marker parameter specification.  (namespace: %s)", n.getNamespace().c_str());
		return false;
	}

	// Iterate over the marker parameters in array
	for(int i = 0; i < marker_params.size(); ++i) {

		// Get current value
		XmlRpc::XmlRpcValue& v = marker_params[i];

		// Check type
		if (v.getType() == XmlRpc::XmlRpcValue::TypeStruct) {

			// Get class
			string class_name = "";
			getStructValue(v, "class", class_name, "");

			// If no class defined, no marker can be added
			if (class_name == "") {
				ROS_WARN("No class defined, skipping parameters");
			}
			// The first class must be default, since the default marker defines values that are not defined
			else if (i==0 && class_name != "default") {
				ROS_ERROR("Make sure to first specify default in YAML file");
				return false;
			}
			// Add the default marker
			else if (i==0 && class_name == "default") {

				// Get al properties
				MarkerInfo& m = object_class_to_marker_map_[class_name];
				m.shape_marker.action = visualization_msgs::Marker::ADD;
				m.shape_marker.ns = ns;
				getStructValue(v, "show_text", m.show_text, visualize::DEFAULT_SHOW_TEXT);
				getStructValue(v, "show_cov", m.show_cov, visualize::DEFAULT_SHOW_COV);
				getStructValue(v, "r", m.shape_marker.color.r, visualize::DEFAULT_COLOR_R);
				getStructValue(v, "g", m.shape_marker.color.g, visualize::DEFAULT_COLOR_G);
				getStructValue(v, "b", m.shape_marker.color.b, visualize::DEFAULT_COLOR_B);
				getStructValue(v, "alpha", m.shape_marker.color.a, visualize::DEFAULT_COLOR_ALPHA);
				getStructValue(v, "scale_x", m.shape_marker.scale.x, visualize::DEFAULT_SCALE_X);
				getStructValue(v, "scale_y", m.shape_marker.scale.y, visualize::DEFAULT_SCALE_Y);
				getStructValue(v, "scale_z", m.shape_marker.scale.z, visualize::DEFAULT_SCALE_Z);
				getStructValue(v, "offset_x_pos", m.shape_marker.pose.position.x, visualize::DEFAULT_OFFSET_X_POS);
				getStructValue(v, "offset_y_pos", m.shape_marker.pose.position.y, visualize::DEFAULT_OFFSET_Y_POS);
				getStructValue(v, "offset_z_pos", m.shape_marker.pose.position.z, visualize::DEFAULT_OFFSET_Z_POS);
				getStructValue(v, "offset_x_rot", m.shape_marker.pose.orientation.x, visualize::DEFAULT_OFFSET_X_ROT);
				getStructValue(v, "offset_y_rot", m.shape_marker.pose.orientation.y, visualize::DEFAULT_OFFSET_Y_ROT);
				getStructValue(v, "offset_z_rot", m.shape_marker.pose.orientation.z, visualize::DEFAULT_OFFSET_Z_ROT);
				getStructValue(v, "offset_w_rot", m.shape_marker.pose.orientation.w, visualize::DEFAULT_OFFSET_W_ROT);
				getStructValue(v, "mesh_resource", m.shape_marker.mesh_resource, "");
				double duration = 0;
				getStructValue(v, "duration", duration, visualize::DEFAULT_MARKER_LIFETIME);
				m.shape_marker.lifetime = ros::Duration(duration);
				m.text_marker.lifetime = ros::Duration(duration);

				// Set geomatric shape
				setMarkerType(v, m);

				ROS_DEBUG("Added marker with class default to %s", ns.c_str());
			}
			// Add marker of any class
			else {

				// Add class to the map if not defined use default values
				MarkerInfo& m = object_class_to_marker_map_[class_name];
				m.shape_marker.action = visualization_msgs::Marker::ADD;
				m.shape_marker.ns = ns;
				getStructValue(v, "show_text", m.show_text, object_class_to_marker_map_["default"].show_text);
				getStructValue(v, "show_cov", m.show_cov, object_class_to_marker_map_["default"].show_cov);
				getStructValue(v, "r", m.shape_marker.color.r, object_class_to_marker_map_["default"].shape_marker.color.r);
				getStructValue(v, "g", m.shape_marker.color.g, object_class_to_marker_map_["default"].shape_marker.color.g);
				getStructValue(v, "b", m.shape_marker.color.b, object_class_to_marker_map_["default"].shape_marker.color.b);
				getStructValue(v, "alpha", m.shape_marker.color.a, object_class_to_marker_map_["default"].shape_marker.color.a);
				getStructValue(v, "scale_x", m.shape_marker.scale.x, object_class_to_marker_map_["default"].shape_marker.scale.x);
				getStructValue(v, "scale_y", m.shape_marker.scale.y, object_class_to_marker_map_["default"].shape_marker.scale.y);
				getStructValue(v, "scale_z", m.shape_marker.scale.z, object_class_to_marker_map_["default"].shape_marker.scale.z);
				getStructValue(v, "offset_x_pos", m.shape_marker.pose.position.x, object_class_to_marker_map_["default"].shape_marker.pose.position.x);
				getStructValue(v, "offset_y_pos", m.shape_marker.pose.position.y, object_class_to_marker_map_["default"].shape_marker.pose.position.y);
				getStructValue(v, "offset_z_pos", m.shape_marker.pose.position.z, object_class_to_marker_map_["default"].shape_marker.pose.position.z);
				getStructValue(v, "offset_x_rot", m.shape_marker.pose.orientation.x, object_class_to_marker_map_["default"].shape_marker.pose.orientation.x);
				getStructValue(v, "offset_y_rot", m.shape_marker.pose.orientation.y, object_class_to_marker_map_["default"].shape_marker.pose.orientation.y);
				getStructValue(v, "offset_z_rot", m.shape_marker.pose.orientation.z, object_class_to_marker_map_["default"].shape_marker.pose.orientation.z);
				getStructValue(v, "offset_w_rot", m.shape_marker.pose.orientation.w, object_class_to_marker_map_["default"].shape_marker.pose.orientation.w);
				double duration = object_class_to_marker_map_["default"].shape_marker.lifetime.toSec();
				getStructValue(v, "duration", duration, duration);
				m.shape_marker.lifetime = ros::Duration(duration);
				m.text_marker.lifetime = ros::Duration(duration);

				// Set appropriate marker geometry
				setMarkerType(v, m);

				ROS_DEBUG("Added marker with class %s to %s", class_name.c_str(), ns.c_str());

			}
		} else {
			ROS_WARN("Parameter specification not a list (parameter ignored)!");
		}
	}

	// Check if text marker is defined
	if (object_class_to_marker_map_.find("text") ==  object_class_to_marker_map_.end()) {
		ROS_ERROR("Marker with class text must be defined!");
		return false;
	}

	// Set text markers for all classes with text marker properties
	visualization_msgs::Marker default_text_marker = object_class_to_marker_map_["text"].shape_marker;
	for(map<string, MarkerInfo>::iterator it = object_class_to_marker_map_.begin(); it != object_class_to_marker_map_.end(); ++it) {
		it->second.text_marker = default_text_marker;
		it->second.text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
	}

	return true;
}


/*
 * Set appropriate marker type
 */
void Visualizer::setMarkerType(XmlRpc::XmlRpcValue& v, MarkerInfo& m) {

	// Check if a mesh is defined
	if (m.shape_marker.mesh_resource != "") {
		m.shape_marker.type = visualization_msgs::Marker::MESH_RESOURCE;
		ROS_INFO("mesh_resource = %s", m.shape_marker.mesh_resource.c_str());
	}
	// If no mesh defined, get geometry type or use default
	else {

		// Get marker type from parameter server
		string marker_type;
		getStructValue(v, "type", marker_type, visualize::DEFAULT_TYPE);

		// Convert type to lower case only
		std::transform(marker_type.begin(), marker_type.end(), marker_type.begin(), ::tolower);

		// Set appropriate type
		if (marker_type == "cube") {
			m.shape_marker.type = visualization_msgs::Marker::CUBE;
		} else if (marker_type == "cylinder") {
			m.shape_marker.type = visualization_msgs::Marker::CYLINDER;
		} else if (marker_type == "arrow") {
			m.shape_marker.type = visualization_msgs::Marker::ARROW;
		} else if (marker_type == "sphere") {
			m.shape_marker.type = visualization_msgs::Marker::SPHERE;
		} else {
			ROS_WARN("Defined marker type %s, which is unknown. Using sphere instead.", marker_type.c_str());
			m.shape_marker.type = visualization_msgs::Marker::SPHERE;
		}
	}

}




/*
 * Get attribute settings (which attributes should be present in the text labels)
 */
bool Visualizer::getAttributeSettings(ros::NodeHandle& n, const string& ns) {

	string param_name = ns + "_attributes";

	// Get marker parameters from the parameter server
	XmlRpc::XmlRpcValue attribute_params;
	if (!n.getParam(param_name, attribute_params)) {
		ROS_ERROR("No global \"%s\" parameters given. (namespace: %s)", param_name.c_str(), n.getNamespace().c_str());
		return false;
	}

	// Check if the marker parameters are of type array
	if (attribute_params.getType() != XmlRpc::XmlRpcValue::TypeArray) {
		ROS_ERROR("Malformed \"attributes\" parameter specification.  (namespace: %s)", n.getNamespace().c_str());
		return false;
	}

	// Iterate over the marker parameters in array
	for(int i = 0; i < attribute_params.size(); ++i) {

		// Get the list
		XmlRpc::XmlRpcValue& v = attribute_params[i];

		// Iterate over the list
		ROS_DEBUG("Attributes for %s:", ns.c_str());
		for (XmlRpc::XmlRpcValue::iterator it = v.begin(); it != v.end(); ++it) {
			string att_name = static_cast<string>(it->first);
			bool att_bool = static_cast<int>(it->second);
			attribute_map_[att_name] = att_bool;
			ROS_DEBUG(" %s will %s shown", att_name.c_str(), att_bool?"be":"not be");
		}

	}

	return true;
}
